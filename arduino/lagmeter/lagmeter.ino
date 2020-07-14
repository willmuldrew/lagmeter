// libs required:
// u8g2 (display)
// AceButton (pushbuttons)


#include <Arduino.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <AceButton.h>

using namespace ace_button;

#define SYNC_TIMEOUT_MILLIS 1000

#define SAMPLE_BUFFER_LENGTH 200  // not much memory on arduino!
#define MULTI_RUN_LENGTH 10
#define LED_PIN 13 // going to light this while the capture's active - helpful for debugging and calibration

// OLED display
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Target interval for light capture measurements
const int sampleInterval = 2000; // micros - approx

// How long to busy sample the sensor (then applies mean)
const int sampleDuration = 1000;

typedef struct datapoint {
  uint16_t timestamp;
  uint8_t value;
} datapoint_t;

// Global state for the light-level capture buffer
long t0 = 0;
int writePosition = SAMPLE_BUFFER_LENGTH ;
datapoint_t captureBuffer[SAMPLE_BUFFER_LENGTH];

// OLED display status messages
char msgBufferA[17];
char msgBufferB[17];

AceButton startCaptureButton(7);
AceButton modeButton(6);
AceButton* buttons[] = { 
  &startCaptureButton, 
  &modeButton
};

int runsToDo = 0;
int currentRun = 0;

float multiRunLagSum = 0;
float multiRunLagSumSq = 0;
int multiRunLagCount = 0;


enum class Mode { 
  KeyboardX,
  AltTab,
  LeftClick,
  AltEscape, // AltEscape is next window switching - faster than alt tab (use shift to go back!)
  Count
};

Mode currentMode = Mode::KeyboardX;

void nextMode() {
  currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::Count)); 
  writeScreenMessage("Mode:", getModeDescription(currentMode));
}

const char* getModeDescription(Mode m) {
  switch(m) {
    case Mode::KeyboardX:
      return "Key-X";
    case Mode::AltTab:
      return "Alt-Tab";
    case Mode::LeftClick:
      return "Left Click";    
    case Mode::AltEscape:
      return "Alt-Escape";
  }
}

void handleButtonEvent(AceButton* pButton, uint8_t eventType, uint8_t /*buttonState*/) {
  if(pButton == &startCaptureButton) {
    switch (eventType) {
      case AceButton::kEventClicked:
        startTest(1);
        break;
      case AceButton::kEventLongPressed:
        startTest(MULTI_RUN_LENGTH);
        break;    
    }
  } else if(pButton == &modeButton) {
    switch (eventType) {
       case AceButton::kEventClicked:     
        nextMode();
        break;    
    }
  }
}


void setup() {
  for(auto pButton : buttons) {
    pinMode(pButton->getPin(), INPUT_PULLUP);  
    pButton->setEventHandler(handleButtonEvent);
    ButtonConfig* buttonConfig = pButton->getButtonConfig();
    buttonConfig->setEventHandler(handleButtonEvent);
    buttonConfig->setFeature(ButtonConfig::kFeatureClick);
    buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
    buttonConfig->setFeature(ButtonConfig::kFeatureSuppressAfterClick);
  }
  
  u8g2.begin();
    
  Serial.begin(115200);  
  Keyboard.begin();
  Mouse.begin();

  clearCaptureBuffer();
  writeScreenMessage("Ready...", "");
  drawScreen();

  pinMode(LED_PIN, OUTPUT);    
  digitalWrite(LED_PIN, LOW);
}

void writeScreenMessage(const char* line1, const char* line2) {
  snprintf(msgBufferA, sizeof(msgBufferA), line1);
  snprintf(msgBufferB, sizeof(msgBufferB), line2);
  drawScreen();   
}

void clearCaptureBuffer() {
   memset(captureBuffer, 0, sizeof(captureBuffer));
}

void drawScreen() {
  u8g2.firstPage();  
  do {
    // Write some messsages
    u8g2.setFont(u8g2_font_7x13B_tf);
    u8g2.drawStr(0,13, msgBufferA);
    u8g2.drawStr(0,26, msgBufferB);

    // Draw a tiny chart
    int nWidth = 72;
    int nHeight = 32;
    int range = 1;

    int minValue = 1024;
    int maxValue = -1;

    for(int n=0;n<SAMPLE_BUFFER_LENGTH;++n) {
      int val = captureBuffer[n].value;
      minValue = min(minValue, val);
      maxValue = max(maxValue, val);
    }    
    
    range = max(maxValue - minValue, 32);        
    for(int x=0;x<nWidth;++x) {
        int nSample = (SAMPLE_BUFFER_LENGTH * x) / nWidth;
        int value = captureBuffer[nSample].value;
        int y = (value - minValue) * 32 / range;
        u8g2.drawPixel(x, 63 - y);
    }
  } while(u8g2.nextPage() );
}


// Read a line from serial up to \n with a timeout
unsigned int serialReadLn(char* buffer, size_t buffLength, int timeoutMillis) {
  unsigned long tStart = millis();  
  unsigned int pos = 0;
  memset(buffer, 0, buffLength);

  while(true) {
    if((millis() - tStart) > timeoutMillis) {
      memset(buffer, 0, buffLength);
      pos = 0;
      break;
    }

    if(Serial.available() > 0) {
      char c = Serial.read();
      if(pos < buffLength) {
        buffer[pos++] = c;
      }
      if(c == '\n') {
        break;
      }    
    }
  }
  return pos;
}

void startCapture() {
  clearCaptureBuffer();
  writePosition = 0; 
  t0 = micros();    
  digitalWrite(LED_PIN, HIGH);
}

// Attempt to sync clock with host on serial bus. Requires relevant lagcap python process to be running
// on the other end...  will wait SYNC_TIMEOUT_MILLIS before giving up to prevent the lack of correct
// host process hanging the arduino
//
// 3 way handshake:
//  * the arduino snaps local micros()
//  * ardunio says SYNC to host
//  * host snaps system time and responds OK
//  * ardunio snaps local micros() again
//  * arduino reports both micros timestamps to host
//
// Host now knows that it's system timestamp lies between the two micros() timestamps.  It can then translate
// ardunio micros() timestamps -> system time with a sub milli accuracy (round-trip is ~0.5ms IIRC)
// 
// Issues:
//  micros() wraps once an hour so could be unlucky... will deal with this at the host end by adding 2**32
//  RTT is variable - could get remote host to respond with AGAIN to retry until a decent RTT is achieved
boolean trySerialClockSync() {
  char strBuffer[64];
  unsigned long localMicrosA = 0;
  unsigned long localMicrosB = 0;

  localMicrosA = micros();
  Serial.print("SYNC\r\n");
  Serial.flush(); 
  
  if(serialReadLn(strBuffer, sizeof(strBuffer), SYNC_TIMEOUT_MILLIS) > 0){      
    if(strcmp(strBuffer, "OK\r\n") == 0) {        
      localMicrosB = micros();
      sprintf(strBuffer, "SYNCED %ld %ld\r\n", localMicrosA, localMicrosB);
      Serial.print(strBuffer);
      Serial.flush();         
      return true;      
    }
  }   
  return false;  
}

boolean isCaptureRunning() {
  return writePosition < SAMPLE_BUFFER_LENGTH;
}

// Take as many readings as possible in the time we're given
int readSmoothedAnalog(int durationMicros){
  unsigned long start = micros();
  unsigned long now = start;
  float sum = 0.0;
  int count = 0;

  // micros() wraps!
  while(now - start < durationMicros && now >= start) {
    sum += analogRead(A0);
    count++;
    now = micros();
  }
  if(count == 0) { 
    // uhoh - either durationMicros was too short, we had a weird hang, or micros wrapped - abort!
    return -1;
  }
  return (int)(sum / count);
}

void startTest(int runs){
  writeScreenMessage("Test", "starting...");  
  if(trySerialClockSync()) {
    delay(1000); // give the remote host a chance to kick off pcaps etc...
  }       

  multiRunLagCount = 0;
  multiRunLagSum = 0;
  multiRunLagSumSq = 0;
  
  runsToDo = runs;
  currentRun = 0;
}

void loop() {  
  if(isCaptureRunning()) {
    // If the capture is running we don't want to do anything else
    unsigned long t = micros();
    int reading = readSmoothedAnalog(sampleDuration);
    if(reading >= 0) {    
      captureBuffer[writePosition].value = reading / 4;
      captureBuffer[writePosition].timestamp = (t - t0) / 1000;
      writePosition++;
    } else {
      // skip - something went a bit wrong!
    }
    
    if(writePosition == SAMPLE_BUFFER_LENGTH) {
      digitalWrite(LED_PIN, LOW); 
      // We've done our last measurement - do some processing for local display and dump results to the 
      // serial port      
      dumpResultsToSerial();
      processResults();
      
      currentRun++;
      if(runsToDo > 1 && currentRun == runsToDo) {
        // We've finished a multirun
        processMultiResults();
      }

      // If we generated a keypress, let's delete it for convenience...  Sometimes it gets annoying though
      // if your backspaces hit a browser!
      doHIDUndo();
      // Update the screen (including drawing a chart)
      drawScreen();      
    } else {
      // busy loop to delay until next sample is scheduled...
      unsigned long now = micros();
      while(now >= t && now < t + sampleInterval) {
        now = micros(); 
      }
    }
  } else if(runsToDo - currentRun > 0){  
    char buff[128];
    sprintf(buff, "run %d/%d", currentRun, runsToDo);
    writeScreenMessage(buff, "");
    if(currentRun > 0) {
      delay(random(500, 2000));
    }
    startCapture();
    doHIDOutput();    
  } else {
    for(auto pButton : buttons) {
      pButton->check();
    }
  }
}

void doHIDOutput() {
  switch(currentMode) {
    case Mode::KeyboardX:
      Keyboard.print("x");
      break;
    case Mode::AltTab:
      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press(KEY_TAB);
      delay(1);
      Keyboard.releaseAll();        
      break;
    case Mode::LeftClick:
      Mouse.click();
      break;
    case Mode::AltEscape:
      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press(KEY_ESC);
      delay(1);
      Keyboard.releaseAll();     
  }        
}

void doHIDUndo() {
  delay(500);
  switch(currentMode) {
    case Mode::KeyboardX:
      Keyboard.write(KEY_BACKSPACE);
      break;
    case Mode::AltTab:
      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press(KEY_TAB);
      delay(1);
      Keyboard.releaseAll(); 
      break;
    case Mode::AltEscape:
      Keyboard.press(KEY_RIGHT_SHIFT);
      Keyboard.press(KEY_LEFT_ALT);
      Keyboard.press(KEY_ESC);
      delay(1);
      Keyboard.releaseAll(); 
  }        
}

void dumpResultsToSerial() {
  // Dump CSV results to serial bus - easy to load into a pandas dataframe
  Serial.println("resultsStart");
  Serial.println("t0,t,value");
  for(int i=0;i<SAMPLE_BUFFER_LENGTH;++i) {
    Serial.print(t0 / 1000);
    Serial.print(",");
    Serial.print(captureBuffer[i].timestamp);
    Serial.print(",");
    Serial.println(captureBuffer[i].value);      
  }
  Serial.println("resultsEnd");
  Serial.flush();
}

int signum(int x) {
  return (x > 0) - (x < 0);
}

void processResults() {
  // use first N samples to establish start, last N to establish end, work out when we crossed the half way mark
  const int samplesToAverage = 10;  
  float startSum = 0, endSum = 0;
  
  for(int n=0;n<samplesToAverage;n++) {
    startSum += captureBuffer[n].value;
    endSum += captureBuffer[SAMPLE_BUFFER_LENGTH - n - 1].value;
  }

  float startMean = startSum / samplesToAverage;
  float endMean = endSum / samplesToAverage;
  int midPoint = (int)(0.5 * startMean + 0.5 * endMean + 0.5);
  int startPoint = (int)(0.9 * startMean + 0.1 * endMean + 0.5);
  int endPoint = (int)(0.1 * startMean + 0.9 * endMean + 0.5);

  int cmpsgn = startMean < endMean ? 1 : -1;
  int midN = -1, startN = -1, endN = -1;
  for(int n=0;n<SAMPLE_BUFFER_LENGTH;++n) {
    int x = captureBuffer[n].value;
    if(midN == -1 && signum(x - midPoint) == cmpsgn) {
      midN = n;
    }
    if(endN == -1 && signum(x - endPoint) == cmpsgn) {
      endN = n;
    }
  }

  // Noise can mean that if our start/end points are too close to the start/end means, then the startPoint can 
  // be breached erroneously.  If we go through the samples backwards, hopefully we won't jump the gun!
  for(int n=SAMPLE_BUFFER_LENGTH-1;n>=0;--n) {
    int x = captureBuffer[n].value;
    if(startN == -1 && signum(startPoint - x) == cmpsgn) {
      startN = n;
    }
  }
  
  if(abs(startMean - endMean) < 10) {    
    sprintf(msgBufferA, "ERR:range low");
    sprintf(msgBufferB, "(%d-%d)", (int)startMean, (int)endMean);
    Serial.println("DEBUG: range low");
  } else if(midN == -1 || startN == -1 || endN == -1) {
    sprintf(msgBufferA, "ERR:crossing");
    sprintf(msgBufferB, "not found");
    Serial.println("DEBUG: crossing not found");
  } else {
    int midTime = captureBuffer[midN].timestamp;
    int responseTime = captureBuffer[endN].timestamp - captureBuffer[startN].timestamp;
    
    sprintf(msgBufferA, "L:%d R:%d", midTime, responseTime);
    sprintf(msgBufferB, "Rng:%d-%d", (int)startMean, (int)endMean);

    multiRunLagCount++;
    multiRunLagSum += midTime;
    multiRunLagSumSq += midTime * midTime;
        
    Serial.println(msgBufferA);
    Serial.println(msgBufferB);    
  }  
}

void processMultiResults() {
  if(multiRunLagCount > 0) {    
    float mean = multiRunLagSum / multiRunLagCount;    
    float stddev = sqrt(multiRunLagSumSq / multiRunLagCount - mean * mean);
  
    sprintf(msgBufferA, "L(m):%d std: %d", (int)(mean + 0.5), (int)(stddev + 0.5));
    sprintf(msgBufferB, "count:%d", multiRunLagCount);  
  } else {
    sprintf(msgBufferA, "no valid");  
    sprintf(msgBufferB, "readings");  
  }
}
