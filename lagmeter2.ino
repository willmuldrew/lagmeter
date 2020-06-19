// libs required:
// u8g2
// 

#include <Arduino.h>
#include <Keyboard.h>
#include <Mouse.h>
#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define SAMPLE_BUFFER_LENGTH 144  // not much memory on arduino!

// are we using/do we have a mems MPU attached - can tape one to a real keyboard and use it to 
// trigger capture when we hammer a key!
//#define HAS_MPU

#ifdef HAS_MPU 
float mag_a_prev = -1;
const int MPU=0x68;
#endif

// Target interval for light capture measurements
const int sampleInterval = 1500; // micros - approx

// How long to busy sample the sensor (then applies mean)
const int sampleDuration = 1000;

typedef struct datapoint {
  uint16_t timestamp;
  uint16_t value;
} datapoint_t;

// Global state for the light-level capture buffer
long t0 = 0;
int writePosition = SAMPLE_BUFFER_LENGTH ;
datapoint_t captureBuffer[SAMPLE_BUFFER_LENGTH];

// OLED display status messages
char msgBufferA[17];
char msgBufferB[17];

// Flag for button press interrupt
boolean buttonPressed = false;

// if we've send a keyboard event, then after a short time we want to delete it again
boolean doBackspace = false;

void setup() {
  u8g2.begin();
    
  Serial.begin(115200);

  // Button to trigger keypress/capture
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), onButtonPress, FALLING);

  clearCaptureBuffer();
  Keyboard.begin();
  Mouse.begin();

#ifdef HAS_MPU
  setupMPU();
#endif

  writeScreenMessage("Ready...", "");
  drawScreen();
}

void writeScreenMessage(const char* line1, const char* line2) {
  snprintf(msgBufferA, sizeof(msgBufferA), line1);
  snprintf(msgBufferB, sizeof(msgBufferB), line2);
  drawScreen();   
}

#ifdef HAS_MPU
void setupMPU() { 
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // reset 
  Wire.write(0);    
  Wire.endTransmission(true);
}

float pollMPU(float threshold) {
  // You can get the MPU-6050 to do clever motion detection and fire interrupts, but 
  // I got this working pretty easily.
  // 
  // Improvements:
  //  * use interrupts and hardware motion detection (need to decode the datasheet)
  //  * use the onboard FIFO to pull back a history 
  //
  // Useful - https://www.i2cdevlib.com/devices/mpu6050#registers

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);  
  int16_t AcX=Wire.read()<<8|Wire.read();    
  int16_t AcY=Wire.read()<<8|Wire.read();  
  int16_t AcZ=Wire.read()<<8|Wire.read();  
  
  float mag_a = ((float)AcX) * ((float)AcX) + ((float)AcY) * ((float)AcY) + ((float)AcZ) * ((float)AcZ);

  if(mag_a_prev >= 0 && abs(mag_a - mag_a_prev) > threshold) {
    mag_a_prev = -1;
    return true;
  } else {
    mag_a_prev = mag_a;  
    return false;    
  }
}
#endif // HAS_MPU


void clearCaptureBuffer() {
   memset(captureBuffer, 0, sizeof(captureBuffer));
}

// IRQ handler for button press
void onButtonPress() {
  buttonPressed = true;
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

    while(Serial.available() > 0) {
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
}

#define SYNC_TIMEOUT_MILLIS 1000

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

// Take as many readings as possible 
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

void loop() {
  
  if(isCaptureRunning()) {
    // If the capture is running we don't want to do anything else
    unsigned long t = micros();
    int reading = readSmoothedAnalog(sampleDuration);
    if(reading >= 0) {    
      captureBuffer[writePosition].value = reading;
      captureBuffer[writePosition].timestamp = (t - t0) / 1000;
      writePosition++;
    } else {
      // skip - something went a bit wrong!
    }
    
    if(writePosition == SAMPLE_BUFFER_LENGTH) {
      // We've done our last measurement - do some processing for local display and dump results to the 
      // serial port      
      dumpResultsToSerial();
      processResults();

      // If we generated a keypress, let's delete it for convenience...  Sometimes it gets annoying though
      // if your backspaces hit a browser!
      if(doBackspace) {
        delay(1000);
        Keyboard.write(KEY_BACKSPACE);
        doBackspace = false;
      }

      // Update the screen (including drawing a chart)
      drawScreen();
    } else {
      // busy loop to delay until next sample is scheduled...
      unsigned long now = micros();
      while(now >= t && now < t + sampleInterval) {
        now = micros(); 
      }
    }
  } else {
    // process inputs    
    if(buttonPressed) {        
      if(trySerialClockSync()) {
        delay(1000); // give the remote host a chance to kick off pcaps etc...
      }       
      startCapture();
      Keyboard.print("x");
      doBackspace = true;
  //    Mouse.click();
      buttonPressed = false;
    } 
#ifdef HAS_MPU
    else if(pollMPU(100e6)) {
      // TODO - since we don't control the physical keyboard and the physical keypress is already on it's way across the USB bus
      // we don't want to spend any time doing a clock sync or triggering a pcap.
      //
      // However, we could a). have a continuous pcap into a ring buffer on the host and b). do the clock sync and triggering 
      // after the arduino capture is finished. This would establish the clock offsets, and also tell the lagcap program to 
      // use the last X millis of pcap as results.
      startCapture();
    }   
#endif
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

  if(abs(startMean - endMean) < 20) {    
    sprintf(msgBufferA, "ERR:range low");
    sprintf(msgBufferB, "(%d-%d)", (int)startMean, (int)endMean);
  } else if(midN == -1 || startN == -1 || endN == -1) {
    sprintf(msgBufferA, "ERR:crossing");
    sprintf(msgBufferB, "not found");
  } else {
    int midTime = captureBuffer[midN].timestamp;
    int responseTime = captureBuffer[endN].timestamp - captureBuffer[startN].timestamp;
    
    Serial.println(startN);
    Serial.println(endN);    
    Serial.println(captureBuffer[startN].timestamp);
    Serial.println(captureBuffer[endN].timestamp);
    
    sprintf(msgBufferA, "L:%d R:%d", midTime, responseTime);
    sprintf(msgBufferB, "Rng:%d-%d", (int)startMean, (int)endMean);
  }
}
