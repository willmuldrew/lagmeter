#include <windows.h>
#include <d2d1.h>
#pragma comment(lib, "d2d1")

#include "basewin.h"

#include <vector>

template <class T> void SafeRelease(T **ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

class MainWindow : public BaseWindow<MainWindow>
{
    ID2D1Factory            *pFactory;
    ID2D1HwndRenderTarget   *pRenderTarget;
    ID2D1SolidColorBrush    *pBrush;
    ID2D1SolidColorBrush    *pAltBrush;
    
    boolean altColour;
    std::vector<struct WinMsg*> messages;

    HRESULT CreateGraphicsResources();
    void    DiscardGraphicsResources();
    void    OnPaint();

    void MainWindow::clearMessages();
    void MainWindow::dumpMessages();

public:

    MainWindow() : pFactory(NULL), pRenderTarget(NULL), pBrush(NULL), altColour(false)
    {
    }

    PCWSTR  ClassName() const { return L"LagWinApp Window Class"; }
    LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam);
};


HRESULT MainWindow::CreateGraphicsResources()
{
    HRESULT hr = S_OK;
    if (pRenderTarget == NULL)
    {
        RECT rc;
        GetClientRect(m_hwnd, &rc);

        D2D1_SIZE_U size = D2D1::SizeU(rc.right, rc.bottom);

        hr = pFactory->CreateHwndRenderTarget(
            D2D1::RenderTargetProperties(),
            D2D1::HwndRenderTargetProperties(m_hwnd, size),
            &pRenderTarget);

        if (SUCCEEDED(hr))
        {
            hr = pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(1.0f, 1.0f, 1.0f), &pBrush);
            hr = pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(0, 0, 0), &pAltBrush);
        }
    }
    return hr;
}

void MainWindow::DiscardGraphicsResources()
{
    SafeRelease(&pRenderTarget);
    SafeRelease(&pBrush);
}

void MainWindow::OnPaint()
{
    HRESULT hr = CreateGraphicsResources();
    if (SUCCEEDED(hr))
    {
        PAINTSTRUCT ps;
        BeginPaint(m_hwnd, &ps);
        pRenderTarget->BeginDraw();
        pRenderTarget->Clear(this->altColour ? D2D1::ColorF(D2D1::ColorF::Black) : D2D1::ColorF(D2D1::ColorF::White));
        hr = pRenderTarget->EndDraw();
        if (FAILED(hr) || hr == D2DERR_RECREATE_TARGET)
        {
            DiscardGraphicsResources();
        }
        EndPaint(m_hwnd, &ps);
    }
}

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR, int nCmdShow)
{
    MainWindow win;

    if (!win.Create(L"LagWinApp - hit x to toggle black/white", WS_OVERLAPPEDWINDOW))
    {
        return 0;
    }

    ShowWindow(win.Window(), nCmdShow);

    // Run the message loop.

    MSG msg = { };
    while (GetMessage(&msg, NULL, 0, 0))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return 0;
}

struct WinMsg {
    int64_t timestampMillis;
    char* message;

    WinMsg(int64_t timestampMillis, char* message) {
        this->timestampMillis = timestampMillis;
        this->message = message;
    }
};

int64_t getUnixTimeMicros() {
    FILETIME now;
    GetSystemTimeAsFileTime(&now);

    const int64_t UNIX_TIME_START = 0x019DB1DED53E8000; //January 1, 1970 (start of Unix epoch) in "ticks"
    const int64_t TICKS_PER_MICRO = 10; //a tick is 100ns

    LARGE_INTEGER li;
    li.LowPart = now.dwLowDateTime;
    li.HighPart = now.dwHighDateTime;

    //Convert ticks since 1/1/1970 into millis
    return (li.QuadPart - UNIX_TIME_START) / TICKS_PER_MICRO;
}

void MainWindow::clearMessages() {
    for (auto msg = messages.begin(); msg != messages.end(); ++msg) {
        delete (*msg);
    }
    messages.clear();
}

void MainWindow::dumpMessages() {
    char tmpBuff[1024];    
    char pathBuff[2048];

    GetTempPathA(1024, tmpBuff);
    snprintf(pathBuff, sizeof(pathBuff), "%s\\lagwinapp_msgs.csv", tmpBuff);

    FILE* f = NULL;        
    if (fopen_s(&f, pathBuff, "w") == 0) {

        fprintf(f, "timestampMicros,msg\r\n");
        for (auto msg = messages.begin(); msg != messages.end(); ++msg) {
            WinMsg* pMsg = (*msg);
            fprintf(f, "%lld,%s\r\n", pMsg->timestampMillis, pMsg->message);
        }

        fclose(f);
        OutputDebugStringA("Written messages to: ");
        OutputDebugStringA(pathBuff);
        OutputDebugStringA("\r\n");
    }
    else 
    {
        OutputDebugStringA("Unable to open file\r\n");
    }
}

LRESULT MainWindow::HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam)
{
    switch (uMsg)
    {
    case WM_CREATE:
        if (FAILED(D2D1CreateFactory(
            D2D1_FACTORY_TYPE_SINGLE_THREADED, &pFactory)))
        {
            return -1;  // Fail CreateWindowEx.
        }
        return 0;

    case WM_DESTROY:
        DiscardGraphicsResources();
        SafeRelease(&pFactory);
        PostQuitMessage(0);
        return 0;
    case WM_PAINT:        
        messages.push_back(new WinMsg(getUnixTimeMicros(), "WM_PAINT (pre)"));
        OnPaint();
        messages.push_back(new WinMsg(getUnixTimeMicros(), "WM_PAINT (post)"));        
        return 0;
    case WM_KEYDOWN:
    case WM_LBUTTONDOWN:
        clearMessages();
        messages.push_back(new WinMsg(getUnixTimeMicros(), "WM_KEYDOWN(x)"));
        altColour = !altColour;
        RedrawWindow(m_hwnd, NULL, NULL, RDW_INVALIDATE | RDW_UPDATENOW);
        return 0;
    case WM_KEYUP:
    case WM_LBUTTONUP:
        dumpMessages();
        clearMessages();
    
    }

    return DefWindowProc(m_hwnd, uMsg, wParam, lParam);
}
