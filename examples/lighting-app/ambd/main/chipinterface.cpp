#include <platform_stdlib.h>

#include "Globals.h"
#include "LEDWidget.h"
#include "CHIPDeviceManager.h"
#include "DeviceCallbacks.h"
#include "Server.h"

#include <platform/CHIPDeviceLayer.h>
#include <support/CHIPMem.h>
#include "chip_porting.h"

#include <platform/AMBD/AMBDConfig.h>

extern "C"{ void * __dso_handle = 0 ;}

using namespace ::chip;
using namespace ::chip::DeviceManager;
using namespace ::chip::DeviceLayer;

namespace
{
    void SetupPretendDevices(void)
    {
        //TODO
    }

    class AppCallbacks : public AppDelegate
    {
    public:
        void OnReceiveError() override { };
        void OnRendezvousStarted() override {  };
        void OnRendezvousStopped() override {};
        void OnPairingWindowOpened() override { };
        void OnPairingWindowClosed() override { };
    };

} // namespace

#ifdef CONFIG_PLATFORM_8721D
#define STATUS_LED_GPIO_NUM PB_5
#elif defined(CONFIG_PLATFORM_8710C)
#define STATUS_LED_GPIO_NUM PA_20
#else
#define STATUS_LED_GPIO_NUM NC
#endif

static DeviceCallbacks EchoCallbacks;

extern "C" void ChipTest(void)
{
    printf("In ChipTest()\r\n");
    CHIP_ERROR err = CHIP_NO_ERROR;

    printf("initPrefr\n");
    initPref();

    CHIPDeviceManager &deviceMgr = CHIPDeviceManager::GetInstance();

    err = deviceMgr.Init(&EchoCallbacks);

    if (err != CHIP_NO_ERROR)
    {
        printf("DeviceManagerInit() - ERROR!\r\n");
    }
    else
    {
        printf("DeviceManagerInit() - OK\r\n");
    }

    SetupPretendDevices();

    AppCallbacks callbacks;
    chip::Server::GetInstance().Init(&callbacks);

    statusLED1.Init(STATUS_LED_GPIO_NUM);

    while(true)
        vTaskDelay( pdMS_TO_TICKS(50) );
}

bool lowPowerClusterSleep()
{
    return true;
}