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


extern "C" void DCTTest(void)
{
    chip::DeviceLayer::Internal::AMBDConfig ttest;

    char buf[10];
    uint8_t barray[100],carray[100];
    size_t outLen;
    uint32_t val1 = 555, val2=0;
    bool a=1,b=0, _exist=0;
    int8_t i = 0;

    // string
    printf("===== string ===== \n");
    ttest.WriteConfigValueStr(ttest.kConfigKey_SerialNum,"aaabbbccc");
    ttest.ReadConfigValueStr(ttest.kConfigKey_SerialNum,buf, sizeof(buf),outLen);
    printf("buf = %s, outLen=%d \n",buf, outLen);

    // u32
    printf("===== u32 ===== \n");
    ttest.WriteConfigValue(ttest.kConfigKey_SerialNum,val1);
    printf("val2=%lu\n",val2);
    ttest.ReadConfigValue(ttest.kConfigKey_SerialNum,val2);
    printf("val2=%lu\n",val2);
    _exist = ttest.ConfigValueExists(ttest.kConfigKey_SerialNum);
    printf("SerialNum exist = %d\n",_exist);

    // bool
    printf("===== bool ===== \n");
    printf("b=%d\n",b);
    ttest.WriteConfigValue(ttest.kConfigKey_MfrDeviceId,a);
    ttest.ReadConfigValue(ttest.kConfigKey_MfrDeviceId,b);
    printf("b=%d\n",b);

    // binary
    printf("===== binary ===== \n");
    for(i=0;i<100;i++){
        barray[i]=i;
        carray[i]=0;
    }

    printf("array=\n");
    for(i=0;i<100;i++)
        printf("%d ",carray[i]);
    printf("\n");

    ttest.WriteConfigValueBin(ttest.kConfigKey_MfrDeviceId,barray,sizeof(barray));
    ttest.ReadConfigValueBin(ttest.kConfigKey_MfrDeviceId,carray,sizeof(carray),outLen);

    printf("outlen = %d\n",outLen);
    printf("array=\n");
    for(i=0;i<100;i++)
        printf("%d ",carray[i]);
    printf("\n");

    _exist = ttest.ConfigValueExists(ttest.kConfigKey_MfrDeviceId);
    printf("MfrDeviceId exist = %d\n",_exist);
    printf("delete kConfigKey_MfrDeviceId\n");
    ttest.ClearConfigValue(ttest.kConfigKey_MfrDeviceId);
    _exist = ttest.ConfigValueExists(ttest.kConfigKey_MfrDeviceId);
    printf("MfrDeviceId exist = %d\n",_exist);
}

extern "C" void ChipTest(void)
{
    printf("In ChipTest()\r\n");
    CHIP_ERROR err = CHIP_NO_ERROR;

    printf("initPrefr\n");
    initPref();

    //DCTTest();

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
    InitServer(&callbacks);

    statusLED1.Init(STATUS_LED_GPIO_NUM);

    while(true)
        vTaskDelay( pdMS_TO_TICKS(50) );
}

bool lowPowerClusterSleep()
{
    return true;
}
