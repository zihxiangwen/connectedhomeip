/*
 *
 *    Copyright (c) 2020-2021 Project CHIP Authors
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

/**
 *    @file
 *          Provides an implementation of the BLEManager singleton object
 *          for the Ameba platforms.
 */

/* this file behaves like a config.h, comes first */
#include <platform/internal/CHIPDeviceLayerInternal.h>
#include <crypto/CHIPCryptoPAL.h>

#if CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE
#include <ble/CHIPBleServiceData.h>

#include "stdio.h"
#include "timers.h"

//Ameba BLE related header files
#include "gap_conn_le.h"
#include "bt_config_app_task.h"
#include "bt_config_app_main.h"
#include "bt_config_peripheral_app.h"
#include "bte.h"
#include "rtk_coex.h"
#include "trace_app.h"
#include "gap_adv.h"
#include "wifi_conf.h"
#include "gap_adv.h"
#include "gap.h"
#include "os_sched.h"

extern void wifi_bt_coex_set_bt_on(void);
/*******************************************************************************
 * Local data types
 *******************************************************************************/
using namespace ::chip;
using namespace ::chip::Ble;

namespace chip {
namespace DeviceLayer {
namespace Internal {

namespace {

/*******************************************************************************
 * Macros & Constants definitions
 *******************************************************************************/
#define APP_MAX_LINKS                   4
#define MAX_ADV_DATA_LEN                31
#define CHIP_ADV_DATA_TYPE_FLAGS        0x01
#define CHIP_ADV_DATA_FLAGS             0x06
#define CHIP_ADV_DATA_TYPE_SERVICE_DATA 0x16

#define LOOP_EV_BLE (0x08)

/* ble app task configuration */
#define CHIP_DEVICE_CONFIG_BLE_APP_TASK_PRIORITY (HOST_TASK_PRIORITY - 1)
#define CHIP_DEVICE_CONFIG_BLE_APP_TASK_STACK_SIZE (1024)

/* advertising configuration */
#define CHIP_ADV_SHORT_UUID_LEN (2)

/* FreeRTOS sw timer */
TimerHandle_t sbleAdvTimeoutTimer;

/* Used by BLE App Task to handle asynchronous GATT events */
EventGroupHandle_t bleAppTaskLoopEvent;

/* keep the device ID of the connected peer */
uint8_t device_id;

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
[zl_dbg] From esp sdk, need to change */

/** Type of UUID */
enum {
    /** 16-bit UUID (BT SIG assigned) */
    BLE_UUID_TYPE_16 = 16,

    /** 32-bit UUID (BT SIG assigned) */
    BLE_UUID_TYPE_32 = 32,

    /** 128-bit UUID */
    BLE_UUID_TYPE_128 = 128,
};

typedef struct {
    /** Type of the UUID */
    uint8_t type;
} ble_uuid_t;

/** 16-bit UUID */
typedef struct {
    ble_uuid_t u;
    uint16_t value;
} ble_uuid16_t;
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

const ble_uuid16_t ShortUUID_CHIPoBLEService = { BLE_UUID_TYPE_16, 0xFFF6 };
const ChipBleUUID ChipUUID_CHIPoBLEChar_RX = { { 0x18, 0xEE, 0x2E, 0xF5, 0x26, 0x3D, 0x45, 0x59, 0x95, 0x9F, 0x4F, 0x9C, 0x42, 0x9F,
                                                 0x9D, 0x11 } };
const ChipBleUUID ChipUUID_CHIPoBLEChar_TX = { { 0x18, 0xEE, 0x2E, 0xF5, 0x26, 0x3D, 0x45, 0x59, 0x95, 0x9F, 0x4F, 0x9C, 0x42, 0x9F,
                                                 0x9D, 0x12 } };
} // namespace

BLEManagerImpl BLEManagerImpl::sInstance;

CHIP_ERROR BLEManagerImpl::_Init()
{
printf("BLEManagerImpl::_Init----------------------------------------------Start\r\n");
    CHIP_ERROR err;

    // Initialize the CHIP BleLayer.
    err = BleLayer::Init(this, this, &SystemLayer);
    SuccessOrExit(err);

    mServiceMode = ConnectivityManager::kCHIPoBLEServiceMode_Enabled;

    // Check if BLE stack is initialized
    VerifyOrExit(!mFlags.Has(Flags::kAMEBABLEStackInitialized), err = CHIP_ERROR_INCORRECT_STATE);

    /*[zl_dbg]Add Ameba Init function*/
 /*   bt_trace_init();
    bt_config_stack_config_init();
    bte_init();
    le_gap_init(APP_MAX_LINKS);
    bt_config_app_le_profile_init();
    bt_config_app_le_gap_init();
    bt_config_task_init();

    bt_coex_init();
*/
    printf("\n************************************************Before bt_config_init**********************************************\r\n");
    err = bt_config_init();
    printf("\n***********************************************After bt_config_init((((((((((((((((((((((((((((((((((((((((((((((((\r\n");
    SuccessOrExit(err);

    //Set related flags
    mFlags.ClearAll().Set(Flags::kAdvertisingEnabled, CHIP_DEVICE_CONFIG_CHIPOBLE_ENABLE_ADVERTISING_AUTOSTART);
    mFlags.Set(Flags::kAMEBABLEStackInitialized);
    mFlags.Set(Flags::kAdvertisingEnabled, CHIP_DEVICE_CONFIG_CHIPOBLE_ENABLE_ADVERTISING_AUTOSTART ? true : false);
    mFlags.Set(Flags::kFastAdvertisingEnabled);

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] Maynot need to call DriveBLEState here, if we init BLEStack here (not in DriveBLEState)

    PlatformMgr().ScheduleWork(DriveBLEState, 0);
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    PlatformMgr().ScheduleWork(DriveBLEState, 0);

exit:
    return err;
}

uint16_t BLEManagerImpl::_NumConnections(void)
{
printf("BLEManagerImpl::_NumConnections:::::::::::::::\r\n");
    uint16_t numCons = 0;
    for (uint16_t i = 0; i < kMaxConnections; i++)
    {
        if (mSubscribedConIds[i] != BLE_CONNECTION_UNINITIALIZED)
        {
            numCons++;
        }
    }

    return numCons;
}

bool BLEManagerImpl::RemoveConnection(uint8_t connectionHandle)
{
    CHIPoBLEConState * bleConnState = GetConnectionState(connectionHandle, true);
    bool status                     = false;

    if (bleConnState != NULL)
    {
        memset(bleConnState, 0, sizeof(CHIPoBLEConState));
        status = true;
    }
    return status;
}

void BLEManagerImpl::AddConnection(uint8_t connectionHandle)
{
    CHIPoBLEConState * bleConnState = GetConnectionState(connectionHandle, true);

    if (bleConnState != NULL)
    {
        memset(bleConnState, 0, sizeof(CHIPoBLEConState));
        bleConnState->allocated        = 1;
        bleConnState->connectionHandle = connectionHandle;
    }
}

BLEManagerImpl::CHIPoBLEConState * BLEManagerImpl::GetConnectionState(uint8_t connectionHandle, bool allocate)
{
    uint8_t freeIndex = kMaxConnections;

    for (uint8_t i = 0; i < kMaxConnections; i++)
    {
        if (mBleConnections[i].allocated == 1)
        {
            if (mBleConnections[i].connectionHandle == connectionHandle)
            {
                return &mBleConnections[i];
            }
        }

        else if (i < freeIndex)
        {
            freeIndex = i;
        }
    }

    if (allocate)
    {
        if (freeIndex < kMaxConnections)
        {
            return &mBleConnections[freeIndex];
        }

        ChipLogError(DeviceLayer, "Failed to allocate CHIPoBLEConState");
    }

    return NULL;
}

CHIP_ERROR BLEManagerImpl::_SetCHIPoBLEServiceMode(CHIPoBLEServiceMode val)
{
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(val != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_INVALID_ARGUMENT);
    VerifyOrExit(mServiceMode != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (val != mServiceMode)
    {
        mServiceMode = val;
        PlatformMgr().ScheduleWork(DriveBLEState, 0);
    }

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::_SetAdvertisingEnabled(bool val)
{
printf("BLEManagerImpl::_SetAdvertisingEnabled----------------------------------------------OK \r\n");
printf("val = %d\r\n",val);
    CHIP_ERROR err = CHIP_NO_ERROR;

    VerifyOrExit(mServiceMode != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (mFlags.Has(Flags::kAdvertisingEnabled) != val)
    {
        mFlags.Set(Flags::kAdvertisingEnabled, val);
        PlatformMgr().ScheduleWork(DriveBLEState, 0);
    }

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::_SetAdvertisingMode(BLEAdvertisingMode mode)
{
    switch (mode)
    {
    case BLEAdvertisingMode::kFastAdvertising:
        mFlags.Set(Flags::kFastAdvertisingEnabled, true);
        break;
    case BLEAdvertisingMode::kSlowAdvertising:
        mFlags.Set(Flags::kFastAdvertisingEnabled, false);
        break;
    default:
        return CHIP_ERROR_INVALID_ARGUMENT;
    }
    mFlags.Set(Flags::kRestartAdvertising);
    PlatformMgr().ScheduleWork(DriveBLEState, 0);
    return CHIP_NO_ERROR;
}

CHIP_ERROR BLEManagerImpl::_GetDeviceName(char * buf, size_t bufSize)
{
printf("BLEManagerImpl::_GetDeviceName:::::::::::::::\r\n");
    if (strlen(mDeviceName) >= bufSize)
    {
        return CHIP_ERROR_BUFFER_TOO_SMALL;
    }
    strcpy(buf, mDeviceName);
    return CHIP_NO_ERROR;
}

CHIP_ERROR BLEManagerImpl::_SetDeviceName(const char * deviceName)
{
printf("BLEManagerImpl::_SetDeviceName:::::::::::::::\r\n");
    CHIP_ERROR err = CHIP_NO_ERROR;
    
    VerifyOrExit(mServiceMode != ConnectivityManager::kCHIPoBLEServiceMode_NotSupported, err = CHIP_ERROR_UNSUPPORTED_CHIP_FEATURE);

    if (deviceName != NULL && deviceName[0] != 0)
    {
        if (strlen(deviceName) >= kMaxDeviceNameLength)
        {
            return CHIP_ERROR_INVALID_ARGUMENT;
        }
        strcpy(mDeviceName, deviceName);
        mFlags.Set(Flags::kDeviceNameSet);
        ChipLogProgress(DeviceLayer, "Setting device name to : \"%s\"", deviceName);
    }
    else
    {
        mDeviceName[0] = 0;
        mFlags.Clear(Flags::kDeviceNameSet);
    }

exit:
    return err;
}

void BLEManagerImpl::_OnPlatformEvent(const ChipDeviceEvent * event)
{
printf("BLEManagerImpl::_OnPlatformEvent:::::::::::::::\r\n");
    switch (event->Type)
    {
    // Platform specific events
    case DeviceEventType::kCHIPoBLESubscribe:
        HandleSubscribeReceived(event->CHIPoBLESubscribe.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_TX);
        {
            ChipDeviceEvent connEstEvent;
            connEstEvent.Type = DeviceEventType::kCHIPoBLEConnectionEstablished;
            PlatformMgr().PostEvent(&connEstEvent);
        }
	break;

    case DeviceEventType::kCHIPoBLEUnsubscribe: {
            ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEUnsubscribe");
            HandleUnsubscribeReceived(event->CHIPoBLEUnsubscribe.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_TX);
        }
        break;

    case DeviceEventType::kCHIPoBLEWriteReceived: {
            ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEWriteReceived");
            HandleWriteReceived(event->CHIPoBLEWriteReceived.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_RX, PacketBufferHandle::Adopt(event->CHIPoBLEWriteReceived.Data));
        }
        break;

    case DeviceEventType::kCHIPoBLEConnectionError: {
            ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEConnectionError");
            HandleConnectionError(event->CHIPoBLEConnectionError.ConId, event->CHIPoBLEConnectionError.Reason);
        }
        break;

    case DeviceEventType::kCHIPoBLEIndicateConfirm: {
            ChipLogProgress(DeviceLayer, "_OnPlatformEvent kCHIPoBLEIndicateConfirm");
            HandleIndicationConfirmation(event->CHIPoBLEIndicateConfirm.ConId, &CHIP_BLE_SVC_ID, &ChipUUID_CHIPoBLEChar_TX);
        }
        break;

    case DeviceEventType::kFabricMembershipChange:
    case DeviceEventType::kServiceProvisioningChange:
    case DeviceEventType::kAccountPairingChange:
    case DeviceEventType::kWiFiConnectivityChange:

// If CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED is enabled, when there is a change to the device's provisioning state and device is now fully provisioned, the CHIPoBLE advertising will be disabled.
#if CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED
    if (ConfigurationMgr().IsFullyProvisioned())
    {
        mFlags.Clear(Flags::kAdvertisingEnabled);
        ChipLogProgress(DeviceLayer, "CHIPoBLE advertising disabled because device is fully provisioned");
    }
#endif // CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED
    ChipLogProgress(DeviceLayer, "Updating advertising data");

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] See if the kAdvertisingRefresh is needed. If so, set flag and call DriveBLEState

    //mFlags.Clear(Flags::kAdvertisingConfigured);
    //mFlags.Set(Flags::kAdvertisingRefreshNeeded);
    //DriveBLEState();
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    break;
    
    default:
        ChipLogProgress(DeviceLayer, "_OnPlatformEvent default:  event->Type = %d", event->Type);
        break;
    }
}

bool BLEManagerImpl::SubscribeCharacteristic(BLE_CONNECTION_OBJECT conId, const ChipBleUUID * svcId, const ChipBleUUID * charId)
{
    printf("BLEManagerImpl::SubscribeCharacteristic:::::::::::::::OK\r\n");
    ChipLogProgress(DeviceLayer, "BLEManagerImpl::SubscribeCharacteristic() not supported");
    return false;
}

bool BLEManagerImpl::UnsubscribeCharacteristic(BLE_CONNECTION_OBJECT conId, const ChipBleUUID * svcId, const ChipBleUUID * charId)
{
    printf("BLEManagerImpl::UnsubscribeCharacteristic:::::::::::::::OK\r\n");
    ChipLogProgress(DeviceLayer, "BLEManagerImpl::UnsubscribeCharacteristic() not supported");
    return false;
}

bool BLEManagerImpl::CloseConnection(BLE_CONNECTION_OBJECT conId)
{
    printf("BLEManagerImpl::CloseConnection:::::::::::::::OK\r\n");
    CHIP_ERROR err;

    ChipLogProgress(DeviceLayer, "Closing BLE GATT connection (con %u)", conId);

    //Ameba Ble close function
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] function to be vetified
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    err = MapBLEError(le_disconnect(conId));
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "le_disconnect() failed: %s", ErrorStr(err));
    }
    
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] See if the kAdvertisingRefresh is needed. If so, set flag and call DriveBLEState
    
    // Force a refresh of the advertising state.
    mFlags.Set(Flags::kAdvertisingRefreshNeeded);
    mFlags.Clear(Flags::kAdvertisingConfigured);
    PlatformMgr().ScheduleWork(DriveBLEState, 0);
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    
    return (err == CHIP_NO_ERROR);
}

uint16_t BLEManagerImpl::GetMTU(BLE_CONNECTION_OBJECT conId) const
{
    printf("BLEManagerImpl::GetMTU:::::::::::::::OK\r\n");

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] Apply similar function of esp's 'ble_att_mtu'
    int ret = 0;
    ret = ble_att_mtu(conId)
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    return 1;
}

bool BLEManagerImpl::SendWriteRequest(BLE_CONNECTION_OBJECT conId, const ChipBleUUID * svcId, const ChipBleUUID * charId, PacketBufferHandle pBuf)
{
    printf("BLEManagerImpl::SendWriteRequest:::::::::::::::OK\r\n");
    ChipLogError(DeviceLayer, "BLEManagerImpl::SendWriteRequest() not supported");
    return false;
}

bool BLEManagerImpl::SendReadRequest(BLE_CONNECTION_OBJECT conId, const ChipBleUUID * svcId, const ChipBleUUID * charId, PacketBufferHandle pBuf)
{
    printf("BLEManagerImpl::SendReadRequest:::::::::::::::OK\r\n");
    ChipLogError(DeviceLayer, "BLEManagerImpl::SendReadRequest() not supported");
    return false;
}

bool BLEManagerImpl::SendReadResponse(BLE_CONNECTION_OBJECT conId, BLE_READ_REQUEST_CONTEXT requestContext, const ChipBleUUID * svcId, const ChipBleUUID * charId)
{
    printf("BLEManagerImpl::SendReadResponse:::::::::::::::OK\r\n");
    ChipLogError(DeviceLayer, "BLEManagerImpl::SendReadResponse() not supported");
    return false;
}

void BLEManagerImpl::NotifyChipConnectionClosed(BLE_CONNECTION_OBJECT conId)
{
    // Nothing to do
}

bool BLEManagerImpl::SendIndication(BLE_CONNECTION_OBJECT conId, const ChipBleUUID * svcId, const ChipBleUUID * charId, PacketBufferHandle data)
{
    printf("BLEManagerImpl::SendIndication:::::::::::::::OK\r\n");
    CHIP_ERROR err = CHIP_NO_ERROR;
    //struct os_mbuf * om;
    
    VerifyOrExit(IsSubscribed(conId), err = CHIP_ERROR_INVALID_ARGUMENT);
    printf("Sending indication for CHIPoBLE TX characteristic (con %u, len %u)\r\n", conId, data->DataLength());

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] Apply similar function of esp 'ble_gattc_notify_custom'

    om = ble_hs_mbuf_from_flat(data->Start(), data->DataLength());
    if (om == NULL)
    {
        ChipLogError(DeviceLayer, "ble_hs_mbuf_from_flat failed:");
        err = CHIP_ERROR_NO_MEMORY;
        ExitNow();
    }
    err = MapBLEError(ble_gattc_notify_custom(conId, mTXCharCCCDAttrHandle, om));
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "ble_gattc_notify_custom() failed: %s", ErrorStr(err));
        ExitNow();
    }
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    
exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "BLEManagerImpl::SendIndication() failed: %s", ErrorStr(err));
        return false;
    }
    return true;
}

/*******************************************************************************
 * Private functions
 *******************************************************************************/

CHIP_ERROR BLEManagerImpl::ConfigureAdvertisingData(void)
{
    printf("BLEManagerImpl::ConfigureAdvertisingData:::::::::::::::OK\r\n");
    CHIP_ERROR err;
    uint8_t advData[MAX_ADV_DATA_LEN] = { 0 };
    uint8_t advPayload[MAX_ADV_DATA_LEN] = { 0 };
    uint8_t deviceIdInfoLength = 0;
    ChipBLEDeviceIdentificationInfo deviceIdInfo;
    uint8_t index = 0;
    uint32_t bleAdvTimeoutMs;
    uint16_t adv_int_min;
    uint16_t adv_int_max;
    T_GAP_DEV_STATE new_state;

    // If the device name is not specified, generate a CHIP-standard name based on the bottom digits of the Chip device id.
    uint16_t discriminator;
    printf("GET DISCRIMINATOR HERE ===============================\n");
    SuccessOrExit(err = ConfigurationMgr().GetSetupDiscriminator(discriminator));
    printf("Discriminator: %d\n", discriminator);

   if (!mFlags.Has(Flags::kDeviceNameSet))
    {
        snprintf(mDeviceName, sizeof(mDeviceName), "%s%04u", CHIP_DEVICE_CONFIG_BLE_DEVICE_NAME_PREFIX, discriminator);
        mDeviceName[kMaxDeviceNameLength] = 0;
    }

    // Configure the BLE device name.
   printf("mDeviceName: %s\n", mDeviceName);
   le_set_gap_param(GAP_PARAM_DEVICE_NAME, kMaxDeviceNameLength, mDeviceName);

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] replace ble_svc_gap_device_name_set to z2 function

    err = MapBLEError(ble_svc_gap_device_name_set(mDeviceName));
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "xxxxxxxxxxxxxxx() failed: %s", ErrorStr(err));
        ExitNow();
    }
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

    /**************** Prepare advertising data *******************************************/
    memset(advData, 0, sizeof(advData));
    advData[index++] = 0x02;                                                                // length
    advData[index++] = CHIP_ADV_DATA_TYPE_FLAGS;                                            // AD type : flags
    advData[index++] = CHIP_ADV_DATA_FLAGS;                                                 // AD value
    advData[index++] = 0x03;                                                                // length
    advData[index++] = 0x03;                                     // AD type: (Service Data - 16-bit UUID)

    advData[index++] = static_cast<uint8_t>(ShortUUID_CHIPoBLEService.value & 0xFF);        // AD value
    advData[index++] = static_cast<uint8_t>((ShortUUID_CHIPoBLEService.value >> 8) & 0xFF); // AD value

    err = ConfigurationMgr().GetBLEDeviceIdentificationInfo(deviceIdInfo);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "GetBLEDeviceIdentificationInfo(): %s", ErrorStr(err));
        ExitNow();
    }

    VerifyOrExit(index + sizeof(deviceIdInfo) <= sizeof(advData), err = CHIP_ERROR_OUTBOUND_MESSAGE_TOO_BIG);
    advData[index++] = (uint8_t) (sizeof(deviceIdInfo));				//length
    advData[index++] = 0x09;								//complete local name
    memcpy(&advData[index], &deviceIdInfo, sizeof(deviceIdInfo));
    index = static_cast<uint8_t>(index + sizeof(deviceIdInfo));

    if (mFlags.Has(Flags::kFastAdvertisingEnabled)) {
	    printf("Fast Advertising Enabled\n");
	    adv_int_min = CHIP_DEVICE_CONFIG_BLE_FAST_ADVERTISING_INTERVAL_MIN;
	    adv_int_max = CHIP_DEVICE_CONFIG_BLE_FAST_ADVERTISING_INTERVAL_MAX;
	    bleAdvTimeoutMs = CHIP_DEVICE_CONFIG_BLE_ADVERTISING_INTERVAL_CHANGE_TIME;
    }
    else {
	    printf("Using slow advertising\n");
	    adv_int_min = CHIP_DEVICE_CONFIG_BLE_SLOW_ADVERTISING_INTERVAL_MIN;
	    adv_int_max = CHIP_DEVICE_CONFIG_BLE_SLOW_ADVERTISING_INTERVAL_MAX;
	    bleAdvTimeoutMs = CHIP_DEVICE_CONFIG_BLE_ADVERTISING_TIMEOUT;
    }
    printf("bleAdvTimeoutMs: %d\n", bleAdvTimeoutMs);
    printf("adv_int_min: %d\n", adv_int_min);
    printf("adv_int_max: %d\n", adv_int_max);
    

    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    //le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(advData), (void *)advData);	//set advData
    //le_register_app_cb(bt_config_app_gap_callback);

    bt_config_app_le_gap_init_chip();
    bt_config_task_init();

    bt_coex_init();

    //Wait BT init complete*
    do {
            os_delay(100);
            le_get_gap_param(GAP_PARAM_DEV_STATE , &new_state);
    } while (new_state.gap_init_state != GAP_INIT_STATE_STACK_READY);

    //Start BT WIFI coexistence
    wifi_btcoex_set_bt_on();

    /**************** Prepare scan response data *******************************************/
    // Construct the Chip BLE Service Data to be sent in the scan response packet.
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] replace ble_gap_adv_set_data to z2 function

    err = MapBLEError(ble_gap_adv_set_data(advData, sizeof(advData)));
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "ble_gap_adv_set_data failed: %s %d", ErrorStr(err), discriminator);
        ExitNow();
    }
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::StartAdvertising(void)
{
    printf("BLEManagerImpl::StartAdvertising:::::::::::::::\r\n");
    CHIP_ERROR err           = CHIP_NO_ERROR;

    err = ConfigureAdvertisingData();		
    SuccessOrExit(err);

    if (err == CHIP_NO_ERROR)
    /* schedule NFC emulation stop */
    {
        ChipDeviceEvent advChange;
        advChange.Type = DeviceEventType::kCHIPoBLEAdvertisingChange;
        advChange.CHIPoBLEAdvertisingChange.Result = kActivity_Started;
        PlatformMgr().PostEvent(&advChange);
    }

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] Add z2 advertisinig function
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    printf("\n*********************************before le_adv_start()********************************/\n");
    le_adv_start();
    printf("\n*********************************after le_adv_start()********************************/\n");
    //err = bt_config_adv();
//    bt_config_app_set_adv_data();
//    bt_config_send_msg(1); //Start ADV
 //   set_bt_config_state(BC_DEV_IDLE); // BT Config Ready
  //  err = z2_adv_fun
    //SuccessOrExit(err);


    printf("TIMEOUT HERE\n");
    //StartBleAdvTimeoutTimer(bleAdvTimeoutMs);
    mFlags.Set(Flags::kAdvertising);
    mFlags.Clear(Flags::kRestartAdvertising);
    //printf("TIMEOUT HERE\n");
    //StartBleAdvTimeoutTimer(bleAdvTimeoutMs);

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::StopAdvertising(void)
{
    CHIP_ERROR err;

    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] need to match Ameba StopAdvertising function*/

    //bt_config_send_msg(0);
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    CancelBleAdvTimeoutTimer();

    // Change flag status to the 'not Advertising state'
    if (mFlags.Has(Flags::kAdvertising))
    {
        mFlags.Clear(Flags::kAdvertising);
        mFlags.Set(Flags::kFastAdvertisingEnabled);

        ChipLogProgress(DeviceLayer, "CHIPoBLE advertising stopped");

        // Post a CHIPoBLEAdvertisingChange(Stopped) event.
        {
            ChipDeviceEvent advChange;
            advChange.Type                             = DeviceEventType::kCHIPoBLEAdvertisingChange;
            advChange.CHIPoBLEAdvertisingChange.Result = kActivity_Stopped;
            PlatformMgr().PostEvent(&advChange);
        }
    }
    return CHIP_NO_ERROR;
}

CHIP_ERROR BLEManagerImpl::MapBLEError(int bleErr)
{
printf("BLEManagerImpl::NotifyChipConnectionClosed:::::::::::::::\r\n");
    switch (bleErr)
    {
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] need to match Ameba BLE return code to CHIP's
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    case 1:
        return CHIP_NO_ERROR;
    default:
        return CHIP_ERROR_INCORRECT_STATE;
    }
}

void BLEManagerImpl::DriveBLEState(void)
{
printf("BLEManagerImpl::DriveBLEState:::::::::::::::\r\n");
printf("mServiceMode = %d:::::::::::::::\r\n",mServiceMode);
    CHIP_ERROR err = CHIP_NO_ERROR;

    // Check if BLE stack is initialized
    VerifyOrExit(mFlags.Has(Flags::kAMEBABLEStackInitialized), /* */);

// If CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED is enabled and the device is fully provisioned, the CHIPoBLE advertising will be disabled.
#if CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED
    if (ConfigurationMgr().IsFullyProvisioned())
    {
        mFlags.Clear(Flags::kAdvertisingEnabled);
        ChipLogProgress(DeviceLayer, "CHIPoBLE advertising disabled because device is fully provisioned");
    }
#endif // CHIP_DEVICE_CONFIG_CHIPOBLE_DISABLE_ADVERTISING_WHEN_PROVISIONED

    // Start advertising if needed...
    if (mServiceMode == ConnectivityManager::kCHIPoBLEServiceMode_Enabled && mFlags.Has(Flags::kAdvertisingEnabled))
    {
        // Start/re-start advertising if not already started, or if there is a pending change
        // to the advertising configuration.
        if (!mFlags.Has(Flags::kAdvertising) || mFlags.Has(Flags::kRestartAdvertising))
        {
            err = StartAdvertising();
            SuccessOrExit(err);
        }
    }
    // Otherwise, stop advertising if it is enabled.
    else if (mFlags.Has(Flags::kAdvertising))
    {
        err = StopAdvertising();
        SuccessOrExit(err);
        ChipLogProgress(DeviceLayer, "Stopped Advertising");
    }

exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "Disabling CHIPoBLE service due to error: %s", ErrorStr(err));
        mServiceMode = ConnectivityManager::kCHIPoBLEServiceMode_Disabled;
    }
}

void BLEManagerImpl::DriveBLEState(intptr_t arg)
{
printf("BLEManagerImpl::DriveBLEState arg=%d:::::::::::::::\r\n",arg);
    sInstance.DriveBLEState();
}

/*******************************************************************************
 * BLE App Task Processing
 *******************************************************************************/


/*******************************************************************************
 * BLE stack callbacks
 *******************************************************************************/


/*******************************************************************************
 * Add to message queue functions
 *******************************************************************************/


/*******************************************************************************
 * FreeRTOS Task Management Functions
 *******************************************************************************/

void BLEManagerImpl::BleAdvTimeoutHandler(TimerHandle_t xTimer)
{
    if (sInstance.mFlags.Has(Flags::kFastAdvertisingEnabled))
    {
        ChipLogDetail(DeviceLayer, "bleAdv Timeout : Start slow advertisment");

        sInstance.mFlags.Clear(Flags::kFastAdvertisingEnabled);
        // Stop advertising, change interval and restart it;
        sInstance.StopAdvertising();
        sInstance.StartAdvertising();
        sInstance.StartBleAdvTimeoutTimer(CHIP_DEVICE_CONFIG_BLE_ADVERTISING_TIMEOUT); // Slow advertise for 15 Minutes
    }
    else if (sInstance._IsAdvertisingEnabled())
    {
        // Advertisement time expired. Stop advertising
        ChipLogDetail(DeviceLayer, "bleAdv Timeout : Stop advertisement");
        sInstance.StopAdvertising();
    }
}

void BLEManagerImpl::CancelBleAdvTimeoutTimer(void)
{
    if (xTimerStop(sbleAdvTimeoutTimer, 0) == pdFAIL)
    {
        ChipLogError(DeviceLayer, "Failed to stop BledAdv timeout timer");
    }
}

void BLEManagerImpl::StartBleAdvTimeoutTimer(uint32_t aTimeoutInMs)
{
    if (xTimerIsTimerActive(sbleAdvTimeoutTimer))
    {
        CancelBleAdvTimeoutTimer();
    }

    // timer is not active, change its period to required value (== restart).
    // FreeRTOS- Block for a maximum of 100 ticks if the change period command
    // cannot immediately be sent to the timer command queue.
    if (xTimerChangePeriod(sbleAdvTimeoutTimer, aTimeoutInMs / portTICK_PERIOD_MS, 100) != pdPASS)
    {
        ChipLogError(DeviceLayer, "Failed to start BledAdv timeout timer");
    }
}


bool BLEManagerImpl::UnsetSubscribed(uint16_t conId)
{
printf("BLEManagerImpl::UnsetSubscribed:::::::::::::::OK\r\n");
    for (uint16_t i = 0; i < kMaxConnections; i++)
    {
        if (mSubscribedConIds[i] == conId)
        {
            mSubscribedConIds[i] = BLE_CONNECTION_UNINITIALIZED;
            return true;
        }
    }
    return false;
}

bool BLEManagerImpl::IsSubscribed(uint16_t conId)
{
printf("BLEManagerImpl::IsSubscribed:::::::::::::::OK\r\n");
    if (conId != BLE_CONNECTION_UNINITIALIZED)
    {
        for (uint16_t i = 0; i < kMaxConnections; i++)
        {
            if (mSubscribedConIds[i] == conId)
            {
                return true;
            }
        }
    }
    return false;
}


} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
#endif // CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE
