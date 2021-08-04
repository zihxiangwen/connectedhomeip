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
#include "bt_config_service.h"
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
#include "bt_config_service.h"
#include "profile_server.h"
#include "complete_ble_service.h"
#include "app_msg.h"

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
	chip_blemgr_set_callback_func((chip_blemgr_callback)(ble_callback_dispatcher), this);
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

void BLEManagerImpl::HandleTXCharRead(struct ble_gatt_char_context * param)
{
printf("BLEManagerImpl::HandleTXCharRead -------------------------------- not supported\r\n");
    /* Not supported */
    ChipLogError(DeviceLayer, "BLEManagerImpl::HandleTXCharRead() not supported");
}

void BLEManagerImpl::HandleTXCharCCCDRead(void * param)
{
printf("BLEManagerImpl::HandleTXCharCCCDRead -------------------------------- not supported\r\n");
    /* Not Supported */
    ChipLogError(DeviceLayer, "BLEManagerImpl::HandleTXCharCCCDRead() not supported");
}

void BLEManagerImpl::HandleTXCharCCCDWrite(struct ble_gap_event * gapEvent)
{
printf("BLEManagerImpl::HandleTXCharCCCDWrite xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx TB Verified\r \n");
    CHIP_ERROR err = CHIP_NO_ERROR;
    bool indicationsEnabled;
    bool notificationsEnabled;

    //ChipLogProgress(DeviceLayer,"Write request command received for CHIPoBLE TX CCCD characteristic (con %" PRIu16" ) indicate = %d notify = %d",gapEvent->subscribe.conn_handle, gapEvent->subscribe.cur_indicate, gapEvent->subscribe.cur_notify);

    // Determine if the client is enabling or disabling indications/notification.
    indicationsEnabled   = gapEvent->subscribe.cur_indicate;
    notificationsEnabled = gapEvent->subscribe.cur_notify;

    // If the client has requested to enabled indications/notifications
    if (indicationsEnabled || notificationsEnabled)
    {
        // If indications are not already enabled for the connection...
        if (!IsSubscribed(gapEvent->subscribe.conn_handle))
        {
            // Record that indications have been enabled for this connection.  If this fails because
            err = SetSubscribed(gapEvent->subscribe.conn_handle);
            VerifyOrExit(err != CHIP_ERROR_NO_MEMORY, err = CHIP_NO_ERROR);
            SuccessOrExit(err);
        }
    }

    else
    {
        // If indications had previously been enabled for this connection, record that they are no longer
        // enabled.
        UnsetSubscribed(gapEvent->subscribe.conn_handle);
    }

    // Post an event to the Chip queue to process either a CHIPoBLE Subscribe or Unsubscribe based on
    // whether the client is enabling or disabling indications.
    {
        ChipDeviceEvent event;
        event.Type = (indicationsEnabled || notificationsEnabled) ? DeviceEventType::kCHIPoBLESubscribe
                                                                  : DeviceEventType::kCHIPoBLEUnsubscribe;
        event.CHIPoBLESubscribe.ConId = gapEvent->subscribe.conn_handle;
        PlatformMgr().PostEvent(&event);
    }

    ChipLogProgress(DeviceLayer, "CHIPoBLE %s received",
                    (indicationsEnabled || notificationsEnabled) ? "subscribe" : "unsubscribe");

exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "HandleTXCharCCCDWrite() failed: %s", ErrorStr(err));
        // TODO: fail connection???
    }

    return;
}

CHIP_ERROR BLEManagerImpl::HandleTXComplete(struct ble_gap_event * gapEvent)
{
printf("BLEManagerImpl::HandleTXComplete xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx TB Verified\r\n");
    ChipLogProgress(DeviceLayer, "Confirm received for CHIPoBLE TX characteristic indication (con %" PRIu16 ") status= %d ",
                    gapEvent->notify_tx.conn_handle, gapEvent->notify_tx.status);

    // Signal the BLE Layer that the outstanding indication is complete.
    if (gapEvent->notify_tx.status == 0 || gapEvent->notify_tx.status == 14/*BLE_HS_EDONE!!!!*/)
    {
        // Post an event to the Chip queue to process the indicate confirmation.
        ChipDeviceEvent event;
        event.Type                          = DeviceEventType::kCHIPoBLEIndicateConfirm;
        event.CHIPoBLEIndicateConfirm.ConId = gapEvent->notify_tx.conn_handle;
        PlatformMgr().PostEvent(&event);
    }

    else
    {
        ChipDeviceEvent event;
        event.Type                           = DeviceEventType::kCHIPoBLEConnectionError;
        event.CHIPoBLEConnectionError.ConId  = gapEvent->notify_tx.conn_handle;
        event.CHIPoBLEConnectionError.Reason = BLE_ERROR_CHIPOBLE_PROTOCOL_ABORT;
        PlatformMgr().PostEvent(&event);
    }

    return CHIP_NO_ERROR;
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

CHIP_ERROR BLEManagerImpl::HandleGAPConnect(uint16_t conn_id)
{
printf("BLEManagerImpl::HandleGAPConnect xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx TB Verified \r\n");
    CHIP_ERROR err = CHIP_NO_ERROR;
    //ChipLogProgress(DeviceLayer, "BLE GAP connection established (con %" PRIu16 ")", conn_id);

    // Track the number of active GAP connections.
    mNumGAPCons++;
    err = SetSubscribed(conn_id);
    printf("HandleGapConnect Error: %d\r\n", err);
    VerifyOrExit(err != CHIP_ERROR_NO_MEMORY, err = CHIP_NO_ERROR);
    SuccessOrExit(err);

    mFlags.Set(Flags::kAdvertisingRefreshNeeded);
    mFlags.Clear(Flags::kAdvertisingConfigured);

exit:
    return err;
}

CHIP_ERROR BLEManagerImpl::HandleGAPDisconnect(uint16_t conn_id)
{
printf("BLEManagerImpl::HandleGAPDisconnect xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx TB Verified \r\n");
    //ChipLogProgress(DeviceLayer, "BLE GAP connection terminated (con %" PRIu16 " reason 0x%02" PRIx8 ")",
                    //gapEvent->disconnect.conn.conn_handle, gapEvent->disconnect.reason);

    // Update the number of GAP connections.
    if (mNumGAPCons > 0)
    {
        mNumGAPCons--;
    }

    if (UnsetSubscribed(conn_id))
    {
	    /*
        CHIP_ERROR disconReason;
        switch (gapEvent->disconnect.reason)
        {
        case 0x13://BLE_ERR_REM_USER_CONN_TERM:
            disconReason = BLE_ERROR_REMOTE_DEVICE_DISCONNECTED;
            break;
        case 0x16://BLE_ERR_CONN_TERM_LOCAL:
            disconReason = BLE_ERROR_APP_CLOSED_CONNECTION;
            break;
        default:
            disconReason = BLE_ERROR_CHIPOBLE_PROTOCOL_ABORT;
            break;
        }
        HandleConnectionError(gapEvent->disconnect.conn.conn_handle, disconReason);
	*/
	printf("Error in HandleGAPDisconnect");
    }

    // Force a reconfiguration of advertising in case we switched to non-connectable mode when
    // the BLE connection was established.
    mFlags.Set(Flags::kAdvertisingRefreshNeeded);
    mFlags.Clear(Flags::kAdvertisingConfigured);

    return CHIP_NO_ERROR;
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
printf("BLEManagerImpl::_SetCHIPoBLEServiceMode----------------------------------------------OK \r\n");
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
printf("BLEManagerImpl::_SetAdvertisingMode ---------mode = %d------------------------------------- \r\n",mode);
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
    printf("BLEManagerImpl::GetMTU:::::::::::::::TBD\r\n");
    int mtu;
    mtu = ble_att_mtu_z2(conId);

    printf("MTU size: %d\r\n", mtu);
    /*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    [zl_dbg] Apply similar function of esp's 'ble_att_mtu'
    int ret = 0;
    ret = ble_att_mtu(conId)
    !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
    return mtu;
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
    advData[index++] = 0x0A;                                                                // length
    advData[index++] = 0x16;                                     // AD type: (Service Data - 16-bit UUID)
    advData[index++] = static_cast<uint8_t>(ShortUUID_CHIPoBLEService.value & 0xFF);        // AD value
    advData[index++] = static_cast<uint8_t>((ShortUUID_CHIPoBLEService.value >> 8) & 0xFF); // AD value

    err = ConfigurationMgr().GetBLEDeviceIdentificationInfo(deviceIdInfo);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "GetBLEDeviceIdentificationInfo(): %s", ErrorStr(err));
        ExitNow();
    }

    VerifyOrExit(index + sizeof(deviceIdInfo) <= sizeof(advData), err = CHIP_ERROR_OUTBOUND_MESSAGE_TOO_BIG);
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
    le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(advData), (void *)advData);	//set advData
    le_register_app_cb(bt_config_app_gap_callback);

    bt_config_app_le_gap_init_chip();
    bt_config_task_init();

    bt_coex_init();

    // print advData
    for(int i=0; i<sizeof(advData); i++) {
	    printf("%X\n", advData[i]);
    }
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
    le_adv_stop();
    vTaskDelay(100);
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
printf("BLEManagerImpl::MapBLEError:::::::::::::::\r\n");
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

CHIP_ERROR BLEManagerImpl::SetSubscribed(uint16_t conId)
{
printf("SetSubscribed called\r\n");
//printf("BLEManagerImpl::HandleGAPDisconnect xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx TB Verified \r\n");
    uint16_t freeIndex = kMaxConnections;

    for (uint16_t i = 0; i < kMaxConnections; i++)
    {
        if (mSubscribedConIds[i] == conId)
        {
            return CHIP_NO_ERROR;
        }
        else if (mSubscribedConIds[i] == BLE_CONNECTION_UNINITIALIZED && i < freeIndex)
        {
            freeIndex = i;
        }
    }

    if (freeIndex < kMaxConnections)
    {
        mSubscribedConIds[freeIndex] = conId;
        return CHIP_NO_ERROR;
    }
    else
    {
        return CHIP_ERROR_NO_MEMORY;
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

int BLEManagerImpl::ble_svr_gap_msg_event(void *param, T_IO_MSG *p_gap_msg)
{
printf("BLEManagerImpl::ble_svr_gap_event xxxxxxxxxxevent->type=%dxxxxxxxxxxxxxxxxxxxx TB Verified \r\n", p_gap_msg->subtype);
    T_LE_GAP_MSG gap_msg;
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));
    CHIP_ERROR err = CHIP_NO_ERROR;
    uint16_t conn_id = gap_msg.msg_data.gap_conn_state_change.conn_id;
    uint16_t new_state = gap_msg.msg_data.gap_conn_state_change.new_state;
    uint16_t disc_cause = gap_msg.msg_data.gap_conn_state_change.disc_cause;

    printf("CASE: %d======================================\r\n", p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
    case GAP_MSG_LE_CONN_STATE_CHANGE:
        /* A new connection was established or a connection attempt failed */
        if(new_state==GAP_CONN_STATE_CONNECTED) {
                err = sInstance.HandleGAPConnect(conn_id);
                SuccessOrExit(err);
        }
        else if(new_state==GAP_CONN_STATE_DISCONNECTED) {
		printf("disc_cause: %d\r\n", disc_cause);
                err = sInstance.HandleGAPDisconnect(conn_id);
                SuccessOrExit(err);
        }
        break;

	/*
    case 1: //BLE_GAP_EVENT_DISCONNECT:
        err = sInstance.HandleGAPDisconnect(event);
        SuccessOrExit(err);
        break;
	*/

    case 9: //BLE_GAP_EVENT_ADV_COMPLETE:
        printf("BLE_GAP_EVENT_ADV_COMPLETE event\r\n");
        break;

    case 14://BLE_GAP_EVENT_SUBSCRIBE:
        //if (event->subscribe.attr_handle == sInstance.mTXCharCCCDAttrHandle)
        {
            //sInstance.HandleTXCharCCCDWrite(event);
	    printf("BLE_GAP_EVENT_SUBSCRIBE\r\n");
        }

        break;

    case 13://BLE_GAP_EVENT_NOTIFY_TX:
	printf("BLE_GAP_EVENT_NOTIFY_TX\r\n");
        //err = sInstance.HandleTXComplete(event);
        //SuccessOrExit(err);
        break;

    case GAP_MSG_LE_CONN_MTU_INFO: //BLE_GAP_EVENT_MTU:
        printf("BLE_GAP_EVENT_MTU \r\n");
        break;

    default:
        break;
    }

exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "Disabling CHIPoBLE service due to error: %s", ErrorStr(err));
        sInstance.mServiceMode = ConnectivityManager::kCHIPoBLEServiceMode_Disabled;
    }

    // Schedule DriveBLEState() to run.
    PlatformMgr().ScheduleWork(DriveBLEState, 0);

    return err;
}

int BLEManagerImpl::ble_svr_gap_event(void *param, int cb_type, void *p_cb_data)
{
printf("BLEManagerImpl::ble_svr_gap_event xxxxxxxxxxevent->type=%dxxxxxxxxxxxxxxxxxxxx TB Verified \r\n", cb_type);
    CHIP_ERROR err = CHIP_NO_ERROR;
	T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;
	switch (cb_type)
		{
#if defined(CONFIG_PLATFORM_8721D)
		case GAP_MSG_LE_DATA_LEN_CHANGE_INFO:
		//printf("GAP_MSG_LE_DATA_LEN_CHANGE_INFO: conn_id %d, tx octets 0x%x, max_tx_time 0x%x", p_data->p_le_data_len_change_info->conn_id, p_data->p_le_data_len_change_info->max_tx_octets,  p_data->p_le_data_len_change_info->max_tx_time);
			APP_PRINT_INFO3("GAP_MSG_LE_DATA_LEN_CHANGE_INFO: conn_id %d, tx octets 0x%x, max_tx_time 0x%x",
							p_data->p_le_data_len_change_info->conn_id,
							p_data->p_le_data_len_change_info->max_tx_octets,
							p_data->p_le_data_len_change_info->max_tx_time);
			break;
#endif
		case GAP_MSG_LE_MODIFY_WHITE_LIST:
			//APP_PRINT_INFO2("GAP_MSG_LE_MODIFY_WHITE_LIST: operation %d, cause 0x%x",
							//p_data->p_le_modify_white_list_rsp->operation,
							//p_data->p_le_modify_white_list_rsp->cause);
			break;
	
		default:
			//APP_PRINT_ERROR1("bt_config_app_gap_callback: unhandled cb_type 0x%x", cb_type);
			break;
		}


    return err;
}


//int BLEManagerImpl::gatt_svr_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt * ctxt, void * arg)
int BLEManagerImpl::gatt_svr_chr_access(void *param, T_SERVER_ID service_id, TBTCONFIG_CALLBACK_DATA *p_data)
{
    //struct ble_gatt_char_context param;
    int err = 0;
    //memset(&param, 0, sizeof(struct ble_gatt_char_context));

    if (service_id == SERVICE_PROFILE_GENERAL_ID)
    {
        T_SERVER_APP_CB_DATA *p_param = (T_SERVER_APP_CB_DATA *)p_data;
        switch (p_param->eventId)
        {
        case PROFILE_EVT_SRV_REG_COMPLETE:// srv register result event.
            //APP_PRINT_INFO1("PROFILE_EVT_SRV_REG_COMPLETE: result %d",
                            //p_param->event_data.service_reg_result);
            break;
        default:
            break;
        }
    } else {
	    uint8_t conn_id = p_data->conn_id;
	    T_SERVICE_CALLBACK_TYPE msg_type = p_data->msg_type;
	    uint8_t *p_value = p_data->msg_data.write.p_value;
	    uint16_t len = p_data->msg_data.write.len;
	    BLEManagerImpl *blemgr = static_cast<BLEManagerImpl *>(param);
	    switch (msg_type)
	    {
	    case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
	/*
	        param.conn_handle = conn_handle;
	        param.attr_handle = attr_handle;
	        param.ctxt        = ctxt;
	        param.arg         = arg;
	        sInstance.HandleTXCharRead(&param);
	    */
	        break;

	 

	    /*
	    case BLE_GATT_ACCESS_OP_READ_DSC:

	 

	        param.conn_handle = conn_handle;
	        param.attr_handle = attr_handle;
	        param.ctxt        = ctxt;
	        param.arg         = arg;
	        sInstance.HandleTXCharCCCDRead(&param);
	        break;
	    */

	 

	    case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
	        //param.conn_handle = conn_handle;
	        //param.attr_handle = attr_handle;
	        //param.ctxt        = ctxt;
	        //param.arg         = arg;
	        sInstance.HandleRXCharWrite(p_value, len, conn_id);
	        break;

		case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
			{
				printf("SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION\r\n");
				TSIMP_CALLBACK_DATA *pp_data;
				pp_data = (TSIMP_CALLBACK_DATA *)p_data;
				switch (pp_data->msg_data.notification_indification_index)
				{
				case SIMP_NOTIFY_INDICATE_V3_ENABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V3_ENABLE");
					}
					break;
		
				case SIMP_NOTIFY_INDICATE_V3_DISABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V3_DISABLE");
					}
					break;
				case SIMP_NOTIFY_INDICATE_V4_ENABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V4_ENABLE");
					}
					break;
				case SIMP_NOTIFY_INDICATE_V4_DISABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V4_DISABLE");
					}
					break;
				case SIMP_NOTIFY_INDICATE_V8_NOTIFY_ENABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V8_NOTIFY_ENABLE");
					}
					break;
				case SIMP_NOTIFY_INDICATE_V8_INDICATE_ENABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V8_INDICATE_ENABLE");
					}
					break;
				case SIMP_NOTIFY_INDICATE_V8_NOTIFY_INDICATE_ENABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V8_NOTIFY_INDICATE_ENABLE");
					}
					break;
				case SIMP_NOTIFY_INDICATE_V8_DISABLE:
					{
						printf("SIMP_NOTIFY_INDICATE_V8_DISABLE");
					}
					break;
				}
			}
			break;


	    default:
	        //err = BLE_ATT_ERR_UNLIKELY;
	        break;
	    }
    }
    PlatformMgr().ScheduleWork(DriveBLEState, 0);

    return err;
}

void BLEManagerImpl::HandleRXCharWrite(uint8_t *p_value, uint16_t len, uint8_t conn_id)
{
    CHIP_ERROR err    = CHIP_NO_ERROR;
    //uint16_t data_len = 0;

    // Copy the data to a packet buffer.
    //data_len               = OS_MBUF_PKTLEN(param->ctxt->om);
    PacketBufferHandle buf = System::PacketBufferHandle::New(len, 0);
    //VerifyOrExit(!buf.IsNull(), err = CHIP_ERROR_NO_MEMORY);
    //VerifyOrExit(buf->AvailableDataLength() >= data_len, err = CHIP_ERROR_BUFFER_TOO_SMALL);
    //ble_hs_mbuf_to_flat(param->ctxt->om, buf->Start(), data_len, NULL);
    memcpy(buf->Start(), p_value, len);
    buf->SetDataLength(len);

    // Post an event to the Chip queue to deliver the data into the Chip stack.
    {
        ChipDeviceEvent event;
        event.Type                        = DeviceEventType::kCHIPoBLEWriteReceived;
        event.CHIPoBLEWriteReceived.ConId = (uint16_t) conn_id;
        event.CHIPoBLEWriteReceived.Data  = std::move(buf).UnsafeRelease();
        PlatformMgr().PostEvent(&event);
    }

//exit:
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "HandleRXCharWrite() failed: %s", ErrorStr(err));
    }
}

int BLEManagerImpl::ble_callback_dispatcher(void *param, void *p_cb_data, int type, T_CHIP_BLEMGR_CALLBACK_TYPE callback_type)
{
    BLEManagerImpl *blemgr = static_cast<BLEManagerImpl *>(param);
    switch (callback_type)
    {
    case CB_PROFILE_CALLBACK:
		blemgr->gatt_svr_chr_access(param, type, (TBTCONFIG_CALLBACK_DATA *)p_cb_data);
        break;
    case CB_GAP_CALLBACK:
        blemgr->ble_svr_gap_event(param, type, p_cb_data);
        break;
    case CB_GAP_MSG_CALLBACK:
        blemgr->ble_svr_gap_msg_event(param, (T_IO_MSG*) p_cb_data);
        break;
    default:
        printf("%s %d error\r\n", __func__, __LINE__);
        break;
    }
    return 0;
}

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
#endif // CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE
