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

#pragma once

#if CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE
	#if 0
		#include "ble_conn_manager.h"
		#include "ble_controller_task_config.h"
		#include "ble_general.h"
		#include "ble_host_task_config.h"
		#include "ble_host_tasks.h"
		#include "controller_interface.h"
		#include "gap_interface.h"
		#include "gatt_db_dynamic.h"
		#include "gatt_server_interface.h"
	#endif
#include "FreeRTOS.h"
#include "event_groups.h"
#include "timers.h"
#include "bt_config_service.h"
#include "app_msg.h"
#include "bt_config_peripheral_app.h"
#include "gap_msg.h"

/**************** These structs are from ESP32 ******************/
/** @brief Passkey query */
struct ble_gap_passkey_params {
    /** Passkey action, can be one of following constants:
     *  - BLE_SM_IOACT_NONE
     *  - BLE_SM_IOACT_OOB
     *  - BLE_SM_IOACT_INPUT
     *  - BLE_SM_IOACT_DISP
     *  - BLE_SM_IOACT_NUMCMP
     */
    uint8_t action;

    /** Passkey to compare, valid for BLE_SM_IOACT_NUMCMP action */
    uint32_t numcmp;
};
struct ble_gap_repeat_pairing {
    /** The handle of the relevant connection. */
    uint16_t conn_handle;

    /** Properties of the existing bond. */
    uint8_t cur_key_size;
    uint8_t cur_authenticated:1;
    uint8_t cur_sc:1;

    /**
     * Properties of the imminent secure link if the pairing procedure is
     * allowed to continue.
     */
    uint8_t new_key_size;
    uint8_t new_authenticated:1;
    uint8_t new_sc:1;
    uint8_t new_bonding:1;
};
typedef struct {
    uint8_t type;
    uint8_t val[6];
} ble_addr_t;

/** Connection security state */
struct ble_gap_sec_state {
    /** If connection is encrypted */
    unsigned encrypted:1;

    /** If connection is authenticated */
    unsigned authenticated:1;

    /** If connection is bonded (security information is stored)  */
    unsigned bonded:1;

    /** Size of a key used for encryption */
    unsigned key_size:5;
};

struct ble_gap_conn_desc {
    /** Connection security state */
    struct ble_gap_sec_state sec_state;

    /** Local identity address */
    ble_addr_t our_id_addr;

    /** Peer identity address */
    ble_addr_t peer_id_addr;

    /** Local over-the-air address */
    ble_addr_t our_ota_addr;

    /** Peer over-the-air address */
    ble_addr_t peer_ota_addr;

    /** Connection handle */
    uint16_t conn_handle;

    /** Connection interval */
    uint16_t conn_itvl;

    /** Connection latency */
    uint16_t conn_latency;

    /** Connection supervision timeout */
    uint16_t supervision_timeout;

    /** Connection Role
     * Possible values BLE_GAP_ROLE_SLAVE or BLE_GAP_ROLE_MASTER
     */
    uint8_t role;

    /** Master clock accuracy */
    uint8_t master_clock_accuracy;
};

/** @brief Extended advertising report */
struct ble_gap_ext_disc_desc {
    /** Report properties bitmask
     *  - BLE_HCI_ADV_CONN_MASK
     *  - BLE_HCI_ADV_SCAN_MASK
     *  - BLE_HCI_ADV_DIRECT_MASK
     *  - BLE_HCI_ADV_SCAN_RSP_MASK
     *  - BLE_HCI_ADV_LEGACY_MASK
     *  */
    uint8_t props;

    /** Advertising data status, can be one of following constants:
     *  - BLE_GAP_EXT_ADV_DATA_STATUS_COMPLETE
     *  - BLE_GAP_EXT_ADV_DATA_STATUS_INCOMPLETE
     *  - BLE_GAP_EXT_ADV_DATA_STATUS_TRUNCATED
     */
    uint8_t data_status;

    /** Legacy advertising PDU type. Valid if BLE_HCI_ADV_LEGACY_MASK props is
     * set. Can be one of following constants:
     *  - BLE_HCI_ADV_RPT_EVTYPE_ADV_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_DIR_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP
     */
    uint8_t legacy_event_type;

    /** Advertiser address */
    ble_addr_t addr;

    /** Received signal strength indication in dBm (127 if unavailable) */
    int8_t rssi;

    /** Advertiser transmit power in dBm (127 if unavailable) */
    int8_t tx_power;

    /** Advertising Set ID */
    uint8_t sid;

    /** Primary advertising PHY, can be one of following constants:
     *  - BLE_HCI_LE_PHY_1M
     *  - BLE_HCI_LE_PHY_CODED
     */
    uint8_t prim_phy;

    /** Secondary advertising PHY, can be one of following constants:
     *  - BLE_HCI_LE_PHY_1M
     *  - LE_HCI_LE_PHY_2M
     *  - BLE_HCI_LE_PHY_CODED
     */
    uint8_t sec_phy;

    /** Periodic advertising interval. 0 if no periodic advertising. */
    uint16_t periodic_adv_itvl;

    /** Advertising Data length */
    uint8_t length_data;

    /** Advertising data */
    uint8_t *data;

    /** Directed advertising address.  Valid if BLE_HCI_ADV_DIRECT_MASK props is
     * set (BLE_ADDR_ANY otherwise).
     */
    ble_addr_t direct_addr;
};

/** @brief Advertising report */
struct ble_gap_disc_desc {
    /** Advertising PDU type. Can be one of following constants:
     *  - BLE_HCI_ADV_RPT_EVTYPE_ADV_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_DIR_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND
     *  - BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP
     */
    uint8_t event_type;

    /** Advertising Data length */
    uint8_t length_data;

    /** Advertiser address */
    ble_addr_t addr;

    /** Received signal strength indication in dBm (127 if unavailable) */
    int8_t rssi;

    /** Advertising data */
    uint8_t *data;

    /** Directed advertising address.  Valid for BLE_HCI_ADV_RPT_EVTYPE_DIR_IND
     * event type (BLE_ADDR_ANY otherwise).
     */
    ble_addr_t direct_addr;
};

struct ble_gap_event { 
    /**
     * Indicates the type of GAP event that occurred.  This is one of the
     * BLE_GAP_EVENT codes.
     */
    uint8_t type;

    /**
     * A discriminated union containing additional details concerning the GAP
     * event.  The 'type' field indicates which member of the union is valid.
     */
    union {
        /**
         * Represents a connection attempt.  Valid for the following event
         * types:
         *     o BLE_GAP_EVENT_CONNECT
         */
        struct {
            /**
             * The status of the connection attempt;
             *     o 0: the connection was successfully established.
             *     o BLE host error code: the connection attempt failed for
             *       the specified reason.
             */
            int status;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } connect;

        /**
         * Represents a terminated connection.  Valid for the following event
         * types:
         *     o BLE_GAP_EVENT_DISCONNECT
         */
        struct {
            /**
             * A BLE host return code indicating the reason for the
             * disconnect.
             */
            int reason;

            /** Information about the connection prior to termination. */
            struct ble_gap_conn_desc conn;
        } disconnect;

        /**
         * Represents an advertising report received during a discovery
         * procedure.  Valid for the following event types:
         *     o BLE_GAP_EVENT_DISC
         */
        struct ble_gap_disc_desc disc;

//#if MYNEWT_VAL(BLE_EXT_ADV)
        /**
         * Represents an extended advertising report received during a discovery
         * procedure.  Valid for the following event types:
         *     o BLE_GAP_EVENT_EXT_DISC
         */
        struct ble_gap_ext_disc_desc ext_disc;
//#endif

        /**
         * Represents a completed discovery procedure.  Valid for the following
         * event types:
         *     o BLE_GAP_EVENT_DISC_COMPLETE
         */
        struct {
            /**
             * The reason the discovery procedure stopped.  Typical reason
             * codes are:
             *     o 0: Duration expired.
             *     o BLE_HS_EPREEMPTED: Host aborted procedure to configure a
             *       peer's identity.
             */
            int reason;
        } disc_complete;

        /**
         * Represents a completed advertise procedure.  Valid for the following
         * event types:
         *     o BLE_GAP_EVENT_ADV_COMPLETE
         */
        struct {
            /**
             * The reason the advertise procedure stopped.  Typical reason
             * codes are:
             *     o 0: Terminated due to connection.
             *     o BLE_HS_ETIMEOUT: Duration expired.
             *     o BLE_HS_EPREEMPTED: Host aborted procedure to configure a
             *       peer's identity.
             */
            int reason;

//#if MYNEWT_VAL(BLE_EXT_ADV)
            /** Advertising instance */
            uint8_t instance;
            /** The handle of the relevant connection - valid if reason=0 */
            uint16_t conn_handle;
            /**
             * Number of completed extended advertising events
             *
             * This field is only valid if non-zero max_events was passed to
             * ble_gap_ext_adv_start() and advertising completed due to duration
             * timeout or max events transmitted.
             * */
            uint8_t num_ext_adv_events;
//#endif
        } adv_complete;

        /**
         * Represents an attempt to update a connection's parameters.  If the
         * attempt was successful, the connection's descriptor reflects the
         * updated parameters.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_CONN_UPDATE
         */
        struct {
            /**
             * The result of the connection update attempt;
             *     o 0: the connection was successfully updated.
             *     o BLE host error code: the connection update attempt failed
             *       for the specified reason.
             */
            int status;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } conn_update;

        /**
         * Represents a peer's request to update the connection parameters.
         * This event is generated when a peer performs any of the following
         * procedures:
         *     o L2CAP Connection Parameter Update Procedure
         *     o Link-Layer Connection Parameters Request Procedure
         *
         * To reject the request, return a non-zero HCI error code.  The value
         * returned is the reject reason given to the controller.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_L2CAP_UPDATE_REQ
         *     o BLE_GAP_EVENT_CONN_UPDATE_REQ
         */
        struct {
            /**
             * Indicates the connection parameters that the peer would like to
             * use.
             */
            const struct ble_gap_upd_params *peer_params;

            /**
             * Indicates the connection parameters that the local device would
             * like to use.  The application callback should fill this in.  By
             * default, this struct contains the requested parameters (i.e.,
             * it is a copy of 'peer_params').
             */
            struct ble_gap_upd_params *self_params;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } conn_update_req;

        /**
         * Represents a failed attempt to terminate an established connection.
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_TERM_FAILURE
         */
        struct {
            /**
             * A BLE host return code indicating the reason for the failure.
             */
            int status;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } term_failure;

        /**
         * Represents an attempt to change the encrypted state of a
         * connection.  If the attempt was successful, the connection
         * descriptor reflects the updated encrypted state.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_ENC_CHANGE
         */
        struct {
            /**
             * Indicates the result of the encryption state change attempt;
             *     o 0: the encrypted state was successfully updated;
             *     o BLE host error code: the encryption state change attempt
             *       failed for the specified reason.
             */
            int status;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } enc_change;

        /**
         * Represents a passkey query needed to complete a pairing procedure.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_PASSKEY_ACTION
         */
        struct {
            /** Contains details about the passkey query. */
            struct ble_gap_passkey_params params;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } passkey;

        /**
         * Represents a received ATT notification or indication.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_NOTIFY_RX
         */
        struct {
            /**
             * The contents of the notification or indication.  If the
             * application wishes to retain this mbuf for later use, it must
             * set this pointer to NULL to prevent the stack from freeing it.
             */
            struct os_mbuf *om;

            /** The handle of the relevant ATT attribute. */
            uint16_t attr_handle;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;

            /**
             * Whether the received command is a notification or an
             * indication;
             *     o 0: Notification;
             *     o 1: Indication.
             */
            uint8_t indication:1;
        } notify_rx;

        /**
         * Represents a transmitted ATT notification or indication, or a
         * completed indication transaction.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_NOTIFY_TX
         */
        struct {
            /**
             * The status of the notification or indication transaction;
             *     o 0:                 Command successfully sent;
             *     o BLE_HS_EDONE:      Confirmation (indication ack) received;
             *     o BLE_HS_ETIMEOUT:   Confirmation (indication ack) never
             *                              received;
             *     o Other return code: Error.
             */
            int status;

            /** The handle of the relevant connection. */
            uint16_t conn_handle;

            /** The handle of the relevant characteristic value. */
            uint16_t attr_handle;

            /**
             * Whether the transmitted command is a notification or an
             * indication;
             *     o 0: Notification;
             *     o 1: Indication.
             */
            uint8_t indication:1;
        } notify_tx;

        /**
         * Represents a state change in a peer's subscription status.  In this
         * comment, the term "update" is used to refer to either a notification
         * or an indication.  This event is triggered by any of the following
         * occurrences:
         *     o Peer enables or disables updates via a CCCD write.
         *     o Connection is about to be terminated and the peer is
         *       subscribed to updates.
         *     o Peer is now subscribed to updates after its state was restored
         *       from persistence.  This happens when bonding is restored.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_SUBSCRIBE
         */
        struct {
            /** The handle of the relevant connection. */
            uint16_t conn_handle;

            /** The value handle of the relevant characteristic. */
            uint16_t attr_handle;

            /** One of the BLE_GAP_SUBSCRIBE_REASON codes. */
            uint8_t reason;

            /** Whether the peer was previously subscribed to notifications. */
            uint8_t prev_notify:1;

            /** Whether the peer is currently subscribed to notifications. */
            uint8_t cur_notify:1;

            /** Whether the peer was previously subscribed to indications. */
            uint8_t prev_indicate:1;

            /** Whether the peer is currently subscribed to indications. */
            uint8_t cur_indicate:1;
        } subscribe;

        /**
         * Represents a change in an L2CAP channel's MTU.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_MTU
         */
        struct {
            /** The handle of the relevant connection. */
            uint16_t conn_handle;

            /**
             * Indicates the channel whose MTU has been updated; either
             * BLE_L2CAP_CID_ATT or the ID of a connection-oriented channel.
             */
            uint16_t channel_id;

            /* The channel's new MTU. */
            uint16_t value;
        } mtu;

        /**
         * Represents a change in peer's identity. This is issued after
         * successful pairing when Identity Address Information was received.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_IDENTITY_RESOLVED
         */
        struct {
            /** The handle of the relevant connection. */
            uint16_t conn_handle;
        } identity_resolved;

        /**
         * Represents a peer's attempt to pair despite a bond already existing.
         * The application has two options for handling this event type:
         *     o Retry: Return BLE_GAP_REPEAT_PAIRING_RETRY after deleting the
         *              conflicting bond.  The stack will verify the bond has
         *              been deleted and continue the pairing procedure.  If
         *              the bond is still present, this event will be reported
         *              again.
         *     o Ignore: Return BLE_GAP_REPEAT_PAIRING_IGNORE.  The stack will
         *               silently ignore the pairing request.
         *
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_REPEAT_PAIRING
         */
        struct ble_gap_repeat_pairing repeat_pairing;

        /**
         * Represents a change of PHY. This is issue after successful
         * change on PHY.
         */
        struct {
            int status;
            uint16_t conn_handle;

            /**
             * Indicates enabled TX/RX PHY. Possible values:
             *     o BLE_GAP_LE_PHY_1M
             *     o BLE_GAP_LE_PHY_2M
             *     o BLE_GAP_LE_PHY_CODED
             */
            uint8_t tx_phy;
            uint8_t rx_phy;
        } phy_updated;
//#if MYNEWT_VAL(BLE_PERIODIC_ADV)
        /**
         * Represents a periodic advertising sync established during discovery
         * procedure. Valid for the following event types:
         *     o BLE_GAP_EVENT_PERIODIC_SYNC
         */
        struct {
            /** BLE_ERR_SUCCESS on success or error code on failure. Other
             * fields are valid only for success
             */
            uint8_t status;
            /** Periodic sync handle */
            uint16_t sync_handle;

            /** Advertising Set ID */
            uint8_t sid;

            /** Advertiser address */
            ble_addr_t adv_addr;

            /** Advertising PHY, can be one of following constants:
             *  - BLE_HCI_LE_PHY_1M
             *  - LE_HCI_LE_PHY_2M
             *  - BLE_HCI_LE_PHY_CODED
            */
            uint8_t adv_phy;

            /** Periodic advertising interval */
            uint16_t per_adv_ival;

            /** Advertiser clock accuracy */
            uint8_t adv_clk_accuracy;
        } periodic_sync;

        /**
         * Represents a periodic advertising report received on established
         * sync. Valid for the following event types:
         *     o BLE_GAP_EVENT_PERIODIC_REPORT
         */
        struct {
            /** Periodic sync handle */
            uint16_t sync_handle;

            /** Advertiser transmit power in dBm (127 if unavailable) */
            int8_t tx_power;

            /** Received signal strength indication in dBm (127 if unavailable) */
            int8_t rssi;

            /** Advertising data status, can be one of following constants:
             *  - BLE_HCI_PERIODIC_DATA_STATUS_COMPLETE
             *  - BLE_HCI_PERIODIC_DATA_STATUS_INCOMPLETE
             *  - BLE_HCI_PERIODIC_DATA_STATUS_TRUNCATED
             */
            uint8_t data_status;

            /** Advertising Data length */
            uint8_t data_length;

            /** Advertising data */
            uint8_t *data;
        } periodic_report;

        /**
         * Represents a periodic advertising sync lost of established sync.
         * Sync lost reason can be BLE_HS_ETIMEOUT (sync timeout) or
         * BLE_HS_EDONE (sync terminated locally).
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_PERIODIC_SYNC_LOST
         */
        struct {
            /** Periodic sync handle */
            uint16_t sync_handle;

            /** Reason for sync lost, can be BLE_HS_ETIMEOUT for timeout or
             * BLE_HS_EDONE for locally terminated sync
             */
            int reason;
        } periodic_sync_lost;
//#endif

//#if MYNEWT_VAL(BLE_EXT_ADV)
        /**
         * Represents a scan request for an extended advertising instance where
         * scan request notifications were enabled.
         * Valid for the following event types:
         *     o BLE_GAP_EVENT_SCAN_REQ_RCVD
         */
        struct {
            /** Extended advertising instance */
            uint8_t instance;
            /** Address of scanner */
            ble_addr_t scan_addr;
        } scan_req_rcvd;
//#endif
    };
};

namespace chip {
namespace DeviceLayer {
namespace Internal {

using namespace chip::Ble;

/**
 * Concrete implementation of the BLEManager singleton object for the Ameba platforms.
 */
class BLEManagerImpl final : public BLEManager, private BleLayer, private BlePlatformDelegate, private BleApplicationDelegate
{
    // Allow the BLEManager interface class to delegate method calls to
    // the implementation methods provided by this class.
    friend BLEManager;

private:
    // ===== Members that implement the BLEManager internal interface.

    CHIP_ERROR _Init(void);
    CHIPoBLEServiceMode _GetCHIPoBLEServiceMode(void);
    CHIP_ERROR _SetCHIPoBLEServiceMode(CHIPoBLEServiceMode val);
    bool _IsAdvertisingEnabled(void);
    CHIP_ERROR _SetAdvertisingEnabled(bool val);
    bool _IsAdvertising(void);
    CHIP_ERROR _SetAdvertisingMode(BLEAdvertisingMode mode);
    CHIP_ERROR _GetDeviceName(char * buf, size_t bufSize);
    CHIP_ERROR _SetDeviceName(const char * deviceName);
    uint16_t _NumConnections(void);
    void _OnPlatformEvent(const ChipDeviceEvent * event);
    BleLayer * _GetBleLayer(void);

    // ===== Members that implement virtual methods on BlePlatformDelegate.

    bool SubscribeCharacteristic(BLE_CONNECTION_OBJECT conId, const Ble::ChipBleUUID * svcId,
                                 const Ble::ChipBleUUID * charId) override;
    bool UnsubscribeCharacteristic(BLE_CONNECTION_OBJECT conId, const Ble::ChipBleUUID * svcId,
                                   const Ble::ChipBleUUID * charId) override;
    bool CloseConnection(BLE_CONNECTION_OBJECT conId) override;
    uint16_t GetMTU(BLE_CONNECTION_OBJECT conId) const override;
    bool SendIndication(BLE_CONNECTION_OBJECT conId, const Ble::ChipBleUUID * svcId, const Ble::ChipBleUUID * charId,
                        System::PacketBufferHandle pBuf) override;
    bool SendWriteRequest(BLE_CONNECTION_OBJECT conId, const Ble::ChipBleUUID * svcId, const Ble::ChipBleUUID * charId,
                          System::PacketBufferHandle pBuf) override;
    bool SendReadRequest(BLE_CONNECTION_OBJECT conId, const Ble::ChipBleUUID * svcId, const Ble::ChipBleUUID * charId,
                         System::PacketBufferHandle pBuf) override;
    bool SendReadResponse(BLE_CONNECTION_OBJECT conId, BLE_READ_REQUEST_CONTEXT requestContext, const Ble::ChipBleUUID * svcId,
                          const Ble::ChipBleUUID * charId) override;

    // ===== Members that implement virtual methods on BleApplicationDelegate.

    void NotifyChipConnectionClosed(BLE_CONNECTION_OBJECT conId) override;

    // ===== Members for internal use by the following friends.

    friend BLEManager & BLEMgr(void);
    friend BLEManagerImpl & BLEMgrImpl(void);

    static BLEManagerImpl sInstance;

    // ===== Private members reserved for use by this class only.

    typedef enum {
	BC_DEV_DISABLED            = 0x0,
	BC_DEV_INIT                = 0x1,
	BC_DEV_IDLE                = 0x2,
	BC_DEV_BT_CONNECTED        = 0x3,
	BC_DEV_DEINIT              = 0x4,
    } BC_device_state_t;

    enum class Flags : uint8_t
    {
        kAdvertisingEnabled      = 0x0001,
        kFastAdvertisingEnabled  = 0x0002,
        kAdvertising             = 0x0004,
        kRestartAdvertising      = 0x0008,
        kAMEBABLEStackInitialized = 0x0010,
        kDeviceNameSet           = 0x0020,
        kAdvertisingRefreshNeeded = 0x030, /**< The advertising configuration/state in ESP BLE layer needs to be updated. */
        kAdvertisingConfigured    = 0x040, /**< CHIPoBLE advertising has been configured in the ESP BLE layer. */
    };
    BitFlags<BLEManagerImpl::Flags> mFlags;

    enum
    {
        kMaxConnections      = BLE_LAYER_NUM_BLE_ENDPOINTS,
        kMaxDeviceNameLength = 16,
        kUnusedIndex         = 0xFF,
    };

    struct CHIPoBLEConState
    {
        uint16_t mtu : 10;
        uint16_t allocated : 1;
        uint16_t subscribed : 1;
        uint16_t unused : 4;
        uint8_t connectionHandle;
        uint8_t bondingHandle;
    };
    CHIPoBLEConState mBleConnections[kMaxConnections];

    CHIPoBLEServiceMode mServiceMode;
    
    uint16_t mNumGAPCons;
    uint16_t mTXCharCCCDAttrHandle;
    uint16_t mSubscribedConIds[kMaxConnections];
    //uint8_t mAdvHandle;	//no use
    char mDeviceName[kMaxDeviceNameLength + 1];
    CHIP_ERROR MapBLEError(int bleErr);

    void DriveBLEState(void);
    //CHIP_ERROR ConfigureAdvertising(void);	//no use
    CHIP_ERROR StartAdvertising(void);
    CHIP_ERROR StopAdvertising(void);
    CHIP_ERROR ConfigureAdvertisingData(void);

    void HandleRXCharWrite(uint8_t *, uint16_t, uint8_t);
    void HandleTXCharRead(struct ble_gatt_char_context * param);
    void HandleTXCharCCCDRead(void * param);
    void HandleTXCharCCCDWrite(int, int, int);
    CHIP_ERROR HandleTXComplete(int);
    CHIP_ERROR HandleGAPConnect(uint16_t);
    CHIP_ERROR HandleGAPDisconnect(uint16_t, uint16_t);
    CHIP_ERROR SetSubscribed(uint16_t conId);
    bool UnsetSubscribed(uint16_t conId);
    bool IsSubscribed(uint16_t conId);
    
    bool RemoveConnection(uint8_t connectionHandle);
    void AddConnection(uint8_t connectionHandle);
    
    BLEManagerImpl::CHIPoBLEConState * GetConnectionState(uint8_t connectionHandle, bool allocate);
    static int ble_svr_gap_msg_event(void *param, T_IO_MSG *p_gap_msg);
    static int ble_svr_gap_event(void *param, int cb_type, void *p_cb_data);
    static int gatt_svr_chr_access(void *param, T_SERVER_ID service_id, TBTCONFIG_CALLBACK_DATA *p_data);
    static int ble_callback_dispatcher(void *param, void *p_cb_data, int type, T_CHIP_BLEMGR_CALLBACK_TYPE callback_type);
    static void DriveBLEState(intptr_t arg);
    static void BleAdvTimeoutHandler(TimerHandle_t xTimer);
    static void CancelBleAdvTimeoutTimer(void);
    static void StartBleAdvTimeoutTimer(uint32_t aTimeoutInMs);
};

/**
 * Returns a reference to the public interface of the BLEManager singleton object.
 *
 * Internal components should use this to access features of the BLEManager object
 * that are common to all platforms.
 */
inline BLEManager & BLEMgr(void)
{
    return BLEManagerImpl::sInstance;
}

/**
 * Returns the platform-specific implementation of the BLEManager singleton object.
 *
 * Internal components can use this to gain access to features of the BLEManager
 * that are specific to the platforms.
 */
inline BLEManagerImpl & BLEMgrImpl(void)
{
    return BLEManagerImpl::sInstance;
}

inline BleLayer * BLEManagerImpl::_GetBleLayer()
{
    return this;
}

inline BLEManager::CHIPoBLEServiceMode BLEManagerImpl::_GetCHIPoBLEServiceMode(void)
{
    return mServiceMode;
}

inline bool BLEManagerImpl::_IsAdvertisingEnabled(void)
{
    return mFlags.Has(Flags::kAdvertisingEnabled);
}

inline bool BLEManagerImpl::_IsAdvertising(void)
{
    return mFlags.Has(Flags::kAdvertising);
}

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip

#endif // CHIP_DEVICE_CONFIG_ENABLE_CHIPOBLE
