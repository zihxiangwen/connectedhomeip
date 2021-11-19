/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
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

#include <platform_stdlib.h>

#ifdef CONFIG_PLATFORM_8721D
// TODO need to fix compile in hal layer
//#include "flash_api.h"
#else
#include "flash_api.h"
#endif
#include <device_lock.h>

#include "CHIPDeviceManager.h"
#include "DeviceCallbacks.h"

#include "chip_porting.h"
#include <lwip_netconf.h>

#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <lib/support/ErrorStr.h>
#include <platform/Ameba/AmebaConfig.h>
#include <platform/CHIPDeviceLayer.h>
#include <support/CHIPMem.h>

#include <BdxOtaSender.h>
#include <app/clusters/ota-provider/ota-provider.h>
#include <ota-provider-common/OTAProviderExample.h>

extern "C" {
void * __dso_handle = 0;
}

#ifdef CONFIG_PLATFORM_8721D
typedef struct {
    uint32_t FLASH_Id;               /*!< Specifies the flash vendor ID.
                                This parameter can be a value of @ref FLASH_VENDOR_ID_definitions */
    uint8_t FLASH_cur_bitmode;       /*!< Specifies the current bitmode of SPIC.
                                This parameter can be a value of @ref FLASH_BIT_Mode_definitions */
    uint8_t FLASH_baud_rate;         /*!< Specifies the spi_sclk divider value. The frequency of spi_sclk is derived from:
                                Frequency of spi_sclk = Frequency of oc_clk / (2 * FLASH_baud_rate) */
    uint8_t FLASH_baud_boot;         /*!< Specifies the spi_sclk divider value for rom boot. The frequency of spi_sclk is derived from:
                                Frequency of spi_sclk = Frequency of oc_clk / (2 * FLASH_baud_rate) */
    uint32_t FLASH_cur_cmd;          /*!< Specifies the current read cmd which is used to read data from flash
                                in current bitmode. */

    /* status bits define */
    uint32_t FLASH_QuadEn_bit;       /*!< Specifies the QE bit in status register which is used to enable Quad I/O mode . */
    uint32_t FLASH_Busy_bit;         /*!< Specifies the WIP(Write in Progress) bit in status register which indicates whether
                                the device is busy in program/erase/write status register progress. */
    uint32_t FLASH_WLE_bit;          /*!< Specifies the WEL(Write Enable Latch) bit in status register which indicates whether
                                the device will accepts program/erase/write status register instructions*/
    uint32_t FLASH_Status2_exist;        /*!< Specifies whether this flash chip has Status Register2 or not.
                                This parameter can be 0/1. 0 means it doesn't have Status Register2, 1 means
                                it has Status Register2.*/

    /* calibration data */
    uint8_t FLASH_rd_sample_phase_cal;   /*!< Specifies the read sample phase obtained from calibration. this is cal sample phase get from high speed cal */
    uint8_t FLASH_rd_sample_phase;   /*!< Specifies the read sample phase obtained from calibration. this is current sample phase */
    uint8_t FLASH_rd_dummy_cyle[3];  /*!< Specifies the read dummy cycle of different bitmode according to
                                flash datasheet*/

    /* valid R/W command set */
    uint32_t FLASH_rd_dual_o;            /*!< Specifies dual data read cmd */
    uint32_t FLASH_rd_dual_io;           /*!< Specifies dual data/addr read cmd */
    uint32_t FLASH_rd_quad_o;        /*!< Specifies quad data read cmd */
    uint32_t FLASH_rd_quad_io;       /*!< Specifies quad data/addr read cmd */
    uint32_t FLASH_wr_dual_i;            /*!< Specifies dual data write cmd */
    uint32_t FLASH_wr_dual_ii;           /*!< Specifies dual data/addr write cmd */
    uint32_t FLASH_wr_quad_i;            /*!< Specifies quad data write cmd */
    uint32_t FLASH_wr_quad_ii;           /*!< Specifies quad data/addr write cmd */
    uint32_t FALSH_dual_valid_cmd;       /*!< Specifies valid cmd of dual bitmode to program/read flash in auto mode */
    uint32_t FALSH_quad_valid_cmd;   /*!< Specifies valid cmd of quad bitmode to program/read flash in auto mode */

    /* other command set */
    uint8_t FLASH_cmd_wr_en;         /*!< Specifies the Write Enable(WREN) instruction which is for setting WEL bit*/
    uint8_t FLASH_cmd_rd_id;         /*!< Specifies the Read ID instruction which is for getting the identity of the flash divice.*/
    uint8_t FLASH_cmd_rd_status;     /*!< Specifies the Read Status Register instruction which is for getting the status of flash */
    uint8_t FLASH_cmd_rd_status2;        /*!< Specifies the Read Status Register2 instruction which is for getting the status2 of flash */
    uint8_t FLASH_cmd_wr_status;     /*!< Specifies the Write Status Register instruction which is for setting the status register of flash */
    uint8_t FLASH_cmd_wr_status2;        /*!< Specifies the Write Status Register2 instruction which is for setting the status register2 of flash.
                                 In some flash chips, status2 write cmd != status1 write cmd,
                                 like: GD25Q32C, GD25Q64C,GD25Q128C etc.*/
    uint8_t FLASH_cmd_chip_e;            /*!< Specifies the Erase Chip instruction which is for erasing the whole chip*/
    uint8_t FLASH_cmd_block_e;       /*!< Specifies the Erase Block instruction which is for erasing 64kB*/
    uint8_t FLASH_cmd_sector_e;      /*!< Specifies the Erase Sector instruction which is for erasing 4kB*/
    uint8_t FLASH_cmd_pwdn_release;  /*!< Specifies the Release from Deep Power Down instruction which is for exiting power down mode.*/
    uint8_t FLASH_cmd_pwdn;          /*!< Specifies the Deep Power Down instruction which is for entering power down mode.*/

    /* debug log */
    uint8_t debug;                   /*!< Specifies whether or not to print debug log.*/

    /* new calibration */
    uint8_t phase_shift_idx;         /*!< Specifies the phase shift idx in new calibration.*/

    uint8_t FLASH_addr_phase_len;    /*!< Specifies the number of bytes in address phase (between command phase and write/read phase).
                                This parameter can be 0/1/2/3. 0 means 4-byte address mode in SPI Flash.*/
    uint8_t FLASH_pseudo_prm_en;     /*!< Specifies whether SPIC enables SPIC performance read mode or not.*/
    uint8_t FLASH_pinmux;            /*!< Specifies which pinmux is used. PINMUX_S0 or PINMUX_S1*/

    uint32_t FLASH_rd_fast_single;   /*!< Specifies fast read cmd in auto mode.*/
} FLASH_InitTypeDef2;

struct flash_s2 {
    FLASH_InitTypeDef2 SpicInitPara;
};
typedef struct flash_s2 flash_t;
#endif

extern "C" int flash_stream_read(flash_t *obj, uint32_t address, uint32_t len, uint8_t * data);

using chip::BitFlags;
using chip::app::Clusters::OTAProviderDelegate;
using chip::bdx::TransferControlFlags;
using chip::Callback::Callback;
using chip::Messaging::ExchangeManager;
using namespace ::chip;
using namespace ::chip::System;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceManager;
using namespace ::chip::DeviceLayer;

CHIP_ERROR OnBlockQuery(void * context, chip::System::PacketBufferHandle & blockBuf, size_t & size, bool & isEof, uint32_t offset);
void OnTransferComplete(void * context);
void OnTransferFailed(void * context, BdxSenderErrorTypes status);

namespace {
static DeviceCallbacks EchoCallbacks;
BdxOtaSender bdxServer;

// TODO: this should probably be done dynamically
constexpr chip::EndpointId kOtaProviderEndpoint = 0;
constexpr uint32_t kMaxBdxBlockSize                 = 1024;
constexpr chip::System::Clock::Timeout kBdxTimeout  = chip::System::Clock::Seconds16(5 * 60); // Specification mandates >= 5 minutes
constexpr chip::System::Clock::Timeout kBdxPollFreq = chip::System::Clock::Milliseconds32(500);
const char * otaFilename                            = "hello-world.bin";
//const esp_partition_t * otaPartition                = nullptr;
uint32_t otaImageLen                 = 0;
uint32_t otaTransferInProgress       = false;

chip::Callback::Callback<OnBdxBlockQuery> onBlockQueryCallback(OnBlockQuery, nullptr);
chip::Callback::Callback<OnBdxTransferComplete> onTransferCompleteCallback(OnTransferComplete, nullptr);
chip::Callback::Callback<OnBdxTransferFailed> onTransferFailedCallback(OnTransferFailed, nullptr);
} // namespace

CHIP_ERROR OnBlockQuery(void * context, chip::System::PacketBufferHandle & blockBuf, size_t & size, bool & isEof, uint32_t offset)
{
    if (otaTransferInProgress == false)
    {
        if (otaImageLen == 0)
        {
            ChipLogProgress(DeviceLayer, "OTA partition not found");
            return CHIP_ERROR_OPEN_FAILED;
        }
        otaTransferInProgress = true;
    }

    uint16_t blockBufAvailableLength = blockBuf->AvailableDataLength();
    uint16_t transferBlockSize       = bdxServer.GetTransferBlockSize();

    size = (blockBufAvailableLength < transferBlockSize) ? blockBufAvailableLength : transferBlockSize;

    if (offset + size >= otaImageLen)
    {
        size  = otaImageLen - offset;
        isEof = true;
    }
    else
    {
        isEof = false;
    }

    flash_t     flash;
    //TODO: Use the OTA image header specified in the specification
    //      Right now use hardcode for testing
    uint32_t    imageStartAddr = 0x156000;
    device_mutex_lock(1);
    int err = flash_stream_read(&flash, imageStartAddr + offset, size, blockBuf->Start());
    device_mutex_unlock(1);

    if (err != TRUE)
    {
        ChipLogProgress(DeviceLayer, "Failed to read %d bytes from offset %d", size, imageStartAddr + offset);
        size  = 0;
        isEof = false;
        return CHIP_ERROR_READ_FAILED;
    }

    ChipLogProgress(DeviceLayer, "Read %d bytes from offset %d", size, imageStartAddr + offset);
    return CHIP_NO_ERROR;
}

void OnTransferComplete(void * context)
{
    ChipLogProgress(DeviceLayer, "OTA Image Transfer Complete");
    otaTransferInProgress = false;
}

void OnTransferFailed(void * context, BdxSenderErrorTypes status)
{
    ChipLogProgress(DeviceLayer, "OTA Image Transfer Failed, status:%x", status);
    otaTransferInProgress = false;
}

extern "C" void ChipTest(void)
{
    ChipLogProgress(DeviceLayer, "ota-provider-app Demo!");
    CHIP_ERROR err = CHIP_NO_ERROR;

    initPref();

    OTAProviderExample otaProvider;

    CHIPDeviceManager & deviceMgr = CHIPDeviceManager::GetInstance();

    err = deviceMgr.Init(&EchoCallbacks);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(DeviceLayer, "DeviceManagerInit() - ERROR!\r\n");
    }
    else
    {
        ChipLogProgress(DeviceLayer, "DeviceManagerInit() - OK\r\n");
    }

    chip::Server::GetInstance().Init();

    // Initialize device attestation config
    SetDeviceAttestationCredentialsProvider(Examples::GetExampleDACProvider());

    err = chip::Server::GetInstance().GetExchangeManager().RegisterUnsolicitedMessageHandlerForProtocol(chip::Protocols::BDX::Id,
                                                                                                          &bdxServer);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogProgress(DeviceLayer, "RegisterUnsolicitedMessageHandler failed: %s", chip::ErrorStr(err));
        return;
    }

    BdxOtaSenderCallbacks callbacks;
    callbacks.onBlockQuery       = &onBlockQueryCallback;
    callbacks.onTransferComplete = &onTransferCompleteCallback;
    callbacks.onTransferFailed   = &onTransferFailedCallback;
    bdxServer.SetCallbacks(callbacks);

    // If OTA image is available in flash storage then set to update available
#if 0
    otaPartition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "ota_data");
    if (otaPartition != nullptr)
    {
        ChipLogProgress(DeviceLayer, "Partition found %s address:0x%x size:0x%x", otaPartition->label, otaPartition->address, otaPartition->size);

        // TODO: Use the OTA image header specified in the specification
        //       Right now we are using just image length instead of full header
        esp_partition_read(otaPartition, 0, &otaImageLen, sizeof(otaImageLen));
        if (otaImageLen > otaPartition->size)
        {
            otaImageLen = 0;
        }
        ChipLogProgress(DeviceLayer, "OTA image length %d bytes", otaImageLen);
    }
    else
    {
        ChipLogProgress(DeviceLayer, "OTA partition not found");
    }
#else
    //TODO : Use the OTA image header specified in the specification
    //        Right now hardcode for testing
    otaImageLen = 172032;
#endif
    if (otaImageLen > 0)
    {
        otaProvider.SetQueryImageBehavior(OTAProviderExample::kRespondWithUpdateAvailable);
        otaProvider.SetOTAFilePath(otaFilename);
    }

    chip::app::Clusters::OTAProvider::SetDelegate(kOtaProviderEndpoint, &otaProvider);

    BitFlags<TransferControlFlags> bdxFlags;
    bdxFlags.Set(TransferControlFlags::kReceiverDrive);
    err = bdxServer.PrepareForTransfer(&chip::DeviceLayer::SystemLayer(), chip::bdx::TransferRole::kSender, bdxFlags,
                                         kMaxBdxBlockSize, kBdxTimeout, kBdxPollFreq);
    if (err != CHIP_NO_ERROR)
    {
        ChipLogError(BDX, "Failed to init BDX server: %s", chip::ErrorStr(err));
        return;
    }

    while (true)
        vTaskDelay(pdMS_TO_TICKS(50));
}
