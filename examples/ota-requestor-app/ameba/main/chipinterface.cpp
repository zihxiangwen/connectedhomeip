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

#include "CHIPDeviceManager.h"
#include "DeviceCallbacks.h"

#include "chip_porting.h"
#include <lwip_netconf.h>

#include <app/clusters/identify-server/identify-server.h>
#include <app/server/OnboardingCodesUtil.h>
#include <app/server/Server.h>
#include <credentials/DeviceAttestationCredsProvider.h>
#include <credentials/examples/DeviceAttestationCredsExample.h>
#include <lib/support/ErrorStr.h>
#include <platform/Ameba/AmebaConfig.h>
#include <platform/CHIPDeviceLayer.h>
#include <setup_payload/ManualSetupPayloadGenerator.h>
#include <setup_payload/QRCodeSetupPayloadGenerator.h>
#include <support/CHIPMem.h>

#include "OTARequesterImpl.h"

extern "C" {
void * __dso_handle = 0;
}

using namespace ::chip;
using namespace ::chip::System;
using namespace ::chip::Credentials;
using namespace ::chip::DeviceManager;
using namespace ::chip::DeviceLayer;

static DeviceCallbacks EchoCallbacks;
//CmdArgs queryImageCmdArgs, applyUpdateCmdArgs;

char gipAddress[20];
uint32_t gnodeId;

void QueryImageTimerHandler(Layer * systemLayer, void * appState)
{
    ChipLogProgress(DeviceLayer, "Calling SendQueryImageCommand(), ipAddress = %s, nodeId = %lu",gipAddress, gnodeId);
    OTARequesterImpl::GetInstance().SendQueryImageCommand(gipAddress, gnodeId);
}

void ApplyUpdateTimerHandler(Layer * systemLayer, void * appState)
{
    ChipLogProgress(DeviceLayer, "Calling SendApplyUpdateRequestCommand(),ipAddress = %s, nodeId = %lu",gipAddress, gnodeId);
    OTARequesterImpl::GetInstance().SendQueryImageCommand(gipAddress, gnodeId);
}

extern "C" void amebaQueryImageCmdHandler(char * ipAddress, uint32_t nodeId)
{
    ChipLogProgress(DeviceLayer, "Calling amebaQueryImageCmdHandler");
    strcpy(gipAddress, ipAddress);
    gnodeId = nodeId;

    /* Start one shot timer with 1 second timeout to send ApplyUpdateRequest command */
    chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(1 * 1000), QueryImageTimerHandler, nullptr);
}

extern "C" void amebaApplyUpdateCmdHandler(char * ipAddress, uint32_t nodeId)
{
    ChipLogProgress(DeviceLayer, "Calling amebaApplyUpdateCmdHandler");
    strcpy(gipAddress, ipAddress);
    gnodeId = nodeId;
    /* Start one shot timer with 1 second timeout to Query for OTA image */
    chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(1 * 1000), ApplyUpdateTimerHandler, nullptr);
}

extern "C" void ChipTest(void)
{
    ChipLogProgress(DeviceLayer, "OTA Requestor Demo!");
    CHIP_ERROR err = CHIP_NO_ERROR;

    initPref();

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

    while (true)
        vTaskDelay(pdMS_TO_TICKS(50));
}

bool lowPowerClusterSleep()
{
    return true;
}
