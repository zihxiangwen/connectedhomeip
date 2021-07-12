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

/**
 *    @file
 *      This file implements the CHIP Device Interface that is used by
 *      applications to interact with the CHIP stack
 *
 */

#include <stdlib.h>

#include "CHIPDeviceManager.h"
#include <app/util/basic-types.h>
#include <support/CHIPMem.h>
#include <support/CodeUtils.h>
#include <support/ErrorStr.h>
#include "Globals.h"
#include "LEDWidget.h"
#include <app/common/gen/attribute-id.h>
#include <app/common/gen/attribute-type.h>
#include <app/common/gen/cluster-id.h>

using namespace ::chip;

namespace chip {

namespace DeviceManager {

using namespace ::chip::DeviceLayer;

CHIP_ERROR CHIPDeviceManager::Init(void)
{
    CHIP_ERROR err;

    err = PlatformMgr().InitChipStack();
    SuccessOrExit(err);

    err = Platform::MemoryInit();
    SuccessOrExit(err);

    // // Start a task to run the CHIP Device event loop.
    err = PlatformMgr().StartEventLoopTask();
    if (err != CHIP_NO_ERROR)
    {
        printf("StartEventLoopTask() - ERROR!\r\n");
    }
    else
    {
        printf("StartEventLoopTask() - OK\r\n");
    }

 exit:
     return err;
}
} // namespace DeviceManager
} // namespace chip

void emberAfPostAttributeChangeCallback(EndpointId endpointId, ClusterId clusterId, AttributeId attributeId, uint8_t mask,
                                        uint16_t manufacturerCode, uint8_t type, uint16_t size, uint8_t * value)
{
    ChipLogProgress(Zcl, "Cluster callback: %" PRIx32, clusterId);
    if (clusterId == ZCL_ON_OFF_CLUSTER_ID)
    {
        if (attributeId != ZCL_ON_OFF_ATTRIBUTE_ID)
        {
            ChipLogProgress(Zcl, "Unknown attribute ID: %" PRIx32, attributeId);
            return;
        }

        statusLED1.Set(*value);
    }
    else
    {
        ChipLogProgress(Zcl, "Unknown cluster ID: %" PRIx32, clusterId);
        return;
    }
}
