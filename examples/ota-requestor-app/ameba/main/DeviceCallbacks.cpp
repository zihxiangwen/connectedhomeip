/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
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
 * @file DeviceCallbacks.cpp
 *
 * Implements all the callbacks to the application from the CHIP Stack
 *
 **/
#include "DeviceCallbacks.h"

#include <app-common/zap-generated/attribute-id.h>
#include <app-common/zap-generated/cluster-id.h>
#include <app/Command.h>
#include <app/server/Dnssd.h>
#include <support/CodeUtils.h>
#include <support/logging/CHIPLogging.h>
#include <support/logging/Constants.h>

static const char * TAG = "app-devicecallbacks";

using namespace ::chip;
using namespace ::chip::Inet;
using namespace ::chip::System;
using namespace ::chip::DeviceLayer;
using namespace ::chip::DeviceManager;
using namespace ::chip::Logging;

void DeviceCallbacks::DeviceEventCallback(const ChipDeviceEvent * event, intptr_t arg)
{
    switch (event->Type)
    {
    case DeviceEventType::kInternetConnectivityChange:
        OnInternetConnectivityChange(event);
        break;

    case DeviceEventType::kSessionEstablished:
        OnSessionEstablished(event);
        break;

    case DeviceEventType::kInterfaceIpAddressChanged:
        if ((event->InterfaceIpAddressChanged.Type == InterfaceIpChangeType::kIpV4_Assigned) ||
            (event->InterfaceIpAddressChanged.Type == InterfaceIpChangeType::kIpV6_Assigned))
        {
            // MDNS server restart on any ip assignment: if link local ipv6 is configured, that
            // will not trigger a 'internet connectivity change' as there is no internet
            // connectivity. MDNS still wants to refresh its listening interfaces to include the
            // newly selected address.
            chip::app::DnssdServer::Instance().StartServer();
        }
        break;
    }
}

void DeviceCallbacks::PostAttributeChangeCallback(EndpointId endpointId, ClusterId clusterId, AttributeId attributeId, uint8_t mask,
                                                  uint8_t type, uint16_t size, uint8_t * value)
{
    ChipLogProgress(DeviceLayer,
             "PostAttributeChangeCallback - Cluster ID: '" ChipLogFormatMEI
             "', EndPoint ID: '0x%02x', Attribute ID: '" ChipLogFormatMEI "'",
             ChipLogValueMEI(clusterId), endpointId, ChipLogValueMEI(attributeId));

    // TODO handle this callback in switch statement
    ChipLogProgress(DeviceLayer, "Unhandled cluster ID: %d", clusterId);

    ChipLogProgress(DeviceLayer, "Current free heap: %d\n", xPortGetFreeHeapSize());

}

void DeviceCallbacks::OnInternetConnectivityChange(const ChipDeviceEvent * event)
{
    if (event->InternetConnectivityChange.IPv4 == kConnectivity_Established)
    {
        ChipLogProgress(DeviceLayer, "Server ready at: %s:%d", event->InternetConnectivityChange.address, CHIP_PORT);
        chip::app::DnssdServer::Instance().StartServer();
    }
    else if (event->InternetConnectivityChange.IPv4 == kConnectivity_Lost)
    {
        ChipLogProgress(DeviceLayer, "Lost IPv4 connectivity...");
    }
    if (event->InternetConnectivityChange.IPv6 == kConnectivity_Established)
    {
        ChipLogProgress(DeviceLayer, "IPv6 Server ready...");
        chip::app::DnssdServer::Instance().StartServer();
    }
    else if (event->InternetConnectivityChange.IPv6 == kConnectivity_Lost)
    {
        ChipLogProgress(DeviceLayer, "Lost IPv6 connectivity...");
    }
}

void DeviceCallbacks::OnSessionEstablished(const ChipDeviceEvent * event)
{
    if (event->SessionEstablished.IsCommissioner)
    {
        ChipLogProgress(DeviceLayer, "Commissioner detected!");
    }
}
