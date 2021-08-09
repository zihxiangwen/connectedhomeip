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

#include <platform/CHIPDeviceLayer.h>
#include <support/CodeUtils.h>
#include <support/logging/CHIPLogging.h>

#include "ServiceProvisioning.h"
#include <wifi_conf.h>

using namespace ::chip::DeviceLayer;

CHIP_ERROR SetWiFiStationProvisioning(const char * ssid, const char * key)
{
    printf("%s %d %s %sToDo\r\n", __func__,__LINE__, ssid, key);
    ConnectivityMgr().SetWiFiStationMode(ConnectivityManager::kWiFiStationMode_Disabled);

    rtw_wifi_setting_t wifiConfig;

    // Set the wifi configuration
    memset(&wifiConfig, 0, sizeof(wifiConfig));
    memcpy(wifiConfig.ssid, ssid, strlen(ssid) + 1);
    memcpy(wifiConfig.password, key, strlen(key) + 1);
    wifiConfig.mode = RTW_MODE_STA;

    // Configure the ESP WiFi interface.
    int err = CHIP_SetWiFiConfig(&wifiConfig);
    if (err != 0)
    {
        ChipLogError(DeviceLayer, "_SetWiFiConfig() failed: %d", err);
        return err;
    }

    ConnectivityMgr().SetWiFiStationMode(ConnectivityManager::kWiFiStationMode_Disabled);
    ConnectivityMgr().SetWiFiStationMode(ConnectivityManager::kWiFiStationMode_Enabled);

    return CHIP_NO_ERROR;
}

