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

/* this file behaves like a config.h, comes first */
#include <platform/internal/CHIPDeviceLayerInternal.h>
#include <platform/AMBD/AMBDConfig.h>
#include <core/CHIPEncoding.h>
#include <support/CHIPMem.h>
#include <support/CHIPMemString.h>
#include <support/CodeUtils.h>
#include <support/logging/CHIPLogging.h>
#include "chip_porting.h"

enum {
    kPrefsTypeBoolean = 1,
    kPrefsTypeInteger = 2,
    kPrefsTypeString = 3,
    kPrefsTypeBuffer = 4,
    kPrefsTypeBinary = 5
};

namespace chip {
namespace DeviceLayer {
namespace Internal {

// *** CAUTION ***: Changing the names or namespaces of these values will *break* existing devices.

// NVS namespaces used to store device configuration information.
const char AMBDConfig::kConfigNamespace_ChipFactory[]  = "chip-factory";
const char AMBDConfig::kConfigNamespace_ChipConfig[]   = "chip-config";
const char AMBDConfig::kConfigNamespace_ChipCounters[] = "chip-counters";

// Keys stored in the chip-factory namespace
const AMBDConfig::Key AMBDConfig::kConfigKey_SerialNum           = { kConfigNamespace_ChipFactory, "serial-num" };
const AMBDConfig::Key AMBDConfig::kConfigKey_MfrDeviceId         = { kConfigNamespace_ChipFactory, "device-id" };
const AMBDConfig::Key AMBDConfig::kConfigKey_MfrDeviceCert       = { kConfigNamespace_ChipFactory, "device-cert" };
const AMBDConfig::Key AMBDConfig::kConfigKey_MfrDeviceICACerts   = { kConfigNamespace_ChipFactory, "device-ca-certs" };
const AMBDConfig::Key AMBDConfig::kConfigKey_MfrDevicePrivateKey = { kConfigNamespace_ChipFactory, "device-key" };
const AMBDConfig::Key AMBDConfig::kConfigKey_ProductRevision     = { kConfigNamespace_ChipFactory, "product-rev" };
const AMBDConfig::Key AMBDConfig::kConfigKey_ManufacturingDate   = { kConfigNamespace_ChipFactory, "mfg-date" };
const AMBDConfig::Key AMBDConfig::kConfigKey_SetupPinCode        = { kConfigNamespace_ChipFactory, "pin-code" };
const AMBDConfig::Key AMBDConfig::kConfigKey_SetupDiscriminator  = { kConfigNamespace_ChipFactory, "discriminator" };

// Keys stored in the chip-config namespace
const AMBDConfig::Key AMBDConfig::kConfigKey_FabricId                    = { kConfigNamespace_ChipConfig, "fabric-id" };
const AMBDConfig::Key AMBDConfig::kConfigKey_ServiceConfig               = { kConfigNamespace_ChipConfig, "service-config" };
const AMBDConfig::Key AMBDConfig::kConfigKey_PairedAccountId             = { kConfigNamespace_ChipConfig, "account-id" };
const AMBDConfig::Key AMBDConfig::kConfigKey_ServiceId                   = { kConfigNamespace_ChipConfig, "service-id" };
const AMBDConfig::Key AMBDConfig::kConfigKey_GroupKeyIndex               = { kConfigNamespace_ChipConfig, "group-key-index" };
const AMBDConfig::Key AMBDConfig::kConfigKey_LastUsedEpochKeyId          = { kConfigNamespace_ChipConfig, "last-ek-id" };
const AMBDConfig::Key AMBDConfig::kConfigKey_FailSafeArmed               = { kConfigNamespace_ChipConfig, "fail-safe-armed" };
const AMBDConfig::Key AMBDConfig::kConfigKey_WiFiStationSecType          = { kConfigNamespace_ChipConfig, "sta-sec-type" };
const AMBDConfig::Key AMBDConfig::kConfigKey_OperationalDeviceId         = { kConfigNamespace_ChipConfig, "op-device-id" };
const AMBDConfig::Key AMBDConfig::kConfigKey_OperationalDeviceCert       = { kConfigNamespace_ChipConfig, "op-device-cert" };
const AMBDConfig::Key AMBDConfig::kConfigKey_OperationalDeviceICACerts   = { kConfigNamespace_ChipConfig, "op-device-ca-certs" };
const AMBDConfig::Key AMBDConfig::kConfigKey_OperationalDevicePrivateKey = { kConfigNamespace_ChipConfig, "op-device-key" };
const AMBDConfig::Key AMBDConfig::kConfigKey_RegulatoryLocation          = { kConfigNamespace_ChipConfig, "regulatory-location" };
const AMBDConfig::Key AMBDConfig::kConfigKey_CountryCode                 = { kConfigNamespace_ChipConfig, "country-code" };
const AMBDConfig::Key AMBDConfig::kConfigKey_Breadcrumb                  = { kConfigNamespace_ChipConfig, "breadcrumb" };

CHIP_ERROR AMBDConfig::ReadConfigValue(Key key, bool & val)
{
    uint32_t intVal;
    int32_t success=0;

    success = getPref_bool_new(key.Namespace, key.Name, &intVal);
    if (!success)
        printf("getPref_u32_new: %s/%s failed\n", key.Namespace, key.Name);

    val = (intVal != 0);

    if (success == 1)
        return CHIP_NO_ERROR;
    else
        return CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND;
}

CHIP_ERROR AMBDConfig::ReadConfigValue(Key key, uint32_t & val)
{
    int32_t success=0;

    success = getPref_u32_new(key.Namespace, key.Name, &val);
    if (!success)
        printf("getPref_u32_new: %s/%s failed\n", key.Namespace, key.Name);

    if (success == 1)
        return CHIP_NO_ERROR;
    else
        return CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND;
}

CHIP_ERROR AMBDConfig::ReadConfigValue(Key key, uint64_t & val)
{
    int32_t success=0;

    success = getPref_u64_new(key.Namespace, key.Name, &val);
    if (!success)
        printf("getPref_u32_new: %s/%s failed\n", key.Namespace, key.Name);

    if (success == 1)
        return CHIP_NO_ERROR;
    else
        return CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND;

}

CHIP_ERROR AMBDConfig::ReadConfigValueStr(Key key, char * buf, size_t bufSize, size_t & outLen)
{
    int32_t success=0;

    success = getPref_str_new(key.Namespace, key.Name, buf, bufSize, &outLen);
    if (!success)
        printf("getPref_str_new: %s/%s failed\n", key.Namespace, key.Name);

    if (success == 1)
    {
        return CHIP_NO_ERROR;
    }
    else
    {
        outLen = 0;
        return CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND;
    }
}

CHIP_ERROR AMBDConfig::ReadConfigValueBin(Key key, uint8_t * buf, size_t bufSize, size_t & outLen)
{
    int32_t success=0;

    success = getPref_bin_new(key.Namespace, key.Name, buf, bufSize, &outLen);
    if (!success)
        printf("getPref_bin_new: %s/%s failed\n", key.Namespace, key.Name);

    if (success == 1)
    {
        return CHIP_NO_ERROR;
    }
    else
    {
        outLen = 0;
        return CHIP_DEVICE_ERROR_CONFIG_NOT_FOUND;
    }

}

CHIP_ERROR AMBDConfig::WriteConfigValue(Key key, bool val)
{
    int32_t success;
    uint8_t value;

    if (val == 1)
        value = 1;
    else
        value = 0;
    success = setPref_new(key.Namespace, key.Name, &value, 1);
    if (!success)
        printf("setPref: %s/%s = %s failed\n", key.Namespace, key.Name, value ? "true" : "false");

    return CHIP_NO_ERROR;
}

CHIP_ERROR AMBDConfig::WriteConfigValue(Key key, uint32_t val)
{
    int32_t success;

    success = setPref_new(key.Namespace, key.Name, (uint8_t *)&val, sizeof(uint32_t));
    if (!success)
        printf("setPref: %s/%s = %d(0x%x) failed\n", key.Namespace, key.Name, val, val);

    return CHIP_NO_ERROR;
}

CHIP_ERROR AMBDConfig::WriteConfigValue(Key key, uint64_t val)
{
    int32_t success;

    success = setPref_new(key.Namespace, key.Name, (uint8_t *)&val, sizeof(uint64_t));
    if (!success)
        printf("setPref: %s/%s = %d(0x%x) failed\n", key.Namespace, key.Name, val, val);

    return CHIP_NO_ERROR;
}

CHIP_ERROR AMBDConfig::WriteConfigValueStr(Key key, const char * str)
{
    int32_t success;

    success = setPref_new(key.Namespace, key.Name, (uint8_t *)str, strlen(str) + 1);
    if (!success)
        printf("setPref: %s/%s = %s failed\n", key.Namespace, key.Name, str);
    return CHIP_NO_ERROR;
}

CHIP_ERROR AMBDConfig::WriteConfigValueStr(Key key, const char * str, size_t strLen)
{
    CHIP_ERROR err;
    chip::Platform::ScopedMemoryBuffer<char> strCopy;

    if (str != NULL)
    {
        strCopy.Calloc(strLen + 1);
        VerifyOrExit(strCopy, err = CHIP_ERROR_NO_MEMORY);
        strncpy(strCopy.Get(), str, strLen);
    }
    err = AMBDConfig::WriteConfigValueStr(key, strCopy.Get());
exit:
    return err;
}

CHIP_ERROR AMBDConfig::WriteConfigValueBin(Key key, const uint8_t * data, size_t dataLen)
{
    int32_t success;

    success = setPref_new(key.Namespace, key.Name, (uint8_t *)data, dataLen);
    if (!success)
        printf("setPref: %s/%s failed\n", key.Namespace, key.Name);

    return CHIP_NO_ERROR;
}

CHIP_ERROR AMBDConfig::ClearConfigValue(Key key)
{
    int32_t success;

    success = deleteKey(key.Namespace, key.Name);
    if (!success)
        printf("%s : %s/%s failed\n",__FUNCTION__, key.Namespace, key.Name);

    return CHIP_NO_ERROR;
}

bool AMBDConfig::ConfigValueExists(Key key)
{
    int32_t success;

    success = checkExist(key.Namespace, key.Name);

    return success;
}

CHIP_ERROR AMBDConfig::EnsureNamespace(const char * ns)
{
    int32_t success = -1;

    success = registerPref(ns);
    if (success != 0)
    {
        printf("dct_register_module failed\n");
    }

    return CHIP_NO_ERROR;
}

CHIP_ERROR AMBDConfig::ClearNamespace(const char * ns)
{
    int32_t success = -1;

    success = clearPref(ns);
    if (success != 0)
    {
        printf("ClearNamespace failed\n");
    }

    return CHIP_NO_ERROR;
}

void AMBDConfig::RunConfigUnitTest() {}

} // namespace Internal
} // namespace DeviceLayer
} // namespace chip
