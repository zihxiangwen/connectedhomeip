/*
 *
 *    Copyright (c) 2021 Project CHIP Authors
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
 *          Platform-specific key value storage implementation for AmebaD
 */
/* this file behaves like a config.h, comes first */
#include <platform/KeyValueStoreManager.h>
#include <support/CodeUtils.h>
#include "chip_porting.h"

namespace chip {
namespace DeviceLayer {
namespace PersistedStorage {

KeyValueStoreManagerImpl KeyValueStoreManagerImpl::sInstance;

CHIP_ERROR KeyValueStoreManagerImpl::_Get(const char * key, void * value, size_t value_size, size_t * read_bytes_size,
                                          size_t offset_bytes)
{
    CHIP_ERROR err = CHIP_NO_ERROR;
    int32_t ret = -1;
    char* _key = (char*) malloc(strlen(key) + 1);

    if (!value)
    {
        return(err = CHIP_ERROR_INVALID_ARGUMENT);
    }
    if (offset_bytes > 0)
    {
        // Offset and partial reads are not supported in nvs, for now just return NOT_IMPLEMENTED. Support can be added in the
        // future if this is needed.
        return (err = CHIP_ERROR_NOT_IMPLEMENTED);
    }

    strcpy(_key, key);
    ret = getPref_bin_new("CHIP_KVS", _key, (uint8_t *) value, value_size, read_bytes_size);
    if(TRUE == ret)
    {
        err = CHIP_NO_ERROR;
        *read_bytes_size = value_size;
    }
    else
    {
        err = CHIP_ERROR_INTERNAL;
    }

    free(_key);
    return err;
}

CHIP_ERROR KeyValueStoreManagerImpl::_Put(const char * key, const void * value, size_t value_size)
{
    CHIP_ERROR err = CHIP_NO_ERROR;
    int32_t ret = -1;
    char* _key = (char*) malloc(strlen(key) + 1);

    if (!value)
    {
        return(err = CHIP_ERROR_INVALID_ARGUMENT);
    }

    strcpy(_key,key);
    ret = setPref_new("CHIP_KVS", _key, (uint8_t *)value, value_size);
    if(TRUE == ret)
        err = CHIP_NO_ERROR;
    else
        err = CHIP_ERROR_INTERNAL;

    free(_key);

    return err;
}


CHIP_ERROR KeyValueStoreManagerImpl::_Delete(const char * key)
{
    CHIP_ERROR err = CHIP_NO_ERROR;
    char* _key = (char*) malloc(strlen(key) + 1);

    strcpy(_key,key);
    registerPref("CHIP_KVS");
    if(TRUE == deleteKey("CHIP_KVS",_key))
        err = CHIP_NO_ERROR;
    else
        err = CHIP_ERROR_INTERNAL;

    free(_key);
    return err;
}

} // namespace PersistedStorage
} // namespace DeviceLayer
} // namespace chip
