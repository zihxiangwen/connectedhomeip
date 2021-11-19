# ota-provider-app (ameba)

### Build and flash OTA apps


```
# Build OTA Provider
$PWD/../ambd_sdk_with_chip_non_NDA/project/realtek_amebaD_va0_example/GCC-RELEASE/build.sh $PWD ninja $PWD/out otap

# Build OTA Requestor
$PWD/../ambd_sdk_with_chip_non_NDA/project/realtek_amebaD_va0_example/GCC-RELEASE/build.sh $PWD ninja $PWD/out otar
```

### Commission both apps

```
# Commission OTA Provider
$ ./chip-tool pairing ble-wifi 12344321 <ssid> <passphrase> 0 20202021 3840

# Commission OTA Requestor
$ ./chip-tool pairing ble-wifi 12342222 <ssid> <passphrase> 0 20202021 3840
```

## QueryImage from requestor console

```
#QueryImage 
ameba console> ATS%=<OtaProviderIpAddress>,<OtaProviderNodeId>
```

When the image download is complete device waits for an ApplyUpdate command, so
fire following command from Requestor app

```
#ApplyUpdateRequest
ameba console> ATS^=<OtaProviderIpAddress>,<OtaProviderNodeId>
```

After this Requestor will run hello-world application.
