# ota-requestor-app

A prototype application that demonstrates device OTA requester.

For now, this application rely on the [Linux OTA Providor app](../linux)

### Setup Linux OTA Provider

-   Build the linux OTA Provider

```
# Change to chip top level directory
$ cd ../../../
$ scripts/examples/gn_build_example.sh examples/ota-provider-app/linux out/debug chip_config_network_layer_ble=false
```

-   Build CHIP Tool

```
$ scripts/examples/gn_build_example.sh examples/chip-tool out/debug
```

-   In terminal 1 run linux ota provider. One can build the)
    or use one from
    [here](https://github.com/pankore/ambd_sdk_with_chip_non_NDA/blob/master/project/realtek_amebaD_va0_example/GCC-RELEASE/project_hp/asdk/image/km4_boot_all.bin)

```
$ ./out/debug/chip-ota-provider-app -f hello-world.bin
```

-   In terminal 2 run the CHIP tool to provision OTA provider

```
$ ./out/debug/chip-tool pairing 12344321 20202021
```

Now we have a OTA provider ready for use

### Building the Example Application

-   Build the Application

-   Flash application

-   Provision OTA Requester

```
./out/debug/chip-tool pairing ble-wifi 12342222 <ssid> <passphrase> 0 20202021 3840
```

-   After commissioning is successful, query for OTA image. Head over to ameba
    console and fire the following command. This command start the OTA image
    tranfer in 10 seconds.

```
ATS%=<OtaProviderIpAddress>,<OtaProviderNodeId>
```

-   Once transfer is complete it applies OTA and boots up from another
    partition.

## Features

-   Code for running a full BDX download exists in BDX
-   Sends QueryImage command using command line
-   Downloads a file over BDX served by an OTA Provider server

## Limitations

-   Do not support ApplyUpdateRequest command
-   Needs chip-tool to commission the OTA Provider device first because the Node
    ID and IP Address of the OTA Provider must be supplied to this reference
    application
-   Does not verify QueryImageResponse message contents or status
-   Does not support AnnounceOTAProvider command or OTA Requestor attributes
