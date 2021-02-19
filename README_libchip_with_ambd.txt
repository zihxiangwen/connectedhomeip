
1. git clone https://github.com/hank820/connectedhomeip.git
   cd connectedhomeip
   git submodule update --init

2. Follow the step to setup CHIP building environment
   https://github.com/hank820/connectedhomeip/blob/master/docs/BUILDING.md

3. Follow the step to compile all-cluster-app/esp32 example
   https://github.com/hank820/connectedhomeip/tree/master/examples/all-clusters-app/esp32

4. git checkout dev_base_gn

5. copy connectedhomeip\examples\all-clusters-app\esp32\build\chip\args.gn
   to   connectedhomeip\examples\all-clusters-app\ambd\build\chip\args.gn

6. Edit connectedhomeip\examples\all-clusters-app\ambd\build\chip\args.gn
    6.1 esp32_ar = "xtensa-esp32-elf-ar" to ambd_ar = "arm-none-eabi-ar"
    6.2 esp32_cc = "xtensa-esp32-elf-gcc" to ambd_cc = "arm-none-eabi-gcc"
    6.3 esp32_cxx = "xtensa-esp32-elf-c++" to ambd_cxx = "arm-none-eabi-c++"
    6.4 esp32_cpu = "esp32" to ambd_cpu = "esp32"
    6.5 modify -mlongcalls to -mlong-calls in target_cflags_c
	6.6 modify -mlongcalls to -mlong-calls in target_cflags_cc

7. export PATH=$PATH:(Add ambd toolchain path)
   EX: export PATH=$PATH:/home/test/disk64/chip/ambd/ambd_sdk_with_chip/project/realtek_amebaD_va0_example/GCC-RELEASE/project_hp/toolchain/linux/asdk-6.4.1/linux/newlib/bin

8. Generate ninja (need to change absolute path)
cd ~/disk64/chip/ambd/connectedhomeip && PW_ENVSETUP_QUIET=1 . scripts/activate.sh && cd ~/disk64/chip/ambd/connectedhomeip/config/ambd/components/chip && gn gen --check --fail-on-unused-args /home/test/disk64/chip/ambd/connectedhomeip/examples/all-clusters-app/ambd/build/chip

9. Build libCHIP.a (need to change absolute path)
cd ~/disk64/chip/ambd/connectedhomeip/config/ambd/components/chip ; ninja -C /home/test/disk64/chip/ambd/connectedhomeip/examples/all-clusters-app/ambd/build/chip

10. Output libCHIP.a in /home/test/disk64/chip/ambd/connectedhomeip/examples/all-clusters-app/ambd/build/chip/lib/

Next Action: 
             Porting src/platform/AMBD/*
             Change include path by connectedhomeip\examples\all-clusters-app\ambd\build\chip\args.gn target_cflags_c ,target_cflags_cc 

Note: 
             esp32 chip args.gn generate by connectedhomeip\config\esp32\components\chip\component.mk
 

