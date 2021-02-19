
1. Setup environment which can build PASS with all-cluster-app/esp32
2. copy connectedhomeip\examples\all-clusters-app\esp32\build\chip\args.gn to connectedhomeip\examples\all-clusters-app\ambd\build\chip\args.gn
3. Edit connectedhomeip\examples\all-clusters-app\ambd\build\chip\args.gn
    esp32_ar = "xtensa-esp32-elf-ar" to ambd_ar = "arm-none-eabi-ar"
    esp32_cc = "xtensa-esp32-elf-gcc" to ambd_cc = "arm-none-eabi-gcc"
    esp32_cxx = "xtensa-esp32-elf-c++" to ambd_cxx = "arm-none-eabi-c++"
    esp32_cpu = "esp32" to ambd_cpu = "esp32"
	
4. Generate ninja (need to change absolute path)
cd ~/disk64/chip/ambd/connectedhomeip && PW_ENVSETUP_QUIET=1 . scripts/activate.sh && cd ~/disk64/chip/ambd/connectedhomeip/config/ambd/components/chip && gn gen --check --fail-on-unused-args /home/test/disk64/chip/ambd/connectedhomeip/examples/all-clusters-app/ambd/build/chip

5. Build libCHIP.a (need to change absolute path)
cd ~/disk64/chip/ambd/connectedhomeip/config/ambd/components/chip ; ninja -C /home/test/disk64/chip/ambd/connectedhomeip/examples/all-clusters-app/ambd/build/chip

