# VALVE FW

- Installation
```
git clone --recursive git@git.nanochip.vn:pas/pas-valve-fw.git 
cd pas-valve-fw
git submodule update --init --recursive
make menuconfig
make flash monitor -j8
```


esptool.py --chip esp32 --port "COM3" --baud 921600 --before "default_reset" --after "hard_reset" write_flash -z --flash_mode "dio" --flash_freq "40m" --flash_size detect 0x1000 bootloader.bin 0x10000 esp-touch-app.bin 0x8000 partitions.bin

## Notes

- using esp-idf branch `release/v3.2`
- backup using `CMake`
