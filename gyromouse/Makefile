PROGRAM=gyromouse
EXTRA_COMPONENTS = extras/i2c extras/rboot-ota ../../gyromouse/components/RF24
PROGRAM_SRC_DIR=src src/tasks src/tasks/device src/device src/device/drivers
# ESPPORT=/dev/ttyUSB0
# ESPPORT=/dev/ttyS0
# ESPTOOL=esptool.py
# ESPTOOL=esptool

include ${ESP_OPEN_RTOS_PATH}/common.mk

# esptool -p COM19 --baud 115200 write_flash -fs 16MB -fm qio -ff 40m 0x0000 ..\..\platform\esp-open-rtos\bootloader\firmware_prebuilt\rboot.bin 0x1000 ..\..\platform\esp-open-rtos\bootloader\firmware_prebuilt\blank_config.bin 0x2000 .\gyromouse.bin
# esptool -p COM21 --baud 115200 write_flash 0x2000 .\gyromouse.bin
