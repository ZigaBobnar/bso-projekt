#!/bin/bash

# Check if the script is being run as root
if [ "$EUID" -eq 0 ]; then
    echo "Please do not run as root."
    exit
fi

set -e

# For debugging of this script, print all commands
# set -x


INITIAL_DIR=$(pwd)

# If single argument is supplied, store it into variable
if [ $# -eq 1 ]; then
    PLATFORM_PATH=$1
else
    PLATFORM_PATH="platform"
fi
# Remove slash if platform path ends with it
PLATFORM_PATH=${PLATFORM_PATH%/}

ABS_PLATFORM_PATH="$PLATFORM_PATH"
if [[ $ABS_PLATFORM_PATH != /* ]]; then
    ABS_PLATFORM_PATH="$(pwd)/$PLATFORM_PATH"
fi

if [ ! -d "$ABS_PLATFORM_PATH" ]; then
    mkdir -p $ABS_PLATFORM_PATH
fi

cd $ABS_PLATFORM_PATH


##
# xtensa-lx106-elf toolchain
# https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/linux-setup.html
##

XTENSA_LX106_ELF_TOOLCHAIN_NAME="xtensa-lx106-elf-gcc8_4_0-esp-2020r3-linux-amd64"
XTENSA_LX106_ELF_TOOLCHAIN_PATH=$(pwd)/xtensa-lx106-elf
if [ ! -d "$XTENSA_LX106_ELF_TOOLCHAIN_PATH" ]; then
    wget https://dl.espressif.com/dl/$XTENSA_LX106_ELF_TOOLCHAIN_NAME.tar.gz -O $XTENSA_LX106_ELF_TOOLCHAIN_NAME.tar.gz
    tar -xzf $XTENSA_LX106_ELF_TOOLCHAIN_NAME.tar.gz
fi
export PATH=$XTENSA_LX106_ELF_TOOLCHAIN_PATH/bin:$PATH





cd $ABS_PLATFORM_PATH

##
# lx106-hal - Hardware Abstraction Layer for ESP8266
##

LX106_HAL_NAME="lx106-hal"
if [ ! -d "$LX106_HAL_NAME" ]; then
    git clone https://github.com/tommie/lx106-hal.git $LX106_HAL_NAME
fi
LX106_HAL_PATH=$(pwd)/$LX106_HAL_NAME

cd $LX106_HAL_PATH
autoreconf -i
mkdir -p build
cd build
../configure --host=xtensa-lx106-elf --prefix=$XTENSA_LX106_ELF_TOOLCHAIN_PATH/xtensa-lx106-elf
make -j6
make install




cd $ABS_PLATFORM_PATH

##
# esp-open-rtos
##

ESP_OPEN_RTOS_NAME="esp-open-rtos"
if [ ! -d "$ESP_OPEN_RTOS_NAME" ]; then
    git clone --recursive https://github.com/Superhouse/esp-open-rtos.git $ESP_OPEN_RTOS_NAME
fi
ESP_OPEN_RTOS_PATH=$(pwd)/$ESP_OPEN_RTOS_NAME

cd $ESP_OPEN_RTOS_NAME
git pull
git submodule init
git submodule update --recursive --progress

cd $ESP_OPEN_RTOS_PATH
# Fix the format for esptool
sed -i 's/$(FLASH_SIZE)m/$(FLASH_SIZE)MB/' parameters.mk


cd $ABS_PLATFORM_PATH

##
# Finish
##

# Move the $PLATFORM_PATH/activate file to $PLATFORM_PATH/activate.(current date).bak if it exists
if [ -f $ABS_PLATFORM_PATH/activate ]; then
#     BACKUP_CONFIG_NAME="activate.$(date +%Y-%m-%d_%H-%M-%S).bak"
#     echo "Your old activate configuration was moved to $BACKUP_CONFIG_NAME"
#     mv $ABS_PLATFORM_PATH/activate $ABS_PLATFORM_PATH/$BACKUP_CONFIG_NAME
    rm $ABS_PLATFORM_PATH/activate
fi

# Append configuration to platform_config file, so it can be used by `source platform/activate`
echo "#!/bin/bash" >> $ABS_PLATFORM_PATH/activate
echo "export XTENSA_LX106_ELF_TOOLCHAIN_PATH=$XTENSA_LX106_ELF_TOOLCHAIN_PATH" >> $ABS_PLATFORM_PATH/activate
echo "export LX106_HAL_PATH=$LX106_HAL_PATH" >> $ABS_PLATFORM_PATH/activate
echo "export ESP_OPEN_RTOS_PATH=$ESP_OPEN_RTOS_PATH" >> $ABS_PLATFORM_PATH/activate
echo "export PATH=$XTENSA_LX106_ELF_TOOLCHAIN_PATH/bin:\$PATH" >> $ABS_PLATFORM_PATH/activate

# Check if esptool.py is available in path
if ! command -v esptool.py &> /dev/null
then
    echo "esptool.py could not be found. Please install it using 'pip install esptool'."
fi

cd $INITIAL_DIR
echo "Done. Use 'source $PLATFORM_PATH/activate' to activate the platform."
