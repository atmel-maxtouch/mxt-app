obp-utils {#mainpage}
=========

Command line utilities for Atmel maXTouch chips which use the Object Based Protocol (OBP).

Downloading, building & running
-------------------------------

    git clone https://github.com/atmel-maxtouch/obp-utils.git

There are two build harnesses, for Android and autotools:

### Android

    ndk-build

Binaries will be placed in libs/armeabi

#### Running on Android

    adb push libs/armeabi/mxt-app /data/local/tmp/
    adb shell /data/local/tmp/mxt-app [command]

### autotools

    ./autogen.sh && make

For arm-gnueabi:

    ./autogen.sh --host=arm-linux-gnueabi && make

Hardware backends
-----------------

The utils have three backends for hardware access:

### sysfs

This is used in conjunction with Atmel kernel driver

### USB

Many maXTouch devices support a USB mode which reports touches via USB HID.
They also have a control interface compatible with obp-utils.

USB mode is built by autotools when libusb is available.

### i2c-dev

by giving adapter and address on command line

mxt-app
-------

### Usage:
    mxt-app [command] [args]

When run with no options, access menu interface.

### Available commands:

    -h [--help]                : display this help and exit
    -R [--read]                : read from object
    -t [--test]                : run all self tests
    -W [--write]               : write to object
    --flash FIRMWARE           : send FIRMWARE to bootloader
    --reset                    : reset device
    --reset-bootloader         : reset device in bootloader mode
    --backup                   : backup configuration to NVRAM
    --calibrate                : send calibrate command
    --load FILE                : upload config from FILE
    --save FILE                : save config to FILE
    --debug-dump FILE          : capture T37 diagnostic data to FILE
    -g                         : store golden references
    --version                  : print version

### Valid options:

    -n [--count] COUNT         : read/write COUNT registers
    -f [--format]              : format register output
    -I [--instance] INSTANCE   : select object INSTANCE
    -r [--register] REGISTER   : start at REGISTER
    -T [--type] TYPE           : select object TYPE
    -v [--verbose] LEVEL       : print additional debug

### For TCP socket:

    -C [--bridge-client] HOST  : connect over TCP to HOST
    -S [--bridge-server]       : start TCP socket server
    -p [--port] PORT           : TCP port (default 4000)

### For bootloader mode:

    --firmware-version VERSION : Check firmware VERSION before and after flash

### For T37 diagnostic data:

    --frames N                 : Capture N frames of data
    --references               : Dump references data

### For i2c-dev and bootloader mode:

    -d [--i2c-adapter] ADAPTER : i2c adapter, eg "2"
    -a [--i2c-address] ADDRESS : i2c address, eg "4a"

### For T68 serial data:

    --t68-file FILE            : Upload FILE
    --t68-datatype DATATYPE    : Select DATATYPE

### Examples:

    ./mxt-app -R -n7 -r0      : Read info block
    ./mxt-app -R -T9 --format : Read T9 object, formatted output
    ./mxt-app -W -T7 0000     : Zero first two bytes of T7
    ./mxt-app --test          : run self tests
