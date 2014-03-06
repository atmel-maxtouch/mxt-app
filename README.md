% MXT-APP(1) User manual
% Nick Dyer
% 29 Nov 2013

# NAME

mxt-app - command line utility for maXTouch devices

# SYNOPSIS

mxt-app [*command*] \[*options*\]...

# DESCRIPTION

mxt-app is a utility for managing Atmel maXTouch touch controllers and other
devices that support Atmel Object Based Protocol.

If no *command* is not given, mxt-app will provide an interactive menu based
interface.

# OBJECT PROTOCOL

The Atmel Object Based Protocol defines how device registers (normally
accessed via I2C) are mapped to different functions within the devices. This
interface organises the register map into separate objects each of which is
given a T number. *mxt-app* can inspect and alter the object configuration,
and view diagnostic data, while the device is running.

For a description of object protocol, see *Atmel AT42QT1085
Object Protocol Guide*, available from atmel.com.

The meaning of the configuration bytes within the objects may be found in the
Protocol Guide documentation released with each device, and is only provided
by Atmel under NDA.

# GENERAL COMMANDS

`-h [--help]`
:   Display a brief summary of available options and exit.

`-i [--info]`
:   Print the ID information and object table.

`-M [--messages]`
:   Print the messages.

`--reset`
:   Reset device.

`--calibrate`
:   Send calibrate command.

`--backup`
:   Backup configuration to NVRAM.

`-g`
:   Write Golden Reference calibration to NVRAM.

`--self-cap-tune`
:   Tune and calibrate the self capacitance settings and store them to NVRAM.

`-t [--test]`
:   Run all self tests using the T25 Self Test object.

`--version`
:   print version of mxt-app.

# CONFIGURATION FILE COMMANDS

`--load *FILE*`
:   Upload config from *FILE*, write it to NVRAM, and reset device. The
    configuration may be in `.xcfg` or `OBP_RAW` format.

`--save *FILE*`
:   Save config to *FILE* in `OBP_RAW` format.

# REGISTER READ/WRITE COMMANDS

`-R [--read]`
:   Read data from the device.

`-W [--write]`
:   Write data to the device.

`-n [--count] *COUNT*`
:   read/write *COUNT* registers

`-f [--format]`
:   format register output

`-I [--instance] *INSTANCE*`
:   select object *INSTANCE*

`-r [--register] *REGISTER*`
:   start at *REGISTER*

`-T [--type] *TYPE*`
:   select object *TYPE*

## EXAMPLES

### Read info block:

    $ mxt-app -R -n7 -r0
    82 19 11 AA 18 0E 16

### Read T7 Power Config object:

    $ mxt-app -R -T7
    32 FF 05 43

### Zero first two bytes of T7:

    $ mxt-app -W -T7 0000

### Read T7 Power Config object, formatted output:

    $ mxt-app -R -T7 --format
    GEN_POWERCONFIG_T7

    00: 0x00    0 0000 0000
    01: 0x00    0 0000 0000
    02: 0x05    5 0000 0101
    03: 0x43   67 0100 0011

# TCP SOCKET COMMANDS

*mxt-app* supports connection over TCP using a ASCII protocol which allows
mxt-app to act as a bridge so that Atmel proprietary tools such as *Object
Server* can access the device.

`-C [--bridge-client] *HOST*`
:   Connect over TCP to *HOST*

`-S [--bridge-server]`
:   Start TCP socket server

`-p [--port] PORT`
:   TCP port (default 4000)

# BOOTLOADER OPTIONS

`--flash *FIRMWARE*`
:   Flash *FIRMWARE* to device. The firmware file should be in `.enc` format.

`--reset-bootloader`
:   Reset device in bootloader mode. In bootloader mode the device will cease
    normal operation until a firmware is sent. The I2C address or USB PID will
    change. The only valid command in this mode is `--flash`. A hard power
    cycle will return the device to normal Object Protocol mode, unless the
    firmware image is corrupted. This command is only provided for debugging
    purposes: in most cases `--flash` will manage the change to/from
    bootloader mode before/after flash.

`--firmware-version *VERSION*`
:   The .enc file format does not provide the firmware version in a form
    available to mxt-app. If it is provided via this switch, mxt-app can check
    firmware *VERSION* before and after flash. It will skip the flash process
    if the firmware version is already correct. It will also check for a
    successful flash on completion. The version must be provided in the format
    `1.0.AA`.

# T37 DIAGNOSTIC DATA OPTIONS

`--debug-dump *FILE*`
:   The T37 Diagnostic Data object provides raw access to touch reference/delta
    measurements from the touch screen. Diagnostic data is written to *FILE* in
    CSV format. The format is compatible with the Atmel Hawkeye utility.

`--frames *N*`
:   Capture *N* frames of data

`--references`
:   Dump references data (normal mode is to capture touch deltas)

# T68 SERIAL DATA COMMANDS

`--t68-file *FILE*`
:   Upload *FILE* to the device via the T68 Serial Data object.

`--t68-datatype *DATATYPE*`
:   Set *DATATYPE* of the file. This will be automatically detected from the
    file itself in most cases.

# FINDING AND SPECIFYING DEVICE

By default mxt-app will scan available devices and connect to the first device
it finds.

`-q [--query]`
:   Scan for devices and output a list.

`-d [--device] *DEVICESTRING*`
:   Connect to a particular device specified by *DEVICESTRING* which is given
    in the same format as output by `--query`.

There are three connection methods supported for hardware access:

## sysfs

This is used in conjunction with the Linux kernel driver. It accesses sysfs
attributes under the directory

    /sys/bus/i2c/drivers/dddddddd/b-00xx/

Where

d
:   driver name - `atmel_mxt_ts`, `Atmel MXTXXXX`, etc

b
:   i2c adapter

xx
:   i2c address

A specific USB device can be specified by giving a device
option `-d sysfs:PATH` as given by `-q/--query` option

The sysfs attributes used under this directory are

`mem_access`
:   Access to raw I2C address space.

`debug_enable`
:   Output messages from the device to dmesg log as hexadecimal.

`debug_v2_enable`, `debug_msg`, `debug_notify`
:   Optional improved binary interface to retrieve messages

They are provided when using the Atmel kernel driver from github, and may be
supported by other device.

## USB

Many maXTouch devices support a USB mode which reports touches via USB HID. In
addition, evaluation boards may use a "bridge chip" which interfaces I2C to the
same protocol.

USB mode will be built by autotools when libusb is available.

A specific USB device can be specified by giving a device
option `-d usb:001-003` which corresponds to the bus and device numbers
given by the `-q/--query` option and lsusb.

## I2C debug interface

Devices can be accessed directly via the *i2c-dev* I2C debug interface by giving
adapter and address on command line.

The i2c-dev interface is documented in the kernel source, in
    Documentation/i2c/dev-interface

It is enabled on a system if files /dev/i2c-x are present.

To use i2c-dev, provide a device string such as `-d i2c-dev:1-004a`.

There is no scanning support. You must identify the correct adapter and address
by reference to the protocol guide or to the platform setup.

It is possible to use the `--flash` command with a device already in bootloader
mode, by specifying the bootloader address.

# DEBUG OPTIONS

`-v [--verbose] *LEVEL*`
:   print additional debug. *LEVEL* is one of 0 (Silent), 1 (Warnings and
    Errors), 2 (Info), 3 (Debug), 4 (Verbose). Debug and Verbose are only
    available if built in.

# EXIT VALUES

0
:   Success
Negative
:   Failure

# COMPILING FROM SOURCE

To download the source code using git:

    git clone https://github.com/atmel-maxtouch/obp-utils.git

There are two build harnesses, for Android and autotools:

## Android

    ndk-build

Binaries will be placed in libs/armeabi

### Running on Android

    adb push libs/armeabi/mxt-app /data/local/tmp/
    adb shell /data/local/tmp/mxt-app [command]

If executable permissions have not been set, run:
    adb shell chmod 777 /data/local/tmp/mxt-app

## autotools

To compile using autotools:

    ./autogen.sh && make

To cross-compile:

    ./autogen.sh --host=arm-linux-gnueabi && make

## Version numbering

A version number is generated by `git describe` during the build process and
is reported by `--version` and to debug logs.

A typical version might be `1.15-29-g8321` which means, 29 commits after the
release tag 1.15, with a git SHA id beginning with 8321.

If the source is not checked out using git (for example by clicking on the
github "Download ZIP" link), then the version from the file VERSION in the
source archive is used.

The suffix `-mod` is appended if there are uncommitted changes in the source
code.
