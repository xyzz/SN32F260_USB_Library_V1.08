#!/bin/bash

arm-none-eabi-gcc -mthumb -mcpu=cortex-m0 -I inc/ -D__irq= \
    -nostartfiles -nostdlib \
    -g -Og \
    -T linker.x \
    ./main.c \
    ./Usb/usbdesc.c \
    ./Usb/usbepfunc.c \
    ./Usb/usbhw.c \
    ./Usb/usbram.c \
    ./Usb/usbuser.c \
    ./UsbHid/hidram.c \
    ./UsbHid/hiduser.c \
    ./Utility/Utility.c \
    ./RTE/Device/SN32F268F/system_SN32F260.c \
    -lgcc \
    -o main.elf

arm-none-eabi-objcopy -O binary main.elf main.bin
