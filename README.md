# AXPB011 - TNx Protocol Bridge #

# Toolchain #

This project was developed using [this compiler](https://xpack.github.io/blog/2022/05/30/arm-none-eabi-gcc-v11-2-1-1-2-released/).


# Pre-requisites #

This project can be built/developed in any IDE. This README assume Eclipse is being used (other IDEs may differ, but the general method should be the same). 

Make must be installed on your system:

* For the makefile to run, it needs to know the location of the compiler. This can be done by creating a new system environment variable called ARM, and setting the path to the top level folder of the toolchain. For example:
    ```
    Variable name = ARM
    Variable value = C:/.../xpack-arm-none-eabi-gcc-11.2.1-1.2
    ```


# Building the Project #

This repository consists of two projects:
* Runtime application 
* DFU bootloader application

The projects will only build if the toolchain and environment variable have been set up correctly. To build and link the project, navigate to the top-level project folder:
* axpb011/AXPB011
* axpb011/AXPB011_DFU

Run the following command to compile and link the project:
```
make all
```

If setup correctly, the project should build.

**IMPORTANT**: There may be some warnings that look like the following:
```
Peripherals/GD32F3x0_usbfs_library/driver/Source/drv_usb_core.c: In function 'usb_txfifo_write':
Peripherals/GD32F3x0_usbfs_library/driver/Source/drv_usb_core.c:214:9: warning: 'packed' attribute ignored for type 'uint32_t *' {aka 'long unsigned int *'} [-Wattributes]
  214 |         *fifo = *((__attribute__((__packed__)) uint32_t *)src_buf);
      |         ^
Peripherals/GD32F3x0_usbfs_library/driver/Source/drv_usb_core.c: In function 'usb_rxfifo_read':
Peripherals/GD32F3x0_usbfs_library/driver/Source/drv_usb_core.c:237:9: warning: 'packed' attribute ignored for type 'uint32_t *' {aka 'long unsigned int *'} [-Wattributes]
  237 |         *(__attribute__((__packed__)) uint32_t *)dest_buf = *fifo;
      |         ^
```
These can be ignored.

# Target Device #
This project targets the GD32F350K8.


# Pin Mappings #

| Pin Function  | Pin Name           |
| ------------- | ------------------ |
| OSC_IN        | PF0                |
| OSC_OUT       | PF1                |
| BOOT_SEL      | PA0                |
| nRESET        | PA2                |
| nIRQ          | PB8                |
| USB_DM        | PA11               |
| USB_DP        | PA12               |
| SWDIO         | PA13               |
| SWCLK         | PA14               |
| COMMS_SELECT  | PA1                |
| SPI_nSS       | PA4                |
| SPI_SCK       | PA5                |
| SPI_MISO      | PA6                |
| SPI_MOSI      | PA7                |
| I2C_SCL       | PB6                |
| I2C_SDA       | PB7                | 
| LED_USB       | PB0                |
| LED_AXIOM     | PB1                |


# Programming the Device #

The AXPB011 relies on a custom bootloader to function correctly. The bootloader resides at the start of flash, with the runtime program starting after.

The addresses are:
```C
Bootloader Flash Start = 0x08000000
Runtime Flash Start = 0x08004000
```


# Boot Select #

The BOOT_SEL pin is read after a reset. 

* If the pin is read logic 1 (high) the chip will enumerate as a DFU device. 
    * The correct [DFU Driver](https://gd32mcu.com/download/down/document_id/244/path_type/1) must be installed.
* If the pin is read logic 0 (low) the chip will run the user application, provided flash at address 0x08004000 is not blank and the chip hasn't been reset from a command.