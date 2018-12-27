# NT3H2111 Driver

This is a driver needed by `NFCEEPROOM` constructor for the NXP NT3H2111 NFC chip.

## Requirements

This driver requires the NXP NT3211 chip. This is present on the NXP OM23221ARD development kits.

## Configuration

Driver needs to be instantiated with valid pin names for I2C communication, GPO and RF disable pin.

## Building instructions

Driver will be built as part of the mbed-os build. Place the driver in the root directory of the mbed-os application. This can be done by placing a `.lib` file in the root of your application with a repository address with the driver. For example, to include this driver you can create a file called `eeprom_driver.lib` with these contents:

```
https://github.com/ARMmbed/mbed-nfc-nt3h2111
```
