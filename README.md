# LSM6DS3 Driver

Source code for the LSM6DS3 I2C Inertial Measurement Unit for ESP-IDF.

## Introduction

This driver implements communications with the LSM6DS3 using the new i2c esp-idf driver (`driver/i2c_master.h`).

It is used to communicate with the LSM6DS3 gyroscope / accelerometer / thermometer using the ESP-IDF framework I2C master driver.

## Installation

To install the driver, you can clone the repository and place it in the `components` folder of your project.

```bash
git clone https://github.com/TNY-Robotics/lsm6ds3-esp-idf.git components/lsm6ds3
```

## Usage

The driver exposes a simple header file named `lsm6ds3.h`, that you can include using :

```c
#include "lsm6ds3.h"
```

## Examples

An example of how to create, configure, read and delete an lsm6ds3 object using the LSM6DS3 driver can be found in the `examples` folder.

## Licence

This driver is under the MIT Licence.

## Author

This driver was created by the [TNY Robotics](https://tny-robotics.com) team.

For any questions or suggestions, please contact us at [contact@tny-robotics.com](mailto:contact@tny-robotics.com).