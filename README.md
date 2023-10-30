# Circuitpython_QMI8658C

## Introduction

CircuitPython helper library for the QMI8658C 6-DoF Accelerometer and Gyroscope

## Dependencies

This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.

## Usage Example

```
import time
import board
import qmi8658c

i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = qmi8658c.QMI8658C(i2c)

while True:
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(mpu.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(mpu.gyro))
    print("Temperature: %.2f C"%mpu.temperature)
    print("")
    time.sleep(1)
```