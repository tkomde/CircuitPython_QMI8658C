# SPDX-FileCopyrightText: 2019 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`qmi8658C`
================================================================================

CircuitPython helper library for the QMI8658C 6-DoF Accelerometer and Gyroscope

"""

# imports

__version__ = "0.0.0+auto.0"

from math import radians
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_struct_array import StructArray
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_bus_device import i2c_device

_QMI8658C_WHO_AM_I = 0x0  # WHO_AM_I register
_QMI8658C_REVISION_ID = 0x1  # Divice ID register

_QMI8658C_TIME_OUT = 0x30  # time data byte register
_QMI8658C_TEMP_OUT = 0x33  # temp data byte register
_QMI8658C_ACCEL_OUT = 0x35  # base address for sensor data reads
_QMI8658C_GYRO_OUT = 0x3b  # base address for sensor data reads

STANDARD_GRAVITY = 9.80665

class QMI8658C:
    _device_id = ROUnaryStruct(_QMI8658C_WHO_AM_I, "B")
    _revision_id = ROUnaryStruct(_QMI8658C_REVISION_ID, "B")

    _config1 = RWBits(8, 0x02, 0)
    _config2 = RWBits(8, 0x03, 0)
    _config3 = RWBits(8, 0x04, 0)
    _config4 = RWBits(8, 0x05, 0)
    _config5 = RWBits(8, 0x06, 0)
    _config6 = RWBits(8, 0x07, 0)
    _config7 = RWBits(8, 0x08, 0)

    _raw_time_data = StructArray(_QMI8658C_TIME_OUT, "<H", 3)
    _raw_temp_data = StructArray(_QMI8658C_TEMP_OUT, "B", 2)#
    _raw_accel_data = StructArray(_QMI8658C_ACCEL_OUT, "<h", 6)
    _raw_gyro_data = StructArray(_QMI8658C_GYRO_OUT, "<h", 6)
    _raw_accel_gyro_data = StructArray(_QMI8658C_ACCEL_OUT, "<h", 12)

    def __init__(self,i2c_bus: I2C, address=0X6B) -> None:
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != 0x05:
            raise RuntimeError("Failed to find QMI8658C")

        #Config
        # REG CTRL1 Enables 4-wire SPI interface,  address auto increment, SPI read data big endian
        self._config1 = 0b01100000
        # REG CTRL2 : QMI8658CAccRange_8g  and QMI8658CAccOdr_125Hz
        self._config2 = 0b00100110
        # REG CTRL3 : QMI8658CGyrRange_512dps and QMI8658CGyrOdr_125Hz
        self._config3 = 0b01010110
        # REG CTRL4 : No magnetometer
        self._config4 = 0x00
        # REG CTRL5 : Disables Gyroscope And Accelerometer Low-Pass Filter
        self._config5 = 0x00
        # REG CTRL6 : Disables Motion on Demand.
        self._config6 = 0x00
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        self._config7 = 0b00000011

    @property
    def timestamp(self) -> int:
        raw_timestamp = self._raw_time_data
        return raw_timestamp[0][0]

    @property
    def temperature(self) -> float:
        raw_temperature = self._raw_temp_data
        temp = raw_temperature[0][0] /256 + raw_temperature[1][0]
        return temp

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """Acceleration X, Y, and Z axis data in :math:`m/s^2`"""
        raw_data = self._raw_accel_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        accel_scale = 4096

        # setup range dependant scaling
        accel_x = (raw_x / accel_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / accel_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / accel_scale) * STANDARD_GRAVITY

        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self) -> Tuple[float, float, float]:
        """Gyroscope X, Y, and Z axis data in :math:`º/s`"""
        raw_data = self._raw_gyro_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        gyro_scale = 64

        # setup range dependant scaling
        gyro_x = radians(raw_x / gyro_scale)
        gyro_y = radians(raw_y / gyro_scale)
        gyro_z = radians(raw_z / gyro_scale)

        return (gyro_x, gyro_y, gyro_z)
