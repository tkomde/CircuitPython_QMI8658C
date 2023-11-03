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
from time import sleep
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


class AccRange:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`accelerometer_range`.

    * :py:attr:`AccRange.RANGE_2_G`
    * :py:attr:`AccRange.RANGE_4_G`
    * :py:attr:`AccRange.RANGE_8_G`
    * :py:attr:`AccRange.RANGE_16_G`

    """

    RANGE_2_G = 0  # +/- 2g
    RANGE_4_G = 1  # +/- 4g
    RANGE_8_G = 2  # +/- 8g (default value)
    RANGE_16_G = 3  # +/- 16g


class GyroRange:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`gyro_range`.

    * :py:attr:`GyroRange.RANGE_16_DPS`
    * :py:attr:`GyroRange.RANGE_32_DPS`
    * :py:attr:`GyroRange.RANGE_64_DPS`
    * :py:attr:`GyroRange.RANGE_128_DPS`
    * :py:attr:`GyroRange.RANGE_256_DPS`
    * :py:attr:`GyroRange.RANGE_512_DPS`
    * :py:attr:`GyroRange.RANGE_1024_DPS`
    * :py:attr:`GyroRange.RANGE_2048_DPS`

    """

    RANGE_16_DPS = 0  # +/- 16 deg/s
    RANGE_32_DPS = 1  # +/- 32 deg/s
    RANGE_64_DPS = 2  # +/- 64 deg/s
    RANGE_128_DPS = 3  # +/- 128 deg/s
    RANGE_256_DPS = 4  # +/- 256 deg/s (default value)
    RANGE_512_DPS = 5  # +/- 512 deg/s
    RANGE_1024_DPS = 6  # +/- 1024 deg/s
    RANGE_2048_DPS = 7  # +/- 2048 deg/s


class AccRate:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`accelerometer_rate`.
    Accelerometer low power mode must be a gyro disabled.

    * :py:attr:`AccRate.RATE_8000_HZ`
    * :py:attr:`AccRate.RATE_4000_HZ`
    * :py:attr:`AccRate.RATE_2000_HZ`
    * :py:attr:`AccRate.RATE_1000_HZ`
    * :py:attr:`AccRate.RATE_500_HZ`
    * :py:attr:`AccRate.RATE_250_HZ`
    * :py:attr:`AccRate.RATE_125_HZ`
    * :py:attr:`AccRate.RATE_62_HZ`
    * :py:attr:`AccRate.RATE_31_HZ`
    * :py:attr:`AccRate.RATE_LP_128_HZ`
    * :py:attr:`AccRate.RATE_LP_21_HZ`
    * :py:attr:`AccRate.RATE_LP_11_HZ`
    * :py:attr:`AccRate.RATE_LP_3_HZ`

    """

    RATE_8000_HZ = 0
    RATE_4000_HZ = 1
    RATE_2000_HZ = 2
    RATE_1000_HZ = 3
    RATE_500_HZ = 4
    RATE_250_HZ = 5
    RATE_125_HZ = 6 #(default value)
    RATE_62_HZ = 7
    RATE_31_HZ = 8
    RATE_LP_128_HZ = 12
    RATE_LP_21_HZ = 13
    RATE_LP_11_HZ = 14
    RATE_LP_3_HZ = 15


class GyroRate:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`gyro_rate`.

    * :py:attr:`AccRate.RATE_8000_HZ`
    * :py:attr:`AccRate.RATE_4000_HZ`
    * :py:attr:`AccRate.RATE_2000_HZ`
    * :py:attr:`AccRate.RATE_1000_HZ`
    * :py:attr:`AccRate.RATE_500_HZ`
    * :py:attr:`AccRate.RATE_250_HZ`
    * :py:attr:`AccRate.RATE_125_HZ`
    * :py:attr:`AccRate.RATE_62_HZ`
    * :py:attr:`AccRate.RATE_31_HZ`

    """

    RATE_8000_HZ = 0
    RATE_4000_HZ = 1
    RATE_2000_HZ = 2
    RATE_1000_HZ = 3
    RATE_500_HZ = 4
    RATE_250_HZ = 5
    RATE_125_HZ = 6 #(default value)
    RATE_62_HZ = 7
    RATE_31_HZ = 8
    

class QMI8658C:
    _device_id = ROUnaryStruct(_QMI8658C_WHO_AM_I, "B")
    _revision_id = ROUnaryStruct(_QMI8658C_REVISION_ID, "B")

    _ctrl1 = RWBits(8, 0x02, 0)
    #_ctrl2 = RWBits(8, 0x03, 0)
    _accelerometer_range = RWBits(3, 0x03, 4)
    _accelerometer_rate = RWBits(4, 0x03, 0)
    #_ctrl3 = RWBits(8, 0x04, 0)
    _gyro_range = RWBits(3, 0x04, 4)
    _gyro_rate = RWBits(4, 0x04, 0)
    _ctrl4 = RWBits(8, 0x05, 0)
    _ctrl5 = RWBits(8, 0x06, 0)
    _ctrl6 = RWBits(8, 0x07, 0)
    #_ctrl7 = RWBits(8, 0x08, 0)
    _accelerometer_enable = RWBits(1, 0x08, 0)
    _gyro_enable = RWBits(1, 0x08, 1)
    
    _raw_time_data = StructArray(_QMI8658C_TIME_OUT, "<H", 3)
    _raw_temp_data = StructArray(_QMI8658C_TEMP_OUT, "B", 2)#
    _raw_accel_data = StructArray(_QMI8658C_ACCEL_OUT, "<h", 6)
    _raw_gyro_data = StructArray(_QMI8658C_GYRO_OUT, "<h", 6)
    _raw_accel_gyro_data = StructArray(_QMI8658C_ACCEL_OUT, "<h", 12)

    # these vars are called very frequently
    _acc_scale = 1
    _gyro_scale = 1

    def __init__(self,i2c_bus: I2C, address=0X6B) -> None:
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        #print(f"_device_id/_revision_id {self._device_id}/{self._revision_id}")

        if self._device_id != 0x05:
            raise RuntimeError("Failed to find QMI8658C")

        #Config
        # REG CTRL1 Enables 4-wire SPI interface,  address auto increment, SPI read data big endian
        self._ctrl1 = 0b01100000
        # REG CTRL2 : QMI8658CAccRange_8g  and QMI8658CAccOdr_125Hz
        #self._ctrl2 = (2 << 4) + 0b0110 
        self.accelerometer_range = AccRange.RANGE_8_G
        sleep(0.01)
        self.accelerometer_rate = AccRate.RATE_125_HZ
        sleep(0.01)
        # REG CTRL3 : QMI8658CGyrRange_512dps and QMI8658CGyrOdr_125Hz
        #self._ctrl3 = (5 << 4) + 0b0110
        self.gyro_range = GyroRange.RANGE_512_DPS
        sleep(0.01)
        self.gyro_rate = GyroRate.RATE_125_HZ
        sleep(0.01)
        # REG CTRL4 : No magnetometer
        self._ctrl4 = 0x00
        # REG CTRL5 : Disables Gyroscope And Accelerometer Low-Pass Filter
        self._ctrl5 = 0x00
        # REG CTRL6 : Disables Motion on Demand.
        self._ctrl6 = 0x00
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        #self._ctrl7 = 0b00000011
        sleep(0.01)
        self._accelerometer_enable = 1
        sleep(0.1)
        self._gyro_enable = 1
        sleep(0.1)

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

        # setup range dependant scaling
        accel_x = (raw_x / self._acc_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / self._acc_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / self._acc_scale) * STANDARD_GRAVITY

        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self) -> Tuple[float, float, float]:
        """Gyroscope X, Y, and Z axis data in :math:`º/s`"""
        raw_data = self._raw_gyro_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        # setup range dependant scaling
        gyro_x = radians(raw_x / self._gyro_scale)
        gyro_y = radians(raw_y / self._gyro_scale)
        gyro_z = radians(raw_z / self._gyro_scale)

        return (gyro_x, gyro_y, gyro_z)

    @property
    def raw_acc_gyro(self) -> Tuple[float, float, float]:
        """Raw data extraction"""
        raw_data = self._raw_accel_gyro_data

        return (raw_data[0][0], raw_data[1][0], raw_data[2][0], raw_data[3][0], raw_data[4][0], raw_data[5][0])

    @property
    def accelerometer_range(self) -> int:
        """The measurement range of all accelerometer axes. Must be a `AccRange`"""
        return self._accelerometer_range

    @accelerometer_range.setter
    def accelerometer_range(self, value: int) -> None:
        if (value < 0) or (value > 3):
            raise ValueError("accelerometer_range must be a AccRange")
        
        if value == AccRange.RANGE_16_G:
            self._acc_scale = 2048
        if value == AccRange.RANGE_8_G:
            self._acc_scale = 4096
        if value == AccRange.RANGE_4_G:
            self._acc_scale = 8192
        if value == AccRange.RANGE_2_G:
            self._acc_scale = 16384

        self._accelerometer_range = value 
        sleep(0.01)

    @property
    def accelerometer_rate(self) -> int:
        """The measurement rate of all accelerometer axes. Must be a `AccRate`"""
        return self._accelerometer_rate

    @accelerometer_rate.setter
    def accelerometer_rate(self, value: int) -> None:
        if (value < 0 or value > 15 or (value >= 9 and value <= 11)):
            raise ValueError("accelerometer_rate must be a AccRate")
        
        if ((value >= 12 and value <= 15) and self._gyro_enable == 1):
            raise ValueError("accelerometer low power mode must be a gyro disabled")
        
        self._accelerometer_rate = value
        sleep(0.01)

    @property
    def gyro_range(self) -> int:
        """The measurement range of all gyroscope axes. Must be a `GyroRange`"""
        return self._gyro_range

    @gyro_range.setter
    def gyro_range(self, value: int) -> None:
        if (value < 0) or (value > 7):
            raise ValueError("gyro_range must be a GyroRange")
        
        if value == GyroRange.RANGE_16_DPS:
            self._gyro_scale = 2048
        if value == GyroRange.RANGE_32_DPS:
            self._gyro_scale = 1024
        if value == GyroRange.RANGE_64_DPS:
            self._gyro_scale = 512
        if value == GyroRange.RANGE_128_DPS:
            self._gyro_scale = 256
        if value == GyroRange.RANGE_256_DPS:
            self._gyro_scale = 128
        if value == GyroRange.RANGE_512_DPS:
            self._gyro_scale = 64
        if value == GyroRange.RANGE_1024_DPS:
            self._gyro_scale = 32
        if value == GyroRange.RANGE_2048_DPS:
            self._gyro_scale = 16

        self._gyro_range = value
        sleep(0.01)

    @property
    def gyro_rate(self) -> int:
        """The measurement rate of all gyroscope axes. Must be a `GyroRate`"""
        return self._gyro_rate

    @gyro_rate.setter
    def gyro_rate(self, value: int) -> None:
        if (value < 0 or value > 8):
            raise ValueError("gyro_rate must be a GyroRate")
        self._gyro_rate = value
        sleep(0.01)

    @property
    def accelerometer_enable(self) -> int:
        return self._accelerometer_enable

    @accelerometer_enable.setter
    def accelerometer_enable(self, value: int) -> None:
        if (value < 0 or value > 1):
            raise ValueError("accelerometer_enable must be a 0/1")
        self._accelerometer_enable = value
        sleep(0.1)
        
    @property
    def gyro_enable(self) -> int:
        return self._gyro_enable

    @gyro_enable.setter
    def gyro_enable(self, value: int) -> None:
        if (value < 0 or value > 1):
            raise ValueError("gyro_enable must be a 0/1")
        self._gyro_enable = value
        sleep(0.1)