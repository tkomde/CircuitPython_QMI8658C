# SPDX-FileCopyrightText: 2017 Scott Shawcroft, written for Adafruit Industries
# SPDX-FileCopyrightText: Copyright (c) 2023 Taiki Komoda for JINS Inc.
#
# SPDX-License-Identifier: Unlicense
"""
Sample script for MPU6886
"""

import time
import board
from busio import I2C
import qmi8658c

i2c = I2C(board.IMU_SCL, board.IMU_SDA)
sensor = qmi8658c.QMI8658C(i2c)

while True:
    ac = sensor.acceleration
    gy = sensor.gyro
    print(f"Acceleration: X:{ac[0]:.2f}, Y:{ac[1]:.2f}, Z:{ac[2]:.2f} m/s^2")
    print(f"Gyro X:{gy[0]:.2f}, Y:{gy[1]:.2f}, Z:{gy[2]:.2f} rad/s")
    print(f"Temperature: {sensor.temperature:.2f} C")
    print(f"Timestamp: {sensor.timestamp}")

    time.sleep(1)
