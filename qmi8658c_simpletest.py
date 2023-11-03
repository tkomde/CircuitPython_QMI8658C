#Sample script for MPU6886

import time
import board
from busio import I2C
import qmi8658c

i2c = I2C(board.IMU_SCL, board.IMU_SDA)
sensor = qmi8658c.QMI8658C(i2c)

while True:
    print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2"%(sensor.acceleration))
    print("Gyro X:%.2f, Y: %.2f, Z: %.2f degrees/s"%(sensor.gyro))
    print("Temperature: %.2f C"%(sensor.temperature))
    
    time.sleep(1)
