import time
import board
import busio
import adafruit_mpu6050
import math

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)
angle = 0
start_time = time.time()
while True:
    try:
        x, y, z = mpu.gyro
        if abs(z) < 10:  z = 0
        angle = angle + z * (start_time - time.time())
        start_time = time.time()
        print(angle)
    # print("Temperature: %.2f C" % mpu.temperature)
    # print("")
    except IOError:
        pass
