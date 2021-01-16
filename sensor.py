import sys
import signal
import RPi.GPIO as GPIO
import time
import math
import socket

#socket for sending metadata
sock_metadata = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_station_address_metadata = ('192.168.137.1', 3333)
ENCODER_1 = 22
ENCODER_2 = 23
encoder_1_pulses = 0
encoder_2_pulses = 0
velocity_1 = 0
velocity_2 = 0


""" Display compass heading data five times per second """
import time
from math import atan2, degrees
import board
import busio
import adafruit_mpu6050
import math

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)
angle = 0
x_coordinate = 0
y_coordinate = 0
x_velocity = 0
x_current_acc = 0
gyro_start_time = time.time()
coordinates_start_time = time.time()
z_offset = 0
velocity_start_time = time.time()

def calculate_coordinates():
    global x_coordinate
    global y_coordinate
    global coordinates_start_time
    global angle
    global x_velocity
    global x_current_acc
    global z_offset
    try:
        x_acc, y_acc, z_acc = mpu.acceleration
        x_acc -= z_offset
        # print(x_acc)
        # if abs(x_acc) < 0.4: x_acc = 0
        # print(x_acc)
        interval = coordinates_start_time - time.time()
        x_coordinate = x_coordinate + (x_velocity * interval) + (0.5 * x_current_acc * interval * interval)
        y_coordinate = x_coordinate * math.tan(math.radians(angle))
        # print(encoder_1_pulses, " and -- ", encoder_2_pulses)
        # print(angle)
        if encoder_1_pulses > 10  and encoder_2_pulses > 10:
            x_velocity = x_velocity + x_current_acc * interval
            # print(x_velocity)
            x_current_acc = x_acc * 100
        else:
            x_velocity = 0
            x_current_acc = 0

        coordinates_start_time = time.time()
    except IOError:
        pass

def calculate_heading():
    global angle
    global gyro_start_time
    global z_offset
    try:
        x, y, z = mpu.gyro
        # print("y_acc = ", y)
        z = z - z_offset
        # print('z = ', z)
        if abs(z) < 4 :  z = 0
        angle = angle - z * (gyro_start_time - time.time())
        if angle > 360: angle = angle - 360
        if angle < -360: angle = angle + 360
        gyro_start_time = time.time()
    except IOError:
        pass

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def encoder_1_pulse_detection_handler(gpio_number):
    global encoder_1_pulses
    encoder_1_pulses =  encoder_1_pulses + 1

def encoder_2_pulse_detection_handler(gpio_number):
    global encoder_2_pulses
    encoder_2_pulses =  encoder_2_pulses + 1

def mpu_trip(gpio_number):
    print("trigger")

def calibrate_gyro():
    try :
        global z_offset
        global mpu
        total_z_degree = 0
        for i in range(1, 1000):
            _, _ , z_degree = mpu.gyro
            total_z_degree += z_degree
        z_offset = total_z_degree / 1000
    except IOError:
        calibrate_gyro()


def send_metadata(location_queue):
    global encoder_1_pulses
    global encoder_2_pulses
    global x_coordinate
    global y_coordinate
    global velocity_start_time
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_1, GPIO.FALLING,
                          callback=encoder_1_pulse_detection_handler)
    GPIO.setup(ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_2, GPIO.FALLING,
                          callback=encoder_2_pulse_detection_handler)
    GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(27, GPIO.FALLING, callback=mpu_trip)
    GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # offset = get_heading(sensor)
    # distance = 0
    time.sleep(3)
    calibrate_gyro()
    velocity = 0
    while True:
        time.sleep(0.1)

        calculate_heading()
        # calculate_coordinates()
        time_interval = time.time() - velocity_start_time
        distance = velocity * time_interval
        velocity_1 = (GPIO.input(16) - GPIO.input(20)) * (encoder_1_pulses * 20.42 * (1 / time_interval) / 374.0)
        encoder_1_pulses = 0
        velocity_2 = (GPIO.input(19) - GPIO.input(26)) * (encoder_2_pulses * 20.42 * (1 / time_interval) / 374.0)
        encoder_2_pulses = 0
        velocity_start_time = time.time()
        velocity = velocity_1 + (velocity_2 - velocity_1) / 2

        x_coordinate = x_coordinate + distance * math.cos(math.radians(angle))
        y_coordinate = y_coordinate + distance * math.sin(math.radians(angle))
        # print('angle = ', angle)
        # print('x = ', x_coordinate, 'y = ', y_coordinate)
        sock_metadata.sendto(b'v='+bytearray(str(round(velocity,2)).encode()), control_station_address_metadata)
        sock_metadata.sendto(b'a='+bytearray(str(round(angle, 2)).encode()), control_station_address_metadata)
        sock_metadata.sendto(b'x='+bytearray(str(round(x_coordinate, 2)).encode()), control_station_address_metadata)
        sock_metadata.sendto(b'y='+bytearray(str(round(y_coordinate, 2)).encode()), control_station_address_metadata)
        if angle > 160 and angle < 190 and x_coordinate > 130 and x_coordinate < 160 and y_coordinate > 90 and y_coordinate < 130:
            location_queue.put(True)


    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

