import sys
import signal
import RPi.GPIO as GPIO
import time
import math
import socket

#socket for sending metadata
sock_metadata = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_station_address_metadata = ('192.168.2.14', 3333)
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

def calculate_coordinates():
    global x_coordinate
    global y_coordinate
    global coordinates_start_time
    global angle
    global x_velocity
    global x_current_acc
    try:
        x_acc, y_acc, z_acc = mpu.acceleration
        x_acc -= 0.5
        # print(x_acc)
        if abs(x_acc) < 0.4: x_acc = 0
        interval = coordinates_start_time - time.time()
        x_coordinate = x_coordinate + (x_velocity * interval) + (0.5 * x_current_acc * interval * interval)
        y_coordinate = x_coordinate * math.tan(math.radians(angle))
        # print(encoder_1_pulses, " and -- ", encoder_2_pulses)
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
    try:
        x, y, z = mpu.gyro
        # print("y_acc = ", y)
        if abs(z) < 10:  z = 0
        angle = angle + z * (gyro_start_time - time.time())
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

def send_metadata(metadata_queue):
    global encoder_1_pulses
    global encoder_2_pulses
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_1, GPIO.FALLING,
                          callback=encoder_1_pulse_detection_handler)
    GPIO.setup(ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_2, GPIO.FALLING,
                          callback=encoder_2_pulse_detection_handler)
    GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(27, GPIO.FALLING, callback=mpu_trip)
    # offset = get_heading(sensor)
    # distance = 0
    while True:
        encoder_1_pulses = 0
        encoder_2_pulses = 0
        time.sleep(0.1)
        calculate_heading()
        calculate_coordinates()
        velocity = x_velocity / math.cos(math.radians(angle))
        # heading = get_heading(sensor) - offset
        # x = str(math.cos(math.radians(angle)) * distance)
        # y = str(math.sin(math.radians(angle)) * distance)
        print('x = ', x_coordinate , 'y = ', y_coordinate)
        # print(encoder_1_pulses, ' and ', encoder_2_pulses)
        # print("heading ", heading)
        # print("x = ", distance, " ;  y = ", y)
        # print("heading: {:.2f} degrees".format(heading))
        # print("velocity: ", velocity, " cm/s")
        # print('angle = ', angle)
        sock_metadata.sendto(b'start', control_station_address_metadata)
        sock_metadata.sendto(bytearray(str(velocity).encode()), control_station_address_metadata)
        sock_metadata.sendto(bytearray(str(x_coordinate).encode()), control_station_address_metadata)
        sock_metadata.sendto(bytearray(str(y_coordinate).encode()), control_station_address_metadata)



    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

