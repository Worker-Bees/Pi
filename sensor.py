import sys
import signal
import RPi.GPIO as GPIO
import time
import socket

#socket for sending metadata
sock_metadata = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_station_address = ('192.168.137.1', 3333)
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
import adafruit_lsm303dlh_mag
import math

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
x_offset, y_offset, _ = sensor.magnetic

def vector_2_degrees(x, y):
    # az = 90 - atan(y / x) * 180 / pi
    angle = degrees (atan2(y, x))
    if angle < 0:
        angle += 360
    return degrees(atan2(y,x))


def get_heading(_sensor):
    magnet_x, magnet_y, _ = _sensor.magnetic
    return vector_2_degrees(magnet_x, magnet_y)

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def encoder_1_pulse_detection_handler(gpio_number):
    global encoder_1_pulses
    encoder_1_pulses =  encoder_1_pulses + 1

def encoder_2_pulse_detection_handler(gpio_number):
    global encoder_2_pulses
    encoder_2_pulses =  encoder_2_pulses + 1

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
    offset = get_heading(sensor)
    distance = 0
    while True:
        time.sleep(0.1)
        # print(encoder_1_pulses)
        velocity_1 = 20.42 * (encoder_1_pulses / 374.0) * 10
        velocity_2 = 20.42 * (encoder_2_pulses / 374.0) * 10
        # distance = distance + abs(encoder_1_pulses * 20.42 / 374.0)
        distance = distance + abs((encoder_1_pulses * 20.42 / 374) + (encoder_1_pulses * 20.42 / 374  - encoder_2_pulses * 20.42 / 374) / 2)
        velocity = str(velocity_1 + (velocity_2 - velocity_1) / 2)
        heading = get_heading(sensor) - offset
        x = math.cos(math.radians(heading)) * distance
        y = math.sin(math.radians(heading)) * distance
        # print(encoder_1_pulses, ' and ', encoder_2_pulses)
        print("heading ", heading)
        # print("x = ", distance, " ;  y = ", y)
        # print("heading: {:.2f} degrees".format(get_heading(sensor)))
        # print("velocity: ", velocity, " cm/s")
        sock_metadata.sendto(bytearray(velocity.encode()), control_station_address)
        encoder_1_pulses = 0
        encoder_2_pulses = 0


    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

