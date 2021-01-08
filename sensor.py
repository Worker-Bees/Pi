import sys
import signal
import RPi.GPIO as GPIO
import time
import socket

#socket for sending metadata
sock_metadata = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_station_address = ('192.168.137.1', 3333)
ENCODER_1 = 27
ENCODER_2 = 23
encoder_1_pulses = 0
encoder_2_pulses = 0
velocity_1 = 0
velocity_2 = 0
def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def encoder_1_pulse_detection_handler(gpio_number):
    global encoder_1_pulses
    encoder_1_pulses =  encoder_1_pulses + 1

def encoder_2_pulse_detection_handler(gpio_number):
    global encoder_2_pulses
    encoder_2_pulses =  encoder_2_pulses + 1
if __name__ == '__main__':

    GPIO.setmode(GPIO.BCM)

    GPIO.setup(ENCODER_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_1, GPIO.FALLING,
                          callback=encoder_1_pulse_detection_handler)
    GPIO.setup(ENCODER_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(ENCODER_2, GPIO.FALLING,
                          callback=encoder_2_pulse_detection_handler)

    while True:
        time.sleep(0.1)
        # print(encoder_1_pulses)
        velocity_1 = 20.42 * (encoder_1_pulses / 347.0) * 10
        velocity_2 = 20.42 * (encoder_2_pulses / 347.0) * 10
        velocity = str(velocity_1 + (velocity_2 - velocity_1) / 2)
        print(velocity)
        encoder_1_pulses = 0
        encoder_2_pulses = 0
        sock_metadata.sendto(bytearray(velocity.encode()), control_station_address)


    signal.signal(signal.SIGINT, signal_handler)
    signal.pause()

