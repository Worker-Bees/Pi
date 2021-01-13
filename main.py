import numpy as np
import cv2 as cv
import os
import sensor
import socket
import base64
import serial
import threading
import ObjectDetection
import RPi.GPIO as GPIO
from multiprocessing import Process, Queue
import time
# socket_io = socketio.Client()
# socket_io.connect('http://192.168.1.5:8000')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_station_address = ('192.168.137.1', 2711)



# Create new socket to listen to key press from JavaFX
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pi_address = ('192.168.137.254', 2345)
# Bind socket2 to server
sock2.bind(pi_address)

def live_stream(metadata_queue):
    detection_mode = False
    cap = cv.VideoCapture(0)
    # cap.set(3, 480)
    # cap.set(4, 360)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    i = 0
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        if cv.waitKey(1) == ord('q'):
            break

        if not metadata_queue.empty():
            detection_mode = metadata_queue.get_nowait()

        if detection_mode is True:
            frame_contours = ObjectDetection.getContours(frame)
            reval, buffer = cv.imencode('.jpeg', frame_contours)
        else:
            reval, buffer = cv.imencode('.jpeg', frame)

        encoded_string = base64.b64encode(buffer)
        temp = 0
        sock.sendto(b'start', control_station_address)
        while(len(encoded_string) - temp > 1024):
            sock.sendto(b'' + encoded_string[temp:temp+1024], control_station_address)
            temp = temp + 1024;
        sock.sendto(b'' + encoded_string[temp:len(encoded_string)+1], control_station_address)
        sock.sendto(b'finished', control_station_address)
        time.sleep(0.03)
        # break;
        # cv.imshow('frame', grayImage)
        # i = i + 1
        # if i == 100: break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


def receiveCommands():
    # Listen for key press
    print("Start manual control process")
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    # GPIO.setmode(GPIO.BOARD)
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.LOW)
    ser.flush()
    while True:
        ser.flush()
        data, address = sock2.recvfrom(10)
        if len(data) > 1:
            GPIO.output(17, GPIO.HIGH)
            time.sleep(0.01)
            GPIO.output(17, GPIO.LOW)
            ser.write(b'x');
            if (data == b'auto'): ser.write(b'm');
        else: ser.write(data)


def main():
    manual_control_process = Process(target=receiveCommands, args=(''))
    manual_control_process.start()
    metadata_queue = Queue()
    sending_metadata_process = Process(target=sensor.send_metadata, args=(metadata_queue,))
    sending_metadata_process.start()
    live_stream(metadata_queue)
    manual_control_process.join()
    sending_metadata_process.join()



if __name__ == "__main__":
    main()



