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
control_station_address = ('10.247.169.53', 2711)



# Create new socket to listen to key press from JavaFX
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pi_address = ('10.247.169.49', 2345)
# Bind socket2 to server
sock2.bind(pi_address)

def live_stream(mode_queue):
    detection_mode = True
    cap = cv.VideoCapture(0)
    cap.set(10, 150)
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

        if not mode_queue.empty():
            detection_mode = mode_queue.get_nowait()

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
        time.sleep(0.01)
        # break;
        # cv.imshow('frame', grayImage)
        # i = i + 1
        # if i == 100: break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


def receiveCommands(location_queue, mode_queue):
    # Listen for key press
    print("Start commanding process")
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    # GPIO.setmode(GPIO.BOARD)
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.LOW)
    ser.flush()
    manual_mode = False
    sock2.setblocking(False)
    data = b''
    # trigger manual mode for testing, remomve later
    ##--------------------------
    while True:
        data = b'/'
        # try:
        #     print(ser.read())
        # except UnicodeDecodeError:
        #     pass

        try:
            data, address = sock2.recvfrom(10)
        except IOError:
            pass

        if manual_mode == False and not location_queue.empty():
            location_info = location_queue.get_nowait()
            if location_info == "GATE_ZONE":
                change_mode()
            elif location_info == "PALLET_ZONE":
                change_mode()
                mode_queue.put(True)

        if data == b'manual':
            manual_mode = True
            mode_queue.put(False)
            change_mode()

        elif manual_mode == True and len(data) == 1 and data != b'/':
            print(data)
            ser.write(data)

def change_mode():
    GPIO.output(17, GPIO.HIGH)
    time.sleep(0.01)
    GPIO.output(17, GPIO.LOW)

def main():
    # print("gere")
    mode_queue = Queue()
    location_queue = Queue()
    manual_control_process = Process(target=receiveCommands, args=(location_queue, mode_queue, ))
    manual_control_process.start()
    sending_metadata_process = Process(target=sensor.send_metadata, args=(location_queue,))
    sending_metadata_process.start()
    live_stream(mode_queue)
    manual_control_process.join()
    sending_metadata_process.join()



if __name__ == "__main__":
    main()



