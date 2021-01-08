import numpy as np
import cv2 as cv
import os
import socket
import base64
import serial
import threading
import ObjectDetection
from multiprocessing import Process
import time
# socket_io = socketio.Client()
# socket_io.connect('http://192.168.1.5:8000')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
control_station_address = ('192.168.137.57', 2711)

# Create new socket to listen to key press from JavaFX
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pi_address = ('192.168.137.254', 2345)
# Bind socket2 to server
sock2.bind(pi_address)

def test_openCV():

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
        # frame_contours = ObjectDetection.getContours(frame)
        reval, buffer = cv.imencode('.jpeg', frame)
        encoded_string = base64.b64encode(buffer)
        # image = encoded_string.decode('utf-8')
        # socket_io.emit('image', image)
        temp = 0
        sock.sendto(b'start', control_station_address)
        while(len(encoded_string) - temp > 1024):
            sock.sendto(b'' + encoded_string[temp:temp+1024], control_station_address)
            temp = temp + 1024;
        # sock.sendto(frame, server_address);
        sock.sendto(b'' + encoded_string[temp:len(encoded_string)+1], control_station_address)
        sock.sendto(b'finished', control_station_address)
        # break;
        # cv.imshow('frame', grayImage)
        # i = i + 1
        # if i == 100: break

    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()


def receiveCommands():
    # Listen for key press
    print("start new process")
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    while True:
        data, address = sock2.recvfrom(1024)
        print(data.decode())
        ser.write(data)
        time.sleep(0.1)

def main():
    # thread1 = MyThread(1, "Thread-1")
    # thread1.start()
    # print("here")
    p = Process(target=receiveCommands, args=(''))
    p.start()
    # test_openCV()
    p.join()

if __name__ == "__main__":
    main()