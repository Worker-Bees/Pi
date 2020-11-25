import numpy as np
import cv2 as cv
import os
import socketio
import socket
import base64
import time

# socket_io = socketio.Client()
# socket_io.connect('http://192.168.1.5:8000')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('192.168.1.3', 2711)
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
        grayImage = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        reval, buffer = cv.imencode('.jpeg', grayImage)
        encoded_string = base64.b64encode(buffer)
        # image = encoded_string.decode('utf-8')
        # socket_io.emit('image', image)
        temp = 0
        sock.sendto(b'start', server_address)
        while(len(encoded_string) - temp > 1024):
            sock.sendto(b'' + encoded_string[temp:temp+1024], server_address)
            temp = temp + 1024;
        # sock.sendto(frame, server_address);
        sock.sendto(b'' + encoded_string[temp:len(encoded_string)+1], server_address)
        sock.sendto(b'finished', server_address)
        # break;
        # cv.imshow('frame', grayImage)
        # i = i + 1
        # if i == 100: break
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

def main():
    test_openCV()

if __name__ == "__main__":
    main()