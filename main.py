import numpy as np
import cv2 as cv
import os
import socket
import base64
import threading
import ObjectDetection

# socket_io = socketio.Client()
# socket_io.connect('http://192.168.1.5:8000')
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('localhost', 2711)

# Create new socket to listen to key press from JavaFX
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address2 = ('localhost', 2345)
def test_openCV():
    # Bind socket2 to server
    sock2.bind(server_address2)

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
        frame_contours = ObjectDetection.getContours(frame)
        grayImage = cv.cvtColor(frame_contours, cv.COLOR_BGR2GRAY)
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


class MyThread (threading.Thread):
    def __init__(self, thread_id, name):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name

    def run(self):
        print("Starting " + self.name)
        # Listen for key press
        while True:
            data, address = sock2.recvfrom(1024)
            print("hello = ", data.decode())


def main():
    thread1 = MyThread(1, "Thread-1")
    thread1.start()
    print("here")
    test_openCV()


if __name__ == "__main__":
    main()