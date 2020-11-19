import numpy as np
import cv2 as cv
import os
import socketio
import base64
import time

socket_io = socketio.Client()
socket_io.connect('http://192.168.1.6:8000')

os.environ['DISPLAY'] = ':0'
def test_openCV():

    cap = cv.VideoCapture(0)
    cap.set(3, 480)
    cap.set(4, 360)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
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
        image = encoded_string.decode('utf-8')
        socket_io.emit('image', image)
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

def main():
    print("Hello World!")
    test_openCV()

if __name__ == "__main__":
    main()