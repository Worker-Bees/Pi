from time import sleep
import serial
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

while True:

    received_data = ser.read();
    sleep(0.3)
    data_left = ser.inWaiting()
    received_data += ser.read(data_left)
    print (received_data.decode())
    ser.write("hehe")