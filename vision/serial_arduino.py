#coding=utf-8
import serial
import cv2

def serial_init(port):
    serial_ard = serial.Serial(port, 9600, timeout=0.01)
    serial_ard.flushInput()

    return serial_ard

def serial_send(serial_ard, message):
    message_ascii = ('&' + message + '#').encode('utf-8')
    try:
        serial_ard.write(message_ascii)
        print("send:", message_ascii)
    except:
        print("Failed to send.")
        serial_ard.close()


def serial_read(serial_ard):
    serial_message = serial_ard.readline()
    # serial_ard.flushInput()

    if len(serial_message):
        print("read: ", serial_message.decode('utf-8'))
        return True
    else:
        return False

if __name__ == "__main__":
    serial_ard = serial.Serial('/dev/ttyACM0', 9600, timeout=0.5)
    serial_ard.flushInput()

    while 1:
        read = serial_ard.readline().decode('utf-8')
        print(read)