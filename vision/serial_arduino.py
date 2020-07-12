#coding=utf-8
import serial
import cv2

def serial_init(port):
    serial_ard = serial.Serial(port, 9600, timeout=1)
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
    serial_ard.flushInput()


def serial_read(serial_ard):
    serial_message = serial_ard.read(5)
    if len(serial_message):
        print("read", serial_message.decode('utf-8'))
        return 1
    else:
        return 0

if __name__ == "__main__":
    ser = serial_init('/dev/ttyACM0')
    # cv2.namedWindow("1")
    # cv2.waitKey(1000)
    while 1:
        read = ser.read(10).decode('utf-8')
        print(read)
        # serial_send(ser, 'the')
        # key = cv2.waitKey(1)
        # if key == 27:
        #     break
    # cv2.destroyAllWindows()