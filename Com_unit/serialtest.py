import time
import serial
import threading

ser = serial.Serial(
    port='\dev\rfcomm0',  # need to fin the correct port
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)
if not ser.open():
    ser.open()


def lisener():
    """""
    This function is an early version of the 
    constan lisner funtion that will         
    run in the back ground
    """""
    while True:
        out = []
        temp = ser.read().hex()

        # bitshift down and mask away everythign that is not part of the 4bit addres
        addr = ((temp >> 4) & 0x0F)
        # bitshift down and mask away everythign that is not part of the 3bit byte count
        count = ((temp >> 1) & 0x07)

        out.append(temp)
        i = 0
        while(i < count):
            temp = ser.read().hex()
            out.append(temp)
            i += 1
        print(out)
    # return out


def write(send):
    data_bytes = bytes(send)
    ser.write(data_bytes)


def main():
    tl0 = threading.Thread(target=lisener)
    tl0.start

    while True:
        pass


if __name__ == '__main__':
    main()
