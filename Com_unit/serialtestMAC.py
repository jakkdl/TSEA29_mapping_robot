""" import serial

ser = serial.Serial(
    port='/dev/cu.Firefly-71B7-SPP',  # need to fin the correct port
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

while True:
    temp = ser.read().hex()
    print(temp) """

# 00056603/1B7

import time
import serial
import threading

ser = serial.Serial(
    port='/dev/tty.Firefly-71B7-SPP',  # need to fin the correct port
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)


def listener():
    """ This function is an early version of the
    constant listener funtion that will
    run in the background """

    while True:

        temp = ser.read()[0]
        print("TEMP: ", temp)
        header = ((temp >> 4) & 0xF0) | (
            (temp << 4) & 0x0F)

        print("HEADER: ", header)
        out = []
        addr = ((header >> 4) & 0x0F)
        count = ((header >> 1) & 0x07)

        out.append(addr)
        out.append(count)
        i = 0
        while(i < count):
            temp = ser.read()[0]
            package = ((temp >> 4) & 0xF0) | (
                (temp << 4) & 0x0F)
            out.append(package)
            i += 1
        print(out)


def write():
    data_bytes = bytes(b'\xB2\x01')
    while True:
        time.sleep(10)
        ser.write(data_bytes)


def main():
    t0 = threading.Thread(target=listener)
    t1 = threading.Thread(target=write)

    t0.start()
    t1.start()

    while True:
        pass


if __name__ == '__main__':
    main()
