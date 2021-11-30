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


def lisener():
    """""
    This function is an early version of the 
    constan lisner funtion that will         
    run in the back ground
    """""
    while True:
        temp = ser.read().hex()
        header = ((temp >> 4) & 0xF0) | ((temp << 4) & 0x0F)
        out = []
        addr = ((header >> 4) & 0x0F)
        count = ((header >> 1) & 0x07)

        out.append(addr)
        out.append(count)
        i = 0
        while(i < count):
            temp = ser.read().hex()
            paket = ((temp >> 4) & 0xF0) | ((temp << 4) & 0x0F)
            out.append(paket)
            i += 1
        print(out)


def Write():
    data_bytes = bytes(b'\xB2\x01')
    while True:
        time.sleep()
        ser.write(data_bytes)


def main():
    tl0 = threading.Thread(target=lisener)
    tl1 = threading.Thread(target=write)

    t0.start
    t1.start

    while True:
        pass


if __name__ == '__main__':
    main()
