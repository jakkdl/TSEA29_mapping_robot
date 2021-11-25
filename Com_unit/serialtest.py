import time
import serial
import threading

ser = serial.Serial(
    port='\dev\rfcomm0', #need to fin the correct port
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
        header = ser.read()
        out = []
        addr = ( ( header >> 4 ) & 0x0F )
        count = ( ( header >> 1) & 0x07 )

        out.append( addr )
        out.append( count )
        for _ in range( count - 1 ):
            paket = ser.read()
            out.append( paket )
        else:
            print(out)

def Write():
    data_bytes = bytes(b'\xB2\x01')
    while True:
        time.sleep()
        ser.write( data_bytes )

tl0 = threading.Thread( target = lisener )
tl1 = threading.Thread( target = write)

t0.start
t1.start

while True:
    pass
