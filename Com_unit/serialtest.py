import time
import serial

ser = serial.Serial(
    port='', #need to fin the correct port
    baudrate=9600,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

if ser.isOpen():
    ser.close()
ser.open()
ser.isOpen()

data_bytes = bytes(b'\xCE\xFF\x00\x11\x22\x33\x00\xFF')
ser.write( data_bytes )
out = ''
# wait 3s
time.sleep(3)
print('--------------------')
while ser.inWaiting() > 0:
    out += ser.read(40)

if out != '':
    print(">>" + out)
print('--------------------')

ser.close()
