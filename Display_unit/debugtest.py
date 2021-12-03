import serial
import threading
import time

ser = serial.Serial(
    port='/dev/rfcomm0',
    baudrate=115200,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

g_output = []
g_dict = {"command": 0xB2, "kd": 0xD2, "kp": 0xE2}

def listener():
    global g_output
    while True:
        while not ser.in_waiting:
            pass

        out = []
        temp = ser.read()[0]

        # bitshift right and mask away everything that is not part of the 4-bit address
        addr = ((temp >> 4) & 0x0F)
        # bitshift right and mask away everything that is not part of the 3-bit byte count
        count = ((temp >> 1) & 0x07)

        # out.append(temp)
        out.append(addr)
        out.append(count)
        i = 0
        while(i < count):
            temp = ser.read()
            if len(temp):
                result = temp[0]
            else:
                break
            out.append(result)
            i += 1
        g_output.append(out)

def consolOut():
    """
    this funtion reads the incomming data from the port that is saved to
    g_output. Each packet are a list in g_output where index 0 is the
    address and 1 is the byte count and index 2-6 is the byte.

    uint16_t are sent with lower part first.
    """
    global g_output
    while g_output:
        nrOut = ""
        #debug
        if g_output[0][0] == 12:
            if len(g_output) == 4:
                nrOut = str(g_output[0][2]) + " " + str(g_output[0][3] << 8 | g_output[0][4])
            else:
                nrOut = str(g_output[0])
            nrOut = "DEBUG: " + nrOut
        #lidar forward
        if g_output[0][0] == 0:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "Lidar Forward: " + str(nrOut)
            
        # lidar backwards
        elif g_output[0][0] == 1:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "Lidar Backwards: " + str(nrOut)

            # IR front left
        elif g_output[0][0] == 2:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "IR Front Left: " + str(nrOut)

            # IR back left
        elif g_output[0][0] == 3:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "IR Back Left: " + str(nrOut)

            # IR right front
        elif g_output[0][0] == 4:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "IR Front Right: " + str(nrOut)

            # IR right back
        elif g_output[0][0] == 5:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "IR Back Right: " + str(nrOut)

            # gyro
        elif g_output[0][0] == 6:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "Gyro: " + str(nrOut)

            # odometer
        elif g_output[0][0] == 7:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "Odometer: " + str(nrOut)

            # direction
        elif g_output[0][0] == 9:
            nrOut = g_output[0][3] << 8 | g_output[0][2]
            nrOut = "Direction: " + str(nrOut)
        else:
            nrOut = "Unknow address: " + str(g_output[0])
        
        #add the current nrout which is our output to debug file
        f = open("debug.txt", "w")
        f.write(nrOut)
        f.close()

        #remove the current list from the g_output
        g_output.pop(0)

def packageMaker(operation, byteList):
    global g_dict

    listToSend = [g_dict.get(operation)] + byteList

    for byte in listToSend:
        package = bytearray()
        package.append(byte)
        ser.write(package)
        time.sleep(0.1)
    time.sleep(1)

def main():
    #start a lisner thread that read port pakets
    t1 = threading.Thread(target=listener)
    t1.start()

    """
    #starts a read thread that adds the paket to a file and console
    t2 = threading.Thread(target=consolOut)
    t2.start()
    """
    

    while True:
        #val = input("Enter your paket to send 0-6 (sends stop after 3s): ")
        if val == 0:
            threading.Thread(target=packageMaker,
                                args=("command", [0])).start()

        elif val == 1:
            threading.Thread(target=packageMaker,
                                args=("command", [3])).start()

        elif val == 2:
            threading.Thread(target=packageMaker,
                                args=("command", [4])).start()

        elif val == 3:
            threading.Thread(target=packageMaker,
                                args=("command", [5])).start()
        elif val == 4:
            threading.Thread(target=packageMaker,
                                args=("command", [3])).start()

        elif val == 5:
            threading.Thread(target=packageMaker,
                                args=("command", [4])).start()

        elif val == 6:
            threading.Thread(target=packageMaker,
                                args=("command", [5])).start()
        
        time.sleep(2)
        threading.Thread(target=packageMaker, args=("command", [0])).start() 

if __name__ == '__main__':
    main()