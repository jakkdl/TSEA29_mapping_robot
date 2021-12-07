from struct import pack
import serial
import threading
import time

from serial.serialutil import Timeout

#port defention dont changed things beside port
ser = serial.Serial(
    port='/dev/rfcomm0', #this part is where you pu thr firefly port
    baudrate=115200,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

"""time out timmer"""
g_timeout = 0.5

"""CHange here to adjust times"""
g_time_stop = 1 #time before stop is sent
g_time_sample = 1 #time before we print out g_output
g_time_delay = 0.5 #delay after the stop has ben sent to when we read all info in g_output

"""glboal static vars"""
g_output = [] #will contian all pakets in following format [addres, byte count, paket, ... , paket] up to 7 pakets
g_dict = {"command": 0xB2, "kd": 0xD2, "kp": 0xE2} #this has the header should not be change unless you know what you are doing

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

    while True:
        if g_output:
            nrOut = ""

            #debug
            if g_output[0][0] == 12:
                    #pd paket
                if g_output[0][3] == 0xFF:
                    nrOut = "/nPropotional: " + str(g_output[0][5] << 8 | g_output[0][4])
                    nrOut = nrOut + " /nDerivative Y: " + str(g_output[0][7] << 8 | g_output[0][6])
                    nrOut = nrOut + " /nCTE: " + str( float(g_output[0][8] << 8 | g_output[0][9]) )
                    #postison paket
                elif g_output[0][3] == 0xFE:
                    nrOut = "/nPospaket X: " + str(g_output[0][5] << 8 | g_output[0][4])
                    nrOut = nrOut + " /nPospaket Y: " + str(g_output[0][7] << 8 | g_output[0][6])
                    #Navigation goal paket
                elif g_output[0][3] == 0xFD:
                    nrOut = "/nNavigationGoal X: " + str(g_output[0][5] << 8 | g_output[0][4])
                    nrOut = nrOut + " /nNavigationGoal Y: " + str(g_output[0][7] << 8 | g_output[0][6])
                    #reference palet
                elif g_output[0][3] == 0xFC:
                    nrOut = "/nReference Pos X: " + str(g_output[0][5] << 8 | g_output[0][4])
                    nrOut = nrOut + " /nReference PosX Y: " + str(g_output[0][7] << 8 | g_output[0][6])
                elif g_output[0][3] == 0xFB:
                    nrOut = "/nCurrentHeading: " + str(g_output[0][5] << 8 | g_output[0][4])
                    nrOut = nrOut + " /nNavigationGoalHeading: " + str(g_output[0][7] << 8 | g_output[0][6])
                elif len(g_output[0]) == 5:
                    nrOut = str(g_output[0][2]) + " " + str(g_output[0][4] << 8 | g_output[0][3])
                else:
                    nrOut = str(g_output[0])
                nrOut = "DEBUG: /n" + nrOut

            #lidar forward
            elif g_output[0][0] == 0:
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
                nrOut = "L: " + str(g_output[0][3]) + " R: " + str(g_output[0][2])
                nrOut = "Odometer: " + str(nrOut)

                # position
            elif g_output[0][0] == 8:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Position: " + str(nrOut)

                # direction
            elif g_output[0][0] == 9:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Direction: " + str(nrOut)

                # map update
            elif g_output[0][0] == 10:
                    nrOut = "Map update: " + str(g_output[0][2]) + " " + str(g_output[0][3]) + " " + str( g_output[0][4] )

            else:
                nrOut = "Unknow paket: " + str(g_output[0])
            
            #add the current nrout which is our output to debug file
            f = open("debug.txt", "a")
            f.write(nrOut + "\n")
            f.close()
            g_output.pop(0)

def packageMaker(operation, byteList):
    """
    this function sends the byte to the fire fly
    """
    print("in sender")
    global g_dict

    listToSend = [g_dict.get(operation)] + byteList
    print("list: ", listToSend)

    for byte in listToSend:
        package = bytearray()
        package.append(byte)
        ser.write(package)
        print("sent: ", package)
        time.sleep(0.1)
    time.sleep(1)

def main():
    """
    this function does everything in reagard to choosing what to send
    """
    #clear the debug file before run
    f = open("debug.txt", "w")
    f.write("")
    f.close()

    global g_output
    global g_time
    global g_timeout
    #start a lisner thread that read port pakets

    t1 = threading.Thread(target=listener)
    t1.start()

    t2 = threading.Thread(target=consolOut)
    t2.start()

    while True:
        
        print("-------------------------------------------------------")
        val = int(input("Enter a value: "))
        if val == 0:
            threading.Thread(target=packageMaker,
                                args=("command", [0])).start()
        elif val == 1:
            threading.Thread(target=packageMaker,
                                args=("command", [1])).start()

        elif val == 2:
            threading.Thread(target=packageMaker,
                                args=("command", [2])).start()

        elif val == 3:
            threading.Thread(target=packageMaker,
                                args=("command", [3])).start()

        elif val == 4:
            threading.Thread(target=packageMaker,
                                args=("command", [4])).start()

        elif val == 5:
            threading.Thread(target=packageMaker,
                                args=("command", [5])).start()
                                
        elif val == 6:
            threading.Thread(target=packageMaker,
                                args=("command", [6])).start()
        elif val == 10:      
            g_timeout = 30
            consolOut()
            g_timeout = 0.5
    
        elif val == 13:
            kp = input("Enter kp")
            threading.Thread(target=packageMaker,
                                args=("Kp", [kp])).start()
                                
        elif val == 14:
            kd = input("Enter Kd")
            threading.Thread(target=packageMaker,
                                args=("Kd", [Kd])).start()
        
        time.sleep(g_time_delay)
        

        

if __name__ == '__main__':
    main()