import serial
import threading
import time
from tkinter import *


#port defention dont changed things beside port
ser = serial.Serial(
    port='/dev/rfcomm0', #this part is where you pu thr firefly port
    baudrate=115200,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

g_output = []

g_map_data = []


g_dict = {"command": 0xB2, "kd": 0xE2, "kp": 0xD2}

g_robot_x = 24
g_robot_y = 0
g_autonomous = False


def listener():
    global g_output
    while True:
        while not ser.in_waiting:
            pass

        out = []
        temp = ser.read()[0]
        if valid_header(temp):
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
            
        else:
            print("In valid header recived not printe to file: ", temp)
 
def uint16_to_int16(value):
    """
    convert from an unsigned 16 bit to signed bit 16
    """
    if (value > 32768):
        return value - 65536
    return value

def uint8_to_int8(value):
    """
    convert from an unsigned 8 bit to signed bit 8
    """
    if (value > 128):
        return value - 256
    return value

def consolOut():
    """
    this funtion reads the incomming data from the port that is saved to
    g_output. Each packet are a list in g_output where index 0 is the
    address and 1 is the byte count and index 2-6 is the byte.

    uint16_t are sent with lower part first.
    """
    global g_map_data
    while True:
        if g_output:
            nrOut = ""

            # debug
            if g_output[0][0] == 12:
                # pd paket
                if g_output[0][2] == 255:
                    if len(g_output[0]) == 7:
                        nrOut = "\nPropotional: " + \
                            str(uint16_to_int16(
                                g_output[0][4] << 8 | g_output[0][3]))
                        nrOut = nrOut + " \nDerivative: " + \
                            str(uint16_to_int16(
                                g_output[0][6] << 8 | g_output[0][5]))
                        #nrOut = nrOut + " \nCTE: " + str( float(g_output[0][8] << 8 | g_output[0][9]) )
                    else:
                        nrOut = "paket miss match: " + str(g_output[0])

                    # Navigation goal paket
                elif g_output[0][2] == 254:
                    if len(g_output[0]) == 7:
                        nrOut = "\nNavigationGoal X: " + \
                            str(g_output[0][4] << 8 | g_output[0][3])
                        nrOut = nrOut + " \nNavigationGoal Y: " + \
                            str(g_output[0][6] << 8 | g_output[0][5])
                    else:
                        nrOut = "paket miss match: " + str(g_output[0])

                    # reference palet
                elif g_output[0][2] == 253:
                    if len(g_output[0]) == 7:
                        nrOut = "\nReference Pos X: " + \
                            str(g_output[0][4] << 8 | g_output[0][3])
                        nrOut = nrOut + " \nReference Pos Y: " + \
                            str(g_output[0][6] << 8 | g_output[0][5])
                    else:
                        nrOut = "paket miss match: " + str(g_output[0])

                    # nav goal heading paket
                elif g_output[0][2] == 252:
                    if len(g_output[0]) == 5:
                        nrOut = nrOut + " \nNavigationGoalHeading: " + \
                            str(g_output[0][4] << 8 | g_output[0][3])
                    else:
                        nrOut = "paket miss match: " + str(g_output[0])

                    # debugs id followed by int 16
                elif 42 <= g_output[0][2] <= 46:
                    if len(g_output[0]) == 5:
                        nrOut = str(
                            g_output[0][2]) + " " + str(uint16_to_int16(g_output[0][4] << 8 | g_output[0][3]))
                    else:
                        nrOut = "paket miss match: " + str(g_output[0])

                    # break paket end of pd loop
                elif g_output[0][2] == 100:
                    nrOut = "\n"*10

                    # generic debug id + uint 16
                elif len(g_output[0]) == 5:
                    nrOut = "Debug: " + \
                        str(g_output[0][2]) + " " + \
                        str(g_output[0][4] << 8 | g_output[0][3])

                    # debug with unknow id and none generic
                else:
                    nrOut = "unknow debug: " + str(g_output[0])

            # lidar forward
            elif g_output[0][0] == 0:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "Lidar Forward: " + str(nrOut)
                else:
                    nrOut = "Lidar Forward paket miss match " + \
                        str(g_output[0])

            # lidar backwards
            elif g_output[0][0] == 1:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "Lidar Backwards: " + str(nrOut)
                else:
                    nrOut = "Lidar Backwards paket miss match " + \
                        str(g_output[0])

                # IR front left
            elif g_output[0][0] == 2:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Front Left: " + str(nrOut)
                else:
                    nrOut = "IR Front Left paket miss match " + \
                        str(g_output[0])

                # IR back left
            elif g_output[0][0] == 3:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Back Left: " + str(nrOut)
                else:
                    nrOut = "IR Back Left paket miss match " + str(g_output[0])

                # IR right front
            elif g_output[0][0] == 4:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Front Right: " + str(nrOut)
                else:
                    nrOut = "IR Front Right paket miss match " + \
                        str(g_output[0])

                # IR right back
            elif g_output[0][0] == 5:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Back Right: " + str(nrOut)
                else:
                    nrOut = "IR Back Right paket miss match " + \
                        str(g_output[0])

                # gyro
            elif g_output[0][0] == 6:
                if len(g_output[0]) == 4:
                    nrOut = uint16_to_int16(
                        g_output[0][3] << 8 | g_output[0][2])
                    nrOut = "Gyro: " + str(nrOut)
                else:
                    nrOut = "Gyro paket miss match " + str(g_output[0])

                # odometer
            elif g_output[0][0] == 7:
                if len(g_output[0]) == 4:
                    nrOut = "L: " + \
                        str(g_output[0][3]) + " R: " + str(g_output[0][2])
                    nrOut = "Odometer: " + str(nrOut)
                else:
                    nrOut = "Odometer paket miss match " + str(g_output[0])

                # position
            elif g_output[0][0] == 8:
                if len(g_output[0]) == 6:
                    nrOut = "\nPositionX: " + \
                        str(g_output[0][3] << 8 | g_output[0][2])
                    nrOut += "\nPositionY: " + \
                        str(g_output[0][5] << 8 | g_output[0][4])
                else:
                    nrOut = "Position paket miss match " + str(g_output[0])

                # direction
            elif g_output[0][0] == 9:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "Direction: " + str(nrOut)
                else:
                    nrOut = "Direction paket miss match " + str(g_output[0])

                # map update
            elif g_output[0][0] == 10:
                if len(g_output[0]) == 5:
                    nrOut = "Map update: " + str(g_output[0][2]) + " " + str(
                        g_output[0][3]) + " " + str(uint8_to_int8(g_output[0][4]))
                    g_map_data.append(
                        [(g_output[0][2]), (g_output[0][3]), uint8_to_int8(g_output[0][4])])
                else:
                    nrOut = "Map update paket miss match " + str(g_output[0])

                # last case should never happen
            else:
                nrOut = "Unknow paket we should never be here check code: " + \
                    str(g_output[0])

            #print("nrOut: ", nrOut)
            f = open("debug.txt", "a")
            f.write(nrOut +"\n")
            f.close()
            g_output.pop(0)
            time.sleep(0.01)



def mm_to_grid(x, y):
    return int(x / 400), int(y / 400)


def valid_header(header):
    """
    this function returns true if we recive a valid header
    last bit is 0
    4 last bits are a valid address
    3 first bit are a value of 0 to 7
    """
    return ( not( ( header ) & 0x01) ) and ( 0x0 <= ( ( header >> 4 ) & 0x0F ) <= 0xF ) and ( 0 <= ( ( header >> 1 ) & 0x07 ) <= 7)

def packageMaker(operation, byteList):
    """
    this function sends the byte to the fire fly
    """
    listToSend = [g_dict.get(operation)] + byteList
    print("list: ", listToSend)

    for byte in listToSend:
        package = bytearray()
        package.append(byte)
        ser.write(package)
        print("sent: ", package)
        time.sleep(0.1)
    time.sleep(0.1)

def main():
    """
    this function does everything in reagard to choosing what to send
    """
    #clear the debug file before run
    f = open("debug.txt", "w")
    f.write("")
    f.close()

    #start a lisner thread that read port pakets
    t1 = threading.Thread(target=listener)
    t1.start()

    t2 = threading.Thread(target=consolOut)
    
    print("Use graphic")
    print("0: No")
    print("1: Yes")
    val = int(input("Enter value: "))
    if  val == 1:
        t4 = threading.Thread(target=graphic)
        
    t3 = threading.Thread(target=menu)
    t3.start()
    if  val == 1:
        t4.start()

    t2.start()
    

     
def graphic():
    """
    this function start the the graphic map function
    """
    # graphics
    root = Tk()
    navMap = Map(root).grid(row=0, column=0, padx=5, pady=5)
    root.title("Gudrid Interface")
    root.mainloop()



class Constants:

    FRAME_WIDTH = 1445
    FRAME_HEIGHT = 1000
    DELAY = 1
    ONE_STEP = 20
    CELL_SIZE = 20
    PADDING = 10
    AUTO_SCROLL = True
    ROBOT_COLOR = "red"
    WALL_COLOR = "black"
    EMPTY_COLOR = "green"
    UNKNOWN_COLOR = "gray"



class Map(LabelFrame):
    
    """Graphical representation of the robot's movement and the room"""

    def __init__(self, parent):
        """Constructor"""
        LabelFrame.__init__(self, parent)
        self.parent = parent
        self.after(Constants.DELAY, self.onTimer)
        CELL_SIZE = 20
        SPACING = 4
        global g_robot_x
        global g_robot_y
        self.canvas = Canvas(self, width=CELL_SIZE * 49, height=CELL_SIZE * 25)
        for x in range(49):
            for y in range(25):

                # Create an "xy" tag for each rectangle on the map
                mirrored_x = 48 - x
                coord = (mirrored_x, y)
                stringTuple = tuple(map(str, coord))
                tag = stringTuple[0] + "," + stringTuple[1]

                self.canvas.create_rectangle(x * CELL_SIZE + SPACING, y * CELL_SIZE + SPACING,
                                             (x + 1)*CELL_SIZE, (y + 1)*CELL_SIZE, fill=Constants.UNKNOWN_COLOR, width=0, tags=tag)

        self.canvas.create_rectangle(g_robot_x * CELL_SIZE + SPACING, g_robot_y * CELL_SIZE + SPACING, 25 *
                                     CELL_SIZE, 1 * CELL_SIZE, fill=Constants.ROBOT_COLOR, tags='robot')
        self.canvas.pack(side=LEFT)

    def updateMap(self):
        """if g_map_update has been reciver from console out and update the grid"""
        global g_map_data
        if g_map_data:
            x = g_map_data[0][0]
            y = g_map_data[0][1]
            cell_type = g_map_data[0][2]
            cell = self.canvas.find_withtag(str(x) + "," + str(y))
            #print(len(g_map_data))
            #print("map debug: ", x, y, cell_type)

            # wall
            if cell_type < 20:
                self.canvas.itemconfig(cell, fill=Constants.WALL_COLOR)
            # empty
            elif cell_type > -20:
                self.canvas.itemconfig(cell, fill=Constants.EMPTY_COLOR)
            else:
                pass
            
            g_map_data.pop(0)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateMap()
        self.after(Constants.DELAY, self.onTimer)



def menu():
    """menu function that prints the menu so you dont need to rember stuff"""
    global g_dict
    while True:
        #you can do this part in one print but this also works
        print("-------------------------------------------------------")
        print("0: Stop")
        print("1: Start")
        print("2: Forward")
        print("3: Backwards")
        print("4: forward left")
        print("5: forward right")
        print("6: rotate left")
        print("7: rotate right")
        print("13: Kp")
        print("14: Kd")
        print("-------------------------------------------------------")

        val = int(input("Enter a value: "))
        if  0 <= val <= 7:
            threading.Thread(target=packageMaker,
                                args=("command", [ val ])).start()
        elif val == 13:
            kp = int(input("Enter kp: "))
            threading.Thread(target=packageMaker,
                                args=("kp", [kp])).start()                   
        elif val == 14:
            kd = int(input("Enter Kd: "))
            threading.Thread(target=packageMaker,
                                args=("kd", [kd])).start()
        
        time.sleep(0.5)

        
if __name__ == '__main__':
    main()