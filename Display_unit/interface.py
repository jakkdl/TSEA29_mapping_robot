from struct import pack
from tkinter import *
import serial
import threading
import time

"""port needs to be changed depending on which computer you are using"""
ser = serial.Serial(
    port='/dev/rfcomm0', #this port should be changed to your own port
    baudrate=115200,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
) 

# ser = serial.Serial(
# #     port='/dev/tty.Firefly-71B7-SPP',  # this port should be changed to your own port
# #     baudrate=115200,
# #     parity=serial.PARITY_EVEN,
# #     stopbits=serial.STOPBITS_ONE,
# #     bytesize=serial.EIGHTBITS,
# #     timeout=1
# # )

g_output = []

g_sensor_data = []
g_map_data = []

g_pos_data = []
g_currentpos_data = []

g_direction = []

g_dict = {"command": 0xB2, "kd": 0xE2, "kp": 0xD2}

g_robot_x = 24
g_robot_y = 0
g_autonomous = False


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
            print(len(g_map_data))
            print("map debug: ", x, y, cell_type)

            # wall
            if cell_type < 20:
                self.canvas.itemconfig(cell, fill=Constants.WALL_COLOR)
            # empty
            elif cell_type > -20:
                self.canvas.itemconfig(cell, fill=Constants.EMPTY_COLOR)
            else:
                pass
            
            g_map_data.pop(0)

    def moveRobot(self):
        """animates the robot's movement"""

        global g_currentpos_data
        global g_robot_x
        global g_robot_y
        cellsize = 20
        spacing = 4

        if g_currentpos_data:
            x = g_currentpos_data[0][0]
            y = g_currentpos_data[0][1]
            #print("x: ", x)
            #print("y: ", y)
            robot = self.canvas.find_withtag('robot')
            self.canvas.move(robot, -(x - g_robot_x)*cellsize,
                             (y - g_robot_y)*cellsize)
            g_robot_x = x
            g_robot_y = y
            g_currentpos_data.pop(0)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.moveRobot()
        self.updateMap()
        self.after(Constants.DELAY, self.onTimer)


class Console(LabelFrame):

    """Console for displaying sensor data and other relevant information"""

    def __init__(self, parent):
        """Constructor"""
        LabelFrame.__init__(self, parent, bg='black')
        self.parent = parent
        self.after(Constants.DELAY, self.onTimer)
        self.index = 0
        self.canvas = Canvas(self, bg='black', width=970, height=306)
        self.frame = Frame(self.canvas, bg='black')
        self.scrollBar = Scrollbar(self)
        self.createScrollableContainer()

    def updateScollRegion(self):
        self.canvas.update_idletasks()
        self.canvas.config(scrollregion=self.frame.bbox())

    def createScrollableContainer(self):
        self.canvas.config(yscrollcommand=self.scrollBar.set,
                           highlightthickness=0)
        self.scrollBar.config(orient=VERTICAL, command=self.canvas.yview)

        self.scrollBar.pack(fill=Y, side=RIGHT, expand=FALSE)
        self.canvas.pack(fill=BOTH, side=LEFT, expand=TRUE)
        self.canvas.create_window(0, 0, window=self.frame, anchor=NW)

    def updateConsole(self):
        '''updates the console'''
        global g_sensor_data
        if g_sensor_data:
            nrOut = g_sensor_data[0]
            g_sensor_data.pop(0)
            self.index += 1
            if self.index % 100 == 0:
                print("----- DESTROYING CONSOLE CONTENT -----")
                for widget in self.frame.winfo_children():
                    widget.destroy()
            Label(self.frame, text=nrOut,
                  bg='black', fg='white').grid(row=self.index, sticky=W)
            self.updateScollRegion()
        if Constants.AUTO_SCROLL:
            self.canvas.yview_moveto(1)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateConsole()
        self.after(Constants.DELAY, self.onTimer)


class Controls(LabelFrame):

    """Keyboard controls"""

    def __init__(self, parent):
        """Constructor"""
        LabelFrame.__init__(self, parent)
        self.parent = parent
        self.bind_all("<Key>", self.onKeyPressed)
        self.bind_all("<KeyRelease>", self.onKeyReleased)
        CELL_SIZE = 20
        self.canvas = Canvas(self, bg='gray', width=420,
                             height=CELL_SIZE * 25 - 1)
        self.canvas.pack(side=LEFT)

        rightArrow = [270, 150, 270, 250, 320, 200]
        leftArrow = [150, 150, 150, 250, 100, 200]
        upArrow = [210, 90, 260, 140, 160, 140]
        downArrow = [210, 310, 260, 260, 160, 260]

        self.canvas.create_polygon(rightArrow, tags="right_arrow")
        self.canvas.create_polygon(leftArrow, tags="left_arrow")
        self.canvas.create_polygon(upArrow, tags="up_arrow")
        self.canvas.create_polygon(downArrow, tags="down_arrow")

        self.modeButton = Button(self, text="MODE", width=10,
                                 command=self.setNavigationMode, state=NORMAL, highlightbackground='gray')
        self.modeButton_window = self.canvas.create_window(
            210, 350, anchor=S, window=self.modeButton)

        self.stopButton = Button(self, text="STOP", width=10,
                                 command=self.stopRobot, state=NORMAL, highlightbackground='gray')
        self.stopButton_window = self.canvas.create_window(
            210, 210, anchor=S, window=self.stopButton)

        self.canvas.create_text(90,
                                385, font=("Purisa", 20), text="KD: ", )
        self.canvas.create_text(90,
                                415, font=("Purisa", 20), text="KP: ", )

        self.inputKd = Entry(self, highlightbackground='gray')
        self.inputKd_window = self.canvas.create_window(
            210, 400, anchor=S, window=self.inputKd)

        self.inputKp = Entry(self, highlightbackground='gray')
        self.inputKp_window = self.canvas.create_window(
            210, 430, anchor=S, window=self.inputKp)

        self.pdButton = Button(self, text="SET PD", width=10,
                               command=self.setPd, state=NORMAL, highlightbackground='gray')
        self.pdButton_window = self.canvas.create_window(
            210, 470, anchor=S, window=self.pdButton)

    def onKeyPressed(self, e):
        '''controls direction variables with cursor keys'''
        global g_autonomous
        key = e.keysym
        if not g_autonomous:

            LEFT_CURSOR_KEY = "Left"
            if key == LEFT_CURSOR_KEY:
                arrow = self.canvas.find_withtag("left_arrow")
                self.canvas.itemconfig(arrow, fill='green')
                threading.Thread(target=packageMaker,
                                 args=("command", [4])).start()
                print("Rotate left")

            RIGHT_CURSOR_KEY = "Right"
            if key == RIGHT_CURSOR_KEY:
                arrow = self.canvas.find_withtag("right_arrow")
                self.canvas.itemconfig(arrow, fill='green')
                threading.Thread(target=packageMaker,
                                 args=("command", [5])).start()
                print("Rotate right")

            UP_CURSOR_KEY = "Up"
            if key == UP_CURSOR_KEY:
                arrow = self.canvas.find_withtag("up_arrow")
                self.canvas.itemconfig(arrow, fill='green')
                threading.Thread(target=packageMaker,
                                 args=("command", [2])).start()
                print("Go forward")

            DOWN_CURSOR_KEY = "Down"
            if key == DOWN_CURSOR_KEY:
                arrow = self.canvas.find_withtag("down_arrow")
                self.canvas.itemconfig(arrow, fill='green')
                threading.Thread(target=packageMaker,
                                 args=("command", [3])).start()
                print("Go backwards")

        # Pauses the autoscroll in the console
        SPACE = "space"
        if key == SPACE:
            Constants.AUTO_SCROLL = not Constants.AUTO_SCROLL

    def onKeyReleased(self, e):
        global g_autonomous
        key = e.keysym
        if not g_autonomous:

            LEFT_CURSOR_KEY = "Left"
            if key == LEFT_CURSOR_KEY:
                arrow = self.canvas.find_withtag("left_arrow")
                self.canvas.itemconfig(arrow, fill='black')
                print("Rotate left")

            RIGHT_CURSOR_KEY = "Right"
            if key == RIGHT_CURSOR_KEY:
                arrow = self.canvas.find_withtag("right_arrow")
                self.canvas.itemconfig(arrow, fill='black')
                print("Rotate right")

            UP_CURSOR_KEY = "Up"
            if key == UP_CURSOR_KEY:
                arrow = self.canvas.find_withtag("up_arrow")
                self.canvas.itemconfig(arrow, fill='black')
                print("Go forward")

            DOWN_CURSOR_KEY = "Down"
            if key == DOWN_CURSOR_KEY:
                arrow = self.canvas.find_withtag("down_arrow")
                self.canvas.itemconfig(arrow, fill='black')
                print("Go backwards")

    def setNavigationMode(self):
        print("SETTING NAVIGATION MODE")
        global g_dict
        global g_autonomous
        if g_autonomous:
            packageMaker("command", [0])
            g_autonomous = False
        else:
            packageMaker("command", [1])
            g_autonomous = True

    def setPd(self):
        """Function called when pressing SET PD"""
        kd = self.inputKd.get()
        kp = self.inputKp.get()

        threading.Thread(target=packageMaker, args=("kd", [int(kd)])).start()
        threading.Thread(target=packageMaker, args=("kp", [int(kp)])).start()

    def stopRobot(self):
        threading.Thread(target=packageMaker, args=("command", [0])).start()


class Information(LabelFrame):

    """Section for displaying some general info about the robot's current state"""

    def __init__(self, parent):
        """Constructor"""
        LabelFrame.__init__(self, parent)
        self.parent = parent
        self.after(Constants.DELAY, self.onTimer)
        self.canvas = Canvas(self, bg='gray', width=420, height=299)
        self.canvas.create_text(Constants.PADDING,
                                Constants.PADDING, font=("Purisa", 20), text="Position: ", anchor=NW, tags="position_text")
        self.canvas.create_text(Constants.PADDING,
                                Constants.PADDING * 4, font=("Purisa", 20), text="Direction: ", anchor=NW, tags="direction_text")
        self.canvas.create_text(Constants.PADDING, Constants.PADDING * 7,
                                font=("Purisa", 20), text="Mode: ", anchor=NW, tags="mode_text")
        self.canvas.pack(side=LEFT)

    def updateInformation(self):
        """updates the console"""
        global g_autonomous
        global g_currentpos_data
        mode = self.canvas.find_withtag("mode_text")
        pos = self.canvas.find_withtag("position_text")
        dir = self.canvas.find_withtag("direction_text")
        if g_currentpos_data:
            x, y = mm_to_grid(g_currentpos_data[0][0], g_currentpos_data[0][1])
        direction = ""

        if g_direction:
            if g_direction[0] == 0:
                direction = "RIGHT"
            elif g_direction[0] == 1:
                direction = "UP"
            elif g_direction[0] == 2:
                direction = "LEFT"
            elif g_direction[0] == 3:
                direction = "DOWN"

        if g_currentpos_data:
            g_currentpos_data.pop(0)
        if g_direction:
            g_direction.pop(0)

        self.canvas.itemconfig(pos, text="Position: " + str(x) + ", " + str(y))
        self.canvas.itemconfig(dir, text="Direction: " + direction)

        if(g_autonomous):
            self.canvas.itemconfig(mode, text="Mode: AUTONOMOUS")
        else:
            self.canvas.itemconfig(mode, text="Mode: MANUAL")


    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateInformation()
        self.after(Constants.DELAY, self.onTimer)


def valid_header(header):
    """
    this function returns true if we recive a valid header
    last bit is 0
    4 last bits are a valid address
    3 first bit are a value of 0 to 7
    """
    return (not((header) & 0x01)) and (0x0 <= ((header >> 4) & 0x0F) <= 0xF) and (0 <= ((header >> 1) & 0x07) <= 7)


def listener():
    print("----- LISTENING FOR BLUETOOTH INPUT -----")
    global g_output

    while True:
        while not ser.in_waiting:
            pass

        out = []
        header = ser.read()[0]
        if valid_header(header):
            # bitshift right and mask away everything that is not part of the 4-bit address
            addr = ((header >> 4) & 0x0F)
            # bitshift right and mask away everything that is not part of the 3-bit byte count
            count = ((header >> 1) & 0x07)

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
            g_output.append(out.copy())
            #packet_parser()
        else:
            print("In valid header recived not printe to file: ", header)


def packageMaker(operation, byteList):

    global g_dict

    listToSend = [g_dict.get(operation)] + byteList

    for byte in listToSend:
        package = bytearray()
        package.append(byte)
        ser.write(package)
        print("Package: ", package)
        time.sleep(0.1)
    time.sleep(1)


def packet_parser():
    """
    this funtion reads the incomming data from the port that is saved to
    g_output. Each packet are a list in g_output where index 0 is the
    address and 1 is the byte count and index 2-6 is the byte.

    uint16_t are sent with lower part first.
    """
    global g_output
    global g_map_data
    global g_sensor_data
    global g_pos_data
    global g_currentpos_data
    global g_direction

    global g_map_used_date

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
                    g_pos_data.append(
                        [(g_output[0][3] << 8 | g_output[0][2]), (g_output[0][5] << 8 | g_output[0][4])])
                    x, y = mm_to_grid(
                        (g_output[0][3] << 8 | g_output[0][2]), g_output[0][5] << 8 | g_output[0][4])
                    g_currentpos_data.append([x, y])
                else:
                    nrOut = "Position paket miss match " + str(g_output[0])

                # direction
            elif g_output[0][0] == 9:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "Direction: " + str(nrOut)
                    dir = g_output[0][3] >> 5
                    g_direction.append(dir)
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

            g_sensor_data.append(nrOut)
            #print("nrOut: ", nrOut)
            g_output.pop(0)
            time.sleep(0.01)


def uint16_to_int16(value):
    """
    convert from an unsigned 16 bit to signed bit 16
    """
    if (value > 32767):
        return value - 65534
    return value


def uint8_to_int8(value):
    """
    convert from an unsigned 8 bit to signed bit 8
    """
    if (value > 127):
        return value - 254
    return value


def mm_to_grid(x, y):
    """
    convert x and y from mm to grid square cardinal cords
    """
    return int(x / 400), int(y/400)


def main():
    # thread for incoming bluetooth stream
    t1 = threading.Thread(target=listener)
    t1.start()

    # thread for reading all incomming data
    t2 = threading.Thread(target=packet_parser)
    t2.start()

    # main thread graphics
    root = Tk()
    navMap = Map(root).grid(row=0, column=0, padx=5, pady=5)
    console = Console(root).grid(row=1, column=0, padx=5, pady=5)
    controls = Controls(root).grid(row=0, column=1, padx=5, pady=5)
    information = Information(root).grid(row=1, column=1, padx=5, pady=5)
    root.geometry("1445x840")
    root.title("Gudrid Interface")
    root.mainloop()


if __name__ == '__main__':
    main()
