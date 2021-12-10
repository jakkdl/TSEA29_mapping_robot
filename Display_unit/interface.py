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

g_output = []
g_output_debug = []
nrOut = ""
g_dict = {"command": 0xB2, "kd": 0xE2, "kp": 0xD2}

g_file_raw = False  # out put raw package data to file
g_file = False  # out put console date to file

g_x = -1
g_y = -1
g_infromation_update = False
g_color = "red"

g_pos_x = -1
g_pos_y = -1
g_infromation_update = False
g_pos_update = False


class Constants:

    FRAME_WIDTH = 1445
    FRAME_HEIGHT = 1000
    DELAY = 50
    ONE_STEP = 20
    CELL_SIZE = 20
    PADDING = 10
    AUTO_SCROLL = True
    AUTONOMOUS = True
    ROBOT_COLOR = "red"
    WALL_COLOR = "blue"


class Map(LabelFrame):

    """Graphical representation of the robot's movement and the room"""

    def __init__(self, parent):
        """Constructor"""
        LabelFrame.__init__(self, parent)
        self.parent = parent
        self.after(Constants.DELAY, self.onTimer)
        CELL_SIZE = 20
        SPACING = 4
        self.canvas = Canvas(self, width=CELL_SIZE * 49, height=CELL_SIZE * 25)

        for x in range(49):
            for y in range(25):

                # Create an "xy" tag for each rectangle on the map
                coord = (x, y)
                stringTuple = tuple(map(str, coord))
                tag = stringTuple[0] + "," + stringTuple[1]

                self.canvas.create_rectangle(x * CELL_SIZE + SPACING, y * CELL_SIZE + SPACING,
                                             (x + 1)*CELL_SIZE, (y + 1)*CELL_SIZE, fill='gray', width=0, tags=tag)

        self.canvas.create_rectangle(Constants.ROBOT_X * CELL_SIZE + SPACING, Constants.ROBOT_Y * CELL_SIZE + SPACING, 25 *
                                     CELL_SIZE, 25 * CELL_SIZE, fill='red', tags='robot')
        self.canvas.pack(side=LEFT)

    def updateMap(self):
        """if g_map_update has been reciver from console out and update the grid"""
        global g_infromation_update
        if g_infromation_update:
            square = self.canvas.find_withtag(
                str(g_x) + "," + str(g_y))
            self.canvas.itemconfig(square, fill=Constants.WALL_COLOR)
            self.after(Constants.DELAY, self.onTimer)
            g_infromation_update = False

    def moveRobot(self):
        """animates the robot's movement"""
        global g_pos_update
        if g_pos_update:
            global g_pos_x
            global g_pos_y
            x = g_pos_x
            y = g_pos_y
            robot = self.canvas.find_withtag('robot')
            self.canvas.move(robot, x, y)

    def onTimer(self):
        '''creates a cycle each timer event'''
        # self.moveRobot() this logic does not work with this implemetation
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

        global g_output
        global nrOut
        # lidar forward
        global g_output

        if g_output:

            # debug
            if g_output[0][0] == 12:
                # pd package
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
                        nrOut = "Package miss match: " + str(g_output[0])

                    # Navigation goal package
                elif g_output[0][2] == 254:
                    if len(g_output[0]) == 7:
                        nrOut = "\nNavigationGoal X: " + \
                            str(g_output[0][4] << 8 | g_output[0][3])
                        nrOut = nrOut + " \nNavigationGoal Y: " + \
                            str(g_output[0][6] << 8 | g_output[0][5])
                    else:
                        nrOut = "Paket miss match: " + str(g_output[0])

                    # reference package
                elif g_output[0][2] == 253:
                    if len(g_output[0]) == 7:
                        nrOut = "\nReference Pos X: " + \
                            str(g_output[0][4] << 8 | g_output[0][3])
                        nrOut = nrOut + " \nReference Pos Y: " + \
                            str(g_output[0][6] << 8 | g_output[0][5])
                    else:
                        nrOut = "Package miss match: " + str(g_output[0])

                    # nav goal heading package
                elif g_output[0][2] == 252:
                    if len(g_output[0]) == 5:
                        nrOut = nrOut + " \nNavigationGoalHeading: " + \
                            str(g_output[0][4] << 8 | g_output[0][3])
                    else:
                        nrOut = "Package miss match: " + str(g_output[0])

                    # debugs id followed by int 16
                elif 42 <= g_output[0][2] <= 46:
                    if len(g_output[0]) == 5:
                        nrOut = str(
                            g_output[0][2]) + " " + str(uint16_to_int16(g_output[0][4] << 8 | g_output[0][3]))
                    else:
                        nrOut = "Package miss match: " + str(g_output[0])

                    # break package end of pd loop
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
                    nrOut = "Lidar Forward package miss match " + \
                        str(g_output[0])

            # lidar backwards
            elif g_output[0][0] == 1:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "Lidar Backwards: " + str(nrOut)
                else:
                    nrOut = "Lidar Backwards package miss match " + \
                        str(g_output[0])

                # IR front left
            elif g_output[0][0] == 2:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Front Left: " + str(nrOut)
                else:
                    nrOut = "IR Front Left package miss match " + \
                        str(g_output[0])

                # IR back left
            elif g_output[0][0] == 3:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Back Left: " + str(nrOut)
                else:
                    nrOut = "IR Back Left package miss match " + \
                        str(g_output[0])

                # IR right front
            elif g_output[0][0] == 4:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Front Right: " + str(nrOut)
                else:
                    nrOut = "IR Front Right package miss match " + \
                        str(g_output[0])

                # IR right back
            elif g_output[0][0] == 5:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "IR Back Right: " + str(nrOut)
                else:
                    nrOut = "IR Back Right package miss match " + \
                        str(g_output[0])

                # gyro
            elif g_output[0][0] == 6:
                if len(g_output[0]) == 4:
                    nrOut = uint16_to_int16(
                        g_output[0][3] << 8 | g_output[0][2])
                    nrOut = "Gyro: " + str(nrOut)
                else:
                    nrOut = "Gyro package miss match " + str(g_output[0])

                # odometer
            elif g_output[0][0] == 7:
                if len(g_output[0]) == 4:
                    nrOut = "L: " + \
                        str(g_output[0][3]) + " R: " + str(g_output[0][2])
                    nrOut = "Odometer: " + str(nrOut)
                else:
                    nrOut = "Odometer package miss match " + str(g_output[0])

                # position
            elif g_output[0][0] == 8:
                if len(g_output[0]) == 6:
                    nrOut = "\nPositionX: " + \
                        str(g_output[0][3] << 8 | g_output[0][2])
                    nrOut += "\nPositionY: " + \
                        str(g_output[0][5] << 8 | g_output[0][4])
                    #g_pos_x = g_output[0][3] << 8 | g_output[0][2]
                    #g_pos_y = g_output[0][5] << 8 | g_output[0][4]
                    #g_infromation_update = True
                    #g_pos_update = True
                else:
                    nrOut = "Position package miss match " + str(g_output[0])

                # direction
            elif g_output[0][0] == 9:
                if len(g_output[0]) == 4:
                    nrOut = g_output[0][3] << 8 | g_output[0][2]
                    nrOut = "Direction: " + str(nrOut)
                else:
                    nrOut = "Direction package miss match " + \
                        str(g_output[0])

                # map update
            elif g_output[0][0] == 10:
                if len(g_output[0]) == 5:
                    nrOut = "Map update: " + str(g_output[0][2]) + " " + str(
                        g_output[0][3]) + " " + str(uint8_to_int8(g_output[0][4]))
                    #g_x = g_output[0][2]
                    #g_y = g_output[0][3]
                    # if uint8_to_int8( g_output[0][4] ) < 0:
                    #    g_color = "blue"
                    # else:
                    #    g_color = "green"
                    #g_map_update = True
                else:
                    nrOut = "Map update package miss match " + \
                        str(g_output[0])

                # last case should never happen
            else:
                nrOut = "Unknow package we should never be here check code: " + \
                    str(g_output[0])

            g_output.pop(0)

        if g_output:
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

        key = e.keysym

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

        key = e.keysym

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
        global g_dict
        if Constants.AUTONOMOUS:
            packageMaker("command", [0])
            Constants.AUTONOMOUS = False
        else:
            packageMaker("command", [1])
            Constants.AUTONOMOUS = True

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
                                Constants.PADDING * 4, font=("Purisa", 20), text="Direction: ", anchor=NW, tags="heading_text")
        self.canvas.create_text(Constants.PADDING, Constants.PADDING * 7,
                                font=("Purisa", 20), text="Mode: ", anchor=NW, tags="mode_text")
        self.canvas.pack(side=LEFT)

    def updateInformation(self):
        """updates the console"""
        mode = self.canvas.find_withtag("mode_text")
        pos = self.canvas.find_withtag("position_text")
        if(Constants.AUTONOMOUS):
            self.canvas.itemconfig(mode, text="Mode: AUTONOMOUS")
        else:
            self.canvas.itemconfig(mode, text="Mode: MANUAL")

        global g_infromation_update
        if g_infromation_update:
            global g_pos_x
            global g_pos_y
            x = g_pos_x
            y = g_pos_y
            xy = str(x) + " , " + str(y)
            self.canvas.itemconfig(pos, text="Position: " + xy)
            g_infromation_update = False

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
    if g_file:
        global g_output_debug

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
            if g_file:
                g_output_debug.append(out)

        else:
            print("In valid header recived not printe to file: ", temp)


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


def consoleOut():
    """
    this funtion reads the incomming data from the port that is saved to
    g_output. Each packet are a list in g_output where index 0 is the
    address and 1 is the byte count and index 2-6 is the byte.

    uint16_t are sent with lower part first.
    """
    global g_output_debug

    while True:
        if g_output_debug:
            nrOut = ""

            # debug
            if g_output_debug[0][0] == 12:
                # pd package
                if g_output_debug[0][2] == 255:
                    if len(g_output_debug[0]) == 7:
                        nrOut = "\nPropotional: " + \
                            str(uint16_to_int16(
                                g_output_debug[0][4] << 8 | g_output_debug[0][3]))
                        nrOut = nrOut + " \nDerivative: " + \
                            str(uint16_to_int16(
                                g_output_debug[0][6] << 8 | g_output_debug[0][5]))
                        #nrOut = nrOut + " \nCTE: " + str( float(g_output[0][8] << 8 | g_output[0][9]) )
                    else:
                        nrOut = "Package miss match: " + str(g_output_debug[0])

                    # Navigation goal package
                elif g_output_debug[0][2] == 254:
                    if len(g_output_debug[0]) == 7:
                        nrOut = "\nNavigationGoal X: " + \
                            str(g_output_debug[0][4] <<
                                8 | g_output_debug[0][3])
                        nrOut = nrOut + " \nNavigationGoal Y: " + \
                            str(g_output_debug[0][6] <<
                                8 | g_output_debug[0][5])
                    else:
                        nrOut = "Package miss match: " + str(g_output_debug[0])

                    # reference package
                elif g_output_debug[0][2] == 253:
                    if len(g_output_debug[0]) == 7:
                        nrOut = "\nReference Pos X: " + \
                            str(g_output_debug[0][4] <<
                                8 | g_output_debug[0][3])
                        nrOut = nrOut + " \nReference Pos Y: " + \
                            str(g_output_debug[0][6] << 8 | g_output[0][5])
                    else:
                        nrOut = "Package miss match: " + str(g_output_debug[0])

                    # nav goal heading package
                elif g_output_debug[0][2] == 252:
                    if len(g_output_debug[0]) == 5:
                        nrOut = nrOut + " \nNavigationGoalHeading: " + \
                            str(g_output_debug[0][4] <<
                                8 | g_output_debug[0][3])
                    else:
                        nrOut = "Package miss match: " + str(g_output_debug[0])

                    # debugs id followed by int 16
                elif 42 <= g_output_debug[0][2] <= 46:
                    if len(g_output_debug[0]) == 5:
                        nrOut = str(g_output_debug[0][2]) + " " + str(
                            uint16_to_int16(g_output_debug[0][4] << 8 | g_output_debug[0][3]))
                    else:
                        nrOut = "Package miss match: " + str(g_output_debug[0])

                    # break package end of pd loop
                elif g_output_debug[0][2] == 100:
                    nrOut = "\n"*10

                    # generic debug id + uint 16
                elif len(g_output_debug[0]) == 5:
                    nrOut = "Debug: " + \
                        str(g_output_debug[0][2]) + " " + \
                        str(g_output_debug[0][4] << 8 | g_output_debug[0][3])

                    # debug with unknow id and none generic
                else:
                    nrOut = "unknow debug: " + str(g_output_debug[0])

            # lidar forward
            elif g_output_debug[0][0] == 0:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "Lidar Forward: " + str(nrOut)
                else:
                    nrOut = "Lidar Forward package miss match " + \
                        str(g_output_debug[0])

            # lidar backwards
            elif g_output_debug[0][0] == 1:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "Lidar Backwards: " + str(nrOut)
                else:
                    nrOut = "Lidar Backwards package miss match " + \
                        str(g_output_debug[0])

                # IR front left
            elif g_output_debug[0][0] == 2:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "IR Front Left: " + str(nrOut)
                else:
                    nrOut = "IR Front Left package miss match " + \
                        str(g_output_debug[0])

                # IR back left
            elif g_output_debug[0][0] == 3:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "IR Back Left: " + str(nrOut)
                else:
                    nrOut = "IR Back Left package miss match " + \
                        str(g_output_debug[0])

                # IR right front
            elif g_output_debug[0][0] == 4:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "IR Front Right: " + str(nrOut)
                else:
                    nrOut = "IR Front Right package miss match " + \
                        str(g_output_debug[0])

                # IR right back
            elif g_output_debug[0][0] == 5:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "IR Back Right: " + str(nrOut)
                else:
                    nrOut = "IR Back Right package miss match " + \
                        str(g_output_debug[0])

                # gyro
            elif g_output_debug[0][0] == 6:
                if len(g_output_debug[0]) == 4:
                    nrOut = uint16_to_int16(
                        g_output_debug[0][3] << 8 | g_output_debug[0][2])
                    nrOut = "Gyro: " + str(nrOut)
                else:
                    nrOut = "Gyro package miss match " + str(g_output_debug[0])

                # odometer
            elif g_output_debug[0][0] == 7:
                if len(g_output_debug[0]) == 4:
                    nrOut = "L: " + \
                        str(g_output_debug[0][3]) + \
                        " R: " + str(g_output_debug[0][2])
                    nrOut = "Odometer: " + str(nrOut)
                else:
                    nrOut = "Odometer package miss match " + \
                        str(g_output_debug[0])

                # position
            elif g_output_debug[0][0] == 8:
                if len(g_output_debug[0]) == 6:
                    nrOut = "\nPositionX: " + \
                        str(g_output_debug[0][3] << 8 | g_output_debug[0][2])
                    nrOut += "\nPositionY: " + \
                        str(g_output_debug[0][5] << 8 | g_output_debug[0][4])
                else:
                    nrOut = "Position package miss match " + \
                        str(g_output_debug[0])

                # direction
            elif g_output_debug[0][0] == 9:
                if len(g_output_debug[0]) == 4:
                    nrOut = g_output_debug[0][3] << 8 | g_output_debug[0][2]
                    nrOut = "Direction: " + str(nrOut)
                else:
                    nrOut = "Direction package miss match " + \
                        str(g_output_debug[0])

                # map update
            elif g_output[0][0] == 10:
                if len(g_output[0]) == 5:
                    nrOut = "Map update: " + str(g_output_debug[0][2]) + " " + str(
                        g_output_debug[0][3]) + " " + str(uint8_to_int8(g_output_debug[0][4]))
                    g_x = g_output_debug[0][2]
                    g_y = g_output_debug[0][3]
                    if uint8_to_int8(g_output_debug[0][4]) < 0:
                        g_color = "blue"
                    else:
                        g_color = "green"
                    g_map_update = True
                else:
                    nrOut = "Map update package miss match " + \
                        str(g_output_debug[0])

                # last case should never happen
            else:
                nrOut = "Unknow package we should never be here check code: " + \
                    str(g_output_debug[0])

            # add the current nrout which is our output to debug file
            f = open("debug.txt", "a")
            f.write(nrOut + "\n")
            f.close()
            g_output_debug.pop(0)


def main():
    # reset debug console data
    if g_file:
        f0 = open("debug.txt", "w")
        f0.write("")
        f0.close()

    # reset raw package data
    if g_file_raw:
        f1 = open("debugraw.txt", "w")
        f1.write("")
        f1.close()

    # package listener thread

    root = Tk()
    navMap = Map(root).grid(row=0, column=0, padx=5, pady=5)
    console = Console(root).grid(row=1, column=0, padx=5, pady=5)
    controls = Controls(root).grid(row=0, column=1, padx=5, pady=5)
    information = Information(root).grid(row=1, column=1, padx=5, pady=5)
    root.geometry("1445x840")
    root.title("Gudrid Interface")
    root.mainloop()

    # Create thread for incoming bluetooth stream
    t1 = threading.Thread(target=listener)
    t1.start()


if __name__ == '__main__':
    main()
