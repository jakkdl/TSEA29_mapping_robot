from struct import pack
from tkinter import *
import serial
import threading
import time
#import serialtestMAC

ser = serial.Serial(
    port='/dev/tty.Firefly-71B7-SPP',  # need to fin the correct port
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

g_output = []
g_dict = {'kd': 0xD2, 'kp': 0xE2}

""" 
Get g_navigationMap -> Redraw the map
Get currentPosX and currentPosY -> Redraw the robot + output in information section
Get direction -> Output in information section
Get sensordata -> Output in console
"""

#nearby_devices = bluetooth.discover_devices(lookup_names=True)

"""
INSTRUCTIONS:

Use arrow keys to send movement input. Press space to pause autoscrolling in the console.
"""


""" def init_port():
    ser = serial.Serial(
        port='/dev/cu.Bluetooth-Incoming-Port',
        baudrate=9600,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )
    while(True):

        temp = ser.read().hex()
        print(temp)
 """


class Constants:

    FRAME_WIDTH = 1445
    FRAME_HEIGHT = 1000
    DELAY = 300
    ONE_STEP = 20
    CELL_SIZE = 20
    PADDING = 10
    ROBOT_X = 24
    ROBOT_Y = 24
    AUTO_SCROLL = True
    AUTONOMOUS = True


class Map(LabelFrame):

    """Graphical representation of the robot's movement and the room"""

    def __init__(self, parent):
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
        # self.updateMap()

    def updateMap(self):
        global g_output
        if g_output:
            if (g_output[0][0] == 10) and (g_output[0][4] == 1):
                square = self.canvas.find_withtag(
                    str(g_output[0][2]) + "," + str(g_output[0][3]))
                self.canvas.itemconfig(square, fill='blue')
                g_output.pop(0)

    def getCoordinates(self):
        print("Fetching currentXPos and currentYPos")

    def getDirection():
        print("Direction")

    def moveRobot(self):
        '''animates the robot's movement'''
        robot = self.canvas.find_withtag('robot')
        self.canvas.move(robot, Constants.ONE_STEP, 0)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.moveRobot()
        # self.updateMap()
        self.after(Constants.DELAY, self.onTimer)


class Console(LabelFrame):

    """Console for displaying sensor data and other relevant information"""

    def __init__(self, parent):
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
        if g_output:
            self.index += 1
            Label(self.frame, text="Some sensor data: "+str(self.index),
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

        self.button1 = Button(self, text="MODE", width=10,
                              command=self.setNavigationMode, state=NORMAL)
        self.button1_window = self.canvas.create_window(
            210, 350, anchor=S, window=self.button1)

        self.canvas.create_text(90,
                                385, font=("Purisa", 20), text="KD: ", )
        self.canvas.create_text(90,
                                415, font=("Purisa", 20), text="KP: ", )

        self.inputKd = Entry(self)
        self.inputKd_window = self.canvas.create_window(
            210, 400, anchor=S, window=self.inputKd)

        self.inputKp = Entry(self)
        self.inputKp_window = self.canvas.create_window(
            210, 430, anchor=S, window=self.inputKp)

        self.button1 = Button(self, text="SET PD", width=10,
                              command=self.setPd, state=NORMAL)
        self.button1_window = self.canvas.create_window(
            210, 470, anchor=S, window=self.button1)

    def onKeyPressed(self, e):
        '''controls direction variables with cursor keys'''

        key = e.keysym

        LEFT_CURSOR_KEY = "Left"
        if key == LEFT_CURSOR_KEY:
            arrow = self.canvas.find_withtag("left_arrow")
            self.canvas.itemconfig(arrow, fill='green')
            print("Rotate left")

        RIGHT_CURSOR_KEY = "Right"
        if key == RIGHT_CURSOR_KEY:
            arrow = self.canvas.find_withtag("right_arrow")
            self.canvas.itemconfig(arrow, fill='green')
            print("Rotate right")

        UP_CURSOR_KEY = "Up"
        if key == UP_CURSOR_KEY:
            arrow = self.canvas.find_withtag("up_arrow")
            self.canvas.itemconfig(arrow, fill='green')
            print("Go forward")

        DOWN_CURSOR_KEY = "Down"
        if key == DOWN_CURSOR_KEY:
            arrow = self.canvas.find_withtag("down_arrow")
            self.canvas.itemconfig(arrow, fill='green')
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
        Constants.AUTONOMOUS = not Constants.AUTONOMOUS

    def setPd(self):
        """Function called when pressing SET PD"""
        kd = self.inputKd.get()
        kp = self.inputKp.get()

        packageMaker("kd", [int(kd)])
        packageMaker("kp", [int(kp)])


class Information(LabelFrame):

    """Section for displaying some general info about the robot's current state"""

    def __init__(self, parent):
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

    def updateInforamation(self):
        '''updates the console'''
        mode = self.canvas.find_withtag("mode_text")
        if(Constants.AUTONOMOUS):
            self.canvas.itemconfig(mode, text="Mode: AUTONOMOUS")
        else:
            self.canvas.itemconfig(mode, text="Mode: MANUAL")

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateInforamation()
        self.after(Constants.DELAY, self.onTimer)


def listener():
    print("Hello")
    global g_output
    while True:

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
            temp = ser.read()[0]
            out.append(temp)
            i += 1
        print(out)
        g_output.append(out)


""" def write(send):
    data_bytes = bytes(send)

    # ser.write(data_bytes) """


def packageMaker(operation, byteList):

    global g_dict

    output = bytearray()
    output.append(g_dict.get(operation))

    for byte in byteList:
        output.append(byte)

    print(str(output))

    ser.write(output)


def check_output():
    global g_output
    if g_output:
        print(g_output)
        g_output = []


def main():

    t1 = threading.Thread(target=listener)
    t1.start()

    # graphics
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
