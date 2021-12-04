from tkinter import *
import serial
import threading
import time

"""port needs to be changed depending on which computer you are using"""
ser = serial.Serial(
    port='/dev/rfcomm0',
    baudrate=115200,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

g_output = []
nrOut = ""
g_dict = {"command": 0xB2, "kd": 0xD2, "kp": 0xE2}


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
        global g_output
        if g_output:
            if (g_output[0][0] == 10) and (g_output[0][4] == 1):
                square = self.canvas.find_withtag(
                    str(g_output[0][2]) + "," + str(g_output[0][3]))
                self.canvas.itemconfig(square, fill='blue')
                g_output.pop(0)

    def moveRobot(self):
        '''animates the robot's movement'''
        global g_output
        if g_output:
            if g_output[0][0] == 8:
                x = g_output[0][2] << 8 | g_output[0][3]
                y = g_output[0][4] << 8 | g_output[0][5]
                robot = self.canvas.find_withtag('robot')
                self.canvas.move(robot, x, y)

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

        global g_output
        global nrOut
        # lidar forward
        if g_output:
            # print("Length of g_output: ", len(g_output[0]))
            if g_output[0][0] == 0:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Lidar Forward: " + str(nrOut)
                g_output.pop(0)

            # lidar backwards
            elif g_output[0][0] == 1:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Lidar Backwards: " + str(nrOut)
                g_output.pop(0)

                # IR front left
            elif g_output[0][0] == 2:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "IR Front Left: " + str(nrOut)
                g_output.pop(0)

                # IR back left
            elif g_output[0][0] == 3:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "IR Back Left: " + str(nrOut)
                g_output.pop(0)

                # IR right front
            elif g_output[0][0] == 4:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "IR Front Right: " + str(nrOut)
                g_output.pop(0)

                # IR right back
            elif g_output[0][0] == 5:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "IR Back Right: " + str(nrOut)
                g_output.pop(0)

                # gyro
            elif g_output[0][0] == 6:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Gyro: " + str(nrOut)
                g_output.pop(0)

                # odometer
            elif g_output[0][0] == 7:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Odometer: " + str(nrOut)
                g_output.pop(0)

                # direction
            elif g_output[0][0] == 9:
                nrOut = g_output[0][3] << 8 | g_output[0][2]
                nrOut = "Direction: " + str(nrOut)
                g_output.pop(0)

                # debug
            elif g_output[0][0] == 12:
                print("G_output:", g_output)
                if g_output[0][1] == 3:
                    nrOut = str(g_output[0][2]) + " " + \
                        str(g_output[0][4] << 8 | g_output[0][3])
                    """else:
                    debug = len(g_output[0])
                    nrOut = "Debug: " + str(debug)
                    i = 2
                    while(i < debug):
                        nrOut += " " + str(g_output[0][i])"""
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
            packageMaker("command", [4])
            print("Rotate left")

        RIGHT_CURSOR_KEY = "Right"
        if key == RIGHT_CURSOR_KEY:
            arrow = self.canvas.find_withtag("right_arrow")
            self.canvas.itemconfig(arrow, fill='green')
            packageMaker("command", [5])
            print("Rotate right")

        UP_CURSOR_KEY = "Up"
        if key == UP_CURSOR_KEY:
            arrow = self.canvas.find_withtag("up_arrow")
            self.canvas.itemconfig(arrow, fill='green')
            packageMaker("command", [2])
            print("Go forward")

        DOWN_CURSOR_KEY = "Down"
        if key == DOWN_CURSOR_KEY:
            arrow = self.canvas.find_withtag("down_arrow")
            self.canvas.itemconfig(arrow, fill='green')
            packageMaker("command", [3])
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

    def updateInforamation(self):
        '''updates the console'''
        global g_output
        mode = self.canvas.find_withtag("mode_text")
        pos = self.canvas.find_withtag("position_text")
        if(Constants.AUTONOMOUS):
            self.canvas.itemconfig(mode, text="Mode: AUTONOMOUS")
        else:
            self.canvas.itemconfig(mode, text="Mode: MANUAL")
        if g_output:
            if g_output[0][0] == 8:
                x = g_output[0][2] << 8 | g_output[0][3]
                y = g_output[0][4] << 8 | g_output[0][5]
                xy = str(x) + " , " + str(y)
                self.canvas.itemconfig(pos, text="Position: " + xy)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateInforamation()
        self.after(Constants.DELAY, self.onTimer)


def listener():
    print("----- LISTENING FOR BLUETOOTH INPUT -----")
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
                print(temp)
                break
            out.append(result)
            i += 1
        # print("OUT: ", out)
        g_output.append(out)


def packageMaker(operation, byteList):

    listToSend = [g_dict.get(operation)] + byteList

    for byte in listToSend:
        package = bytearray()
        package.append(byte)
        ser.write(package)
        print("Package: ", package)
        time.sleep(0.1)
    time.sleep(1)


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
