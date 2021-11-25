from tkinter import *
from ctypes import *

""" 
Get g_navigationMap -> Redraw the map
Get currentPosX and currentPosY -> Redraw the robot + output in information section
Get direction -> Outpu in information section
Get sensordata -> Output in console
"""


class Constants:

    FRAME_WIDTH = 1445
    FRAME_HEIGHT = 1000
    MAP_DELAY = 400
    CONSOLE_DELAY = 400
    ONE_STEP = 20
    CELL_SIZE = 20
    PADDING = 10
    ROBOT_X = 24
    ROBOT_Y = 24
    AUTO_SCROLL = True

    # MAP will be replaced with the internal map representation for the robot (g_navigationMap)
    MAP = [[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0],
           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]


class Map(LabelFrame):

    """Graphical representation of the robot's movement and the room"""

    def __init__(self, parent):
        LabelFrame.__init__(self, parent)
        self.parent = parent
        self.after(Constants.MAP_DELAY, self.onTimer)
        CELL_SIZE = 20
        SPACING = 2
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
        self.updateMap()

    def updateMap(self):

        for row in range(25):
            for col in range(49):
                if(Constants.MAP[row][col] == 1):
                    square = self.canvas.find_withtag(
                        str(col) + "," + str(row))
                    self.canvas.itemconfig(square, fill='blue')

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
        self.after(Constants.MAP_DELAY, self.onTimer)


class Console(LabelFrame):

    """Console for displaying sensor data and other relevant information"""

    def __init__(self, parent):
        LabelFrame.__init__(self, parent, bg='black')
        self.parent = parent
        self.after(Constants.CONSOLE_DELAY, self.onTimer)
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
        self.index += 1
        Label(self.frame, text="Some sensor data: "+str(self.index),
              bg='black', fg='white').grid(row=self.index, sticky=W)
        self.updateScollRegion()
        if Constants.AUTO_SCROLL:
            self.canvas.yview_moveto(1)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateConsole()
        self.after(Constants.CONSOLE_DELAY, self.onTimer)


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

        rightArrow = [270, 200, 270, 300, 320, 250]
        leftArrow = [150, 200, 150, 300, 100, 250]
        upArrow = [210, 140, 260, 190, 160, 190]
        downArrow = [210, 360, 260, 310, 160, 310]

        self.canvas.create_polygon(rightArrow, tags="right_arrow")
        self.canvas.create_polygon(leftArrow, tags="left_arrow")
        self.canvas.create_polygon(upArrow, tags="up_arrow")
        self.canvas.create_polygon(downArrow, tags="down_arrow")

        button1 = Button(self, text="MODE", anchor=S)
        button1.configure(width=10, activebackground="#33B5E5", relief=FLAT)
        button1_window = self.canvas.create_window(
            210, 450, anchor=S, window=button1)

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
            if Constants.AUTO_SCROLL:
                Constants.AUTO_SCROLL = False
            else:
                Constants.AUTO_SCROLL = True

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


class Information(LabelFrame):

    """Section for displaying some general info about the robot's current state"""

    def __init__(self, parent):
        LabelFrame.__init__(self, parent)
        self.parent = parent
        canvas = Canvas(self, bg='gray', width=420, height=299)
        canvas.create_text(Constants.PADDING,
                           Constants.PADDING, font=("Purisa", 20), text="Position: ", anchor=NW, tags="position_text")
        canvas.create_text(Constants.PADDING,
                           Constants.PADDING * 4, font=("Purisa", 20), text="Direction: ", anchor=NW, tags="heading_text")
        canvas.create_text(Constants.PADDING, Constants.PADDING * 7,
                           font=("Purisa", 20), text="Mode: ", anchor=NW, tags="mode_text")
        canvas.pack(side=LEFT)


def main():
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
