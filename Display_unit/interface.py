from tkinter import *


class Constants:

    FRAME_WIDTH = 1445
    FRAME_HEIGHT = 1000
    MAP_DELAY = 400
    CONSOLE_DELAY = 400
    ONE_STEP = 20
    CELL_SIZE = 20
    PADDING = 10


class Map(LabelFrame):
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
                tag = stringTuple[0] + stringTuple[1]

                self.canvas.create_rectangle(x * CELL_SIZE + SPACING, y * CELL_SIZE + SPACING,
                                             (x + 1)*CELL_SIZE, (y + 1)*CELL_SIZE, fill='gray', width=0, tags=tag)

        self.canvas.create_rectangle(24 * CELL_SIZE + SPACING, 24 * CELL_SIZE + SPACING, 25 *
                                     CELL_SIZE, 25 * CELL_SIZE, fill='red', tags='robot')
        self.canvas.pack(side=LEFT)

    def moveRobot(self):
        '''animates the robot's movement'''
        robot = self.canvas.find_withtag('robot')
        self.canvas.move(robot, Constants.ONE_STEP, 0)
        #square = self.find_withtag('56')
        #self.itemconfig(square, fill='blue')

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.moveRobot()
        self.after(Constants.MAP_DELAY, self.onTimer)


class Console(LabelFrame):
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
        Label(self.frame, text="Print: "+str(self.index),
              bg='black', fg='white').grid(row=self.index, sticky=W)
        self.updateScollRegion()
        self.canvas.yview_moveto(1)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.updateConsole()
        self.after(Constants.CONSOLE_DELAY, self.onTimer)


class Controls(LabelFrame):
    def __init__(self, parent):
        LabelFrame.__init__(self, parent)
        self.parent = parent
        CELL_SIZE = 20
        canvas = Canvas(self, bg='gray', width=420, height=CELL_SIZE * 25 - 1)
        canvas.pack(side=LEFT)


class Information(LabelFrame):
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
    root.geometry("1445x1000")
    root.title("Gudrid Interface")
    root.mainloop()


if __name__ == '__main__':
    main()
