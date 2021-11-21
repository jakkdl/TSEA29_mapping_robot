from tkinter import *
from typing import Collection


class Constants:

    FRAME_WIDTH = 1445
    FRAME_HEIGHT = 1000
    DELAY = 400
    ONE_STEP = 20
    CELL_SIZE = 20
    PADDING = 10


class Window(Frame):

    def __init__(self):
        super().__init__(bg='grey39', padx=Constants.PADDING, pady=Constants.PADDING)
        self.master.title('Gudrid Interface')
        self.initInterface()
        self.pack()

    def initInterface(self):

        cellMap = Map(self).grid(row=0, column=0)
        console = Console(self).grid(row=1, column=0)
        directions = Controls(self).grid(row=0, column=1)
        information = Information(self).grid(row=1, column=1)


class Map(Canvas):
    def __init__(self, parent):
        Canvas.__init__(self, parent, width=Constants.CELL_SIZE*49,
                        height=Constants.CELL_SIZE*25)
        self.parent = parent
        self.after(Constants.DELAY, self.onTimer)
        self.createMap()

    def createMap(self):

        CELL_SIZE = 20

        for x in range(49):
            for y in range(25):

                # Create an "xy" tag for each rectangle on the map
                coord = (x, y)
                stringTuple = tuple(map(str, coord))
                tag = stringTuple[0] + stringTuple[1]

                self.create_rectangle(x * CELL_SIZE + 2, y * CELL_SIZE + 2,
                                      (x + 1)*CELL_SIZE, (y + 1)*CELL_SIZE, fill='gray', width=0, tags=tag)

        self.create_rectangle(24 * CELL_SIZE + 2, 24 * CELL_SIZE + 2, 25 *
                              CELL_SIZE, 25 * CELL_SIZE, fill='red', tags='robot')

    def moveRobot(self):
        '''animates the robot's movement'''
        robot = self.find_withtag('robot')
        self.move(robot, 20, 0)

    def onTimer(self):
        '''creates a cycle each timer event'''
        self.moveRobot()
        self.after(Constants.DELAY, self.onTimer)


class Console(Canvas):
    def __init__(self, parent):
        Canvas.__init__(self, parent, width=Constants.CELL_SIZE*49,
                        height=300)
        self.parent = parent
        self.createConsole()

    def createConsole(self):
        self.create_rectangle(0, 0, Constants.CELL_SIZE *
                              49, 300, fill='gray', width=0)


class Controls(Canvas):
    def __init__(self, parent):
        Canvas.__init__(self, parent, width=300,
                        height=Constants.CELL_SIZE*25)
        self.parent = parent
        self.createDirections()

    def createDirections(self):
        self.create_rectangle(0, 0, 300, 1000, fill='gray', width=0)


class Information(Canvas):
    def __init__(self, parent):
        Canvas.__init__(self, parent, width=300,
                        height=300)
        self.parent = parent
        self.createDirections()

    def createDirections(self):
        self.create_rectangle(0, 0, 300, 300, fill='gray', width=0)


def main():
    root = Tk()
    window = Window()
    root.mainloop()


if __name__ == '__main__':
    main()
