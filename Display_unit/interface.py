from tkinter import *


class Constants:

    FRAME_WIDTH = 1500
    FRAME_HEIGHT = 1000
    DELAY = 100
    CELL_SIZE = 19
    PADDING = 15


class Map(Canvas):

    def __init__(self):
        super().__init__(width=Constants.FRAME_WIDTH, height=Constants.FRAME_HEIGHT,
                         background="grey50", highlightthickness=0)
        self.init_map()
        self.pack()

    def init_map(self):
        self.running = True
        self.createObjects()
        #self.bind_all("<Key>", self.onKeyPressed)
        self.after(Constants.DELAY, self.onTimer)

    def createObjects(self):
        for x in range(49):
            for y in range(25):
                self.create_rectangle(x*Constants.CELL_SIZE+2, y*Constants.CELL_SIZE+2, (x+1)*Constants.CELL_SIZE,
                                      (y+1)*Constants.CELL_SIZE, fill='grey39', width=0)

        self.create_rectangle(24*Constants.CELL_SIZE+2,
                              24*Constants.CELL_SIZE+2, 25*Constants.CELL_SIZE, 25*Constants.CELL_SIZE, fill='red', tags="robot")

        self.create_rectangle(49*Constants.CELL_SIZE+10,
                              Constants.PADDING, 1430, 300, fill='grey60', width=0, tags="directionWindow")
        self.create_rectangle(49*Constants.CELL_SIZE+10,
                              310, 1430, 610, fill='grey60', width=0, tags="steeringWindow")
        self.create_rectangle(
            Constants.PADDING, 490, 925, 830, fill='grey60', width=0, tags="consoleWindow")

        print("Creating objects")

    def onTimer(self):
        '''creates a cycle each timer event'''
        if self.running:
            self.moveRobot()
            self.after(Constants.DELAY, self.onTimer)
            print("Running")
        else:
            self.doneRunning()

    def doneRunning(self):
        print("Done running")

    def moveRobot(self):
        '''moves the robot on the map display'''
        robot = self.find_withtag("robot")
        self.move(robot, Constants.CELL_SIZE, 0)


class Interface(Frame):

    def __init__(self):
        super().__init__()

        self.master.title('Gudrid Interface')
        self.map = Map()
        self.pack()


def main():
    root = Tk()
    nib = Interface()
    root.mainloop()


if __name__ == '__main__':
    main()
