from tkinter import *


class Cons:

    FRAME_WIDTH = 1000
    FRAME_HEIGHT = 1000


class Map(Canvas):

    def __init__(self):
        super().__init__(width=Cons.FRAME_WIDTH, height=Cons.FRAME_HEIGHT,
                         background="gray", highlightthickness=0)

        self.pack()


class Interface(Frame):

    def __init__(self):
        super().__init__()

        self.master.title('Gudrid Interface')
        self.map = Map()
        self.pack()


if __name__ == '__main__':
    root = Tk()
    nib = Interface()
    root.mainloop()
