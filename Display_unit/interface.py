import turtle

window = turtle.Screen()
map_frame = turtle.Turtle()
console_frame = turtle.Turtle()
movement_frame = turtle.Turtle()
steering_frame = turtle.Turtle()

# Will probably use Tkinter or PyQT instead of turtle


def create_window():

    WIDTH = 1000
    HEIGHT = 1000

    window.title("Gudrid Interface")
    window.bgcolor("gray")
    window.setup(width=WIDTH, height=HEIGHT)

    map_frame.speed(0)
    map_frame.shape("square")
    map_frame.color("white")
    map_frame.penup()
    map_frame.goto(-190, 190)
    map_frame.shapesize(stretch_wid=30, stretch_len=30)

    window.tracer(0)

# loop
    while True:
        window.update()


if __name__ == '__main__':
    create_window()
