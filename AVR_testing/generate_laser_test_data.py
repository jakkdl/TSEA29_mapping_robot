import math
from math import *
from dataclasses import dataclass

@dataclass
class Sensor:
    x: int
    y: int
    direction: int

sensors = [
        (120, 0, 0),  #lidar_forward
        (-120, 0, 2), #lidar_backward
        (80, 70, 1),  #ir_leftfront
        (-85, 70, 1), #ir_leftback
        (80, -70, 3), #ir_rightfront
        (-85, -70, 3),#ir_rightback
        ]

def grid_to_mm(coord):
    return coord*400 + 200

def calc_dist(pos_x, pos_y, sensor, heading):
    start_x = (pos_x
            + (math.cos(heading) * sensors[sensor][0]
                + math.cos(heading + math.tau/4) * sensors[sensor][1]))
    start_y = (pos_y
            + (math.sin(heading) * sensors[sensor][0]
                + math.sin(heading + math.tau/4) * sensors[sensor][1]))
    laser_dir = (heading + sensors[sensor][2]*math.tau/4) % math.tau
    if (laser_dir < math.tau / 4):
        x_wall = math.ceil(start_x/400) *400#+400
        y_wall = math.ceil(start_y/400) *400#+400
    elif (laser_dir < math.tau / 2):
        x_wall = math.floor(start_x/400)*400#-400
        y_wall = math.ceil(start_y/400) *400#+400
    elif (laser_dir < math.tau / 4*3):
        x_wall = math.floor(start_x/400)*400#-400
        y_wall = math.floor(start_y/400)*400#-400
    else:
        x_wall = math.ceil(start_x/400) *400#+400
        y_wall = math.floor(start_y/400)*400#-400

    if cos(laser_dir) == 0:
        return abs(start_y - y_wall) / abs(math.sin(laser_dir))
    if sin(laser_dir) == 0:
        return abs(start_x - x_wall) / abs(math.cos(laser_dir))

    x_dist = abs(start_x - x_wall) / abs(math.cos(laser_dir))
    y_dist = abs(start_y - y_wall) / abs(math.sin(laser_dir))
    if (x_dist < y_dist):
        #print("x:", pos_x, start_x, x_wall, x_dist)
        pass
    else:
        laser_dir_2 = heading + sensors[sensor][2]*math.tau/4 + math.tau/128
        #print(laser_dir_2, math.cos(laser_dir_2))
        end_x = start_x + y_dist*math.cos(laser_dir_2)
        #print(start_x + y_dist*math.cos(laser_dir))
        end_y = start_y + y_dist*math.sin(laser_dir_2)

        #print(math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2))
        #print(f'y: pos {pos_x, pos_y}, start ({start_x}, {start_y}) '
                #f'end ({end_x:.2f}, {end_y:.2f}) {y_wall} {y_dist:.2f}')
        #print(calc_heading_y(pos_x, pos_y, end_x, end_y, y_wall))

    return min(x_dist, y_dist)

def calc_heading_y(pos_x, pos_y, end_x, end_y, y_wall, sensor, distance):
    distance += abs(sensors[sensor][1])
    h_dist = sqrt(sensors[sensor][0]**2 + distance**2)
    heading_shift = acos(distance / h_dist)
    whr = asin((end_y-pos_y)/h_dist)

    a = start_x - pos_x
    b = start_y - pos_y
    c = y_wall - pos_y
    print(f'a {a} b: {b} c: {c}')
    v = math.tau/16
    print(a*math.sin(v), b*math.cos(v), c)
    #print(math.sqrt(a*a+b*b-c*c)-a, b-c)
    #return 2* math.atan2(math.sqrt(a*a+b*b-c*c)-a, b-c)

def calc_dists(x, y, heading):
    return [calc_dist(x, y, i, heading) for i in range(0, 6)]

def laser_cos(v, i):
    return cos(v + sensors[i][2]/4*tau)
def laser_sin(v, i):
    return sin(v + sensors[i][2]/4*tau)

def calculate_dif(pos):
    if pos < 0:
        return pos
    res = pos % GRID_SIZE
    if res > GRID_SIZE/2:
        return res-GRID_SIZE;
    return res;

def calc_laser_data(x, y, heading, i, dist):
    startx = x + cos(heading) * sensors[i][0] + cos(heading + tau/4) * sensors[i][1]
    starty = x + sin(heading) * sensors[i][0] + sin(heading + tau/4) * sensors[i][1]
    endx = startx + laser_cos(heading, i) * dist
    endy = starty + laser_sin(heading, i) * dist
    xdif = calculate_dif(endx)
    ydif = calculate_dif(endy)

    heading + sensors[i][2]/4*tau

def generate_data():
    x = grid_to_mm(10)
    y = grid_to_mm(10)
    length = 16
    resstart = 'uint16_t test_sensor_values[16][6] = {\n'
    resend = '}'
    lines = []
        #l = ['{:3d}'.format(round(calc_dist(x, y, s, i*math.tau/64))) for s in range(0, 6)]
        #print("{", ", ".join(l), "},", sep='')
    for i in range(0, 16):
        lines.append([calc_dist(x, y, s, i*math.tau/64) for s in range(0, 6)])
    print(resstart,
            ",\n".join(
            '    {'+",".join(
                '{:3d}'.format(round(v)) for v in line)+'}' for line in lines),
            resend, sep='')

        
#def calc_adjusts(heading):
#    x = grid_to_mm(4)
#    y = grid_to_mm(4)
#    for heading in range(0, tau, tau/16):
#        dists = calc_dists(x, y, heading)
#        for heading_error in (-tau/64, +tau/64):

if __name__ == '__main__':
    #for i in range(0, 6):
        #print(calc_dist(x, y, i, h));
    generate_data()
