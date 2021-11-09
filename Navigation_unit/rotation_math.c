//TSEA29, initial author JL
//rotation math, given in ~pseudocode
//
// need to make decision on where 0 degrees is, which of left/right-turning is increasing/decreasing
// angles, and where (0, 0) is.

int AXLE_WIDTH = 200; // must be measured
int MID_TO_WHEEL_CENTER = 141; //must be measured
int WHEEL_CIRCUMFEENCE = 65;
int COGS = 12; // should be checked

// ### Formula for new heading, given how much we rotated on the spot ###
// old_heading [radians], measured clockwise from starting position.
// direction==1 for turning to the right
// direction==-1 for turning to the left
// cog_steps [mm] from the ordometer
int rotated_on_the_spot(float old_heading, int direction, int cog_steps)
{

    // from the formula of circle sector
    // see image rotate_on_the_spot_heading_update.jpg
    float new_heading = (old_heading + direction *
        (cog_steps * WHEEL_CIRCUMFERENCE / COGS) / MID_TO_WHEEL_CENTER);
    return new_heading;
}

// Formula for new heading and position, given that one wheel is stationary
/// and the other has moved, so we're rotating around the stationary wheel
//
// This is a meme/WIP, and will never actually be used. But is indicative of what
// might be needed for full tracking of heading&position given arbitrary wheel turns.
//
// old_heading [radians]
// direction==1 if driving forward, -1 if reverse
// turning_wheel==1 for left wheel, -1 for right wheel
// old_pos_x, old_pos_y positions of the midpoint of the robot
void one_wheel_rotation(float old_heading,
        int direction,
        int turning_wheel,
        int old_pos_x,
        int old_pos_y,
        float &new_heading,
        int &new_pos_x,
        int &new_pos_y)
{
    // from the formula of circle sector
    // same math as rotated_on_the_spot, except it can turn in two directions
    // see single_wheel_turn_heading_update.jpg
    new_heading = (old_heading + direction * turning_wheel *
            (cog_steps * WHEEL_CIRCUMFERENCE / COGS) / MID_TO_WHEEL_CENTER);

    // DARK MAGIC
    // a bunch of trigonometry and circle sector stuff
    // see single_wheel_turn_position_update_SKETCH.jpg for a proof of concept
    if (direction == 1 && turning_wheel == 1)
    {
        float angle_change = new_heading - old_heading;
        float small_angle = (0.5 * math.tau - angle_change) / 2 - old_heading;
        float base = 2 * math.sin(angle_change/2) / MID_TO_WHEEL_CENTER;

        // needs +/- depending on quadrant
        new_pos_x = old_pos_x + math.cos(small_angle) / base;
        new_pos_y = old_pos_y + math.sin(small_angle) / base;
    }
    else
    {
        arghlabarg;
    }


}

// the number of ordometer steps that represent a full turn
// the robot is 200x200, so the radius is ~100
int MAGIC_ORDO = 2 * math.pi * 100 / WHEEL_CIRCUMFERENCE / COGS; //approximate theoretical value


void draw_laser_line(int left_ordo, int right_ordo, int pos_x, int pos_y, int distance)
{
    // see laser_intersecting_pot_walls.jpg
    // where x_r,y_r is pos_x,pos_y
    // x_s, y_s is laser_x, laser_y
    // d is distance
    // x_n , y_n is end_x, end_y

    // Calculate heading (can likely be skipped, and instead just store/calculate
    // cos(heading) and sin(heading) directly)
    heading = heading(left_ordo, right_ordo);

    // calculate the front laser's position
    // depends on how directions/coordinates are defined
    laser_x = pos_x + cos(heading)*laser_distance;
    laser_y = pos_y + sin(heading)*laser_distance;

    // Calculate X, Y for endpoint when laser hits wall -> end_x, end_y
    // given the distance.
    int end_x = (laser_x + cos(heading)*distance) / 400;
    int end_y = (laser_y + sin(heading)*distance) / 400;
    // Do error checking that this is where there can be a wall.
    // Also we should try and throw out values that are incorrect due to the wall being
    // too close (the measured voltage from IR, and therefore the distance, will be misleading).
    // We can perhaps correct heading and/or position given this.
    //
    // Mark map[end_x][end_y] as a wall
    map[end_x][end_y] -= 1;



    // pos_x and pos_y are in mm coordinates, to translate to wall coordinates we divide
    // by 400 since wall units are 400mm wide.


    // A line drawn across a square grid at an angle will intersect with points at
    // (X_0, Y_0), (X_0+1, Y_1), (X_0+2, Y_2) and (X_0, Y_0), (X_1, Y_0+1), (X_2, Y_0+2)
    // where these will yield the same set of points for 45 degrees / tau/8 radians.
    // We handle these separately

    // calculate the first point where the laser crosses x%400 == 0
    // TODO rounding up/down, and +/- depends on heading
    int x_0 = math.ceil(laser_x / 400);
    int y_0 = laser_y + sin(heading) * (laser_x / 400) - x_0;
    // number of potential walls the laser crosses before hitting the end wall
    int max_steps = end_x - x_0;

    for (int steps = 0; steps < max_steps; ++steps)
    {
        int x = x_0 + steps;
        int y = math.floor(y_0 + steps * sin(heading));

        // mark as empty space
        map[x][y] += 1;
    }

    //do the same for y%400 == 0
    y_0 = math.ceil(laser_y / 400);
    x_0 = laser_x + cos(heading) * (laser_y / 400) - y_0;
    // number of potential walls the laser crosses before hitting the end wall
    max_steps = end_y - y_0;

    for (int steps = 0; steps < max_steps; ++steps)
    {
        int y = y_0 + steps;
        int x = math.floor(x_0 + steps * sin(heading));

        // mark as empty space
        map[x][y] += 1;
    }


    // Then repeat this for all four lasers, with their position slightly different
    // and pointing at different angles.


    // We can optionally also check if the robot is currently standing on top of
    // a potential wall (or several), and mark those as empty.
    // This can be more or less ambitious, taking into account the width of the robot and stuff.

}


// Helper function for determining where a laser crosses a potential wall
// Takes ordometer steps for left & right wheelpair, and returns a delta_x to be used
// when drawing the imaginary "lines" of a laser as it crosses the floor.
// currently unused
float delta_x(int left_ordo, int right_ordo) {
    int res = 1;
    int stuff;
    if (left_ordo > right_ordo)
    {
        stuff = (left_ordo - right_ordo) % MAGIC_ORDO;
        if (stuff < MAGIC_ORDO/2)
        {
            res = -1;
        }
    }
    else
    {
        stuff = (right_ordo - left_ordo) % MAGIC_ORDO;
        if (stuff > MAGIC_ORDO/2)
        {
            res = -1;
        }
    }

    // v = 2*math.pi* stuff / MAGIC_ORDO
    // d_x = sin(v)
    return MAGIC_TABLE[stuff];

}
