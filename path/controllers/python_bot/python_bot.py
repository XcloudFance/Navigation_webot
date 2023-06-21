from controller import Robot, Motor, GPS, Compass, Keyboard
import math

TIME_STEP = 16
TARGET_POINTS_SIZE = 16
DISTANCE_TOLERANCE = 1.5
MAX_SPEED = 7.0
TURN_COEFFICIENT = 4.0
X, Y, Z, ALPHA = 0, 1, 2, 3
LEFT, RIGHT = 0, 1

class Vector:
    def __init__(self, u, v):
        self.u = u
        self.v = v

targets = [Vector(-4.209318, 9.147717), Vector(0.946812, 9.404304), Vector(0.175989, -1.784311), Vector(-2.805353, -8.829694), 
           Vector(17.746730, -8.8202851), Vector(26.28, -8.82), Vector(26.28, 8.37), Vector(43.4, 8.37), 
           Vector(47.88, -4.81), Vector(43.4, 8.37), Vector(26.28, -8.82), Vector(17.74, -8.82), 
           Vector(-2.8, 8.82), Vector(0.17,-1.78),Vector(0.94,9.4),Vector(-4.2,9.14)]
current_target_index = 0
autopilot = True
old_autopilot = True
old_key = -1

def modulus_double(a, m):
    div = int(a / m)
    r = a - div * m
    if r < 0.0:
        r += m
    return r

robot = Robot()

names = ["left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
         "right motor 1", "right motor 2", "right motor 3", "right motor 4"]
motors = [robot.getDevice(name) for name in names]
for motor in motors:
    motor.setPosition(float('inf'))

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

def robot_set_speed(left, right):
    for i in range(4):
        motors[i + 0].setVelocity(left)
        motors[i + 4].setVelocity(right)

def check_keyboard():
    speeds = [0.0, 0.0]
    global autopilot, old_key, old_autopilot

    key = keyboard.getKey()
    if key >= 0:
        if key == Keyboard.UP:
            speeds[LEFT] = MAX_SPEED
            speeds[RIGHT] = MAX_SPEED
            autopilot = False
        elif key == Keyboard.DOWN:
            speeds[LEFT] = -MAX_SPEED
            speeds[RIGHT] = -MAX_SPEED
            autopilot = False
        elif key == Keyboard.RIGHT:
            speeds[LEFT] = MAX_SPEED
            speeds[RIGHT] = -MAX_SPEED
            autopilot = False
        elif key == Keyboard.LEFT:
            speeds[LEFT] = -MAX_SPEED
            speeds[RIGHT] = MAX_SPEED
            autopilot = False
        elif key == ord('P'):
            if key != old_key:
                position_3d = gps.getValues()
                print("position: {%f, %f}\n" % (position_3d[X], position_3d[Y]))
        elif key == ord('A'):
            if key != old_key:
                autopilot = not autopilot
    if autopilot != old_autopilot:
        old_autopilot = autopilot
        if autopilot:
            print("auto control\n")
        else:
            print("manual control\n")

    robot_set_speed(speeds[LEFT], speeds[RIGHT])
    old_key = key

def norm(v):
    return math.sqrt(v.u * v.u + v.v * v.v)

def normalize(v):
    n = norm(v)
    v.u /= n
    v.v /= n

def minus(v, v1, v2):
    v.u = v1.u - v2.u
    v.v = v1.v - v2.v

def run_autopilot():
    speeds = [0.0, 0.0]
    global current_target_index

    position_3d = gps.getValues()
    north_3d = compass.getValues()

    position = Vector(position_3d[X], position_3d[Y])

    direction = Vector(0, 0)
    minus(direction, targets[current_target_index], position)
    distance = norm(direction)
    normalize(direction)

    robot_angle = math.atan2(north_3d[0], north_3d[1])
    target_angle = math.atan2(direction.v, direction.u)
    beta = modulus_double(target_angle - robot_angle, 2.0 * math.pi) - math.pi

    if beta > 0:
        beta = math.pi - beta
    else:
        beta = -beta - math.pi

    if distance < DISTANCE_TOLERANCE:
        index_char = "th"
        if current_target_index == 0:
            index_char = "st"
        elif current_target_index == 1:
            index_char = "nd"
        elif current_target_index == 2:
            index_char = "rd"
        print("%d%s target reached\n" % (current_target_index + 1, index_char))
        current_target_index += 1
        current_target_index %= TARGET_POINTS_SIZE
    else:
        speeds[LEFT] = MAX_SPEED - math.pi + TURN_COEFFICIENT * beta
        speeds[RIGHT] = MAX_SPEED - math.pi - TURN_COEFFICIENT * beta

    robot_set_speed(speeds[LEFT], speeds[RIGHT])

print("You can drive this robot:")
print("Select the 3D window and use cursor keys:")
print("Press 'A' to return to the autopilot mode")
print("Press 'P' to get the robot position")
print("\n")

robot.step(1000)

robot_set_speed(MAX_SPEED, MAX_SPEED)

while robot.step(TIME_STEP) != -1:
    check_keyboard()
    if autopilot:
        run_autopilot()
