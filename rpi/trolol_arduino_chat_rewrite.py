import math
import time
import board
import busio
from adafruit_servokit import ServoKit
import math
import time

# Assuming a library like Adafruit_PCA9685 is used for servo control
# from adafruit_servokit import ServoKit
# kit = ServoKit(channels=16)

# Servo pins are just placeholders here as they would be set up differently in Python
# depending on the library and hardware being used. You'll need to adapt this part.
servo_pins = [[0, 1, 2], [4, 5, 6], [8, 9, 10], [12, 13, 14]]

# Size of the robot
length_a = 55.0
length_b = 77.5
length_c = 27.5
length_side = 71.0
z_absolute = -28.0

# Constants for movement
z_default = -50.0
z_up = -30.0
z_boot = z_absolute
x_default = 62.0
x_offset = 0.0
y_start = 0.0
y_step = 40.0
y_default = x_default

# Movement variables
site_now = [[[0.0] * 3 for _ in range(4)]]
site_expect = [[[0.0] * 3 for _ in range(4)]]
temp_speed = [[[0.0] * 3 for _ in range(4)]]
move_speed = 0.0
speed_multiple = 1.0

# Constants for speed
spot_turn_speed = 4.0
leg_move_speed = 8.0
body_move_speed = 3.0
stand_seat_speed = 1.0

# Placeholder for rest counter
rest_counter = 0

# Function parameter
KEEP = 255.0

# Math constant
pi = math.pi

# Temporary lengths for turn calculations
temp_a = math.sqrt((2 * x_default + length_side) ** 2 + y_step ** 2)
temp_b = 2 * (y_start + y_step) + length_side
temp_c = math.sqrt((2 * x_default + length_side) ** 2 + (2 * y_start + y_step + length_side) ** 2)
temp_alpha = math.acos((temp_a ** 2 + temp_b ** 2 - temp_c ** 2) / (2 * temp_a * temp_b))

# Sites for turn
turn_x1 = (temp_a - length_side) / 2
turn_y1 = y_start + y_step / 2
turn_x0 = turn_x1 - temp_b * math.cos(temp_alpha)
turn_y0 = temp_b * math.sin(temp_alpha) - turn_y1 - length_side

def set_site(leg, x, y, z):
    length_x, length_y, length_z = 0.0, 0.0, 0.0

    if x != KEEP:
        length_x = x - site_now[leg][0]
    if y != KEEP:
        length_y = y - site_now[leg][1]
    if z != KEEP:
        length_z = z - site_now[leg][2]

    length = math.sqrt(length_x ** 2 + length_y ** 2 + length_z ** 2)

    if length != 0:
        temp_speed[leg][0] = length_x / length * move_speed * speed_multiple
        temp_speed[leg][1] = length_y / length * move_speed * speed_multiple
        temp_speed[leg][2] = length_z / length * move_speed * speed_multiple

    if x != KEEP:
        site_expect[leg][0] = x
    if y != KEEP:
        site_expect[leg][1] = y
    if z != KEEP:
        site_expect[leg][2] = z

# Example servo control initialization (adapt based on your library and hardware)
# def servo_attach():
#     for i in range(4):
#         for j in range(3):
#             # This would be where you initialize the servo control, e.g.,
#             # kit.servo[servo_pins[i][j]].angle = 90  # Example initialization
#             pass

# def servo_detach():
#     # Depending on your control library, you might implement detachment.
#     pass
