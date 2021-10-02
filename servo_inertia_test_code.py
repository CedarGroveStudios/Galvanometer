# Proportional X/Y Servo Control with Feather M4 and Servo Wing
# (c) 2018, 2021 Cedar Grove Studios
# servo_inertia_test_code.py 2018-04-24 v12

import board
import busio
import time
import neopixel as neo
import random as rn
from analogio import AnalogIn
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Instantiate Servo Wing
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Instantiate servos
# Specify min/max pulse parameters for Tower SG92R micro servo
servo_x = servo.Servo(pca.channels[0], actuation_range=180, min_pulse=500, max_pulse=2350)
servo_y = servo.Servo(pca.channels[1], actuation_range=180, min_pulse=500, max_pulse=2350)

# Define joystick input pins
x_pin = AnalogIn(board.A0)
y_pin = AnalogIn(board.A1)

# Dim on-board neopixel and show yellow start-up indicator
pixel = neo.NeoPixel(board.NEOPIXEL,1, brightness=0.01, auto_write=False)
pixel[0] = (200, 200, 0)
pixel.write()

time.sleep(0.1)

# ## Helpers ##
def get_angle(pin):
    """Read analog pin value, convert to angle (-90 to +90)"""
    return ((pin.value * 180) / 65536) - 90

def inertia_move(x=0, y=0, target_x=0, target_y=0, rate=0, increment=1):
    """Proportionally move horizontal and vertical servos from x/y position
    to target x/y. Angle x/y parameters range from -90 degrees to +90 degrees
    with 0/0 as center position. Rate is specified as seconds per 180 degree
    movement (servo end-to-end mechanical span). Increment is the servo movement
    resolution; the number of degrees of movement per step."""

    # Moving towards target = red neopixel
    pixel[0] = (255, 0, 0)
    pixel.write()

    increment = max(0.001, min(increment, 180))  # Constrain increment value

    # Execute an immediate return to center when current and target = 0
    if (x == 0 and target_x == 0) and (y == 0 and target_y == 0):
        servo_x.angle = x + 90
        servo_y.angle = y + 90

    # ## Proportional movement control
    delta_x = target_x - x
    delta_y = target_y - y
    # Calculate incremental steps needed for x and y deltas
    # Movement speed is controlled by larger delta
    if abs(delta_x) < abs(delta_y):
        steps = abs(delta_y / increment)
        if steps == 0:
            steps = 1
        x_increment = delta_x / steps
        y_increment = increment
        if delta_y < 0:
            y_increment = -1 * increment
    else:
        steps = abs(delta_x / increment)
        if steps == 0:
            steps = 1
        x_increment = increment
        y_increment = delta_y / steps
        if delta_x < 0:
            x_increment = -1 * increment

    # Move the servos to target angle
    for i in range(int(abs(steps))):
        x = max(-90, min(x, +90))  # Constrain angle values
        y = max(-90, min(x, +90))
        servo_x.angle = x + 90  # Nomalize to servos' 0-180 range
        servo_y.angle = y + 90
        time.sleep(rate * (increment / 180))  # hold at position based upon rate
        x = x + x_increment
        y = y + y_increment

    x = target_x
    y = target_y

    # Target reached = green neopixel
    pixel[0] = (0, 255, 0)
    pixel.write()

    return (x, y) # return current x, y position values

# ### Set Initial Parameters ###
"""Experiment with these parameters to get the right movement feel. Movement
rate is the time to move the servo from -90 to +90 degrees (180-degree
displacement). Servo increment is the smallest movement of the servo, a servo
movement step. Servo hold time is the delay between completed movements.
The servo increment is useful for simulating saccadic eye movement.
Future: Consider using three trimpots attached to analog inputs for real-time
adjustment of these settings."""
MOVE_RATE = 3  # number of seconds to move full scale (0-180)
SERVO_INCREMENT = .5  # smallest movement in degrees; the servo "step"
SERVO_HOLD = 0.1  # number of seconds to hold position before repositioning

# Quickly center the servos before starting
x = y = 0
(x, y) = inertia_move(x, y, x, y, MOVE_RATE, SERVO_INCREMENT)
time.sleep(SERVO_HOLD)  # hold the current position

print("servo_inertia_test_code.py 2018-04-24 v12")

# ### Main Loop ###
while True:
    # Insert code here if analog trimpots are used for rate, increment, and hold parameters

    # Read angles from joystick
    joystick_x = get_angle(x_pin)
    joystick_y = get_angle(y_pin)

    # Create some random values for testing without joystick
    # joystick_x = (rn.random() * 180) - 90
    # joystick_y = (rn.random() * 180) - 90

    # Move from current position to joystick values
    (x, y) = inertia_move(x, y, joystick_x, joystick_y, MOVE_RATE, SERVO_INCREMENT)
    time.sleep(SERVO_HOLD)  # hold the current position
