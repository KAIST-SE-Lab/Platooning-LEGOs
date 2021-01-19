#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.messaging import BluetoothMailboxClient, NumericMailbox
import random

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the Ultrasonic Sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=118)

# Current speed, normal speed, maximum speed, and minimum speed
DRIVE_SPEED = 90
NORMAL_SPEED = 90
MAX_SPEED = 165
MIN_SPEED = 80

# Calculate the light threshold. Choose values based on your measurements.
BLACK = 5
WHITE = 70
threshold = (BLACK + WHITE) / 2

# Set the gain of the PID controller.
PROPORTIONAL_GAIN = 0.4
INTEGRAL_GAIN = 0.15
DERIVATIVE_GAIN = 0.7

# Intialize variables related to PID controller.
deviation = 0.0
derivative = 0.0
integral = 0.0
last_deviation = 0.0

# Lane change state
# step 0: drive on the outer lane
# step 1: change lane (from outer to inner)
# step 2: drive on the inner lane
# step 3: change lane (from inner to outer)
step = 0
# Time that state changed previously
previousStateChangedTime = 0
# Time that robot has stopped
stop_time = 0
# The flag to control robot stop or not
stopping = False

data = DataLog('time', 'step', 'color', 'speed', 'distance', 'stop', 'deviation', 'integral', 'derivative',
    'front_id', 'front_time', 'front_step', 'front_speed', 'front_distance')

# Initialize Bluetooth client and mailboxes for message passing.
client = BluetoothMailboxClient()
mbox_id = NumericMailbox('id2', client)
mbox_time = NumericMailbox('time2', client)
mbox_lane = NumericMailbox('lane2', client)
mbox_speed = NumericMailbox('speed2', client)
mbox_distance = NumericMailbox('distance2', client)

# Server robot name
SERVER = 'ev3-2'
print('establishing connection...')
client.connect(SERVER)
print('ev3-2 connected!')

# Wait until receive message from the negotiator.
while True:
    msg_id = mbox_id.read()
    if msg_id != None:
        break

# Start time.
watch = StopWatch()
watch.reset()

while True:
    time = watch.time()
    color = line_sensor.reflection()
    distance = obstacle_sensor.distance()
    # noise = random.randrange(-100,101)
    # distance = distance + noise
    # if distance < 0:
    #     distance = 0
    # elif distance > 2550:
    #     distance = 2550
    # else:
    #     None
    
    # Read messages received from the front robot(negotiator) .
    front_id = mbox_id.read()
    front_time = mbox_time.read()
    front_lane = mbox_lane.read()
    front_speed = mbox_speed.read()
    front_distance = mbox_distance.read()
    # print("id: %d time: %d lane: %d speed: %d distance: %d"%(front_id, front_time, front_lane, front_speed, front_distance))

    # Follow the front robot(negotiator) to change lane. (step 0->1 and step 2->3)
    # While changing lane, change state if the white part is detected. (step 1->2 and step 3->0)
    if step == 0:
        if front_lane == 1 and time >= previousStateChangedTime+1000:
            step = 1
            previousStateChangedTime = time
    elif step == 1:
        if color > 60 and time >= previousStateChangedTime+2500:
            integral = 0.0
            derivative = 0.0
            step = 2
            previousStateChangedTime = time
    elif step == 2:
        if front_lane == 3 and time >= previousStateChangedTime+1000:
            step = 3
            previousStateChangedTime = time
    elif step == 3:
        if color > 57 and time >= previousStateChangedTime+2000:
            integral = 0.0
            derivative = 0.0
            step = 0
            previousStateChangedTime = time

    # Keep safe distance from the front robot(negotiator).
    if distance <= 300:
        if not stopping and time >= stopTime + 1000 and step != 1 and step != 3:
            stopping = True
            stopTime = time
        # else:
        #     if time >= stopTime+1000:
        #         if step == 0 or step == 2:
        #             step = step + 1
        #             previousStateChangedTime = time
        #         integral = 0.0
        #         derivative = 0.0
        #         stopTime = time
        #         stopping = False
    else:
        stopping = False
        stopTime = time

    # ...
    if step == 1 or step == 3 or abs(integral) > 100:
        DRIVE_SPEED = NORMAL_SPEED
    elif distance >= 300:
        DRIVE_SPEED *= 1.05
        if DRIVE_SPEED > MAX_SPEED:
            DRIVE_SPEED = MAX_SPEED
    else:
        DRIVE_SPEED = NORMAL_SPEED

    if not stopping:
        # Calculate the deviation from the threshold.
        deviation = color - threshold
        # Calculate the integral.
        if deviation > -10 and deviation < 10:
            integral = 0
        elif deviation * last_deviation < 0:
            integral = 0
        else:
            integral = integral + deviation
        # Calculate the derivative.
        derivative = deviation - last_deviation

        # Calculate the turn rate.
        turn_rate = (PROPORTIONAL_GAIN * deviation) + (INTEGRAL_GAIN * integral) + (DERIVATIVE_GAIN * derivative)
        
        # step 0: use the calculated turn_rate
        # step 1: robot is turning right
        # step 2: use the opposite calculated turn_rate
        # step 3: robot is turning left
        if step == 1:
            turn_rate = 13.5
        elif step == 2:
            turn_rate = -1*turn_rate
        elif step == 3:
            turn_rate = -11

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        last_deviation = deviation
    else: 
        robot.stop()
    
    # print("step: "+str(step))
    # print("color: "+str(color))
    # print("speed: "+str(DRIVE_SPEED))
    # print("distance: "+str(distance))
    # print("stopOrNot: "+str(stopping))
    # print("stop time: "+str(stop_time))
    # print("time: "+str(time))
    # print("time difference: "+str(end_time-time))
    # print("deviation: "+str(deviation))
    # print("integral: "+str(integral))
    # print("derivative: "+str(derivative))

    # Store data log.
    data.log(time, step, color, DRIVE_SPEED, distance, stopping, deviation, integral, derivative,
     front_id, front_time, front_lane, front_speed, front_distance)

    end_time = watch.time()
    # Keep time of each loop constant 200ms.
    wait_time = 0
    if (end_time-time) < 200:
        wait_time = 200-(end_time-time)
    wait(wait_time)
