#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.messaging import BluetoothMailboxServer, NumericMailbox
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
WHITE = 75
threshold = (BLACK + WHITE) / 2

# Set the gain of the PID controller.
PROPORTIONAL_GAIN = 0.6
INTEGRAL_GAIN = 0.1
DERIVATIVE_GAIN = 0.5

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

# Initialize Bluetooth server.
server = BluetoothMailboxServer()

# Initialize mailboxes for the front robot(leader).
mbox1_id = NumericMailbox('id1', server)
mbox1_time = NumericMailbox('time1', server)
mbox1_lane = NumericMailbox('lane1', server)
mbox1_speed = NumericMailbox('speed1', server)
mbox1_distance = NumericMailbox('distance1', server)

# Initialize mailboxes for the follower.
mbox2_id = NumericMailbox('id2', server)
mbox2_time = NumericMailbox('time2', server)
mbox2_lane = NumericMailbox('lane2', server)
mbox2_speed = NumericMailbox('speed2', server)
mbox2_distance = NumericMailbox('distance2', server)

print('waiting two clients')
server.wait_for_connection(2)
print('connected!')

# Send robot id to leader and follower.
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)
mbox1ID.send(2)
mbox2ID.send(2)

# Start time.
watch = StopWatch()
watch.reset()

while True:
    time = watch.time()
    color = line_sensor.reflection()
    distance = obstacle_sensor.distance()

    # Send messages to the follower.
    mbox2_id.send(2)
    mbox2_time.send(time)
    mbox2_lane.send(step)
    mbox2_speed.send(DRIVE_SPEED)
    mbox2_distance.send(distance)

    # Read messages received from the front robot(leader).
    front_id = mbox1_id.read()
    front_time = mbox1_time.read()
    front_lane = mbox1_lane.read()
    front_speed = mbox1_speed.read()
    front_distance = mbox1_distance.read()
    # print("id: %d time: %d lane: %d speed: %d distance: %d"%(front_id, front_time, front_lane, front_speed, front_distance))
    
    # Follow the front robot(leader) to change lane. (step 0->1 and step 2->3)
    # While changing lane, change state if the white part is detected. (step 1->2 and step 3->0)
    if step == 0:
        if front_lane == 1 and time >= previousStateChangedTime+1000:
            step = 1
            previousStateChangedTime = time
    elif step == 1:
        if color > 65 and time >= previousStateChangedTime+3000:
            integral = 0.0
            derivative = 0.0
            step = 2
            previousStateChangedTime = time
    elif step == 2:
        if front_lane == 3 and time >= previousStateChangedTime+1000:
            step = 3
            previousStateChangedTime = time
    elif step == 3:
        if color > 65 and time >= previousStateChangedTime+2000:
            integral = 0.0
            derivative = 0.0
            step = 0
            previousStateChangedTime = stopTime

    # Keep safe distance from the front robot(leader).
    if distance <= 300:
        if not stopping and time >= stopTime + 1000 and step != 1 and step != 3:
            stopping = True
            stopTime = time
        # else:
        #     if time >= stopTime+2000:
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

    # While changing lane and the absolute value of integral is larger than 100, 
    # keep the normal speed.
    # Based on distance, accelerate or keep the normal speed
    if step == 1 or step == 3 or abs(integral) > 100:
        DRIVE_SPEED = NORMAL_SPEED
    elif distance > 330:
        DRIVE_SPEED = DRIVE_SPEED * 1.05
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
            turn_rate = 13
        elif step == 2:
            turn_rate = -1*turn_rate
        elif step == 3:
            turn_rate = -8

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)

        last_deviation = deviation

    else: 
        robot.stop()
    
    end_time = watch.time()

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

    # Store data log
    data.log(time, step, color, DRIVE_SPEED, distance, stopping, deviation, integral, derivative,
     front_id, front_time, front_lane, front_speed, front_distance)

    # Keep time of each loop constant 100ms.
    wait_time = 0
    if (end_time-time) < 100:
        wait_time = 100-(end_time-time)
    wait(wait_time)
