#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import DataLog, StopWatch, wait
from pybricks.messaging import BluetoothMailboxClient, BluetoothMailboxServer, NumericMailbox
import random

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Initialize the color sensor.
line_sensor = ColorSensor(Port.S3)

# Initialize the ultrasonic sensor. It is used to detect
# obstacles as the robot drives around.
obstacle_sensor = UltrasonicSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=118)

""" Configuration can be modified. """
# Current speed, normal speed, maximum speed, and minimum speed
DRIVE_SPEED = 90
NORMAL_SPEED = 90
MAX_SPEED = 130
MIN_SPEED = 80

""" Configuration can be modified. """
# Calculate the light threshold. Choose values based on your measurements.
BLACK = 5
WHITE = 70
threshold = (BLACK + WHITE) / 2

""" Configuration can be modified. """
# Set the gain of the PID controller.
PROPORTIONAL_GAIN = 0.4
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

""" Configuration can be modified. """
data = DataLog('time', 'step', 'color', 'speed', 'distance', 'stop', 'deviation', 'integral', 'derivative')

""" Client """
# Initialize Bluetooth client and mailboxes for message passing.
client = BluetoothMailboxClient()
mbox_id = NumericMailbox('id1', client)
mbox_time = NumericMailbox('time1', client)
mbox_lane = NumericMailbox('lane1', client)
mbox_speed = NumericMailbox('speed1', client)
mbox_distance = NumericMailbox('distance1', client)

# Server robot name
SERVER = 'ev3-2'
print('establishing connection...')
client.connect(SERVER)
print('ev3-2 connected!')

# Wait until receive message from server.
while True:
    msg_id = mbox_id.read()
    if msg_id != None:
        break
""" Client """

# """ Server """
# # Initialize Bluetooth server.
# server = BluetoothMailboxServer()

# # Initialize mailboxes for the front robot.
# mbox1_id = NumericMailbox('id1', server)
# mbox1_time = NumericMailbox('time1', server)
# mbox1_lane = NumericMailbox('lane1', server)
# mbox1_speed = NumericMailbox('speed1', server)
# mbox1_distance = NumericMailbox('distance1', server)

# # Initialize mailboxes for the follower.
# mbox2_id = NumericMailbox('id2', server)
# mbox2_time = NumericMailbox('time2', server)
# mbox2_lane = NumericMailbox('lane2', server)
# mbox2_speed = NumericMailbox('speed2', server)
# mbox2_distance = NumericMailbox('distance2', server)

# print('waiting two clients')
# server.wait_for_connection(2)
# print('connected!')

# """ More messages can be added. """
# # Send robot id to clients.
# mbox1ID.send(2)
# mbox2ID.send(2)
# mbox1ID.send(2)
# mbox2ID.send(2)
# mbox1ID.send(2)
# mbox2ID.send(2)
# mbox1ID.send(2)
# """ Server """

# Start time.
watch = StopWatch()
watch.reset()

while True:
    time = watch.time()
    color = line_sensor.reflection()
    distance = obstacle_sensor.distance()

    """ Message sending or receiving """
    # Send messages to the follower.
    mbox_id.send(1)
    mbox_time.send(time)
    mbox_lane.send(step)
    mbox_speed.send(DRIVE_SPEED)
    mbox_distance.send(distance)

    # # Read messages received from the front robot.
    # front_id = mbox1_id.read()
    # front_time = mbox1_time.read()
    # front_lane = mbox1_lane.read()
    # front_speed = mbox1_speed.read()
    # front_distance = mbox1_distance.read()
    # # print("id: %d time: %d lane: %d speed: %d distance: %d"%(front_id, front_time, front_lane, front_speed, front_distance))
    
    """ Based on vehicle role, algorithm can be modified. """
    # Follow the front robot(leader) to change lane. (step 0->1 and step 2->3)
    # While changing lane, change state if the white part is detected. (step 1->2 and step 3->0)
    if step == 0:
        None
        # if front_lane == 1 and time >= previousStateChangedTime+1000:
        #     step = 1
        #     previousStateChangedTime = time
    elif step == 1:
        if color > 65 and time >= previousStateChangedTime+2000:
            integral = 0.0
            derivative = 0.0
            step = 2
            previousStateChangedTime = time
    elif step == 2:
        None
        # if front_lane == 3 and time >= previousStateChangedTime+1000:
        #     step = 3
        #     previousStateChangedTime = time
    elif step == 3:
        if color > 65 and time >= previousStateChangedTime+2000:
            integral = 0.0
            derivative = 0.0
            step = 0
            previousStateChangedTime = stopTime

    """ Safe distance and stopping time can be varied. """
    """ If vehicle should not change lane after detecting obstacle or front vehicle, """
    """ the else section can be removed. """
    # If an obstable is detected, robot stops 1000ms and does lane change
    # Only when the robot is driving on the lane
    if distance <= 500:
        if not stopping and time >= stop_time+1000 and step != 1 and step != 3:
            stopping = True
            stop_time = time
        """ else section """
        else:
            if time >= stop_time+1000:
                if step == 0 or step == 2:
                    step = step + 1
                    previousStateChangedTime = time
                integral = 0.0
                derivative = 0.0
                stopping = False
                stop_time = time
        """ else section """
    else:
        stopping = False
        stop_time = time
    
    """ Speed Adaptation algorithm can be developed. """
    # While changing lane and the absolute value of integral is larger than 100, 
    # keep the minimum speed.
    # Based on distance, increase or decrease speed.
    if step == 1 or step == 3 or abs(integral) > 100:
        DRIVE_SPEED = MIN_SPEED
    elif distance > 600:
        if DRIVE_SPEED < MAX_SPEED:
            DRIVE_SPEED = DRIVE_SPEED + 0.5
    elif distance > 550 and distance <= 600:
        DRIVE_SPEED = NORMAL_SPEED
    elif distance > 500 and distance <= 550:
        if DRIVE_SPEED > MIN_SPEED:
            DRIVE_SPEED = DRIVE_SPEED - 1
    else:
        DRIVE_SPEED = MIN_SPEED

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
        
        """ turn_rate can be modified. """
        # step 0: use the calculated turn_rate
        # step 1: robot is turning right
        # step 2: use the opposite calculated turn_rate
        # step 3: robot is turning left
        if step == 1:
            turn_rate = 13
        elif step == 2:
            turn_rate = -1*turn_rate
        elif step == 3:
            turn_rate = -15

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
    
    # Store data log.
    data.log(time, step, color, DRIVE_SPEED, distance, stopping, deviation, integral, derivative)

    # Keep time of each loop constant 100ms.
    wait_time = 0
    if (end_time-time) < 100:
        wait_time = 100-(end_time-time)
    wait(wait_time)
	