
import time
from functools import partial

import cv2
import numpy as np
from pupil_apriltags import Detector
from simple_pid import PID
from spherov2 import scanner
from spherov2.sphero_edu import Color, SpheroEduAPI, EventType
import math


import simulator
from rpi_sensor import RPiSensor, RPiCamera

# Part 1: Acting Rationally - Cliff & Obstacle Detection

# 1. Obstacle Avoidance
def on_collision(droid: SpheroEduAPI):
    droid.stop_roll()
    droid.set_main_led(Color(255, 0, 0))
    time.sleep(1)
    droid.set_heading(droid.get_heading() + 180)
    droid.set_speed(80)
    droid.set_main_led(Color(0, 0, 0))

# droid.register_event(EventType.on_collision, on_collision)


def obstacle_avoidance(droid: SpheroEduAPI, sensor: RPiSensor):
    threshold = 5
    if sensor.get_obstacle_distance() < threshold:
        speed = droid.get_speed()
        droid.set_speed(0)
        time.sleep(0.5)
        droid.set_heading(droid.get_heading() + 180)
        droid.set_speed(speed)


def obstacle_avoidance_improved(droid: SpheroEduAPI, sensor: RPiSensor):
    threshold = 5
    if sensor.get_obstacle_distance() < threshold:
        speed = droid.get_speed()
        heading = droid.get_heading()
        droid.set_speed(0)
        time.sleep(0.5)

        droid.set_heading(heading - 90)
        dist_left = sensor.get_obstacle_distance()
        droid.set_heading(heading + 90)
        dist_right = sensor.get_obstacle_distance()

        if dist_left > dist_right and dist_left >= threshold:
            droid.set_heading(heading - 90)
        elif dist_right > dist_left and dist_right >= threshold:
            droid.set_heading(heading + 90)
        else:
            droid.set_heading(heading + 180)
        droid.set_speed(speed)


# 2. Cliff Avoidance
def cliff_avoidance(droid: SpheroEduAPI, sensor: RPiSensor):
    if sensor.get_cliff():
        speed = droid.get_speed()
        droid.stop_roll()
        time.sleep(0.5)
        droid.set_heading(droid.get_heading() + 180)
        droid.set_speed(speed)


# 3. Cliff & Obstacle Avoidancespeed = droid.get_speed()
#         heading = droid.get_heading()
#         droid.set_speed(0)
#         time.sleep(0.5)
def cliff_obstacle_avoidance(droid: SpheroEduAPI, sensor: RPiSensor):
    threshold = 5
    if sensor.get_obstacle_distance() < threshold or sensor.get_cliff():
        heading = droid.get_heading()
        speed = droid.get_speed()

        droid.set_speed(0)
        time.sleep(0.5)

        droid.set_heading(heading - 90)
        dist_left = sensor.get_obstacle_distance()
        cliff_left = sensor.get_cliff()
        droid.set_heading(heading + 90)
        dist_right = sensor.get_obstacle_distance()
        cliff_right = sensor.get_cliff()

        if cliff_right or dist_right <= threshold or dist_right < dist_left:
            if not cliff_left:
                droid.set_heading(heading - 90)
            else:
                droid.set_heading(heading + 180)
        elif cliff_left or dist_left <= threshold or dist_left < dist_right:
            if not cliff_right:
                droid.set_heading(heading + 90)
            else:
                droid.set_heading(heading + 180)
        else:
            droid.set_heading(heading + 180)
        droid.set_speed(speed)


def droid_explore():
    sim = simulator.test_maze4()  # replace with test_maze1, 2, 3, or 4
    droid, sensor = sim.droid, sim.sensor
    # replace the above lines with the line below to use the real droid
    #     with RPiSensor('IP_ADDRESS') as sensor, SpheroEduAPI(scanner.find_toy()) as droid:
    droid.register_event(EventType.on_collision, on_collision)
    droid.set_heading(-90)  # init heading, modify it for different maze
    droid.set_speed(50)  # init speed
    delay_sim = 0.01 # delay for simulator 
    delay_real = 0.1 # add delay for receiving sensor data (suggest 0.1 - 0.5 s)
    while True:
        cliff_obstacle_avoidance(droid, sensor)  # replace with control functions you implemented
        sim.update_location(delay_sim)  # comment out this line if you are using a real droid
        # time.sleep(delay_real) # remember to add delay when experimenting on real robots


# Part 2: Maximize Performance Measure - AprilTag Tracking
def april_tag_tracking(droid: SpheroEduAPI, pid: PID, at_center):
    """Use the given PID to calculate the heading offset needed for the AprilTag to be in the center of the frame.
    PID's input should be the X coordinate of the AprilTag and it outputs the offset for the heading.

    :param at_center: The (x, y) coordinate of the AprilTag on the frame
    """

    output = pid(at_center)
    droid.set_heading(droid.get_heading() - output) #According to piazza, this is the right way to adjust heading

def distance(a,b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def april_tag_following(droid: SpheroEduAPI, pid: PID, at_corner):
    """Use the given PID to calculate the speed needed for the AprilTag to be the right size on the frame.
    PID's input should be the average length of 4 sides of the AprilTag on the frame and it outputs the desired speed.

    :param at_corner: A 4x2 array that is the four (x, y) coordinates of the AprilTag on the frame
    """
    sum_length = distance(at_corner[0], at_corner[3])
    for i in range(3):
        sum_length += distance(at_corner[i], at_corner[i + 1])
    mean_length = sum_length / 4 # get the average length
    output = pid(mean_length)
    droid.set_speed(output) # use the output of pid to set speed

def rolling_with_u():
    with RPiCamera('tcp://IP_ADDRESS:65433') as camera, SpheroEduAPI(scanner.find_toy()) as droid:
        at_detector = Detector(families='tag36h11', nthreads=4)

        # Tune the parameters (optional)
        Kp_heading = 0.03
        Ki_heading = 0
        Kd_heading = 0

        Kp_speed = 2
        Ki_speed = 0
        Kd_speed = 0

        # Set Target
        desired_x_coord = 512
        desired_side_length = 150

        # Set up PIDs
        pid_heading = PID(Kp_heading, Ki_heading, Kd_heading, output_limits=(-60, 60), setpoint=desired_x_coord)
        pid_speed = PID(Kp_speed, Ki_speed, Kd_speed, output_limits=(0, 255), setpoint=desired_side_length)

        tag = True
        partial_pid_heading = partial(pid_heading, dt=1 / 30)
        partial_pid_speed = partial(pid_speed, dt=1 / 30)
        while True:
            img = camera.get_frame()
            tags = at_detector.detect(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))  # detect tags on the image

            if len(tags) != 1:
                if tag:
                    print('No Tag or Multiple Tags Detected!')
                    tag = False
                    droid.stop_roll()
                    pid_heading.reset()
                    pid_speed.reset()
            else:
                tag = True
                center = tags[0].center  # center in pixels
                corners = tags[0].corners  # four corners

                # plot
                cv2.circle(img, (int(center[0]), int(center[1])), 3, (0, 255, 0), 2)
                cv2.polylines(img, np.int32((corners,)), True, (0, 255, 0), 2)

                april_tag_tracking(droid, partial_pid_heading, center)
                april_tag_following(droid, partial_pid_speed, corners)

            cv2.imshow('AprilTag Rolling', img)
            if cv2.waitKey(1) == ord('q'):
                cv2.destroyAllWindows()
                break


if __name__ == '__main__':
     droid_explore()  # Part 1
    # rolling_with_u()  # Part 2
    # toy = scanner.find_toy(toy_name='D2-BBD9')
    # with SpheroEduAPI(toy) as droid:
    #     droid.roll(0, 100, 1)
