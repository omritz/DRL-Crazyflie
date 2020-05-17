import numpy as np
import time
import math
import cv2
from pylab import array, arange, uint8
from PIL import Image
import eventlet
from eventlet import Timeout
import multiprocessing as mp
# Change the path below to point to the directoy where you installed the AirSim PythonClient
# sys.path.append('C:/Users/Kjell/Google Drive/MASTER-THESIS/AirSimpy')
# import setup_path
from airsim.client import *


class myAirSimClient():

    def __init__(self):
        initX = 0
        initY = 0
        initZ = -3

        # connect to the AirSim simulator
        self.client = MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.client.takeoffAsync().join()
        self.client.moveToPositionAsync(initX, initY, initZ, 5).join()

        self.home_pos = self.client.simGetVehiclePose()

        self.home_ori = self.client.simGetGroundTruthKinematics().orientation

        self.z = -3

    @staticmethod
    def toEulerianAngle(q):
        z = q.z_val
        y = q.y_val
        x = q.x_val
        w = q.w_val
        ysqr = y * y

        # roll (x-axis rotation)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        roll = math.atan2(t0, t1)

        # pitch (y-axis rotation)
        t2 = +2.0 * (w * y - z * x)
        if (t2 > 1.0):
            t2 = 1
        if (t2 < -1.0):
            t2 = -1.0
        pitch = math.asin(t2)

        # yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        yaw = math.atan2(t3, t4)

        return pitch, roll, yaw

    def getPitchRollYaw(self):
        return self.toEulerianAngle(self.client.simGetGroundTruthKinematics().orientation)

    def getPosition(self):
        return self.client.simGetVehiclePose().position

    def straight(self, duration, speed):
        pitch, roll, yaw = self.getPitchRollYaw()
        vx = math.cos(yaw) * speed
        vy = math.sin(yaw) * speed
        self.client.moveByVelocityZAsync(vx, vy, self.z, duration, DrivetrainType.ForwardOnly)
        start = time.time()
        return start, duration

    def yaw_right(self, duration):
        self.client.rotateByYawRateAsync(30, duration)
        start = time.time()
        return start, duration

    def yaw_left(self, duration):
        self.client.rotateByYawRateAsync(-30, duration)
        start = time.time()
        return start, duration

    def take_action(self, action):

        # check if copter is on level cause sometimes he goes up without a reason
        x = 0
        while self.getPosition().z_val < -7.0:
            self.client.moveToZAsync(-6, 3)
            time.sleep(1)
            print(self.getPosition().z_val, "and", x)
            x = x + 1
            if x > 10:
                return True

        start = time.time()
        duration = 0

        collided = False

        if action == 0:

            start, duration = self.straight(1, 4)

            while duration > time.time() - start:
                if self.client.simGetCollisionInfo().has_collided:
                    return True

            self.client.moveByVelocityAsync(0, 0, 0, 1)
            self.client.rotateByYawRateAsync(0, 1)

        if action == 1:

            start, duration = self.yaw_right(0.8)

            while duration > time.time() - start:
                if self.client.simGetCollisionInfo().has_collided:
                    return True

            self.client.moveByVelocityAsync(0, 0, 0, 1)
            self.client.rotateByYawRateAsync(0, 1)

        if action == 2:

            start, duration = self.yaw_left(1)

            while duration > time.time() - start:
                if self.client.simGetCollisionInfo().has_collided:
                    return True

            self.client.moveByVelocityAsync(0, 0, 0, 1)
            self.client.rotateByYawRateAsync(0, 1)

        return collided

    def goal_direction(self, goal, pos):

        pitch, roll, yaw = self.getPitchRollYaw()
        yaw = math.degrees(yaw)

        pos_angle = math.atan2(goal[1] - pos.y_val, goal[0] - pos.x_val)
        pos_angle = math.degrees(pos_angle) % 360

        track = math.radians(pos_angle - yaw)

        return ((math.degrees(track) - 180) % 360) - 180


    def get_state_from_sim(self, track):
        return (self.client.getDistanceSensorData(distance_sensor_name = "Distance", vehicle_name = "SimpleFlight").distance)


    def AirSim_reset(self):

        self.client.reset()
        time.sleep(0.2)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        time.sleep(1)
        self.client.moveToZAsync(self.z, 3)
        time.sleep(3)
