import cflib
import logging
import sys
import time
import math
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig

URI = 'radio://0/80/2M'

if len(sys.argv) > 1:
    URI = sys.argv[1]
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class MyCrazyFlieClient:

    def __init__(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        cf = Crazyflie(rw_cache='./cache')
        self.scf = SyncCrazyflie(URI, cf=cf)
        self.scf.open_link()
        self.client = None
        log_config = LogConfig(name='kalman', period_in_ms=100)
        log_config.add_variable('kalman.stateX', 'float')
        log_config.add_variable('kalman.stateY', 'float')
        self.logger_pos = SyncLogger(self.scf, log_config)
        log_config_2 = LogConfig(name='stabilizer', period_in_ms=100)
        log_config_2.add_variable('stabilizer.yaw', 'float')
        self.logger_orientation = SyncLogger(self.scf, log_config_2)
        self.home_pos = self.get_position()
        print('Home = %s ' % self.home_pos)
        self.orientation = self.get_orientation()
        self.client = MotionCommander(self.scf)
        self.client.take_off(height=0.6)

    def land(self):
        self.client.land()
        self.scf.close_link()

    def check_if_in_target(self, goal):
        pos = self.get_position()
        distance = np.sqrt(np.power((goal[0] - pos[0]), 2) + np.power((goal[1] - pos[1]), 2))
        if distance < 1:
            self.land()
            return True
        return False

    def get_position(self):
        self.logger_pos.connect()
        data = self.logger_pos.next()[1]
        self.logger_pos.disconnect()
        self.logger_pos._queue.empty()
        return [data['kalman.stateX'], data['kalman.stateY']]

    def get_orientation(self):
        self.logger_orientation.connect()
        data = self.logger_orientation.next()[1]
        self.logger_orientation.disconnect()
        self.logger_orientation._queue.empty()
        return data['stabilizer.yaw']

    def straight(self, speed):
        self.client.forward(0.25, speed)
        start = time.time()
        return start

    def yaw_right(self):
        self.client.turn_right(30, 72)
        start = time.time()
        return start

    def yaw_left(self):
        self.client.turn_left(30, 72)
        start = time.time()
        return start

    def take_action(self, action):

        start = time.time()

        collided = False

        if action == 0:
            start = self.straight(0.25)

        if action == 1:
            start = self.yaw_right()

        if action == 2:
            start = self.yaw_left()
        # print(start)
        return collided

    def goal_direction(self, goal, pos):
        yaw = self.get_orientation()
        pos_angle = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
        pos_angle = math.degrees(pos_angle) % 360
        track = math.radians(pos_angle - yaw)
        return ((math.degrees(track) - 180) % 360) - 180

    def observe(self, goal):
        position_now = self.get_position()
        track = self.goal_direction(goal, position_now)
        distance = np.sqrt(np.power((goal[0] - position_now[0]), 2) + np.power((goal[1] - position_now[1]), 2))
        distance_sensor = Multiranger(self.scf)
        distance_sensor.start()
        front_distance_sensor = distance_sensor.front
        while front_distance_sensor is None:
            time.sleep(0.01)
            front_distance_sensor = distance_sensor.front
        distance_sensor.stop()
        print('Position now = %s ' % position_now)
        print('Track = %s ' % track)
        print('Distance to target: %s' % distance)
        print('Front distance: %s' % front_distance_sensor)
        return position_now[0], position_now[1], track, distance, front_distance_sensor


