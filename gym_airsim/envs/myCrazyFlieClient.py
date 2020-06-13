import cflib
import logging
import sys
import time
import math
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


def is_close(range):
    MIN_DISTANCE = 0.2  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE

class MyCrazyFlieClient:

    def __init__(self):
        cflib.crtp.init_drivers(enable_debug_driver=False)
        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(URI, cf=cf) as scf:
            log_config = LogConfig(name='kalman', period_in_ms=100)
            log_config.add_variable('kalman.stateX', 'float')
            log_config.add_variable('kalman.stateY', 'float')
            logger_pos = SyncLogger(scf, log_config)
            log_config_2 = LogConfig(name='stabilizer', period_in_ms=100)
            log_config_2.add_variable('stabilizer.yaw', 'float')
            logger_orientation = SyncLogger(scf, log_config_2)
            self.home_pos = self.get_position(logger_pos)
            self.orientation = self.get_orientation(logger_orientation)
            print(self.home_pos)
            print(self.orientation)


    def get_position(self, logger):
        logger.connect()
        data = logger.next()[1]
        logger.disconnect()
        logger._queue.empty()
        return data['kalman.stateX'], data['kalman.stateY']

    def get_orientation(self, logger):
        logger.connect()
        data = logger.next()[1]
        logger.disconnect()
        logger._queue.empty()
        return data['stabilizer.yaw']


#     @staticmethod
#     def toEulerianAngle(q):
#         z = q.z_val
#         y = q.y_val
#         x = q.x_val
#         w = q.w_val
#         ysqr = y * y
#
#         # roll (x-axis rotation)
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + ysqr)
#         roll = math.atan2(t0, t1)
#
#         # pitch (y-axis rotation)
#         t2 = +2.0 * (w * y - z * x)
#         if (t2 > 1.0):
#             t2 = 1
#         if (t2 < -1.0):
#             t2 = -1.0
#         pitch = math.asin(t2)
#
#         # yaw (z-axis rotation)
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (ysqr + z * z)
#         yaw = math.atan2(t3, t4)
#
#         return pitch, roll, yaw
#
#     def getPitchRollYaw(self):
#         return self.toEulerianAngle(self.client.simGetGroundTruthKinematics().orientation)
#
#     def getPosition(self):
#         return self.client.simGetVehiclePose().position
#
#     def straight(self, duration, speed):
#         pitch, roll, yaw = self.getPitchRollYaw()
#         vx = math.cos(yaw) * speed
#         vy = math.sin(yaw) * speed
#         self.client.moveByVelocityZAsync(vx, vy, self.z, duration, DrivetrainType.ForwardOnly).join()
#         start = time.time()
#         return start, duration
#
#     def yaw_right(self, duration):
#         self.client.rotateByYawRateAsync(30, duration).join()
#         start = time.time()
#         return start, duration
#
#     def yaw_left(self, duration):
#         self.client.rotateByYawRateAsync(-30, duration).join()
#         start = time.time()
#         return start, duration
#
#
#     def stop(self):
#         self.client.moveByVelocityAsync(0, 0, 0, 0.01).join()
#         self.client.rotateByYawRateAsync(0, 0.01).join()
#
#     def take_action(self, action):
#         with MotionCommander(scf) as motion_commander:
#         # with Multiranger(scf) as multiranger:
#         # print(self.client.simGetObjectPose(object_name='Cone'))
#         # print(self.client.simGetMeshPositionVertexBuffers())
#
#         # check if copter is on level cause sometimes he goes up without a reason
#         # x = 0
#         # while self.getPosition().z_val < -7.0:
#         #     self.client.moveToZAsync(-3, 3)
#         #     time.sleep(0.2)
#         #     print(self.getPosition().z_val, "and", x)
#         #     x = x + 1
#         #     if x > 10:
#         #         return True
#
#         start = time.time()
#         duration = 0
#
#         collided = False
#
#         if action == 0:
#
#             start, duration = self.straight(1, 4)
#
#             while duration > time.time() - start:
#                 if self.client.simGetCollisionInfo().has_collided:
#                     return True
#             self.stop()
#
#         if action == 1:
#
#             start, duration = self.yaw_right(0.8)
#
#             while duration > time.time() - start:
#                 if self.client.simGetCollisionInfo().has_collided:
#                     return True
#             self.stop()
#
#         if action == 2:
#
#             start, duration = self.yaw_left(1)
#
#             while duration > time.time() - start:
#                 if self.client.simGetCollisionInfo().has_collided:
#                     return True
#             self.stop()
#
#         return collided
#
#     def goal_direction(self, goal, pos):
#
#         pitch, roll, yaw = self.getPitchRollYaw()
#         yaw = math.degrees(yaw)
#
#         pos_angle = math.atan2(goal[1] - pos.y_val, goal[0] - pos.x_val)
#         pos_angle = math.degrees(pos_angle) % 360
#
#         track = math.radians(pos_angle - yaw)
#
#         return ((math.degrees(track) - 180) % 360) - 180
#
#     def get_state_from_sim(self, track, distance, now):
#         with Multiranger(scf) as multiranger:
#             pass
#         print(self.client.getDistanceSensorData(distance_sensor_name = "Distance",
#                                                 vehicle_name = "SimpleFlight").distance)
#
#         return (now.x_val, now.y_val, track, distance, (self.client.getDistanceSensorData(distance_sensor_name="Distance",
#                                                                     vehicle_name="SimpleFlight").distance))
#
#     def AirSim_reset(self):
#
#         self.client.reset()
#         time.sleep(0.2)
#         self.client.enableApiControl(True)
#         self.client.armDisarm(True)
#         time.sleep(1)
#         self.client.moveToZAsync(self.z, 3).join()
#         time.sleep(1)
#
#
#
#
# # def straight(duration, speed):
# #     motion_commander.start_linear_motion(v_x, v_y)
#
#
# # def perform_action(action):
# #     if action == 0:
# #         Crazyflie.
#



if __name__ == '__main__':
    crazyclient = MyCrazyFlieClient()
    pass
    # Initialize the low-level drivers (don't list the debug drivers)
    # cflib.crtp.init_drivers(enable_debug_driver=False)
    #
    # cf = Crazyflie(rw_cache='./cache')
    # with SyncCrazyflie(URI, cf=cf) as scf:
        # log_config = LogConfig(name='controller', period_in_ms=2000)
        # log_config.add_variable('controller.pitch', 'float')
        # log_config.add_variable('controller.yaw', 'float')
        # log_config.add_variable('kalman_states.statePZ', 'float')
        # log_config = LogConfig(name='Kalman Position', period_in_ms=500)
        # log_config.add_variable('kalman.varPX', 'float')
        # log_config.add_variable('kalman.varPY', 'float')
        # log_config.add_variable('kalman.varPZ', 'float')
        # with SyncLogger(scf, log_config) as logger:
        #     for log_entry in logger:
        #         data = log_entry[1]
        #         print(data)
        # with MotionCommander(scf) as motion_commander:
        #     with Multiranger(scf) as multiranger:
        #         keep_flying = True
        #
        #         while keep_flying:
        #             VELOCITY = 0.5
        #             velocity_x = 0.0
        #             velocity_y = 0.0
        #
        #             if is_close(multiranger.front):
        #                 velocity_x -= VELOCITY
        #             if is_close(multiranger.back):
        #                 velocity_x += VELOCITY
        #
        #             if is_close(multiranger.left):
        #                 velocity_y -= VELOCITY
        #             if is_close(multiranger.right):
        #                 velocity_y += VELOCITY
        #
        #             if is_close(multiranger.up):
        #                 keep_flying = False
        #
        #             motion_commander.start_linear_motion(
        #                 velocity_x, velocity_y, 0)
        #
        #             time.sleep(0.1)
        #
        #     print('Demo terminated!')
