import airsim # sudo pip install airsim
import pprint
import numpy as np
import math
import time
import cv2
# import settings

from PIL import Image
# from pylab import array, uint8, arange

# import msgs


class AirLearningClient(airsim.MultirotorClient):
    def __init__(self):

        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient("127.0.0.1")
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        # self.home_pos = self.client.getPosition()
        # self.home_ori = self.client.getOrientation()
        # self.z = -4
    #
    # def goal_direction(self, goal, pos):
    #
    #     pitch, roll, yaw = self.client.getPitchRollYaw()
    #     yaw = math.degrees(yaw)
    #
    #     pos_angle = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
    #     pos_angle = math.degrees(pos_angle) % 360
    #
    #     track = math.radians(pos_angle - yaw)
    #
    #     return ((math.degrees(track) - 180) % 360) - 180
    #
    # def getConcatState(self, goal):
    #
    #     now = self.drone_pos()
    #     track = self.goal_direction(goal, now)
    #     encoded_depth = self.getScreenDepthVis(track)
    #     encoded_depth_shape = encoded_depth.shape
    #     encoded_depth_1d = encoded_depth.reshape(1, encoded_depth_shape[0]*encoded_depth_shape[1])
    #
    #     #ToDo: Add RGB, velocity etc
    #     if(settings.position):
    #         pos = self.get_distance(goal)
    #         concat_state = np.concatenate((encoded_depth_1d, pos), axis = None)
    #         concat_state_shape = concat_state.shape
    #         concat_state = concat_state.reshape(1, concat_state_shape[0])
    #         concat_state = np.expand_dims(concat_state, axis=0)
    #     else:
    #         concat_state = encoded_depth_1d
    #
    #     return concat_state
    #
    # def getScreenGrey(self):
    #     responses = self.client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
    #     response = responses[0]
    #     img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    #     if((responses[0].width !=0 or responses[0].height!=0)):
    #         img_rgba = img1d.reshape(response.height, response.width, 4)
    #         rgb = cv2.cvtColor(img_rgba, cv2.COLOR_BGRA2BGR)
    #         grey = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    #     else:
    #         print("Something bad happened! Restting AirSim!")
    #         self.AirSim_reset()
    #         grey = np.ones(144,256)
    #     return grey
    #
    # def getScreenRGB(self):
    #     responses = self.client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])
    #     response = responses[0]
    #     img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
    #     if ((responses[0].width != 0 or responses[0].height != 0)):
    #         img_rgba = img1d.reshape(response.height, response.width, 4)
    #         rgb = cv2.cvtColor(img_rgba, cv2.COLOR_BGRA2BGR)
    #     else:
    #         print("Something bad happened! Restting AirSim!")
    #         self.AirSim_reset()
    #         rgb = np.ones(144, 256, 3)
    #     return rgb
    #
    # def getScreenDepthVis(self, track):
    #
    #     responses = self.client.simGetImages([airsim.ImageRequest(0, airsim.ImageType.DepthPerspective, True, False)])
    #     #responses = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis,True, False)])
    #
    #     if(responses == None):
    #         print("Camera is not returning image!")
    #         print("Image size:"+str(responses[0].height)+","+str(responses[0].width))
    #     else:
    #         img1d = np.array(responses[0].image_data_float, dtype=np.float)
    #
    #     img1d = 255 / np.maximum(np.ones(img1d.size), img1d)
    #     if((responses[0].width!=0 or responses[0].height!=0)):
    #         img2d = np.reshape(img1d, (responses[0].height, responses[0].width))
    #
    #     else:
    #         print("Something bad happened! Restting AirSim!")
    #         self.AirSim_reset()
    #         img2d = np.ones((144, 256))
    #
    #     image = np.invert(np.array(Image.fromarray(img2d.astype(np.uint8), mode='L')))
    #
    #     factor = 10
    #     maxIntensity = 255.0  # depends on dtype of image data
    #
    #     # Decrease intensity such that dark pixels become much darker, bright pixels become slightly dark
    #     newImage1 = (maxIntensity) * (image / maxIntensity) ** factor
    #     newImage1 = array(newImage1, dtype=uint8)
    #
    #     # small = cv2.resize(newImage1, (0,0), fx=0.39, fy=0.38)
    #     small = cv2.resize(newImage1, (0, 0), fx=1.0, fy=1.0)
    #
    #     cut = small[20:40, :]
    #     # print(cut.shape)
    #
    #     info_section = np.zeros((10, small.shape[1]), dtype=np.uint8) + 255
    #     info_section[9, :] = 0
    #
    #     line = np.int((((track - -180) * (small.shape[1] - 0)) / (180 - -180)) + 0)
    #     '''
    #     print("\n")
    #     print("Track:"+str(track))
    #     print("\n")
    #     print("(Track - -180):"+str(track - -180))
    #     print("\n")
    #     print("Num:"+str((track - -180)*(100-0)))
    #     print("Num_2:"+str((track+180)*(100-0)))
    #     print("\n")
    #     print("Den:"+str(180 - -180))
    #     print("Den_2:"+str(180+180))
    #     print("line:"+str(line))
    #     '''
    #     if line != (0 or small.shape[1]):
    #         info_section[:, line - 1:line + 2] = 0
    #     elif line == 0:
    #         info_section[:, 0:3] = 0
    #     elif line == small.shape[1]:
    #         info_section[:, info_section.shape[1] - 3:info_section.shape[1]] = 0
    #
    #     total = np.concatenate((info_section, small), axis=0)
    #     #cv2.imwrite("test.png",total)
    #     # cv2.imshow("Test", total)
    #     # cv2.waitKey(0)
    #     #total = np.reshape(total, (154,256))
    #     return total
    #
    # def drone_pos(self):
    #     x = self.client.getPosition().x_val
    #     y = self.client.getPosition().y_val
    #     z = self.client.getPosition().z_val
    #
    #     return np.array([x, y, z])
    #
    # def drone_velocity(self):
    #     v_x = self.client.getVelocity().x_val
    #     v_y = self.client.getVelocity().y_val
    #     v_z = self.client.getVelocity().z_val
    #
    #     return np.array([v_x, v_y, v_z])
    #
    # def get_distance(self, goal):
    #     now = self.client.getPosition()
    #     xdistance = (goal[0] - now.x_val)
    #     ydistance = (goal[1] - now.y_val)
    #     euclidean = np.sqrt(np.power(xdistance,2) + np.power(ydistance,2))
    #
    #     return np.array([xdistance, ydistance, euclidean])
    #
    # def get_velocity(self):
    #     return np.array([self.client.get_velocity().x_val, self.client.get_velocity().y_val, self.client.get_velocity().z_val])
    #
    # def AirSim_reset(self):
    #     self.client.reset()
    #     time.sleep(0.2)
    #     self.client.enableApiControl(True)
    #     self.client.armDisarm(True)
    #     time.sleep(0.2)
    #     self.client.moveByVelocityAsync(0, 0, -5, 2, drivetrain=0, vehicle_name='').join()
    #     self.client.moveByVelocityAsync(0, 0, 0, 1, drivetrain=0, vehicle_name='').join()
    #
    # def unreal_reset(self):
    #     self.client, connection_established = self.client.resetUnreal()
    #     return connection_established
    #
    # def take_continious_action(self, action):
    #
    #     if(msgs.algo == 'DDPG'):
    #         pitch = action[0]
    #         roll = action[1]
    #         throttle = action[2]
    #         yaw_rate = action[3]
    #         duration = action[4]
    #         self.client.moveByAngleThrottleAsync(pitch, roll, throttle, yaw_rate, duration, vehicle_name='').join()
    #     else:
    #         #pitch = np.clip(action[0], -0.261, 0.261)
    #         #roll = np.clip(action[1], -0.261, 0.261)
    #         #yaw_rate = np.clip(action[2], -3.14, 3.14)
    #         if(settings.move_by_velocity):
    #             vx = np.clip(action[0], -1.0, 1.0)
    #             vy = np.clip(action[1], -1.0, 1.0)
    #             #print("Vx, Vy--------------->"+str(vx)+", "+ str(vy))
    #             #self.client.moveByAngleZAsync(float(pitch), float(roll), -6, float(yaw_rate), settings.duration_ppo).join()
    #             self.client.moveByVelocityZAsync(float(vx), float(vy), -6, 0.5, 1, yaw_mode=airsim.YawMode(True, 0)).join()
    #         elif(settings.move_by_position):
    #             pos = self.drone_pos()
    #             delta_x = np.clip(action[0], -1, 1)
    #             delta_y = np.clip(action[1], -1, 1)
    #             self.client.moveToPositionAsync(float(delta_x + pos[0]), float(delta_y + pos[1]), -6, 0.9, yaw_mode=airsim.YawMode(False, 0)).join()
    #     collided = (self.client.getMultirotorState().trip_stats.collision_count > 0)
    #     collided = (self.client.getMultirotorState().trip_stats.collision_count > 0)
    #
    #     return collided
    #
    #     #Todo : Stabilize drone
    #     #self.client.moveByAngleThrottleAsync(0, 0,1,0,2).join()
    #
    #     #TODO: Get the collision info and use that to reset the simuation.
    #     #TODO: Put some sleep in between the calls so as not to crash on the same lines as DQN
    # def straight(self, speed, duration):
    #     pitch, roll, yaw  = self.client.getPitchRollYaw()
    #     vx = math.cos(yaw) * speed
    #     vy = math.sin(yaw) * speed
    #     self.client.moveByVelocityZAsync(vx, vy, self.z, duration, 1).join()
    #     start = time.time()
    #     return start, duration
    #
    # def backup(self, speed, duration):
    #     pitch, roll, yaw = self.client.getPitchRollYaw()
    #     vx = math.cos(yaw) * speed * -0.5
    #     vy = math.sin(yaw) * speed * -0.5
    #     self.client.moveByVelocityZAsync(-vx, -vy, self.z, duration, 0).join()
    #     start = time.time()
    #     return start, duration
    #
    # def yaw_right(self, rate, duration):
    #     self.client.rotateByYawRateAsync(rate, duration).join()
    #     print("yaw_right, rate:" + str(rate) + " duration:"+ str(duration))
    #     start = time.time()
    #     return start, duration
    #
    # def yaw_left(self, rate, duration):
    #     print("yaw_left, rate:" + str(rate) + " duration:"+ str(duration))
    #     self.client.rotateByYawRateAsync(-rate, duration).join()
    #     start = time.time()
    #     return start, duration
    #
    # def pitch_up(self, duration):
    #     self.client.moveByVelocityZAsync(0.5,0,1,duration,0).join()
    #     start = time.time()
    #     return start, duration
    #
    # def pitch_down(self, duration):
    #     self.client.moveByVelocityZAsync(0.5,0.5,self.z,duration,0).join()
    #     start = time.time()
    #     return start, duration
    #
    # def move_forward_Speed(self, speed_x = 0.5, speed_y = 0.5, duration = 0.5):
    #     pitch, roll, yaw  = self.client.getPitchRollYaw()
    #     vel = self.client.getVelocity()
    #     vx = math.cos(yaw) * speed_x - math.sin(yaw) * speed_y
    #     vy = math.sin(yaw) * speed_x + math.cos(yaw) * speed_y
    #     if speed_x <= 0.01:
    #         drivetrain = 0
    #         #yaw_mode = YawMode(is_rate= True, yaw_or_rate = 0)
    #     elif speed_x > 0.01:
    #         drivetrain = 0
    #         #yaw_mode = YawMode(is_rate= False, yaw_or_rate = 0)
    #     self.client.moveByVelocityZAsync(vx = (vx +vel.x_val)/2 ,
    #                          vy = (vy +vel.y_val)/2 , #do this to try and smooth the movement
    #                          z = self.z,
    #                          duration = duration,
    #                          drivetrain = drivetrain,
    #                         ).join()
    #     start = time.time()
    #     return start, duration
    #
    # def take_discrete_action(self, action):
    #
    #    # check if copter is on level cause sometimes he goes up without a reason
    #     """
    #     x = 0
    #     while self.client.getPosition().z_val < -7.0:
    #         self.client.moveToZAsync(-6, 3).join()
    #         time.sleep(1)
    #         print(self.client.getPosition().z_val, "and", x)
    #         x = x + 1
    #         if x > 10:
    #             return True
    #     start = time.time()
    #     duration = 0
    #     """
    #
    #     if action == 0:
    #         start, duration = self.straight(settings.mv_fw_spd_5, settings.mv_fw_dur)
    #     if action == 1:
    #         start, duration = self.straight(settings.mv_fw_spd_4, settings.mv_fw_dur)
    #     if action == 2:
    #         start, duration = self.straight(settings.mv_fw_spd_3, settings.mv_fw_dur)
    #     if action == 3:
    #         start, duration = self.straight(settings.mv_fw_spd_2, settings.mv_fw_dur)
    #     if action == 4:
    #         start, duration = self.straight(settings.mv_fw_spd_1, settings.mv_fw_dur)
    #     if action == 5:
    #         start, duration = self.move_forward_Speed(settings.mv_fw_spd_5, settings.mv_fw_spd_5, settings.mv_fw_dur)
    #     if action == 6:
    #         start, duration = self.move_forward_Speed(settings.mv_fw_spd_4, settings.mv_fw_spd_4, settings.mv_fw_dur)
    #     if action == 7:
    #         start, duration = self.move_forward_Speed(settings.mv_fw_spd_3, settings.mv_fw_spd_3, settings.mv_fw_dur)
    #     if action == 8:
    #         start, duration = self.move_forward_Speed(settings.mv_fw_spd_2, settings.mv_fw_spd_2, settings.mv_fw_dur)
    #     if action == 9:
    #         start, duration = self.move_forward_Speed(settings.mv_fw_spd_1, settings.mv_fw_spd_1, settings.mv_fw_dur)
    #     if action == 10:
    #         start, duration = self.backup(settings.mv_fw_spd_5, settings.mv_fw_dur)
    #     if action == 11:
    #         start, duration = self.backup(settings.mv_fw_spd_4, settings.mv_fw_dur)
    #     if action == 12:
    #         start, duration = self.backup(settings.mv_fw_spd_3, settings.mv_fw_dur)
    #     if action == 13:
    #         start, duration = self.backup(settings.mv_fw_spd_2, settings.mv_fw_dur)
    #     if action == 14:
    #         start, duration = self.backup(settings.mv_fw_spd_1, settings.mv_fw_dur)
    #     if action == 15:
    #         start, duration = self.yaw_right(settings.yaw_rate_1_1, settings.rot_dur)
    #     if action == 16:
    #         start, duration = self.yaw_right(settings.yaw_rate_1_2, settings.rot_dur)
    #     if action == 17:
    #         start, duration = self.yaw_right(settings.yaw_rate_1_4, settings.rot_dur)
    #     if action == 18:
    #         start, duration = self.yaw_right(settings.yaw_rate_1_8, settings.rot_dur)
    #     if action == 19:
    #         start, duration = self.yaw_right(settings.yaw_rate_1_16, settings.rot_dur)
    #     if action == 20:
    #         start, duration = self.yaw_right(settings.yaw_rate_2_1, settings.rot_dur)
    #     if action == 21:
    #         start, duration = self.yaw_right(settings.yaw_rate_2_2, settings.rot_dur)
    #     if action == 22:
    #         start, duration = self.yaw_right(settings.yaw_rate_2_4, settings.rot_dur)
    #     if action == 23:
    #         start, duration = self.yaw_right(settings.yaw_rate_2_8, settings.rot_dur)
    #     if action == 24:
    #         start, duration = self.yaw_right(settings.yaw_rate_2_16, settings.rot_dur)
    #
    #
    #     collided = (self.client.getMultirotorState().trip_stats.collision_count > 0)
    #     #print("collided:", self.client.getCollisionInfo().has_collided," timestamp:" ,self.client.getCollisionInfo().time_stamp)
    #     #print("collided " + str(self.client.getMultirotorState().trip_stats.collision_count > 0))
    #
    #     return collided



import airsim
import time
import numpy as np
import os
import tempfile
import pprint
import cv2

# connect to the AirSim simulator
client = airsim.MultirotorClient("127.0.0.1")
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)
#
# imu_data = client.getImuData()
# s = pprint.pformat(imu_data)
# print("imu_data: %s" % s)

# barometer_data = client.getBarometerData()
# s = pprint.pformat(barometer_data)
# print("barometer_data: %s" % s)

magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)

gps_data = client.getGpsData(gps_name="Gps", vehicle_name = "SimpleFlight")
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

s = client.getDistanceSensorData(distance_sensor_name = "Distance", vehicle_name = "SimpleFlight")
print("distance_data: %s" % s)
# airsim.wait_key('Press any key to takeoff')
client.takeoffAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPositionAsync(-10, 10, -10, 5).join()

client.hoverAsync().join()
s = client.getDistanceSensorData(distance_sensor_name = "Distance", vehicle_name = "SimpleFlight")
print("distance_data: %s" % s)
state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

# airsim.wait_key('Press any key to take images')
# get camera images from the car
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
    airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
print('Retrieved images: %d' % len(responses))

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

# airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
# if __name__ == '__main__':
#     client = AirLearningClient()
#     state = client.getMultirotorState()
#     s = pprint.pformat(state)
#     print("state: %s" % s)