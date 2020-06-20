import gym
from gym import spaces
from gym.utils import seeding
from gym_airsim.envs.myAirSimClient import *
from airsim.client import *

logger = logging.getLogger(__name__)

airSimClient = myAirSimClient()

class AirSimEnv(gym.Env):

    def __init__(self):
        # x_pos, y_pos, direction to goal, distance to goal, distance_sensor.front
        self.observation_space = spaces.Box(low=-500, high=500, shape=(5,))
        self.state = np.zeros((5,), dtype=np.uint8)

        self.action_space = spaces.Discrete(3)
        self.goal_list = [[9.0, -5.0], [10.0, 6.0], [9.0, 0.0]]
        index = np.random.randint(0, len(self.goal_list) - 1)
        self.goal = self.goal_list[index]  # global xy coordinates

        self.episodeN = 0
        self.stepN = 0
        self.dis = np.sqrt(np.power((self.goal[0] - airSimClient.home_pos.x_val), 2) +
                           np.power((self.goal[1] - airSimClient.home_pos.y_val), 2))
        self.allLogs = {'reward': [0], 'distance': [self.dis], 'track': [-2], 'action': [1]}
        # print(self.allLogs)

        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def computeReward(self, now, track_now, distance):

        distance_now = distance
        distance_before = self.allLogs['distance'][-1]

        r = 0

        if self.stepN > 5 and len(set(self.allLogs['distance'][len(self.allLogs['distance']):len(self.allLogs['distance'])-5:-1])) == 1:
            r = r - 100
        else:
            r = -1 + (distance_before - distance_now)

        return r

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        self.addToLog('action', action)

        self.stepN += 1

        collided = airSimClient.take_action(action)

        now = airSimClient.getPosition()
        # print('X % s' % now.x_val)
        track = airSimClient.goal_direction(self.goal, now)
        print('track % s' % track)
        self.dis = np.sqrt(np.power((self.goal[0] - now.x_val), 2) + np.power((self.goal[1] - now.y_val), 2))
        if collided:
            done = True
            reward = -100.0
        elif collided == 99:
            done = True
            reward = 0.0
        else:
            done = False
            reward = self.computeReward(now, track, self.dis)

        # You made it
        if self.dis < 1:
            done = True
            reward = 100.0

        self.addToLog('reward', reward)
        rewardSum = np.sum(self.allLogs['reward'])
        self.addToLog('distance', self.dis)
        self.addToLog('track', track)

        # Terminate the episode on large cumulative amount penalties,
        # since drone probably got into an unexpected loop of some sort
        if rewardSum < -100:
            done = True

        info = {"x_pos": now.x_val, "y_pos": now.y_val}
        self.state = airSimClient.get_state_from_sim(track, self.dis, now)
        print(info)
        print('Reward for this action: %s'% reward)
        print('Distance to target: %s' % self.dis)
        return self.state, reward, done, info

    def addToLog(self, key, value):
        if key not in self.allLogs:
            self.allLogs[key] = []
        self.allLogs[key].append(value)

    def reset(self):
        """
        Resets the state of the environment and returns an initial observation.

        # Returns
            observation (object): The initial observation of the space. Initial reward is assumed to be 0.
        """
        airSimClient.AirSim_reset()

        self.stepN = 0
        self.episodeN += 1
        self.allLogs = {'reward': [0], 'distance': [self.dis], 'track': [-2], 'action': [1]}
        index = np.random.randint(0, len(self.goal_list) - 1)
        self.goal = self.goal_list[index]  # global xy coordinates
        now = airSimClient.getPosition()
        track = airSimClient.goal_direction(self.goal, now)
        self.dis = np.sqrt(np.power((self.goal[0] - now.x_val), 2) + np.power((self.goal[1] - now.y_val), 2))
        self.state = airSimClient.get_state_from_sim(track, self.dis, now)
        return self.state
