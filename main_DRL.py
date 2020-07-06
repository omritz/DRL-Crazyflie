from DQN_agent import Agent
import numpy as np
import gym
from Utils import plotLearning
from tensorflow.keras.models import load_model
import tensorflow as tf
import gym_airsim.envs
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

def plot_dones(episodes,done_array):
    sns.set(style="darkgrid")
    # Create the data
    x = [i+1 for i in range(episodes)]
    running_avg = np.empty(episodes)
    for t in range(episodes):
        running_avg[t] = np.mean(done_array[max(0, t - 20):(t + 1)])
    y = running_avg
    data = pd.DataFrame(data={'Game': x, 'Done Average': y})
    # Initialize figure and ax

    sns.regplot(x="Game", y="Done Average",
                 data=data, scatter_kws={"color": "green"}, line_kws={"color": "blue"} )
    plt.savefig('plots/dones')


if __name__ == '__main__':
    tf.compat.v1.disable_eager_execution()
    env_name = 'AirSimEnv-v42'
    env = gym.make(env_name)
    lr = 0.0005
    n_games = 5000
    agent = Agent(gamma=0.99, epsilon=1.0, lr=lr, input_dims=env.observation_space.shape,
                  n_actions=env.action_space.n, mem_size=100000, batch_size=64,
                  epsilon_end=0.01, fname=env_name+'.h5')
    scores = []
    eps_history = []
    dones = np.zeros(n_games)
    '''---------------------------------------------------------------------------------'''
    # evaluate main
    #
    # model = load_model('models/AirSimEnv-v42_10000Ep.h5')
    # for i in range(n_games):
    #     done = False
    #     score = 0
    #     observation = env.reset()
    #     print(observation)
    #     while not done:
    #         # action = agent.choose_action(observation)
    #         state = np.array([observation])
    #         print(state)
    #         actions = model.predict(state)
    #         action = np.argmax(actions)
    #         observation_, reward, done, info = env.step(action)
    #         if reward == 100:
    #             dones += 1
    #         score += reward
    #         # agent.store_transition(observation, action, reward, observation_, done)
    #         observation = observation_
    #     # eps_history.append(agent.epsilon)
    #     scores.append(score)
    #     avg_score = np.mean(scores[-100:])
    #     print('episode: ', i, 'score %.2f' % score,
    #           'average_score %.2f' % avg_score,
    #           'epsilon %.2f' )
    # print('Success rate is: %s' % (dones/n_games)*100)
    '''------------------------------------------------------------------------------------'''
    # Training main
    latest = tf.train.latest_checkpoint('checkpoints')
    print(latest)
    agent.load_weights(latest)
    for i in range(n_games):
        done = False
        score = 0
        observation = env.reset()
        while not done:
            action = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            score += reward
            if reward == 100:
                dones[i] = 1
            agent.store_transition(observation, action, reward, observation_, done)
            observation = observation_
            agent.learn()
        eps_history.append(agent.epsilon)
        scores.append(score)
        avg_score = np.mean(scores[-100:])
        print('episode: ', i, 'score %.2f' % score,
              'average_score %.2f' % avg_score,
              'epsilon %.2f' % agent.epsilon)
    agent.save_weights(str(i) + '_episodes_r_-50')
    agent.save_model()
    filename = env_name + '.png'
    x = [i+1 for i in range(n_games)]
    plotLearning(x, scores, eps_history, '5000-episode_r_-50')
    plot_dones(x, dones)
