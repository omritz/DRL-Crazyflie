from DQN_agent import Agent
import numpy as np
import gym
from mpl_toolkits.mplot3d import Axes3D
from Utils import plotLearning
from tensorflow.keras.models import load_model
import tensorflow as tf
import gym_airsim.envs
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import json


def plot_path(path, goal, game):
    path = np.array(path)
    # sns.set(style="darkgrid")
    sns.set_style("whitegrid", {'axes.grid': False})
    x = path[:, 0]
    y = path[:, 1]
    z = path[:, 2]
    fig = plt.figure(figsize=(6, 6))
    # plt.style.use('seaborn-darkgrid')
    ax = Axes3D(fig)  # Method 1
    ax.scatter(x, y, z, marker='o')
    ax.scatter(goal[0], goal[1], z[-1], marker='o', s=400)
    ax.scatter(0, 0, z[0], marker='o', s=250, c='green')
    ax.set_xticks(np.arange(-4, 18, 2))
    ax.set_yticks(np.arange(-12, 10, 2))
    ax.set_zticks([-2, 0, 2])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.savefig('plots/path_%s' % game)
    plt.clf()
    # plt.show()


def plot_dones(done_array, game):
    sns.set(style="darkgrid")
    # Create the data
    x = [k+1 for k in range(game+1)]
    # running_avg = np.empty(episodes)
    # for t in range(episodes):
    #     running_avg[t] = np.mean(done_array[max(0, t - 20):(t + 1)])
    y = ['red' if done_array[k] == 0 else 'green' for k in range(game+1)]
    data = pd.DataFrame(data={'Game': x, 'Done': done_array})
    # Initialize figure and ax
    print(data)
    kwargs = {'faceColor': y, 's': 100}
    sns.scatterplot(x='Game', y='Done', data=data, **kwargs)
    plt.savefig('plots/dones_%s' % str(game))
    plt.clf()


def plot_learning(scores, epsilons):
    fig, ax = plt.subplots()
    ax1 = ax.twinx()
    x = [i+1 for i in range(len(scores))]
    running_avg = np.empty(len(scores))
    for t in range(len(scores)):
        running_avg[t] = np.mean(scores[max(0, t - 20):(t + 1)])
    data1 = pd.DataFrame(data={'Game': x, 'Score': running_avg, 'epsilon': epsilons})
    data2 = pd.DataFrame(data={'epsilon': epsilons})
    sns.lineplot(x='Game', y='Score', color='C1', data=data1,ax=ax)
    sns.lineplot(data=data2['epsilon'], color="C0", ax=ax1)
    ax.set_ylabel("Score", color="C1")
    ax1.set_ylabel("Epsilon", color="C0")
    ax1.tick_params(axis='y', colors="C0")
    ax.tick_params(axis='y', colors="C1")
    plt.savefig('plots/learning_%s' % str(len(scores)))
    plt.clf()


def plot_success_rate(dones, game):
    sns.set(style="darkgrid")
    # Create the data
    x = [k + 1 for k in range(game+1)]
    running_avg = np.empty(game+1)
    for t in range(game+1):
        running_avg[t] = np.mean(dones[max(0, t - 20):(t + 1)])
    data = pd.DataFrame(data={'Episode': x, 'Success Rate': running_avg})
    # Initialize figure and ax
    sns.lineplot(x='Episode', y='Success Rate', color='C0', data=data)
    plt.savefig('plots/Success_rate_%s' % str(game))
    plt.clf()


if __name__ == '__main__':
    tf.compat.v1.disable_eager_execution()
    env_name = 'AirSimEnv-v42'
    env = gym.make(env_name)
    lr = 0.0005
    n_games = 3000
    agent = Agent(gamma=0.99, epsilon=0.746, lr=lr, input_dims=env.observation_space.shape,
                  n_actions=env.action_space.n, mem_size=100000, batch_size=64,
                  epsilon_end=0.01, fname='_New_R_target.h5')
    # scores = []
    # eps_history = []
    # dones = []
    with open('data.json') as json_data:
        data = json.load(json_data)
    # data = {'eps_history': [], 'scores': [], 'dones': []}
    print(data)
    scores = data['scores']
    eps_history = data['eps_history']
    dones = data['dones']
    '''---------------------------------------evaluate main------------------------------------------'''
    #
    # model = load_model('models/AirSimEnv-v42_10000Ep.h5')
    # path_array = []
    # goals = []
    # for i in range(n_games):
    #     path = []
    #     done = False
    #     score = 0
    #     observation = env.reset()
    #     goals.append(env.goal)
    #     print(observation)
    #     while not done:
    #         state = np.array([observation])
    #         print(state)
    #         actions = model.predict(state)
    #         action = np.argmax(actions)
    #         observation_, reward, done, info = env.step(action)
    #         if reward == 100:
    #             dones[i] = 1
    #         score += reward
    #         observation = observation_
    #         path.append(env.get_path())
    #     scores.append(score)
    #     avg_score = np.mean(scores[-100:])
    #     path_array.append(path)
    # print(path_array)
    # # plot_path(path_array, env.goal)
    # for i in range(len(dones)):
    #     if dones[i]:
    #         plot_path(path_array[i], goals[i], i)
    # plot_dones(n_games, dones)
    # print('Success rate is: %s' % ((dones/n_games)*100))
    '''-------------------------------Training main--------------------------------------------'''
    #
    latest = tf.train.latest_checkpoint('checkpoints')
    print(latest)
    agent.load_weights(latest)
    for i in range(254, n_games, 1):
        agent.epsilon = agent.epsilon - agent.eps_dec if agent.epsilon > agent.eps_min else agent.eps_min
        done = False
        score = 0
        observation = env.reset()
        while not done:
            action = agent.choose_action(observation)
            observation_, reward, done, info = env.step(action)
            score += reward
            if done and reward > 500:
                dones.append(1)
            elif done:
                dones.append(0)
            agent.store_transition(observation, action, reward, observation_, done)
            observation = observation_
            agent.learn()
            print('Reward for this action: %s' % reward)
            print('The action taken: %s' % action)
            print('---------------------------------------------------------------------------------------------------')
        if i % 10 == 0:
            agent.hard_update()
        eps_history.append(agent.epsilon)
        scores.append(score)
        avg_score = np.mean(scores[-100:])
        print('episode: ', i, 'score %.2f' % score,
              'average_score %.2f' % avg_score,
              'epsilon %.2f' % agent.epsilon)
        # agent.save_weights('ckpt_New_R_target_net')
        if i % 50 == 0:
            agent.save_weights(str(i) + '_episodes_New_R_target')
            agent.save_model(str(i))
            plot_learning(scores, eps_history)
            plot_success_rate(dones, i)
        data['eps_history'] = eps_history
        data['scores'] = scores
        data['dones'] = dones
        with open('data.json', 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=4)
    agent.save_weights(str(i) + '_episodes_New_R_target')
    agent.save_model(str(i))
    filename = env_name + '.png'
    x = [i+1 for i in range(n_games)]
    plotLearning(x, scores, eps_history, '3000-_episodes_New_R_target')
    plot_learning(scores, eps_history)
    plot_dones(dones, i)
    plot_success_rate(dones, i)
