import seaborn as sns
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import gym_airsim.envs
import gym
from DQN_agent import Agent
import tensorflow as tf


def plot_learning(games, scores, epsilons):
    fig, ax = plt.subplots()
    ax1 = ax.twinx()
    x = [i+1 for i in range(games)]
    running_avg = np.empty(games)
    for t in range(games):
        running_avg[t] = np.mean(scores[max(0, t - 20):(t + 1)])
    data1 = pd.DataFrame(data={'Game': x, 'Score': running_avg, 'epsilon': epsilons})
    data2 = pd.DataFrame(data={'epsilon': epsilons})
    sns.lineplot(x='Game', y='Score', color='C1', data=data1,ax=ax)
    sns.lineplot(data=data2['epsilon'], color="C0", ax=ax1)
    ax.set_ylabel("Score", color="C1")
    ax1.set_ylabel("Epsilon", color="C0")
    ax1.tick_params(axis='y', colors="C0")
    ax.tick_params(axis='y', colors="C1")

    # Initialize figure and ax

    plt.savefig('plots/learning')


def plot_success_rate(dones, games):
    sns.set(style="darkgrid")
    # Create the data
    x = [i + 1 for i in range(games)]
    running_avg = np.empty(games)
    for t in range(games):
        running_avg[t] = np.mean(dones[max(0, t - 20):(t + 1)])
    data = pd.DataFrame(data={'Episode': x, 'Success Rate': running_avg})
    # Initialize figure and ax
    print(data)
    sns.lineplot(x='Episode', y='Success Rate', color='C0', data=data)
    plt.savefig('plots/Success_rate')
    plt.clf()

def test():
    x=[1,2,3]
    print(x[:2])

if __name__ == '__main__':
    # for i in range(10):
    #     test()
    tf.compat.v1.disable_eager_execution()
    env_name = 'AirSimEnv-v42'
    env = gym.make(env_name)
    lr = 0.0005
    n_games = 4000
    agent = Agent(gamma=0.99, epsilon=0.1, lr=lr, input_dims=env.observation_space.shape,
                  n_actions=env.action_space.n, mem_size=100000, batch_size=64,
                  epsilon_end=0.01, fname='_New_R.h5')
    scores = []
    eps_history = []
    dones = np.zeros(n_games)
    observation = env.reset()
    action = agent.choose_action(observation)
    observation_, reward, done, info = env.step(action)
    # games = 50
#     scores = np.random.choice([1, 0],50)
#     epsilon = np.linspace(1, 0.1)
#     print(scores)
#     plot_success_rate(scores, games)
#     pass
# # fig, ax1 = plt.subplots()
#
# ax2 = ax1.twinx()
#
# sns.barplot(x="Announced Year", y="Amount Awarded", data=df, ax=ax2, alpha=.5)
# sns.scatterplot(x=np.arange(0,len(df)), y="Number of Awarded", data=df, ax=ax1)
#
# fig.tight_layout()  # otherwise the right y-label is slightly clipped
#
# plt.title('2016 to 2019 Announcements')