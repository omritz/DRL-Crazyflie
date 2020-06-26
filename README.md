# DRL-Crazyflie
Deep reinforcement learning with bitcraze crazyflie 2.1.

## Project details:
In this project we implement a DQN algorithem for goal seeking in nano drone from [Bitcraze](https://www.bitcraze.io).

### This implementation contains:

1. Deep Q-network and Q-learning
2. Experience replay memory
    - to reduce the correlations between consecutive updates
3. Network for Q-learning targets are fixed for intervals
    - to reduce the correlations between target and predicted Q-values
