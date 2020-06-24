# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from tensorflow.keras.models import load_model
import tensorflow as tf
import myCrazyFlieClient
import time


def choose_action(observation):
    global model
    observation = np.array([observation])
    actions = model.predict(observation)
    action = np.argmax(actions)

    return action


if __name__ == '__main__':
    tf.compat.v1.disable_eager_execution()
    goal = [2.1, -2.1]
    model = load_model('models/1000 episodes.h5')
    client = myCrazyFlieClient.MyCrazyFlieClient()
    done = False
    try:
        while not done:
            state = client.observe(goal)
            action = choose_action(state)
            client.take_action(action)
            done = client.check_if_in_target(goal)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('dsadfsdfsdfsdfdsfdsf')
        client.land()




