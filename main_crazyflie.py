import numpy as np
from tensorflow.keras.models import load_model
import tensorflow as tf
import myCrazyFlieClient


def choose_action(observation):
    global model
    observation = np.array([observation])
    actions = model.predict(observation)
    action = np.argmax(actions)

    return action


if __name__ == '__main__':
    tf.compat.v1.disable_eager_execution()
    goal = [5, 5]
    model = load_model('models/AirSimEnv-v42-210Ep.h5')
    client = myCrazyFlieClient.MyCrazyFlieClient()
    while True:
        state = client.observe(goal)
        action = choose_action(state)
        print('action: %s' % action)
        client.take_action(action)

