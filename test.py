import numpy as np
import random
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from tensorflow.keras.optimizers import Adam
import gym
from cartpole import CartPoleEnv
from statistics import mean
import os
from venv import bot
import time

class DQN_Agent:

    def __init__(self, observation_space = 1, action_space = 1, exploration_rate = 1, 
                 learning_rate = 0.001):

        self.action_space = action_space
        self.observation_space = observation_space
        self.exploration_rate = exploration_rate
        self.learning_rate = learning_rate
        self.discount_factor = 0.95
        self.batch_size = 64
    
        self.exploration_decay = 0.999
        self.memory_size = 2000
        
        self.RAM = deque(maxlen=int(self.memory_size))
        self.ROM = deque(maxlen=int(self.memory_size*0.1))
        

        self.model = Sequential()
        self.model.add(Dense(64, input_shape=(self.observation_space,), activation="relu", kernel_initializer='he_uniform'))
        self.model.add(Dense(32, activation="relu", kernel_initializer='he_uniform'))
        self.model.add(Dense(16, activation="relu", kernel_initializer='he_uniform'))
        self.model.add(Dense(self.action_space, activation="linear", kernel_initializer='he_uniform'))
        self.model.compile(loss="mse", optimizer=Adam(learning_rate=self.learning_rate))

    # memory of the agent
    def remember(self, state, action, reward, next_state, done):
        self.RAM.append((state, action, reward, next_state, done))
        if len(self.ROM) != self.memory_size*0.1:
          self.ROM.append((state, action, reward, next_state, done))

    # how agent decides to take an action:
    # randomly picks a number between 0 and 1, if less than exploration rate, agent chooses a random action
    # else, agent chooses the one with the largest Q value
    def policy(self, state):
        if np.random.rand() < self.exploration_rate:
            return random.randrange(self.action_space)
        q_values = self.model.predict(state, verbose=0)
        return np.argmax(q_values[0])

    # train the deep Q learning network using past memory
    def experience_replay(self):
        memory = self.RAM + self.ROM
        if len(memory) < self.batch_size:
            return
        batch = random.sample(memory, self.batch_size)
        state = np.zeros((self.batch_size, self.observation_space))
        next_state = np.zeros((self.batch_size, self.observation_space))
        action, reward, done = [], [], []

        for i in range(self.batch_size):
            state[i] = batch[i][0]
            action.append(batch[i][1])
            reward.append(batch[i][2])
            next_state[i] = batch[i][3]
            done.append(batch[i][4])
        # do batch prediction to save speed
        target = self.model.predict(state, verbose=0)
        target_next = self.model.predict(next_state, verbose=0)

        for i in range(self.batch_size):
            # correction on the Q value for the action used
            if done[i]:
                #print(reward[i])
                target[i][action[i]] = reward[i]
            else:
                # Standard - DQN
                # DQN chooses the max Q value among next actions
                # selection and evaluation of action is on the target Q Network
                # Q_max = max_a' Q_target(s', a')
                target[i][action[i]] = reward[i] + self.discount_factor * (np.amax(target_next[i]))

        # Train the Neural Network with batches
        self.model.fit(state, target, batch_size=self.batch_size, verbose=0)
        
        self.exploration_rate *= self.exploration_decay
        self.exploration_rate = max(0.01, self.exploration_rate)

    def save(self):
        cwd = os.getcwd()
        path = os.path.join(cwd, 'DQN_model')
        self.model.save(path)

def train_network():
        
    env = CartPoleEnv(render_mode='human') #gym.make('CartPole-v0', render_mode='human')
        #env = gym.wrappers.RecordVideo(gym.make('CartPole-v0', render_mode='rgb_array'), 
        #                                video_folder=path, episode_trigger = lambda x: x % 2 == 0,
        #                                name_prefix='train')

    env=bot()
    observation_space = 2
    action_space = 2

    agent = DQN_Agent(observation_space, action_space)
    rewards = []
    run = 0
    done = False
    while run < 55000 and done == False:
        run += 1
        #state = env.reset(seed=5)
        state = np.array([0,0])
        state = np.reshape(state, [1, observation_space])
        total_rewards = 0
        #cd 
        startTime=time.time()
        
        while True:
            #env.render()
            time.sleep(0.02)
            action = agent.policy(state)
            #print(action)
            state_next, reward, terminal, info, _ = env.step(action)
            state_next=state_next/180*np.pi

            state_next = np.array([state_next])
            reward = -200*(abs(state_next[0][0]) - abs(state[0][0]))
            state_next = np.reshape(state_next, [1, observation_space])
            agent.remember(state, action, reward, state_next, terminal)
            state = state_next
            total_rewards += 1
                
            if terminal and ((time.time()-startTime)>5):
                verbose = "Episodes: " + str(run) + ", Exploration: " + str(agent.exploration_rate) + ", Score: " + str(total_rewards) + '\n'
                print(verbose)
                rewards.append(total_rewards)
                if run >= 5 and mean(rewards[-5:]) >=495:
                    done = True
                break
            agent.experience_replay()
    agent.save()
    print("Training Completed!")
    env.close()

if __name__ == '__main__':
    train_network()
        
