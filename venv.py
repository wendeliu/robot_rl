import math
import numpy as np
from Robot import realAgent

class bot():
    
    def __init__(self):
        self.theta_threshold_radians = 8
        self.agent=realAgent()
        self.agent.sysInit()
        self.state=[0,0]
        self.steps_beyond_terminated=0
        self.count =0
        self.step_threshold=5000
    def step(self,input):
    
        if input >0.5:
            self.agent.impulse(100)
        else:
            self.agent.impulse(-100)

        self.state=self.agent.getState()
        theta = self.state[0]
        self.count += 1
        
        terminated = bool(
                #x < -self.x_threshold
                #or x > self.x_threshold
                theta < -self.theta_threshold_radians
                or theta > self.theta_threshold_radians
                or self.count >= self.step_threshold
            )
        if not terminated:
            reward = 1.0
        elif self.steps_beyond_terminated is None:
            # Pole just fell!
            self.steps_beyond_terminated = 0
            reward = 1.0
        else:
            self.steps_beyond_terminated += 1
            reward = 0.0
        
        return np.array(self.state, dtype=np.float32), reward, terminated, False, {}
