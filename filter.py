import numpy as np

class LowPassFilter:
    def __init__(self,dt,f_corte, initial_value=0.0):
        self.dt = dt
        self.f_corte = f_corte
        self.state = initial_value
    def update(self,measure):
        tau = 1/(2*np.pi*self.f_corte)
        alpha = self.dt/(tau+self.dt)
        self.state = self.state + alpha*(measure-self.state)
        return self.state
    def reset(self,value=0.0):
        self.state = value