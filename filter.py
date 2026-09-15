import numpy as np

class LowPassFilter:
    def __init__(self,dt,f_corte, initial_value=0.0):
        self.dt = dt
        self.f_corte = f_corte
        self.state = initial_value
        if self.f_corte <= 0:
            tau = 1/(2*np.pi*self.f_corte)
            self.alpha = self.dt/(tau+self.dt)
        else:
            self.alpha = 1.0
            
    def update(self,measure):
        if self.f_corte <= 0:
            self.state = measure
            return self.state
        self.state = self.state + self.alpha*(measure-self.state)
        return self.state
    
    def reset(self,value=0.0):
        self.state = value