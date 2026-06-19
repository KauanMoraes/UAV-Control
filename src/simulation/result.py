# simulation/result.py

from dataclasses import dataclass

import numpy as np


@dataclass
class SimulationResult:
    """
    Container for simulation outputs.

    Attributes
    ----------
    time : np.ndarray
        Time vector.

    states : np.ndarray
        State history with shape (n_states, N).

    controls : np.ndarray
        Control history with shape (N, n_controls).
    """

    time: np.ndarray

    states: np.ndarray

    controls: np.ndarray

    @property
    def position(self) -> np.ndarray:
        return self.states[0:3]

    @property
    def velocity(self) -> np.ndarray:
        return self.states[3:6]

    @property
    def attitude(self) -> np.ndarray:
        return self.states[6:9]

    @property
    def angular_velocity(self) -> np.ndarray:
        return self.states[9:12]