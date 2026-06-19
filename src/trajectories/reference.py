from dataclasses import dataclass

@dataclass(frozen=True)
class TrajectoryReference:

    x: float
    y: float

    vx: float
    vy: float
    # For the future implementations
    z: float = 1.0 
    vz: float = 0.0
    yaw: float = 0.0