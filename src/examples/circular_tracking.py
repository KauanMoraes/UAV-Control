from src.drone.default_quadrotor import (
    create_default_quadrotor
)

from src.trajectories.circular import (
    CircularTrajectory
)

from src.controllers.position_controller import (
    PositionController
)

from src.controllers.attitude_controller import (
    AttitudeController
)

from src.controllers.altitude_controller import (
    AltitudeController
)

from src.simulation.simulation import (
    Simulation
)

drone = create_default_quadrotor()

trajectory = CircularTrajectory()

position_controller = (
    PositionController(
        gravity=9.81
    )
)

attitude_controller = (
    AttitudeController()
)

altitude_controller = (
    AltitudeController(
        drone.parameters
    )
)

simulation = Simulation(
    drone=drone,

    trajectory=trajectory,

    position_controller=position_controller,

    attitude_controller=attitude_controller,

    altitude_controller=altitude_controller,

    t_final=25
)

result = simulation.run()

print(result.__dict__.keys())