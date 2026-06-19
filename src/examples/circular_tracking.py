from drone.default_quadrotor import (
    create_default_quadrotor
)

from trajectories.circular import (
    CircularTrajectory
)

from controllers.position_controller import (
    PositionController
)

from controllers.attitude_controller import (
    AttitudeController
)

from controllers.altitude_controller import (
    AltitudeController
)

from simulation.simulation import (
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