# src/visualization/trajectory.py

from pathlib import Path
import matplotlib.pyplot as plt
from src.simulation.result import SimulationResult


def plot_xy_trajectory(
    result: SimulationResult,
    save_path: str | None = None
):

    x = result.position[0]
    y = result.position[1]

    fig, ax = plt.subplots(
        figsize=(8, 8)
    )

    ax.plot(
        x,
        y,
        label="Drone"
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax.set_title(
        "XY Trajectory"
    )

    ax.grid(True)

    ax.axis("equal")

    ax.legend()

    if save_path:

        Path(
            save_path
        ).parent.mkdir(
            parents=True,
            exist_ok=True
        )

        fig.savefig(
            save_path,
            dpi=300,
            bbox_inches="tight"
        )

    return fig