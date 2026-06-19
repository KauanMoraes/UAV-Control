# src/visualization/attitude.py

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from src.simulation.result import (
    SimulationResult
)


def plot_attitude(
    result: SimulationResult,
    save_path: str | None = None
):

    phi = np.rad2deg(
        result.attitude[0]
    )

    theta = np.rad2deg(
        result.attitude[1]
    )

    psi = np.rad2deg(
        result.attitude[2]
    )

    fig, ax = plt.subplots(
        figsize=(10, 6)
    )

    ax.plot(
        result.time,
        phi,
        label="Roll"
    )

    ax.plot(
        result.time,
        theta,
        label="Pitch"
    )

    ax.plot(
        result.time,
        psi,
        label="Yaw"
    )

    ax.set_xlabel(
        "Time [s]"
    )

    ax.set_ylabel(
        "Angle [deg]"
    )

    ax.set_title(
        "Attitude"
    )

    ax.grid(True)

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