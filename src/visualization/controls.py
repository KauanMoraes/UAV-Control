# src/visualization/controls.py

from pathlib import Path

import matplotlib.pyplot as plt

from src.simulation.result import (
    SimulationResult
)


def plot_controls(
    result: SimulationResult,
    save_path: str | None = None
):

    u = result.controls

    fig, ax = plt.subplots(
        figsize=(10, 6)
    )

    ax.plot(
        u[:, 0],
        label="Thrust"
    )

    ax.plot(
        u[:, 1],
        label="Tau Phi"
    )

    ax.plot(
        u[:, 2],
        label="Tau Theta"
    )

    ax.plot(
        u[:, 3],
        label="Tau Psi"
    )

    ax.set_title(
        "Control Inputs"
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