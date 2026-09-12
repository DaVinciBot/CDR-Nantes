"""Entry point of the vision package: the LiDAR debug GUI.

    cd robot1/rasp && python -m vision
"""

from .gui import run_gui


def run() -> None:
    """Start the lidar graphical interface."""
    run_gui()


if __name__ == "__main__":
    run()
