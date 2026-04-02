"""Entry point for the modular lidar package."""

try:
    from .lidar_gui import run_gui
except ImportError:
    from lidar_gui import run_gui


def run() -> None:
    """Start the lidar graphical interface."""
    run_gui()


if __name__ == "__main__":
    run()
