"""SIMULATION-ONLY helpers (Gazebo / flyx lab). They refuse to start on the drone."""
import os


def refuse_on_drone(node_name: str, override: bool = False):
    """The Jetson's flight-controller UART exists only on the real drone."""
    if os.path.exists('/dev/ttyTHS1') and not override:
        raise SystemExit(f'{node_name} is SIMULATION ONLY and /dev/ttyTHS1 exists (this looks like the drone). '
                         'Refusing to start.')
