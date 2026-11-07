"""Jolie's planner as a flight program on the Sky Beta contract (run as jolie_course).

    fly_pathplan.py  the ROS node (/visual_slam/tracking/odometry + /skybeta/state in,
                     /skybeta/setpoint/pose out; no /fmu)
    mission.py       the mission state machine (pure Python, unit-tested offline)
    px4_link.py      frames and constants (pure Python, unit-tested offline)
"""
