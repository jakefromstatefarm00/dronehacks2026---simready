# policies/

Trained networks the node can load (`policy:=<file>.npz` or `policy: <file>.npz` in
config/mission.yaml). Empty on purpose: no policy ships with the package. Until you have trained
one that beats the baseline, fly `policy:=carrot`.

Make one (README section 7):

    python -m team_mission.rl.train --out runs/v1
    python -m team_mission.rl.evaluate carrot --episodes 200
    python -m team_mission.rl.evaluate runs/v1/policy.npz --episodes 200     # same seed: must beat carrot
    cp runs/v1/policy.npz policies/v1.npz                                     # or: python -m team_mission.rl.export runs/v1/best_model.zip policies/v1.npz
    # on the drone / in Gazebo: rebuild (team_build.sh / colcon build), then policy:=v1.npz

Each .npz carries its contract version (OBS_VERSION in control.py) and the speed limits it was
trained with; the node refuses a wrong version and warns about different limits.
