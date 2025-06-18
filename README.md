# Minecraft ROS Reinforcement Learning Agent
This repo introduces three executable ROS nodes: the Rewarder, StateFormer, and Teleop nodes. These nodes are instantiated automatically by the algorithm to train a policy, but can be run individually for debugging.


## Usual setup:

```
# clone this repo

cd minecraft_ros2
docker compose up # build the container

# in another terminal
./bash.sh            # shortcut for entering the running container

# source all overlays (including ros2_java_ws)

```
### Note:
My Dockerfile adds an additional workspace to fill in the holes of the minecraft_ros2 repo. Might not be needed in the future/ever. Try building it and sourcing it - it may fix the DigBlock service.

## Custom Setup
Make sure the container's up and running (`docker compose up`).

### Python Libraries
For all Python libraries, in the container (use `./bash.sh` again to open bash in the container) do:
```
apt update
apt install python3.10-venv
cd custom
python3 -m venv .venv
. .venv/bin/activate
pip install pygame numpy torch pyyaml cv_bridge opencv-python matplotlib    # and all future libs to be used
```

### New ROS Nodes and Agent
Now, get the custom ROS workspace in there. On the host machine, run:
```
./copy_scripts.sh      # shortcut for 'docker cp ...'
```
The files will be moved into the container in the right spot. Now build the custom workspace from back inside the container:
```
cd custom/custom_ws
colcon build
. install/setup.bash
```

Also source `/ws/ros2_java_ws` and the underlay `/opt/...`.

Now ready to run.

```mermaid
flowchart TD
    Z((Minecraft))
    Z -- redstone signal, RGB, D ----> R(ROS)
    R -- reward, RGB, D ----> P((Agent))
    P -- one-hot action --> R
    R -- Twist, dig --> Z
```

# Running
## Teleop, StateFormer, and Rewarder
1. Start Minecraft (`docker compose up`) and open up a world.
2. In another terminal:
```
./bash.sh # now you're in the container
. custom/custom_ws/install/setup.bash       # agent nodes now available
ros2 run agent keys                         # or 'state_former' or 'rewarder'
```
Make sure the black popup window is in focus. WASD+space to move, arrow keys to orient, and B to dig.

## Reinforcement Learning

### Grassy Superflat
1. Open up the superflat grass_block world in Minecraft.

2. Make the oak tree at 0 -60 4

3. Add the following redstone device, and point it to the tree:

4. source all ROS workspaces and the virtual environment created in Custom Setup

5. In `/ws/custom/custom_ws/`, run:
```
python3 src/agent/agent/policy_performer.py (--checkpoint /ws/custom/checkpoints/best.pth (--test))
```
