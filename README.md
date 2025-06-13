

## Usual setup:
```
docker compose up # build the container

# in another terminal
./bash.sh

# source all overlays (including ros2_java_ws)

```
### Note:
My Dockerfile adds an additional workspace to fill in the holes of the minecraft_ros2 repo. Might not be needed in the future/ever. Try building it and sourcing it - it may fix the DigBlock service.



## Custom setup
Make sure the container's up and running (`docker compose up`).

For all ROS-related libraries, in the container (use `./bash.sh` again to open bash in the container) do:
```
apt install python3.10-venv
cd custom
python3 -m venv .venv
. .venv/bin/activate
pip install pygame, numpy       # and all future libs to be used
```
Now, get the custom ROS workspace in there. On the host machine, run:
```
./copy_scripts.sh
```
The files will be moved into the container. Now build the custom workspace from back inside the container:
```
cd custom/custom_ws     # CHECK - might be `cd custom` or `cd custom_ws`
colcon build
. install/setup.bash
```
Now ready to run (for brevity: `ros2 run agent keys`)



# Running
1. Start Minecraft (`docker compose up`) and open up a world
2. In another terminal:
```
./bash.sh # now you're in the container
. custom/custom_ws/install/setup.bash       # agent nodes now available
ros2 run agent keys
```
Make sure the black popup window is in focus. WASD+space to move, arrow keys to orient, and B to dig.
