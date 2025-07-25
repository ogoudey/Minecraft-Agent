# Minecraft ROS Reinforcement Learning Agent
This repo introduces three executable ROS nodes: the Rewarder, StateFormer, and Teleop nodes. These nodes are instantiated automatically by the algorithm to train a policy, but can be run individually for debugging.


## Installation:

```
# clone this repo
git clone https://github.com/ogoudey/Minecraft-Agent.git

cd Minecraft-Agent

# clone the minecraft_ros2 repo
git clone https://github.com/minecraft-ros2/minecraft_ros2.git

cd minecraft_ros2
xhost +local:root         # for GUI permissions for docker
docker compose up         # build the container

# in another terminal
./copy_scripts.sh         # puts custom code in the container


./bash.sh                 # shortcut for entering the running container

. quickstart.sh           # sources all the ROS workspaces, makes Python venv and moves cwd to the custom ROS workspace
```


# Running
## Testing performance
1. Start Minecraft (`docker compose up`) and open up a world.
2. In another terminal:
```
./bash.sh # now you're in the container
. quickstart
ros2 run agent keys
```
Make sure the black popup window is in focus. WASD+space to move, arrow keys to orient, and B to dig.

## Reinforcement Learning

### Grassy Superflat
Currently only one training environment is supported.


#### How it works
Policy evaluation comes from the following redstone device, which senses if particular blocks are broken:

```mermaid
flowchart TD
    Z((Minecraft))
    Z -- redstone signal, RGB, D ----> R(ROS)
    R -- reward, RGB, D ----> P((Agent))
    P -- one-hot action --> R
    R -- Twist, dig --> Z
```


![Screenshot from 2025-06-17 21-51-22](https://github.com/user-attachments/assets/0e287634-8891-46b2-838e-55b3024ba72f)
For a tree at 0 -60 4, the Minecraft command block has the line:
```
execute if block 0 -60 4 minecraft:oak_log if block 0 -59 4 minecraft:oak_log if block 0 -58 4 minecraft:oak_log if block 0 -57 4 minecraft:oak_log
```

ROS subscribes to that particular command block by setting a redstone pub_sub block to publish from within Minecraft:
```
/data merge block -3 -60 16 {CustomName:"redstone/my_block",pollRate:10}
```
where the coordinates are of the `minecraft_ros2:redstone_pub_sub` block in the device above. (The ROS node Rewarder subscribes to `/redstone/my_block/output` so "my_block" is important.)

### Extras
Hit F3 + P to enable/disable pause on lost focus.
Hide the chat in settings.
