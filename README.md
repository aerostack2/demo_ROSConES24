# ROSCon ES'24 Aerostack2 demonstration
This repository contains the demonstration used in the Aerostack2 Tutorial in ROSCon ES'24.

Slides can be found [online](https://docs.google.com/presentation/d/1hjP8gxDRpMIkM-t7CMRemirabQHAWMtExiB2WkvZuYg/edit#slide=id.g301f70bfd0b_1_3) or in the [slides](slides.pdf).

## Installation steps
First step is to clone this repository:

```bash
git clone https://github.com/aerostack2/demo_ROSConES24.git
cd demo_ROSConES24
```

For this demo we are going to use Aerostack2 v1.1 which is available throught apt.
We provide two ways of setting everything up, using binaries or using docker.

### Setup using binaries
Prerequisites:
- Ubuntu 22.04
- ROS 2 Humble


You can follow [Aerostack2 setup guide](https://aerostack2.github.io/_00_getting_started/binary_install.html) for installing and setting up aerostack2.
Install Aerostack2 using apt:

```bash
sudo apt install ros-humble-aerostack2
```

> Remember to ``` source /opt/ros/humble/setup.bash ``` on each terminal you are going to use for runnning the demo.

> WARNING: Set ``` export ROS_LOCALHOST_ONLY=1 ``` to avoid conflicts with other ROS 2 user.

### Setup using Docker (RECOMENDED)

All the demo is designed to be easily used and modify using docker. Go to the root folder of the repository and run:

```bash
xhost + # this will enable gazebo visualization
docker compose up -d # use the -d for keep the container alive in background
```

> If your host machine is Windows, change display configuration at `docker-compose.yaml`. See [Issue 4](https://github.com/aerostack2/demo_ROSConES24/issues/4). Kudos to [@dvdmc](https://github.com/dvdmc).

With this there is a running instance of the container with this project mounted in ```/root/demo_ROSConES24```.
Now you can run as much terminals as you need by running: 

```bash
docker exec -it aerostack2_roscon /bin/bash
```

> For stopping the container run ```xhost - ; docker compose down ``` command on the repo root folder. This will also remove the access to the XServer from the container.


### Examples
1. [Motion reference control example](example1/README.md)
2. [Plan execution control example](example2/README.md)
3. [From sim to real, using Crazyflies](example3/README.md)
4. [Drone-convoy example](example4/README.md)
