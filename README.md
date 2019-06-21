# ROS driver for Techman robot

A driver provides ROS support for techman robots. TM5_700 is available for ROS Kinetic.

This is originally forked from [techman_robot](https://github.com/kentsai0319/techman_robot). Because looks like that repo isn't actively maintained, this repo copies that repo and does some development here.

> __NOTE__:  
This project is in development. Currently, only TM5_700 is available, there will be TM5_900 and TM10_1300 in the future. Note that we will use "tm700" to represent TM5_700 in this package.  


## Overview

* Action interface on */joint\_trajectory\_action* for integration with __MoveIt__
* Publishes robot joint state on */joint\_states*
* Publishes TCP position on */tool\_position*
* Publishes TCP velocity on */tool\_velocity*
* Publishes TCP force on */wrench*
* Service call to set outputs on */tm\_driver/set\_io*


## Installation

Make a catkin workspace like:

    cd && mkdir -p catkin_ws/src
    
Copy all packages into `catkin_ws/src`, then run `catkin_make` to build the packages. Make sure you have installed all necessary dependencies. You can also use the Docker files to quickly set up development environment.

## Usage with Moveit

### test in simulation:

To bring up moveit environment in simulation mode, run:  
```roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch```

### run with real robot:

For TM5-700 as an example, the robotic arm runs Windows OS. By default, it enables DHCP to obtain IP automatically. You can either use the dynamic IP, or set up a static Intranet IP like 192.168.x.x on the robot. Make sure you can ping the robot at the IP.

The firewall setting on Windows can interfere with robot connection. If experiencing with connection problem, check with firewall setting of Windows.

To bring up moveit environment and connect to real robot, run:  
```roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch sim:=False robot_ip:=<robot ip>```

## Usage with Gazebo
To bring up the simulated robot in Gazebo, run:  
```roslaunch tm_gazebo tm700.launch```


## Docker

Build a docker image to test the robot:

    docker build . -t ros-tm-700 --rm

Note that this docker image is proposed for Mac. Because GLX support of X11 on Mac isn't good enough, running the docker container will launch x11vnc and provide noVNC connection to the Linux desktop in the container. Launching the container like:

    docker run -it --rm -p 6080:80 ros-tm-700

Then connect to the Linux desktop in a browser at `http://127.0.0.1:6080/`.

For Linux, x11vnc and noVNC are not necessary. There is Dockerfile for Linux:


    docker build . -f Dockerfile.linux -t ros-tm-700-linux --rm

    docker run -it ros-tm-700-linux /bin/bash
    
To make X11 forwarding work, you should set a proper `DISPLAY` inside the docker container.     

After launching docker container, build the ROS packages:

    source /opt/ros/kinetic/setup.bash && cd /root/catkin_ws && catkin_make

There might be an issue like library inconsistency, so it might need to update packages like `apt-get update`.

## Built-in camera of TM5-700

It it quite troublesome to use the built-in camera of TM5-700 on Mac. It is because Docker for Mac doesn't support USB device passthrough (see the related [issue](https://github.com/docker/for-mac/issues/900)). One solution is to use `docker-machine` to run docker daemon inside a Virtualbox VM. Virtualbox can expose USB devices on host computer to the running VM. There is good [article](https://dev.to/rubberduck/using-usb-with-docker-for-mac-3fdd) describing how to do that:

```
# create and start the machine
docker-machine create -d virtualbox default

# stop the vm
docker-machine stop

# you can enable USB port on the VM by doing following. But you can also set it up through Virtualbox GUI.
vboxmanage modifyvm default --usb on

# enable USB 2.0, if you installed the extension pack.
vboxmanage modifyvm default --usbehci on

# you can run this to see usb device list
vboxmanage list usbhost

# setup a usb filter so your device automatically gets connected to the Virtualbox VM. Can do this through Virtualbox GUI.
vboxmanage usbfilter add 0 --target default --name ftdi --vendorid 0x0403 --productid 0x6015

# Go ahead and start the VM back up
docker-machine start

# setup your terminal to let your docker client to use the docker daemon inside running Virtualbox VM
eval $(docker-machine env default)
```

After above, you should run your docker container with `--privileged` flag. Using `lsusb` command should show the USB device up in the list.

## CHG2 gripper with TM5-700

`tm700_gripper_test` ROS package contains a ROS node used to test CHG2 gripper on TM5-700. You can launch the node like:

    roslaunch tm700_gripper_test tm_gripper_test.launch  robot_ip:=<robot ip>

You can input `open` and `close` commands to test gripper open and close actions.
