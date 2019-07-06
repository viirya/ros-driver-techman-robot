# ROS driver for Techman robot

A driver provides ROS support for techman robots. TM5_700 is available for ROS Kinetic.

This is originally forked from [techman_robot](https://github.com/kentsai0319/techman_robot). Because looks like that repo isn't actively maintained, this repo copies that repo and does some development here.

## Overview

### ROS interfaces

* Action interface on */joint\_trajectory\_action* for integration with __MoveIt__
* Publishes robot joint state on */joint\_states*
* Publishes TCP position on */tool\_position*
* Publishes TCP velocity on */tool\_velocity*
* Publishes TCP force on */wrench*
* Service call to set outputs on */tm\_driver/set\_io*

### Packages

* tm_driver: ROS node for driving the robotic arm. This publishes robot information to topics as above.
* tm_description: The urdf files for robotics. 
* tm700_camera_test: A ROS node used for testing built-in camera.
* tm700_gripper_test: A ROS node used for testing CHG2 gripper attached to robotic arm.

## Installation

Make a catkin workspace like:

    cd && mkdir -p catkin_ws/src
    
Copy all packages into `catkin_ws/src`, then run `catkin_make` to build the packages. Make sure you have installed all necessary dependencies. You can also use the Docker files to quickly set up development environment.

## Usage with Moveit

### Test in simulation:

To bring up moveit environment in simulation mode, run:  
```roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch```

### Run with real robot:

For TM5-700 as an example, the robotic arm runs Windows OS. By default, it enables DHCP to obtain IP automatically. You can either use the dynamic IP, or set up a static Intranet IP like 192.168.x.x on the robot. Make sure you can ping the robot at the IP.

The firewall setting on Windows can interfere with robot connection. If experiencing with connection problem, check with firewall setting of Windows.

To bring up moveit environment and connect to real robot, run:  
```roslaunch tm700_moveit_config tm700_moveit_planning_execution.launch sim:=False robot_ip:=<robot ip>```

## Usage with Gazebo

To bring up the simulated robot in Gazebo, run:

    roslaunch tm_gazebo tm700.launch


To run simulated robot in Gazebo and control it through MoveIt, run:

    roslaunch tm_gazebo tm700_gazebo_moveit.launch


## Development environment in Docker

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

Our TM5-700 has a built-in uEye camera.

### Connect to built-in camera of TM5-700 in Linux container

We can't connect to the built-in camera in Linux container. Although we can see the uEye camera in camera list of uEye's camera manager, opening the camera results failure.

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

The TM5-700 model we test has built-in USB camera from IDS, the Linux driver for the camera can be found [here](https://en.ids-imaging.com/download-ueye-lin64.html). Although in Linux container the camera is shown in USB device list, and the driver toolkit also can detect the camera, the demo program fails to open the camera and capture an image. Unless we can solve this issue, we can't use the USB camera in Linux container.

### Use built-in camera on Linux machine

We tried with two ROS package for uEye camera: [ueye](http://wiki.ros.org/ueye) and [ueye_cam](http://wiki.ros.org/ueye_cam). Not sure if `ueye` was built with too old uEye library, running the node from `ueye` causes segmentation fault on Ubuntu 16.04.

`ueye_cam` is ok to use, although it isn't workable out-of-box. For the camera on our TM5-700, we need to set

```
 <param name="image_width" type="int" value="1280" />
 <param name="image_height" type="int" value="720" />
 <param name="image_top" type="int" value="0" /> <!-- -1: center -->
 <param name="image_left" type="int" value="0" /> <!-- -1: center -->
```

And set the color mode to `bayer_rggb8`:

```
<param name="color_mode" type="str" value="bayer_rggb8" />
```

in launch file, e.g, `debug.launch`, to make `ueye_cam` work normally.

After setting that, remember to save camera parameter into file using uEye's ueye demo tool. The parameter file location is `~/.ros/camera_conf/<camera_name>.ini` by default. Please see `ueye_cam` document for more parameters.

To launch ROS node of `ueye_cam`:

```
roslaunch ueye_cam debug.launch  # for debugging
```

Then, using `rostopic list`, you should see a topic `/camera/image_raw` that `ueye_cam` node publishes image data to.

You can use `tm700_camera_test` node in this repo to test it. Run `roslaunch tm700_camera_test tm_camera_test.launch image_topic:=<value>`. The node will subscribe the given topic and show captured images in a window. `ueye_cam` publishes to `camera/image_raw` topic by default.


## CHG2 gripper with TM5-700

`tm700_gripper_test` ROS package contains a ROS node used to test CHG2 gripper on TM5-700. You can launch the node like:

    roslaunch tm700_gripper_test tm_gripper_test.launch  robot_ip:=<robot ip>

You can input `open` and `close` commands to test gripper open and close actions.
