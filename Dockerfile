FROM ct2034/vnc-ros-kinetic-full

ENV DEBIAN_FRONTEND noninteractive

# The key added in vnc-ros-kinetic-full doesn't work now. Update to new key.
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# The added key at http://download.opensuse.org/repositories/home:Horst3180/xUbuntu_16.04/Release.key doesn't work now.
# It seems to use only for arc-theme package. Remove the repo.
RUN rm /etc/apt/sources.list.d/arc-theme.list

RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-kinetic-moveit ros-kinetic-gazebo-dev ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-ros-controllers \
    ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-pkgs ros-kinetic-industrial-robot-simulator \
    ros-kinetic-rosconsole ros-kinetic-usb-cam ros-kinetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN cd /root && mkdir -p catkin_ws/src
ADD techman_robot /root/catkin_ws/src/techman_robot
ADD tm700_moveit_config /root/catkin_ws/src/tm700_moveit_config
ADD tm900_moveit_config /root/catkin_ws/src/tm900_moveit_config
ADD tm_description /root/catkin_ws/src/tm_description
ADD tm_driver /root/catkin_ws/src/tm_driver
ADD tm_gazebo /root/catkin_ws/src/tm_gazebo
ADD tm_kinematics /root/catkin_ws/src/tm_kinematics
ADD tm_msgs /root/catkin_ws/src/tm_msgs

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"
