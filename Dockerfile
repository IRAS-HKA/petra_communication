##############################################################################
##                                 Base Image                               ##
##############################################################################

FROM ros:dashing-ros-base-bionic

##############################################################################
##                                 Dependecies                              ##
##############################################################################
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-dashing-desktop=0.7.4-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.5 \
    python3-pip \
    && \
apt-get clean && \
rm -rf /var/lib/apt/lists/*
RUN pip3 install cmake==3.18.4
RUN apt-get update && apt-get install -y ros-dashing-behaviortree-cpp-v3


##############################################################################
##                           Make ROS Workspace                             ##
##############################################################################

ARG GIT_USERNAME $GIT_USERNAME
ARG GIT_PASSWORD $GIT_PASSWORD

WORKDIR /home
RUN mkdir -p $GIT_USERNAME/ros2_ws/src

WORKDIR /home/$GIT_USERNAME/ros2_ws/src


RUN git clone -b dev https://$GIT_USERNAME:$GIT_PASSWORD@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/petra/petra_interfaces.git
RUN git clone -b dev https://$GIT_USERNAME:$GIT_PASSWORD@www.w.hs-karlsruhe.de/gitlab/iras/core/ros_core.git

COPY . ./petra_communication

WORKDIR /home/$GIT_USERNAME/ros2_ws/src/ros_core
RUN git clone https://$GIT_USERNAME:$GIT_PASSWORD@www.w.hs-karlsruhe.de/gitlab/iras/core/cpp_core.git


WORKDIR /home/$GIT_USERNAME/ros2_ws/
RUN	/bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build"
RUN echo "source /home/$GIT_USERNAME/ros2_ws/install/setup.bash" >> ~/.bashrc




