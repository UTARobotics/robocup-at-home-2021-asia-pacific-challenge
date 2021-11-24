FROM devrt/ros-devcontainer-vscode:melodic-desktop

USER root

ENV DEBIAN_FRONTEND noninteractive

# install depending packages (install moveit! algorithms on the workspace side, since moveit-commander loads it from the workspace)
RUN apt-get update && \
    apt-get install -y git ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-commander ros-$ROS_DISTRO-move-base-msgs ros-$ROS_DISTRO-ros-numpy ros-$ROS_DISTRO-geometry && \
    apt-get clean

# install bio_ik
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    mkdir -p /bio_ik_ws/src && \
    cd /bio_ik_ws/src && \
    catkin_init_workspace && \
    git clone --depth=1 https://github.com/TAMS-Group/bio_ik.git && \
    cd .. && \
    catkin_make install -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO -DCATKIN_ENABLE_TESTING=0 && \
    cd / && rm -r /bio_ik_ws

# Install python3-pip
RUN apt-get update
RUN apt-get -y install python3-pip python3-catkin-pkg-modules python3-rospkg-modules 

USER developer
#install YoloV3
RUN cd ~
RUN wget https://raw.githubusercontent.com/ultralytics/yolov3/master/requirements.txt
RUN pip3 install scikit-build
RUN pip3 install --upgrade setuptools pip
RUN python3 -m pip install -r ~/requirements.txt
RUN python3 -m pip install sparseml sparsezoo deepsparse

USER root

# create workspace folder
RUN mkdir -p /workspace/src

# copy our algorithm to workspace folder
ADD . /workspace/src

# install dependencies defined in package.xml
RUN cd /workspace && /ros_entrypoint.sh rosdep install --from-paths src --ignore-src -r -y

# compile and install our algorithm
RUN cd /workspace && /ros_entrypoint.sh catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/$ROS_DISTRO

# command to run the algorithm
CMD roslaunch robocup_hsr_simulator run.launch
