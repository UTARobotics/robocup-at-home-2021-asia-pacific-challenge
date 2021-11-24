FROM devrt/ros-devcontainer-vscode:melodic-desktop

USER root

ENV DEBIAN_FRONTEND noninteractive

# Install python3-pip
RUN apt-get update
RUN apt-get -y install python3-pip python3-catkin-pkg-modules python3-rospkg-modules 

USER developer

# Install ipykernel
RUN pip3 install ipykernel
RUN python3 -m ipykernel install --user

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
RUN rosnode kill /move_group
# command to run the algorithm
CMD roslaunch robocup_hsr_simulator run.launch
