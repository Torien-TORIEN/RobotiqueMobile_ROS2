# Docker Image for the mobile robotic module

# This docker image is created from the ubuntu:22.04 image
FROM ubuntu:22.04

# To avoid interactive prompt when installing ros-humble-desktop
ENV DEBIAN_FRONTEND noninteractive
# To deal with the interactive question asked when installing ros-humble-desktop
ENV TZ=Europe/Paris

# To deal with locale information (time zone, coding...)
RUN apt-get update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# To handle the user and sudo command
ENV USERNAME docker

# Install the sudo command
RUN apt-get install -y sudo
RUN useradd -m $USERNAME
RUN echo "$USERNAME:$USERNAME" | chpasswd
RUN usermod --shell /bin/bash $USERNAME
RUN usermod -aG sudo $USERNAME
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME
RUN chmod 0440 /etc/sudoers.d/$USERNAME
# 1000 because it is the user and group id for my machine
RUN usermod  --uid 1000 $USERNAME
RUN groupmod --gid 1000 $USERNAME

# From there, it is the tutorial https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe -y
RUN apt-get update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install ros-humble-desktop -y

# Then it is from the tutorial https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_LOCALHOST_ONLY=1" >> /home/$USERNAME/.bashrc

# Install of gazebo
RUN apt-get install lsb-release wget gnupg -y
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update
RUN apt-get install ignition-fortress -y
RUN apt-get install ros-humble-ros-gz -y
RUN apt-get install ros-humble-ros-ign-bridge -y

# Install colcon (https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
# note that for now it is needed to install the 58.2.0 version of setup tools for colcon to work... Hopefully this will be corrected...
RUN apt-get install python3-pip -y
RUN pip3 install setuptools==58.2.0
RUN apt-get install python3-colcon-common-extensions -y

RUN apt-get install ros-humble-robot-state-publisher -y
RUN apt-get install ros-humble-joint-state-publisher-gui -y
# Setup colcon tab completion
# RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# To source the ros2 installed custom package directly in a new bash
#RUN echo "ROSSETUP=/home/docker/wdir/install/setup.bash\nif [ -f \"\$ROSSETUP\" ]; then\n  echo \"source \$ROSSETUP\"\n  source \$ROSSETUP\nfi" >> /home/$USERNAME/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

RUN usermod -aG rdma $USERNAME

USER $USERNAME
# note that this docker image does not have any CMD, it should be started with at least a bash command
