# Base image
ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION}

# Dev container arguments
ARG ROS_DISTRO
ARG USERNAME
ARG UID
ARG GID

SHELL ["/bin/bash", "-c"]

# Avoiding tzdata requirement for user interaction during installation
ENV DEBIAN_FRONTEND=noninteractive

# Minimal setup
RUN apt update && \
    apt install -y sudo locales lsb-release plocate gnupg gedit htop byobu\
    wget unzip vim git mesa-utils && \
    dpkg-reconfigure locales && \
    apt upgrade -y

## Install ROS2
# Ensure that the Ubuntu Universe repository is enabled
RUN apt install software-properties-common -y && \
    add-apt-repository universe && \
    # Add the ROS 2 GPG key with apt
    apt update && sudo apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    # Add the repository to your sources list
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    # Install development tools
    apt update && apt install ros-dev-tools -y && \
    # ROS2 installation
    apt update && apt upgrade -y && \
    apt install ros-$ROS_DISTRO-desktop -y && \
    # Install rqt and its plugins
    apt update && sudo apt install '~nros-$ROS_DISTRO-rqt*' -y

RUN apt install ros-$ROS_DISTRO-kinova-gen3-7dof-robotiq-2f-85-moveit-config -y && \
    apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y && \
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    apt install python3-colcon-common-extensions python3-vcstool -y
WORKDIR /home/$USERNAME/iri_lab

RUN git clone -b humble --single-branch https://github.com/Kinovarobotics/ros2_kortex.git src/ros2_kortex && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos && \
    vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos

RUN apt update && apt install wget -y && \
    # Install Gazebo and Ignition Fortress
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && apt-get install ignition-fortress -y && \
    vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos

RUN git clone https://github.com/harshaguda/colab_dress_interfaces.git src/colab_dress_interfaces && \
    git clone https://github.com/harshaguda/colab_dress.git src/colab_dress

RUN . /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y -r && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

RUN echo "source /home/$USERNAME/iri_lab/install/setup.bash" >> /home/$USERNAME/.bashrc

## Create new user and home directory
RUN groupadd -f --gid $GID $USERNAME && \
    useradd --uid $UID --gid $GID --create-home $USERNAME && \
    echo $USERNAME:$USERNAME | chpasswd && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    adduser --disabled-password  $USERNAME sudo && \
    chown -R $USERNAME:$USERNAME /home/$USERNAME && \
    # Set the ownership of the overlay workspace to the new user
    mkdir -p /home/$USERNAME/iri_lab/iri_ws/src && \
    mkdir -p /home/$USERNAME/iri_lab/labrobotica/drivers && \
    mkdir -p /home/$USERNAME/iri_lab/labrobotica/algorithms && \
    chown -R $UID:$GID /home/$USERNAME/iri_lab

USER $USERNAME

# Install pip and set PATH
RUN sudo apt install python3-pip -y
ENV PATH="/home/$USERNAME/.local/bin:${PATH}"
#RUN python3 -m pip install --upgrade pip

# ROS Environment setup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    bash -c "source ~/.bashrc"

# Build the ROS workspace
WORKDIR /home/$USERNAME/iri_lab/iri_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build && \
    echo "source /home/$USERNAME/iri_lab/iri_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Configure byobu
RUN mkdir /home/$USERNAME/.byobu/ && \
    echo "set -g default-command '/bin/bash --login'" >> /home/$USERNAME/.byobu/.tmux.conf

# Copy inputrc script
WORKDIR /home/$USERNAME
# COPY inputrc_script ./.inputrc

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,display
