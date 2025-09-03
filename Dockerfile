# Load base image
ARG ROS_DISTRO=default
ARG UBUNTU_VERSION=default
FROM pmlab_base_u${UBUNTU_VERSION}_${ROS_DISTRO}

ARG ROS_DISTRO
ARG USERNAME    
ARG KORTEX_VERSION

# Authorize SSH Host
RUN mkdir -p ~/.ssh && \
    chmod 0700 ~/.ssh && \
    ssh-keyscan -p 2202 gitlab.iri.upc.edu > ~/.ssh/known_hosts
# Add the keys and set permissions
RUN --mount=type=secret,id=ssh_key,required \
    sudo cp /run/secrets/ssh_key ~/.ssh/id_rsa && \
    sudo chmod 600 ~/.ssh/id_rsa && \
    sudo chown $USERNAME ~/.ssh/id_rsa

# Install pip3 (for conan installation)
RUN sudo apt install python3-pip -y && \
    python3 -m pip install --upgrade pip

# Install KORTEX driver
RUN python3 -m pip install conan==1.59 && \
    conan config set general.revisions_enabled=1 && \
    conan profile new default --detect > /dev/null && \
    conan profile update settings.compiler.libcxx=libstdc++11 default
WORKDIR /home/$USERNAME/iri_lab/labrobotica/drivers
RUN git clone https://github.com/Kinovarobotics/kortex.git
WORKDIR /home/$USERNAME/iri_lab/labrobotica/drivers/kortex
RUN if [ $KORTEX_VERSION == 2.3 ] ; then git checkout v2.3.0 ; fi
WORKDIR /home/$USERNAME/iri_lab/labrobotica/drivers/kortex/api_cpp/examples
RUN ./scripts/build-gcc.sh
WORKDIR /home/$USERNAME/iri_lab/labrobotica/drivers/kortex/api_python
RUN if [ $KORTEX_VERSION == 2.3 ] ; then \
    wget https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.3.0/kortex_api-2.3.0.post34-py3-none-any.whl \
    && python3 -m pip install kortex_api-2.3.0.post34-py3-none-any.whl ;\
    else wget https://artifactory.kinovaapps.com/artifactory/generic-public/kortex/API/2.6.0/kortex_api-2.6.0.post3-py3-none-any.whl \
    && python3 -m pip install kortex_api-2.6.0.post3-py3-none-any.whl ; fi

# Install ROS KORTEX driver
WORKDIR /home/$USERNAME/iri_lab/iri_ws/src
RUN git clone -b $ROS_DISTRO-devel https://github.com/Kinovarobotics/ros_kortex.git && \
    git clone https://github.com/Kinovarobotics/ros_kortex_vision.git && \
    git clone https://github.com/harshaguda/dressing_kinova.git
WORKDIR /home/$USERNAME/iri_lab/iri_ws/src/ros_kortex
RUN if [ $KORTEX_VERSION == 2.3 && $ROS_DISTRO == melodic ] ; then git checkout v2.3.0 ; fi
WORKDIR /home/$USERNAME/iri_lab/iri_ws
RUN rosdep update && \
    sudo apt update && \
    if [ $ROS_DISTRO == melodic ] ; then \
    sudo apt install ros-melodic-moveit ros-melodic-ros-control ros-melodic-ros-controllers -y ;\
    else rosdep install --from-paths src --ignore-src -y ; fi

# ros_kortex_vision dependeciess
RUN sudo apt-get install ros-$ROS_DISTRO-rgbd-launch -y && \
    sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-base -y

# Install ros_robotiq_gripper
RUN if [ $ROS_DISTRO == melodic ] ; then python -m pip install minimalmodbus ; \
    else python3 -m pip install minimalmodbus ; fi
WORKDIR /home/$USERNAME/iri_lab/iri_ws/src

# Install librealsense dependencies
RUN sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y && \
    sudo apt-get install git wget cmake build-essential -y && \
    sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y

RUN pip3 install mediapipe pyrealsense2 stable-baselines3 gymnasium facenet-pytorch emotiefflib[all] transformers
RUN git clone https://github.com/harshaguda/actionvideomae.git

# Build the ROS workspace
WORKDIR /home/$USERNAME/iri_lab/iri_ws
RUN . /opt/ros/$ROS_DISTRO/setup.bash && catkin_make

WORKDIR /home/$USERNAME
RUN sudo apt update && sudo apt upgrade -y

# Remove the keys
RUN rm -rf ~/.ssh/
