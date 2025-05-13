#!/bin/bash

# Path to the configuration file
CONFIG_FILE="config.cfg"

# Load the configuration file if it exists
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    echo "Configuration file $CONFIG_FILE not found!"
    exit 1
fi

# Allow overriding variables through script arguments
UBUNTU_VERSION=${1:-$UBUNTU_VERSION}
ROS_DISTRO=${2:-$ROS_DISTRO}
DOCKER_USER=${3:-$DOCKER_USER}
KORTEX_VERSION=${4:-$KORTEX_VERSION}
IMAGE_NAME="pmlab_base_u${UBUNTU_VERSION}_${ROS_DISTRO}_kinova_${KORTEX_VERSION}"

# Print build info
echo "======================="
echo " Building docker image "
echo " IMAGE_TAG:      ${IMAGE_NAME}"
echo " ROS DISTRO:     ${ROS_DISTRO}"
echo " UBUNTU V.:      ${UBUNTU_VERSION}"
echo " DOCKER_USER:    ${DOCKER_USER}"
echo "======================="

# Build the Docker image
docker build --secret id=ssh_key,src=$HOME/.ssh/id_rsa \
             --build-arg ROS_DISTRO="${ROS_DISTRO}" \
             --build-arg UBUNTU_VERSION="${UBUNTU_VERSION}" \
             --build-arg USERNAME="${DOCKER_USER}" \
             --build-arg KORTEX_VERSION="${KORTEX_VERSION}" \
             -f Dockerfile \
             -t "${IMAGE_NAME}" .
