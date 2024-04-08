# syntax=docker/dockerfile:1
ARG BASE_IMAGE=docker.io/polymathrobotics/ros_base:humble

FROM ${BASE_IMAGE}

ARG SRC_DIR=/src
ARG COLCON_WS=/colcon_ws
ARG COLCON_EXTEND=/opt/ros/${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive

ENV COLCON_WS=${COLCON_WS}
ARG COLCON_SRC=${COLCON_WS}/src
ARG POLYMATH_INSTALL=/opt/polymathrobotics/
ARG COLCON_BUILD_ARGS=

COPY ${SRC_DIR} ${COLCON_SRC}

# Install dependencies
RUN apt-get update \
  && rosdep update --rosdistro "${ROS_DISTRO}" \
  && rosdep install -i -y --from-paths ${COLCON_SRC} --skip-keys="rviz2 gazebo_ros gazebo_ros_pkgs gazebo_ros2_control gazebo_plugins gazebo" \
  && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}
ARG _COLCON_BUILD_ARGS="${COLCON_BUILD_ARGS} --install-base ${POLYMATH_INSTALL}"
RUN . $COLCON_EXTEND/setup.sh && \
    colcon build ${_COLCON_BUILD_ARGS}

# Cleanup workspace
RUN rm -rf src/* build/* log/*

WORKDIR /
# setup entrypoint
COPY entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
