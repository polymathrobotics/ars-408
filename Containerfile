# syntax=docker/dockerfile:1
ARG BASE_IMAGE=docker.io/polymathrobotics/ros_base:humble
FROM $BASE_IMAGE

ARG COLCON_WS=/colcon_ws
ARG COLCON_EXTEND=/opt/ros/${ROS_DISTRO}
ARG DEBIAN_FRONTEND=noninteractive

ENV COLCON_WS=${COLCON_WS}
ARG COLCON_SRC=${COLCON_WS}/src
ARG POLYMATH_INSTALL=/opt/polymathrobotics/
ARG COLCON_BUILD_ARGS=

# TODO: make this included with git/vcs
COPY CAN_Recording ${COLCON_SRC}/Can_Recording
COPY radar_conti_ars408 ${COLCON_SRC}/radar_conti_ars408
COPY radar_conti_ars408_msgs ${COLCON_SRC}/radar_conti_ars408_msgs
COPY ros2_socketcan_bridge ${COLCON_SRC}/ros2_socketcan_bridge

# Install dependencies
RUN apt-get update \
  && rosdep update --rosdistro "${ROS_DISTRO}" \
  && rosdep install -i -y --from-paths ${COLCON_SRC} \
  && rm -rf /var/lib/apt/lists/*

WORKDIR ${COLCON_WS}
ARG _COLCON_BUILD_ARGS="${COLCON_BUILD_ARGS} --install-base ${POLYMATH_INSTALL}"
RUN . $COLCON_EXTEND/setup.sh && \
    colcon build ${_COLCON_BUILD_ARGS}

# Cleanup workspace
RUN rm -rf src/* build/* log/*

WORKDIR /
# setup entrypoint
COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

