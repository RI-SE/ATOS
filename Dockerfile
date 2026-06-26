ARG ROS_DISTRO=humble
ARG OS=jammy
ARG FROM_IMAGE=ros:${ROS_DISTRO}-ros-base-${OS}
ARG OVERLAY_WS=/opt/ros/${ROS_DISTRO}/overlay_ws

FROM ${FROM_IMAGE} AS cacher

ENV DEBIAN_FRONTEND=noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

ENV ROS_DISTRO=${ROS_DISTRO}
ENV REPO_DIR=/root/atos_git
SHELL ["/bin/bash", "-c"]

WORKDIR ${REPO_DIR}

COPY ./scripts/installation/ ./scripts/installation/
COPY ./atos_gui/requirements.txt ./atos_gui/requirements.txt
COPY ./atos_gui/package.xml ./atos_gui/package.xml
COPY ./atos_interfaces/package.xml ./atos_interfaces/package.xml
COPY ./atos/package.xml ./atos/package.xml
COPY . .
RUN --mount=type=cache,target=/var/cache/apt \
        ./setup_atos.sh
WORKDIR /root/atos_ws
RUN chmod +x /root/atos_git/scripts/run_atosfleetmanagement.sh

EXPOSE 8420 8765 9090 8114

CMD ["/bin/bash", "-lc", "/root/atos_git/scripts/run_atosfleetmanagement.sh"]
