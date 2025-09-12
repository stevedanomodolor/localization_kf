FROM ros:humble-ros-core-jammy AS frozen_stage

ARG UNDERLAY_WS=/opt/underlay_ws
ARG OVERLAY_WS=/opt/overlay_ws

FROM  frozen_stage AS build_stage


# Install dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
 && apt-get install -y -qq \
    python3-pip \
    python3-kconfiglib \
    python3-jinja2 \
    python3-jsonschema \
    python3-future \
    gcc-arm-none-eabi \
    rsync \
    dirmngr \
    build-essential \
    lsb-release \
    wget \
    curl \
    gnupg2 \
    qtbase5-dev \
    cmake  \
    git \
    ros-humble-nav2-util  \
    ros-humble-nav2-lifecycle-manager  \
    ros-humble-angles  \
    ros-humble-plotjuggler-ros \
    && apt-get clean

# install geographiclib
RUN wget https://sourceforge.net/projects/geographiclib/files/distrib-C++/GeographicLib-2.5.2.tar.gz && \
    tar xfpz GeographicLib-2.5.2.tar.gz && \
    cd GeographicLib-2.5.2 && \
    mkdir build && cd build && \
    cmake .. && \
    make && make install && \
    cd ../.. && rm -rf GeographicLib-2.5.2*


RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y -qq gz-garden \
    && apt-get clean

RUN pip3 install vcstool pyros-genmsg


RUN apt-get update && apt-get upgrade -q -y && \
    apt -y install python3-rosdep python3-colcon-common-extensions \
     $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') \
    && apt-get clean

# Copy underlay workspace
ARG UNDERLAY_WS
WORKDIR $UNDERLAY_WS
COPY ./Docker/dependencies.repos ./

# RUN apt-get update && apt-get install -y python3-rosidl-generator-py
# install dependencies underlay workspace
WORKDIR $UNDERLAY_WS
COPY ./underlay/ ./src/underlay

RUN mkdir -p src && \
    vcs import src < ./dependencies.repos && \
    rosdep init && \
    rosdep update && \
    SKIP_KEYS="\
        gz-transport12 \
        gz-common5 \
        gz-math7 \
        gz-msgs9 \
        gz-gui7 \
        gz-cmake3 \
        gz-sim7 \
        zenohc \
        gz-transport7 \
        gz-plugin2 \
    " && \
    rosdep install --from-paths ./ -i -y --rosdistro humble --ignore-src --skip-keys="$SKIP_KEYS" && \
    rm -rf /var/lib/apt/lists/*


# Build the underlay workspace
WORKDIR $UNDERLAY_WS
RUN . /opt/ros/humble/setup.sh && \
    MAKEFLAGS="-j2 -l2" colcon build \
        --symlink-install \
        --event-handlers=console_direct+ \
        --merge-install \
        --parallel-workers 2



FROM build_stage AS dev_stage

# build overlay stage
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS


# Build px4_sim separately so Docker can cache this large build layer.
# When only px4_sim changes, this layer will rebuild; other packages can reuse cache.
COPY ./localization_kf/px4_sim ./src/px4_sim

RUN . /opt/ros/humble/setup.sh && \
    . /opt/underlay_ws/install/setup.sh && \
    MAKEFLAGS="-j2 -l2" colcon build \
        --event-handlers=console_direct+ \
        --symlink-install \
        --merge-install \
        --packages-select px4_sim \
        # --executor sequential
        --parallel-workers 2

# This should be moved up for cleanup later
# Copy the rest of the workspace except the px4_sim directory so we don't overwrite
# the separately-built `src/px4_sim` and avoid rebuilding it unnecessarily.
COPY ./localization_kf/ /tmp/workspace_src

RUN rsync -a --exclude='px4_sim' /tmp/workspace_src/ ./src/ && \
    rm -rf /tmp/workspace_src && \
    . /opt/ros/humble/setup.sh && \
    . /opt/underlay_ws/install/setup.sh && \
    colcon build \
        --event-handlers=console_direct+ \
        --symlink-install \
        --merge-install \
        --packages-skip px4_sim \
        --parallel-workers 2

COPY ./Docker/entrypoint.sh /entrypoint.sh

# ENTRYPOINT [ "/entrypoint.sh" ]