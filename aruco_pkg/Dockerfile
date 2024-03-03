FROM ros:humble-perception

ENV DEBIAN_FRONTEND noninteractive


RUN apt-get update > /dev/null \
	&& apt-get install -y --no-install-recommends apt-utils > /dev/null \
	&& apt-get install -y build-essential > /dev/null \
	&& apt-get install -y afl > /dev/null \
	&& apt-get install -y vim > /dev/null \
	&& apt-get install -y git > /dev/null \
	&& apt-get install -y clang > /dev/null \
	&& apt-get install -y iproute2 > /dev/null \
	&& apt-get install -y lcov > /dev/null \
	&& apt-get install -y tar > /dev/null \
	&& apt-get install -y cmake > /dev/null \
	&& apt-get install -y software-properties-common > /dev/null \
	&& apt-get install -y graphviz-dev > /dev/null


RUN /bin/bash -c "source /opt/ros/humble/setup.bash"
ENV ROS_WS /opt/ros_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS
COPY . src/aruco_pkg
RUN chmod +x src/aruco_pkg/afl-config.bash
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
                  colcon build --packages-select aruco_pkg"

