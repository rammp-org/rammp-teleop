# syntax=docker/dockerfile:1
# rammp-teleop: operator-input nodes (Xbox pad, Meta Quest 3) on the fleet's
# Cyclone DDS graph. rammp-base owns ROS 2 Humble, Cyclone and the pinned
# rammp-interfaces-ros2 in /ros2_ws; this builds rammp_teleop into /module_ws.
# arm64/Jetson only, like every rammp-base image.
FROM ghcr.io/rammp-org/rammp-base:1.0.0-jp6

# joy: the Xbox pad. adb + git-lfs: the Quest (oculus_reader pushes an APK to
# the headset over adb; the APK ships through LFS, so lfs must be installed
# BEFORE the clone or you get a 132-byte pointer and a silent failure later).
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-humble-joy android-tools-adb git git-lfs python3-pip python3-scipy \
 && rm -rf /var/lib/apt/lists/* \
 && git lfs install --system

RUN git clone --depth 1 https://github.com/rail-berkeley/oculus_reader.git /opt/oculus_reader \
 && pip3 install --no-cache-dir -e /opt/oculus_reader

# rammp-base ships rosdep uninitialised; the index must exist before
# `rosdep install` below can resolve package.xml keys.
RUN rosdep update --rosdistro humble

WORKDIR /module_ws
COPY . /module_ws/src/rammp-teleop/

RUN . /opt/ros/humble/setup.sh \
 && . /ros2_ws/install/setup.sh \
 && apt-get update \
 && rosdep install --from-paths src --ignore-src -y \
 && rm -rf /var/lib/apt/lists/* \
 && colcon build \
 && rm -rf /module_ws/build /module_ws/log

# Default: the Xbox pad. Fragments override this per alternative. `ros2 launch`
# forwards SIGTERM to its children, which scripts/smoke.sh checks.
CMD ["ros2", "launch", "rammp_teleop", "xbox.launch.py"]
