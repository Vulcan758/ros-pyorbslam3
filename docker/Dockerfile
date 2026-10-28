# ros-pyorbslam3 runtime image.
#   docker build -f docker/Dockerfile --build-arg ROS_DISTRO=humble -t ros-pyorbslam3:humble .
#   docker run --rm -it --net=host -v /path/to/data:/data ros-pyorbslam3:humble \
#     ros2 launch pyorbslam3_ros dataset.launch.py dataset_path:=/data/dataset-room1_512_16
# ORB-SLAM3 comes prebuilt in the pyorbslam3 wheel, so nothing heavy is compiled here.
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip libgl1 libegl1 libopengl0 libsm6 libice6 libxext6 libx11-6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . src/ros-pyorbslam3

RUN echo "yaml file:///ws/src/ros-pyorbslam3/rosdep/pyorbslam3.yaml" \
        > /etc/ros/rosdep/sources.list.d/50-pyorbslam3.list \
    && apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build && rm -rf log

# fetch ORBvoc.txt at build time so containers start offline
RUN python3 -c "import pyorbslam3; print(pyorbslam3.vocabulary_path())"

RUN sed -i 's|^exec "\$@"|. /ws/install/setup.bash\nexec "$@"|' /ros_entrypoint.sh
CMD ["bash"]
