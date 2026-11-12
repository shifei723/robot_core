FROM ros:jazzy-ros-base
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-jazzy-clearpath-simulator ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-ros-gz \
 ros-jazzy-pcl-conversions ros-jazzy-cv-bridge ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest \
 python3-colcon-common-extensions python3-numpy python3-scipy python3-matplotlib python3-fonttools \
 python3-pytest python3-catkin-pkg build-essential libeigen3-dev libnlopt-cxx-dev \
 libgl1-mesa-dri libegl1 mesa-utils fonts-noto-cjk fonts-croscore && rm -rf /var/lib/apt/lists/*
ENV QT_QPA_PLATFORM=offscreen LIBGL_ALWAYS_SOFTWARE=1
CMD ["bash"]
