FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    bluez \
    dbus-x11 \
    fluxbox \
    joystick \
    libarmadillo-dev \
    libeigen3-dev \
    libgl1-mesa-dri \
    libnlopt-dev \
    liboctomap-dev \
    libopencv-dev \
    libpcl-dev \
    libusb-dev \
    mesa-utils \
    python3-bluez \
    python3-matplotlib \
    qtbase5-dev \
    ros-noetic-cv-bridge \
    ros-noetic-diagnostic-updater \
    ros-noetic-laser-geometry \
    ros-noetic-mavros-msgs \
    ros-noetic-nodelet \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-roslint \
    ros-noetic-rqt-gui-cpp \
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-tf2 \
    tigervnc-common \
    tigervnc-standalone-server \
    x11-utils \
    xfonts-base \
    xterm \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libnlopt-cxx-dev \
    libusb-1.0-0-dev \
 && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" ros \
 && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home \
      --shell /bin/bash ros

# The diff-drive launch publishes the Scout URDF; git records benchmark provenance.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git ros-noetic-robot-state-publisher \
 && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /usr/local/bin/flowforge-entrypoint
RUN chmod 0755 /usr/local/bin/flowforge-entrypoint
ENV FLOWFORGE_CONTAINER=1 \
    DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    QT_X11_NO_MITSHM=1 \
    DISABLE_ROS1_EOL_WARNINGS=1
USER ros
ENTRYPOINT ["/usr/local/bin/flowforge-entrypoint"]
