FROM mxwilliam/mxck:mxck-humble-ubuntu-22.04

# Completely clean and reconfigure ROS repository
RUN rm -f /etc/apt/sources.list.d/ros2*.list \
&& rm -f /usr/share/keyrings/ros* \
&& curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update system and install ROS packages for video/image extraction
RUN apt update \
&& apt install --yes --no-install-recommends \
    ros-$ROS_DISTRO-image-view \
    ros-$ROS_DISTRO-image-transport \
&& rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    git+https://github.com/william-mx/ros2_numpy.git

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN echo 'source /ros_entrypoint.sh' >> ~/.bashrc

COPY ./.bash_aliases /root/.bash_aliases
