FROM ros:humble

# Example of installing programs
RUN apt-get update && \
    apt-get install -y \
    nano \
    vim \
    bash-completion \
    python3-argcomplete \
    python3-pip \
    terminator \
    ros-humble-usb-cam \
    ros-humble-aruco \
    ros-humble-rmw-cyclonedds-cpp \
    tmux \
    v4l-utils \
    cmake pkg-config \
    swig \
    && rm -rf /var/lib/apt/lists/*



# Create a non-root user
ARG USERNAME=pi5
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN usermod -aG dialout ${USERNAME}

# Install Python packages using pip3
USER $USERNAME
RUN pip3 install --upgrade pip \
    && pip3 uninstall -y serial \
    && pip3 install pyserial \
    && pip3 install numpy \
    && pip install scipy \
    && pip install opencv-contrib-python

# Create the log directory with the correct permissions
# RUN mkdir -p /log/build_$(date '+%Y-%m-%d_%H-%M-%S') \
#     && chown -R $USERNAME:$USERNAME /log

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entry_point.sh /home/${USERNAME}/entry_point.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Clone the Git repository
RUN git clone https://github.com/Fangtnw/MCE07_Senior_project.git /home/${USERNAME}/coop_ws/src
# RUN git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git /home/${USERNAME}/coop_ws/src/ydlidar_ros2_driver
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /home/${USERNAME}/YDLidar-SDK

# Add commands to build and install YDLidar-SDK
WORKDIR /home/${USERNAME}/YDLidar-SDK
RUN mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install \
    && cd .. \
    && pip install . \
    && cd 

# Set permissions for /dev/ttyUSB0
# RUN sudo chmod 777 /dev/ttyUSB0

# Build ROS workspace

# WORKDIR /home/${USERNAME}/coop_ws/src
# RUN rm -r sim_car my_robot_description

USER root
# RUN chown -R ${USERNAME}:${USERNAME} /home/${USERNAME} \
#     && echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# USER ${USERNAME}
# WORKDIR /home/${USERNAME}/coop_ws    
# RUN colcon build --symlink-install

# # Source ROS workspace setup script

# RUN echo "source /home/${USERNAME}/coop_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc


# USER ${USERNAME}
# Set the default command
RUN chmod +x /home/pi5/entry_point.sh
CMD ["bash" ,"/home/pi5/entry_point.sh"]

