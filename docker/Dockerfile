# As start point we recommend to use a EduArt Docker image that comes with already installed libs, like ROS and MRAA.
FROM eduartrobotik/eduart-robot:0.3.4

ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# Add entry point
## A entry point sets up an environment, in this case ROS, that helps to launch the command.
COPY --chown=$USER ./entrypoint /home/$USER/
RUN sudo chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# Adding your sources and build it.
## Either download it or copy it into the image. Below git is used for downloading it. If you want to copy it use COPY instruction like in line 6.
RUN echo "cloning sources" \
    # setup workspace
    && mkdir -p /home/$USER/ros/src/ \
    # cloning sources
    && git clone git@github.com:EduArt-Robotik/edu_robot_control_template.git \

## Building ROS packages
RUN echo "building package" \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    # Building project using colcon. It is important to use "MAKEFLAGS=-j1" on IoT2050, because of limited hardware resources.
    && MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
