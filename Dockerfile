# As start point we recommend to use a EduArt Docker image that comes with already installed libs, like ROS and lib MRAA.
FROM eduartrobotik/eduart-robot:0.5.0

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Add entry point
## A entry point sets up an environment, in this case ROS, that helps to launch the command.
COPY --chown=$USER ./entrypoint /home/$USER/
RUN sudo chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# Adding your sources and build it.
## The sources are copied into the Docker image.
RUN echo "coping sources" \
    # setup workspace
    && mkdir -p /home/$USER/ros/src/edu_robot_control_template/
    # cloning sources

ADD ./ /home/$USER/ros/src/edu_robot_control_template

## Building ROS packages
RUN echo "building package" \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && cd /home/$USER/ros \
    # Building project using colcon. It is important to use "MAKEFLAGS=-j1" on IoT2050, because of limited hardware resources.
    # On other platforms this flag could be removed.
    && MAKEFLAGS="-j1" colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
