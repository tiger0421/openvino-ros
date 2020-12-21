FROM openvino/ubuntu18_dev:2020.3

USER root
ENV DEBIAN_FRONTEND=noninteractive
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1
RUN sed -i 's#http://tw.archive.ubuntu.com/#http://archive.ubuntu.com/#' /etc/apt/sources.list && \
    apt update && \
    apt install -y \
        dpkg \
        software-properties-common \
        git \
        curl \
        wget \
        lsb-release \
        python3-argcomplete \
        gnupg2 \
        dirmngr \
        g++ \
        fluid \
        build-essential \
        vim \
        tmux \
        sudo \
        locales \
        bash-completion \
        tzdata \
    && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' && \
    apt update && \
    apt install -y \
        ros-eloquent-desktop \
        python3-colcon-common-extensions \
    && \
    apt autoremove -y && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*

# add user
ARG DOCKER_UID=1000
ARG DOCKER_USER=openvino
ARG DOCKER_PASSWORD=openvino
RUN gpasswd -a ${DOCKER_USER} sudo \
    && echo ${DOCKER_USER}:${DOCKER_PASSWORD} | chpasswd

USER openvino
RUN echo source /opt/ros/eloquent/setup.bash >> /home/openvino/.bashrc
