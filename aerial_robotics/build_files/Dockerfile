#
# Dockerfile unificato per PX4 + ROS2 Humble + Gazebo Harmonic
# Costruito per arm64 (Apple M-series) con UID/GID passati alla build
#

# 1. SI PARTE DA UBUNTU 22.04 (che supporta arm64)
FROM ubuntu:22.04

# 2. IMPOSTAZIONE DEGLI ENV
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV TZ=UTC
ENV TERM=xterm
ENV CCACHE_UMASK=000
ENV PATH="/usr/lib/ccache:$PATH"
ENV DISPLAY :99

USER root
WORKDIR /

# 3. INSTALLAZIONE PACCHETTI (Base + Tuo File)
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
     build-essential \
    ant \
    binutils-dev \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    dirmngr \
    doxygen \
    gdb \
    gettext \
    gnupg \
    gosu \
    lcov \
    libelf-dev \
    libexpat-dev \
    libvecmath-java \
    libfreetype6-dev \
    libgmp-dev \
    libgtest-dev \
    libisl-dev \
    libmpc-dev \
    libmpfr-dev \
    libpng-dev \
    libssl-dev \
    make \
    ninja-build \
    openssh-client \
    openjdk-11-jre \
    openjdk-11-jdk \
    python3 \
    python3-dev \
    python3-venv \
    rsync \
    screen \
    shellcheck \
    tzdata \
    texinfo \
    u-boot-tools \
    util-linux \
    unzip \
    valgrind \
    xsltproc \
    zip \
    git \
    bash-completion \
    gedit \
    sudo \
    locales \
    software-properties-common \
    curl \
    nano \
    wget \
    python3-pip \
    lsb-release \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && add-apt-repository universe \
 && update-alternatives --set java $(update-alternatives --list java | grep "java-11") \
 && apt-get -y autoremove \
 && apt-get clean autoclean \
 && rm -rf /var/lib/apt/lists/*

# 4. GTest
RUN cd /usr/src/gtest \
    && mkdir build && cd build \
    && cmake .. && make -j$(nproc) \
    && find . -name \*.a -exec cp {} /usr/lib \; \
    && cd .. && rm -rf build

# 5. Dipendenze Python
RUN python3 -m pip install --upgrade pip wheel setuptools
RUN python3 -m pip install argparse argcomplete coverage cerberus empy==3.3.4 jinja2 kconfiglib \
    matplotlib>=3.0.* numpy nunavut>=1.1.0 packaging pkgconfig pyros-genmsg pyulog \
    pyyaml requests serial six toml psutil pyulog wheel jsonschema pynacl lxml

# 6. Setup Ccache
RUN ln -s /usr/bin/ccache /usr/lib/ccache/cc \
    && ln -s /usr/bin/ccache /usr/lib/ccache/c++

# 7. Astyle
RUN wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
    && cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
    && make -f ../build/gcc/Makefile -j$(nproc) && cp bin/astyle /usr/local/bin \
    && rm -rf /tmp/*

# 8. CREAZIONE UTENTE (MODIFICATA PER IL TUO SCRIPT DI BUILD)
# Dichiara gli argomenti che riceveremo dal comando `docker build`
ARG USER_ID
ARG GROUP_ID

# Crea un gruppo e un utente corrispondenti all'ID del tuo host
# Questo evita problemi di permessi con i volumi montati
RUN groupadd -g $GROUP_ID user && \
    useradd --shell /bin/bash -u $USER_ID -g $GROUP_ID -c "" -m user && \
    usermod -a -G dialout user

# Aggiungi l'utente a gruppi utili (sudo, video) per maggior flessibilità
RUN usermod -aG sudo,video user

# 9. Setup X Server
RUN mkdir /tmp/.X11-unix && \
    chmod 1777 /tmp/.X11-unix && \
    chown -R root:root /tmp/.X11-unix

# 10. ROS2 Humble
# (Nota: Eseguiamo come root, ma il sourcing di ROS avverrà nell'entrypoint per l'utente)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y ros-humble-desktop ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*
# Scriviamo il source nel .bashrc dell'UTENTE che abbiamo appena creato
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=1" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=27" >> /root/.bashrc \
    && su user -c "python3 -m pip install --user -U empy==3.3.4 pyros-genmsg setuptools"

# 11. Micro-XRCE-DDS-Agent
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /Micro-XRCE-DDS-Agent \
 && cd /Micro-XRCE-DDS-Agent \
 && mkdir build \
 && cd build \
 && cmake .. \
 && make -j"$(nproc)" \
 && make install \
 && ldconfig /usr/local/lib/

# 12. PlotJuggler
RUN apt-get update && apt-get install -y \
    ros-humble-plotjuggler-ros \
 && rm -rf /var/lib/apt/lists/*

# 13. Gazebo Harmonic
RUN apt-get update \
 && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update \
 && apt-get install -y \
      gz-harmonic \
      libgz-sim8-dev \
      libgz-transport13-dev \
      libgz-sensors8-dev \
      libgz-plugin2-dev \
      libgz-utils2-dev \
      libgz-msgs10-dev \
      protobuf-compiler \
      libprotobuf-dev \
      libopencv-dev \
 && rm -rf /var/lib/apt/lists/*

# 14. ROS–Gazebo bridge (ros_gz)
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-ign-ros2-control-demos \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-usb-cam \
    ros-humble-image-pipeline \
    ros-humble-tf-transformations \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
 && rm -rf /var/lib/apt/lists/*

# 15. PX4-Autopilot git safety
RUN git config --global --add safe.directory '*'

WORKDIR /ros2_ws

# 16. EXPOSE, ENTRYPOINT e CMD
EXPOSE 14556/udp
EXPOSE 14557/udp

# Copia l'entrypoint script (vedi punto 2 sotto)
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Azione di default: avvia una shell bash
CMD ["/bin/bash"]