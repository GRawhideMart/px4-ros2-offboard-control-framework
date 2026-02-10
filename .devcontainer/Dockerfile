FROM osrf/ros:humble-desktop-full

# 1. Installiamo i tool di base e le dipendenze per compilare PX4
RUN apt-get update && apt-get install -y \
    git \
    wget \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    gradle \
    openjdk-11-jdk \
    ros-humble-plotjuggler-ros \
    && rm -rf /var/lib/apt/lists/*

# 2. Installiamo le dipendenze Python necessarie per PX4 SITL
RUN pip3 install --no-cache-dir \
    kconfiglib \
    jinja2 \
    jsonschema \
    future \
    numpy \
    packaging \
    toml \
    pyros-genmsg

# 3. Installiamo e compiliamo il Micro-XRCE-DDS Agent (Il Bridge)
# Senza questo, ROS 2 non sente il drone.
WORKDIR /tmp
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git && \
    cd Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    make install && \
    ldconfig /usr/local/lib

# Usiamo wget per prendere solo lo script di setup dal branch 'main'
RUN wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/ubuntu.sh -O /tmp/ubuntu.sh \
    && wget https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/setup/requirements.txt -O /tmp/requirements.txt \
    && bash /tmp/ubuntu.sh --no-nuttx \
    && rm /tmp/ubuntu.sh /tmp/requirements.txt

# Setup dell'utente (opzionale, ma consigliato per evitare problemi di permessi con i file locali)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Torniamo alla workspace di default
WORKDIR /workspace

# Aggiungi il sourcing di ROS al .bashrc automaticamente ad ogni build
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc