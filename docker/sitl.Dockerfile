FROM ubuntu:20.04

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ARG ARDUPILOT_REPO=https://github.com/lmhonorio/ardupilot.git
ARG ARDUPILOT_REF=master

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    ca-certificates \
    rsync \
    sudo \
    bash-completion \
    lsb-release \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python-is-python3 \
    ccache \
    build-essential \
    gcc \
    g++ \
    make \
    pkg-config \
    gawk \
    wget \
    valgrind \
    screen \
    python3-pexpect \
    libtool \
    libxml2-dev \
    libxslt1-dev \
    python3-setuptools \
    python3-numpy \
    python3-pyparsing \
    python3-psutil \
    xterm \
    xfonts-base \
    python3-matplotlib \
    python3-serial \
    python3-scipy \
    python3-opencv \
    python3-yaml \
    libcsfml-dev \
    libsfml-dev \
 && rm -rf /var/lib/apt/lists/*

RUN git clone --recurse-submodules ${ARDUPILOT_REPO} /opt/ardupilot && \
    cd /opt/ardupilot && \
    git checkout ${ARDUPILOT_REF} && \
    git submodule update --init --recursive

RUN python3 -m pip install --no-cache-dir --upgrade pip && \
    python3 -m pip install --no-cache-dir \
      empy==3.3.4 \
      pexpect \
      ptyprocess \
      pyserial \
      MAVProxy \
      pymavlink \
      geocoder \
      dronecan \
      flake8 \
      future \
      junitparser \
      wsproto \
      tabulate

ENV PATH="/opt/ardupilot/Tools/autotest:${PATH}"

RUN cd /opt/ardupilot && \
    ./waf configure --board sitl && \
    ./waf rover

COPY docker/entrypoints/sitl_entrypoint.sh /usr/local/bin/sitl_entrypoint.sh
COPY docker/entrypoints/run_sitl.sh /usr/local/bin/run_sitl.sh
RUN chmod 755 /usr/local/bin/sitl_entrypoint.sh /usr/local/bin/run_sitl.sh

WORKDIR /opt/ardupilot/Rover
ENTRYPOINT ["/usr/local/bin/sitl_entrypoint.sh"]
CMD ["/usr/local/bin/run_sitl.sh"]
