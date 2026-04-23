FROM ubuntu:24.04

# Suppress interactive apt prompts throughout build and in inner setup scripts
ENV DEBIAN_FRONTEND=noninteractive
# Makes QGroundControl AppImage bypass FUSE (required in Docker without --privileged)
ENV APPIMAGE_EXTRACT_AND_RUN=1

# ── 1. Base packages ──────────────────────────────────────────────────────────
# dos2unix guards against Windows clones with core.autocrlf=true.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git gnupg lsb-release wget sudo \
    xauth \
    libfuse2 \
    python3 python3-pip \
    dos2unix \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Gazebo Harmonic ────────────────────────────────────────────────────────
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ── 3. PX4 build toolchain ────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle build-essential cmake cppcheck \
    file g++ gcc gdb \
    make ninja-build \
    python3-dev python3-setuptools \
    rsync unzip xxd libxml2-utils \
    && rm -rf /var/lib/apt/lists/*

# ── 4. Non-root user (dev, UID 1000) ──────────────────────────────────────────
# QGroundControl refuses to run cleanly as root. The container runs as `dev`;
# passwordless sudo is included because PX4's ubuntu.sh uses `sudo apt-get`
# internally.
RUN userdel --remove ubuntu 2>/dev/null || true \
    && useradd --create-home --uid 1000 --shell /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev

USER dev
WORKDIR /home/dev
ENV PATH="/home/dev/.local/bin:${PATH}"

# ── 5. uv ─────────────────────────────────────────────────────────────────────
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# ── 6. PX4 source + runtime deps + SITL build ─────────────────────────────────
# Pin PX4 by overriding PX4_REF at build time:
#   docker compose build --build-arg PX4_REF=v1.15.2
ARG PX4_REF=main
RUN git clone --depth 1 --branch ${PX4_REF} --recurse-submodules --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git /home/dev/PX4-Autopilot \
    && bash /home/dev/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
    && cd /home/dev/PX4-Autopilot && make px4_sitl_default

# ── 7. QGroundControl ─────────────────────────────────────────────────────────
RUN wget -q -O /home/dev/QGroundControl.AppImage \
        https://github.com/mavlink/qgroundcontrol/releases/latest/download/QGroundControl-x86_64.AppImage \
    && chmod +x /home/dev/QGroundControl.AppImage

# ── 8. Project Python deps (system-wide, reachable by dev) ────────────────────
USER root
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ── 9. Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh

USER dev
WORKDIR /workspace/Simulation
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
