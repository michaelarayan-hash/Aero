FROM ubuntu:24.04

# Suppress interactive apt prompts throughout build and in setup.sh
ENV DEBIAN_FRONTEND=noninteractive
# Makes QGroundControl AppImage bypass FUSE (required in Docker without --privileged)
ENV APPIMAGE_EXTRACT_AND_RUN=1
# uv install dir
ENV PATH="/root/.local/bin:$PATH"

# ── 1. Base packages ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git gnupg lsb-release wget sudo \
    xauth \
    libfuse2 \
    python3 python3-pip \
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
# Installs cmake, ninja, gcc/g++ and other build essentials.
# PX4's ubuntu.sh (run by setup.sh on first container start) installs remaining
# runtime deps (Python packages, simulation-specific tools) after cloning PX4.
RUN apt-get update && apt-get install -y --no-install-recommends \
    astyle build-essential cmake cppcheck \
    file g++ gcc gdb \
    make ninja-build \
    python3-dev python3-setuptools \
    rsync unzip xxd libxml2-utils \
    && rm -rf /var/lib/apt/lists/*

# ── 4. uv ─────────────────────────────────────────────────────────────────────
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# ── 5. Python project deps ────────────────────────────────────────────────────
# Pre-install from requirements.txt so algorithm scripts work immediately.
# Build context is Simulation/ so requirements.txt is at the root of context.
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ── 6. Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /workspace/Simulation
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
