FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# ── 1. Base packages ──────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git gnupg lsb-release wget sudo \
    python3 python3-pip \
    python3-opencv \
    dos2unix \
    && rm -rf /var/lib/apt/lists/*

# ── 2. ROS2 Jazzy ─────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    ros-jazzy-navigation2 \
    ros-jazzy-vision-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# ── 3. Non-root user (dev, UID 1000) ──────────────────────────────────────────
RUN userdel --remove ubuntu 2>/dev/null || true \
    && useradd --create-home --uid 1000 --shell /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev

USER dev
WORKDIR /home/dev
ENV PATH="/home/dev/.local/bin:${PATH}"

# ── 4. uv ─────────────────────────────────────────────────────────────────────
RUN curl -LsSf https://astral.sh/uv/install.sh | sh

# ── 5. Python AI deps via uv venv ─────────────────────────────────────────────
COPY --chown=dev:dev pyproject.toml /tmp/pyproject.toml
RUN uv venv /home/dev/.venv \
    && uv pip install --python /home/dev/.venv/bin/python -r /tmp/pyproject.toml

# ── 6. Shell setup ────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/dev/.bashrc

# ── 7. Entrypoint ─────────────────────────────────────────────────────────────
USER root
COPY docker/entrypoint.sh /entrypoint.sh
RUN dos2unix /entrypoint.sh && chmod +x /entrypoint.sh

USER dev
WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
