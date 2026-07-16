# =============================================================
# ROS2 (Humble) build — two stages, same philosophy as the legacy
# dockerfile: builder compiles, runtime gets no compilers.
#
# Cache layer order (most stable → least stable):
#   1. apt packages          — never change
#   2. pip: ultralytics      — LONGEST step (pulls torch), rarely changes.
#      Pinned FIRST so no manifest or source edit ever re-downloads it.
#   3. pip: remaining deps   — rarely change
#   4. package.xml manifests → rosdep  — change when a package's deps change
#   5. workspace source      → colcon  — changes constantly
# =============================================================


# =============================================================
# Stage 1: builder
# =============================================================
FROM ros:humble AS builder

ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. apt: never changes ----
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-pybind11-vendor \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. ultralytics: heaviest layer, cached before everything else ----
RUN pip3 install --no-cache-dir ultralytics

# ---- 3. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    opencv-python-headless \
    fastapi \
    "uvicorn[standard]" \
    pyzmq \
    jinja2

# ---- 3b. build-tooling compat ----
# The pip layers pull setuptools >=71 into /usr/local, which requires
# packaging >=22; Ubuntu 22.04's system packaging is 21.3 and breaks every
# `setup.py egg_info` with:
#   canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
# Pin both. setuptools <80 keeps `setup.py install`, which colcon still uses.
RUN pip3 install --no-cache-dir --upgrade "setuptools<80" "packaging>=22"

WORKDIR /ws

# ---- 4. rosdep layer: keyed on manifests only ----
COPY ros2_ws/src/cobot_interfaces/package.xml  src/cobot_interfaces/package.xml
COPY ros2_ws/src/cobot_camera/package.xml      src/cobot_camera/package.xml
COPY ros2_ws/src/cobot_vision/package.xml      src/cobot_vision/package.xml
COPY ros2_ws/src/motoman_cpp/package.xml       src/motoman_cpp/package.xml
COPY ros2_ws/src/cobot_control/package.xml     src/cobot_control/package.xml
COPY ros2_ws/src/cobot_tasks/package.xml       src/cobot_tasks/package.xml
COPY ros2_ws/src/cobot_web/package.xml         src/cobot_web/package.xml
COPY ros2_ws/src/cobot_bringup/package.xml     src/cobot_bringup/package.xml

RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf src

# ---- 5. source + build ----
COPY ros2_ws/src src

RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release


# =============================================================
# Stage 2: runtime
# Lean image — install tree only: launch files, message definitions,
# and the compiled pybind .so in one artifact. No compilers, no source.
# =============================================================
FROM ros:humble-ros-core AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. apt runtime libs ----
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. ultralytics: same earliest-layer rule as the builder ----
RUN pip3 install --no-cache-dir ultralytics

# ---- 3. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    opencv-python-headless \
    fastapi \
    "uvicorn[standard]" \
    pyzmq \
    jinja2

WORKDIR /ws

# ---- install tree from builder ----
COPY --from=builder /ws/install /ws/install

COPY docker/entrypoint.sh /entrypoint.sh
# Strip CRs in case the file was checked out with Windows line endings —
# a CRLF shebang breaks exec ("/usr/bin/env: 'bash\r': No such file").
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

# ---- Environment ----
# NOTE: model weights are NOT baked — the vision_workspace bind mount is the
# source of truth (the legacy baked copy was shadowed by the mount anyway).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    YOLO_CONFIG_DIR=/tmp/Ultralytics

EXPOSE 8000

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cobot_bringup", "robot.launch.py"]
