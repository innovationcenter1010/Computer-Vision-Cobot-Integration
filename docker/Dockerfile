# =============================================================
# ROS2 (Humble) build — two stages, same philosophy as the legacy
# dockerfile: builder compiles, runtime gets no compilers.
#
# Cache layer order (most stable → least stable):
#   1. apt packages          — never change
#   2. onnxruntime tarball   — one pinned download, rarely changes.
#      (ultralytics / torch are NOT in the image any more: training and the
#      .pt -> .onnx export happen on the dev PC, see vision_workspace/tools.)
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
    curl \
    ca-certificates \
    python3-pip \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- 2. ONNX Runtime: official prebuilt release, unpacked to /opt/onnxruntime ----
# cobot_detector links against it (CMake ONNXRUNTIME_ROOT) and installs the
# .so next to its executable, so the runtime stage needs nothing extra.
ARG ORT_VERSION=1.19.2
RUN curl -fsSL -o /tmp/ort.tgz \
      https://github.com/microsoft/onnxruntime/releases/download/v${ORT_VERSION}/onnxruntime-linux-x64-${ORT_VERSION}.tgz \
    && mkdir -p /opt/onnxruntime \
    && tar -xzf /tmp/ort.tgz -C /opt/onnxruntime --strip-components=1 \
    && rm /tmp/ort.tgz

# ---- 3. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    "numpy==1.26.4" \
    "opencv-python-headless==4.10.0.84" \
    "fastapi==0.143.1" \
    "uvicorn[standard]==0.54.0" \
    "pyzmq==27.2.0" \
    "jinja2==3.1.6"

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
COPY ros2_ws/src/cobot_health/package.xml      src/cobot_health/package.xml
COPY ros2_ws/src/cobot_camera/package.xml      src/cobot_camera/package.xml
COPY ros2_ws/src/cobot_vision/package.xml      src/cobot_vision/package.xml
COPY ros2_ws/src/cobot_detector/package.xml    src/cobot_detector/package.xml
COPY ros2_ws/src/cobot_control/package.xml     src/cobot_control/package.xml
COPY ros2_ws/src/cobot_operation/package.xml   src/cobot_operation/package.xml
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
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DONNXRUNTIME_ROOT=/opt/onnxruntime

# Unit tests for the C++ detector core (decode / NMS / letterbox / registry)
# and the cobot_control controller-profile parsers / setup loader run at
# build time so a regression fails the image, not the robot. The golden test
# skips itself here (models live in the bind mount, not the image).
RUN . /opt/ros/humble/setup.sh && . install/setup.sh && \
    colcon test --merge-install --packages-select cobot_detector cobot_control \
      --event-handlers console_direct+ && \
    colcon test-result --verbose


# =============================================================
# Stage 2: runtime
# Lean image — install tree only: launch files, message definitions,
# and the compiled C++ nodes (cobot_control + libYMConnect.so, cobot_detector
# + libonnxruntime.so beside them) in one artifact. No compilers, no source.
# =============================================================
FROM ros:humble-ros-core AS runtime

ENV DEBIAN_FRONTEND=noninteractive

# ---- 1. apt runtime libs ----
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rclcpp-action \
    libyaml-cpp0.7 \
    libopencv-core4.5d \
    libopencv-imgproc4.5d \
    libopencv-imgcodecs4.5d \
    && rm -rf /var/lib/apt/lists/*

# (no ultralytics / torch: cobot_detector runs the ONNX models through the
#  libonnxruntime.so installed beside it under /ws/install)

# ---- 2. remaining pip deps ----
RUN pip3 install --no-cache-dir \
    "numpy==1.26.4" \
    "opencv-python-headless==4.10.0.84" \
    "fastapi==0.143.1" \
    "uvicorn[standard]==0.54.0" \
    "pyzmq==27.2.0" \
    "jinja2==3.1.6"

WORKDIR /ws

# ---- install tree from builder ----
COPY --from=builder /ws/install /ws/install

COPY docker/entrypoint.sh /entrypoint.sh
# Strip CRs in case the file was checked out with Windows line endings —
# a CRLF shebang breaks exec ("/usr/bin/env: 'bash\r': No such file").
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

# ---- Environment ----
# NOTE: models are NOT baked — the runtime_models bind mount is the source
# of truth: /ws/runtime_models/models.yaml + <name>/<name>.onnx (see
# cobot_detector/config/models.example.yaml).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

EXPOSE 8000

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "cobot_bringup", "robot.launch.py"]
