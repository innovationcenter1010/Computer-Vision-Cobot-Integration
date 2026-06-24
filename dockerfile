# =============================================================
# Stage 1: C++ Builder
# Compiles the MotomanRobot pybind extension.
# Nothing from this stage except the compiled package ends up
# in the final image.
#
# Cache layer order (most stable → least stable):
#   1. apt packages        — never changes
#   2. pip dependencies    — rarely changes
#   3. python/ source      — changes often, but doesn't trigger recompile
#   4. CMakeLists + cpp/   — changes during C++ dev, triggers recompile
#   5. cmake + make        — only runs when layer 4 changes
# =============================================================
FROM python:3.11-slim AS builder
 
ENV DEBIAN_FRONTEND=noninteractive
 
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*
 
WORKDIR /app
 
# ---- 2. Python deps — cached across all C++ rebuilds ----
RUN pip3 install --no-cache-dir ultralytics
 
RUN pip3 install --no-cache-dir \
    opencv-python-headless \
    numpy \
    pyyaml \
    fastapi \
    "uvicorn[standard]" \
    pyzmq \
    jinja2
 
# ---- 3. Python source — cmake writes the .so into this tree ----
# Copied before cpp/ so Python-only changes don't bust the compile cache
COPY python/ /app/python/
 
# ---- 4. C++ source — changing this is the only thing that triggers make ----
COPY CMakeLists.txt /app/CMakeLists.txt
COPY cpp/           /app/cpp/
 
# ---- 5. Compile ----
RUN mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)
 
 
# =============================================================
# Stage 2: Runtime
# Lean image — no compilers, no build artifacts, no training data.
# =============================================================
FROM python:3.11-slim AS runtime
 
ENV DEBIAN_FRONTEND=noninteractive
 
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    usbutils \
    && rm -rf /var/lib/apt/lists/*
 
WORKDIR /app
 
# ---- Python dependencies ----
RUN pip3 install --no-cache-dir ultralytics
 
RUN pip3 install --no-cache-dir \
    opencv-python-headless \
    numpy \
    pyyaml \
    fastapi \
    "uvicorn[standard]" \
    pyzmq \
    jinja2
 
# ---- Python package from builder ----
# Contains application source + motoman_cpp*.so + libYMConnect.so,
# all written into python/cobot_application/cobot/ by cmake.
COPY --from=builder /app/python/ /app/python/
 
# ---- AstraSDK runtime libs ----
COPY cpp/third_party/AstraSDK/lib/ /app/cpp/third_party/AstraSDK/lib/
 
RUN pip3 install --no-cache-dir -e ./python
 
# ---- YOLO model weights (runtime only, no training runs) ----
COPY yolo26m-obb.pt  /app/yolo26m-obb.pt
COPY yolo26n-obb.pt  /app/yolo26n-obb.pt
 
COPY vision_workspace/models/cans_obb_best.pt \
     /app/vision_workspace/models/cans_obb_best.pt
 
# ---- Environment ----
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics
# libYMConnect.so sits alongside motoman_cpp*.so; $ORIGIN RPATH in cmake
# handles resolution automatically. This is a fallback.
ENV LD_LIBRARY_PATH=/app/python/cobot_application/cobot:$LD_LIBRARY_PATH
 
EXPOSE 8000
EXPOSE 5550
EXPOSE 5551
 
CMD ["python3", "-m", "cobot_application.main"]