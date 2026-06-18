FROM python:3.11-slim

ENV DEBIAN_FRONTEND=noninteractive

# =========================================================
# System dependencies
# =========================================================

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3 \
    python3-dev \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# Working directory
# =========================================================

WORKDIR /app


RUN pip3 install --break-system-packages \
    ultralytics \
    opencv-python-headless \
    numpy \
    pyyaml \
    fastapi \
    "uvicorn[standard]" \
    jinja2


# =========================================================
# Copy python project toml
# =========================================================

COPY python/pyproject.toml /tmp/python/pyproject.toml


# ========================================================
# Copy Source Code
# ========================================================

COPY . /app

# compile the orbec sdk
RUN cd /app/python/cobot_application/third_party/orbbec_sdk_v2 && \
    mkdir -p build install && \
    cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=../install \
        -DCMAKE_CXX_FLAGS="-Wno-error -Wno-maybe-uninitialized" \
        -DCMAKE_C_FLAGS="-Wno-error" && \
    make -j$(nproc) && \
    make install

#copy the orbec sdk into the python wrapper 
RUN cp -r \
    /app/python/cobot_application/third_party/orbbec_sdk_v2/install \
    /app/python/cobot_application/third_party/orbbec_sdk_v2_python/

ENV ORBBEC_SDK_PATH=/app/python/cobot_application/third_party/orbbec_sdk_v2/install
ENV CMAKE_PREFIX_PATH=$ORBBEC_SDK_PATH:$CMAKE_PREFIX_PATH
ENV LD_LIBRARY_PATH=$ORBBEC_SDK_PATH/lib:$LD_LIBRARY_PATH

#compile the python wrappers
RUN pip3 install -e /app/python/cobot_application/third_party/orbbec_sdk_v2_python

#comile the python code
RUN pip3 install --break-system-packages -e ./python

# =========================================================
# Build project
# =========================================================

RUN mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# =========================================================
# Python dependencies
# =========================================================

RUN pip3 install --break-system-packages --no-cache-dir -e ./python

# =========================================================
# Runtime environment
# =========================================================

ENV LD_LIBRARY_PATH=/app/third_party/YMConnect:$LD_LIBRARY_PATH

ENV PYTHONPATH=/app/build/python:$PYTHONPATH

EXPOSE 8000

# =========================================================
# Debugging
# =========================================================

RUN echo "=== BUILT MODULES ===" && \
    find /app/build -name "*.so"

# =========================================================
# Start app
# =========================================================

CMD ["python3", "-m", "cobot_application.main"]