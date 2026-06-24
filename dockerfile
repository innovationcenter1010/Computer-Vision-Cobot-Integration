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
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# Working directory
# =========================================================

WORKDIR /app

# seperate ultralitics dowsload to speed up dev build time
RUN pip3 install --break-system-packages \
    ultralytics


RUN pip3 install --break-system-packages \
    opencv-python-headless \
    numpy \
    pyyaml \
    fastapi \
    "uvicorn[standard]" \
    pyzmq \
    jinja2


# =========================================================
# Copy python project toml
# =========================================================

COPY python/pyproject.toml /tmp/python/pyproject.toml


# ========================================================
# Copy Source Code
# ========================================================

COPY . /app

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


# Port for web ui
EXPOSE 8000

# Ports for camera ZMQ streaming
EXPOSE 5550
EXPOSE 5551

# =========================================================
# Debugging
# =========================================================

RUN echo "=== BUILT MODULES ===" && \
    find /app/build -name "*.so"

# =========================================================
# Start app
# =========================================================

CMD ["python3", "-m", "cobot_application.main"]