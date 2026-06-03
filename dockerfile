FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# =========================================================
# System dependencies
# =========================================================

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# =========================================================
# Working directory
# =========================================================

WORKDIR /app

# =========================================================
# Copy project
# =========================================================

COPY . /app

# =========================================================
# Build project
# =========================================================

RUN mkdir -p build && cd build && \
    cmake .. && \
    make -j$(nproc)

# =========================================================
# Python dependencies
# =========================================================

RUN pip3 install --break-system-packages --no-cache-dir \
    -r python/requirements.txt

# =========================================================
# Runtime environment
# =========================================================

ENV LD_LIBRARY_PATH=/app/third_party/YMConnect:$LD_LIBRARY_PATH

ENV PYTHONPATH=/app/build/python:$PYTHONPATH

# =========================================================
# Debugging
# =========================================================

RUN echo "=== BUILT MODULES ===" && \
    find /app/build -name "*.so"

# =========================================================
# Start app
# =========================================================

CMD ["python3", "python/tests/motion_polling_test.py"]

