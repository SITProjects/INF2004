FROM mcr.microsoft.com/devcontainers/base:bookworm

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install ARM GCC cross-compiler, Newlib standard library, and build tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    git \
    clang-format \
    python3 \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/pre-commit \
    && /opt/pre-commit/bin/pip install --no-cache-dir pre-commit

RUN git clone \
        --depth 1 \
        https://github.com/raspberrypi/pico-sdk.git \
        /opt/pico-sdk && \
    cd /opt/pico-sdk && \
    git submodule update --init --recursive

ENV PICO_SDK_PATH=/opt/pico-sdk

RUN chmod -R a+rX /opt/pico-sdk

ENV PATH="/opt/pre-commit/bin:${PATH}"

ENV DEBIAN_FRONTEND=dialog
