# Stage 1: Build & C++ Unit Tests
FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    libzmq3-dev \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY CMakeLists.txt ./
COPY icd ./icd
COPY include ./include
COPY src ./src
COPY tests ./tests

RUN cmake -B build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j"$(nproc)" && \
    ctest --test-dir build --output-on-failure

# Stage 2: Runtime + Python Ground Station & E2E Test Harness
FROM ubuntu:24.04 AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    libzmq5 \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/aethersat-sil

COPY --from=builder /workspace/build/aethersat_sim /usr/local/bin/aethersat_sim
COPY icd ./icd
COPY ground_station ./ground_station
COPY tests/python ./tests/python

RUN pip3 install --no-cache-dir --break-system-packages -r ground_station/requirements.txt

EXPOSE 5555 5556

ENTRYPOINT ["/usr/local/bin/aethersat_sim"]
CMD ["--pub-endpoint", "tcp://*:5555", "--cmd-endpoint", "tcp://*:5556", "--rate-hz", "50", "--time-scale", "1.0"]
