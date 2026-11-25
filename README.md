# AetherSat-SIL: Spacecraft Software-in-the-Loop (SIL) Simulator

A production-grade C++17 & Python 3 **Spacecraft Software-in-the-Loop (SIL) Simulator** targeting Ubuntu 24.04 / Docker. AetherSat-SIL models LEO orbital mechanics, rigid-body quaternion attitude dynamics with reaction-wheel PD control, electrical power generation/eclipse cycling, authentic virtual avionics buses (MIL-STD-1553B, CAN 2.0A, UART), CCSDS 133.0-B-2 Space Packet framing with CRC-16-CCITT, and ZeroMQ telemetry/telecommand streaming to an ICD-driven Python Ground Station.

## Architecture

- **Physics (`include/aethersat/physics/`, `src/physics/`)**
  - `rk4.hpp`: Generic templated 4th-order Runge-Kutta integrator over `std::array<double, N>`.
  - `orbital_propagator`: 2-body LEO propagator ($\mu = 398600.4418\text{ km}^3/\text{s}^2$, $R_E = 6378.137\text{ km}$, ~500 km circular orbit) with cylindrical Earth-shadow eclipse model.
  - `attitude_adcs`: Unit-quaternion kinematics, Euler rigid-body dynamics ($\mathbf{I} = \text{diag}(12, 12, 8)\text{ kg}\cdot\text{m}^2$), quaternion-error PD controller, reaction-wheel torque/momentum limits, and `WHEEL_SATURATION` fault injection.
  - `power_eps`: Solar generation (120 W sunlit / 0 W eclipse), bus load (45 W nominal / 20 W `SAFE_MODE`), battery SoC (0–100%), and bus voltage (24.0–28.8 V).
- **CCSDS 133.0-B-2 Codec (`include/aethersat/ccsds/`, `src/ccsds/`)**
  - Exact big-endian 6-byte primary header packing + trailing 2-byte CRC-16-CCITT (`poly=0x1021`, `init=0xFFFF`).
  - `encode()` / `decode()` returning `std::optional<SpacePacket>`.
- **ICD-Driven Virtual Avionics Buses (`icd/spacecraft_icd.json`, `include/aethersat/bus/`, `src/bus/`)**
  - **UART** (9600/115200 baud, `0xAA 0x55` sync + XOR checksum): Star Tracker attitude quaternion stream.
  - **CAN 2.0A** (11-bit ID `0x101` cmd / `0x201` telemetry, 8-byte payload): Reaction Wheel torque & momentum.
  - **MIL-STD-1553B** (20-bit Command/Status/Data words with sync & odd parity, RT-04): EPS Power Controller with `BUS_1553_TIMEOUT` fault injection triggering Flight Software `SAFE_MODE`.
- **ZeroMQ Transport & Python Ground Station (`src/main.cpp`, `ground_station/`)**
  - Binary CCSDS telemetry over ZMQ `PUB` (APID `0x100` ADCS/Orbit, APID `0x200` EPS/Health).
  - Binary CCSDS telecommands over ZMQ `REP` (APID `0x300`: `CMD_SET_ATTITUDE`, `CMD_SET_MODE`, `CMD_INJECT_FAULT`).

## Building & Testing (Ubuntu 24.04)

```bash
sudo apt-get update && sudo apt-get install -y \
  build-essential cmake pkg-config libzmq3-dev python3 python3-pip

pip3 install --break-system-packages -r ground_station/requirements.txt

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"

# Run C++ GoogleTest unit tests
ctest --test-dir build --output-on-failure

# Run Python end-to-end integration tests
pytest -v
```

## Running the Simulator & Ground Station

```bash
# Terminal 1: Start simulator
./build/aethersat_sim --pub-endpoint tcp://*:5555 --cmd-endpoint tcp://*:5556 --rate-hz 50 --time-scale 1.0

# Terminal 2: Stream decoded telemetry
python3 ground_station/ground_control.py --listen

# Send a 30-degree Z-axis slew telecommand
python3 ground_station/ground_control.py --slew 0.9659258 0.0 0.0 0.2588190

# Inject a MIL-STD-1553B timeout fault (triggers SAFE_MODE)
python3 ground_station/ground_control.py --inject-fault BUS_1553_TIMEOUT

# Command SAFE_MODE directly
python3 ground_station/ground_control.py --safe-mode
```

## Docker

```bash
docker build -t aethersat-sil .
docker run --rm -p 5555:5555 -p 5556:5556 aethersat-sil
```
