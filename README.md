# GTSAM Examples

This repository provides minimal working examples demonstrating how to use [GTSAM](https://github.com/borglab/gtsam) for 2D pose graph optimization and localization tasks using `Pose2`, `NonlinearFactorGraph`, and `LevenbergMarquardtOptimizer`.

## Directory Structure

```
gtsam-examples/
├── src/
│   ├── gtsam.hpp             # Aggregated GTSAM headers for ease of use
│   ├── odometry.cpp          # Odometry-only optimization example
│   └── localization.cpp      # Example with prior + odometry
├── CMakeLists.txt            # CMake build script
├── .gitignore
├── LICENSE
└── README.md
```

## Build Instructions

### Prerequisites

Ensure you have installed:

- GTSAM (compiled and installed via `make install`)
- Eigen3
- TBB (Threading Building Blocks)

On Ubuntu:

```bash
sudo apt install libeigen3-dev libtbb-dev
```

### Build

```bash
mkdir build && cd build
cmake ..
make
```

## Running the Examples

After building:

```bash
./odometry
./localization
```

## License

This project is licensed under the MIT License.