# ArUco-Tracking

Real-time object tracking and pose estimation system built with C++17 and OpenCV. The application utilizes computer vision to detect physical ArUco markers, compute their 3D spatial pose via the SolvePnP algorithm, and map their physical movements to virtual entities on screen.

## Features

- **Real-time Pose Estimation:** Extracts 3D position and orientation of physical markers using `cv::solvePnP`.
- **Euler Angles Computation:** Converts Rodrigues rotation matrices into Euler angles to accurately map physical rotations to virtual vehicles.
- **Collision Detection:** Implements a Euclidean distance-based collision detection system between virtual vehicles and target objects.
- **ArUco Marker Detection:** Utilizes OpenCV's ArUco module (dictionary `DICT_6X6_250`) for robust tracking of marker IDs 0 and 1.

## Tech Stack

- **C++17**
- **OpenCV 4.x** (Modules: `core`, `highgui`, `calib3d`, `aruco`)
- **CMake**

## ⚠️ Physical Markers & Calibration (CRITICAL)

For the spatial calibration and pose estimation to function correctly, **the physical ArUco markers must be printed with an exact side length of 30 mm**. 

Failure to maintain this precise physical dimension will result in incorrect 3D spatial calculations and erratic virtual tracking.

Ready-to-print vector files (`.svg`) for the required markers (ID 0 and ID 1 from the `DICT_6X6_250` dictionary) are located in the [`/markers`](./markers) directory of this repository.

## Build Instructions

This project uses CMake for building. Ensure you have a C++17 compatible compiler, CMake, and OpenCV 4.x installed on your system.

```bash
# 1. Clone the repository
git clone https://github.com/sebciulina/ArUco-Tracking.git
cd ArUco-Tracking

# 2. Create build directory
mkdir build
cd build

# 3. Configure and build
cmake ..
cmake --build .
```

## Running the Application

After a successful build, ensure your web camera is connected and run the executable from the terminal:

- **Linux/macOS:** `./ArUcoTracker`
- **Windows:** `Debug\ArUcoTracker.exe` or `Release\ArUcoTracker.exe`

Press `ESC` while the application window is focused to exit.
