# Calibration and Verification Tools

This repository contains tools for camera calibration and verification using ArUco markers.

## Prerequisites

*   Python 3.8+
*   OpenCV compatible camera (e.g., Raspberry Pi HQ Camera)
*   Printed Checkerboard (for calibration)
*   Printed ArUco Marker (any supported family, for verification)

## Setup

### 1. Create and Activate Virtual Environment

#### Option A: Using venv (Standard Python)

**Windows:**
```powershell
python -m venv venv
.\venv\Scripts\Activate
```

**Linux/macOS:**
```bash
python3 -m venv venv
source venv/bin/activate
```

#### Option B: Using Conda

If you prefer Conda, you can create an environment from the provided `environment.yml` file:

```bash
conda env create -f environment.yml
conda activate aero_calib
```

### 2. Install Dependencies

If you used **Option A (venv)**, install the required packages:

```bash
pip install -r requirements.txt
```

*(Note: If you used Option B, dependencies were installed automatically.)*

## Usage

### Step 1: Capture Calibration Images

Capture a set of images of a checkerboard from different angles.

```bash
python Code/Callibration/1_capture_images.py
```

*   **SPACE**: Capture image
*   **D**: Delete last image
*   **Q**: Quit

Aim for at least 20-30 diverse images. They will be saved in `calib_images/`.

### Step 2: Calibrate Camera

Process the captured images to generate the camera matrix and distortion coefficients.

```bash
python Code/Callibration/2_calibrate.py
```

*   **Input**: Images from `calib_images/`
*   **Output**: `calibration.npz` (contains intrinsic parameters)
*   **Preview**: Undistorted sample images in `undistorted/`

### Step 3: Detect ArUco Markers

Detect ArUco and AprilTag markers with live pose estimation across multiple dictionary families simultaneously.

**Important**:
*   Ensure `calibration.npz` exists (from Step 2).
*   Pass `--marker-size` if your marker is not **100mm**.

```bash
# Scan all supported dictionaries
python Code/Callibration/aruco_detect.py

# Specific families only
python Code/Callibration/aruco_detect.py --dicts 4x4_1000 cv_apriltag_36h11

# Custom marker size
python Code/Callibration/aruco_detect.py --marker-size 150

# List all supported dictionary names
python Code/Callibration/aruco_detect.py --list-dicts
```

*   **Controls**:
    *   **Q**: Quit
    *   **V**: Verify distance (prompts for true Z distance, saves error to `aruco_results.csv`)
    *   **D**: Toggle legend between all dictionaries / detected only

**Supported families**: `4x4_1000`, `5x5_1000`, `6x6_1000`, `7x7_1000`, `aruco_original`, `cv_apriltag_16h5`, `cv_apriltag_25h9`, `cv_apriltag_36h10`, `cv_apriltag_36h11`

## Troubleshooting

*   **Calibration fails**: Ensure checkerboard corners are clearly visible in all images.
*   **Verification error high**: Check if `MARKER_SIZE_MM` in `3_aruco_test.py` matches your printed marker size exactly.
