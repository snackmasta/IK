# DIS Optical Flow & RANSAC Motion Estimation Guide

A standalone, focused technical guide for implementing **DIS (Dense Inverse Search) Optical Flow** combined with **RANSAC 2D Partial Affine Motion Estimation** in Python/OpenCV for any robotics, computer vision, or camera motion tracking project.

---

## 1. Overview & Key Concepts

This pipeline estimates 2D rigid camera motion between two consecutive grayscale frames ($I_{t-1}$ and $I_t$) in two distinct phases:

1. **DIS (Dense Inverse Search) Optical Flow**: Computes a dense flow field $(u, v)$ for every pixel in the image using patch matching and variational refinement. DIS is extremely fast and robust for high-framerate real-time applications.
2. **RANSAC Partial Affine Fitting**: Grid-samples flow vectors across the image and uses RANSAC (Random Sample Consensus) to fit a partial affine matrix ($tx, ty, \theta, s$). This rejects noise, dynamic obstacles, and outliers to extract true 2D camera translation, rotation, and scaling.

```
┌───────────────────┐     ┌───────────────────┐
│ Previous Frame    │     │ Current Frame     │
│ (Grayscale)       │     │ (Grayscale)       │
└─────────┬─────────┘     └─────────┬─────────┘
          │                         │
          └────────────┬────────────┘
                       │
                       ▼
          [ cv2.DISOpticalFlow ] ──> Dense Flow Field (u, v)
                       │
                       ▼
          [ Grid Sampling (P_old -> P_new) ]
                       │
                       ▼
          [ Magnitude Filtering (< 80px) ]
                       │
                       ▼
          [ cv2.estimateAffinePartial2D (RANSAC) ]
                       │
                       ▼
     ┌───────────────────────────────────────────┐
     │ Extracted Motion Parameters:              │
     │  - Translation (tx, ty) in pixels         │
     │  - Rotation angle (theta) in radians      │
     │  - Scale factor (s)                       │
     │  - Inlier Points & Count                  │
     └───────────────────────────────────────────┘
```

---

## 2. Mathematical Formulation

### 2.1 Grid Sampling & Flow Vectors
For an image of height $H$ and width $W$, a regular grid of points $P_{\text{old}}$ is sampled with step size $S$ (e.g. 8 pixels):

$$P_{\text{old}}^{(i)} = (x_i, y_i) \quad \text{for } x_i \in \{S/2, 3S/2, \dots\}, y_i \in \{S/2, 3S/2, \dots\}$$

The displacement vector $(u_i, v_i)$ at point $(x_i, y_i)$ is extracted from the DIS flow matrix, forming candidate target points $P_{\text{new}}$:

$$P_{\text{new}}^{(i)} = (x_i + u_i, y_i + v_i)$$

### 2.2 Magnitude Sanity Filtering
Vectors with unnaturally large displacements (likely due to lighting flickering or frame corruption) are filtered prior to RANSAC:

$$\text{valid}_i = \left( \sqrt{u_i^2 + v_i^2} < \text{MAX\_FLOW\_STEP} \right)$$

### 2.3 Partial Affine Transformation Matrix
The partial affine transformation constrains the 2D transform to translation, uniform scaling, and rotation (4 degrees of freedom):

$$M = \begin{bmatrix} M_{0,0} & M_{0,1} & M_{0,2} \\ M_{1,0} & M_{1,1} & M_{1,2} \end{bmatrix} = \begin{bmatrix} s \cos\theta & -s \sin\theta & tx \\ s \sin\theta & s \cos\theta & ty \end{bmatrix}$$

RANSAC robustly estimates $M$ by iteratively testing subsets of point correspondences $(P_{\text{old}}, P_{\text{new}})$ and keeping the matrix that maximizes the number of inlier points satisfying:

$$\| P_{\text{new}}^{(i)} - M \cdot P_{\text{old}}^{(i)} \|_2 \le \text{ransacReprojThreshold}$$

### 2.4 Extracting Motion Parameters
Once $M$ is calculated, motion components are directly extracted:

- **Translation ($tx, ty$)**:
  $$tx = M_{0,2}, \quad ty = M_{1,2}$$

- **Scale Factor ($s$)**:
  $$s = \sqrt{M_{0,0}^2 + M_{1,0}^2}$$

- **Rotation Angle ($\theta$)**:
  $$\theta = \arctan2(M_{1,0}, M_{0,0}) \quad \text{(in radians)}$$

---

## 3. Minimal Python Implementation

Here is a self-contained, lightweight Python module (`dis_ransac_flow.py`) implementing DIS Optical Flow + RANSAC with no external dependencies other than `opencv-python` and `numpy`.

```python
import cv2
import numpy as np
import math

class DISRansacFlowEstimator:
    def __init__(self, step=8, max_flow_step=80.0, ransac_threshold=2.0, min_inliers=3):
        """
        Initializes the DIS Optical Flow + RANSAC Estimator.
        
        :param step: Grid spacing in pixels for sampling the dense flow field.
        :param max_flow_step: Maximum valid displacement magnitude in pixels.
        :param ransac_threshold: RANSAC reprojection error threshold in pixels.
        :param min_inliers: Minimum required inliers for a valid estimation.
        """
        self.step = step
        self.max_flow_step = max_flow_step
        self.ransac_threshold = ransac_threshold
        self.min_inliers = min_inliers
        
        # Initialize DIS Optical Flow with ULTRAFAST preset
        self.dis = cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_ULTRAFAST)

    def estimate_motion(self, prev_gray, curr_gray):
        """
        Estimates 2D translation (tx, ty), scale, rotation (theta), and inliers
        between two grayscale images.
        
        :param prev_gray: Previous frame (2D numpy array, uint8)
        :param curr_gray: Current frame (2D numpy array, uint8)
        :return: dict with keys ('tx', 'ty', 'scale', 'theta', 'inliers', 'pts_old', 'pts_new')
                 or None if estimation fails.
        """
        if prev_gray is None or curr_gray is None:
            return None

        h, w = prev_gray.shape

        # 1. Compute Dense Optical Flow field
        flow = self.dis.calc(prev_gray, curr_gray, None)

        # 2. Create grid of points
        ys, xs = np.mgrid[self.step//2:h:self.step, self.step//2:w:self.step].astype(np.float32)
        P_old = np.stack((xs, ys), axis=-1).reshape(-1, 2)

        # 3. Lookup flow vectors at grid coordinates
        u = flow[ys.astype(int), xs.astype(int), 0]
        v = flow[ys.astype(int), xs.astype(int), 1]
        flow_vectors = np.stack((u, v), axis=-1).reshape(-1, 2)
        P_new = P_old + flow_vectors

        # 4. Filter out extreme displacements
        magnitudes = np.linalg.norm(flow_vectors, axis=1)
        valid_mask = magnitudes < self.max_flow_step
        if np.count_nonzero(valid_mask) < self.min_inliers:
            return None

        P_old_filtered = P_old[valid_mask]
        P_new_filtered = P_new[valid_mask]

        # 5. Fit Partial Affine Transform using RANSAC
        affine_result = cv2.estimateAffinePartial2D(
            P_old_filtered,
            P_new_filtered,
            method=cv2.RANSAC,
            ransacReprojThreshold=self.ransac_threshold
        )

        if affine_result is None or affine_result[0] is None:
            return None

        M, inlier_mask = affine_result
        if inlier_mask is None:
            return None

        inlier_mask = inlier_mask.ravel().astype(bool)
        inlier_count = np.count_nonzero(inlier_mask)

        if inlier_count < self.min_inliers:
            return None

        # 6. Extract motion parameters from matrix M
        tx = float(M[0, 2])
        ty = float(M[1, 2])
        scale = float(math.sqrt(M[0, 0]**2 + M[1, 0]**2))
        theta = float(math.atan2(M[1, 0], M[0, 0]))  # Rotation in radians

        inlier_old = P_old_filtered[inlier_mask]
        inlier_new = P_new_filtered[inlier_mask]

        return {
            "tx": tx,                  # Horizontal pixel shift (+ is right)
            "ty": ty,                  # Vertical pixel shift (+ is down)
            "scale": scale,            # Zoom/scale factor (~1.0 if constant distance)
            "theta": theta,            # Rotation angle in radians
            "inlier_count": inlier_count,
            "pts_old": inlier_old,     # Matched inlier points in prev frame
            "pts_new": inlier_new      # Matched inlier points in curr frame
        }
```

---

## 4. Integration & Usage Example

Below is a complete script demonstrating how to use `DISRansacFlowEstimator` on a video stream or camera feed.

```python
import cv2
from dis_ransac_flow import DISRansacFlowEstimator

# Initialize estimator
estimator = DISRansacFlowEstimator(step=8, max_flow_step=80.0, ransac_threshold=2.0)

cap = cv2.VideoCapture(0)
prev_gray = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Optional: downscale image for extra speed (e.g. 50% scale)
    small_frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

    if prev_gray is not None:
        motion = estimator.estimate_motion(prev_gray, gray)

        if motion:
            tx = motion["tx"]
            ty = motion["ty"]
            scale = motion["scale"]
            theta_deg = motion["theta"] * (180.0 / 3.14159)
            inliers = motion["inlier_count"]

            print(f"Shift: tx={tx:+.2f}px, ty={ty:+.2f}px | Scale={scale:.3f} | Rot={theta_deg:+.2f}° | Inliers={inliers}")

            # Draw tracked inlier motion vectors on frame
            for p1, p2 in zip(motion["pts_old"], motion["pts_new"]):
                pt1 = (int(p1[0]), int(p1[1]))
                pt2 = (int(p2[0]), int(p2[1]))
                cv2.arrowedLine(small_frame, pt1, pt2, (0, 255, 0), 1, tipLength=0.3)

    prev_gray = gray

    cv2.imshow("DIS + RANSAC Flow", small_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## 5. Parameter Tuning Guide

| Parameter | Recommended Value | Description |
| :--- | :--- | :--- |
| `step` | `8` | Grid sampling step size in pixels. Lower values (e.g., `4`) increase point density but require slightly more RANSAC processing. Higher values (e.g., `12`) speed up processing. |
| `max_flow_step` | `80.0` | Rejects displacement vectors larger than 80px. Helps filter out frame drops or severe lighting flicker. |
| `ransacReprojThreshold` | `2.0` | Maximum pixel reprojection error allowed for a point to be classified as an inlier. |
| `min_inliers` | `3` | Minimum number of inliers required to accept the transformation (since 2D affine needs at least 2 points / 4 constraints). |
| DIS Preset | `DISOPTICAL_FLOW_PRESET_ULTRAFAST` | Fastest preset suitable for 60+ FPS real-time tracking. Can be changed to `PRESET_FAST` or `PRESET_MEDIUM` if higher accuracy on slow hardware is required. |
