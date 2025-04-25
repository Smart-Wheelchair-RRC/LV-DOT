# Perception Module README

## Overview

This ROS node performs 3D object detection, tracking, and classification (dynamic/static) using data from multiple sensors (depth camera, color camera, LiDAR) and potentially external object detectors (like YOLO). It fuses information from different sources, filters detections, tracks objects over time using a Kalman filter, and classifies them based on their motion characteristics. The results are published for visualization and downstream tasks.

## Key Components

*   **Parameter Initialization:** The `initParam()` method loads parameters from the ROS parameter server. This includes sensor topics, camera intrinsic parameters, DBSCAN clustering settings, Kalman filter configurations, and various thresholds used for filtering and classification.
*   **ROS Publishers/Subscribers:**
    *   `registerPub()`: Sets up ROS publishers for outputting processed data like annotated images, point clouds, detected 3D bounding boxes, object trajectories, and velocity markers.
    *   `registerCallback()`: Sets up ROS subscribers to receive input sensor data (depth images, odometry, color images, LiDAR point clouds) and external detection results (e.g., YOLO 2D bounding boxes).
*   **Sensor Callbacks:** Methods like `depthPoseCB()`, `depthOdomCB()`, `colorImgCB()`, `yoloDetectionCB()`, and `lidarCloudCB()` handle incoming data from their respective sensors. They perform necessary conversions and store the data for the main processing loop.
*   **Detection Pipeline:**
    *   `dbscanDetect()`: Processes depth images. It projects depth points into 3D, filters them based on configured parameters, and applies DBSCAN clustering to identify potential objects and generate 3D bounding boxes.
    *   `uvDetect()`: (If applicable) Processes depth images using a UV detector to find visual bounding boxes, which are then transformed into the world coordinate frame.
    *   `lidarDetect()`: Processes LiDAR point clouds. It performs clustering (e.g., DBSCAN) on the LiDAR points to detect objects and extract their 3D bounding boxes.
*   **Fusion and Filtering:** The `filterLVBBoxes()` method integrates bounding boxes generated from different detection methods (e.g., visual and LiDAR). It filters out unlikely detections (outliers) and associates 2D detections (like those from YOLO) with the 3D boxes to enhance classification accuracy.
*   **Tracking and Classification:**
    *   `boxAssociation()` (and helper functions): Associates newly detected bounding boxes with existing tracks from previous frames based on metrics like Intersection over Union (IoU).
    *   `kalmanFilterAndUpdateHist()`: Maintains a Kalman filter for each tracked object. It predicts the object's state (position, velocity, potentially acceleration) and updates it based on the new measurements (associated bounding box). It also manages the history of bounding boxes and point clouds for each track.
    *   `classificationCB()`: Classifies tracked objects as either **dynamic** or **static** based on their motion history, velocity estimates, and consistency checks (detailed below).
*   **Visualization and Output:** Publishes various intermediate and final results to ROS topics for visualization in tools like RViz. This includes:
    *   Annotated images (e.g., color image with 2D/3D boxes projected).
    *   Processed point clouds.
    *   3D bounding boxes (color-coded: **Blue** for Dynamic, **Yellow** for Static).
    *   Object trajectories.
    *   Velocity markers.
*   **Utility Functions:** Contains helper functions for tasks like coordinate transformations (e.g., bounding box transformations between frames), point filtering, IoU calculation, and extracting dynamic points from clusters.

## Main Data Flow

1.  **Data Ingestion:** Sensor data (depth, color, LiDAR, odometry) and external detections (YOLO) are received via ROS callbacks.
2.  **Detection:** Independent detection modules (`dbscanDetect`, `uvDetect`, `lidarDetect`) process the relevant sensor data to generate candidate 3D bounding boxes.
3.  **Fusion & Filtering:** Detections from different sources are fused, and outliers are filtered (`filterLVBBoxes`). YOLO detections are associated if available.
4.  **Association:** New detections are associated with existing tracks (`boxAssociation`).
5.  **Tracking:** The state (position, velocity) of each tracked object is updated using a Kalman filter (`kalmanFilterAndUpdateHist`). History is maintained.
6.  **Classification:** Tracked objects are classified as dynamic or static (`classificationCB`).
7.  **Publication:** Results (bounding boxes, trajectories, velocities, visualization markers) are published to ROS topics.

## Dynamic vs. Static Obstacle Classification

The `classificationCB()` function determines if a tracked object is dynamic or static.

**Exceptions (No Classification Needed):**

1.  **YOLO Human Detection:** If YOLO identifies the object as a human (`is_human` flag is true), it is automatically classified as **dynamic**.
2.  **Insufficient History:** If the track history (`boxHist_`) is too short (less than `classifyHistoryThresh_` frames), classification is skipped for this cycle.
3.  **Forced Dynamic:** If an object has been consistently classified as dynamic for a sufficient number of recent frames (`dynamicHist_ >= forceDynamicThresh_`), it remains classified as **dynamic**.

**Classification Logic (for other objects):**

For objects not meeting the exception criteria, classification relies on point cloud voting and Kalman filter velocity:

1.  **Point Cloud Voting:**
    *   Iterate through points in the object's *current* point cloud cluster.
    *   For each point, find its nearest neighbor in the point cloud cluster from `curFrameGap` frames ago.
    *   Calculate the point's velocity vector (`Vcur`) based on this displacement over time (`dt_ * curFrameGap`).
    *   Compare the direction of `Vcur` with the bounding box's velocity vector (`Vbox`, calculated from bounding box displacement - see Velocity Calculation section).
    *   If the velocity direction is similar (dot product check `velSim > 0`) and the point's speed (`Vcur.norm()`) exceeds `dynaVelThresh_`, increment the `votes` counter.
    *   Calculate the `voteRatio = votes / numPoints`.
2.  **Kalman Filter Velocity:**
    *   Get the velocity norm (`Vkf.norm()`) estimated by the Kalman filter (`boxHist_[i][0].Vx`, `boxHist_[i][0].Vy`).
3.  **Thresholding:**
    *   If `voteRatio > dynaVoteThresh_` **AND** `Vkf.norm() > dynaVelThresh_`, the object is considered a **dynamic candidate** for the current frame.
4.  **Dynamic Consistency Check:**
    *   A counter (`dynamicHist_`) tracks consecutive frames where the object was a dynamic candidate.
    *   If `dynamicHist_ >= dynamicConsistThresh_`, the object is finally classified as **dynamic**.

**Static Classification:**

*   Any object that does not meet the criteria to be classified as dynamic (including exceptions and the voting/consistency checks) is classified as **static** by default.

**Summary Table:**

| Condition                                                                 | Classified as Dynamic?        |
| :------------------------------------------------------------------------ | :---------------------------- |
| YOLO detects human (`is_human == true`)                                   | Yes (Immediately)             |
| Forced dynamic (`dynamicHist_ >= forceDynamicThresh_`)                     | Yes (Immediately)             |
| `voteRatio > dynaVoteThresh_` AND `Vkf.norm() > dynaVelThresh_`            | Yes (After `dynamicConsistThresh_` frames) |
| Otherwise (including insufficient history or failed threshold/vote check) | No (Static)                   |

## Velocity Calculation

Velocity is estimated using two primary methods:

1.  **Bounding Box Displacement (Direct Calculation):**
    *   **Where:** Used within `classificationCB` primarily for the point cloud voting mechanism.
    *   **How:** Calculates velocity (`Vbox`) by dividing the change in the bounding box center position between the current frame (`boxHist_[i][0]`) and a past frame (`boxHist_[i][curFrameGap]`) by the elapsed time (`dt_ * curFrameGap`).
    *   `Vbox.x = (boxHist_[i][0].x - boxHist_[i][curFrameGap].x) / (dt_ * curFrameGap)`
    *   `Vbox.y = (boxHist_[i][0].y - boxHist_[i][curFrameGap].y) / (dt_ * curFrameGap)`
    *   `Vbox.z = (boxHist_[i][0].z - boxHist_[i][curFrameGap].z) / (dt_ * curFrameGap)`
2.  **Kalman Filter Estimation:**
    *   **Where:** Updated in `kalmanFilterAndUpdateHist()`, used for tracking, classification checks (`Vkf.norm()`), and published output.
    *   **How:** The Kalman filter maintains an internal state including velocity (`Vx`, `Vy`). It predicts the velocity and updates it based on position measurements (bounding box centers). The estimated velocity components are stored directly in the bounding box structure (`boxHist_[i][0].Vx`, `boxHist_[i][0].Vy`).

**Point Cloud Voting Velocity:**

*   While not a primary tracking velocity, individual point velocities (`Vcur`) are calculated within `classificationCB` during the voting process by comparing nearest neighbor points between frames. This is used *only* to check motion consistency against `Vbox` for dynamic classification.

**Summary Table:**

| Method                    | Where Used                    | What It Measures                     | Primary Use                               |
| :------------------------ | :---------------------------- | :----------------------------------- | :---------------------------------------- |
| Bounding Box Displacement | `classificationCB`            | Raw velocity from position change    | Point cloud voting direction reference    |
| Kalman Filter Estimate    | `kalmanFilterAndUpdateHist`   | Smoothed velocity estimate (`Vx`,`Vy`) | Tracking, Classification, Output Velocity |
| Point Cloud Voting        | `classificationCB`            | Individual point velocity            | Dynamic/Static voting mechanism           |

**In Practice:** The smoothed **Kalman filter velocity** (`Vx`, `Vy`) is the primary velocity estimate used for tracking the object's state and for publishing/visualization. The **bounding box displacement** velocity (`Vbox`) serves as a reference within the dynamic classification logic.

## Visualization

The node publishes data for visualization in RViz:

*   **Bounding Boxes:** Color-coded 3D boxes.
    *   **Blue:** Dynamic Objects
    *   **Yellow:** Static Objects
*   **Point Clouds:** Filtered and clustered point clouds.
*   **Trajectories:** Path history of tracked objects.
*   **Velocity Markers:** Arrows indicating the estimated velocity (from Kalman filter) of tracked objects.
*   **Images:** Raw or annotated color/depth images.
