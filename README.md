### Codes for Lane Detection and Post Process

* The Jetson folder contains the CNN part from [Ultra-Fast-Lane-Detection](https://github.com/cfzd/Ultra-Fast-Lane-Detection), you can download the trained model from [the author's link](https://drive.google.com/file/d/1WCYyur5ZaWczH15ecmeDowrW30xcLrCn/view) into this folder
* The lane_detection_msgs contains the msgs used
* The lane_detection_Nuc contains the post processing codes to estimate lane function



### Compile the rosmsgs

### How to Run (not bus)

1. Start a roscore
2. Run the  [inferenceCNN.py](Jetson/inferenceCNN.py) 
3. Run the  [postProcessLaneDetection.py](lane_detection_Nuc/postProcessLaneDetection.py) to publish lane detection result in rslidar_front frame
3. Or Run the [postProcessLaneDetectionApplanix.py](lane_detection_Nuc/postProcessLaneDetectionApplanix.py)  to publish lane detection result in applanix frame
4. Rviz, use the  [LaneDetection.rviz](lane_detection_Nuc/LaneDetection.rviz) 

### How to Run (bus)

There are two cases when running the lane detection and post process modules: a) real-test and b) using recorded rosbag. 

For case b), the recorded rosbag contains only the compressed image; for case a), it will be better to use the raw image directly as it is faster to load.

So base on the cases, we need to modify the parameter `is_compressed_image` in `Jetson3` `src/CameraOnlyLaneDetectionBus/inferenceCNN.py`, line 126. Change  `is_compressed_image=True` when using rosbag, change  `is_compressed_image=False` when real-test.

To run the code:

1. Start a roscore on Nuc
1. Connect the Jetson3 to Nuc. Run `setup_ros` in Jetson3 terminal
2. Run the inferenceCNN.py on Jetson3. `python3 src/CameraOnlyLaneDetectionBus/inferenceCNN.py`
3. Run the postprocess code on Nuc. `python3 -W ignore src/lane_detection_Nuc_yellow_marker_only/postProcessLaneDetectionApplanix.py`
4. Rviz, and load the rviz configure file in the `lane_detection_Nuc_yellow_marker_only` folder.



### Output

1. topic name `/Lane_Detection_Result`
2. topic message content

```python
float64[] CenterLaneX    # Yellow Marking  Direction --> Front
float64[] CenterLaneY    # Yellow Marking  Direction --> Left
float64[] CenterLaneXRaw # Raw detection result of Yellow Marking  Direction --> Front
float64[] CenterLaneYRaw # Raw detection result of Yellow Marking  Direction --> Left
float64[] CenterLineX    # 2 meter from centerlane  Direction --> Front
float64[] CenterLineY    # 2 meter from centerlane  Direction --> Left
float64[] RightCurbX     # Right Curb  Direction --> Front
float64[] RightCurbY     # Right Curb  Direction --> Left
float64 PolyFitC2        # C2 of the  f(y)=C2x^2+C1x+C0
float64 PolyFitC1        # C1 of the  f(y)=C2x^2+C1x+C0
float64 PolyFitC0        # C0 of the  f(y)=C2x^2+C1x+C0
float64 LateralDis       # C0 of the  f(y)=C2x^2+C1x+C0
float64 Confidence
```

