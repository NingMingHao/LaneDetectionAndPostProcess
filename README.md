### Codes for Lane Detection and Post Process

* The Jetson folder contains the CNN part from [Ultra-Fast-Lane-Detection](https://github.com/cfzd/Ultra-Fast-Lane-Detection), you can download the trained model from [the author's link](https://drive.google.com/file/d/1WCYyur5ZaWczH15ecmeDowrW30xcLrCn/view) into this folder
* The lane_detection_msgs contains the msgs used
* The lane_detection_Nuc contains the post processing codes to estimate lane function



### Compile the rosmsgs

### How to Run

1. Start a roscore
2. Run the  [inferenceCNN.py](Jetson/inferenceCNN.py) 
3. Run the  [postProcessLaneDetection.py](lane_detection_Nuc/postProcessLaneDetection.py) to publish lane detection result in rslidar_front frame
3. Or Run the [postProcessLaneDetectionApplanix.py](lane_detection_Nuc/postProcessLaneDetectionApplanix.py)  to publish lane detection result in applanix frame
4. Rviz, use the  [LaneDetection.rviz](lane_detection_Nuc/LaneDetection.rviz) 



### Output

1. topic name `/Lane_Detection_Result`
2. topic message content

```python
float64[] CenterLaneX # Yellow Marking  Direction --> Front
float64[] CenterLaneY # Yellow Marking  Direction --> Left
float64[] CenterLineX # 2 meter from centerlane  Direction --> Front
float64[] CenterLineY # 2 meter from centerlane  Direction --> Left
float64[] RightCurbX  # Right Curb  Direction --> Front
float64[] RightCurbY  # Right Curb  Direction --> Left
float64 PolyFitC2     # C2 of the  f(y)=C2x^2+C1x+C0
float64 PolyFitC1     # C1 of the  f(y)=C2x^2+C1x+C0
float64 PolyFitC0     # C0 of the  f(y)=C2x^2+C1x+C0
float64 LateralDis    # C0 of the  f(y)=C2x^2+C1x+C0
float64 Confidence
```

