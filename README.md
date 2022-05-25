### Codes for Lane Detection and Post Process

* The Jetson folder contains the CNN part from [Ultra-Fast-Lane-Detection](https://github.com/cfzd/Ultra-Fast-Lane-Detection), you can download the trained model from [the author's link](https://drive.google.com/file/d/1WCYyur5ZaWczH15ecmeDowrW30xcLrCn/view) into this folder
* The lane_detection_msgs contains the msgs used
* The lane_detection_Nuc contains the post processing codes to estimate lane function



### Compile the rosmsgs

### How to Run

1. Start a roscore
2. Run the  [inferenceCNN.py](Jetson/inferenceCNN.py) 
3. Run the  [postProcessLaneDetection.py](lane_detection_Nuc/postProcessLaneDetection.py) 
4. Rviz, use the  [LaneDetection.rviz](lane_detection_Nuc/LaneDetection.rviz) 