## First build the LaneDetectionMsg in the folder lane_detection_msgs


## How to run the code
python FakeLaneDetection.py

## How to subscribe the detection result
topic name: "/Lane_Detection_Result"

float64[] CenterLaneX # Yellow Marking  Direction --> Front
float64[] CenterLaneY # Yellow Marking  Direction --> Left
float64[] CenterLineX # 2 meter from centerlane  Direction --> Front
float64[] CenterLineY # 2 meter from centerlane  Direction --> Left
float64[] RightCurbX  # Right Curb  Direction --> Front
float64[] RightCurbY  # Right Curb  Direction --> Left





