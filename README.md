# racing_obstacle_detection

```
root@ubuntu:/userdata# ros2 topic list
/camera_info
/cmd_vel
/hbmem_img
/image
/image_raw
/parameter_events
/racing_obstacle_detection
/racing_track_center_detection
/rosout
```

```
root@ubuntu:/userdata# ros2 topic info /racing_obstacle_detection
Type: ai_msgs/msg/PerceptionTargets
Publisher count: 0
Subscription count: 1
```

```
root@ubuntu:/userdata# ros2 interface show ai_msgs/msg/PerceptionTargets
# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# 感知结果

# 消息头
std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

# 感知结果的处理帧率
# fps val is invalid if fps is less than 0
int16 fps

# 性能统计信息，比如记录每个模型推理的耗时
Perf[] perfs
        #
        #
        #
        string type
        builtin_interfaces/Time stamp_start
                int32 sec
                uint32 nanosec
        builtin_interfaces/Time stamp_end
                int32 sec
                uint32 nanosec
        float64 time_ms_duration

# 感知目标集合
Target[] targets
        #
        #
        #
        string type
        uint64 track_id
        Roi[] rois
                #
                #
                #
                string type
                sensor_msgs/RegionOfInterest rect
                        #
                        uint32 x_offset  #
                                         # (0 if the ROI includes the left edge of the image)
                        uint32 y_offset  #
                                         # (0 if the ROI includes the top edge of the image)
                        uint32 height    #
                        uint32 width     #
                        bool do_rectify
                float32 confidence
        Attribute[] attributes
                #
                #
                #
                string type
                float32 value
                float32 confidence
        Point[] points
                #
                #
                #
                string type
                geometry_msgs/Point32[] point
                        #
                        #
                        float32 x
                        float32 y
                        float32 z
                float32[] confidence
        Capture[] captures
                #
                #
                #
                std_msgs/Header header
                        builtin_interfaces/Time stamp
                                int32 sec
                                uint32 nanosec
                        string frame_id
                sensor_msgs/Image img
                        std_msgs/Header header #
                                builtin_interfaces/Time stamp
                                        int32 sec
                                        uint32 nanosec
                                string frame_id
                                                     # Header frame_id should be optical frame of camera
                                                     # origin of frame should be optical center of cameara
                                                     # +x should point to the right in the image
                                                     # +y should point down in the image
                                                     # +z should point into to plane of the image
                                                     # If the frame_id here and the frame_id of the CameraInfo
                                                     # message associated with the image conflict
                                                     # the behavior is undefined
                        uint32 height                #
                        uint32 width                 #
                        string encoding       #
                                              # taken from the list of strings in include/sensor_msgs/image_encodings.hpp
                        uint8 is_bigendian    #
                        uint32 step           #
                        uint8[] data          #
                float32[] features
                DBResult db_result
                        #
                        #
                        #
                        string db_type
                        string match_id
                        float32 distance
                        float32 similarity

# 消失目标集合
Target[] disappeared_targets
        #
        #
        #
        string type
        uint64 track_id
        Roi[] rois
                #
                #
                #
                string type
                sensor_msgs/RegionOfInterest rect
                        #
                        uint32 x_offset  #
                                         # (0 if the ROI includes the left edge of the image)
                        uint32 y_offset  #
                                         # (0 if the ROI includes the top edge of the image)
                        uint32 height    #
                        uint32 width     #
                        bool do_rectify
                float32 confidence
        Attribute[] attributes
                #
                #
                #
                string type
                float32 value
                float32 confidence
        Point[] points
                #
                #
                #
                string type
                geometry_msgs/Point32[] point
                        #
                        #
                        float32 x
                        float32 y
                        float32 z
                float32[] confidence
        Capture[] captures
                #
                #
                #
                std_msgs/Header header
                        builtin_interfaces/Time stamp
                                int32 sec
                                uint32 nanosec
                        string frame_id
                sensor_msgs/Image img
                        std_msgs/Header header #
                                builtin_interfaces/Time stamp
                                        int32 sec
                                        uint32 nanosec
                                string frame_id
                                                     # Header frame_id should be optical frame of camera
                                                     # origin of frame should be optical center of cameara
                                                     # +x should point to the right in the image
                                                     # +y should point down in the image
                                                     # +z should point into to plane of the image
                                                     # If the frame_id here and the frame_id of the CameraInfo
                                                     # message associated with the image conflict
                                                     # the behavior is undefined
                        uint32 height                #
                        uint32 width                 #
                        string encoding       #
                                              # taken from the list of strings in include/sensor_msgs/image_encodings.hpp
                        uint8 is_bigendian    #
                        uint32 step           #
                        uint8[] data          #
                float32[] features
                DBResult db_result
                        #
                        #
                        #
                        string db_type
                        string match_id
                        float32 distance
                        float32 similarity
```

```
root@ubuntu:/userdata# ros2 topic echo /racing_obstacle_detection
---
header:
  stamp:
    sec: 1751621021
    nanosec: 88866000
  frame_id: '12823'
fps: 26
perfs: []
targets: []
disappeared_targets: []
---
header:
  stamp:
    sec: 1751621021
    nanosec: 120834000
  frame_id: '12824'
fps: 26
perfs: []
targets: []
disappeared_targets: []
---
header:
  stamp:
    sec: 1751621021
    nanosec: 152842000
  frame_id: '12825'
fps: 26
perfs: []
targets: []
disappeared_targets: []
---
header:
  stamp:
    sec: 1751621021
    nanosec: 184868000
  frame_id: '12826'
fps: 26
perfs: []
targets: []
disappeared_targets: []
---
header:
  stamp:
    sec: 1751621021
    nanosec: 220864000
  frame_id: '12827'
fps: 26
perfs: []
targets: []
disappeared_targets: []
---
```

```
---
header:
  stamp:
    sec: 1751621818
    nanosec: 11624000
  frame_id: '9639'
fps: 30
perfs: []
targets:
- type: construction_cone
  track_id: 0
  rois:
  - type: ''
    rect:
      x_offset: 104
      y_offset: 42
      height: 12
      width: 13
      do_rectify: false
    confidence: 0.5756282210350037
  attributes: []
  points: []
  captures: []
- type: construction_cone
  track_id: 0
  rois:
  - type: ''
    rect:
      x_offset: 105
      y_offset: 37
      height: 25
      width: 12
      do_rectify: false
    confidence: 0.5515782833099365
  attributes: []
  points: []
  captures: []
- type: construction_cone
  track_id: 0
  rois:
  - type: ''
    rect:
      x_offset: 377
      y_offset: 119
      height: 161
      width: 134
      do_rectify: false
    confidence: 0.3465600609779358
  attributes: []
  points: []
  captures: []
disappeared_targets: []
---
```