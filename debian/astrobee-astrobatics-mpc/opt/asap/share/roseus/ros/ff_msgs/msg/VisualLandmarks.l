;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::VisualLandmarks)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'VisualLandmarks (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::VISUALLANDMARKS")
  (make-package "FF_MSGS::VISUALLANDMARKS"))

(in-package "ROS")
;;//! \htmlinclude VisualLandmarks.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::VisualLandmarks
  :super ros::object
  :slots (_header _camera_id _pose _landmarks ))

(defmethod ff_msgs::VisualLandmarks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera_id __camera_id) 0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:landmarks __landmarks) (let (r) (dotimes (i 0) (push (instance ff_msgs::VisualLandmark :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera_id (round __camera_id))
   (setq _pose __pose)
   (setq _landmarks __landmarks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:camera_id
   (&optional __camera_id)
   (if __camera_id (setq _camera_id __camera_id)) _camera_id)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:landmarks
   (&rest __landmarks)
   (if (keywordp (car __landmarks))
       (send* _landmarks __landmarks)
     (progn
       (if __landmarks (setq _landmarks (car __landmarks)))
       _landmarks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _camera_id
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; ff_msgs/VisualLandmark[] _landmarks
    (apply #'+ (send-all _landmarks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _camera_id
       (write-long _camera_id s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; ff_msgs/VisualLandmark[] _landmarks
     (write-long (length _landmarks) s)
     (dolist (elem _landmarks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _camera_id
     (setq _camera_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; ff_msgs/VisualLandmark[] _landmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks (let (r) (dotimes (i n) (push (instance ff_msgs::VisualLandmark :init) r)) r))
     (dolist (elem- _landmarks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::VisualLandmarks :md5sum-) "bc9fc09f69d7758225ed41ee26f6375e")
(setf (get ff_msgs::VisualLandmarks :datatype-) "ff_msgs/VisualLandmarks")
(setf (get ff_msgs::VisualLandmarks :definition-)
      "# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# The observation from a camera image, of image coordinates
# and associated 3D coordinates. Used for sparse mapping and AR tags.

Header header # header with timestamp
uint32 camera_id # image ID, associated with registration pulse
geometry_msgs/Pose pose # estimated camera pose from features
ff_msgs/VisualLandmark[] landmarks # list of all landmarks

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: ff_msgs/VisualLandmark
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# A single observation of a feature, with an image coordinate
# and known 3D coordinate

float32 x # known 3D position x
float32 y # known 3D position y
float32 z # known 3D position z
float32 u # feature image coordinate x
float32 v # feature image coordinate y

")



(provide :ff_msgs/VisualLandmarks "bc9fc09f69d7758225ed41ee26f6375e")


