;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DepthOdometry)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DepthOdometry (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DEPTHODOMETRY")
  (make-package "FF_MSGS::DEPTHODOMETRY"))

(in-package "ROS")
;;//! \htmlinclude DepthOdometry.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::DepthOdometry
  :super ros::object
  :slots (_header _odometry _correspondences _valid_image_points _valid_points_3d _runtime ))

(defmethod ff_msgs::DepthOdometry
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:odometry __odometry) (instance ff_msgs::Odometry :init))
    ((:correspondences __correspondences) (let (r) (dotimes (i 0) (push (instance ff_msgs::DepthCorrespondence :init) r)) r))
    ((:valid_image_points __valid_image_points) nil)
    ((:valid_points_3d __valid_points_3d) nil)
    ((:runtime __runtime) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _odometry __odometry)
   (setq _correspondences __correspondences)
   (setq _valid_image_points __valid_image_points)
   (setq _valid_points_3d __valid_points_3d)
   (setq _runtime (float __runtime))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:odometry
   (&rest __odometry)
   (if (keywordp (car __odometry))
       (send* _odometry __odometry)
     (progn
       (if __odometry (setq _odometry (car __odometry)))
       _odometry)))
  (:correspondences
   (&rest __correspondences)
   (if (keywordp (car __correspondences))
       (send* _correspondences __correspondences)
     (progn
       (if __correspondences (setq _correspondences (car __correspondences)))
       _correspondences)))
  (:valid_image_points
   (&optional __valid_image_points)
   (if __valid_image_points (setq _valid_image_points __valid_image_points)) _valid_image_points)
  (:valid_points_3d
   (&optional __valid_points_3d)
   (if __valid_points_3d (setq _valid_points_3d __valid_points_3d)) _valid_points_3d)
  (:runtime
   (&optional __runtime)
   (if __runtime (setq _runtime __runtime)) _runtime)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/Odometry _odometry
    (send _odometry :serialization-length)
    ;; ff_msgs/DepthCorrespondence[] _correspondences
    (apply #'+ (send-all _correspondences :serialization-length)) 4
    ;; bool _valid_image_points
    1
    ;; bool _valid_points_3d
    1
    ;; float32 _runtime
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/Odometry _odometry
       (send _odometry :serialize s)
     ;; ff_msgs/DepthCorrespondence[] _correspondences
     (write-long (length _correspondences) s)
     (dolist (elem _correspondences)
       (send elem :serialize s)
       )
     ;; bool _valid_image_points
       (if _valid_image_points (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid_points_3d
       (if _valid_points_3d (write-byte -1 s) (write-byte 0 s))
     ;; float32 _runtime
       (sys::poke _runtime (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/Odometry _odometry
     (send _odometry :deserialize buf ptr-) (incf ptr- (send _odometry :serialization-length))
   ;; ff_msgs/DepthCorrespondence[] _correspondences
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _correspondences (let (r) (dotimes (i n) (push (instance ff_msgs::DepthCorrespondence :init) r)) r))
     (dolist (elem- _correspondences)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bool _valid_image_points
     (setq _valid_image_points (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid_points_3d
     (setq _valid_points_3d (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _runtime
     (setq _runtime (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::DepthOdometry :md5sum-) "d00049c091a5ccf31e3eef01d010e9fa")
(setf (get ff_msgs::DepthOdometry :datatype-) "ff_msgs/DepthOdometry")
(setf (get ff_msgs::DepthOdometry :definition-)
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

Header header
ff_msgs/Odometry odometry
ff_msgs/DepthCorrespondence[] correspondences
bool valid_image_points
bool valid_points_3d
float32 runtime

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
MSG: ff_msgs/Odometry
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

time source_time 
time target_time
geometry_msgs/PoseWithCovariance sensor_F_source_T_target
geometry_msgs/PoseWithCovariance body_F_source_T_target

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

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
MSG: ff_msgs/DepthCorrespondence
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
ImagePoint source_image_point
ImagePoint target_image_point
geometry_msgs/Point source_3d_point
geometry_msgs/Point target_3d_point

================================================================================
MSG: ff_msgs/ImagePoint
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

float32 x
float32 y

")



(provide :ff_msgs/DepthOdometry "d00049c091a5ccf31e3eef01d010e9fa")


