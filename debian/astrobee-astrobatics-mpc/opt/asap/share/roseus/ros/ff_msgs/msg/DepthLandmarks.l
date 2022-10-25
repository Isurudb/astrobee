;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DepthLandmarks)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DepthLandmarks (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DEPTHLANDMARKS")
  (make-package "FF_MSGS::DEPTHLANDMARKS"))

(in-package "ROS")
;;//! \htmlinclude DepthLandmarks.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::DepthLandmarks
  :super ros::object
  :slots (_header _camera_id _end_seen _update_global_pose _sensor_T_handrail _sensor_t_line_points _sensor_t_line_endpoints _sensor_t_plane_points _landmarks ))

(defmethod ff_msgs::DepthLandmarks
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera_id __camera_id) 0)
    ((:end_seen __end_seen) 0)
    ((:update_global_pose __update_global_pose) 0)
    ((:sensor_T_handrail __sensor_T_handrail) (instance geometry_msgs::Pose :init))
    ((:sensor_t_line_points __sensor_t_line_points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point32 :init) r)) r))
    ((:sensor_t_line_endpoints __sensor_t_line_endpoints) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:sensor_t_plane_points __sensor_t_plane_points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point32 :init) r)) r))
    ((:landmarks __landmarks) (let (r) (dotimes (i 0) (push (instance ff_msgs::DepthLandmark :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera_id (round __camera_id))
   (setq _end_seen (round __end_seen))
   (setq _update_global_pose (round __update_global_pose))
   (setq _sensor_T_handrail __sensor_T_handrail)
   (setq _sensor_t_line_points __sensor_t_line_points)
   (setq _sensor_t_line_endpoints __sensor_t_line_endpoints)
   (setq _sensor_t_plane_points __sensor_t_plane_points)
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
  (:end_seen
   (&optional __end_seen)
   (if __end_seen (setq _end_seen __end_seen)) _end_seen)
  (:update_global_pose
   (&optional __update_global_pose)
   (if __update_global_pose (setq _update_global_pose __update_global_pose)) _update_global_pose)
  (:sensor_T_handrail
   (&rest __sensor_T_handrail)
   (if (keywordp (car __sensor_T_handrail))
       (send* _sensor_T_handrail __sensor_T_handrail)
     (progn
       (if __sensor_T_handrail (setq _sensor_T_handrail (car __sensor_T_handrail)))
       _sensor_T_handrail)))
  (:sensor_t_line_points
   (&rest __sensor_t_line_points)
   (if (keywordp (car __sensor_t_line_points))
       (send* _sensor_t_line_points __sensor_t_line_points)
     (progn
       (if __sensor_t_line_points (setq _sensor_t_line_points (car __sensor_t_line_points)))
       _sensor_t_line_points)))
  (:sensor_t_line_endpoints
   (&rest __sensor_t_line_endpoints)
   (if (keywordp (car __sensor_t_line_endpoints))
       (send* _sensor_t_line_endpoints __sensor_t_line_endpoints)
     (progn
       (if __sensor_t_line_endpoints (setq _sensor_t_line_endpoints (car __sensor_t_line_endpoints)))
       _sensor_t_line_endpoints)))
  (:sensor_t_plane_points
   (&rest __sensor_t_plane_points)
   (if (keywordp (car __sensor_t_plane_points))
       (send* _sensor_t_plane_points __sensor_t_plane_points)
     (progn
       (if __sensor_t_plane_points (setq _sensor_t_plane_points (car __sensor_t_plane_points)))
       _sensor_t_plane_points)))
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
    ;; uint8 _end_seen
    1
    ;; uint8 _update_global_pose
    1
    ;; geometry_msgs/Pose _sensor_T_handrail
    (send _sensor_T_handrail :serialization-length)
    ;; geometry_msgs/Point32[] _sensor_t_line_points
    (apply #'+ (send-all _sensor_t_line_points :serialization-length)) 4
    ;; geometry_msgs/Point[] _sensor_t_line_endpoints
    (apply #'+ (send-all _sensor_t_line_endpoints :serialization-length)) 4
    ;; geometry_msgs/Point32[] _sensor_t_plane_points
    (apply #'+ (send-all _sensor_t_plane_points :serialization-length)) 4
    ;; ff_msgs/DepthLandmark[] _landmarks
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
     ;; uint8 _end_seen
       (write-byte _end_seen s)
     ;; uint8 _update_global_pose
       (write-byte _update_global_pose s)
     ;; geometry_msgs/Pose _sensor_T_handrail
       (send _sensor_T_handrail :serialize s)
     ;; geometry_msgs/Point32[] _sensor_t_line_points
     (write-long (length _sensor_t_line_points) s)
     (dolist (elem _sensor_t_line_points)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _sensor_t_line_endpoints
     (write-long (length _sensor_t_line_endpoints) s)
     (dolist (elem _sensor_t_line_endpoints)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point32[] _sensor_t_plane_points
     (write-long (length _sensor_t_plane_points) s)
     (dolist (elem _sensor_t_plane_points)
       (send elem :serialize s)
       )
     ;; ff_msgs/DepthLandmark[] _landmarks
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
   ;; uint8 _end_seen
     (setq _end_seen (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _update_global_pose
     (setq _update_global_pose (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Pose _sensor_T_handrail
     (send _sensor_T_handrail :deserialize buf ptr-) (incf ptr- (send _sensor_T_handrail :serialization-length))
   ;; geometry_msgs/Point32[] _sensor_t_line_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_t_line_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point32 :init) r)) r))
     (dolist (elem- _sensor_t_line_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _sensor_t_line_endpoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_t_line_endpoints (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _sensor_t_line_endpoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point32[] _sensor_t_plane_points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_t_plane_points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point32 :init) r)) r))
     (dolist (elem- _sensor_t_plane_points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; ff_msgs/DepthLandmark[] _landmarks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _landmarks (let (r) (dotimes (i n) (push (instance ff_msgs::DepthLandmark :init) r)) r))
     (dolist (elem- _landmarks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::DepthLandmarks :md5sum-) "7fc86a54f996c15d2798a19b023404dc")
(setf (get ff_msgs::DepthLandmarks :datatype-) "ff_msgs/DepthLandmarks")
(setf (get ff_msgs::DepthLandmarks :definition-)
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
# An observation of a handrail from a depth image.

Header header                                  # Image header, with time stamp
uint32 camera_id                               # Image ID, associated with registration
uint8 end_seen                                 # Whether the handrail endpoint was detected
uint8 update_global_pose                       # Whether to update the global pose
geometry_msgs/Pose sensor_T_handrail           # Handrail center in the sensor frame
geometry_msgs/Point32[] sensor_t_line_points   # Detected line points
geometry_msgs/Point[] sensor_t_line_endpoints  # Detected line endpoints
geometry_msgs/Point32[] sensor_t_plane_points  # Detected plane points
ff_msgs/DepthLandmark[] landmarks              # List of landmarks seen TODO(rsoussan): This should be removed

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
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z
================================================================================
MSG: ff_msgs/DepthLandmark
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
# A landmark seen from a depth landmark

float32 u     # First coordinate in the image plane
float32 v     # Second coordinate in the image plane
float32 w     # Depth

")



(provide :ff_msgs/DepthLandmarks "7fc86a54f996c15d2798a19b023404dc")


