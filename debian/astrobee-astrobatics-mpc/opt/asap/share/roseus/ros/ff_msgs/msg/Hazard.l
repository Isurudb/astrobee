;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Hazard)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Hazard (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::HAZARD")
  (make-package "FF_MSGS::HAZARD"))

(in-package "ROS")
;;//! \htmlinclude Hazard.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TYPE_UNKNOWN*" (find-package "FF_MSGS::HAZARD"))
(shadow '*TYPE_UNKNOWN* (find-package "FF_MSGS::HAZARD"))
(defconstant ff_msgs::Hazard::*TYPE_UNKNOWN* 0)
(intern "*TYPE_KEEP_IN*" (find-package "FF_MSGS::HAZARD"))
(shadow '*TYPE_KEEP_IN* (find-package "FF_MSGS::HAZARD"))
(defconstant ff_msgs::Hazard::*TYPE_KEEP_IN* 1)
(intern "*TYPE_KEEP_OUT*" (find-package "FF_MSGS::HAZARD"))
(shadow '*TYPE_KEEP_OUT* (find-package "FF_MSGS::HAZARD"))
(defconstant ff_msgs::Hazard::*TYPE_KEEP_OUT* 2)
(intern "*TYPE_OBSTACLE*" (find-package "FF_MSGS::HAZARD"))
(shadow '*TYPE_OBSTACLE* (find-package "FF_MSGS::HAZARD"))
(defconstant ff_msgs::Hazard::*TYPE_OBSTACLE* 3)
(defclass ff_msgs::Hazard
  :super ros::object
  :slots (_header _type _hazard ))

(defmethod ff_msgs::Hazard
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:type __type) 0)
    ((:hazard __hazard) (instance geometry_msgs::PointStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _type (round __type))
   (setq _hazard __hazard)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:hazard
   (&rest __hazard)
   (if (keywordp (car __hazard))
       (send* _hazard __hazard)
     (progn
       (if __hazard (setq _hazard (car __hazard)))
       _hazard)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _type
    1
    ;; geometry_msgs/PointStamped _hazard
    (send _hazard :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/PointStamped _hazard
       (send _hazard :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/PointStamped _hazard
     (send _hazard :deserialize buf ptr-) (incf ptr- (send _hazard :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::Hazard :md5sum-) "4afedfcfa47c26e5781409fe253f8e48")
(setf (get ff_msgs::Hazard :datatype-) "ff_msgs/Hazard")
(setf (get ff_msgs::Hazard :definition-)
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
# Message used to notify choreographer of hazards

# Header with timestamp
std_msgs/Header header

# Type of hazard
uint8 type
uint8 TYPE_UNKNOWN  = 0
uint8 TYPE_KEEP_IN  = 1
uint8 TYPE_KEEP_OUT = 2
uint8 TYPE_OBSTACLE = 3

# Spatio-tempral information about the hazard
geometry_msgs/PointStamped hazard
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
MSG: geometry_msgs/PointStamped
# This represents a Point with reference coordinate frame and timestamp
Header header
Point point

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :ff_msgs/Hazard "4afedfcfa47c26e5781409fe253f8e48")


