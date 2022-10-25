;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::JointSampleStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'JointSampleStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::JOINTSAMPLESTAMPED")
  (make-package "FF_MSGS::JOINTSAMPLESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude JointSampleStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::JointSampleStamped
  :super ros::object
  :slots (_header _samples ))

(defmethod ff_msgs::JointSampleStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:samples __samples) (let (r) (dotimes (i 0) (push (instance ff_msgs::JointSample :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _samples __samples)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:samples
   (&rest __samples)
   (if (keywordp (car __samples))
       (send* _samples __samples)
     (progn
       (if __samples (setq _samples (car __samples)))
       _samples)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/JointSample[] _samples
    (apply #'+ (send-all _samples :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/JointSample[] _samples
     (write-long (length _samples) s)
     (dolist (elem _samples)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/JointSample[] _samples
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _samples (let (r) (dotimes (i n) (push (instance ff_msgs::JointSample :init) r)) r))
     (dolist (elem- _samples)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::JointSampleStamped :md5sum-) "ada8b7552d00a44b3992523c84bda644")
(setf (get ff_msgs::JointSampleStamped :datatype-) "ff_msgs/JointSampleStamped")
(setf (get ff_msgs::JointSampleStamped :definition-)
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
# An array of Joint sample messages.

# Header with timestamp
std_msgs/Header header

# Joint samples
ff_msgs/JointSample[] samples

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
MSG: ff_msgs/JointSample
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
# JointSample message, based off of rapid::JointSample

# Flag values for joint status. 
# Joint is enabled
uint8 JOINT_ENABLED    = 0      # Joint enabled
uint8 JOINT_DISABLED   = 1      # Joint disabled


# Angle position (in radians) of the joint
float32 angle_pos

# Angle velocity (in radians/sec) of the joint
float32 angle_vel

# Angle acceleration (in radians/sec^2) of the joint (not being used)
float32 angle_acc

# Current draw of joint motor
float32 current

# Torque sensed at the joint (not being used)
float32 torque

# Temperature of the joint (in Celsius)
float32 temperature

# Bit field representing the state of the joint
uint16 status

# Human-readable name
string name

")



(provide :ff_msgs/JointSampleStamped "ada8b7552d00a44b3992523c84bda644")


