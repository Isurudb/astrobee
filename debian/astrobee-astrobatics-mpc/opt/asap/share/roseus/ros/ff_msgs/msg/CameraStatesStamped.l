;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CameraStatesStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CameraStatesStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CAMERASTATESSTAMPED")
  (make-package "FF_MSGS::CAMERASTATESSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CameraStatesStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::CameraStatesStamped
  :super ros::object
  :slots (_header _states ))

(defmethod ff_msgs::CameraStatesStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:states __states) (let (r) (dotimes (i 0) (push (instance ff_msgs::CameraState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _states __states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/CameraState[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/CameraState[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/CameraState[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance ff_msgs::CameraState :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::CameraStatesStamped :md5sum-) "8196b4a79fabfdcae8f271a13d0b1973")
(setf (get ff_msgs::CameraStatesStamped :datatype-) "ff_msgs/CameraStatesStamped")
(setf (get ff_msgs::CameraStatesStamped :definition-)
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
# An array of CameraState messages.

# Header with timestamp
std_msgs/Header header

# Camera states
ff_msgs/CameraState[] states

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
MSG: ff_msgs/CameraState
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
# CameraState message, *MUST* be kept in sync with camera portion of
# rapid::ext::astrobee::TelemetryState

# nav_cam, dock_cam, etc.
string camera_name

# streaming to ground
bool streaming

# image width
uint16 stream_width
# image height
uint16 stream_height
# Rate in Hz
float32 stream_rate

# recording to disk
bool recording

# image width
uint16 record_width
# image height
uint16 record_height
# Rate in Hz
float32 record_rate

# only for sci cam
float32 bandwidth

")



(provide :ff_msgs/CameraStatesStamped "8196b4a79fabfdcae8f271a13d0b1973")


