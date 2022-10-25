;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::TimeDiffStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'TimeDiffStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::TIMEDIFFSTAMPED")
  (make-package "FF_MSGS::TIMEDIFFSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude TimeDiffStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::TimeDiffStamped
  :super ros::object
  :slots (_header _time_diff_sec ))

(defmethod ff_msgs::TimeDiffStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time_diff_sec __time_diff_sec) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time_diff_sec (float __time_diff_sec))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time_diff_sec
   (&optional __time_diff_sec)
   (if __time_diff_sec (setq _time_diff_sec __time_diff_sec)) _time_diff_sec)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _time_diff_sec
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _time_diff_sec
       (sys::poke _time_diff_sec (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _time_diff_sec
     (setq _time_diff_sec (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::TimeDiffStamped :md5sum-) "923869d9e60d4fadfc7104bd76be64ed")
(setf (get ff_msgs::TimeDiffStamped :datatype-) "ff_msgs/TimeDiffStamped")
(setf (get ff_msgs::TimeDiffStamped :definition-)
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
# Message to send time difference between the mlp and llp

# Header with timestamp
std_msgs/Header header

# Time diff in seconds between llp and mlp
float32 time_diff_sec

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

")



(provide :ff_msgs/TimeDiffStamped "923869d9e60d4fadfc7104bd76be64ed")


