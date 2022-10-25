;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::TimeSyncStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'TimeSyncStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::TIMESYNCSTAMPED")
  (make-package "FF_MSGS::TIMESYNCSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude TimeSyncStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::TimeSyncStamped
  :super ros::object
  :slots (_header _remote_processor _mlp_time _remote_time ))

(defmethod ff_msgs::TimeSyncStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:remote_processor __remote_processor) "")
    ((:mlp_time __mlp_time) (instance ros::time :init))
    ((:remote_time __remote_time) (instance ros::time :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _remote_processor (string __remote_processor))
   (setq _mlp_time __mlp_time)
   (setq _remote_time __remote_time)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:remote_processor
   (&optional __remote_processor)
   (if __remote_processor (setq _remote_processor __remote_processor)) _remote_processor)
  (:mlp_time
   (&optional __mlp_time)
   (if __mlp_time (setq _mlp_time __mlp_time)) _mlp_time)
  (:remote_time
   (&optional __remote_time)
   (if __remote_time (setq _remote_time __remote_time)) _remote_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _remote_processor
    4 (length _remote_processor)
    ;; time _mlp_time
    8
    ;; time _remote_time
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _remote_processor
       (write-long (length _remote_processor) s) (princ _remote_processor s)
     ;; time _mlp_time
       (write-long (send _mlp_time :sec) s) (write-long (send _mlp_time :nsec) s)
     ;; time _remote_time
       (write-long (send _remote_time :sec) s) (write-long (send _remote_time :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _remote_processor
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _remote_processor (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; time _mlp_time
     (send _mlp_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _mlp_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; time _remote_time
     (send _remote_time :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _remote_time :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::TimeSyncStamped :md5sum-) "37bae83a741b528e46eb78c2ad2d0190")
(setf (get ff_msgs::TimeSyncStamped :datatype-) "ff_msgs/TimeSyncStamped")
(setf (get ff_msgs::TimeSyncStamped :definition-)
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

# Processor that the heartbeat came from
string remote_processor

# Current time on the mlp
time mlp_time

# Time in the incoming heartbeat
time remote_time

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



(provide :ff_msgs/TimeSyncStamped "37bae83a741b528e46eb78c2ad2d0190")


