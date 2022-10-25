;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::AccessControlStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'AccessControlStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ACCESSCONTROLSTATESTAMPED")
  (make-package "FF_MSGS::ACCESSCONTROLSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude AccessControlStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::AccessControlStateStamped
  :super ros::object
  :slots (_header _controller _cookie ))

(defmethod ff_msgs::AccessControlStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:controller __controller) "")
    ((:cookie __cookie) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _controller (string __controller))
   (setq _cookie (string __cookie))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:controller
   (&optional __controller)
   (if __controller (setq _controller __controller)) _controller)
  (:cookie
   (&optional __cookie)
   (if __cookie (setq _cookie __cookie)) _cookie)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _controller
    4 (length _controller)
    ;; string _cookie
    4 (length _cookie)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _controller
       (write-long (length _controller) s) (princ _controller s)
     ;; string _cookie
       (write-long (length _cookie) s) (princ _cookie s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _controller
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _controller (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cookie
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cookie (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::AccessControlStateStamped :md5sum-) "7f6db2b63dd70f4a02a19c88e8bcdc59")
(setf (get ff_msgs::AccessControlStateStamped :datatype-) "ff_msgs/AccessControlStateStamped")
(setf (get ff_msgs::AccessControlStateStamped :definition-)
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
# State of the access control node. Loosely based off of AccessControlState.idl 
# from RAPID.

# Header with timestamp
std_msgs/Header header

# Name of operator in control of the robot
string controller

# String that the access control node generates upon receiving a request control
# command. Cookie will be blank after a successful grab control command.
string cookie

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



(provide :ff_msgs/AccessControlStateStamped "7f6db2b63dd70f4a02a19c88e8bcdc59")


