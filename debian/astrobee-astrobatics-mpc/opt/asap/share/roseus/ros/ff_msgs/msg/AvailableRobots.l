;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::AvailableRobots)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'AvailableRobots (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::AVAILABLEROBOTS")
  (make-package "FF_MSGS::AVAILABLEROBOTS"))

(in-package "ROS")
;;//! \htmlinclude AvailableRobots.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*BUMBLE*" (find-package "FF_MSGS::AVAILABLEROBOTS"))
(shadow '*BUMBLE* (find-package "FF_MSGS::AVAILABLEROBOTS"))
(defconstant ff_msgs::AvailableRobots::*BUMBLE* 1)
(intern "*HONEY*" (find-package "FF_MSGS::AVAILABLEROBOTS"))
(shadow '*HONEY* (find-package "FF_MSGS::AVAILABLEROBOTS"))
(defconstant ff_msgs::AvailableRobots::*HONEY* 2)
(intern "*QUEEN*" (find-package "FF_MSGS::AVAILABLEROBOTS"))
(shadow '*QUEEN* (find-package "FF_MSGS::AVAILABLEROBOTS"))
(defconstant ff_msgs::AvailableRobots::*QUEEN* 4)
(defclass ff_msgs::AvailableRobots
  :super ros::object
  :slots (_header _available_robots ))

(defmethod ff_msgs::AvailableRobots
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:available_robots __available_robots) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _available_robots __available_robots)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:available_robots
   (&optional __available_robots)
   (if __available_robots (setq _available_robots __available_robots)) _available_robots)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8[] _available_robots
    (* 1    (length _available_robots)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8[] _available_robots
     (write-long (length _available_robots) s)
     (princ _available_robots s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8[] _available_robots
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _available_robots (make-array n :element-type :char))
     (replace _available_robots buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::AvailableRobots :md5sum-) "530b27dcdeb1f15db5e85c3997fe3149")
(setf (get ff_msgs::AvailableRobots :datatype-) "ff_msgs/AvailableRobots")
(setf (get ff_msgs::AvailableRobots :definition-)
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
# Message containing the Astrobee's connected to the current Astrobee 

std_msgs/Header header # header with time stamp

uint8 BUMBLE = 1
uint8 HONEY  = 2
uint8 QUEEN  = 4

# Robots connected to the current robot through DDS
uint8[] available_robots

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



(provide :ff_msgs/AvailableRobots "530b27dcdeb1f15db5e85c3997fe3149")


