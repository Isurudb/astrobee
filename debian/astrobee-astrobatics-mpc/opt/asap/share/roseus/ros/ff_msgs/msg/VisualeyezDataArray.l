;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::VisualeyezDataArray)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'VisualeyezDataArray (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::VISUALEYEZDATAARRAY")
  (make-package "FF_MSGS::VISUALEYEZDATAARRAY"))

(in-package "ROS")
;;//! \htmlinclude VisualeyezDataArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::VisualeyezDataArray
  :super ros::object
  :slots (_header _measurements ))

(defmethod ff_msgs::VisualeyezDataArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:measurements __measurements) (let (r) (dotimes (i 0) (push (instance ff_msgs::VisualeyezData :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _measurements __measurements)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:measurements
   (&rest __measurements)
   (if (keywordp (car __measurements))
       (send* _measurements __measurements)
     (progn
       (if __measurements (setq _measurements (car __measurements)))
       _measurements)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/VisualeyezData[] _measurements
    (apply #'+ (send-all _measurements :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/VisualeyezData[] _measurements
     (write-long (length _measurements) s)
     (dolist (elem _measurements)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/VisualeyezData[] _measurements
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _measurements (let (r) (dotimes (i n) (push (instance ff_msgs::VisualeyezData :init) r)) r))
     (dolist (elem- _measurements)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::VisualeyezDataArray :md5sum-) "85657b6672b01ba00fca31f22b2a2220")
(setf (get ff_msgs::VisualeyezDataArray :datatype-) "ff_msgs/VisualeyezDataArray")
(setf (get ff_msgs::VisualeyezDataArray :definition-)
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
# Raw Visualeyez data array and timestamp.

Header header                           # Header with timestamp
ff_msgs/VisualeyezData[] measurements   # List of all measurements
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
MSG: ff_msgs/VisualeyezData
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
# Raw Visualeyez data.

uint8 tcmid                         # Transmission control module ID
uint8 ledid                         # Light emitting diode ID
geometry_msgs/Vector3 position      # Coordinate 
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :ff_msgs/VisualeyezDataArray "85657b6672b01ba00fca31f22b2a2220")


