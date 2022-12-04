;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Heartbeat)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Heartbeat (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::HEARTBEAT")
  (make-package "FF_MSGS::HEARTBEAT"))

(in-package "ROS")
;;//! \htmlinclude Heartbeat.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::Heartbeat
  :super ros::object
  :slots (_header _nodelet_manager _node _faults ))

(defmethod ff_msgs::Heartbeat
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:nodelet_manager __nodelet_manager) "")
    ((:node __node) "")
    ((:faults __faults) (let (r) (dotimes (i 0) (push (instance ff_msgs::Fault :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _nodelet_manager (string __nodelet_manager))
   (setq _node (string __node))
   (setq _faults __faults)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:nodelet_manager
   (&optional __nodelet_manager)
   (if __nodelet_manager (setq _nodelet_manager __nodelet_manager)) _nodelet_manager)
  (:node
   (&optional __node)
   (if __node (setq _node __node)) _node)
  (:faults
   (&rest __faults)
   (if (keywordp (car __faults))
       (send* _faults __faults)
     (progn
       (if __faults (setq _faults (car __faults)))
       _faults)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _nodelet_manager
    4 (length _nodelet_manager)
    ;; string _node
    4 (length _node)
    ;; ff_msgs/Fault[] _faults
    (apply #'+ (send-all _faults :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _nodelet_manager
       (write-long (length _nodelet_manager) s) (princ _nodelet_manager s)
     ;; string _node
       (write-long (length _node) s) (princ _node s)
     ;; ff_msgs/Fault[] _faults
     (write-long (length _faults) s)
     (dolist (elem _faults)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _nodelet_manager
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _nodelet_manager (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _node
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _node (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/Fault[] _faults
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faults (let (r) (dotimes (i n) (push (instance ff_msgs::Fault :init) r)) r))
     (dolist (elem- _faults)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::Heartbeat :md5sum-) "00da4d5a529de9d778f78787c87e306c")
(setf (get ff_msgs::Heartbeat :datatype-) "ff_msgs/Heartbeat")
(setf (get ff_msgs::Heartbeat :definition-)
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
# Fault State Message which doubles as a heartbeat

# Header with timestamp
std_msgs/Header header

# Name of the nodelet manager the node is running in
string nodelet_manager

# Name of node publishing the heartbeat
string node

# Faults that are currently occurring in the node
ff_msgs/Fault[] faults

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
MSG: ff_msgs/Fault
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
# Fault message is used to provide all the information about an occurring fault

time time_of_fault        # Time when fault occurred

uint32 id                 # id specifying fault

string msg                # string specifying why the fault occurred

ff_msgs/FaultData[] data  # Data used for fault analysis

================================================================================
MSG: ff_msgs/FaultData
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
# Fault data messsage contains information of why the fault occurred

uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float
uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int
uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string

string key  # Specifies what the data in the msg is, can only be 32 chars long

uint8 data_type   # Specifies the type of data in the message

float32 f   # Value used for fault analysis, data_type must be 0
int32 i     # Value used for fault analysis, data_type must be 1
string s    # String used for fault analysis, data_type must be 2

")



(provide :ff_msgs/Heartbeat "00da4d5a529de9d778f78787c87e306c")

