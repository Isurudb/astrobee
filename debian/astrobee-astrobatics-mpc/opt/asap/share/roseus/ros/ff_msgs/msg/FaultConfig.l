;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::FaultConfig)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'FaultConfig (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FAULTCONFIG")
  (make-package "FF_MSGS::FAULTCONFIG"))

(in-package "ROS")
;;//! \htmlinclude FaultConfig.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::FaultConfig
  :super ros::object
  :slots (_header _subsystems _nodes _faults ))

(defmethod ff_msgs::FaultConfig
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:subsystems __subsystems) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:nodes __nodes) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:faults __faults) (let (r) (dotimes (i 0) (push (instance ff_msgs::FaultInfo :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _subsystems __subsystems)
   (setq _nodes __nodes)
   (setq _faults __faults)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:subsystems
   (&optional __subsystems)
   (if __subsystems (setq _subsystems __subsystems)) _subsystems)
  (:nodes
   (&optional __nodes)
   (if __nodes (setq _nodes __nodes)) _nodes)
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
    ;; string[] _subsystems
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _subsystems)) 4
    ;; string[] _nodes
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _nodes)) 4
    ;; ff_msgs/FaultInfo[] _faults
    (apply #'+ (send-all _faults :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _subsystems
     (write-long (length _subsystems) s)
     (dolist (elem _subsystems)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _nodes
     (write-long (length _nodes) s)
     (dolist (elem _nodes)
       (write-long (length elem) s) (princ elem s)
       )
     ;; ff_msgs/FaultInfo[] _faults
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
   ;; string[] _subsystems
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _subsystems (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _subsystems i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodes (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _nodes i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; ff_msgs/FaultInfo[] _faults
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _faults (let (r) (dotimes (i n) (push (instance ff_msgs::FaultInfo :init) r)) r))
     (dolist (elem- _faults)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::FaultConfig :md5sum-) "8fa101ee4a73bc04b7a0e50a88c2dff9")
(setf (get ff_msgs::FaultConfig :datatype-) "ff_msgs/FaultConfig")
(setf (get ff_msgs::FaultConfig :definition-)
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
# Fault config message used to inform DDS of all the faults in the system

# Header with timestamp
std_msgs/Header header

string[] subsystems         # A list of all the subsystem names in the system

string[] nodes              # A list of all the node names in the system
ff_msgs/FaultInfo[] faults  # A list of all of the faults in the system

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
MSG: ff_msgs/FaultInfo
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
# Fault info message is used in the fault config message to contain all the 
# information GDS needs to know about a fault

uint16 subsystem    # index into subsystem names array found in fault config msg

uint16 node         # index into node names array found in fault config msg

uint32 id           # id corresponding to the fault

bool warning        # whether the fault is a warning or not

string description  # A short description of why the fault occurred

")



(provide :ff_msgs/FaultConfig "8fa101ee4a73bc04b7a0e50a88c2dff9")


