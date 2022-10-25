;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::FaultState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'FaultState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FAULTSTATE")
  (make-package "FF_MSGS::FAULTSTATE"))

(in-package "ROS")
;;//! \htmlinclude FaultState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STARTING_UP*" (find-package "FF_MSGS::FAULTSTATE"))
(shadow '*STARTING_UP* (find-package "FF_MSGS::FAULTSTATE"))
(defconstant ff_msgs::FaultState::*STARTING_UP* 0)
(intern "*FUNCTIONAL*" (find-package "FF_MSGS::FAULTSTATE"))
(shadow '*FUNCTIONAL* (find-package "FF_MSGS::FAULTSTATE"))
(defconstant ff_msgs::FaultState::*FUNCTIONAL* 1)
(intern "*FAULT*" (find-package "FF_MSGS::FAULTSTATE"))
(shadow '*FAULT* (find-package "FF_MSGS::FAULTSTATE"))
(defconstant ff_msgs::FaultState::*FAULT* 2)
(intern "*BLOCKED*" (find-package "FF_MSGS::FAULTSTATE"))
(shadow '*BLOCKED* (find-package "FF_MSGS::FAULTSTATE"))
(defconstant ff_msgs::FaultState::*BLOCKED* 3)
(intern "*RELOADING_NODELETS*" (find-package "FF_MSGS::FAULTSTATE"))
(shadow '*RELOADING_NODELETS* (find-package "FF_MSGS::FAULTSTATE"))
(defconstant ff_msgs::FaultState::*RELOADING_NODELETS* 4)
(defclass ff_msgs::FaultState
  :super ros::object
  :slots (_header _state _hr_state _faults ))

(defmethod ff_msgs::FaultState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:hr_state __hr_state) "")
    ((:faults __faults) (let (r) (dotimes (i 0) (push (instance ff_msgs::Fault :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _hr_state (string __hr_state))
   (setq _faults __faults)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:hr_state
   (&optional __hr_state)
   (if __hr_state (setq _hr_state __hr_state)) _hr_state)
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
    ;; uint8 _state
    1
    ;; string _hr_state
    4 (length _hr_state)
    ;; ff_msgs/Fault[] _faults
    (apply #'+ (send-all _faults :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _state
       (write-byte _state s)
     ;; string _hr_state
       (write-long (length _hr_state) s) (princ _hr_state s)
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
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _hr_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hr_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
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

(setf (get ff_msgs::FaultState :md5sum-) "4ded9e5628846b2af7eff4a5b8d34c68")
(setf (get ff_msgs::FaultState :datatype-) "ff_msgs/FaultState")
(setf (get ff_msgs::FaultState :definition-)
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
# Fault state message used to alert the ground of the current faults. It is also
# used to express to the executive that a fault has occurred that indirectly
# affects the motion of the robot.

std_msgs/Header header

# Not sent to the ground, only used by the executive to determine what commands
# to accept.
uint8 state
# System starting up
uint8 STARTING_UP           = 0
# No faults are occurring in system
uint8 FUNCTIONAL            = 1
# Faults are occurring in the system which may or may not leave the robot
# functional
uint8 FAULT                 = 2
# A fault has occurred that indirectly affects the motion of the robot
uint8 BLOCKED               = 3
# Recovering from nodes dying on startup
uint8 RELOADING_NODELETS    = 4

# A human readable version of the state - only really used for when nodes die on
# startup and need to be restarted.
string hr_state

# Faults occurring in the astrobee system, can only send 32 faults down
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



(provide :ff_msgs/FaultState "4ded9e5628846b2af7eff4a5b8d34c68")


