;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ArmState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ArmState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ARMSTATE")
  (make-package "FF_MSGS::ARMSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*INITIALIZING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*INITIALIZING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*INITIALIZING* 0)
(intern "*UNKNOWN*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*UNKNOWN* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*UNKNOWN* 1)
(intern "*STOWED*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*STOWED* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*STOWED* 2)
(intern "*DEPLOYED*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*DEPLOYED* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*DEPLOYED* 3)
(intern "*SETTING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*SETTING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*SETTING* 4)
(intern "*PANNING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*PANNING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*PANNING* 5)
(intern "*TILTING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*TILTING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*TILTING* 6)
(intern "*STOWING_SETTING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*STOWING_SETTING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*STOWING_SETTING* 7)
(intern "*STOWING_PANNING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*STOWING_PANNING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*STOWING_PANNING* 8)
(intern "*STOWING_TILTING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*STOWING_TILTING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*STOWING_TILTING* 9)
(intern "*DEPLOYING_PANNING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*DEPLOYING_PANNING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*DEPLOYING_PANNING* 10)
(intern "*DEPLOYING_TILTING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*DEPLOYING_TILTING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*DEPLOYING_TILTING* 11)
(intern "*CALIBRATING*" (find-package "FF_MSGS::ARMSTATE"))
(shadow '*CALIBRATING* (find-package "FF_MSGS::ARMSTATE"))
(defconstant ff_msgs::ArmState::*CALIBRATING* 12)
(defclass ff_msgs::ArmState
  :super ros::object
  :slots (_header _state _fsm_event _fsm_state ))

(defmethod ff_msgs::ArmState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:fsm_event __fsm_event) "")
    ((:fsm_state __fsm_state) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _fsm_event (string __fsm_event))
   (setq _fsm_state (string __fsm_state))
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
  (:fsm_event
   (&optional __fsm_event)
   (if __fsm_event (setq _fsm_event __fsm_event)) _fsm_event)
  (:fsm_state
   (&optional __fsm_state)
   (if __fsm_state (setq _fsm_state __fsm_state)) _fsm_state)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _state
    1
    ;; string _fsm_event
    4 (length _fsm_event)
    ;; string _fsm_state
    4 (length _fsm_state)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _state
       (write-byte _state s)
     ;; string _fsm_event
       (write-long (length _fsm_event) s) (princ _fsm_event s)
     ;; string _fsm_state
       (write-long (length _fsm_state) s) (princ _fsm_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _state 127) (setq _state (- _state 256)))
   ;; string _fsm_event
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_event (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fsm_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::ArmState :md5sum-) "4424d991014c0bf2f6a1521cbeb69659")
(setf (get ff_msgs::ArmState :datatype-) "ff_msgs/ArmState")
(setf (get ff_msgs::ArmState :definition-)
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
# The state of the arm behavior

# Header with timestamp
std_msgs/Header header

# Tee current state
int8 state                         # Current state
int8 INITIALIZING        = 0       # Waiting on child services, actions, etc.
int8 UNKNOWN             = 1       # Waiting on feedback from driver
int8 STOWED              = 2       # The arm is stowed
int8 DEPLOYED            = 3       # The arm is deployed
int8 SETTING             = 4       # The gripper is being set to a value
int8 PANNING             = 5       # We are panning as part of a move
int8 TILTING             = 6       # We are tilting as part of a move
int8 STOWING_SETTING     = 7       # We are closing the gripper for stowing
int8 STOWING_PANNING     = 8       # We are panning to zero for stowing
int8 STOWING_TILTING     = 9       # We are tilting to zero for stowing
int8 DEPLOYING_PANNING   = 10      # We are panning to zero for stowing
int8 DEPLOYING_TILTING   = 11      # We are tilting to zero for stowing
int8 CALIBRATING         = 12      # We are calibrating the gripper

# A human readble version of the (event) -> [state] transition
string fsm_event
string fsm_state

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



(provide :ff_msgs/ArmState "4424d991014c0bf2f6a1521cbeb69659")


