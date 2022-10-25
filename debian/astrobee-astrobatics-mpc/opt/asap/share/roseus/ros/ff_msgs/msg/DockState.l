;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DockState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DockState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DOCKSTATE")
  (make-package "FF_MSGS::DOCKSTATE"))

(in-package "ROS")
;;//! \htmlinclude DockState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*RECOVERY_SWITCHING_TO_ML_LOC*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*RECOVERY_SWITCHING_TO_ML_LOC* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*RECOVERY_SWITCHING_TO_ML_LOC* 15)
(intern "*RECOVERY_MOVING_TO_APPROACH_POSE*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*RECOVERY_MOVING_TO_APPROACH_POSE* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*RECOVERY_MOVING_TO_APPROACH_POSE* 14)
(intern "*RECOVERY_WAITING_FOR_SPIN_DOWN*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*RECOVERY_WAITING_FOR_SPIN_DOWN* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*RECOVERY_WAITING_FOR_SPIN_DOWN* 13)
(intern "*RECOVERY_SWITCHING_TO_NO_LOC*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*RECOVERY_SWITCHING_TO_NO_LOC* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*RECOVERY_SWITCHING_TO_NO_LOC* 12)
(intern "*INITIALIZING*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*INITIALIZING* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*INITIALIZING* 11)
(intern "*UNKNOWN*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*UNKNOWN* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*UNKNOWN* 10)
(intern "*DOCKING_MAX_STATE*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_MAX_STATE* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_MAX_STATE* 7)
(intern "*DOCKING_SWITCHING_TO_ML_LOC*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_SWITCHING_TO_ML_LOC* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_SWITCHING_TO_ML_LOC* 7)
(intern "*DOCKING_MOVING_TO_APPROACH_POSE*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_MOVING_TO_APPROACH_POSE* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_MOVING_TO_APPROACH_POSE* 6)
(intern "*DOCKING_SWITCHING_TO_AR_LOC*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_SWITCHING_TO_AR_LOC* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_SWITCHING_TO_AR_LOC* 5)
(intern "*DOCKING_MOVING_TO_COMPLETE_POSE*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_MOVING_TO_COMPLETE_POSE* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_MOVING_TO_COMPLETE_POSE* 4)
(intern "*DOCKING_CHECKING_ATTACHED*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_CHECKING_ATTACHED* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_CHECKING_ATTACHED* 3)
(intern "*DOCKING_WAITING_FOR_SPIN_DOWN*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_WAITING_FOR_SPIN_DOWN* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_WAITING_FOR_SPIN_DOWN* 2)
(intern "*DOCKING_SWITCHING_TO_NO_LOC*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKING_SWITCHING_TO_NO_LOC* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKING_SWITCHING_TO_NO_LOC* 1)
(intern "*DOCKED*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*DOCKED* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*DOCKED* 0)
(intern "*UNDOCKING_SWITCHING_TO_ML_LOC*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*UNDOCKING_SWITCHING_TO_ML_LOC* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*UNDOCKING_SWITCHING_TO_ML_LOC* -1)
(intern "*UNDOCKING_WAITING_FOR_SPIN_UP*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*UNDOCKING_WAITING_FOR_SPIN_UP* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*UNDOCKING_WAITING_FOR_SPIN_UP* -2)
(intern "*UNDOCKING_MOVING_TO_APPROACH_POSE*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*UNDOCKING_MOVING_TO_APPROACH_POSE* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*UNDOCKING_MOVING_TO_APPROACH_POSE* -3)
(intern "*UNDOCKED*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*UNDOCKED* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*UNDOCKED* -4)
(intern "*UNDOCKING_MAX_STATE*" (find-package "FF_MSGS::DOCKSTATE"))
(shadow '*UNDOCKING_MAX_STATE* (find-package "FF_MSGS::DOCKSTATE"))
(defconstant ff_msgs::DockState::*UNDOCKING_MAX_STATE* -4)
(defclass ff_msgs::DockState
  :super ros::object
  :slots (_header _state _fsm_event _fsm_state ))

(defmethod ff_msgs::DockState
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

(setf (get ff_msgs::DockState :md5sum-) "09d842ed9109dbc2bf96976f8fb2eaa5")
(setf (get ff_msgs::DockState :datatype-) "ff_msgs/DockState")
(setf (get ff_msgs::DockState :definition-)
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
# Response for Dock/Undock goals

# Header with timestamp
std_msgs/Header header

# Feedback
int8 state
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14
int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13
int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12
int8 INITIALIZING                       = 11
int8 UNKNOWN                            = 10
int8 DOCKING_MAX_STATE                  = 7
int8 DOCKING_SWITCHING_TO_ML_LOC        = 7
int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6
int8 DOCKING_SWITCHING_TO_AR_LOC        = 5
int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4
int8 DOCKING_CHECKING_ATTACHED          = 3
int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2
int8 DOCKING_SWITCHING_TO_NO_LOC        = 1
int8 DOCKED                             = 0
int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1
int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2
int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3
int8 UNDOCKED                           = -4
int8 UNDOCKING_MAX_STATE                = -4

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



(provide :ff_msgs/DockState "09d842ed9109dbc2bf96976f8fb2eaa5")


