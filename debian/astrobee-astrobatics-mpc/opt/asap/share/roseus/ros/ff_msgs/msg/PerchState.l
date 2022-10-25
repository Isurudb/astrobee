;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::PerchState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'PerchState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::PERCHSTATE")
  (make-package "FF_MSGS::PERCHSTATE"))

(in-package "ROS")
;;//! \htmlinclude PerchState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*RECOVERY_MOVING_TO_RECOVERY_POSE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*RECOVERY_MOVING_TO_RECOVERY_POSE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*RECOVERY_MOVING_TO_RECOVERY_POSE* 18)
(intern "*RECOVERY_SWITCHING_TO_ML_LOC*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*RECOVERY_SWITCHING_TO_ML_LOC* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*RECOVERY_SWITCHING_TO_ML_LOC* 17)
(intern "*RECOVERY_STOWING_ARM*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*RECOVERY_STOWING_ARM* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*RECOVERY_STOWING_ARM* 16)
(intern "*RECOVERY_MOVING_TO_APPROACH_POSE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*RECOVERY_MOVING_TO_APPROACH_POSE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*RECOVERY_MOVING_TO_APPROACH_POSE* 15)
(intern "*RECOVERY_OPENING_GRIPPER*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*RECOVERY_OPENING_GRIPPER* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*RECOVERY_OPENING_GRIPPER* 14)
(intern "*INITIALIZING*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*INITIALIZING* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*INITIALIZING* 13)
(intern "*UNKNOWN*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNKNOWN* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNKNOWN* 12)
(intern "*PERCHING_MAX_STATE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_MAX_STATE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_MAX_STATE* 11)
(intern "*PERCHING_SWITCHING_TO_HR_LOC*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_SWITCHING_TO_HR_LOC* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_SWITCHING_TO_HR_LOC* 11)
(intern "*PERCHING_MOVING_TO_APPROACH_POSE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_MOVING_TO_APPROACH_POSE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_MOVING_TO_APPROACH_POSE* 10)
(intern "*PERCHING_ENSURING_APPROACH_POSE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_ENSURING_APPROACH_POSE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_ENSURING_APPROACH_POSE* 9)
(intern "*PERCHING_DEPLOYING_ARM*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_DEPLOYING_ARM* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_DEPLOYING_ARM* 8)
(intern "*PERCHING_OPENING_GRIPPER*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_OPENING_GRIPPER* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_OPENING_GRIPPER* 7)
(intern "*PERCHING_MOVING_TO_COMPLETE_POSE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_MOVING_TO_COMPLETE_POSE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_MOVING_TO_COMPLETE_POSE* 6)
(intern "*PERCHING_CLOSING_GRIPPER*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_CLOSING_GRIPPER* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_CLOSING_GRIPPER* 5)
(intern "*PERCHING_CHECKING_ATTACHED*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_CHECKING_ATTACHED* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_CHECKING_ATTACHED* 4)
(intern "*PERCHING_WAITING_FOR_SPIN_DOWN*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_WAITING_FOR_SPIN_DOWN* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_WAITING_FOR_SPIN_DOWN* 3)
(intern "*PERCHING_SWITCHING_TO_PL_LOC*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_SWITCHING_TO_PL_LOC* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_SWITCHING_TO_PL_LOC* 2)
(intern "*PERCHING_STOPPING*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHING_STOPPING* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHING_STOPPING* 1)
(intern "*PERCHED*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*PERCHED* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*PERCHED* 0)
(intern "*UNPERCHING_SWITCHING_TO_HR_LOC*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_SWITCHING_TO_HR_LOC* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_SWITCHING_TO_HR_LOC* -1)
(intern "*UNPERCHING_WAITING_FOR_SPIN_UP*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_WAITING_FOR_SPIN_UP* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_WAITING_FOR_SPIN_UP* -2)
(intern "*UNPERCHING_OPENING_GRIPPER*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_OPENING_GRIPPER* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_OPENING_GRIPPER* -3)
(intern "*UNPERCHING_MOVING_TO_APPROACH_POSE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_MOVING_TO_APPROACH_POSE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_MOVING_TO_APPROACH_POSE* -4)
(intern "*UNPERCHING_STOWING_ARM*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_STOWING_ARM* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_STOWING_ARM* -5)
(intern "*UNPERCHING_SWITCHING_TO_ML_LOC*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_SWITCHING_TO_ML_LOC* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_SWITCHING_TO_ML_LOC* -6)
(intern "*UNPERCHED*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHED* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHED* -7)
(intern "*UNPERCHING_MAX_STATE*" (find-package "FF_MSGS::PERCHSTATE"))
(shadow '*UNPERCHING_MAX_STATE* (find-package "FF_MSGS::PERCHSTATE"))
(defconstant ff_msgs::PerchState::*UNPERCHING_MAX_STATE* -7)
(defclass ff_msgs::PerchState
  :super ros::object
  :slots (_header _state _fsm_event _fsm_state ))

(defmethod ff_msgs::PerchState
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

(setf (get ff_msgs::PerchState :md5sum-) "5a3bc3d43070c3cb7655a2601ea68801")
(setf (get ff_msgs::PerchState :datatype-) "ff_msgs/PerchState")
(setf (get ff_msgs::PerchState :definition-)
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
# The state of the perching system

# Header with timestamp
std_msgs/Header header

# Feedback
int8 state

int8 RECOVERY_MOVING_TO_RECOVERY_POSE   = 18
int8 RECOVERY_SWITCHING_TO_ML_LOC       = 17
int8 RECOVERY_STOWING_ARM               = 16
int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 15
int8 RECOVERY_OPENING_GRIPPER           = 14
int8 INITIALIZING                       = 13
int8 UNKNOWN                            = 12
# Used to check the perching/unperching ranges
int8 PERCHING_MAX_STATE                 = 11
int8 PERCHING_SWITCHING_TO_HR_LOC       = 11
int8 PERCHING_MOVING_TO_APPROACH_POSE   = 10
int8 PERCHING_ENSURING_APPROACH_POSE    = 9
int8 PERCHING_DEPLOYING_ARM             = 8
int8 PERCHING_OPENING_GRIPPER           = 7
int8 PERCHING_MOVING_TO_COMPLETE_POSE   = 6
int8 PERCHING_CLOSING_GRIPPER           = 5
int8 PERCHING_CHECKING_ATTACHED         = 4
int8 PERCHING_WAITING_FOR_SPIN_DOWN     = 3
int8 PERCHING_SWITCHING_TO_PL_LOC       = 2
int8 PERCHING_STOPPING                  = 1
int8 PERCHED                            = 0
int8 UNPERCHING_SWITCHING_TO_HR_LOC     = -1
int8 UNPERCHING_WAITING_FOR_SPIN_UP     = -2
int8 UNPERCHING_OPENING_GRIPPER         = -3
int8 UNPERCHING_MOVING_TO_APPROACH_POSE = -4
int8 UNPERCHING_STOWING_ARM             = -5
int8 UNPERCHING_SWITCHING_TO_ML_LOC     = -6
int8 UNPERCHED                          = -7
# Used to check the perching/unperching ranges
int8 UNPERCHING_MAX_STATE               = -7

# A human readable version of the (event) -> [state] transition
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



(provide :ff_msgs/PerchState "5a3bc3d43070c3cb7655a2601ea68801")


