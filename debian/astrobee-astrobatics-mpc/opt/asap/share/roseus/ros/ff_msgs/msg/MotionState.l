;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::MotionState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'MotionState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::MOTIONSTATE")
  (make-package "FF_MSGS::MOTIONSTATE"))

(in-package "ROS")
;;//! \htmlinclude MotionState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*INITIALIZING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*INITIALIZING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*INITIALIZING* 0)
(intern "*IDLE*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*IDLE* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*IDLE* 1)
(intern "*STOPPED*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*STOPPED* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*STOPPED* 2)
(intern "*IDLING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*IDLING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*IDLING* 3)
(intern "*STOPPING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*STOPPING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*STOPPING* 4)
(intern "*PREPPING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*PREPPING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*PREPPING* 5)
(intern "*BOOTSTRAPPING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*BOOTSTRAPPING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*BOOTSTRAPPING* 6)
(intern "*PLANNING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*PLANNING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*PLANNING* 7)
(intern "*PREPARING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*PREPARING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*PREPARING* 8)
(intern "*CONTROLLING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*CONTROLLING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*CONTROLLING* 9)
(intern "*REPLANNING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*REPLANNING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*REPLANNING* 10)
(intern "*HALTING*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*HALTING* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*HALTING* 11)
(intern "*REPLAN_WAIT*" (find-package "FF_MSGS::MOTIONSTATE"))
(shadow '*REPLAN_WAIT* (find-package "FF_MSGS::MOTIONSTATE"))
(defconstant ff_msgs::MotionState::*REPLAN_WAIT* 12)
(defclass ff_msgs::MotionState
  :super ros::object
  :slots (_header _state _fsm_event _fsm_state ))

(defmethod ff_msgs::MotionState
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

(setf (get ff_msgs::MotionState :md5sum-) "ca36119625f2a1b66f63a30344414dac")
(setf (get ff_msgs::MotionState :datatype-) "ff_msgs/MotionState")
(setf (get ff_msgs::MotionState :definition-)
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
# Locked topic that registers updates to the internal dock state

# Header with timestamp
std_msgs/Header header

# The state of the mobility subsystem
int8 state
int8 INITIALIZING        = 0
int8 IDLE                = 1
int8 STOPPED             = 2
int8 IDLING              = 3
int8 STOPPING            = 4
int8 PREPPING            = 5
int8 BOOTSTRAPPING       = 6
int8 PLANNING            = 7
int8 PREPARING           = 8
int8 CONTROLLING         = 9
int8 REPLANNING          = 10
int8 HALTING             = 11
int8 REPLAN_WAIT         = 12

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



(provide :ff_msgs/MotionState "ca36119625f2a1b66f63a30344414dac")


