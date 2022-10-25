;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::LocalizationState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'LocalizationState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::LOCALIZATIONSTATE")
  (make-package "FF_MSGS::LOCALIZATIONSTATE"))

(in-package "ROS")
;;//! \htmlinclude LocalizationState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*INITIALIZING*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*INITIALIZING* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*INITIALIZING* 0)
(intern "*DISABLED*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*DISABLED* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*DISABLED* 1)
(intern "*LOCALIZING*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*LOCALIZING* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*LOCALIZING* 2)
(intern "*SWITCH_WAITING_FOR_PIPELINE*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*SWITCH_WAITING_FOR_PIPELINE* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*SWITCH_WAITING_FOR_PIPELINE* 3)
(intern "*SWITCH_WAITING_FOR_FILTER*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*SWITCH_WAITING_FOR_FILTER* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*SWITCH_WAITING_FOR_FILTER* 4)
(intern "*BIAS_WAITING_FOR_FILTER*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*BIAS_WAITING_FOR_FILTER* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*BIAS_WAITING_FOR_FILTER* 5)
(intern "*RESET_WAITING_FOR_FILTER*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*RESET_WAITING_FOR_FILTER* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*RESET_WAITING_FOR_FILTER* 6)
(intern "*UNSTABLE*" (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(shadow '*UNSTABLE* (find-package "FF_MSGS::LOCALIZATIONSTATE"))
(defconstant ff_msgs::LocalizationState::*UNSTABLE* 7)
(defclass ff_msgs::LocalizationState
  :super ros::object
  :slots (_header _state _fsm_event _fsm_state _pipeline ))

(defmethod ff_msgs::LocalizationState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:fsm_event __fsm_event) "")
    ((:fsm_state __fsm_state) "")
    ((:pipeline __pipeline) (instance ff_msgs::LocalizationPipeline :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _fsm_event (string __fsm_event))
   (setq _fsm_state (string __fsm_state))
   (setq _pipeline __pipeline)
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
  (:pipeline
   (&rest __pipeline)
   (if (keywordp (car __pipeline))
       (send* _pipeline __pipeline)
     (progn
       (if __pipeline (setq _pipeline (car __pipeline)))
       _pipeline)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _state
    4
    ;; string _fsm_event
    4 (length _fsm_event)
    ;; string _fsm_state
    4 (length _fsm_state)
    ;; ff_msgs/LocalizationPipeline _pipeline
    (send _pipeline :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _state
       (write-long _state s)
     ;; string _fsm_event
       (write-long (length _fsm_event) s) (princ _fsm_event s)
     ;; string _fsm_state
       (write-long (length _fsm_state) s) (princ _fsm_state s)
     ;; ff_msgs/LocalizationPipeline _pipeline
       (send _pipeline :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _fsm_event
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_event (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _fsm_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/LocalizationPipeline _pipeline
     (send _pipeline :deserialize buf ptr-) (incf ptr- (send _pipeline :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::LocalizationState :md5sum-) "4fe8f08dfd156a0a44226bd8862089d4")
(setf (get ff_msgs::LocalizationState :datatype-) "ff_msgs/LocalizationState")
(setf (get ff_msgs::LocalizationState :definition-)
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
# The state of the localization system

# Header with timestamp
std_msgs/Header header

# Tee current state
int32 state                                 # Current state
int32 INITIALIZING                    = 0   # Waiting on dependencies
int32 DISABLED                        = 1   # Localization disabled
int32 LOCALIZING                      = 2   # Localization enabled
int32 SWITCH_WAITING_FOR_PIPELINE     = 3   # Waiting for pipeline to stabilize
int32 SWITCH_WAITING_FOR_FILTER       = 4   # Waiting for filter to stabilize
int32 BIAS_WAITING_FOR_FILTER         = 5   # Waiting for bias estimation
int32 RESET_WAITING_FOR_FILTER        = 6   # Waiting for EKF stability
int32 UNSTABLE                        = 7   # Fallback pipeline unstable

# A human readable version of the (event) -> [state] transition
string fsm_event
string fsm_state

# The current localization pipeline being used
ff_msgs/LocalizationPipeline pipeline

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
MSG: ff_msgs/LocalizationPipeline
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
# Information about a pipeline

string id                     # Short id for the pipeline
uint8 mode                    # EKF mode for the pipeline
string name                   # Long name for the pipe
bool requires_filter          # Does this pipeline require the EKF
bool requires_optical_flow    # Does this pipeline require optical flow

")



(provide :ff_msgs/LocalizationState "4fe8f08dfd156a0a44226bd8862089d4")


