;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ExecState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ExecState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::EXECSTATE")
  (make-package "FF_MSGS::EXECSTATE"))

(in-package "ROS")
;;//! \htmlinclude ExecState.msg.html


(intern "*IDLE*" (find-package "FF_MSGS::EXECSTATE"))
(shadow '*IDLE* (find-package "FF_MSGS::EXECSTATE"))
(defconstant ff_msgs::ExecState::*IDLE* 0)
(intern "*EXECUTING*" (find-package "FF_MSGS::EXECSTATE"))
(shadow '*EXECUTING* (find-package "FF_MSGS::EXECSTATE"))
(defconstant ff_msgs::ExecState::*EXECUTING* 1)
(intern "*PAUSED*" (find-package "FF_MSGS::EXECSTATE"))
(shadow '*PAUSED* (find-package "FF_MSGS::EXECSTATE"))
(defconstant ff_msgs::ExecState::*PAUSED* 2)
(intern "*ERROR*" (find-package "FF_MSGS::EXECSTATE"))
(shadow '*ERROR* (find-package "FF_MSGS::EXECSTATE"))
(defconstant ff_msgs::ExecState::*ERROR* 3)
(defclass ff_msgs::ExecState
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::ExecState
  (:init
   (&key
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::ExecState :md5sum-) "10a48ab48fd2106828caec7c2cbb9e91")
(setf (get ff_msgs::ExecState :datatype-) "ff_msgs/ExecState")
(setf (get ff_msgs::ExecState :definition-)
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
# Execution States, based off of the enumeration constants in
# rapid::ext::astrobee::AgentState
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 IDLE      = 0   # Process is idle
uint8 EXECUTING = 1   # Process is executing
uint8 PAUSED    = 2   # Process is paused
uint8 ERROR     = 3   # Process encountered an error

# Execution state
uint8 state

")



(provide :ff_msgs/ExecState "10a48ab48fd2106828caec7c2cbb9e91")


