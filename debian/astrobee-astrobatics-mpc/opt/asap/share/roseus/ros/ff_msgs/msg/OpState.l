;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::OpState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'OpState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::OPSTATE")
  (make-package "FF_MSGS::OPSTATE"))

(in-package "ROS")
;;//! \htmlinclude OpState.msg.html


(intern "*READY*" (find-package "FF_MSGS::OPSTATE"))
(shadow '*READY* (find-package "FF_MSGS::OPSTATE"))
(defconstant ff_msgs::OpState::*READY* 0)
(intern "*PLAN_EXECUTION*" (find-package "FF_MSGS::OPSTATE"))
(shadow '*PLAN_EXECUTION* (find-package "FF_MSGS::OPSTATE"))
(defconstant ff_msgs::OpState::*PLAN_EXECUTION* 1)
(intern "*TELEOPERATION*" (find-package "FF_MSGS::OPSTATE"))
(shadow '*TELEOPERATION* (find-package "FF_MSGS::OPSTATE"))
(defconstant ff_msgs::OpState::*TELEOPERATION* 2)
(intern "*AUTO_RETURN*" (find-package "FF_MSGS::OPSTATE"))
(shadow '*AUTO_RETURN* (find-package "FF_MSGS::OPSTATE"))
(defconstant ff_msgs::OpState::*AUTO_RETURN* 3)
(intern "*FAULT*" (find-package "FF_MSGS::OPSTATE"))
(shadow '*FAULT* (find-package "FF_MSGS::OPSTATE"))
(defconstant ff_msgs::OpState::*FAULT* 4)
(defclass ff_msgs::OpState
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::OpState
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

(setf (get ff_msgs::OpState :md5sum-) "11f7f3b40813a20f5f841918254d08c2")
(setf (get ff_msgs::OpState :datatype-) "ff_msgs/OpState")
(setf (get ff_msgs::OpState :definition-)
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
# Operating States, based off of the enumeration constants
# in rapid::ext::astrobee::AgentState.
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 READY            = 0  # Freeflyer is ready to accept commands
uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan
uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command
uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock
# The freeflyer is either executing a fault response or there is a fault
# occurring in the system that prevents the freeflyer from moving
uint8 FAULT            = 4

# Operating state
uint8 state

")



(provide :ff_msgs/OpState "11f7f3b40813a20f5f841918254d08c2")


