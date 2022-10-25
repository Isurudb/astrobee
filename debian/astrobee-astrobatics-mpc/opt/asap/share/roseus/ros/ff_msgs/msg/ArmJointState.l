;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ArmJointState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ArmJointState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ARMJOINTSTATE")
  (make-package "FF_MSGS::ARMJOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmJointState.msg.html


(intern "*UNKNOWN*" (find-package "FF_MSGS::ARMJOINTSTATE"))
(shadow '*UNKNOWN* (find-package "FF_MSGS::ARMJOINTSTATE"))
(defconstant ff_msgs::ArmJointState::*UNKNOWN* 0)
(intern "*STOWED*" (find-package "FF_MSGS::ARMJOINTSTATE"))
(shadow '*STOWED* (find-package "FF_MSGS::ARMJOINTSTATE"))
(defconstant ff_msgs::ArmJointState::*STOWED* 1)
(intern "*DEPLOYING*" (find-package "FF_MSGS::ARMJOINTSTATE"))
(shadow '*DEPLOYING* (find-package "FF_MSGS::ARMJOINTSTATE"))
(defconstant ff_msgs::ArmJointState::*DEPLOYING* 2)
(intern "*STOPPED*" (find-package "FF_MSGS::ARMJOINTSTATE"))
(shadow '*STOPPED* (find-package "FF_MSGS::ARMJOINTSTATE"))
(defconstant ff_msgs::ArmJointState::*STOPPED* 3)
(intern "*MOVING*" (find-package "FF_MSGS::ARMJOINTSTATE"))
(shadow '*MOVING* (find-package "FF_MSGS::ARMJOINTSTATE"))
(defconstant ff_msgs::ArmJointState::*MOVING* 4)
(intern "*STOWING*" (find-package "FF_MSGS::ARMJOINTSTATE"))
(shadow '*STOWING* (find-package "FF_MSGS::ARMJOINTSTATE"))
(defconstant ff_msgs::ArmJointState::*STOWING* 5)
(defclass ff_msgs::ArmJointState
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::ArmJointState
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

(setf (get ff_msgs::ArmJointState :md5sum-) "18fc27f61231440dbfaa96855881bf89")
(setf (get ff_msgs::ArmJointState :datatype-) "ff_msgs/ArmJointState")
(setf (get ff_msgs::ArmJointState :definition-)
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
# Arm Joint State enum.
#
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState

uint8 UNKNOWN   = 0
uint8 STOWED    = 1
uint8 DEPLOYING = 2
uint8 STOPPED   = 3
uint8 MOVING    = 4
uint8 STOWING   = 5

uint8 state

")



(provide :ff_msgs/ArmJointState "18fc27f61231440dbfaa96855881bf89")


