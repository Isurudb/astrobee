;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ArmGripperState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ArmGripperState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ARMGRIPPERSTATE")
  (make-package "FF_MSGS::ARMGRIPPERSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmGripperState.msg.html


(intern "*UNKNOWN*" (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(shadow '*UNKNOWN* (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(defconstant ff_msgs::ArmGripperState::*UNKNOWN* 0)
(intern "*UNCALIBRATED*" (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(shadow '*UNCALIBRATED* (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(defconstant ff_msgs::ArmGripperState::*UNCALIBRATED* 1)
(intern "*CALIBRATING*" (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(shadow '*CALIBRATING* (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(defconstant ff_msgs::ArmGripperState::*CALIBRATING* 2)
(intern "*CLOSED*" (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(shadow '*CLOSED* (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(defconstant ff_msgs::ArmGripperState::*CLOSED* 3)
(intern "*OPEN*" (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(shadow '*OPEN* (find-package "FF_MSGS::ARMGRIPPERSTATE"))
(defconstant ff_msgs::ArmGripperState::*OPEN* 4)
(defclass ff_msgs::ArmGripperState
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::ArmGripperState
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

(setf (get ff_msgs::ArmGripperState :md5sum-) "3857276ebd0698497c3781f3fb94fb88")
(setf (get ff_msgs::ArmGripperState :datatype-) "ff_msgs/ArmGripperState")
(setf (get ff_msgs::ArmGripperState :definition-)
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
# Arm Gripper State enum
#
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState

uint8 UNKNOWN      = 0
uint8 UNCALIBRATED = 1
uint8 CALIBRATING  = 2
uint8 CLOSED       = 3
uint8 OPEN         = 4

uint8 state

")



(provide :ff_msgs/ArmGripperState "3857276ebd0698497c3781f3fb94fb88")


