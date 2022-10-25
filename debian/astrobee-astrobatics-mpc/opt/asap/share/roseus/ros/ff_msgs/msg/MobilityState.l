;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::MobilityState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'MobilityState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::MOBILITYSTATE")
  (make-package "FF_MSGS::MOBILITYSTATE"))

(in-package "ROS")
;;//! \htmlinclude MobilityState.msg.html


(intern "*DRIFTING*" (find-package "FF_MSGS::MOBILITYSTATE"))
(shadow '*DRIFTING* (find-package "FF_MSGS::MOBILITYSTATE"))
(defconstant ff_msgs::MobilityState::*DRIFTING* 0)
(intern "*STOPPING*" (find-package "FF_MSGS::MOBILITYSTATE"))
(shadow '*STOPPING* (find-package "FF_MSGS::MOBILITYSTATE"))
(defconstant ff_msgs::MobilityState::*STOPPING* 1)
(intern "*FLYING*" (find-package "FF_MSGS::MOBILITYSTATE"))
(shadow '*FLYING* (find-package "FF_MSGS::MOBILITYSTATE"))
(defconstant ff_msgs::MobilityState::*FLYING* 2)
(intern "*DOCKING*" (find-package "FF_MSGS::MOBILITYSTATE"))
(shadow '*DOCKING* (find-package "FF_MSGS::MOBILITYSTATE"))
(defconstant ff_msgs::MobilityState::*DOCKING* 3)
(intern "*PERCHING*" (find-package "FF_MSGS::MOBILITYSTATE"))
(shadow '*PERCHING* (find-package "FF_MSGS::MOBILITYSTATE"))
(defconstant ff_msgs::MobilityState::*PERCHING* 4)
(defclass ff_msgs::MobilityState
  :super ros::object
  :slots (_state _sub_state ))

(defmethod ff_msgs::MobilityState
  (:init
   (&key
    ((:state __state) 0)
    ((:sub_state __sub_state) 0)
    )
   (send-super :init)
   (setq _state (round __state))
   (setq _sub_state (round __sub_state))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:sub_state
   (&optional __sub_state)
   (if __sub_state (setq _sub_state __sub_state)) _sub_state)
  (:serialization-length
   ()
   (+
    ;; uint8 _state
    1
    ;; int32 _sub_state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _state
       (write-byte _state s)
     ;; int32 _sub_state
       (write-long _sub_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _sub_state
     (setq _sub_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::MobilityState :md5sum-) "2c5f9184aace6b4675fe28aa28d9047e")
(setf (get ff_msgs::MobilityState :datatype-) "ff_msgs/MobilityState")
(setf (get ff_msgs::MobilityState :definition-)
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
# Mobility states, based off the enumeration constants in
# rapid::ext::astrobee::AgentState
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 DRIFTING        = 0   # Astrobee's propulsion is off
uint8 STOPPING        = 1   # Astrobee is either stopping or stopped
uint8 FLYING          = 2   # Astrobee is flying
uint8 DOCKING         = 3   # Astrobee is either docking or undocking
uint8 PERCHING        = 4   # Astrobee is either perching or unperching

# Mobility state
uint8 state

# Specifies the progress of the action. For docking, this value can be N to -N
# where N through 1 specifies the progress of a docking action, 0 is docked, and
# -1 through -N specifies the process of an undocking action. For stopping, this
# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means
# the robot is stopped. For perching, this value can be N to -N where N through
# 1 specifies the progress of a perching action, 0 is perched, and -1 through
# -N specifies the process of an unperching action.
int32 sub_state

")



(provide :ff_msgs/MobilityState "2c5f9184aace6b4675fe28aa28d9047e")


