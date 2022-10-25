;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SignalState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SignalState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SIGNALSTATE")
  (make-package "FF_MSGS::SIGNALSTATE"))

(in-package "ROS")
;;//! \htmlinclude SignalState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*VIDEO_ON*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*VIDEO_ON* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*VIDEO_ON* 0)
(intern "*VIDEO_OFF*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*VIDEO_OFF* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*VIDEO_OFF* 1)
(intern "*SUCCESS*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*SUCCESS* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*SUCCESS* 3)
(intern "*ENTER_HATCHWAY*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*ENTER_HATCHWAY* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*ENTER_HATCHWAY* 4)
(intern "*UNDOCK*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*UNDOCK* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*UNDOCK* 5)
(intern "*UNPERCH*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*UNPERCH* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*UNPERCH* 6)
(intern "*MOTION_IMPAIRED*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*MOTION_IMPAIRED* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*MOTION_IMPAIRED* 7)
(intern "*THRUST_FORWARD*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*THRUST_FORWARD* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*THRUST_FORWARD* 8)
(intern "*THRUST_AFT*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*THRUST_AFT* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*THRUST_AFT* 9)
(intern "*TURN_RIGHT*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*TURN_RIGHT* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*TURN_RIGHT* 10)
(intern "*TURN_LEFT*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*TURN_LEFT* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*TURN_LEFT* 11)
(intern "*TURN_UP*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*TURN_UP* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*TURN_UP* 12)
(intern "*TURN_DOWN*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*TURN_DOWN* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*TURN_DOWN* 13)
(intern "*CLEAR*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*CLEAR* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*CLEAR* 14)
(intern "*SLEEP*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*SLEEP* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*SLEEP* 15)
(intern "*WAKE*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*WAKE* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*WAKE* 16)
(intern "*STOP_ALL_LIGHTS*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*STOP_ALL_LIGHTS* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*STOP_ALL_LIGHTS* 17)
(intern "*CHARGING*" (find-package "FF_MSGS::SIGNALSTATE"))
(shadow '*CHARGING* (find-package "FF_MSGS::SIGNALSTATE"))
(defconstant ff_msgs::SignalState::*CHARGING* 18)
(defclass ff_msgs::SignalState
  :super ros::object
  :slots (_header _state ))

(defmethod ff_msgs::SignalState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
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
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _state
       (write-byte _state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::SignalState :md5sum-) "1d27ab072e4b4bf58f04e75ff6768d4e")
(setf (get ff_msgs::SignalState :datatype-) "ff_msgs/SignalState")
(setf (get ff_msgs::SignalState :definition-)
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
# Signal state which is based on what the Astrobee is doing. Should be used to
# figure out what should be displayed on the signal lights and touch screen.

# Header with timestamp
std_msgs/Header header

uint8 VIDEO_ON              = 0
uint8 VIDEO_OFF             = 1
uint8 SUCCESS               = 3
uint8 ENTER_HATCHWAY        = 4
uint8 UNDOCK                = 5
uint8 UNPERCH               = 6
uint8 MOTION_IMPAIRED       = 7
uint8 THRUST_FORWARD        = 8
uint8 THRUST_AFT            = 9
uint8 TURN_RIGHT            = 10
uint8 TURN_LEFT             = 11
uint8 TURN_UP               = 12
uint8 TURN_DOWN             = 13
uint8 CLEAR                 = 14
uint8 SLEEP                 = 15
uint8 WAKE                  = 16
uint8 STOP_ALL_LIGHTS       = 17
uint8 CHARGING              = 18

# Signal state
uint8 state

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



(provide :ff_msgs/SignalState "1d27ab072e4b4bf58f04e75ff6768d4e")


