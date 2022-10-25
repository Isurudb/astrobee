; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude SignalState.msg.html

(cl:defclass <SignalState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SignalState (<SignalState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SignalState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SignalState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<SignalState> is deprecated: use ff_msgs-msg:SignalState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SignalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SignalState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SignalState>)))
    "Constants for message type '<SignalState>"
  '((:VIDEO_ON . 0)
    (:VIDEO_OFF . 1)
    (:SUCCESS . 3)
    (:ENTER_HATCHWAY . 4)
    (:UNDOCK . 5)
    (:UNPERCH . 6)
    (:MOTION_IMPAIRED . 7)
    (:THRUST_FORWARD . 8)
    (:THRUST_AFT . 9)
    (:TURN_RIGHT . 10)
    (:TURN_LEFT . 11)
    (:TURN_UP . 12)
    (:TURN_DOWN . 13)
    (:CLEAR . 14)
    (:SLEEP . 15)
    (:WAKE . 16)
    (:STOP_ALL_LIGHTS . 17)
    (:CHARGING . 18))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SignalState)))
    "Constants for message type 'SignalState"
  '((:VIDEO_ON . 0)
    (:VIDEO_OFF . 1)
    (:SUCCESS . 3)
    (:ENTER_HATCHWAY . 4)
    (:UNDOCK . 5)
    (:UNPERCH . 6)
    (:MOTION_IMPAIRED . 7)
    (:THRUST_FORWARD . 8)
    (:THRUST_AFT . 9)
    (:TURN_RIGHT . 10)
    (:TURN_LEFT . 11)
    (:TURN_UP . 12)
    (:TURN_DOWN . 13)
    (:CLEAR . 14)
    (:SLEEP . 15)
    (:WAKE . 16)
    (:STOP_ALL_LIGHTS . 17)
    (:CHARGING . 18))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SignalState>) ostream)
  "Serializes a message object of type '<SignalState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SignalState>) istream)
  "Deserializes a message object of type '<SignalState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SignalState>)))
  "Returns string type for a message object of type '<SignalState>"
  "ff_msgs/SignalState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SignalState)))
  "Returns string type for a message object of type 'SignalState"
  "ff_msgs/SignalState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SignalState>)))
  "Returns md5sum for a message object of type '<SignalState>"
  "1d27ab072e4b4bf58f04e75ff6768d4e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SignalState)))
  "Returns md5sum for a message object of type 'SignalState"
  "1d27ab072e4b4bf58f04e75ff6768d4e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SignalState>)))
  "Returns full string definition for message of type '<SignalState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Signal state which is based on what the Astrobee is doing. Should be used to~%# figure out what should be displayed on the signal lights and touch screen.~%~%# Header with timestamp~%std_msgs/Header header~%~%uint8 VIDEO_ON              = 0~%uint8 VIDEO_OFF             = 1~%uint8 SUCCESS               = 3~%uint8 ENTER_HATCHWAY        = 4~%uint8 UNDOCK                = 5~%uint8 UNPERCH               = 6~%uint8 MOTION_IMPAIRED       = 7~%uint8 THRUST_FORWARD        = 8~%uint8 THRUST_AFT            = 9~%uint8 TURN_RIGHT            = 10~%uint8 TURN_LEFT             = 11~%uint8 TURN_UP               = 12~%uint8 TURN_DOWN             = 13~%uint8 CLEAR                 = 14~%uint8 SLEEP                 = 15~%uint8 WAKE                  = 16~%uint8 STOP_ALL_LIGHTS       = 17~%uint8 CHARGING              = 18~%~%# Signal state~%uint8 state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SignalState)))
  "Returns full string definition for message of type 'SignalState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Signal state which is based on what the Astrobee is doing. Should be used to~%# figure out what should be displayed on the signal lights and touch screen.~%~%# Header with timestamp~%std_msgs/Header header~%~%uint8 VIDEO_ON              = 0~%uint8 VIDEO_OFF             = 1~%uint8 SUCCESS               = 3~%uint8 ENTER_HATCHWAY        = 4~%uint8 UNDOCK                = 5~%uint8 UNPERCH               = 6~%uint8 MOTION_IMPAIRED       = 7~%uint8 THRUST_FORWARD        = 8~%uint8 THRUST_AFT            = 9~%uint8 TURN_RIGHT            = 10~%uint8 TURN_LEFT             = 11~%uint8 TURN_UP               = 12~%uint8 TURN_DOWN             = 13~%uint8 CLEAR                 = 14~%uint8 SLEEP                 = 15~%uint8 WAKE                  = 16~%uint8 STOP_ALL_LIGHTS       = 17~%uint8 CHARGING              = 18~%~%# Signal state~%uint8 state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SignalState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SignalState>))
  "Converts a ROS message object to a list"
  (cl:list 'SignalState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
