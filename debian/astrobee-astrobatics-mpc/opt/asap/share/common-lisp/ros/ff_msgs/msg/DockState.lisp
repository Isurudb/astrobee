; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DockState.msg.html

(cl:defclass <DockState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (fsm_event
    :reader fsm_event
    :initarg :fsm_event
    :type cl:string
    :initform "")
   (fsm_state
    :reader fsm_state
    :initarg :fsm_state
    :type cl:string
    :initform ""))
)

(cl:defclass DockState (<DockState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DockState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DockState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DockState> is deprecated: use ff_msgs-msg:DockState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DockState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <DockState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'fsm_event-val :lambda-list '(m))
(cl:defmethod fsm_event-val ((m <DockState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_event-val is deprecated.  Use ff_msgs-msg:fsm_event instead.")
  (fsm_event m))

(cl:ensure-generic-function 'fsm_state-val :lambda-list '(m))
(cl:defmethod fsm_state-val ((m <DockState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_state-val is deprecated.  Use ff_msgs-msg:fsm_state instead.")
  (fsm_state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DockState>)))
    "Constants for message type '<DockState>"
  '((:RECOVERY_SWITCHING_TO_ML_LOC . 15)
    (:RECOVERY_MOVING_TO_APPROACH_POSE . 14)
    (:RECOVERY_WAITING_FOR_SPIN_DOWN . 13)
    (:RECOVERY_SWITCHING_TO_NO_LOC . 12)
    (:INITIALIZING . 11)
    (:UNKNOWN . 10)
    (:DOCKING_MAX_STATE . 7)
    (:DOCKING_SWITCHING_TO_ML_LOC . 7)
    (:DOCKING_MOVING_TO_APPROACH_POSE . 6)
    (:DOCKING_SWITCHING_TO_AR_LOC . 5)
    (:DOCKING_MOVING_TO_COMPLETE_POSE . 4)
    (:DOCKING_CHECKING_ATTACHED . 3)
    (:DOCKING_WAITING_FOR_SPIN_DOWN . 2)
    (:DOCKING_SWITCHING_TO_NO_LOC . 1)
    (:DOCKED . 0)
    (:UNDOCKING_SWITCHING_TO_ML_LOC . -1)
    (:UNDOCKING_WAITING_FOR_SPIN_UP . -2)
    (:UNDOCKING_MOVING_TO_APPROACH_POSE . -3)
    (:UNDOCKED . -4)
    (:UNDOCKING_MAX_STATE . -4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DockState)))
    "Constants for message type 'DockState"
  '((:RECOVERY_SWITCHING_TO_ML_LOC . 15)
    (:RECOVERY_MOVING_TO_APPROACH_POSE . 14)
    (:RECOVERY_WAITING_FOR_SPIN_DOWN . 13)
    (:RECOVERY_SWITCHING_TO_NO_LOC . 12)
    (:INITIALIZING . 11)
    (:UNKNOWN . 10)
    (:DOCKING_MAX_STATE . 7)
    (:DOCKING_SWITCHING_TO_ML_LOC . 7)
    (:DOCKING_MOVING_TO_APPROACH_POSE . 6)
    (:DOCKING_SWITCHING_TO_AR_LOC . 5)
    (:DOCKING_MOVING_TO_COMPLETE_POSE . 4)
    (:DOCKING_CHECKING_ATTACHED . 3)
    (:DOCKING_WAITING_FOR_SPIN_DOWN . 2)
    (:DOCKING_SWITCHING_TO_NO_LOC . 1)
    (:DOCKED . 0)
    (:UNDOCKING_SWITCHING_TO_ML_LOC . -1)
    (:UNDOCKING_WAITING_FOR_SPIN_UP . -2)
    (:UNDOCKING_MOVING_TO_APPROACH_POSE . -3)
    (:UNDOCKED . -4)
    (:UNDOCKING_MAX_STATE . -4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DockState>) ostream)
  "Serializes a message object of type '<DockState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fsm_event))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fsm_event))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fsm_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fsm_state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DockState>) istream)
  "Deserializes a message object of type '<DockState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fsm_event) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fsm_event) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fsm_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fsm_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DockState>)))
  "Returns string type for a message object of type '<DockState>"
  "ff_msgs/DockState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DockState)))
  "Returns string type for a message object of type 'DockState"
  "ff_msgs/DockState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DockState>)))
  "Returns md5sum for a message object of type '<DockState>"
  "09d842ed9109dbc2bf96976f8fb2eaa5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DockState)))
  "Returns md5sum for a message object of type 'DockState"
  "09d842ed9109dbc2bf96976f8fb2eaa5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DockState>)))
  "Returns full string definition for message of type '<DockState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Response for Dock/Undock goals~%~%# Header with timestamp~%std_msgs/Header header~%~%# Feedback~%int8 state~%int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15~%int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14~%int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13~%int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12~%int8 INITIALIZING                       = 11~%int8 UNKNOWN                            = 10~%int8 DOCKING_MAX_STATE                  = 7~%int8 DOCKING_SWITCHING_TO_ML_LOC        = 7~%int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6~%int8 DOCKING_SWITCHING_TO_AR_LOC        = 5~%int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4~%int8 DOCKING_CHECKING_ATTACHED          = 3~%int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2~%int8 DOCKING_SWITCHING_TO_NO_LOC        = 1~%int8 DOCKED                             = 0~%int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1~%int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2~%int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3~%int8 UNDOCKED                           = -4~%int8 UNDOCKING_MAX_STATE                = -4~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DockState)))
  "Returns full string definition for message of type 'DockState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Response for Dock/Undock goals~%~%# Header with timestamp~%std_msgs/Header header~%~%# Feedback~%int8 state~%int8 RECOVERY_SWITCHING_TO_ML_LOC       = 15~%int8 RECOVERY_MOVING_TO_APPROACH_POSE   = 14~%int8 RECOVERY_WAITING_FOR_SPIN_DOWN     = 13~%int8 RECOVERY_SWITCHING_TO_NO_LOC       = 12~%int8 INITIALIZING                       = 11~%int8 UNKNOWN                            = 10~%int8 DOCKING_MAX_STATE                  = 7~%int8 DOCKING_SWITCHING_TO_ML_LOC        = 7~%int8 DOCKING_MOVING_TO_APPROACH_POSE    = 6~%int8 DOCKING_SWITCHING_TO_AR_LOC        = 5~%int8 DOCKING_MOVING_TO_COMPLETE_POSE    = 4~%int8 DOCKING_CHECKING_ATTACHED          = 3~%int8 DOCKING_WAITING_FOR_SPIN_DOWN      = 2~%int8 DOCKING_SWITCHING_TO_NO_LOC        = 1~%int8 DOCKED                             = 0~%int8 UNDOCKING_SWITCHING_TO_ML_LOC      = -1~%int8 UNDOCKING_WAITING_FOR_SPIN_UP      = -2~%int8 UNDOCKING_MOVING_TO_APPROACH_POSE  = -3~%int8 UNDOCKED                           = -4~%int8 UNDOCKING_MAX_STATE                = -4~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DockState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'fsm_event))
     4 (cl:length (cl:slot-value msg 'fsm_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DockState>))
  "Converts a ROS message object to a list"
  (cl:list 'DockState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':fsm_event (fsm_event msg))
    (cl:cons ':fsm_state (fsm_state msg))
))
