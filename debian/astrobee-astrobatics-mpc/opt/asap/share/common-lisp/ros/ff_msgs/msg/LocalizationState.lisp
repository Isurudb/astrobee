; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude LocalizationState.msg.html

(cl:defclass <LocalizationState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:integer
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
    :initform "")
   (pipeline
    :reader pipeline
    :initarg :pipeline
    :type ff_msgs-msg:LocalizationPipeline
    :initform (cl:make-instance 'ff_msgs-msg:LocalizationPipeline)))
)

(cl:defclass LocalizationState (<LocalizationState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalizationState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalizationState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<LocalizationState> is deprecated: use ff_msgs-msg:LocalizationState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'fsm_event-val :lambda-list '(m))
(cl:defmethod fsm_event-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_event-val is deprecated.  Use ff_msgs-msg:fsm_event instead.")
  (fsm_event m))

(cl:ensure-generic-function 'fsm_state-val :lambda-list '(m))
(cl:defmethod fsm_state-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_state-val is deprecated.  Use ff_msgs-msg:fsm_state instead.")
  (fsm_state m))

(cl:ensure-generic-function 'pipeline-val :lambda-list '(m))
(cl:defmethod pipeline-val ((m <LocalizationState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:pipeline-val is deprecated.  Use ff_msgs-msg:pipeline instead.")
  (pipeline m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<LocalizationState>)))
    "Constants for message type '<LocalizationState>"
  '((:INITIALIZING . 0)
    (:DISABLED . 1)
    (:LOCALIZING . 2)
    (:SWITCH_WAITING_FOR_PIPELINE . 3)
    (:SWITCH_WAITING_FOR_FILTER . 4)
    (:BIAS_WAITING_FOR_FILTER . 5)
    (:RESET_WAITING_FOR_FILTER . 6)
    (:UNSTABLE . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'LocalizationState)))
    "Constants for message type 'LocalizationState"
  '((:INITIALIZING . 0)
    (:DISABLED . 1)
    (:LOCALIZING . 2)
    (:SWITCH_WAITING_FOR_PIPELINE . 3)
    (:SWITCH_WAITING_FOR_FILTER . 4)
    (:BIAS_WAITING_FOR_FILTER . 5)
    (:RESET_WAITING_FOR_FILTER . 6)
    (:UNSTABLE . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalizationState>) ostream)
  "Serializes a message object of type '<LocalizationState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pipeline) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalizationState>) istream)
  "Deserializes a message object of type '<LocalizationState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pipeline) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalizationState>)))
  "Returns string type for a message object of type '<LocalizationState>"
  "ff_msgs/LocalizationState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalizationState)))
  "Returns string type for a message object of type 'LocalizationState"
  "ff_msgs/LocalizationState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalizationState>)))
  "Returns md5sum for a message object of type '<LocalizationState>"
  "4fe8f08dfd156a0a44226bd8862089d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalizationState)))
  "Returns md5sum for a message object of type 'LocalizationState"
  "4fe8f08dfd156a0a44226bd8862089d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalizationState>)))
  "Returns full string definition for message of type '<LocalizationState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# The state of the localization system~%~%# Header with timestamp~%std_msgs/Header header~%~%# Tee current state~%int32 state                                 # Current state~%int32 INITIALIZING                    = 0   # Waiting on dependencies~%int32 DISABLED                        = 1   # Localization disabled~%int32 LOCALIZING                      = 2   # Localization enabled~%int32 SWITCH_WAITING_FOR_PIPELINE     = 3   # Waiting for pipeline to stabilize~%int32 SWITCH_WAITING_FOR_FILTER       = 4   # Waiting for filter to stabilize~%int32 BIAS_WAITING_FOR_FILTER         = 5   # Waiting for bias estimation~%int32 RESET_WAITING_FOR_FILTER        = 6   # Waiting for EKF stability~%int32 UNSTABLE                        = 7   # Fallback pipeline unstable~%~%# A human readable version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%~%# The current localization pipeline being used~%ff_msgs/LocalizationPipeline pipeline~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/LocalizationPipeline~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Information about a pipeline~%~%string id                     # Short id for the pipeline~%uint8 mode                    # EKF mode for the pipeline~%string name                   # Long name for the pipe~%bool requires_filter          # Does this pipeline require the EKF~%bool requires_optical_flow    # Does this pipeline require optical flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalizationState)))
  "Returns full string definition for message of type 'LocalizationState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# The state of the localization system~%~%# Header with timestamp~%std_msgs/Header header~%~%# Tee current state~%int32 state                                 # Current state~%int32 INITIALIZING                    = 0   # Waiting on dependencies~%int32 DISABLED                        = 1   # Localization disabled~%int32 LOCALIZING                      = 2   # Localization enabled~%int32 SWITCH_WAITING_FOR_PIPELINE     = 3   # Waiting for pipeline to stabilize~%int32 SWITCH_WAITING_FOR_FILTER       = 4   # Waiting for filter to stabilize~%int32 BIAS_WAITING_FOR_FILTER         = 5   # Waiting for bias estimation~%int32 RESET_WAITING_FOR_FILTER        = 6   # Waiting for EKF stability~%int32 UNSTABLE                        = 7   # Fallback pipeline unstable~%~%# A human readable version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%~%# The current localization pipeline being used~%ff_msgs/LocalizationPipeline pipeline~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/LocalizationPipeline~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Information about a pipeline~%~%string id                     # Short id for the pipeline~%uint8 mode                    # EKF mode for the pipeline~%string name                   # Long name for the pipe~%bool requires_filter          # Does this pipeline require the EKF~%bool requires_optical_flow    # Does this pipeline require optical flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalizationState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:length (cl:slot-value msg 'fsm_event))
     4 (cl:length (cl:slot-value msg 'fsm_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pipeline))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalizationState>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalizationState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':fsm_event (fsm_event msg))
    (cl:cons ':fsm_state (fsm_state msg))
    (cl:cons ':pipeline (pipeline msg))
))
