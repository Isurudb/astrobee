; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude FaultState.msg.html

(cl:defclass <FaultState> (roslisp-msg-protocol:ros-message)
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
   (hr_state
    :reader hr_state
    :initarg :hr_state
    :type cl:string
    :initform "")
   (faults
    :reader faults
    :initarg :faults
    :type (cl:vector ff_msgs-msg:Fault)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:Fault :initial-element (cl:make-instance 'ff_msgs-msg:Fault))))
)

(cl:defclass FaultState (<FaultState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaultState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaultState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<FaultState> is deprecated: use ff_msgs-msg:FaultState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FaultState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <FaultState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'hr_state-val :lambda-list '(m))
(cl:defmethod hr_state-val ((m <FaultState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:hr_state-val is deprecated.  Use ff_msgs-msg:hr_state instead.")
  (hr_state m))

(cl:ensure-generic-function 'faults-val :lambda-list '(m))
(cl:defmethod faults-val ((m <FaultState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:faults-val is deprecated.  Use ff_msgs-msg:faults instead.")
  (faults m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FaultState>)))
    "Constants for message type '<FaultState>"
  '((:STARTING_UP . 0)
    (:FUNCTIONAL . 1)
    (:FAULT . 2)
    (:BLOCKED . 3)
    (:RELOADING_NODELETS . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FaultState)))
    "Constants for message type 'FaultState"
  '((:STARTING_UP . 0)
    (:FUNCTIONAL . 1)
    (:FAULT . 2)
    (:BLOCKED . 3)
    (:RELOADING_NODELETS . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaultState>) ostream)
  "Serializes a message object of type '<FaultState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hr_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hr_state))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'faults))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'faults))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaultState>) istream)
  "Deserializes a message object of type '<FaultState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hr_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hr_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'faults) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'faults)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:Fault))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaultState>)))
  "Returns string type for a message object of type '<FaultState>"
  "ff_msgs/FaultState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaultState)))
  "Returns string type for a message object of type 'FaultState"
  "ff_msgs/FaultState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaultState>)))
  "Returns md5sum for a message object of type '<FaultState>"
  "4ded9e5628846b2af7eff4a5b8d34c68")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaultState)))
  "Returns md5sum for a message object of type 'FaultState"
  "4ded9e5628846b2af7eff4a5b8d34c68")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaultState>)))
  "Returns full string definition for message of type '<FaultState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault state message used to alert the ground of the current faults. It is also~%# used to express to the executive that a fault has occurred that indirectly~%# affects the motion of the robot.~%~%std_msgs/Header header~%~%# Not sent to the ground, only used by the executive to determine what commands~%# to accept.~%uint8 state~%# System starting up~%uint8 STARTING_UP           = 0~%# No faults are occurring in system~%uint8 FUNCTIONAL            = 1~%# Faults are occurring in the system which may or may not leave the robot~%# functional~%uint8 FAULT                 = 2~%# A fault has occurred that indirectly affects the motion of the robot~%uint8 BLOCKED               = 3~%# Recovering from nodes dying on startup~%uint8 RELOADING_NODELETS    = 4~%~%# A human readable version of the state - only really used for when nodes die on~%# startup and need to be restarted.~%string hr_state~%~%# Faults occurring in the astrobee system, can only send 32 faults down~%ff_msgs/Fault[] faults~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/Fault~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault message is used to provide all the information about an occurring fault~%~%time time_of_fault        # Time when fault occurred~%~%uint32 id                 # id specifying fault~%~%string msg                # string specifying why the fault occurred~%~%ff_msgs/FaultData[] data  # Data used for fault analysis~%~%================================================================================~%MSG: ff_msgs/FaultData~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%# ~%# Fault data messsage contains information of why the fault occurred~%~%uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float~%uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int~%uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string~%~%string key  # Specifies what the data in the msg is, can only be 32 chars long~%~%uint8 data_type   # Specifies the type of data in the message~%~%float32 f   # Value used for fault analysis, data_type must be 0~%int32 i     # Value used for fault analysis, data_type must be 1~%string s    # String used for fault analysis, data_type must be 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaultState)))
  "Returns full string definition for message of type 'FaultState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault state message used to alert the ground of the current faults. It is also~%# used to express to the executive that a fault has occurred that indirectly~%# affects the motion of the robot.~%~%std_msgs/Header header~%~%# Not sent to the ground, only used by the executive to determine what commands~%# to accept.~%uint8 state~%# System starting up~%uint8 STARTING_UP           = 0~%# No faults are occurring in system~%uint8 FUNCTIONAL            = 1~%# Faults are occurring in the system which may or may not leave the robot~%# functional~%uint8 FAULT                 = 2~%# A fault has occurred that indirectly affects the motion of the robot~%uint8 BLOCKED               = 3~%# Recovering from nodes dying on startup~%uint8 RELOADING_NODELETS    = 4~%~%# A human readable version of the state - only really used for when nodes die on~%# startup and need to be restarted.~%string hr_state~%~%# Faults occurring in the astrobee system, can only send 32 faults down~%ff_msgs/Fault[] faults~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/Fault~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault message is used to provide all the information about an occurring fault~%~%time time_of_fault        # Time when fault occurred~%~%uint32 id                 # id specifying fault~%~%string msg                # string specifying why the fault occurred~%~%ff_msgs/FaultData[] data  # Data used for fault analysis~%~%================================================================================~%MSG: ff_msgs/FaultData~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%# ~%# Fault data messsage contains information of why the fault occurred~%~%uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float~%uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int~%uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string~%~%string key  # Specifies what the data in the msg is, can only be 32 chars long~%~%uint8 data_type   # Specifies the type of data in the message~%~%float32 f   # Value used for fault analysis, data_type must be 0~%int32 i     # Value used for fault analysis, data_type must be 1~%string s    # String used for fault analysis, data_type must be 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaultState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'hr_state))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'faults) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaultState>))
  "Converts a ROS message object to a list"
  (cl:list 'FaultState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':hr_state (hr_state msg))
    (cl:cons ':faults (faults msg))
))
