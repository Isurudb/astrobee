; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ExecState.msg.html

(cl:defclass <ExecState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ExecState (<ExecState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ExecState> is deprecated: use ff_msgs-msg:ExecState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ExecState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ExecState>)))
    "Constants for message type '<ExecState>"
  '((:IDLE . 0)
    (:EXECUTING . 1)
    (:PAUSED . 2)
    (:ERROR . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ExecState)))
    "Constants for message type 'ExecState"
  '((:IDLE . 0)
    (:EXECUTING . 1)
    (:PAUSED . 2)
    (:ERROR . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecState>) ostream)
  "Serializes a message object of type '<ExecState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecState>) istream)
  "Deserializes a message object of type '<ExecState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecState>)))
  "Returns string type for a message object of type '<ExecState>"
  "ff_msgs/ExecState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecState)))
  "Returns string type for a message object of type 'ExecState"
  "ff_msgs/ExecState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecState>)))
  "Returns md5sum for a message object of type '<ExecState>"
  "10a48ab48fd2106828caec7c2cbb9e91")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecState)))
  "Returns md5sum for a message object of type 'ExecState"
  "10a48ab48fd2106828caec7c2cbb9e91")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecState>)))
  "Returns full string definition for message of type '<ExecState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Execution States, based off of the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 IDLE      = 0   # Process is idle~%uint8 EXECUTING = 1   # Process is executing~%uint8 PAUSED    = 2   # Process is paused~%uint8 ERROR     = 3   # Process encountered an error~%~%# Execution state~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecState)))
  "Returns full string definition for message of type 'ExecState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Execution States, based off of the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 IDLE      = 0   # Process is idle~%uint8 EXECUTING = 1   # Process is executing~%uint8 PAUSED    = 2   # Process is paused~%uint8 ERROR     = 3   # Process encountered an error~%~%# Execution state~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecState>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecState
    (cl:cons ':state (state msg))
))
