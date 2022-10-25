; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude OpState.msg.html

(cl:defclass <OpState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass OpState (<OpState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OpState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OpState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<OpState> is deprecated: use ff_msgs-msg:OpState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <OpState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<OpState>)))
    "Constants for message type '<OpState>"
  '((:READY . 0)
    (:PLAN_EXECUTION . 1)
    (:TELEOPERATION . 2)
    (:AUTO_RETURN . 3)
    (:FAULT . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'OpState)))
    "Constants for message type 'OpState"
  '((:READY . 0)
    (:PLAN_EXECUTION . 1)
    (:TELEOPERATION . 2)
    (:AUTO_RETURN . 3)
    (:FAULT . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OpState>) ostream)
  "Serializes a message object of type '<OpState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OpState>) istream)
  "Deserializes a message object of type '<OpState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OpState>)))
  "Returns string type for a message object of type '<OpState>"
  "ff_msgs/OpState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OpState)))
  "Returns string type for a message object of type 'OpState"
  "ff_msgs/OpState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OpState>)))
  "Returns md5sum for a message object of type '<OpState>"
  "11f7f3b40813a20f5f841918254d08c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OpState)))
  "Returns md5sum for a message object of type 'OpState"
  "11f7f3b40813a20f5f841918254d08c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OpState>)))
  "Returns full string definition for message of type '<OpState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Operating States, based off of the enumeration constants~%# in rapid::ext::astrobee::AgentState.~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 READY            = 0  # Freeflyer is ready to accept commands~%uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan~%uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command~%uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock~%# The freeflyer is either executing a fault response or there is a fault~%# occurring in the system that prevents the freeflyer from moving~%uint8 FAULT            = 4~%~%# Operating state~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OpState)))
  "Returns full string definition for message of type 'OpState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Operating States, based off of the enumeration constants~%# in rapid::ext::astrobee::AgentState.~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 READY            = 0  # Freeflyer is ready to accept commands~%uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan~%uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command~%uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock~%# The freeflyer is either executing a fault response or there is a fault~%# occurring in the system that prevents the freeflyer from moving~%uint8 FAULT            = 4~%~%# Operating state~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OpState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OpState>))
  "Converts a ROS message object to a list"
  (cl:list 'OpState
    (cl:cons ':state (state msg))
))
