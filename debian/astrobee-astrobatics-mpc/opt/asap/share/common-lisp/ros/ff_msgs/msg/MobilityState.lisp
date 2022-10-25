; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude MobilityState.msg.html

(cl:defclass <MobilityState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (sub_state
    :reader sub_state
    :initarg :sub_state
    :type cl:integer
    :initform 0))
)

(cl:defclass MobilityState (<MobilityState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MobilityState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MobilityState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<MobilityState> is deprecated: use ff_msgs-msg:MobilityState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <MobilityState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'sub_state-val :lambda-list '(m))
(cl:defmethod sub_state-val ((m <MobilityState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:sub_state-val is deprecated.  Use ff_msgs-msg:sub_state instead.")
  (sub_state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MobilityState>)))
    "Constants for message type '<MobilityState>"
  '((:DRIFTING . 0)
    (:STOPPING . 1)
    (:FLYING . 2)
    (:DOCKING . 3)
    (:PERCHING . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MobilityState)))
    "Constants for message type 'MobilityState"
  '((:DRIFTING . 0)
    (:STOPPING . 1)
    (:FLYING . 2)
    (:DOCKING . 3)
    (:PERCHING . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MobilityState>) ostream)
  "Serializes a message object of type '<MobilityState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'sub_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MobilityState>) istream)
  "Deserializes a message object of type '<MobilityState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sub_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MobilityState>)))
  "Returns string type for a message object of type '<MobilityState>"
  "ff_msgs/MobilityState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MobilityState)))
  "Returns string type for a message object of type 'MobilityState"
  "ff_msgs/MobilityState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MobilityState>)))
  "Returns md5sum for a message object of type '<MobilityState>"
  "2c5f9184aace6b4675fe28aa28d9047e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MobilityState)))
  "Returns md5sum for a message object of type 'MobilityState"
  "2c5f9184aace6b4675fe28aa28d9047e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MobilityState>)))
  "Returns full string definition for message of type '<MobilityState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Mobility states, based off the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 DRIFTING        = 0   # Astrobee's propulsion is off~%uint8 STOPPING        = 1   # Astrobee is either stopping or stopped~%uint8 FLYING          = 2   # Astrobee is flying~%uint8 DOCKING         = 3   # Astrobee is either docking or undocking~%uint8 PERCHING        = 4   # Astrobee is either perching or unperching~%~%# Mobility state~%uint8 state~%~%# Specifies the progress of the action. For docking, this value can be N to -N~%# where N through 1 specifies the progress of a docking action, 0 is docked, and~%# -1 through -N specifies the process of an undocking action. For stopping, this~%# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means~%# the robot is stopped. For perching, this value can be N to -N where N through~%# 1 specifies the progress of a perching action, 0 is perched, and -1 through~%# -N specifies the process of an unperching action.~%int32 sub_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MobilityState)))
  "Returns full string definition for message of type 'MobilityState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Mobility states, based off the enumeration constants in~%# rapid::ext::astrobee::AgentState~%#~%# *MUST* be kept in sync with the DDS IDL file in astrobee_common~%~%uint8 DRIFTING        = 0   # Astrobee's propulsion is off~%uint8 STOPPING        = 1   # Astrobee is either stopping or stopped~%uint8 FLYING          = 2   # Astrobee is flying~%uint8 DOCKING         = 3   # Astrobee is either docking or undocking~%uint8 PERCHING        = 4   # Astrobee is either perching or unperching~%~%# Mobility state~%uint8 state~%~%# Specifies the progress of the action. For docking, this value can be N to -N~%# where N through 1 specifies the progress of a docking action, 0 is docked, and~%# -1 through -N specifies the process of an undocking action. For stopping, this~%# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means~%# the robot is stopped. For perching, this value can be N to -N where N through~%# 1 specifies the progress of a perching action, 0 is perched, and -1 through~%# -N specifies the process of an unperching action.~%int32 sub_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MobilityState>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MobilityState>))
  "Converts a ROS message object to a list"
  (cl:list 'MobilityState
    (cl:cons ':state (state msg))
    (cl:cons ':sub_state (sub_state msg))
))
