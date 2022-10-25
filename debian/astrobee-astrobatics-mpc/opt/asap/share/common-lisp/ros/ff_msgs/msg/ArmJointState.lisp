; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmJointState.msg.html

(cl:defclass <ArmJointState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmJointState (<ArmJointState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmJointState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmJointState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmJointState> is deprecated: use ff_msgs-msg:ArmJointState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ArmJointState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmJointState>)))
    "Constants for message type '<ArmJointState>"
  '((:UNKNOWN . 0)
    (:STOWED . 1)
    (:DEPLOYING . 2)
    (:STOPPED . 3)
    (:MOVING . 4)
    (:STOWING . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmJointState)))
    "Constants for message type 'ArmJointState"
  '((:UNKNOWN . 0)
    (:STOWED . 1)
    (:DEPLOYING . 2)
    (:STOPPED . 3)
    (:MOVING . 4)
    (:STOWING . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmJointState>) ostream)
  "Serializes a message object of type '<ArmJointState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmJointState>) istream)
  "Deserializes a message object of type '<ArmJointState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmJointState>)))
  "Returns string type for a message object of type '<ArmJointState>"
  "ff_msgs/ArmJointState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmJointState)))
  "Returns string type for a message object of type 'ArmJointState"
  "ff_msgs/ArmJointState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmJointState>)))
  "Returns md5sum for a message object of type '<ArmJointState>"
  "18fc27f61231440dbfaa96855881bf89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmJointState)))
  "Returns md5sum for a message object of type 'ArmJointState"
  "18fc27f61231440dbfaa96855881bf89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmJointState>)))
  "Returns full string definition for message of type '<ArmJointState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Joint State enum.~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN   = 0~%uint8 STOWED    = 1~%uint8 DEPLOYING = 2~%uint8 STOPPED   = 3~%uint8 MOVING    = 4~%uint8 STOWING   = 5~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmJointState)))
  "Returns full string definition for message of type 'ArmJointState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Joint State enum.~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN   = 0~%uint8 STOWED    = 1~%uint8 DEPLOYING = 2~%uint8 STOPPED   = 3~%uint8 MOVING    = 4~%uint8 STOWING   = 5~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmJointState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmJointState>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmJointState
    (cl:cons ':state (state msg))
))
