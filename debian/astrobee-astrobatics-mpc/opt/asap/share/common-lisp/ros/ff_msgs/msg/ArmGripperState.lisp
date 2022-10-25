; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmGripperState.msg.html

(cl:defclass <ArmGripperState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ArmGripperState (<ArmGripperState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmGripperState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmGripperState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmGripperState> is deprecated: use ff_msgs-msg:ArmGripperState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ArmGripperState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmGripperState>)))
    "Constants for message type '<ArmGripperState>"
  '((:UNKNOWN . 0)
    (:UNCALIBRATED . 1)
    (:CALIBRATING . 2)
    (:CLOSED . 3)
    (:OPEN . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmGripperState)))
    "Constants for message type 'ArmGripperState"
  '((:UNKNOWN . 0)
    (:UNCALIBRATED . 1)
    (:CALIBRATING . 2)
    (:CLOSED . 3)
    (:OPEN . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmGripperState>) ostream)
  "Serializes a message object of type '<ArmGripperState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmGripperState>) istream)
  "Deserializes a message object of type '<ArmGripperState>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmGripperState>)))
  "Returns string type for a message object of type '<ArmGripperState>"
  "ff_msgs/ArmGripperState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmGripperState)))
  "Returns string type for a message object of type 'ArmGripperState"
  "ff_msgs/ArmGripperState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmGripperState>)))
  "Returns md5sum for a message object of type '<ArmGripperState>"
  "3857276ebd0698497c3781f3fb94fb88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmGripperState)))
  "Returns md5sum for a message object of type 'ArmGripperState"
  "3857276ebd0698497c3781f3fb94fb88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmGripperState>)))
  "Returns full string definition for message of type '<ArmGripperState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Gripper State enum~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN      = 0~%uint8 UNCALIBRATED = 1~%uint8 CALIBRATING  = 2~%uint8 CLOSED       = 3~%uint8 OPEN         = 4~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmGripperState)))
  "Returns full string definition for message of type 'ArmGripperState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Gripper State enum~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN      = 0~%uint8 UNCALIBRATED = 1~%uint8 CALIBRATING  = 2~%uint8 CLOSED       = 3~%uint8 OPEN         = 4~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmGripperState>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmGripperState>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmGripperState
    (cl:cons ':state (state msg))
))
