; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude AckStatus.msg.html

(cl:defclass <AckStatus> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AckStatus (<AckStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<AckStatus> is deprecated: use ff_msgs-msg:AckStatus instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <AckStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:status-val is deprecated.  Use ff_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AckStatus>)))
    "Constants for message type '<AckStatus>"
  '((:QUEUED . 0)
    (:EXECUTING . 1)
    (:REQUEUED . 2)
    (:COMPLETED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AckStatus)))
    "Constants for message type 'AckStatus"
  '((:QUEUED . 0)
    (:EXECUTING . 1)
    (:REQUEUED . 2)
    (:COMPLETED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckStatus>) ostream)
  "Serializes a message object of type '<AckStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckStatus>) istream)
  "Deserializes a message object of type '<AckStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckStatus>)))
  "Returns string type for a message object of type '<AckStatus>"
  "ff_msgs/AckStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckStatus)))
  "Returns string type for a message object of type 'AckStatus"
  "ff_msgs/AckStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckStatus>)))
  "Returns md5sum for a message object of type '<AckStatus>"
  "3a9fd9fcbdad61abda5990f950ce8aee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckStatus)))
  "Returns md5sum for a message object of type 'AckStatus"
  "3a9fd9fcbdad61abda5990f950ce8aee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckStatus>)))
  "Returns full string definition for message of type '<AckStatus>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Command status. Based off AckStatus in RAPID DDS~%~%uint8 QUEUED = 0      # Command is in a queue and waiting to be executed~%uint8 EXECUTING = 1   # Command is being executed~%uint8 REQUEUED = 2    # Command is paused and waiting to be restarted ~%uint8 COMPLETED = 3   # Command is finished~%~%uint8 status          # Command status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckStatus)))
  "Returns full string definition for message of type 'AckStatus"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Command status. Based off AckStatus in RAPID DDS~%~%uint8 QUEUED = 0      # Command is in a queue and waiting to be executed~%uint8 EXECUTING = 1   # Command is being executed~%uint8 REQUEUED = 2    # Command is paused and waiting to be restarted ~%uint8 COMPLETED = 3   # Command is finished~%~%uint8 status          # Command status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckStatus>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'AckStatus
    (cl:cons ':status (status msg))
))
