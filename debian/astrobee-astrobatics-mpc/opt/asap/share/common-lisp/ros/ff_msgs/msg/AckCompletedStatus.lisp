; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude AckCompletedStatus.msg.html

(cl:defclass <AckCompletedStatus> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass AckCompletedStatus (<AckCompletedStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AckCompletedStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AckCompletedStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<AckCompletedStatus> is deprecated: use ff_msgs-msg:AckCompletedStatus instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <AckCompletedStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:status-val is deprecated.  Use ff_msgs-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AckCompletedStatus>)))
    "Constants for message type '<AckCompletedStatus>"
  '((:NOT . 0)
    (:OK . 1)
    (:BAD_SYNTAX . 2)
    (:EXEC_FAILED . 3)
    (:CANCELED . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AckCompletedStatus)))
    "Constants for message type 'AckCompletedStatus"
  '((:NOT . 0)
    (:OK . 1)
    (:BAD_SYNTAX . 2)
    (:EXEC_FAILED . 3)
    (:CANCELED . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AckCompletedStatus>) ostream)
  "Serializes a message object of type '<AckCompletedStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AckCompletedStatus>) istream)
  "Deserializes a message object of type '<AckCompletedStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AckCompletedStatus>)))
  "Returns string type for a message object of type '<AckCompletedStatus>"
  "ff_msgs/AckCompletedStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AckCompletedStatus)))
  "Returns string type for a message object of type 'AckCompletedStatus"
  "ff_msgs/AckCompletedStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AckCompletedStatus>)))
  "Returns md5sum for a message object of type '<AckCompletedStatus>"
  "06cb8fd59f3394d8700f8f7e1d9d75a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AckCompletedStatus)))
  "Returns md5sum for a message object of type 'AckCompletedStatus"
  "06cb8fd59f3394d8700f8f7e1d9d75a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AckCompletedStatus>)))
  "Returns full string definition for message of type '<AckCompletedStatus>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Completed command status. Based on AckCompletedStatus from RAPID DDS~%~%uint8 NOT = 0           # Command not completed~%uint8 OK = 1            # Command completed successfully~%uint8 BAD_SYNTAX = 2    # Command not recognized, bad parameters, etc.~%uint8 EXEC_FAILED = 3   # Command failed to execute~%uint8 CANCELED = 4      # Command was canceled by operator~%~%# Completed command status~%uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AckCompletedStatus)))
  "Returns full string definition for message of type 'AckCompletedStatus"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Completed command status. Based on AckCompletedStatus from RAPID DDS~%~%uint8 NOT = 0           # Command not completed~%uint8 OK = 1            # Command completed successfully~%uint8 BAD_SYNTAX = 2    # Command not recognized, bad parameters, etc.~%uint8 EXEC_FAILED = 3   # Command failed to execute~%uint8 CANCELED = 4      # Command was canceled by operator~%~%# Completed command status~%uint8 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AckCompletedStatus>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AckCompletedStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'AckCompletedStatus
    (cl:cons ':status (status msg))
))
