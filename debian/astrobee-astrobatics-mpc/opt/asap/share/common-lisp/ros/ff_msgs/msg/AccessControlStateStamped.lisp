; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude AccessControlStateStamped.msg.html

(cl:defclass <AccessControlStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (controller
    :reader controller
    :initarg :controller
    :type cl:string
    :initform "")
   (cookie
    :reader cookie
    :initarg :cookie
    :type cl:string
    :initform ""))
)

(cl:defclass AccessControlStateStamped (<AccessControlStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AccessControlStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AccessControlStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<AccessControlStateStamped> is deprecated: use ff_msgs-msg:AccessControlStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AccessControlStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'controller-val :lambda-list '(m))
(cl:defmethod controller-val ((m <AccessControlStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:controller-val is deprecated.  Use ff_msgs-msg:controller instead.")
  (controller m))

(cl:ensure-generic-function 'cookie-val :lambda-list '(m))
(cl:defmethod cookie-val ((m <AccessControlStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:cookie-val is deprecated.  Use ff_msgs-msg:cookie instead.")
  (cookie m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AccessControlStateStamped>) ostream)
  "Serializes a message object of type '<AccessControlStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'controller))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'controller))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cookie))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cookie))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AccessControlStateStamped>) istream)
  "Deserializes a message object of type '<AccessControlStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'controller) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'controller) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cookie) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cookie) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AccessControlStateStamped>)))
  "Returns string type for a message object of type '<AccessControlStateStamped>"
  "ff_msgs/AccessControlStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AccessControlStateStamped)))
  "Returns string type for a message object of type 'AccessControlStateStamped"
  "ff_msgs/AccessControlStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AccessControlStateStamped>)))
  "Returns md5sum for a message object of type '<AccessControlStateStamped>"
  "7f6db2b63dd70f4a02a19c88e8bcdc59")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AccessControlStateStamped)))
  "Returns md5sum for a message object of type 'AccessControlStateStamped"
  "7f6db2b63dd70f4a02a19c88e8bcdc59")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AccessControlStateStamped>)))
  "Returns full string definition for message of type '<AccessControlStateStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the access control node. Loosely based off of AccessControlState.idl ~%# from RAPID.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Name of operator in control of the robot~%string controller~%~%# String that the access control node generates upon receiving a request control~%# command. Cookie will be blank after a successful grab control command.~%string cookie~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AccessControlStateStamped)))
  "Returns full string definition for message of type 'AccessControlStateStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the access control node. Loosely based off of AccessControlState.idl ~%# from RAPID.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Name of operator in control of the robot~%string controller~%~%# String that the access control node generates upon receiving a request control~%# command. Cookie will be blank after a successful grab control command.~%string cookie~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AccessControlStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'controller))
     4 (cl:length (cl:slot-value msg 'cookie))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AccessControlStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'AccessControlStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':controller (controller msg))
    (cl:cons ':cookie (cookie msg))
))
