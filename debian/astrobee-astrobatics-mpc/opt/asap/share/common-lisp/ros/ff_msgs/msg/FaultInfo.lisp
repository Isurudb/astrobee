; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude FaultInfo.msg.html

(cl:defclass <FaultInfo> (roslisp-msg-protocol:ros-message)
  ((subsystem
    :reader subsystem
    :initarg :subsystem
    :type cl:fixnum
    :initform 0)
   (node
    :reader node
    :initarg :node
    :type cl:fixnum
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (warning
    :reader warning
    :initarg :warning
    :type cl:boolean
    :initform cl:nil)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass FaultInfo (<FaultInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaultInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaultInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<FaultInfo> is deprecated: use ff_msgs-msg:FaultInfo instead.")))

(cl:ensure-generic-function 'subsystem-val :lambda-list '(m))
(cl:defmethod subsystem-val ((m <FaultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:subsystem-val is deprecated.  Use ff_msgs-msg:subsystem instead.")
  (subsystem m))

(cl:ensure-generic-function 'node-val :lambda-list '(m))
(cl:defmethod node-val ((m <FaultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:node-val is deprecated.  Use ff_msgs-msg:node instead.")
  (node m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <FaultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:id-val is deprecated.  Use ff_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'warning-val :lambda-list '(m))
(cl:defmethod warning-val ((m <FaultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:warning-val is deprecated.  Use ff_msgs-msg:warning instead.")
  (warning m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <FaultInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:description-val is deprecated.  Use ff_msgs-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaultInfo>) ostream)
  "Serializes a message object of type '<FaultInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subsystem)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subsystem)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'warning) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaultInfo>) istream)
  "Deserializes a message object of type '<FaultInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'subsystem)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'subsystem)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'node)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'node)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'warning) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaultInfo>)))
  "Returns string type for a message object of type '<FaultInfo>"
  "ff_msgs/FaultInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaultInfo)))
  "Returns string type for a message object of type 'FaultInfo"
  "ff_msgs/FaultInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaultInfo>)))
  "Returns md5sum for a message object of type '<FaultInfo>"
  "1f6014a9106a0f40b77f475f6f9592fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaultInfo)))
  "Returns md5sum for a message object of type 'FaultInfo"
  "1f6014a9106a0f40b77f475f6f9592fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaultInfo>)))
  "Returns full string definition for message of type '<FaultInfo>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault info message is used in the fault config message to contain all the ~%# information GDS needs to know about a fault~%~%uint16 subsystem    # index into subsystem names array found in fault config msg~%~%uint16 node         # index into node names array found in fault config msg~%~%uint32 id           # id corresponding to the fault~%~%bool warning        # whether the fault is a warning or not~%~%string description  # A short description of why the fault occurred~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaultInfo)))
  "Returns full string definition for message of type 'FaultInfo"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault info message is used in the fault config message to contain all the ~%# information GDS needs to know about a fault~%~%uint16 subsystem    # index into subsystem names array found in fault config msg~%~%uint16 node         # index into node names array found in fault config msg~%~%uint32 id           # id corresponding to the fault~%~%bool warning        # whether the fault is a warning or not~%~%string description  # A short description of why the fault occurred~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaultInfo>))
  (cl:+ 0
     2
     2
     4
     1
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaultInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'FaultInfo
    (cl:cons ':subsystem (subsystem msg))
    (cl:cons ':node (node msg))
    (cl:cons ':id (id msg))
    (cl:cons ':warning (warning msg))
    (cl:cons ':description (description msg))
))
