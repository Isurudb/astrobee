; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude FaultData.msg.html

(cl:defclass <FaultData> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:string
    :initform "")
   (data_type
    :reader data_type
    :initarg :data_type
    :type cl:fixnum
    :initform 0)
   (f
    :reader f
    :initarg :f
    :type cl:float
    :initform 0.0)
   (i
    :reader i
    :initarg :i
    :type cl:integer
    :initform 0)
   (s
    :reader s
    :initarg :s
    :type cl:string
    :initform ""))
)

(cl:defclass FaultData (<FaultData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaultData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaultData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<FaultData> is deprecated: use ff_msgs-msg:FaultData instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <FaultData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:key-val is deprecated.  Use ff_msgs-msg:key instead.")
  (key m))

(cl:ensure-generic-function 'data_type-val :lambda-list '(m))
(cl:defmethod data_type-val ((m <FaultData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:data_type-val is deprecated.  Use ff_msgs-msg:data_type instead.")
  (data_type m))

(cl:ensure-generic-function 'f-val :lambda-list '(m))
(cl:defmethod f-val ((m <FaultData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:f-val is deprecated.  Use ff_msgs-msg:f instead.")
  (f m))

(cl:ensure-generic-function 'i-val :lambda-list '(m))
(cl:defmethod i-val ((m <FaultData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:i-val is deprecated.  Use ff_msgs-msg:i instead.")
  (i m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <FaultData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:s-val is deprecated.  Use ff_msgs-msg:s instead.")
  (s m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FaultData>)))
    "Constants for message type '<FaultData>"
  '((:DATA_TYPE_FLOAT . 0)
    (:DATA_TYPE_INT . 1)
    (:DATA_TYPE_STRING . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FaultData)))
    "Constants for message type 'FaultData"
  '((:DATA_TYPE_FLOAT . 0)
    (:DATA_TYPE_INT . 1)
    (:DATA_TYPE_STRING . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaultData>) ostream)
  "Serializes a message object of type '<FaultData>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'key))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'key))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_type)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'f))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'i)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 's))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaultData>) istream)
  "Deserializes a message object of type '<FaultData>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'key) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_type)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'f) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 's) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 's) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaultData>)))
  "Returns string type for a message object of type '<FaultData>"
  "ff_msgs/FaultData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaultData)))
  "Returns string type for a message object of type 'FaultData"
  "ff_msgs/FaultData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaultData>)))
  "Returns md5sum for a message object of type '<FaultData>"
  "632c6de83aa53364cbd36514ffa5c853")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaultData)))
  "Returns md5sum for a message object of type 'FaultData"
  "632c6de83aa53364cbd36514ffa5c853")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaultData>)))
  "Returns full string definition for message of type '<FaultData>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%# ~%# Fault data messsage contains information of why the fault occurred~%~%uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float~%uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int~%uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string~%~%string key  # Specifies what the data in the msg is, can only be 32 chars long~%~%uint8 data_type   # Specifies the type of data in the message~%~%float32 f   # Value used for fault analysis, data_type must be 0~%int32 i     # Value used for fault analysis, data_type must be 1~%string s    # String used for fault analysis, data_type must be 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaultData)))
  "Returns full string definition for message of type 'FaultData"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%# ~%# Fault data messsage contains information of why the fault occurred~%~%uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float~%uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int~%uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string~%~%string key  # Specifies what the data in the msg is, can only be 32 chars long~%~%uint8 data_type   # Specifies the type of data in the message~%~%float32 f   # Value used for fault analysis, data_type must be 0~%int32 i     # Value used for fault analysis, data_type must be 1~%string s    # String used for fault analysis, data_type must be 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaultData>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'key))
     1
     4
     4
     4 (cl:length (cl:slot-value msg 's))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaultData>))
  "Converts a ROS message object to a list"
  (cl:list 'FaultData
    (cl:cons ':key (key msg))
    (cl:cons ':data_type (data_type msg))
    (cl:cons ':f (f msg))
    (cl:cons ':i (i msg))
    (cl:cons ':s (s msg))
))
