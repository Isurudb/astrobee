; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CommandArg.msg.html

(cl:defclass <CommandArg> (roslisp-msg-protocol:ros-message)
  ((data_type
    :reader data_type
    :initarg :data_type
    :type cl:fixnum
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:boolean
    :initform cl:nil)
   (d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0)
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
   (ll
    :reader ll
    :initarg :ll
    :type cl:integer
    :initform 0)
   (s
    :reader s
    :initarg :s
    :type cl:string
    :initform "")
   (vec3d
    :reader vec3d
    :initarg :vec3d
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (mat33f
    :reader mat33f
    :initarg :mat33f
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CommandArg (<CommandArg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandArg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandArg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CommandArg> is deprecated: use ff_msgs-msg:CommandArg instead.")))

(cl:ensure-generic-function 'data_type-val :lambda-list '(m))
(cl:defmethod data_type-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:data_type-val is deprecated.  Use ff_msgs-msg:data_type instead.")
  (data_type m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:b-val is deprecated.  Use ff_msgs-msg:b instead.")
  (b m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:d-val is deprecated.  Use ff_msgs-msg:d instead.")
  (d m))

(cl:ensure-generic-function 'f-val :lambda-list '(m))
(cl:defmethod f-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:f-val is deprecated.  Use ff_msgs-msg:f instead.")
  (f m))

(cl:ensure-generic-function 'i-val :lambda-list '(m))
(cl:defmethod i-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:i-val is deprecated.  Use ff_msgs-msg:i instead.")
  (i m))

(cl:ensure-generic-function 'll-val :lambda-list '(m))
(cl:defmethod ll-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ll-val is deprecated.  Use ff_msgs-msg:ll instead.")
  (ll m))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:s-val is deprecated.  Use ff_msgs-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'vec3d-val :lambda-list '(m))
(cl:defmethod vec3d-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:vec3d-val is deprecated.  Use ff_msgs-msg:vec3d instead.")
  (vec3d m))

(cl:ensure-generic-function 'mat33f-val :lambda-list '(m))
(cl:defmethod mat33f-val ((m <CommandArg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:mat33f-val is deprecated.  Use ff_msgs-msg:mat33f instead.")
  (mat33f m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CommandArg>)))
    "Constants for message type '<CommandArg>"
  '((:DATA_TYPE_BOOL . 0)
    (:DATA_TYPE_DOUBLE . 1)
    (:DATA_TYPE_FLOAT . 2)
    (:DATA_TYPE_INT . 3)
    (:DATA_TYPE_LONGLONG . 4)
    (:DATA_TYPE_STRING . 5)
    (:DATA_TYPE_VEC3D . 6)
    (:DATA_TYPE_MAT33F . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CommandArg)))
    "Constants for message type 'CommandArg"
  '((:DATA_TYPE_BOOL . 0)
    (:DATA_TYPE_DOUBLE . 1)
    (:DATA_TYPE_FLOAT . 2)
    (:DATA_TYPE_INT . 3)
    (:DATA_TYPE_LONGLONG . 4)
    (:DATA_TYPE_STRING . 5)
    (:DATA_TYPE_VEC3D . 6)
    (:DATA_TYPE_MAT33F . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandArg>) ostream)
  "Serializes a message object of type '<CommandArg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_type)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
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
  (cl:let* ((signed (cl:slot-value msg 'll)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 's))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'vec3d))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'mat33f))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandArg>) istream)
  "Deserializes a message object of type '<CommandArg>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data_type)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'b) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-double-float-bits bits)))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'll) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 's) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 's) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:setf (cl:slot-value msg 'vec3d) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'vec3d)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'mat33f) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'mat33f)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandArg>)))
  "Returns string type for a message object of type '<CommandArg>"
  "ff_msgs/CommandArg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandArg)))
  "Returns string type for a message object of type 'CommandArg"
  "ff_msgs/CommandArg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandArg>)))
  "Returns md5sum for a message object of type '<CommandArg>"
  "c64f399f685551792b2e185eb2878830")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandArg)))
  "Returns md5sum for a message object of type 'CommandArg"
  "c64f399f685551792b2e185eb2878830")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandArg>)))
  "Returns full string definition for message of type '<CommandArg>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An argument to a command sent through RAPID~%#~%# Note that this is approximating a union in DDS. However, this is an~%# inefficient union, and thus each instance will take up at least 89 bytes.~%# However, even with the maximum of 16 arguments to a command, we only have~%# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.~%~%uint8 DATA_TYPE_BOOL     = 0~%uint8 DATA_TYPE_DOUBLE   = 1~%uint8 DATA_TYPE_FLOAT    = 2~%uint8 DATA_TYPE_INT      = 3~%uint8 DATA_TYPE_LONGLONG = 4~%uint8 DATA_TYPE_STRING   = 5~%uint8 DATA_TYPE_VEC3d    = 6~%uint8 DATA_TYPE_MAT33f   = 7~%~%uint8 data_type~%~%bool b~%float64 d~%float32 f~%int32 i~%int64 ll~%string s~%float64[3] vec3d~%float32[9] mat33f~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandArg)))
  "Returns full string definition for message of type 'CommandArg"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An argument to a command sent through RAPID~%#~%# Note that this is approximating a union in DDS. However, this is an~%# inefficient union, and thus each instance will take up at least 89 bytes.~%# However, even with the maximum of 16 arguments to a command, we only have~%# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.~%~%uint8 DATA_TYPE_BOOL     = 0~%uint8 DATA_TYPE_DOUBLE   = 1~%uint8 DATA_TYPE_FLOAT    = 2~%uint8 DATA_TYPE_INT      = 3~%uint8 DATA_TYPE_LONGLONG = 4~%uint8 DATA_TYPE_STRING   = 5~%uint8 DATA_TYPE_VEC3d    = 6~%uint8 DATA_TYPE_MAT33f   = 7~%~%uint8 data_type~%~%bool b~%float64 d~%float32 f~%int32 i~%int64 ll~%string s~%float64[3] vec3d~%float32[9] mat33f~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandArg>))
  (cl:+ 0
     1
     1
     8
     4
     4
     8
     4 (cl:length (cl:slot-value msg 's))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'vec3d) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mat33f) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandArg>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandArg
    (cl:cons ':data_type (data_type msg))
    (cl:cons ':b (b msg))
    (cl:cons ':d (d msg))
    (cl:cons ':f (f msg))
    (cl:cons ':i (i msg))
    (cl:cons ':ll (ll msg))
    (cl:cons ':s (s msg))
    (cl:cons ':vec3d (vec3d msg))
    (cl:cons ':mat33f (mat33f msg))
))
