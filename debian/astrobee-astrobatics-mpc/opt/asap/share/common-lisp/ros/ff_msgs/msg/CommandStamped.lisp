; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CommandStamped.msg.html

(cl:defclass <CommandStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cmd_name
    :reader cmd_name
    :initarg :cmd_name
    :type cl:string
    :initform "")
   (cmd_id
    :reader cmd_id
    :initarg :cmd_id
    :type cl:string
    :initform "")
   (cmd_src
    :reader cmd_src
    :initarg :cmd_src
    :type cl:string
    :initform "")
   (cmd_origin
    :reader cmd_origin
    :initarg :cmd_origin
    :type cl:string
    :initform "")
   (subsys_name
    :reader subsys_name
    :initarg :subsys_name
    :type cl:string
    :initform "")
   (args
    :reader args
    :initarg :args
    :type (cl:vector ff_msgs-msg:CommandArg)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:CommandArg :initial-element (cl:make-instance 'ff_msgs-msg:CommandArg))))
)

(cl:defclass CommandStamped (<CommandStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CommandStamped> is deprecated: use ff_msgs-msg:CommandStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cmd_name-val :lambda-list '(m))
(cl:defmethod cmd_name-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:cmd_name-val is deprecated.  Use ff_msgs-msg:cmd_name instead.")
  (cmd_name m))

(cl:ensure-generic-function 'cmd_id-val :lambda-list '(m))
(cl:defmethod cmd_id-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:cmd_id-val is deprecated.  Use ff_msgs-msg:cmd_id instead.")
  (cmd_id m))

(cl:ensure-generic-function 'cmd_src-val :lambda-list '(m))
(cl:defmethod cmd_src-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:cmd_src-val is deprecated.  Use ff_msgs-msg:cmd_src instead.")
  (cmd_src m))

(cl:ensure-generic-function 'cmd_origin-val :lambda-list '(m))
(cl:defmethod cmd_origin-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:cmd_origin-val is deprecated.  Use ff_msgs-msg:cmd_origin instead.")
  (cmd_origin m))

(cl:ensure-generic-function 'subsys_name-val :lambda-list '(m))
(cl:defmethod subsys_name-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:subsys_name-val is deprecated.  Use ff_msgs-msg:subsys_name instead.")
  (subsys_name m))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <CommandStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:args-val is deprecated.  Use ff_msgs-msg:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandStamped>) ostream)
  "Serializes a message object of type '<CommandStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd_src))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd_src))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd_origin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd_origin))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'subsys_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'subsys_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'args))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandStamped>) istream)
  "Deserializes a message object of type '<CommandStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_src) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd_src) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd_origin) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd_origin) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'subsys_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'subsys_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'args) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'args)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:CommandArg))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandStamped>)))
  "Returns string type for a message object of type '<CommandStamped>"
  "ff_msgs/CommandStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandStamped)))
  "Returns string type for a message object of type 'CommandStamped"
  "ff_msgs/CommandStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandStamped>)))
  "Returns md5sum for a message object of type '<CommandStamped>"
  "ac01350894b1be9e3a7b4f390a14812d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandStamped)))
  "Returns md5sum for a message object of type 'CommandStamped"
  "ac01350894b1be9e3a7b4f390a14812d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandStamped>)))
  "Returns full string definition for message of type '<CommandStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Command Message, loosely based off of the RAPID Command.idl~%~%# Header with timestamp~%std_msgs/Header header~%~%# Command name~%string cmd_name~%~%# Unique identifier for command = unique counter + participant + timestamp~%string cmd_id~%~%# Source of the command, either operators, the system monitor or guest science~%string cmd_src~%~%# Origin of the command, ground for operators, astrobee for another astrobee,~%# sys_monitor for fault responses, and guest_science for guest science~%# commands~%string cmd_origin~%~%# Name of subsystem the command is going to (not used but kept to be consistant~%# with the command idl)~%string subsys_name~%~%# Arguments for the command ~%ff_msgs/CommandArg[] args~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/CommandArg~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An argument to a command sent through RAPID~%#~%# Note that this is approximating a union in DDS. However, this is an~%# inefficient union, and thus each instance will take up at least 89 bytes.~%# However, even with the maximum of 16 arguments to a command, we only have~%# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.~%~%uint8 DATA_TYPE_BOOL     = 0~%uint8 DATA_TYPE_DOUBLE   = 1~%uint8 DATA_TYPE_FLOAT    = 2~%uint8 DATA_TYPE_INT      = 3~%uint8 DATA_TYPE_LONGLONG = 4~%uint8 DATA_TYPE_STRING   = 5~%uint8 DATA_TYPE_VEC3d    = 6~%uint8 DATA_TYPE_MAT33f   = 7~%~%uint8 data_type~%~%bool b~%float64 d~%float32 f~%int32 i~%int64 ll~%string s~%float64[3] vec3d~%float32[9] mat33f~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandStamped)))
  "Returns full string definition for message of type 'CommandStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Command Message, loosely based off of the RAPID Command.idl~%~%# Header with timestamp~%std_msgs/Header header~%~%# Command name~%string cmd_name~%~%# Unique identifier for command = unique counter + participant + timestamp~%string cmd_id~%~%# Source of the command, either operators, the system monitor or guest science~%string cmd_src~%~%# Origin of the command, ground for operators, astrobee for another astrobee,~%# sys_monitor for fault responses, and guest_science for guest science~%# commands~%string cmd_origin~%~%# Name of subsystem the command is going to (not used but kept to be consistant~%# with the command idl)~%string subsys_name~%~%# Arguments for the command ~%ff_msgs/CommandArg[] args~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/CommandArg~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An argument to a command sent through RAPID~%#~%# Note that this is approximating a union in DDS. However, this is an~%# inefficient union, and thus each instance will take up at least 89 bytes.~%# However, even with the maximum of 16 arguments to a command, we only have~%# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.~%~%uint8 DATA_TYPE_BOOL     = 0~%uint8 DATA_TYPE_DOUBLE   = 1~%uint8 DATA_TYPE_FLOAT    = 2~%uint8 DATA_TYPE_INT      = 3~%uint8 DATA_TYPE_LONGLONG = 4~%uint8 DATA_TYPE_STRING   = 5~%uint8 DATA_TYPE_VEC3d    = 6~%uint8 DATA_TYPE_MAT33f   = 7~%~%uint8 data_type~%~%bool b~%float64 d~%float32 f~%int32 i~%int64 ll~%string s~%float64[3] vec3d~%float32[9] mat33f~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'cmd_name))
     4 (cl:length (cl:slot-value msg 'cmd_id))
     4 (cl:length (cl:slot-value msg 'cmd_src))
     4 (cl:length (cl:slot-value msg 'cmd_origin))
     4 (cl:length (cl:slot-value msg 'subsys_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandStamped
    (cl:cons ':header (header msg))
    (cl:cons ':cmd_name (cmd_name msg))
    (cl:cons ':cmd_id (cmd_id msg))
    (cl:cons ':cmd_src (cmd_src msg))
    (cl:cons ':cmd_origin (cmd_origin msg))
    (cl:cons ':subsys_name (subsys_name msg))
    (cl:cons ':args (args msg))
))
