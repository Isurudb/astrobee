; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CompressedFile.msg.html

(cl:defclass <CompressedFile> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (file
    :reader file
    :initarg :file
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass CompressedFile (<CompressedFile>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CompressedFile>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CompressedFile)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CompressedFile> is deprecated: use ff_msgs-msg:CompressedFile instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CompressedFile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <CompressedFile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:id-val is deprecated.  Use ff_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <CompressedFile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:type-val is deprecated.  Use ff_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <CompressedFile>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:file-val is deprecated.  Use ff_msgs-msg:file instead.")
  (file m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CompressedFile>)))
    "Constants for message type '<CompressedFile>"
  '((:TYPE_NONE . 0)
    (:TYPE_DEFLATE . 1)
    (:TYPE_BZ2 . 2)
    (:TYPE_GZ . 3)
    (:TYPE_ZIP . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CompressedFile)))
    "Constants for message type 'CompressedFile"
  '((:TYPE_NONE . 0)
    (:TYPE_DEFLATE . 1)
    (:TYPE_BZ2 . 2)
    (:TYPE_GZ . 3)
    (:TYPE_ZIP . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CompressedFile>) ostream)
  "Serializes a message object of type '<CompressedFile>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CompressedFile>) istream)
  "Deserializes a message object of type '<CompressedFile>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'file) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'file)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CompressedFile>)))
  "Returns string type for a message object of type '<CompressedFile>"
  "ff_msgs/CompressedFile")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CompressedFile)))
  "Returns string type for a message object of type 'CompressedFile"
  "ff_msgs/CompressedFile")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CompressedFile>)))
  "Returns md5sum for a message object of type '<CompressedFile>"
  "2566df54a1dc0c50a1ff95f06b4382df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CompressedFile)))
  "Returns md5sum for a message object of type 'CompressedFile"
  "2566df54a1dc0c50a1ff95f06b4382df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CompressedFile>)))
  "Returns full string definition for message of type '<CompressedFile>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A compressed file that represents a CompressedFile from~%# DDS. Used to send a compressed files to the executive.~%# DDS is constrained to a 128k chunk of data, we are only~%# limited by our imaginations (and the size of an unsigned~%# 32-bit integer)~%~%# Header ~%std_msgs/Header header~%~%# Unique file identification~%int32 id~%~%uint8 TYPE_NONE = 0~%uint8 TYPE_DEFLATE = 1~%uint8 TYPE_BZ2 = 2~%uint8 TYPE_GZ = 3~%uint8 TYPE_ZIP = 4~%~%# Type of compression~%uint8 type~%~%# File contents~%uint8[] file~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CompressedFile)))
  "Returns full string definition for message of type 'CompressedFile"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A compressed file that represents a CompressedFile from~%# DDS. Used to send a compressed files to the executive.~%# DDS is constrained to a 128k chunk of data, we are only~%# limited by our imaginations (and the size of an unsigned~%# 32-bit integer)~%~%# Header ~%std_msgs/Header header~%~%# Unique file identification~%int32 id~%~%uint8 TYPE_NONE = 0~%uint8 TYPE_DEFLATE = 1~%uint8 TYPE_BZ2 = 2~%uint8 TYPE_GZ = 3~%uint8 TYPE_ZIP = 4~%~%# Type of compression~%uint8 type~%~%# File contents~%uint8[] file~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CompressedFile>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'file) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CompressedFile>))
  "Converts a ROS message object to a list"
  (cl:list 'CompressedFile
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':type (type msg))
    (cl:cons ':file (file msg))
))
