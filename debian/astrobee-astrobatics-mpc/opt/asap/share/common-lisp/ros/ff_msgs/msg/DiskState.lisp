; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DiskState.msg.html

(cl:defclass <DiskState> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type cl:string
    :initform "")
   (capacity
    :reader capacity
    :initarg :capacity
    :type cl:integer
    :initform 0)
   (used
    :reader used
    :initarg :used
    :type cl:integer
    :initform 0))
)

(cl:defclass DiskState (<DiskState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DiskState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DiskState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DiskState> is deprecated: use ff_msgs-msg:DiskState instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <DiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:path-val is deprecated.  Use ff_msgs-msg:path instead.")
  (path m))

(cl:ensure-generic-function 'capacity-val :lambda-list '(m))
(cl:defmethod capacity-val ((m <DiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:capacity-val is deprecated.  Use ff_msgs-msg:capacity instead.")
  (capacity m))

(cl:ensure-generic-function 'used-val :lambda-list '(m))
(cl:defmethod used-val ((m <DiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:used-val is deprecated.  Use ff_msgs-msg:used instead.")
  (used m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DiskState>) ostream)
  "Serializes a message object of type '<DiskState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'capacity)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'used)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DiskState>) istream)
  "Deserializes a message object of type '<DiskState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'capacity)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'used)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DiskState>)))
  "Returns string type for a message object of type '<DiskState>"
  "ff_msgs/DiskState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiskState)))
  "Returns string type for a message object of type 'DiskState"
  "ff_msgs/DiskState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DiskState>)))
  "Returns md5sum for a message object of type '<DiskState>"
  "d5dcd3cfca06139f3d404ba9fc6b35be")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DiskState)))
  "Returns md5sum for a message object of type 'DiskState"
  "d5dcd3cfca06139f3d404ba9fc6b35be")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DiskState>)))
  "Returns full string definition for message of type '<DiskState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the state of a filesystem within astrobee~%# Based off of DiskState from rapid::ext::astrobee~%~%string path       # The pathname of the file within the mounted filesystem~%uint64 capacity   # The size of the filesystem~%uint64 used       # The amount of the filesystem being used~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DiskState)))
  "Returns full string definition for message of type 'DiskState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the state of a filesystem within astrobee~%# Based off of DiskState from rapid::ext::astrobee~%~%string path       # The pathname of the file within the mounted filesystem~%uint64 capacity   # The size of the filesystem~%uint64 used       # The amount of the filesystem being used~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DiskState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'path))
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DiskState>))
  "Converts a ROS message object to a list"
  (cl:list 'DiskState
    (cl:cons ':path (path msg))
    (cl:cons ':capacity (capacity msg))
    (cl:cons ':used (used msg))
))
