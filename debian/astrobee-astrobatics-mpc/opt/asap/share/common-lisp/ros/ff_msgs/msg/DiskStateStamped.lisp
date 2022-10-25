; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DiskStateStamped.msg.html

(cl:defclass <DiskStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (processor_name
    :reader processor_name
    :initarg :processor_name
    :type cl:string
    :initform "")
   (disks
    :reader disks
    :initarg :disks
    :type (cl:vector ff_msgs-msg:DiskState)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:DiskState :initial-element (cl:make-instance 'ff_msgs-msg:DiskState))))
)

(cl:defclass DiskStateStamped (<DiskStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DiskStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DiskStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DiskStateStamped> is deprecated: use ff_msgs-msg:DiskStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DiskStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'processor_name-val :lambda-list '(m))
(cl:defmethod processor_name-val ((m <DiskStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:processor_name-val is deprecated.  Use ff_msgs-msg:processor_name instead.")
  (processor_name m))

(cl:ensure-generic-function 'disks-val :lambda-list '(m))
(cl:defmethod disks-val ((m <DiskStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:disks-val is deprecated.  Use ff_msgs-msg:disks instead.")
  (disks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DiskStateStamped>) ostream)
  "Serializes a message object of type '<DiskStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'processor_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'processor_name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'disks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'disks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DiskStateStamped>) istream)
  "Deserializes a message object of type '<DiskStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'processor_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'processor_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'disks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'disks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:DiskState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DiskStateStamped>)))
  "Returns string type for a message object of type '<DiskStateStamped>"
  "ff_msgs/DiskStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DiskStateStamped)))
  "Returns string type for a message object of type 'DiskStateStamped"
  "ff_msgs/DiskStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DiskStateStamped>)))
  "Returns md5sum for a message object of type '<DiskStateStamped>"
  "af262ed457fa1b453f85c47b9f5c607b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DiskStateStamped)))
  "Returns md5sum for a message object of type 'DiskStateStamped"
  "af262ed457fa1b453f85c47b9f5c607b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DiskStateStamped>)))
  "Returns full string definition for message of type '<DiskStateStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the state of a filesystem within astrobee~%# Based off of DiskState from rapid::ext::astrobee~%~%# Header with timestamp~%std_msgs/Header header~%~%string processor_name       # Processor name, either llp, mlp, or hlp~%~%# Information on the mounted filesystem on the processor~%ff_msgs/DiskState[] disks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/DiskState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the state of a filesystem within astrobee~%# Based off of DiskState from rapid::ext::astrobee~%~%string path       # The pathname of the file within the mounted filesystem~%uint64 capacity   # The size of the filesystem~%uint64 used       # The amount of the filesystem being used~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DiskStateStamped)))
  "Returns full string definition for message of type 'DiskStateStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the state of a filesystem within astrobee~%# Based off of DiskState from rapid::ext::astrobee~%~%# Header with timestamp~%std_msgs/Header header~%~%string processor_name       # Processor name, either llp, mlp, or hlp~%~%# Information on the mounted filesystem on the processor~%ff_msgs/DiskState[] disks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/DiskState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the state of a filesystem within astrobee~%# Based off of DiskState from rapid::ext::astrobee~%~%string path       # The pathname of the file within the mounted filesystem~%uint64 capacity   # The size of the filesystem~%uint64 used       # The amount of the filesystem being used~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DiskStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'processor_name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'disks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DiskStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'DiskStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':processor_name (processor_name msg))
    (cl:cons ':disks (disks msg))
))
