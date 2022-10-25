; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude MemStateStamped.msg.html

(cl:defclass <MemStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (ram_total
    :reader ram_total
    :initarg :ram_total
    :type cl:integer
    :initform 0)
   (ram_used
    :reader ram_used
    :initarg :ram_used
    :type cl:integer
    :initform 0)
   (virt_total
    :reader virt_total
    :initarg :virt_total
    :type cl:integer
    :initform 0)
   (virt_used
    :reader virt_used
    :initarg :virt_used
    :type cl:integer
    :initform 0)
   (nodes
    :reader nodes
    :initarg :nodes
    :type (cl:vector ff_msgs-msg:MemState)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:MemState :initial-element (cl:make-instance 'ff_msgs-msg:MemState))))
)

(cl:defclass MemStateStamped (<MemStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MemStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MemStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<MemStateStamped> is deprecated: use ff_msgs-msg:MemStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'ram_total-val :lambda-list '(m))
(cl:defmethod ram_total-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ram_total-val is deprecated.  Use ff_msgs-msg:ram_total instead.")
  (ram_total m))

(cl:ensure-generic-function 'ram_used-val :lambda-list '(m))
(cl:defmethod ram_used-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ram_used-val is deprecated.  Use ff_msgs-msg:ram_used instead.")
  (ram_used m))

(cl:ensure-generic-function 'virt_total-val :lambda-list '(m))
(cl:defmethod virt_total-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:virt_total-val is deprecated.  Use ff_msgs-msg:virt_total instead.")
  (virt_total m))

(cl:ensure-generic-function 'virt_used-val :lambda-list '(m))
(cl:defmethod virt_used-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:virt_used-val is deprecated.  Use ff_msgs-msg:virt_used instead.")
  (virt_used m))

(cl:ensure-generic-function 'nodes-val :lambda-list '(m))
(cl:defmethod nodes-val ((m <MemStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:nodes-val is deprecated.  Use ff_msgs-msg:nodes instead.")
  (nodes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MemStateStamped>) ostream)
  "Serializes a message object of type '<MemStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt_total)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt_used)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt_used)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nodes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'nodes))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MemStateStamped>) istream)
  "Deserializes a message object of type '<MemStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt_total)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt_used)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt_used)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nodes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nodes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:MemState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MemStateStamped>)))
  "Returns string type for a message object of type '<MemStateStamped>"
  "ff_msgs/MemStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MemStateStamped)))
  "Returns string type for a message object of type 'MemStateStamped"
  "ff_msgs/MemStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MemStateStamped>)))
  "Returns md5sum for a message object of type '<MemStateStamped>"
  "331992e4d9de11a301b654de12ecbac9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MemStateStamped)))
  "Returns md5sum for a message object of type 'MemStateStamped"
  "331992e4d9de11a301b654de12ecbac9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MemStateStamped>)))
  "Returns full string definition for message of type '<MemStateStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Memory state message with timestamp.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Machine name (llp, hlp, mlp, etc)~%string name~%~%# Physical Memory (RAM)~%uint32 ram_total   # total physical memeory in system Mb~%uint32 ram_used    # totalphysical memeory used in Mb~%~%# Virtual Memory~%uint32 virt_total  # total virtual memeory in system in Mb~%uint32 virt_used   # total virtual memeory used in Mb~%~%# Individual nodes~%ff_msgs/MemState[] nodes~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/MemState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the Memory.~%~%# The memory load of the node, for the fields given in~%string name~%# Virtual Memory~%uint32 virt        # virtual memeory used in Mb~%uint32 virt_peak   # peak virtual memory used in Mb~%~%# Physical Memory~%uint32 ram        # physical memory used in Mb~%uint32 ram_peak   # peak physical memory used in Mb~%float32 ram_perc  # percentage of physical memory in %~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MemStateStamped)))
  "Returns full string definition for message of type 'MemStateStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Memory state message with timestamp.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Machine name (llp, hlp, mlp, etc)~%string name~%~%# Physical Memory (RAM)~%uint32 ram_total   # total physical memeory in system Mb~%uint32 ram_used    # totalphysical memeory used in Mb~%~%# Virtual Memory~%uint32 virt_total  # total virtual memeory in system in Mb~%uint32 virt_used   # total virtual memeory used in Mb~%~%# Individual nodes~%ff_msgs/MemState[] nodes~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/MemState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the Memory.~%~%# The memory load of the node, for the fields given in~%string name~%# Virtual Memory~%uint32 virt        # virtual memeory used in Mb~%uint32 virt_peak   # peak virtual memory used in Mb~%~%# Physical Memory~%uint32 ram        # physical memory used in Mb~%uint32 ram_peak   # peak physical memory used in Mb~%float32 ram_perc  # percentage of physical memory in %~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MemStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     4
     4
     4
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nodes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MemStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'MemStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':ram_total (ram_total msg))
    (cl:cons ':ram_used (ram_used msg))
    (cl:cons ':virt_total (virt_total msg))
    (cl:cons ':virt_used (virt_used msg))
    (cl:cons ':nodes (nodes msg))
))
