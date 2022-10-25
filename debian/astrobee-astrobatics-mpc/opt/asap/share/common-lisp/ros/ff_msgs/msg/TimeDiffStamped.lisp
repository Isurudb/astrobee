; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude TimeDiffStamped.msg.html

(cl:defclass <TimeDiffStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (time_diff_sec
    :reader time_diff_sec
    :initarg :time_diff_sec
    :type cl:float
    :initform 0.0))
)

(cl:defclass TimeDiffStamped (<TimeDiffStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimeDiffStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimeDiffStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<TimeDiffStamped> is deprecated: use ff_msgs-msg:TimeDiffStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TimeDiffStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'time_diff_sec-val :lambda-list '(m))
(cl:defmethod time_diff_sec-val ((m <TimeDiffStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:time_diff_sec-val is deprecated.  Use ff_msgs-msg:time_diff_sec instead.")
  (time_diff_sec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimeDiffStamped>) ostream)
  "Serializes a message object of type '<TimeDiffStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time_diff_sec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimeDiffStamped>) istream)
  "Deserializes a message object of type '<TimeDiffStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_diff_sec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimeDiffStamped>)))
  "Returns string type for a message object of type '<TimeDiffStamped>"
  "ff_msgs/TimeDiffStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimeDiffStamped)))
  "Returns string type for a message object of type 'TimeDiffStamped"
  "ff_msgs/TimeDiffStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimeDiffStamped>)))
  "Returns md5sum for a message object of type '<TimeDiffStamped>"
  "923869d9e60d4fadfc7104bd76be64ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimeDiffStamped)))
  "Returns md5sum for a message object of type 'TimeDiffStamped"
  "923869d9e60d4fadfc7104bd76be64ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimeDiffStamped>)))
  "Returns full string definition for message of type '<TimeDiffStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message to send time difference between the mlp and llp~%~%# Header with timestamp~%std_msgs/Header header~%~%# Time diff in seconds between llp and mlp~%float32 time_diff_sec~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimeDiffStamped)))
  "Returns full string definition for message of type 'TimeDiffStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message to send time difference between the mlp and llp~%~%# Header with timestamp~%std_msgs/Header header~%~%# Time diff in seconds between llp and mlp~%float32 time_diff_sec~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimeDiffStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimeDiffStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'TimeDiffStamped
    (cl:cons ':header (header msg))
    (cl:cons ':time_diff_sec (time_diff_sec msg))
))
