; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CameraStatesStamped.msg.html

(cl:defclass <CameraStatesStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (states
    :reader states
    :initarg :states
    :type (cl:vector ff_msgs-msg:CameraState)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:CameraState :initial-element (cl:make-instance 'ff_msgs-msg:CameraState))))
)

(cl:defclass CameraStatesStamped (<CameraStatesStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraStatesStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraStatesStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CameraStatesStamped> is deprecated: use ff_msgs-msg:CameraStatesStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CameraStatesStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <CameraStatesStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:states-val is deprecated.  Use ff_msgs-msg:states instead.")
  (states m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraStatesStamped>) ostream)
  "Serializes a message object of type '<CameraStatesStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraStatesStamped>) istream)
  "Deserializes a message object of type '<CameraStatesStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:CameraState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraStatesStamped>)))
  "Returns string type for a message object of type '<CameraStatesStamped>"
  "ff_msgs/CameraStatesStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraStatesStamped)))
  "Returns string type for a message object of type 'CameraStatesStamped"
  "ff_msgs/CameraStatesStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraStatesStamped>)))
  "Returns md5sum for a message object of type '<CameraStatesStamped>"
  "8196b4a79fabfdcae8f271a13d0b1973")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraStatesStamped)))
  "Returns md5sum for a message object of type 'CameraStatesStamped"
  "8196b4a79fabfdcae8f271a13d0b1973")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraStatesStamped>)))
  "Returns full string definition for message of type '<CameraStatesStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An array of CameraState messages.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Camera states~%ff_msgs/CameraState[] states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/CameraState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# CameraState message, *MUST* be kept in sync with camera portion of~%# rapid::ext::astrobee::TelemetryState~%~%# nav_cam, dock_cam, etc.~%string camera_name~%~%# streaming to ground~%bool streaming~%~%# image width~%uint16 stream_width~%# image height~%uint16 stream_height~%# Rate in Hz~%float32 stream_rate~%~%# recording to disk~%bool recording~%~%# image width~%uint16 record_width~%# image height~%uint16 record_height~%# Rate in Hz~%float32 record_rate~%~%# only for sci cam~%float32 bandwidth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraStatesStamped)))
  "Returns full string definition for message of type 'CameraStatesStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An array of CameraState messages.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Camera states~%ff_msgs/CameraState[] states~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/CameraState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# CameraState message, *MUST* be kept in sync with camera portion of~%# rapid::ext::astrobee::TelemetryState~%~%# nav_cam, dock_cam, etc.~%string camera_name~%~%# streaming to ground~%bool streaming~%~%# image width~%uint16 stream_width~%# image height~%uint16 stream_height~%# Rate in Hz~%float32 stream_rate~%~%# recording to disk~%bool recording~%~%# image width~%uint16 record_width~%# image height~%uint16 record_height~%# Rate in Hz~%float32 record_rate~%~%# only for sci cam~%float32 bandwidth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraStatesStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraStatesStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraStatesStamped
    (cl:cons ':header (header msg))
    (cl:cons ':states (states msg))
))
