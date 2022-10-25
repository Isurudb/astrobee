; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude JointSampleStamped.msg.html

(cl:defclass <JointSampleStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (samples
    :reader samples
    :initarg :samples
    :type (cl:vector ff_msgs-msg:JointSample)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:JointSample :initial-element (cl:make-instance 'ff_msgs-msg:JointSample))))
)

(cl:defclass JointSampleStamped (<JointSampleStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointSampleStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointSampleStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<JointSampleStamped> is deprecated: use ff_msgs-msg:JointSampleStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JointSampleStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <JointSampleStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:samples-val is deprecated.  Use ff_msgs-msg:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointSampleStamped>) ostream)
  "Serializes a message object of type '<JointSampleStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'samples))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'samples))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointSampleStamped>) istream)
  "Deserializes a message object of type '<JointSampleStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'samples) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'samples)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:JointSample))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointSampleStamped>)))
  "Returns string type for a message object of type '<JointSampleStamped>"
  "ff_msgs/JointSampleStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointSampleStamped)))
  "Returns string type for a message object of type 'JointSampleStamped"
  "ff_msgs/JointSampleStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointSampleStamped>)))
  "Returns md5sum for a message object of type '<JointSampleStamped>"
  "ada8b7552d00a44b3992523c84bda644")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointSampleStamped)))
  "Returns md5sum for a message object of type 'JointSampleStamped"
  "ada8b7552d00a44b3992523c84bda644")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointSampleStamped>)))
  "Returns full string definition for message of type '<JointSampleStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An array of Joint sample messages.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Joint samples~%ff_msgs/JointSample[] samples~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/JointSample~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# JointSample message, based off of rapid::JointSample~%~%# Flag values for joint status. ~%# Joint is enabled~%uint8 JOINT_ENABLED    = 0      # Joint enabled~%uint8 JOINT_DISABLED   = 1      # Joint disabled~%~%~%# Angle position (in radians) of the joint~%float32 angle_pos~%~%# Angle velocity (in radians/sec) of the joint~%float32 angle_vel~%~%# Angle acceleration (in radians/sec^2) of the joint (not being used)~%float32 angle_acc~%~%# Current draw of joint motor~%float32 current~%~%# Torque sensed at the joint (not being used)~%float32 torque~%~%# Temperature of the joint (in Celsius)~%float32 temperature~%~%# Bit field representing the state of the joint~%uint16 status~%~%# Human-readable name~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointSampleStamped)))
  "Returns full string definition for message of type 'JointSampleStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An array of Joint sample messages.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Joint samples~%ff_msgs/JointSample[] samples~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/JointSample~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# JointSample message, based off of rapid::JointSample~%~%# Flag values for joint status. ~%# Joint is enabled~%uint8 JOINT_ENABLED    = 0      # Joint enabled~%uint8 JOINT_DISABLED   = 1      # Joint disabled~%~%~%# Angle position (in radians) of the joint~%float32 angle_pos~%~%# Angle velocity (in radians/sec) of the joint~%float32 angle_vel~%~%# Angle acceleration (in radians/sec^2) of the joint (not being used)~%float32 angle_acc~%~%# Current draw of joint motor~%float32 current~%~%# Torque sensed at the joint (not being used)~%float32 torque~%~%# Temperature of the joint (in Celsius)~%float32 temperature~%~%# Bit field representing the state of the joint~%uint16 status~%~%# Human-readable name~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointSampleStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointSampleStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'JointSampleStamped
    (cl:cons ':header (header msg))
    (cl:cons ':samples (samples msg))
))
