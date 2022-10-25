; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude JointSample.msg.html

(cl:defclass <JointSample> (roslisp-msg-protocol:ros-message)
  ((angle_pos
    :reader angle_pos
    :initarg :angle_pos
    :type cl:float
    :initform 0.0)
   (angle_vel
    :reader angle_vel
    :initarg :angle_vel
    :type cl:float
    :initform 0.0)
   (angle_acc
    :reader angle_acc
    :initarg :angle_acc
    :type cl:float
    :initform 0.0)
   (current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (torque
    :reader torque
    :initarg :torque
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass JointSample (<JointSample>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointSample>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointSample)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<JointSample> is deprecated: use ff_msgs-msg:JointSample instead.")))

(cl:ensure-generic-function 'angle_pos-val :lambda-list '(m))
(cl:defmethod angle_pos-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:angle_pos-val is deprecated.  Use ff_msgs-msg:angle_pos instead.")
  (angle_pos m))

(cl:ensure-generic-function 'angle_vel-val :lambda-list '(m))
(cl:defmethod angle_vel-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:angle_vel-val is deprecated.  Use ff_msgs-msg:angle_vel instead.")
  (angle_vel m))

(cl:ensure-generic-function 'angle_acc-val :lambda-list '(m))
(cl:defmethod angle_acc-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:angle_acc-val is deprecated.  Use ff_msgs-msg:angle_acc instead.")
  (angle_acc m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:current-val is deprecated.  Use ff_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:torque-val is deprecated.  Use ff_msgs-msg:torque instead.")
  (torque m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:temperature-val is deprecated.  Use ff_msgs-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:status-val is deprecated.  Use ff_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <JointSample>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JointSample>)))
    "Constants for message type '<JointSample>"
  '((:JOINT_ENABLED . 0)
    (:JOINT_DISABLED . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JointSample)))
    "Constants for message type 'JointSample"
  '((:JOINT_ENABLED . 0)
    (:JOINT_DISABLED . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointSample>) ostream)
  "Serializes a message object of type '<JointSample>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointSample>) istream)
  "Deserializes a message object of type '<JointSample>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_pos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_acc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'torque) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointSample>)))
  "Returns string type for a message object of type '<JointSample>"
  "ff_msgs/JointSample")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointSample)))
  "Returns string type for a message object of type 'JointSample"
  "ff_msgs/JointSample")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointSample>)))
  "Returns md5sum for a message object of type '<JointSample>"
  "fe238686c8b329629bd0aa9499404e2e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointSample)))
  "Returns md5sum for a message object of type 'JointSample"
  "fe238686c8b329629bd0aa9499404e2e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointSample>)))
  "Returns full string definition for message of type '<JointSample>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# JointSample message, based off of rapid::JointSample~%~%# Flag values for joint status. ~%# Joint is enabled~%uint8 JOINT_ENABLED    = 0      # Joint enabled~%uint8 JOINT_DISABLED   = 1      # Joint disabled~%~%~%# Angle position (in radians) of the joint~%float32 angle_pos~%~%# Angle velocity (in radians/sec) of the joint~%float32 angle_vel~%~%# Angle acceleration (in radians/sec^2) of the joint (not being used)~%float32 angle_acc~%~%# Current draw of joint motor~%float32 current~%~%# Torque sensed at the joint (not being used)~%float32 torque~%~%# Temperature of the joint (in Celsius)~%float32 temperature~%~%# Bit field representing the state of the joint~%uint16 status~%~%# Human-readable name~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointSample)))
  "Returns full string definition for message of type 'JointSample"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# JointSample message, based off of rapid::JointSample~%~%# Flag values for joint status. ~%# Joint is enabled~%uint8 JOINT_ENABLED    = 0      # Joint enabled~%uint8 JOINT_DISABLED   = 1      # Joint disabled~%~%~%# Angle position (in radians) of the joint~%float32 angle_pos~%~%# Angle velocity (in radians/sec) of the joint~%float32 angle_vel~%~%# Angle acceleration (in radians/sec^2) of the joint (not being used)~%float32 angle_acc~%~%# Current draw of joint motor~%float32 current~%~%# Torque sensed at the joint (not being used)~%float32 torque~%~%# Temperature of the joint (in Celsius)~%float32 temperature~%~%# Bit field representing the state of the joint~%uint16 status~%~%# Human-readable name~%string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointSample>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     2
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointSample>))
  "Converts a ROS message object to a list"
  (cl:list 'JointSample
    (cl:cons ':angle_pos (angle_pos msg))
    (cl:cons ':angle_vel (angle_vel msg))
    (cl:cons ':angle_acc (angle_acc msg))
    (cl:cons ':current (current msg))
    (cl:cons ':torque (torque msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':status (status msg))
    (cl:cons ':name (name msg))
))
