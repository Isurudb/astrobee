; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CpuState.msg.html

(cl:defclass <CpuState> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil)
   (loads
    :reader loads
    :initarg :loads
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (frequency
    :reader frequency
    :initarg :frequency
    :type cl:integer
    :initform 0)
   (max_frequency
    :reader max_frequency
    :initarg :max_frequency
    :type cl:integer
    :initform 0))
)

(cl:defclass CpuState (<CpuState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CpuState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CpuState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CpuState> is deprecated: use ff_msgs-msg:CpuState instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <CpuState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:enabled-val is deprecated.  Use ff_msgs-msg:enabled instead.")
  (enabled m))

(cl:ensure-generic-function 'loads-val :lambda-list '(m))
(cl:defmethod loads-val ((m <CpuState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:loads-val is deprecated.  Use ff_msgs-msg:loads instead.")
  (loads m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <CpuState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:frequency-val is deprecated.  Use ff_msgs-msg:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'max_frequency-val :lambda-list '(m))
(cl:defmethod max_frequency-val ((m <CpuState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:max_frequency-val is deprecated.  Use ff_msgs-msg:max_frequency instead.")
  (max_frequency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CpuState>) ostream)
  "Serializes a message object of type '<CpuState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'loads))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'loads))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_frequency)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_frequency)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CpuState>) istream)
  "Deserializes a message object of type '<CpuState>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'loads) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'loads)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'max_frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'max_frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'max_frequency)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'max_frequency)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CpuState>)))
  "Returns string type for a message object of type '<CpuState>"
  "ff_msgs/CpuState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CpuState)))
  "Returns string type for a message object of type 'CpuState"
  "ff_msgs/CpuState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CpuState>)))
  "Returns md5sum for a message object of type '<CpuState>"
  "3ff6c0a8b78ea1e9087461c1e42b6ca2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CpuState)))
  "Returns md5sum for a message object of type 'CpuState"
  "3ff6c0a8b78ea1e9087461c1e42b6ca2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CpuState>)))
  "Returns full string definition for message of type '<CpuState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of a CPU.~%~%# Processor is on (enabled) or not~%bool enabled~%~%# The load (in percentages) of the cpu, for the fields given in~%# CpuStateStamped~%float32[] loads ~%~%# Current operating frequency in Hz~%uint32 frequency~%~%# Max frequency (may be less than theoretical limit of the processor)~%uint32 max_frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CpuState)))
  "Returns full string definition for message of type 'CpuState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of a CPU.~%~%# Processor is on (enabled) or not~%bool enabled~%~%# The load (in percentages) of the cpu, for the fields given in~%# CpuStateStamped~%float32[] loads ~%~%# Current operating frequency in Hz~%uint32 frequency~%~%# Max frequency (may be less than theoretical limit of the processor)~%uint32 max_frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CpuState>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'loads) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CpuState>))
  "Converts a ROS message object to a list"
  (cl:list 'CpuState
    (cl:cons ':enabled (enabled msg))
    (cl:cons ':loads (loads msg))
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':max_frequency (max_frequency msg))
))
