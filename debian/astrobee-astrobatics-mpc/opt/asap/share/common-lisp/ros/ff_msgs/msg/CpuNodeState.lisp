; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CpuNodeState.msg.html

(cl:defclass <CpuNodeState> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (load
    :reader load
    :initarg :load
    :type cl:float
    :initform 0.0))
)

(cl:defclass CpuNodeState (<CpuNodeState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CpuNodeState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CpuNodeState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CpuNodeState> is deprecated: use ff_msgs-msg:CpuNodeState instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <CpuNodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'load-val :lambda-list '(m))
(cl:defmethod load-val ((m <CpuNodeState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:load-val is deprecated.  Use ff_msgs-msg:load instead.")
  (load m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CpuNodeState>) ostream)
  "Serializes a message object of type '<CpuNodeState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CpuNodeState>) istream)
  "Deserializes a message object of type '<CpuNodeState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'load) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CpuNodeState>)))
  "Returns string type for a message object of type '<CpuNodeState>"
  "ff_msgs/CpuNodeState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CpuNodeState)))
  "Returns string type for a message object of type 'CpuNodeState"
  "ff_msgs/CpuNodeState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CpuNodeState>)))
  "Returns md5sum for a message object of type '<CpuNodeState>"
  "f4103935420c1e1dc393e01ceae6eaff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CpuNodeState)))
  "Returns md5sum for a message object of type 'CpuNodeState"
  "f4103935420c1e1dc393e01ceae6eaff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CpuNodeState>)))
  "Returns full string definition for message of type '<CpuNodeState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of a CPU load for a node.~%~%# Node name~%string name~%~%# The load (in percentages) of the cpu~%float32 load ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CpuNodeState)))
  "Returns full string definition for message of type 'CpuNodeState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of a CPU load for a node.~%~%# Node name~%string name~%~%# The load (in percentages) of the cpu~%float32 load ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CpuNodeState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CpuNodeState>))
  "Converts a ROS message object to a list"
  (cl:list 'CpuNodeState
    (cl:cons ':name (name msg))
    (cl:cons ':load (load msg))
))
