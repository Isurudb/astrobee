; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude MemState.msg.html

(cl:defclass <MemState> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (virt
    :reader virt
    :initarg :virt
    :type cl:integer
    :initform 0)
   (virt_peak
    :reader virt_peak
    :initarg :virt_peak
    :type cl:integer
    :initform 0)
   (ram
    :reader ram
    :initarg :ram
    :type cl:integer
    :initform 0)
   (ram_peak
    :reader ram_peak
    :initarg :ram_peak
    :type cl:integer
    :initform 0)
   (ram_perc
    :reader ram_perc
    :initarg :ram_perc
    :type cl:float
    :initform 0.0))
)

(cl:defclass MemState (<MemState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MemState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MemState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<MemState> is deprecated: use ff_msgs-msg:MemState instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <MemState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'virt-val :lambda-list '(m))
(cl:defmethod virt-val ((m <MemState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:virt-val is deprecated.  Use ff_msgs-msg:virt instead.")
  (virt m))

(cl:ensure-generic-function 'virt_peak-val :lambda-list '(m))
(cl:defmethod virt_peak-val ((m <MemState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:virt_peak-val is deprecated.  Use ff_msgs-msg:virt_peak instead.")
  (virt_peak m))

(cl:ensure-generic-function 'ram-val :lambda-list '(m))
(cl:defmethod ram-val ((m <MemState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ram-val is deprecated.  Use ff_msgs-msg:ram instead.")
  (ram m))

(cl:ensure-generic-function 'ram_peak-val :lambda-list '(m))
(cl:defmethod ram_peak-val ((m <MemState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ram_peak-val is deprecated.  Use ff_msgs-msg:ram_peak instead.")
  (ram_peak m))

(cl:ensure-generic-function 'ram_perc-val :lambda-list '(m))
(cl:defmethod ram_perc-val ((m <MemState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ram_perc-val is deprecated.  Use ff_msgs-msg:ram_perc instead.")
  (ram_perc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MemState>) ostream)
  "Serializes a message object of type '<MemState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram_peak)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram_peak)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ram_perc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MemState>) istream)
  "Deserializes a message object of type '<MemState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'virt_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'virt_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'virt_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'virt_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ram_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ram_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'ram_peak)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'ram_peak)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ram_perc) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MemState>)))
  "Returns string type for a message object of type '<MemState>"
  "ff_msgs/MemState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MemState)))
  "Returns string type for a message object of type 'MemState"
  "ff_msgs/MemState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MemState>)))
  "Returns md5sum for a message object of type '<MemState>"
  "35fa33fe0824ebd7cf296b7a82e3c26b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MemState)))
  "Returns md5sum for a message object of type 'MemState"
  "35fa33fe0824ebd7cf296b7a82e3c26b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MemState>)))
  "Returns full string definition for message of type '<MemState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the Memory.~%~%# The memory load of the node, for the fields given in~%string name~%# Virtual Memory~%uint32 virt        # virtual memeory used in Mb~%uint32 virt_peak   # peak virtual memory used in Mb~%~%# Physical Memory~%uint32 ram        # physical memory used in Mb~%uint32 ram_peak   # peak physical memory used in Mb~%float32 ram_perc  # percentage of physical memory in %~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MemState)))
  "Returns full string definition for message of type 'MemState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# State of the Memory.~%~%# The memory load of the node, for the fields given in~%string name~%# Virtual Memory~%uint32 virt        # virtual memeory used in Mb~%uint32 virt_peak   # peak virtual memory used in Mb~%~%# Physical Memory~%uint32 ram        # physical memory used in Mb~%uint32 ram_peak   # peak physical memory used in Mb~%float32 ram_perc  # percentage of physical memory in %~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MemState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MemState>))
  "Converts a ROS message object to a list"
  (cl:list 'MemState
    (cl:cons ':name (name msg))
    (cl:cons ':virt (virt msg))
    (cl:cons ':virt_peak (virt_peak msg))
    (cl:cons ':ram (ram msg))
    (cl:cons ':ram_peak (ram_peak msg))
    (cl:cons ':ram_perc (ram_perc msg))
))
