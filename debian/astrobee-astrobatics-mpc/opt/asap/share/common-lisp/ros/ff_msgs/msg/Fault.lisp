; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude Fault.msg.html

(cl:defclass <Fault> (roslisp-msg-protocol:ros-message)
  ((time_of_fault
    :reader time_of_fault
    :initarg :time_of_fault
    :type cl:real
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (msg
    :reader msg
    :initarg :msg
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type (cl:vector ff_msgs-msg:FaultData)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:FaultData :initial-element (cl:make-instance 'ff_msgs-msg:FaultData))))
)

(cl:defclass Fault (<Fault>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Fault>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Fault)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<Fault> is deprecated: use ff_msgs-msg:Fault instead.")))

(cl:ensure-generic-function 'time_of_fault-val :lambda-list '(m))
(cl:defmethod time_of_fault-val ((m <Fault>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:time_of_fault-val is deprecated.  Use ff_msgs-msg:time_of_fault instead.")
  (time_of_fault m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <Fault>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:id-val is deprecated.  Use ff_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <Fault>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:msg-val is deprecated.  Use ff_msgs-msg:msg instead.")
  (msg m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Fault>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:data-val is deprecated.  Use ff_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Fault>) ostream)
  "Serializes a message object of type '<Fault>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_of_fault)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_of_fault) (cl:floor (cl:slot-value msg 'time_of_fault)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'msg))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Fault>) istream)
  "Deserializes a message object of type '<Fault>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_of_fault) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'msg) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:FaultData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Fault>)))
  "Returns string type for a message object of type '<Fault>"
  "ff_msgs/Fault")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Fault)))
  "Returns string type for a message object of type 'Fault"
  "ff_msgs/Fault")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Fault>)))
  "Returns md5sum for a message object of type '<Fault>"
  "67f951d3568a6651a818ff487dcc2650")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Fault)))
  "Returns md5sum for a message object of type 'Fault"
  "67f951d3568a6651a818ff487dcc2650")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Fault>)))
  "Returns full string definition for message of type '<Fault>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault message is used to provide all the information about an occurring fault~%~%time time_of_fault        # Time when fault occurred~%~%uint32 id                 # id specifying fault~%~%string msg                # string specifying why the fault occurred~%~%ff_msgs/FaultData[] data  # Data used for fault analysis~%~%================================================================================~%MSG: ff_msgs/FaultData~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%# ~%# Fault data messsage contains information of why the fault occurred~%~%uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float~%uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int~%uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string~%~%string key  # Specifies what the data in the msg is, can only be 32 chars long~%~%uint8 data_type   # Specifies the type of data in the message~%~%float32 f   # Value used for fault analysis, data_type must be 0~%int32 i     # Value used for fault analysis, data_type must be 1~%string s    # String used for fault analysis, data_type must be 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Fault)))
  "Returns full string definition for message of type 'Fault"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Fault message is used to provide all the information about an occurring fault~%~%time time_of_fault        # Time when fault occurred~%~%uint32 id                 # id specifying fault~%~%string msg                # string specifying why the fault occurred~%~%ff_msgs/FaultData[] data  # Data used for fault analysis~%~%================================================================================~%MSG: ff_msgs/FaultData~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%# ~%# Fault data messsage contains information of why the fault occurred~%~%uint8 DATA_TYPE_FLOAT   = 0   # Data in this msg is of type float~%uint8 DATA_TYPE_INT     = 1   # Data in this msg is of type int~%uint8 DATA_TYPE_STRING  = 2   # Data in this msg is of type string~%~%string key  # Specifies what the data in the msg is, can only be 32 chars long~%~%uint8 data_type   # Specifies the type of data in the message~%~%float32 f   # Value used for fault analysis, data_type must be 0~%int32 i     # Value used for fault analysis, data_type must be 1~%string s    # String used for fault analysis, data_type must be 2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Fault>))
  (cl:+ 0
     8
     4
     4 (cl:length (cl:slot-value msg 'msg))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Fault>))
  "Converts a ROS message object to a list"
  (cl:list 'Fault
    (cl:cons ':time_of_fault (time_of_fault msg))
    (cl:cons ':id (id msg))
    (cl:cons ':msg (msg msg))
    (cl:cons ':data (data msg))
))
