; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude SaveSettings.msg.html

(cl:defclass <SaveSettings> (roslisp-msg-protocol:ros-message)
  ((topic_name
    :reader topic_name
    :initarg :topic_name
    :type cl:string
    :initform "")
   (downlinkOption
    :reader downlinkOption
    :initarg :downlinkOption
    :type cl:fixnum
    :initform 0)
   (frequency
    :reader frequency
    :initarg :frequency
    :type cl:float
    :initform 0.0))
)

(cl:defclass SaveSettings (<SaveSettings>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveSettings>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveSettings)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<SaveSettings> is deprecated: use ff_msgs-msg:SaveSettings instead.")))

(cl:ensure-generic-function 'topic_name-val :lambda-list '(m))
(cl:defmethod topic_name-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:topic_name-val is deprecated.  Use ff_msgs-msg:topic_name instead.")
  (topic_name m))

(cl:ensure-generic-function 'downlinkOption-val :lambda-list '(m))
(cl:defmethod downlinkOption-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:downlinkOption-val is deprecated.  Use ff_msgs-msg:downlinkOption instead.")
  (downlinkOption m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <SaveSettings>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:frequency-val is deprecated.  Use ff_msgs-msg:frequency instead.")
  (frequency m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SaveSettings>)))
    "Constants for message type '<SaveSettings>"
  '((:IMMEDIATE . 0)
    (:DELAYED . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SaveSettings)))
    "Constants for message type 'SaveSettings"
  '((:IMMEDIATE . 0)
    (:DELAYED . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveSettings>) ostream)
  "Serializes a message object of type '<SaveSettings>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'topic_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'topic_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'downlinkOption)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveSettings>) istream)
  "Deserializes a message object of type '<SaveSettings>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'topic_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'topic_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'downlinkOption)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frequency) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveSettings>)))
  "Returns string type for a message object of type '<SaveSettings>"
  "ff_msgs/SaveSettings")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveSettings)))
  "Returns string type for a message object of type 'SaveSettings"
  "ff_msgs/SaveSettings")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveSettings>)))
  "Returns md5sum for a message object of type '<SaveSettings>"
  "87300656673b0987cb5b546a70fa697f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveSettings)))
  "Returns md5sum for a message object of type 'SaveSettings"
  "87300656673b0987cb5b546a70fa697f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveSettings>)))
  "Returns full string definition for message of type '<SaveSettings>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# The save settings message contains information about the topics currently~%# being recorded.~%~%# Name of topic~%string topic_name~%~%# Topic saved to disk; upon docking it is downlinked~%uint8 IMMEDIATE   = 0~%~%# Topic saved to disk; upon docking it is transferred to ISS server for later~%# downlink~%uint8 DELAYED     = 1~%~%# Downlink option indicates if and when the data in the rostopic is downlinked~%uint8 downlinkOption~%~%# Times per second to save the data (Hz)~%float32 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveSettings)))
  "Returns full string definition for message of type 'SaveSettings"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# The save settings message contains information about the topics currently~%# being recorded.~%~%# Name of topic~%string topic_name~%~%# Topic saved to disk; upon docking it is downlinked~%uint8 IMMEDIATE   = 0~%~%# Topic saved to disk; upon docking it is transferred to ISS server for later~%# downlink~%uint8 DELAYED     = 1~%~%# Downlink option indicates if and when the data in the rostopic is downlinked~%uint8 downlinkOption~%~%# Times per second to save the data (Hz)~%float32 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveSettings>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'topic_name))
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveSettings>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveSettings
    (cl:cons ':topic_name (topic_name msg))
    (cl:cons ':downlinkOption (downlinkOption msg))
    (cl:cons ':frequency (frequency msg))
))
