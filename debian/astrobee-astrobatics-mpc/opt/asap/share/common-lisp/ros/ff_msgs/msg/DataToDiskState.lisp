; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DataToDiskState.msg.html

(cl:defclass <DataToDiskState> (roslisp-msg-protocol:ros-message)
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
   (recording
    :reader recording
    :initarg :recording
    :type cl:boolean
    :initform cl:nil)
   (topic_save_settings
    :reader topic_save_settings
    :initarg :topic_save_settings
    :type (cl:vector ff_msgs-msg:SaveSettings)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:SaveSettings :initial-element (cl:make-instance 'ff_msgs-msg:SaveSettings))))
)

(cl:defclass DataToDiskState (<DataToDiskState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataToDiskState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataToDiskState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DataToDiskState> is deprecated: use ff_msgs-msg:DataToDiskState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DataToDiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <DataToDiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'recording-val :lambda-list '(m))
(cl:defmethod recording-val ((m <DataToDiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:recording-val is deprecated.  Use ff_msgs-msg:recording instead.")
  (recording m))

(cl:ensure-generic-function 'topic_save_settings-val :lambda-list '(m))
(cl:defmethod topic_save_settings-val ((m <DataToDiskState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:topic_save_settings-val is deprecated.  Use ff_msgs-msg:topic_save_settings instead.")
  (topic_save_settings m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataToDiskState>) ostream)
  "Serializes a message object of type '<DataToDiskState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'recording) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'topic_save_settings))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'topic_save_settings))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataToDiskState>) istream)
  "Deserializes a message object of type '<DataToDiskState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'recording) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'topic_save_settings) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'topic_save_settings)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:SaveSettings))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataToDiskState>)))
  "Returns string type for a message object of type '<DataToDiskState>"
  "ff_msgs/DataToDiskState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataToDiskState)))
  "Returns string type for a message object of type 'DataToDiskState"
  "ff_msgs/DataToDiskState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataToDiskState>)))
  "Returns md5sum for a message object of type '<DataToDiskState>"
  "68d7ec16d4c7bc2b6e1a00776a76b4f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataToDiskState)))
  "Returns md5sum for a message object of type 'DataToDiskState"
  "68d7ec16d4c7bc2b6e1a00776a76b4f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataToDiskState>)))
  "Returns full string definition for message of type '<DataToDiskState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# Data to disk state message used to let ground operators know which topics~%# are currently being recorded.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Name of the latest data to disk file uploaded from the ground~%string name~%~%# Whether the data bagger is recording a bag or not~%bool recording~%~%# An array containing information about the topics being recorded~%ff_msgs/SaveSettings[] topic_save_settings~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/SaveSettings~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# The save settings message contains information about the topics currently~%# being recorded.~%~%# Name of topic~%string topic_name~%~%# Topic saved to disk; upon docking it is downlinked~%uint8 IMMEDIATE   = 0~%~%# Topic saved to disk; upon docking it is transferred to ISS server for later~%# downlink~%uint8 DELAYED     = 1~%~%# Downlink option indicates if and when the data in the rostopic is downlinked~%uint8 downlinkOption~%~%# Times per second to save the data (Hz)~%float32 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataToDiskState)))
  "Returns full string definition for message of type 'DataToDiskState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# Data to disk state message used to let ground operators know which topics~%# are currently being recorded.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Name of the latest data to disk file uploaded from the ground~%string name~%~%# Whether the data bagger is recording a bag or not~%bool recording~%~%# An array containing information about the topics being recorded~%ff_msgs/SaveSettings[] topic_save_settings~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/SaveSettings~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# The save settings message contains information about the topics currently~%# being recorded.~%~%# Name of topic~%string topic_name~%~%# Topic saved to disk; upon docking it is downlinked~%uint8 IMMEDIATE   = 0~%~%# Topic saved to disk; upon docking it is transferred to ISS server for later~%# downlink~%uint8 DELAYED     = 1~%~%# Downlink option indicates if and when the data in the rostopic is downlinked~%uint8 downlinkOption~%~%# Times per second to save the data (Hz)~%float32 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataToDiskState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'topic_save_settings) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataToDiskState>))
  "Converts a ROS message object to a list"
  (cl:list 'DataToDiskState
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':recording (recording msg))
    (cl:cons ':topic_save_settings (topic_save_settings msg))
))
