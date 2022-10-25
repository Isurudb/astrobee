; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetDataToDisk-request.msg.html

(cl:defclass <SetDataToDisk-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type ff_msgs-msg:DataToDiskState
    :initform (cl:make-instance 'ff_msgs-msg:DataToDiskState)))
)

(cl:defclass SetDataToDisk-request (<SetDataToDisk-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDataToDisk-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDataToDisk-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetDataToDisk-request> is deprecated: use ff_msgs-srv:SetDataToDisk-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetDataToDisk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:state-val is deprecated.  Use ff_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDataToDisk-request>) ostream)
  "Serializes a message object of type '<SetDataToDisk-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDataToDisk-request>) istream)
  "Deserializes a message object of type '<SetDataToDisk-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDataToDisk-request>)))
  "Returns string type for a service object of type '<SetDataToDisk-request>"
  "ff_msgs/SetDataToDiskRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDataToDisk-request)))
  "Returns string type for a service object of type 'SetDataToDisk-request"
  "ff_msgs/SetDataToDiskRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDataToDisk-request>)))
  "Returns md5sum for a message object of type '<SetDataToDisk-request>"
  "b4cc86540abfc221f4a113bbc42f4b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDataToDisk-request)))
  "Returns md5sum for a message object of type 'SetDataToDisk-request"
  "b4cc86540abfc221f4a113bbc42f4b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDataToDisk-request>)))
  "Returns full string definition for message of type '<SetDataToDisk-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%ff_msgs/DataToDiskState state~%~%================================================================================~%MSG: ff_msgs/DataToDiskState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# Data to disk state message used to let ground operators know which topics~%# are currently being recorded.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Name of the latest data to disk file uploaded from the ground~%string name~%~%# Whether the data bagger is recording a bag or not~%bool recording~%~%# An array containing information about the topics being recorded~%ff_msgs/SaveSettings[] topic_save_settings~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/SaveSettings~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# The save settings message contains information about the topics currently~%# being recorded.~%~%# Name of topic~%string topic_name~%~%# Topic saved to disk; upon docking it is downlinked~%uint8 IMMEDIATE   = 0~%~%# Topic saved to disk; upon docking it is transferred to ISS server for later~%# downlink~%uint8 DELAYED     = 1~%~%# Downlink option indicates if and when the data in the rostopic is downlinked~%uint8 downlinkOption~%~%# Times per second to save the data (Hz)~%float32 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDataToDisk-request)))
  "Returns full string definition for message of type 'SetDataToDisk-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%ff_msgs/DataToDiskState state~%~%================================================================================~%MSG: ff_msgs/DataToDiskState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# Data to disk state message used to let ground operators know which topics~%# are currently being recorded.~%~%# Header with timestamp~%std_msgs/Header header~%~%# Name of the latest data to disk file uploaded from the ground~%string name~%~%# Whether the data bagger is recording a bag or not~%bool recording~%~%# An array containing information about the topics being recorded~%ff_msgs/SaveSettings[] topic_save_settings~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/SaveSettings~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%~%# The save settings message contains information about the topics currently~%# being recorded.~%~%# Name of topic~%string topic_name~%~%# Topic saved to disk; upon docking it is downlinked~%uint8 IMMEDIATE   = 0~%~%# Topic saved to disk; upon docking it is transferred to ISS server for later~%# downlink~%uint8 DELAYED     = 1~%~%# Downlink option indicates if and when the data in the rostopic is downlinked~%uint8 downlinkOption~%~%# Times per second to save the data (Hz)~%float32 frequency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDataToDisk-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDataToDisk-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDataToDisk-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetDataToDisk-response.msg.html

(cl:defclass <SetDataToDisk-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass SetDataToDisk-response (<SetDataToDisk-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDataToDisk-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDataToDisk-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetDataToDisk-response> is deprecated: use ff_msgs-srv:SetDataToDisk-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetDataToDisk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetDataToDisk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:status-val is deprecated.  Use ff_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDataToDisk-response>) ostream)
  "Serializes a message object of type '<SetDataToDisk-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDataToDisk-response>) istream)
  "Deserializes a message object of type '<SetDataToDisk-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDataToDisk-response>)))
  "Returns string type for a service object of type '<SetDataToDisk-response>"
  "ff_msgs/SetDataToDiskResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDataToDisk-response)))
  "Returns string type for a service object of type 'SetDataToDisk-response"
  "ff_msgs/SetDataToDiskResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDataToDisk-response>)))
  "Returns md5sum for a message object of type '<SetDataToDisk-response>"
  "b4cc86540abfc221f4a113bbc42f4b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDataToDisk-response)))
  "Returns md5sum for a message object of type 'SetDataToDisk-response"
  "b4cc86540abfc221f4a113bbc42f4b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDataToDisk-response>)))
  "Returns full string definition for message of type '<SetDataToDisk-response>"
  (cl:format cl:nil "bool success~%string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDataToDisk-response)))
  "Returns full string definition for message of type 'SetDataToDisk-response"
  (cl:format cl:nil "bool success~%string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDataToDisk-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDataToDisk-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDataToDisk-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDataToDisk)))
  'SetDataToDisk-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDataToDisk)))
  'SetDataToDisk-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDataToDisk)))
  "Returns string type for a service object of type '<SetDataToDisk>"
  "ff_msgs/SetDataToDisk")