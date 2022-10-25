; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude VisualeyezFeedback.msg.html

(cl:defclass <VisualeyezFeedback> (roslisp-msg-protocol:ros-message)
  ((tcmid
    :reader tcmid
    :initarg :tcmid
    :type cl:fixnum
    :initform 0)
   (ledid
    :reader ledid
    :initarg :ledid
    :type cl:fixnum
    :initform 0)
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0))
)

(cl:defclass VisualeyezFeedback (<VisualeyezFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualeyezFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualeyezFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<VisualeyezFeedback> is deprecated: use ff_msgs-msg:VisualeyezFeedback instead.")))

(cl:ensure-generic-function 'tcmid-val :lambda-list '(m))
(cl:defmethod tcmid-val ((m <VisualeyezFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tcmid-val is deprecated.  Use ff_msgs-msg:tcmid instead.")
  (tcmid m))

(cl:ensure-generic-function 'ledid-val :lambda-list '(m))
(cl:defmethod ledid-val ((m <VisualeyezFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ledid-val is deprecated.  Use ff_msgs-msg:ledid instead.")
  (ledid m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <VisualeyezFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:count-val is deprecated.  Use ff_msgs-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualeyezFeedback>) ostream)
  "Serializes a message object of type '<VisualeyezFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tcmid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ledid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualeyezFeedback>) istream)
  "Deserializes a message object of type '<VisualeyezFeedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tcmid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ledid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'count)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualeyezFeedback>)))
  "Returns string type for a message object of type '<VisualeyezFeedback>"
  "ff_msgs/VisualeyezFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualeyezFeedback)))
  "Returns string type for a message object of type 'VisualeyezFeedback"
  "ff_msgs/VisualeyezFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualeyezFeedback>)))
  "Returns md5sum for a message object of type '<VisualeyezFeedback>"
  "2ce06a8bb76a1a67ecd18953ba9bbf84")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualeyezFeedback)))
  "Returns md5sum for a message object of type 'VisualeyezFeedback"
  "2ce06a8bb76a1a67ecd18953ba9bbf84")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualeyezFeedback>)))
  "Returns full string definition for message of type '<VisualeyezFeedback>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Visualeyez feedback.~%~%uint8 tcmid                         # Transmission control module ID~%uint8 ledid                         # Light emitting diode ID~%uint32 count                        # Number of valid measurements~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualeyezFeedback)))
  "Returns full string definition for message of type 'VisualeyezFeedback"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Visualeyez feedback.~%~%uint8 tcmid                         # Transmission control module ID~%uint8 ledid                         # Light emitting diode ID~%uint32 count                        # Number of valid measurements~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualeyezFeedback>))
  (cl:+ 0
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualeyezFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualeyezFeedback
    (cl:cons ':tcmid (tcmid msg))
    (cl:cons ':ledid (ledid msg))
    (cl:cons ':count (count msg))
))
