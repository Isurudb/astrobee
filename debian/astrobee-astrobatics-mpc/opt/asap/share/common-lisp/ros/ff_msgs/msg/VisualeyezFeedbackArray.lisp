; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude VisualeyezFeedbackArray.msg.html

(cl:defclass <VisualeyezFeedbackArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (feedback
    :reader feedback
    :initarg :feedback
    :type (cl:vector ff_msgs-msg:VisualeyezFeedback)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:VisualeyezFeedback :initial-element (cl:make-instance 'ff_msgs-msg:VisualeyezFeedback))))
)

(cl:defclass VisualeyezFeedbackArray (<VisualeyezFeedbackArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualeyezFeedbackArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualeyezFeedbackArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<VisualeyezFeedbackArray> is deprecated: use ff_msgs-msg:VisualeyezFeedbackArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VisualeyezFeedbackArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <VisualeyezFeedbackArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:feedback-val is deprecated.  Use ff_msgs-msg:feedback instead.")
  (feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualeyezFeedbackArray>) ostream)
  "Serializes a message object of type '<VisualeyezFeedbackArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'feedback))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'feedback))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualeyezFeedbackArray>) istream)
  "Deserializes a message object of type '<VisualeyezFeedbackArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'feedback) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'feedback)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:VisualeyezFeedback))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualeyezFeedbackArray>)))
  "Returns string type for a message object of type '<VisualeyezFeedbackArray>"
  "ff_msgs/VisualeyezFeedbackArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualeyezFeedbackArray)))
  "Returns string type for a message object of type 'VisualeyezFeedbackArray"
  "ff_msgs/VisualeyezFeedbackArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualeyezFeedbackArray>)))
  "Returns md5sum for a message object of type '<VisualeyezFeedbackArray>"
  "50480fdca33dc36bf859801c972b691e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualeyezFeedbackArray)))
  "Returns md5sum for a message object of type 'VisualeyezFeedbackArray"
  "50480fdca33dc36bf859801c972b691e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualeyezFeedbackArray>)))
  "Returns full string definition for message of type '<VisualeyezFeedbackArray>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Visualeyez feedback array with timestamp.~%~%Header header                           # Header with timestamp~%ff_msgs/VisualeyezFeedback[] feedback   # List of all measurements~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/VisualeyezFeedback~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Visualeyez feedback.~%~%uint8 tcmid                         # Transmission control module ID~%uint8 ledid                         # Light emitting diode ID~%uint32 count                        # Number of valid measurements~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualeyezFeedbackArray)))
  "Returns full string definition for message of type 'VisualeyezFeedbackArray"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Visualeyez feedback array with timestamp.~%~%Header header                           # Header with timestamp~%ff_msgs/VisualeyezFeedback[] feedback   # List of all measurements~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/VisualeyezFeedback~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Visualeyez feedback.~%~%uint8 tcmid                         # Transmission control module ID~%uint8 ledid                         # Light emitting diode ID~%uint32 count                        # Number of valid measurements~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualeyezFeedbackArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'feedback) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualeyezFeedbackArray>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualeyezFeedbackArray
    (cl:cons ':header (header msg))
    (cl:cons ':feedback (feedback msg))
))
