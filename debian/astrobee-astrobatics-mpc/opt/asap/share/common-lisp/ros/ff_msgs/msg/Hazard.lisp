; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude Hazard.msg.html

(cl:defclass <Hazard> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (hazard
    :reader hazard
    :initarg :hazard
    :type geometry_msgs-msg:PointStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PointStamped)))
)

(cl:defclass Hazard (<Hazard>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Hazard>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Hazard)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<Hazard> is deprecated: use ff_msgs-msg:Hazard instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Hazard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Hazard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:type-val is deprecated.  Use ff_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'hazard-val :lambda-list '(m))
(cl:defmethod hazard-val ((m <Hazard>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:hazard-val is deprecated.  Use ff_msgs-msg:hazard instead.")
  (hazard m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Hazard>)))
    "Constants for message type '<Hazard>"
  '((:TYPE_UNKNOWN . 0)
    (:TYPE_KEEP_IN . 1)
    (:TYPE_KEEP_OUT . 2)
    (:TYPE_OBSTACLE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Hazard)))
    "Constants for message type 'Hazard"
  '((:TYPE_UNKNOWN . 0)
    (:TYPE_KEEP_IN . 1)
    (:TYPE_KEEP_OUT . 2)
    (:TYPE_OBSTACLE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Hazard>) ostream)
  "Serializes a message object of type '<Hazard>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hazard) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Hazard>) istream)
  "Deserializes a message object of type '<Hazard>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hazard) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Hazard>)))
  "Returns string type for a message object of type '<Hazard>"
  "ff_msgs/Hazard")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Hazard)))
  "Returns string type for a message object of type 'Hazard"
  "ff_msgs/Hazard")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Hazard>)))
  "Returns md5sum for a message object of type '<Hazard>"
  "4afedfcfa47c26e5781409fe253f8e48")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Hazard)))
  "Returns md5sum for a message object of type 'Hazard"
  "4afedfcfa47c26e5781409fe253f8e48")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Hazard>)))
  "Returns full string definition for message of type '<Hazard>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to notify choreographer of hazards~%~%# Header with timestamp~%std_msgs/Header header~%~%# Type of hazard~%uint8 type~%uint8 TYPE_UNKNOWN  = 0~%uint8 TYPE_KEEP_IN  = 1~%uint8 TYPE_KEEP_OUT = 2~%uint8 TYPE_OBSTACLE = 3~%~%# Spatio-tempral information about the hazard~%geometry_msgs/PointStamped hazard~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Hazard)))
  "Returns full string definition for message of type 'Hazard"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to notify choreographer of hazards~%~%# Header with timestamp~%std_msgs/Header header~%~%# Type of hazard~%uint8 type~%uint8 TYPE_UNKNOWN  = 0~%uint8 TYPE_KEEP_IN  = 1~%uint8 TYPE_KEEP_OUT = 2~%uint8 TYPE_OBSTACLE = 3~%~%# Spatio-tempral information about the hazard~%geometry_msgs/PointStamped hazard~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Hazard>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hazard))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Hazard>))
  "Converts a ROS message object to a list"
  (cl:list 'Hazard
    (cl:cons ':header (header msg))
    (cl:cons ':type (type msg))
    (cl:cons ':hazard (hazard msg))
))
