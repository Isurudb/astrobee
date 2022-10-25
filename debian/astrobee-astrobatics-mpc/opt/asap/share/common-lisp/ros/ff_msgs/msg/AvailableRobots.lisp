; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude AvailableRobots.msg.html

(cl:defclass <AvailableRobots> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (available_robots
    :reader available_robots
    :initarg :available_robots
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass AvailableRobots (<AvailableRobots>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AvailableRobots>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AvailableRobots)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<AvailableRobots> is deprecated: use ff_msgs-msg:AvailableRobots instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AvailableRobots>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'available_robots-val :lambda-list '(m))
(cl:defmethod available_robots-val ((m <AvailableRobots>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:available_robots-val is deprecated.  Use ff_msgs-msg:available_robots instead.")
  (available_robots m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<AvailableRobots>)))
    "Constants for message type '<AvailableRobots>"
  '((:BUMBLE . 1)
    (:HONEY . 2)
    (:QUEEN . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'AvailableRobots)))
    "Constants for message type 'AvailableRobots"
  '((:BUMBLE . 1)
    (:HONEY . 2)
    (:QUEEN . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AvailableRobots>) ostream)
  "Serializes a message object of type '<AvailableRobots>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'available_robots))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'available_robots))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AvailableRobots>) istream)
  "Deserializes a message object of type '<AvailableRobots>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'available_robots) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'available_robots)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AvailableRobots>)))
  "Returns string type for a message object of type '<AvailableRobots>"
  "ff_msgs/AvailableRobots")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AvailableRobots)))
  "Returns string type for a message object of type 'AvailableRobots"
  "ff_msgs/AvailableRobots")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AvailableRobots>)))
  "Returns md5sum for a message object of type '<AvailableRobots>"
  "530b27dcdeb1f15db5e85c3997fe3149")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AvailableRobots)))
  "Returns md5sum for a message object of type 'AvailableRobots"
  "530b27dcdeb1f15db5e85c3997fe3149")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AvailableRobots>)))
  "Returns full string definition for message of type '<AvailableRobots>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message containing the Astrobee's connected to the current Astrobee ~%~%std_msgs/Header header # header with time stamp~%~%uint8 BUMBLE = 1~%uint8 HONEY  = 2~%uint8 QUEEN  = 4~%~%# Robots connected to the current robot through DDS~%uint8[] available_robots~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AvailableRobots)))
  "Returns full string definition for message of type 'AvailableRobots"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message containing the Astrobee's connected to the current Astrobee ~%~%std_msgs/Header header # header with time stamp~%~%uint8 BUMBLE = 1~%uint8 HONEY  = 2~%uint8 QUEEN  = 4~%~%# Robots connected to the current robot through DDS~%uint8[] available_robots~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AvailableRobots>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'available_robots) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AvailableRobots>))
  "Converts a ROS message object to a list"
  (cl:list 'AvailableRobots
    (cl:cons ':header (header msg))
    (cl:cons ':available_robots (available_robots msg))
))
