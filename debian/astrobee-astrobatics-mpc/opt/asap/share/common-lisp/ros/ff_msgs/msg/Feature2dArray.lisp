; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude Feature2dArray.msg.html

(cl:defclass <Feature2dArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (camera_id
    :reader camera_id
    :initarg :camera_id
    :type cl:integer
    :initform 0)
   (feature_array
    :reader feature_array
    :initarg :feature_array
    :type (cl:vector ff_msgs-msg:Feature2d)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:Feature2d :initial-element (cl:make-instance 'ff_msgs-msg:Feature2d))))
)

(cl:defclass Feature2dArray (<Feature2dArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Feature2dArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Feature2dArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<Feature2dArray> is deprecated: use ff_msgs-msg:Feature2dArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Feature2dArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'camera_id-val :lambda-list '(m))
(cl:defmethod camera_id-val ((m <Feature2dArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:camera_id-val is deprecated.  Use ff_msgs-msg:camera_id instead.")
  (camera_id m))

(cl:ensure-generic-function 'feature_array-val :lambda-list '(m))
(cl:defmethod feature_array-val ((m <Feature2dArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:feature_array-val is deprecated.  Use ff_msgs-msg:feature_array instead.")
  (feature_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Feature2dArray>) ostream)
  "Serializes a message object of type '<Feature2dArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'camera_id)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'feature_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'feature_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Feature2dArray>) istream)
  "Deserializes a message object of type '<Feature2dArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'feature_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'feature_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:Feature2d))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Feature2dArray>)))
  "Returns string type for a message object of type '<Feature2dArray>"
  "ff_msgs/Feature2dArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Feature2dArray)))
  "Returns string type for a message object of type 'Feature2dArray"
  "ff_msgs/Feature2dArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Feature2dArray>)))
  "Returns md5sum for a message object of type '<Feature2dArray>"
  "579cb05879a7a1292a35750f014c3208")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Feature2dArray)))
  "Returns md5sum for a message object of type 'Feature2dArray"
  "579cb05879a7a1292a35750f014c3208")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Feature2dArray>)))
  "Returns full string definition for message of type '<Feature2dArray>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An observation of image points and associated ids, for optical flow.~%~%Header header # header with timestamp~%uint32 camera_id # image ID, linked to registration pulse~%Feature2d[] feature_array # list of observed features~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/Feature2d~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A single observation of a feature, with an ID and coordinates.~%# Used for an optical flow feature.~%~%uint16 id # feature ID~%float32 x # feature x coordinate~%float32 y # feature y coordinate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Feature2dArray)))
  "Returns full string definition for message of type 'Feature2dArray"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An observation of image points and associated ids, for optical flow.~%~%Header header # header with timestamp~%uint32 camera_id # image ID, linked to registration pulse~%Feature2d[] feature_array # list of observed features~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/Feature2d~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A single observation of a feature, with an ID and coordinates.~%# Used for an optical flow feature.~%~%uint16 id # feature ID~%float32 x # feature x coordinate~%float32 y # feature y coordinate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Feature2dArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'feature_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Feature2dArray>))
  "Converts a ROS message object to a list"
  (cl:list 'Feature2dArray
    (cl:cons ':header (header msg))
    (cl:cons ':camera_id (camera_id msg))
    (cl:cons ':feature_array (feature_array msg))
))
