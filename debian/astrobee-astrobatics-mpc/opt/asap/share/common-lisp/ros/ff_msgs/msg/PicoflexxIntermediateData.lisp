; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude PicoflexxIntermediateData.msg.html

(cl:defclass <PicoflexxIntermediateData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (frequency
    :reader frequency
    :initarg :frequency
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (exposure
    :reader exposure
    :initarg :exposure
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (raw
    :reader raw
    :initarg :raw
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass PicoflexxIntermediateData (<PicoflexxIntermediateData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PicoflexxIntermediateData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PicoflexxIntermediateData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<PicoflexxIntermediateData> is deprecated: use ff_msgs-msg:PicoflexxIntermediateData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PicoflexxIntermediateData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'frequency-val :lambda-list '(m))
(cl:defmethod frequency-val ((m <PicoflexxIntermediateData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:frequency-val is deprecated.  Use ff_msgs-msg:frequency instead.")
  (frequency m))

(cl:ensure-generic-function 'exposure-val :lambda-list '(m))
(cl:defmethod exposure-val ((m <PicoflexxIntermediateData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:exposure-val is deprecated.  Use ff_msgs-msg:exposure instead.")
  (exposure m))

(cl:ensure-generic-function 'raw-val :lambda-list '(m))
(cl:defmethod raw-val ((m <PicoflexxIntermediateData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:raw-val is deprecated.  Use ff_msgs-msg:raw instead.")
  (raw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PicoflexxIntermediateData>) ostream)
  "Serializes a message object of type '<PicoflexxIntermediateData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frequency))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'frequency))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'exposure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'exposure))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'raw) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PicoflexxIntermediateData>) istream)
  "Deserializes a message object of type '<PicoflexxIntermediateData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frequency) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frequency)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'exposure) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'exposure)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'raw) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PicoflexxIntermediateData>)))
  "Returns string type for a message object of type '<PicoflexxIntermediateData>"
  "ff_msgs/PicoflexxIntermediateData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PicoflexxIntermediateData)))
  "Returns string type for a message object of type 'PicoflexxIntermediateData"
  "ff_msgs/PicoflexxIntermediateData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PicoflexxIntermediateData>)))
  "Returns md5sum for a message object of type '<PicoflexxIntermediateData>"
  "831d0b822464699c0a04d899b4867483")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PicoflexxIntermediateData)))
  "Returns md5sum for a message object of type 'PicoflexxIntermediateData"
  "831d0b822464699c0a04d899b4867483")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PicoflexxIntermediateData>)))
  "Returns full string definition for message of type '<PicoflexxIntermediateData>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.      #~%#~%# Message for encoding Picoflexx imtermediate data ~%~%std_msgs/Header header~%uint32[] frequency~%uint32[] exposure~%sensor_msgs/Image raw~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PicoflexxIntermediateData)))
  "Returns full string definition for message of type 'PicoflexxIntermediateData"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.      #~%#~%# Message for encoding Picoflexx imtermediate data ~%~%std_msgs/Header header~%uint32[] frequency~%uint32[] exposure~%sensor_msgs/Image raw~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PicoflexxIntermediateData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frequency) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'exposure) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'raw))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PicoflexxIntermediateData>))
  "Converts a ROS message object to a list"
  (cl:list 'PicoflexxIntermediateData
    (cl:cons ':header (header msg))
    (cl:cons ':frequency (frequency msg))
    (cl:cons ':exposure (exposure msg))
    (cl:cons ':raw (raw msg))
))
