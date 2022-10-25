; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DepthLandmark.msg.html

(cl:defclass <DepthLandmark> (roslisp-msg-protocol:ros-message)
  ((u
    :reader u
    :initarg :u
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0))
)

(cl:defclass DepthLandmark (<DepthLandmark>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthLandmark>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthLandmark)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DepthLandmark> is deprecated: use ff_msgs-msg:DepthLandmark instead.")))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <DepthLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:u-val is deprecated.  Use ff_msgs-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <DepthLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:v-val is deprecated.  Use ff_msgs-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <DepthLandmark>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:w-val is deprecated.  Use ff_msgs-msg:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthLandmark>) ostream)
  "Serializes a message object of type '<DepthLandmark>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'u))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthLandmark>) istream)
  "Deserializes a message object of type '<DepthLandmark>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'u) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthLandmark>)))
  "Returns string type for a message object of type '<DepthLandmark>"
  "ff_msgs/DepthLandmark")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthLandmark)))
  "Returns string type for a message object of type 'DepthLandmark"
  "ff_msgs/DepthLandmark")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthLandmark>)))
  "Returns md5sum for a message object of type '<DepthLandmark>"
  "3e053b7c204c5284962522efbf66bb32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthLandmark)))
  "Returns md5sum for a message object of type 'DepthLandmark"
  "3e053b7c204c5284962522efbf66bb32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthLandmark>)))
  "Returns full string definition for message of type '<DepthLandmark>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A landmark seen from a depth landmark~%~%float32 u     # First coordinate in the image plane~%float32 v     # Second coordinate in the image plane~%float32 w     # Depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthLandmark)))
  "Returns full string definition for message of type 'DepthLandmark"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A landmark seen from a depth landmark~%~%float32 u     # First coordinate in the image plane~%float32 v     # Second coordinate in the image plane~%float32 w     # Depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthLandmark>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthLandmark>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthLandmark
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
    (cl:cons ':w (w msg))
))
