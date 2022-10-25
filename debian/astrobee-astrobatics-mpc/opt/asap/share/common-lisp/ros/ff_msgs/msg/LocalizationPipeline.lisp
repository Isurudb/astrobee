; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude LocalizationPipeline.msg.html

(cl:defclass <LocalizationPipeline> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (requires_filter
    :reader requires_filter
    :initarg :requires_filter
    :type cl:boolean
    :initform cl:nil)
   (requires_optical_flow
    :reader requires_optical_flow
    :initarg :requires_optical_flow
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass LocalizationPipeline (<LocalizationPipeline>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalizationPipeline>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalizationPipeline)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<LocalizationPipeline> is deprecated: use ff_msgs-msg:LocalizationPipeline instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <LocalizationPipeline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:id-val is deprecated.  Use ff_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <LocalizationPipeline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:mode-val is deprecated.  Use ff_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <LocalizationPipeline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'requires_filter-val :lambda-list '(m))
(cl:defmethod requires_filter-val ((m <LocalizationPipeline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:requires_filter-val is deprecated.  Use ff_msgs-msg:requires_filter instead.")
  (requires_filter m))

(cl:ensure-generic-function 'requires_optical_flow-val :lambda-list '(m))
(cl:defmethod requires_optical_flow-val ((m <LocalizationPipeline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:requires_optical_flow-val is deprecated.  Use ff_msgs-msg:requires_optical_flow instead.")
  (requires_optical_flow m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalizationPipeline>) ostream)
  "Serializes a message object of type '<LocalizationPipeline>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'requires_filter) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'requires_optical_flow) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalizationPipeline>) istream)
  "Deserializes a message object of type '<LocalizationPipeline>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'requires_filter) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'requires_optical_flow) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalizationPipeline>)))
  "Returns string type for a message object of type '<LocalizationPipeline>"
  "ff_msgs/LocalizationPipeline")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalizationPipeline)))
  "Returns string type for a message object of type 'LocalizationPipeline"
  "ff_msgs/LocalizationPipeline")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalizationPipeline>)))
  "Returns md5sum for a message object of type '<LocalizationPipeline>"
  "19b78d22f6e82148b4ff1aec54ea7e06")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalizationPipeline)))
  "Returns md5sum for a message object of type 'LocalizationPipeline"
  "19b78d22f6e82148b4ff1aec54ea7e06")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalizationPipeline>)))
  "Returns full string definition for message of type '<LocalizationPipeline>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Information about a pipeline~%~%string id                     # Short id for the pipeline~%uint8 mode                    # EKF mode for the pipeline~%string name                   # Long name for the pipe~%bool requires_filter          # Does this pipeline require the EKF~%bool requires_optical_flow    # Does this pipeline require optical flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalizationPipeline)))
  "Returns full string definition for message of type 'LocalizationPipeline"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Information about a pipeline~%~%string id                     # Short id for the pipeline~%uint8 mode                    # EKF mode for the pipeline~%string name                   # Long name for the pipe~%bool requires_filter          # Does this pipeline require the EKF~%bool requires_optical_flow    # Does this pipeline require optical flow~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalizationPipeline>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     1
     4 (cl:length (cl:slot-value msg 'name))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalizationPipeline>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalizationPipeline
    (cl:cons ':id (id msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':name (name msg))
    (cl:cons ':requires_filter (requires_filter msg))
    (cl:cons ':requires_optical_flow (requires_optical_flow msg))
))
