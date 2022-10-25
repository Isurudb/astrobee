; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude CameraState.msg.html

(cl:defclass <CameraState> (roslisp-msg-protocol:ros-message)
  ((camera_name
    :reader camera_name
    :initarg :camera_name
    :type cl:string
    :initform "")
   (streaming
    :reader streaming
    :initarg :streaming
    :type cl:boolean
    :initform cl:nil)
   (stream_width
    :reader stream_width
    :initarg :stream_width
    :type cl:fixnum
    :initform 0)
   (stream_height
    :reader stream_height
    :initarg :stream_height
    :type cl:fixnum
    :initform 0)
   (stream_rate
    :reader stream_rate
    :initarg :stream_rate
    :type cl:float
    :initform 0.0)
   (recording
    :reader recording
    :initarg :recording
    :type cl:boolean
    :initform cl:nil)
   (record_width
    :reader record_width
    :initarg :record_width
    :type cl:fixnum
    :initform 0)
   (record_height
    :reader record_height
    :initarg :record_height
    :type cl:fixnum
    :initform 0)
   (record_rate
    :reader record_rate
    :initarg :record_rate
    :type cl:float
    :initform 0.0)
   (bandwidth
    :reader bandwidth
    :initarg :bandwidth
    :type cl:float
    :initform 0.0))
)

(cl:defclass CameraState (<CameraState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CameraState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CameraState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<CameraState> is deprecated: use ff_msgs-msg:CameraState instead.")))

(cl:ensure-generic-function 'camera_name-val :lambda-list '(m))
(cl:defmethod camera_name-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:camera_name-val is deprecated.  Use ff_msgs-msg:camera_name instead.")
  (camera_name m))

(cl:ensure-generic-function 'streaming-val :lambda-list '(m))
(cl:defmethod streaming-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:streaming-val is deprecated.  Use ff_msgs-msg:streaming instead.")
  (streaming m))

(cl:ensure-generic-function 'stream_width-val :lambda-list '(m))
(cl:defmethod stream_width-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:stream_width-val is deprecated.  Use ff_msgs-msg:stream_width instead.")
  (stream_width m))

(cl:ensure-generic-function 'stream_height-val :lambda-list '(m))
(cl:defmethod stream_height-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:stream_height-val is deprecated.  Use ff_msgs-msg:stream_height instead.")
  (stream_height m))

(cl:ensure-generic-function 'stream_rate-val :lambda-list '(m))
(cl:defmethod stream_rate-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:stream_rate-val is deprecated.  Use ff_msgs-msg:stream_rate instead.")
  (stream_rate m))

(cl:ensure-generic-function 'recording-val :lambda-list '(m))
(cl:defmethod recording-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:recording-val is deprecated.  Use ff_msgs-msg:recording instead.")
  (recording m))

(cl:ensure-generic-function 'record_width-val :lambda-list '(m))
(cl:defmethod record_width-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:record_width-val is deprecated.  Use ff_msgs-msg:record_width instead.")
  (record_width m))

(cl:ensure-generic-function 'record_height-val :lambda-list '(m))
(cl:defmethod record_height-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:record_height-val is deprecated.  Use ff_msgs-msg:record_height instead.")
  (record_height m))

(cl:ensure-generic-function 'record_rate-val :lambda-list '(m))
(cl:defmethod record_rate-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:record_rate-val is deprecated.  Use ff_msgs-msg:record_rate instead.")
  (record_rate m))

(cl:ensure-generic-function 'bandwidth-val :lambda-list '(m))
(cl:defmethod bandwidth-val ((m <CameraState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:bandwidth-val is deprecated.  Use ff_msgs-msg:bandwidth instead.")
  (bandwidth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CameraState>) ostream)
  "Serializes a message object of type '<CameraState>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'camera_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'camera_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'streaming) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stream_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stream_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stream_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stream_height)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'stream_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'recording) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'record_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'record_width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'record_height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'record_height)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'record_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bandwidth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CameraState>) istream)
  "Deserializes a message object of type '<CameraState>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'camera_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'streaming) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stream_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stream_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stream_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stream_height)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'stream_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'recording) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'record_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'record_width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'record_height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'record_height)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'record_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bandwidth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CameraState>)))
  "Returns string type for a message object of type '<CameraState>"
  "ff_msgs/CameraState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CameraState)))
  "Returns string type for a message object of type 'CameraState"
  "ff_msgs/CameraState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CameraState>)))
  "Returns md5sum for a message object of type '<CameraState>"
  "644cfd14384d17cf28911b625a446f53")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CameraState)))
  "Returns md5sum for a message object of type 'CameraState"
  "644cfd14384d17cf28911b625a446f53")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CameraState>)))
  "Returns full string definition for message of type '<CameraState>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# CameraState message, *MUST* be kept in sync with camera portion of~%# rapid::ext::astrobee::TelemetryState~%~%# nav_cam, dock_cam, etc.~%string camera_name~%~%# streaming to ground~%bool streaming~%~%# image width~%uint16 stream_width~%# image height~%uint16 stream_height~%# Rate in Hz~%float32 stream_rate~%~%# recording to disk~%bool recording~%~%# image width~%uint16 record_width~%# image height~%uint16 record_height~%# Rate in Hz~%float32 record_rate~%~%# only for sci cam~%float32 bandwidth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CameraState)))
  "Returns full string definition for message of type 'CameraState"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# CameraState message, *MUST* be kept in sync with camera portion of~%# rapid::ext::astrobee::TelemetryState~%~%# nav_cam, dock_cam, etc.~%string camera_name~%~%# streaming to ground~%bool streaming~%~%# image width~%uint16 stream_width~%# image height~%uint16 stream_height~%# Rate in Hz~%float32 stream_rate~%~%# recording to disk~%bool recording~%~%# image width~%uint16 record_width~%# image height~%uint16 record_height~%# Rate in Hz~%float32 record_rate~%~%# only for sci cam~%float32 bandwidth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CameraState>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'camera_name))
     1
     2
     2
     4
     1
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CameraState>))
  "Converts a ROS message object to a list"
  (cl:list 'CameraState
    (cl:cons ':camera_name (camera_name msg))
    (cl:cons ':streaming (streaming msg))
    (cl:cons ':stream_width (stream_width msg))
    (cl:cons ':stream_height (stream_height msg))
    (cl:cons ':stream_rate (stream_rate msg))
    (cl:cons ':recording (recording msg))
    (cl:cons ':record_width (record_width msg))
    (cl:cons ':record_height (record_height msg))
    (cl:cons ':record_rate (record_rate msg))
    (cl:cons ':bandwidth (bandwidth msg))
))
