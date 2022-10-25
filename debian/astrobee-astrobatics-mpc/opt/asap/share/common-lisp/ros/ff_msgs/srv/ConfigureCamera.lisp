; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude ConfigureCamera-request.msg.html

(cl:defclass <ConfigureCamera-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:float
    :initform 0.0)
   (width
    :reader width
    :initarg :width
    :type cl:integer
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:integer
    :initform 0)
   (bitrate
    :reader bitrate
    :initarg :bitrate
    :type cl:float
    :initform 0.0))
)

(cl:defclass ConfigureCamera-request (<ConfigureCamera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigureCamera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigureCamera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<ConfigureCamera-request> is deprecated: use ff_msgs-srv:ConfigureCamera-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <ConfigureCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:mode-val is deprecated.  Use ff_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <ConfigureCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:rate-val is deprecated.  Use ff_msgs-srv:rate instead.")
  (rate m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ConfigureCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:width-val is deprecated.  Use ff_msgs-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ConfigureCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:height-val is deprecated.  Use ff_msgs-srv:height instead.")
  (height m))

(cl:ensure-generic-function 'bitrate-val :lambda-list '(m))
(cl:defmethod bitrate-val ((m <ConfigureCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:bitrate-val is deprecated.  Use ff_msgs-srv:bitrate instead.")
  (bitrate m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ConfigureCamera-request>)))
    "Constants for message type '<ConfigureCamera-request>"
  '((:BOTH . 0)
    (:RECORDING . 1)
    (:STREAMING . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ConfigureCamera-request)))
    "Constants for message type 'ConfigureCamera-request"
  '((:BOTH . 0)
    (:RECORDING . 1)
    (:STREAMING . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigureCamera-request>) ostream)
  "Serializes a message object of type '<ConfigureCamera-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bitrate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigureCamera-request>) istream)
  "Deserializes a message object of type '<ConfigureCamera-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bitrate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigureCamera-request>)))
  "Returns string type for a service object of type '<ConfigureCamera-request>"
  "ff_msgs/ConfigureCameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigureCamera-request)))
  "Returns string type for a service object of type 'ConfigureCamera-request"
  "ff_msgs/ConfigureCameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigureCamera-request>)))
  "Returns md5sum for a message object of type '<ConfigureCamera-request>"
  "263cde84e0c4384e57b9ce048385281d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigureCamera-request)))
  "Returns md5sum for a message object of type 'ConfigureCamera-request"
  "263cde84e0c4384e57b9ce048385281d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigureCamera-request>)))
  "Returns full string definition for message of type '<ConfigureCamera-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 BOTH      = 0~%uint8 RECORDING = 1~%uint8 STREAMING = 2~%~%uint8 mode~%float32 rate~%uint32 width~%uint32 height~%float32 bitrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigureCamera-request)))
  "Returns full string definition for message of type 'ConfigureCamera-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 BOTH      = 0~%uint8 RECORDING = 1~%uint8 STREAMING = 2~%~%uint8 mode~%float32 rate~%uint32 width~%uint32 height~%float32 bitrate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigureCamera-request>))
  (cl:+ 0
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigureCamera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigureCamera-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':rate (rate msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
    (cl:cons ':bitrate (bitrate msg))
))
;//! \htmlinclude ConfigureCamera-response.msg.html

(cl:defclass <ConfigureCamera-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ConfigureCamera-response (<ConfigureCamera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConfigureCamera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConfigureCamera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<ConfigureCamera-response> is deprecated: use ff_msgs-srv:ConfigureCamera-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConfigureCamera-response>) ostream)
  "Serializes a message object of type '<ConfigureCamera-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConfigureCamera-response>) istream)
  "Deserializes a message object of type '<ConfigureCamera-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConfigureCamera-response>)))
  "Returns string type for a service object of type '<ConfigureCamera-response>"
  "ff_msgs/ConfigureCameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigureCamera-response)))
  "Returns string type for a service object of type 'ConfigureCamera-response"
  "ff_msgs/ConfigureCameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConfigureCamera-response>)))
  "Returns md5sum for a message object of type '<ConfigureCamera-response>"
  "263cde84e0c4384e57b9ce048385281d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConfigureCamera-response)))
  "Returns md5sum for a message object of type 'ConfigureCamera-response"
  "263cde84e0c4384e57b9ce048385281d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConfigureCamera-response>)))
  "Returns full string definition for message of type '<ConfigureCamera-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConfigureCamera-response)))
  "Returns full string definition for message of type 'ConfigureCamera-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConfigureCamera-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConfigureCamera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConfigureCamera-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConfigureCamera)))
  'ConfigureCamera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConfigureCamera)))
  'ConfigureCamera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConfigureCamera)))
  "Returns string type for a service object of type '<ConfigureCamera>"
  "ff_msgs/ConfigureCamera")