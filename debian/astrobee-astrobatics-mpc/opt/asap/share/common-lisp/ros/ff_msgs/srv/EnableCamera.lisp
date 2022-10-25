; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude EnableCamera-request.msg.html

(cl:defclass <EnableCamera-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EnableCamera-request (<EnableCamera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableCamera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableCamera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<EnableCamera-request> is deprecated: use ff_msgs-srv:EnableCamera-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <EnableCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:mode-val is deprecated.  Use ff_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <EnableCamera-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:enable-val is deprecated.  Use ff_msgs-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<EnableCamera-request>)))
    "Constants for message type '<EnableCamera-request>"
  '((:BOTH . 0)
    (:RECORDING . 1)
    (:STREAMING . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'EnableCamera-request)))
    "Constants for message type 'EnableCamera-request"
  '((:BOTH . 0)
    (:RECORDING . 1)
    (:STREAMING . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableCamera-request>) ostream)
  "Serializes a message object of type '<EnableCamera-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableCamera-request>) istream)
  "Deserializes a message object of type '<EnableCamera-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableCamera-request>)))
  "Returns string type for a service object of type '<EnableCamera-request>"
  "ff_msgs/EnableCameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableCamera-request)))
  "Returns string type for a service object of type 'EnableCamera-request"
  "ff_msgs/EnableCameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableCamera-request>)))
  "Returns md5sum for a message object of type '<EnableCamera-request>"
  "4180836a5b8bc96980a6bee8edb99cea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableCamera-request)))
  "Returns md5sum for a message object of type 'EnableCamera-request"
  "4180836a5b8bc96980a6bee8edb99cea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableCamera-request>)))
  "Returns full string definition for message of type '<EnableCamera-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 BOTH      = 0~%uint8 RECORDING = 1~%uint8 STREAMING = 2~%~%uint8 mode~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableCamera-request)))
  "Returns full string definition for message of type 'EnableCamera-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 BOTH      = 0~%uint8 RECORDING = 1~%uint8 STREAMING = 2~%~%uint8 mode~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableCamera-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableCamera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableCamera-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude EnableCamera-response.msg.html

(cl:defclass <EnableCamera-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EnableCamera-response (<EnableCamera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableCamera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableCamera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<EnableCamera-response> is deprecated: use ff_msgs-srv:EnableCamera-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableCamera-response>) ostream)
  "Serializes a message object of type '<EnableCamera-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableCamera-response>) istream)
  "Deserializes a message object of type '<EnableCamera-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableCamera-response>)))
  "Returns string type for a service object of type '<EnableCamera-response>"
  "ff_msgs/EnableCameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableCamera-response)))
  "Returns string type for a service object of type 'EnableCamera-response"
  "ff_msgs/EnableCameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableCamera-response>)))
  "Returns md5sum for a message object of type '<EnableCamera-response>"
  "4180836a5b8bc96980a6bee8edb99cea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableCamera-response)))
  "Returns md5sum for a message object of type 'EnableCamera-response"
  "4180836a5b8bc96980a6bee8edb99cea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableCamera-response>)))
  "Returns full string definition for message of type '<EnableCamera-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableCamera-response)))
  "Returns full string definition for message of type 'EnableCamera-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableCamera-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableCamera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableCamera-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EnableCamera)))
  'EnableCamera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EnableCamera)))
  'EnableCamera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableCamera)))
  "Returns string type for a service object of type '<EnableCamera>"
  "ff_msgs/EnableCamera")