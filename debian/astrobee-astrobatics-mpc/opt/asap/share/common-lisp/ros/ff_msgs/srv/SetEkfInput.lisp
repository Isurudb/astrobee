; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetEkfInput-request.msg.html

(cl:defclass <SetEkfInput-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SetEkfInput-request (<SetEkfInput-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEkfInput-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEkfInput-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetEkfInput-request> is deprecated: use ff_msgs-srv:SetEkfInput-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetEkfInput-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:mode-val is deprecated.  Use ff_msgs-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SetEkfInput-request>)))
    "Constants for message type '<SetEkfInput-request>"
  '((:MODE_MAP_LANDMARKS . 0)
    (:MODE_AR_TAGS . 1)
    (:MODE_HANDRAIL . 2)
    (:MODE_NONE . 3)
    (:MODE_PERCH . 4)
    (:MODE_TRUTH . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SetEkfInput-request)))
    "Constants for message type 'SetEkfInput-request"
  '((:MODE_MAP_LANDMARKS . 0)
    (:MODE_AR_TAGS . 1)
    (:MODE_HANDRAIL . 2)
    (:MODE_NONE . 3)
    (:MODE_PERCH . 4)
    (:MODE_TRUTH . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEkfInput-request>) ostream)
  "Serializes a message object of type '<SetEkfInput-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEkfInput-request>) istream)
  "Deserializes a message object of type '<SetEkfInput-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEkfInput-request>)))
  "Returns string type for a service object of type '<SetEkfInput-request>"
  "ff_msgs/SetEkfInputRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEkfInput-request)))
  "Returns string type for a service object of type 'SetEkfInput-request"
  "ff_msgs/SetEkfInputRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEkfInput-request>)))
  "Returns md5sum for a message object of type '<SetEkfInput-request>"
  "f21f65ebbf246d59f8a1b8f5fe25668b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEkfInput-request)))
  "Returns md5sum for a message object of type 'SetEkfInput-request"
  "f21f65ebbf246d59f8a1b8f5fe25668b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEkfInput-request>)))
  "Returns full string definition for message of type '<SetEkfInput-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 MODE_MAP_LANDMARKS = 0~%uint8 MODE_AR_TAGS       = 1~%uint8 MODE_HANDRAIL      = 2~%uint8 MODE_NONE          = 3~%uint8 MODE_PERCH         = 4~%uint8 MODE_TRUTH         = 5~%~%uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEkfInput-request)))
  "Returns full string definition for message of type 'SetEkfInput-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 MODE_MAP_LANDMARKS = 0~%uint8 MODE_AR_TAGS       = 1~%uint8 MODE_HANDRAIL      = 2~%uint8 MODE_NONE          = 3~%uint8 MODE_PERCH         = 4~%uint8 MODE_TRUTH         = 5~%~%uint8 mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEkfInput-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEkfInput-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEkfInput-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetEkfInput-response.msg.html

(cl:defclass <SetEkfInput-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetEkfInput-response (<SetEkfInput-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetEkfInput-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetEkfInput-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetEkfInput-response> is deprecated: use ff_msgs-srv:SetEkfInput-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetEkfInput-response>) ostream)
  "Serializes a message object of type '<SetEkfInput-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetEkfInput-response>) istream)
  "Deserializes a message object of type '<SetEkfInput-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetEkfInput-response>)))
  "Returns string type for a service object of type '<SetEkfInput-response>"
  "ff_msgs/SetEkfInputResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEkfInput-response)))
  "Returns string type for a service object of type 'SetEkfInput-response"
  "ff_msgs/SetEkfInputResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetEkfInput-response>)))
  "Returns md5sum for a message object of type '<SetEkfInput-response>"
  "f21f65ebbf246d59f8a1b8f5fe25668b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetEkfInput-response)))
  "Returns md5sum for a message object of type 'SetEkfInput-response"
  "f21f65ebbf246d59f8a1b8f5fe25668b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetEkfInput-response>)))
  "Returns full string definition for message of type '<SetEkfInput-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetEkfInput-response)))
  "Returns full string definition for message of type 'SetEkfInput-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetEkfInput-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetEkfInput-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetEkfInput-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetEkfInput)))
  'SetEkfInput-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetEkfInput)))
  'SetEkfInput-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetEkfInput)))
  "Returns string type for a service object of type '<SetEkfInput>"
  "ff_msgs/SetEkfInput")