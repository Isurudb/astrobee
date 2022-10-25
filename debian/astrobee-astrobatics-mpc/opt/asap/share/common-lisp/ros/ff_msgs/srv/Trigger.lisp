; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude Trigger-request.msg.html

(cl:defclass <Trigger-request> (roslisp-msg-protocol:ros-message)
  ((event
    :reader event
    :initarg :event
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Trigger-request (<Trigger-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trigger-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trigger-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<Trigger-request> is deprecated: use ff_msgs-srv:Trigger-request instead.")))

(cl:ensure-generic-function 'event-val :lambda-list '(m))
(cl:defmethod event-val ((m <Trigger-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:event-val is deprecated.  Use ff_msgs-srv:event instead.")
  (event m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Trigger-request>)))
    "Constants for message type '<Trigger-request>"
  '((:UNKNOWN . 0)
    (:RESTART . 1)
    (:SLEEP . 2)
    (:WAKEUP . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Trigger-request)))
    "Constants for message type 'Trigger-request"
  '((:UNKNOWN . 0)
    (:RESTART . 1)
    (:SLEEP . 2)
    (:WAKEUP . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trigger-request>) ostream)
  "Serializes a message object of type '<Trigger-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trigger-request>) istream)
  "Deserializes a message object of type '<Trigger-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'event)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trigger-request>)))
  "Returns string type for a service object of type '<Trigger-request>"
  "ff_msgs/TriggerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trigger-request)))
  "Returns string type for a service object of type 'Trigger-request"
  "ff_msgs/TriggerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trigger-request>)))
  "Returns md5sum for a message object of type '<Trigger-request>"
  "15235914233b414e535b7900827f7f14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trigger-request)))
  "Returns md5sum for a message object of type 'Trigger-request"
  "15235914233b414e535b7900827f7f14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trigger-request>)))
  "Returns full string definition for message of type '<Trigger-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 event~%uint8 UNKNOWN = 0~%uint8 RESTART = 1~%uint8 SLEEP   = 2~%uint8 WAKEUP  = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trigger-request)))
  "Returns full string definition for message of type 'Trigger-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 event~%uint8 UNKNOWN = 0~%uint8 RESTART = 1~%uint8 SLEEP   = 2~%uint8 WAKEUP  = 3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trigger-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trigger-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Trigger-request
    (cl:cons ':event (event msg))
))
;//! \htmlinclude Trigger-response.msg.html

(cl:defclass <Trigger-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Trigger-response (<Trigger-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Trigger-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Trigger-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<Trigger-response> is deprecated: use ff_msgs-srv:Trigger-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Trigger-response>) ostream)
  "Serializes a message object of type '<Trigger-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Trigger-response>) istream)
  "Deserializes a message object of type '<Trigger-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Trigger-response>)))
  "Returns string type for a service object of type '<Trigger-response>"
  "ff_msgs/TriggerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trigger-response)))
  "Returns string type for a service object of type 'Trigger-response"
  "ff_msgs/TriggerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Trigger-response>)))
  "Returns md5sum for a message object of type '<Trigger-response>"
  "15235914233b414e535b7900827f7f14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Trigger-response)))
  "Returns md5sum for a message object of type 'Trigger-response"
  "15235914233b414e535b7900827f7f14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Trigger-response>)))
  "Returns full string definition for message of type '<Trigger-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Trigger-response)))
  "Returns full string definition for message of type 'Trigger-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Trigger-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Trigger-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Trigger-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Trigger)))
  'Trigger-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Trigger)))
  'Trigger-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Trigger)))
  "Returns string type for a service object of type '<Trigger>"
  "ff_msgs/Trigger")