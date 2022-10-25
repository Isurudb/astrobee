; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetState-request.msg.html

(cl:defclass <SetState-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:integer
    :initform 0))
)

(cl:defclass SetState-request (<SetState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetState-request> is deprecated: use ff_msgs-srv:SetState-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:state-val is deprecated.  Use ff_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetState-request>) ostream)
  "Serializes a message object of type '<SetState-request>"
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetState-request>) istream)
  "Deserializes a message object of type '<SetState-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetState-request>)))
  "Returns string type for a service object of type '<SetState-request>"
  "ff_msgs/SetStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetState-request)))
  "Returns string type for a service object of type 'SetState-request"
  "ff_msgs/SetStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetState-request>)))
  "Returns md5sum for a message object of type '<SetState-request>"
  "6d0ec020603882606d0a12cc504ac31c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetState-request)))
  "Returns md5sum for a message object of type 'SetState-request"
  "6d0ec020603882606d0a12cc504ac31c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetState-request>)))
  "Returns full string definition for message of type '<SetState-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetState-request)))
  "Returns full string definition for message of type 'SetState-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%int32 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetState-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetState-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetState-response.msg.html

(cl:defclass <SetState-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetState-response (<SetState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetState-response> is deprecated: use ff_msgs-srv:SetState-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetState-response>) ostream)
  "Serializes a message object of type '<SetState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetState-response>) istream)
  "Deserializes a message object of type '<SetState-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetState-response>)))
  "Returns string type for a service object of type '<SetState-response>"
  "ff_msgs/SetStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetState-response)))
  "Returns string type for a service object of type 'SetState-response"
  "ff_msgs/SetStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetState-response>)))
  "Returns md5sum for a message object of type '<SetState-response>"
  "6d0ec020603882606d0a12cc504ac31c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetState-response)))
  "Returns md5sum for a message object of type 'SetState-response"
  "6d0ec020603882606d0a12cc504ac31c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetState-response>)))
  "Returns full string definition for message of type '<SetState-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetState-response)))
  "Returns full string definition for message of type 'SetState-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetState-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetState)))
  'SetState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetState)))
  'SetState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetState)))
  "Returns string type for a service object of type '<SetState>"
  "ff_msgs/SetState")