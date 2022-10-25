; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude GetFloat-request.msg.html

(cl:defclass <GetFloat-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetFloat-request (<GetFloat-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFloat-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFloat-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetFloat-request> is deprecated: use ff_msgs-srv:GetFloat-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFloat-request>) ostream)
  "Serializes a message object of type '<GetFloat-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFloat-request>) istream)
  "Deserializes a message object of type '<GetFloat-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFloat-request>)))
  "Returns string type for a service object of type '<GetFloat-request>"
  "ff_msgs/GetFloatRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFloat-request)))
  "Returns string type for a service object of type 'GetFloat-request"
  "ff_msgs/GetFloatRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFloat-request>)))
  "Returns md5sum for a message object of type '<GetFloat-request>"
  "283c964108997a34d3fae942dada5b32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFloat-request)))
  "Returns md5sum for a message object of type 'GetFloat-request"
  "283c964108997a34d3fae942dada5b32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFloat-request>)))
  "Returns full string definition for message of type '<GetFloat-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFloat-request)))
  "Returns full string definition for message of type 'GetFloat-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFloat-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFloat-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFloat-request
))
;//! \htmlinclude GetFloat-response.msg.html

(cl:defclass <GetFloat-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetFloat-response (<GetFloat-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFloat-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFloat-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<GetFloat-response> is deprecated: use ff_msgs-srv:GetFloat-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetFloat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:data-val is deprecated.  Use ff_msgs-srv:data instead.")
  (data m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetFloat-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFloat-response>) ostream)
  "Serializes a message object of type '<GetFloat-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFloat-response>) istream)
  "Deserializes a message object of type '<GetFloat-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFloat-response>)))
  "Returns string type for a service object of type '<GetFloat-response>"
  "ff_msgs/GetFloatResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFloat-response)))
  "Returns string type for a service object of type 'GetFloat-response"
  "ff_msgs/GetFloatResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFloat-response>)))
  "Returns md5sum for a message object of type '<GetFloat-response>"
  "283c964108997a34d3fae942dada5b32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFloat-response)))
  "Returns md5sum for a message object of type 'GetFloat-response"
  "283c964108997a34d3fae942dada5b32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFloat-response>)))
  "Returns full string definition for message of type '<GetFloat-response>"
  (cl:format cl:nil "float64 data~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFloat-response)))
  "Returns full string definition for message of type 'GetFloat-response"
  (cl:format cl:nil "float64 data~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFloat-response>))
  (cl:+ 0
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFloat-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFloat-response
    (cl:cons ':data (data msg))
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetFloat)))
  'GetFloat-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetFloat)))
  'GetFloat-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFloat)))
  "Returns string type for a service object of type '<GetFloat>"
  "ff_msgs/GetFloat")