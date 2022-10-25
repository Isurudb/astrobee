; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetBool-request.msg.html

(cl:defclass <SetBool-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBool-request (<SetBool-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBool-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBool-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetBool-request> is deprecated: use ff_msgs-srv:SetBool-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <SetBool-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:enable-val is deprecated.  Use ff_msgs-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBool-request>) ostream)
  "Serializes a message object of type '<SetBool-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBool-request>) istream)
  "Deserializes a message object of type '<SetBool-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBool-request>)))
  "Returns string type for a service object of type '<SetBool-request>"
  "ff_msgs/SetBoolRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBool-request)))
  "Returns string type for a service object of type 'SetBool-request"
  "ff_msgs/SetBoolRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBool-request>)))
  "Returns md5sum for a message object of type '<SetBool-request>"
  "6a0b406242562fc416b2c9c8a3efb051")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBool-request)))
  "Returns md5sum for a message object of type 'SetBool-request"
  "6a0b406242562fc416b2c9c8a3efb051")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBool-request>)))
  "Returns full string definition for message of type '<SetBool-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBool-request)))
  "Returns full string definition for message of type 'SetBool-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool enable~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBool-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBool-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBool-request
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude SetBool-response.msg.html

(cl:defclass <SetBool-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetBool-response (<SetBool-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetBool-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetBool-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetBool-response> is deprecated: use ff_msgs-srv:SetBool-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetBool-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetBool-response>) ostream)
  "Serializes a message object of type '<SetBool-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetBool-response>) istream)
  "Deserializes a message object of type '<SetBool-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetBool-response>)))
  "Returns string type for a service object of type '<SetBool-response>"
  "ff_msgs/SetBoolResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBool-response)))
  "Returns string type for a service object of type 'SetBool-response"
  "ff_msgs/SetBoolResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetBool-response>)))
  "Returns md5sum for a message object of type '<SetBool-response>"
  "6a0b406242562fc416b2c9c8a3efb051")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetBool-response)))
  "Returns md5sum for a message object of type 'SetBool-response"
  "6a0b406242562fc416b2c9c8a3efb051")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetBool-response>)))
  "Returns full string definition for message of type '<SetBool-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetBool-response)))
  "Returns full string definition for message of type 'SetBool-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetBool-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetBool-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetBool-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetBool)))
  'SetBool-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetBool)))
  'SetBool-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetBool)))
  "Returns string type for a service object of type '<SetBool>"
  "ff_msgs/SetBool")