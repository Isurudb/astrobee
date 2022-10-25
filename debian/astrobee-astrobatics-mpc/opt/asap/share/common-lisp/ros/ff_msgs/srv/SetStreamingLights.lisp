; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetStreamingLights-request.msg.html

(cl:defclass <SetStreamingLights-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetStreamingLights-request (<SetStreamingLights-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStreamingLights-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStreamingLights-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetStreamingLights-request> is deprecated: use ff_msgs-srv:SetStreamingLights-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetStreamingLights-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:state-val is deprecated.  Use ff_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStreamingLights-request>) ostream)
  "Serializes a message object of type '<SetStreamingLights-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStreamingLights-request>) istream)
  "Deserializes a message object of type '<SetStreamingLights-request>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStreamingLights-request>)))
  "Returns string type for a service object of type '<SetStreamingLights-request>"
  "ff_msgs/SetStreamingLightsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStreamingLights-request)))
  "Returns string type for a service object of type 'SetStreamingLights-request"
  "ff_msgs/SetStreamingLightsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStreamingLights-request>)))
  "Returns md5sum for a message object of type '<SetStreamingLights-request>"
  "4581db74aae4efc6534413a8b210908c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStreamingLights-request)))
  "Returns md5sum for a message object of type 'SetStreamingLights-request"
  "4581db74aae4efc6534413a8b210908c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStreamingLights-request>)))
  "Returns full string definition for message of type '<SetStreamingLights-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStreamingLights-request)))
  "Returns full string definition for message of type 'SetStreamingLights-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStreamingLights-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStreamingLights-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStreamingLights-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude SetStreamingLights-response.msg.html

(cl:defclass <SetStreamingLights-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetStreamingLights-response (<SetStreamingLights-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetStreamingLights-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetStreamingLights-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetStreamingLights-response> is deprecated: use ff_msgs-srv:SetStreamingLights-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetStreamingLights-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetStreamingLights-response>) ostream)
  "Serializes a message object of type '<SetStreamingLights-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetStreamingLights-response>) istream)
  "Deserializes a message object of type '<SetStreamingLights-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetStreamingLights-response>)))
  "Returns string type for a service object of type '<SetStreamingLights-response>"
  "ff_msgs/SetStreamingLightsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStreamingLights-response)))
  "Returns string type for a service object of type 'SetStreamingLights-response"
  "ff_msgs/SetStreamingLightsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetStreamingLights-response>)))
  "Returns md5sum for a message object of type '<SetStreamingLights-response>"
  "4581db74aae4efc6534413a8b210908c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetStreamingLights-response)))
  "Returns md5sum for a message object of type 'SetStreamingLights-response"
  "4581db74aae4efc6534413a8b210908c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetStreamingLights-response>)))
  "Returns full string definition for message of type '<SetStreamingLights-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetStreamingLights-response)))
  "Returns full string definition for message of type 'SetStreamingLights-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetStreamingLights-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetStreamingLights-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetStreamingLights-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetStreamingLights)))
  'SetStreamingLights-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetStreamingLights)))
  'SetStreamingLights-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetStreamingLights)))
  "Returns string type for a service object of type '<SetStreamingLights>"
  "ff_msgs/SetStreamingLights")