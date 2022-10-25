; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude ResetMap-request.msg.html

(cl:defclass <ResetMap-request> (roslisp-msg-protocol:ros-message)
  ((map_file
    :reader map_file
    :initarg :map_file
    :type cl:string
    :initform ""))
)

(cl:defclass ResetMap-request (<ResetMap-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetMap-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetMap-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<ResetMap-request> is deprecated: use ff_msgs-srv:ResetMap-request instead.")))

(cl:ensure-generic-function 'map_file-val :lambda-list '(m))
(cl:defmethod map_file-val ((m <ResetMap-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:map_file-val is deprecated.  Use ff_msgs-srv:map_file instead.")
  (map_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetMap-request>) ostream)
  "Serializes a message object of type '<ResetMap-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'map_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'map_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetMap-request>) istream)
  "Deserializes a message object of type '<ResetMap-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'map_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'map_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetMap-request>)))
  "Returns string type for a service object of type '<ResetMap-request>"
  "ff_msgs/ResetMapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetMap-request)))
  "Returns string type for a service object of type 'ResetMap-request"
  "ff_msgs/ResetMapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetMap-request>)))
  "Returns md5sum for a message object of type '<ResetMap-request>"
  "a377c8d7c4f71636969846ebf44e4df2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetMap-request)))
  "Returns md5sum for a message object of type 'ResetMap-request"
  "a377c8d7c4f71636969846ebf44e4df2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetMap-request>)))
  "Returns full string definition for message of type '<ResetMap-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string map_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetMap-request)))
  "Returns full string definition for message of type 'ResetMap-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string map_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetMap-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'map_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetMap-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetMap-request
    (cl:cons ':map_file (map_file msg))
))
;//! \htmlinclude ResetMap-response.msg.html

(cl:defclass <ResetMap-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ResetMap-response (<ResetMap-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ResetMap-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ResetMap-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<ResetMap-response> is deprecated: use ff_msgs-srv:ResetMap-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ResetMap-response>) ostream)
  "Serializes a message object of type '<ResetMap-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ResetMap-response>) istream)
  "Deserializes a message object of type '<ResetMap-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ResetMap-response>)))
  "Returns string type for a service object of type '<ResetMap-response>"
  "ff_msgs/ResetMapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetMap-response)))
  "Returns string type for a service object of type 'ResetMap-response"
  "ff_msgs/ResetMapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ResetMap-response>)))
  "Returns md5sum for a message object of type '<ResetMap-response>"
  "a377c8d7c4f71636969846ebf44e4df2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ResetMap-response)))
  "Returns md5sum for a message object of type 'ResetMap-response"
  "a377c8d7c4f71636969846ebf44e4df2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ResetMap-response>)))
  "Returns full string definition for message of type '<ResetMap-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ResetMap-response)))
  "Returns full string definition for message of type 'ResetMap-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ResetMap-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ResetMap-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ResetMap-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ResetMap)))
  'ResetMap-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ResetMap)))
  'ResetMap-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ResetMap)))
  "Returns string type for a service object of type '<ResetMap>"
  "ff_msgs/ResetMap")