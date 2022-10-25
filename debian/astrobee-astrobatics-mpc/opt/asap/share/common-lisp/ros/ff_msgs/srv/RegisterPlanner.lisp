; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude RegisterPlanner-request.msg.html

(cl:defclass <RegisterPlanner-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform "")
   (unregister
    :reader unregister
    :initarg :unregister
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RegisterPlanner-request (<RegisterPlanner-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterPlanner-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterPlanner-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<RegisterPlanner-request> is deprecated: use ff_msgs-srv:RegisterPlanner-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <RegisterPlanner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:name-val is deprecated.  Use ff_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <RegisterPlanner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:description-val is deprecated.  Use ff_msgs-srv:description instead.")
  (description m))

(cl:ensure-generic-function 'unregister-val :lambda-list '(m))
(cl:defmethod unregister-val ((m <RegisterPlanner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:unregister-val is deprecated.  Use ff_msgs-srv:unregister instead.")
  (unregister m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterPlanner-request>) ostream)
  "Serializes a message object of type '<RegisterPlanner-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'unregister) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterPlanner-request>) istream)
  "Deserializes a message object of type '<RegisterPlanner-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'unregister) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterPlanner-request>)))
  "Returns string type for a service object of type '<RegisterPlanner-request>"
  "ff_msgs/RegisterPlannerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterPlanner-request)))
  "Returns string type for a service object of type 'RegisterPlanner-request"
  "ff_msgs/RegisterPlannerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterPlanner-request>)))
  "Returns md5sum for a message object of type '<RegisterPlanner-request>"
  "e247f0a3c6e3085865c44afa8ad187df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterPlanner-request)))
  "Returns md5sum for a message object of type 'RegisterPlanner-request"
  "e247f0a3c6e3085865c44afa8ad187df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterPlanner-request>)))
  "Returns full string definition for message of type '<RegisterPlanner-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string name~%string description~%bool unregister~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterPlanner-request)))
  "Returns full string definition for message of type 'RegisterPlanner-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%string name~%string description~%bool unregister~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterPlanner-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'description))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterPlanner-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterPlanner-request
    (cl:cons ':name (name msg))
    (cl:cons ':description (description msg))
    (cl:cons ':unregister (unregister msg))
))
;//! \htmlinclude RegisterPlanner-response.msg.html

(cl:defclass <RegisterPlanner-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass RegisterPlanner-response (<RegisterPlanner-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterPlanner-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterPlanner-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<RegisterPlanner-response> is deprecated: use ff_msgs-srv:RegisterPlanner-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterPlanner-response>) ostream)
  "Serializes a message object of type '<RegisterPlanner-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterPlanner-response>) istream)
  "Deserializes a message object of type '<RegisterPlanner-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterPlanner-response>)))
  "Returns string type for a service object of type '<RegisterPlanner-response>"
  "ff_msgs/RegisterPlannerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterPlanner-response)))
  "Returns string type for a service object of type 'RegisterPlanner-response"
  "ff_msgs/RegisterPlannerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterPlanner-response>)))
  "Returns md5sum for a message object of type '<RegisterPlanner-response>"
  "e247f0a3c6e3085865c44afa8ad187df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterPlanner-response)))
  "Returns md5sum for a message object of type 'RegisterPlanner-response"
  "e247f0a3c6e3085865c44afa8ad187df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterPlanner-response>)))
  "Returns full string definition for message of type '<RegisterPlanner-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterPlanner-response)))
  "Returns full string definition for message of type 'RegisterPlanner-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterPlanner-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterPlanner-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterPlanner-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegisterPlanner)))
  'RegisterPlanner-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegisterPlanner)))
  'RegisterPlanner-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterPlanner)))
  "Returns string type for a service object of type '<RegisterPlanner>"
  "ff_msgs/RegisterPlanner")