; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude VisualeyezConfig-request.msg.html

(cl:defclass <VisualeyezConfig-request> (roslisp-msg-protocol:ros-message)
  ((action
    :reader action
    :initarg :action
    :type cl:fixnum
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (pub_tf
    :reader pub_tf
    :initarg :pub_tf
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VisualeyezConfig-request (<VisualeyezConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualeyezConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualeyezConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<VisualeyezConfig-request> is deprecated: use ff_msgs-srv:VisualeyezConfig-request instead.")))

(cl:ensure-generic-function 'action-val :lambda-list '(m))
(cl:defmethod action-val ((m <VisualeyezConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:action-val is deprecated.  Use ff_msgs-srv:action instead.")
  (action m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <VisualeyezConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:name-val is deprecated.  Use ff_msgs-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'pub_tf-val :lambda-list '(m))
(cl:defmethod pub_tf-val ((m <VisualeyezConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:pub_tf-val is deprecated.  Use ff_msgs-srv:pub_tf instead.")
  (pub_tf m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<VisualeyezConfig-request>)))
    "Constants for message type '<VisualeyezConfig-request>"
  '((:TRACK . 0)
    (:RECORD . 1)
    (:CALIBRATE . 2)
    (:LOAD . 3)
    (:SAVE . 4))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'VisualeyezConfig-request)))
    "Constants for message type 'VisualeyezConfig-request"
  '((:TRACK . 0)
    (:RECORD . 1)
    (:CALIBRATE . 2)
    (:LOAD . 3)
    (:SAVE . 4))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualeyezConfig-request>) ostream)
  "Serializes a message object of type '<VisualeyezConfig-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pub_tf) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualeyezConfig-request>) istream)
  "Deserializes a message object of type '<VisualeyezConfig-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'action)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'pub_tf) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualeyezConfig-request>)))
  "Returns string type for a service object of type '<VisualeyezConfig-request>"
  "ff_msgs/VisualeyezConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualeyezConfig-request)))
  "Returns string type for a service object of type 'VisualeyezConfig-request"
  "ff_msgs/VisualeyezConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualeyezConfig-request>)))
  "Returns md5sum for a message object of type '<VisualeyezConfig-request>"
  "79a3f4aad18c09360c2499f911dea41f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualeyezConfig-request)))
  "Returns md5sum for a message object of type 'VisualeyezConfig-request"
  "79a3f4aad18c09360c2499f911dea41f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualeyezConfig-request>)))
  "Returns full string definition for message of type '<VisualeyezConfig-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 action~%uint8 TRACK       = 0~%uint8 RECORD      = 1~%uint8 CALIBRATE   = 2~%uint8 LOAD        = 3~%uint8 SAVE        = 4~%~%~%string name~%~%~%bool pub_tf~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualeyezConfig-request)))
  "Returns full string definition for message of type 'VisualeyezConfig-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 action~%uint8 TRACK       = 0~%uint8 RECORD      = 1~%uint8 CALIBRATE   = 2~%uint8 LOAD        = 3~%uint8 SAVE        = 4~%~%~%string name~%~%~%bool pub_tf~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualeyezConfig-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualeyezConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualeyezConfig-request
    (cl:cons ':action (action msg))
    (cl:cons ':name (name msg))
    (cl:cons ':pub_tf (pub_tf msg))
))
;//! \htmlinclude VisualeyezConfig-response.msg.html

(cl:defclass <VisualeyezConfig-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VisualeyezConfig-response (<VisualeyezConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualeyezConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualeyezConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<VisualeyezConfig-response> is deprecated: use ff_msgs-srv:VisualeyezConfig-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <VisualeyezConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualeyezConfig-response>) ostream)
  "Serializes a message object of type '<VisualeyezConfig-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualeyezConfig-response>) istream)
  "Deserializes a message object of type '<VisualeyezConfig-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualeyezConfig-response>)))
  "Returns string type for a service object of type '<VisualeyezConfig-response>"
  "ff_msgs/VisualeyezConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualeyezConfig-response)))
  "Returns string type for a service object of type 'VisualeyezConfig-response"
  "ff_msgs/VisualeyezConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualeyezConfig-response>)))
  "Returns md5sum for a message object of type '<VisualeyezConfig-response>"
  "79a3f4aad18c09360c2499f911dea41f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualeyezConfig-response)))
  "Returns md5sum for a message object of type 'VisualeyezConfig-response"
  "79a3f4aad18c09360c2499f911dea41f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualeyezConfig-response>)))
  "Returns full string definition for message of type '<VisualeyezConfig-response>"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualeyezConfig-response)))
  "Returns full string definition for message of type 'VisualeyezConfig-response"
  (cl:format cl:nil "~%bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualeyezConfig-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualeyezConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualeyezConfig-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VisualeyezConfig)))
  'VisualeyezConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VisualeyezConfig)))
  'VisualeyezConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualeyezConfig)))
  "Returns string type for a service object of type '<VisualeyezConfig>"
  "ff_msgs/VisualeyezConfig")