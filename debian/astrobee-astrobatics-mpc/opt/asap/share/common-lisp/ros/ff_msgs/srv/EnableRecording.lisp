; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude EnableRecording-request.msg.html

(cl:defclass <EnableRecording-request> (roslisp-msg-protocol:ros-message)
  ((enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil)
   (bag_description
    :reader bag_description
    :initarg :bag_description
    :type cl:string
    :initform ""))
)

(cl:defclass EnableRecording-request (<EnableRecording-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableRecording-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableRecording-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<EnableRecording-request> is deprecated: use ff_msgs-srv:EnableRecording-request instead.")))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <EnableRecording-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:enable-val is deprecated.  Use ff_msgs-srv:enable instead.")
  (enable m))

(cl:ensure-generic-function 'bag_description-val :lambda-list '(m))
(cl:defmethod bag_description-val ((m <EnableRecording-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:bag_description-val is deprecated.  Use ff_msgs-srv:bag_description instead.")
  (bag_description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableRecording-request>) ostream)
  "Serializes a message object of type '<EnableRecording-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'bag_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'bag_description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableRecording-request>) istream)
  "Deserializes a message object of type '<EnableRecording-request>"
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'bag_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'bag_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableRecording-request>)))
  "Returns string type for a service object of type '<EnableRecording-request>"
  "ff_msgs/EnableRecordingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableRecording-request)))
  "Returns string type for a service object of type 'EnableRecording-request"
  "ff_msgs/EnableRecordingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableRecording-request>)))
  "Returns md5sum for a message object of type '<EnableRecording-request>"
  "68fd83cd501355d809d1d0420334c998")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableRecording-request)))
  "Returns md5sum for a message object of type 'EnableRecording-request"
  "68fd83cd501355d809d1d0420334c998")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableRecording-request>)))
  "Returns full string definition for message of type '<EnableRecording-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool enable~%string bag_description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableRecording-request)))
  "Returns full string definition for message of type 'EnableRecording-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%bool enable~%string bag_description~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableRecording-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'bag_description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableRecording-request>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableRecording-request
    (cl:cons ':enable (enable msg))
    (cl:cons ':bag_description (bag_description msg))
))
;//! \htmlinclude EnableRecording-response.msg.html

(cl:defclass <EnableRecording-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass EnableRecording-response (<EnableRecording-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EnableRecording-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EnableRecording-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<EnableRecording-response> is deprecated: use ff_msgs-srv:EnableRecording-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EnableRecording-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <EnableRecording-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:status-val is deprecated.  Use ff_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EnableRecording-response>) ostream)
  "Serializes a message object of type '<EnableRecording-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EnableRecording-response>) istream)
  "Deserializes a message object of type '<EnableRecording-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EnableRecording-response>)))
  "Returns string type for a service object of type '<EnableRecording-response>"
  "ff_msgs/EnableRecordingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableRecording-response)))
  "Returns string type for a service object of type 'EnableRecording-response"
  "ff_msgs/EnableRecordingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EnableRecording-response>)))
  "Returns md5sum for a message object of type '<EnableRecording-response>"
  "68fd83cd501355d809d1d0420334c998")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EnableRecording-response)))
  "Returns md5sum for a message object of type 'EnableRecording-response"
  "68fd83cd501355d809d1d0420334c998")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EnableRecording-response>)))
  "Returns full string definition for message of type '<EnableRecording-response>"
  (cl:format cl:nil "bool success~%string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EnableRecording-response)))
  "Returns full string definition for message of type 'EnableRecording-response"
  (cl:format cl:nil "bool success~%string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EnableRecording-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EnableRecording-response>))
  "Converts a ROS message object to a list"
  (cl:list 'EnableRecording-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'EnableRecording)))
  'EnableRecording-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'EnableRecording)))
  'EnableRecording-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EnableRecording)))
  "Returns string type for a service object of type '<EnableRecording>"
  "ff_msgs/EnableRecording")