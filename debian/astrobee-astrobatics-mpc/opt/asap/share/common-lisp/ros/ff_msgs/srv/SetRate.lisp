; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetRate-request.msg.html

(cl:defclass <SetRate-request> (roslisp-msg-protocol:ros-message)
  ((which
    :reader which
    :initarg :which
    :type cl:fixnum
    :initform 0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetRate-request (<SetRate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetRate-request> is deprecated: use ff_msgs-srv:SetRate-request instead.")))

(cl:ensure-generic-function 'which-val :lambda-list '(m))
(cl:defmethod which-val ((m <SetRate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:which-val is deprecated.  Use ff_msgs-srv:which instead.")
  (which m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <SetRate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:rate-val is deprecated.  Use ff_msgs-srv:rate instead.")
  (rate m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SetRate-request>)))
    "Constants for message type '<SetRate-request>"
  '((:COMM_STATUS . 0)
    (:CPU_STATE . 1)
    (:DISK_STATE . 2)
    (:EKF_STATE . 3)
    (:GNC_STATE . 4)
    (:PMC_CMD_STATE . 5)
    (:POSITION . 6)
    (:SPARSE_MAPPING_POSE . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SetRate-request)))
    "Constants for message type 'SetRate-request"
  '((:COMM_STATUS . 0)
    (:CPU_STATE . 1)
    (:DISK_STATE . 2)
    (:EKF_STATE . 3)
    (:GNC_STATE . 4)
    (:PMC_CMD_STATE . 5)
    (:POSITION . 6)
    (:SPARSE_MAPPING_POSE . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRate-request>) ostream)
  "Serializes a message object of type '<SetRate-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'which)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRate-request>) istream)
  "Deserializes a message object of type '<SetRate-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'which)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRate-request>)))
  "Returns string type for a service object of type '<SetRate-request>"
  "ff_msgs/SetRateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRate-request)))
  "Returns string type for a service object of type 'SetRate-request"
  "ff_msgs/SetRateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRate-request>)))
  "Returns md5sum for a message object of type '<SetRate-request>"
  "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRate-request)))
  "Returns md5sum for a message object of type 'SetRate-request"
  "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRate-request>)))
  "Returns full string definition for message of type '<SetRate-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 COMM_STATUS         = 0~%uint8 CPU_STATE           = 1~%uint8 DISK_STATE          = 2~%uint8 EKF_STATE           = 3~%uint8 GNC_STATE           = 4~%uint8 PMC_CMD_STATE       = 5~%uint8 POSITION            = 6~%uint8 SPARSE_MAPPING_POSE = 7~%~%uint8 which~%float32 rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRate-request)))
  "Returns full string definition for message of type 'SetRate-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%uint8 COMM_STATUS         = 0~%uint8 CPU_STATE           = 1~%uint8 DISK_STATE          = 2~%uint8 EKF_STATE           = 3~%uint8 GNC_STATE           = 4~%uint8 PMC_CMD_STATE       = 5~%uint8 POSITION            = 6~%uint8 SPARSE_MAPPING_POSE = 7~%~%uint8 which~%float32 rate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRate-request>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRate-request
    (cl:cons ':which (which msg))
    (cl:cons ':rate (rate msg))
))
;//! \htmlinclude SetRate-response.msg.html

(cl:defclass <SetRate-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetRate-response (<SetRate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetRate-response> is deprecated: use ff_msgs-srv:SetRate-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetRate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:status-val is deprecated.  Use ff_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRate-response>) ostream)
  "Serializes a message object of type '<SetRate-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRate-response>) istream)
  "Deserializes a message object of type '<SetRate-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRate-response>)))
  "Returns string type for a service object of type '<SetRate-response>"
  "ff_msgs/SetRateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRate-response)))
  "Returns string type for a service object of type 'SetRate-response"
  "ff_msgs/SetRateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRate-response>)))
  "Returns md5sum for a message object of type '<SetRate-response>"
  "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRate-response)))
  "Returns md5sum for a message object of type 'SetRate-response"
  "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRate-response>)))
  "Returns full string definition for message of type '<SetRate-response>"
  (cl:format cl:nil "bool success~%string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRate-response)))
  "Returns full string definition for message of type 'SetRate-response"
  (cl:format cl:nil "bool success~%string status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRate-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRate-response
    (cl:cons ':success (success msg))
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRate)))
  'SetRate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRate)))
  'SetRate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRate)))
  "Returns string type for a service object of type '<SetRate>"
  "ff_msgs/SetRate")