; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude PerchResult.msg.html

(cl:defclass <PerchResult> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:integer
    :initform 0)
   (fsm_result
    :reader fsm_result
    :initarg :fsm_result
    :type cl:string
    :initform ""))
)

(cl:defclass PerchResult (<PerchResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PerchResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PerchResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<PerchResult> is deprecated: use ff_msgs-msg:PerchResult instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <PerchResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:response-val is deprecated.  Use ff_msgs-msg:response instead.")
  (response m))

(cl:ensure-generic-function 'fsm_result-val :lambda-list '(m))
(cl:defmethod fsm_result-val ((m <PerchResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_result-val is deprecated.  Use ff_msgs-msg:fsm_result instead.")
  (fsm_result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<PerchResult>)))
    "Constants for message type '<PerchResult>"
  '((:CANCELLED . 5)
    (:ALREADY_PERCHED . 4)
    (:ALREADY_UNPERCHED . 3)
    (:UNPERCHED . 2)
    (:PERCHED . 1)
    (:PREEMPTED . 0)
    (:INVALID_COMMAND . -1)
    (:NOT_IN_UNPERCHED_STATE . -2)
    (:NOT_IN_PERCHED_STATE . -3)
    (:SWITCH_FAILED . -4)
    (:MOTION_FAILED . -5)
    (:ARM_FAILED . -6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'PerchResult)))
    "Constants for message type 'PerchResult"
  '((:CANCELLED . 5)
    (:ALREADY_PERCHED . 4)
    (:ALREADY_UNPERCHED . 3)
    (:UNPERCHED . 2)
    (:PERCHED . 1)
    (:PREEMPTED . 0)
    (:INVALID_COMMAND . -1)
    (:NOT_IN_UNPERCHED_STATE . -2)
    (:NOT_IN_PERCHED_STATE . -3)
    (:SWITCH_FAILED . -4)
    (:MOTION_FAILED . -5)
    (:ARM_FAILED . -6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PerchResult>) ostream)
  "Serializes a message object of type '<PerchResult>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fsm_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fsm_result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PerchResult>) istream)
  "Deserializes a message object of type '<PerchResult>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fsm_result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fsm_result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PerchResult>)))
  "Returns string type for a message object of type '<PerchResult>"
  "ff_msgs/PerchResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PerchResult)))
  "Returns string type for a message object of type 'PerchResult"
  "ff_msgs/PerchResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PerchResult>)))
  "Returns md5sum for a message object of type '<PerchResult>"
  "2036823b5e70e7e2401e3c61a8196cf3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PerchResult)))
  "Returns md5sum for a message object of type 'PerchResult"
  "2036823b5e70e7e2401e3c61a8196cf3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PerchResult>)))
  "Returns full string definition for message of type '<PerchResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Perch result~%int32 response~%int32 CANCELLED                          = 5~%int32 ALREADY_PERCHED                    = 4~%int32 ALREADY_UNPERCHED                  = 3~%int32 UNPERCHED                          = 2~%int32 PERCHED                            = 1~%int32 PREEMPTED                          = 0~%int32 INVALID_COMMAND                    = -1~%int32 NOT_IN_UNPERCHED_STATE             = -2~%int32 NOT_IN_PERCHED_STATE               = -3~%int32 SWITCH_FAILED                      = -4~%int32 MOTION_FAILED                      = -5~%int32 ARM_FAILED                         = -6~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PerchResult)))
  "Returns full string definition for message of type 'PerchResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Perch result~%int32 response~%int32 CANCELLED                          = 5~%int32 ALREADY_PERCHED                    = 4~%int32 ALREADY_UNPERCHED                  = 3~%int32 UNPERCHED                          = 2~%int32 PERCHED                            = 1~%int32 PREEMPTED                          = 0~%int32 INVALID_COMMAND                    = -1~%int32 NOT_IN_UNPERCHED_STATE             = -2~%int32 NOT_IN_PERCHED_STATE               = -3~%int32 SWITCH_FAILED                      = -4~%int32 MOTION_FAILED                      = -5~%int32 ARM_FAILED                         = -6~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PerchResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'fsm_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PerchResult>))
  "Converts a ROS message object to a list"
  (cl:list 'PerchResult
    (cl:cons ':response (response msg))
    (cl:cons ':fsm_result (fsm_result msg))
))
