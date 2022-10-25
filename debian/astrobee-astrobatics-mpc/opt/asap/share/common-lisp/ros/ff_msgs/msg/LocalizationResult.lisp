; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude LocalizationResult.msg.html

(cl:defclass <LocalizationResult> (roslisp-msg-protocol:ros-message)
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

(cl:defclass LocalizationResult (<LocalizationResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalizationResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalizationResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<LocalizationResult> is deprecated: use ff_msgs-msg:LocalizationResult instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <LocalizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:response-val is deprecated.  Use ff_msgs-msg:response instead.")
  (response m))

(cl:ensure-generic-function 'fsm_result-val :lambda-list '(m))
(cl:defmethod fsm_result-val ((m <LocalizationResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_result-val is deprecated.  Use ff_msgs-msg:fsm_result instead.")
  (fsm_result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<LocalizationResult>)))
    "Constants for message type '<LocalizationResult>"
  '((:PIPELINE_ALREADY_ACTIVE . 2)
    (:SUCCESS . 1)
    (:PREEMPTED . 0)
    (:CANCELLED . -1)
    (:INVALID_PIPELINE . -2)
    (:INVALID_COMMAND . -3)
    (:FILTER_NOT_IN_USE . -4)
    (:OPTICAL_FLOW_FAILED . -5)
    (:PIPELINE_TOGGLE_FAILED . -6)
    (:PIPELINE_USE_FAILED . -7)
    (:PIPELINE_UNSTABLE . -8)
    (:SET_INPUT_FAILED . -9)
    (:ESTIMATE_BIAS_FAILED . -10)
    (:RESET_FAILED . -11))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'LocalizationResult)))
    "Constants for message type 'LocalizationResult"
  '((:PIPELINE_ALREADY_ACTIVE . 2)
    (:SUCCESS . 1)
    (:PREEMPTED . 0)
    (:CANCELLED . -1)
    (:INVALID_PIPELINE . -2)
    (:INVALID_COMMAND . -3)
    (:FILTER_NOT_IN_USE . -4)
    (:OPTICAL_FLOW_FAILED . -5)
    (:PIPELINE_TOGGLE_FAILED . -6)
    (:PIPELINE_USE_FAILED . -7)
    (:PIPELINE_UNSTABLE . -8)
    (:SET_INPUT_FAILED . -9)
    (:ESTIMATE_BIAS_FAILED . -10)
    (:RESET_FAILED . -11))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalizationResult>) ostream)
  "Serializes a message object of type '<LocalizationResult>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalizationResult>) istream)
  "Deserializes a message object of type '<LocalizationResult>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalizationResult>)))
  "Returns string type for a message object of type '<LocalizationResult>"
  "ff_msgs/LocalizationResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalizationResult)))
  "Returns string type for a message object of type 'LocalizationResult"
  "ff_msgs/LocalizationResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalizationResult>)))
  "Returns md5sum for a message object of type '<LocalizationResult>"
  "309c1ead50fb170acfed9e9b67a66d27")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalizationResult)))
  "Returns md5sum for a message object of type 'LocalizationResult"
  "309c1ead50fb170acfed9e9b67a66d27")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalizationResult>)))
  "Returns full string definition for message of type '<LocalizationResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%int32 response~%int32 PIPELINE_ALREADY_ACTIVE     =  2   # We are already on this mode~%int32 SUCCESS                     =  1   # The switch was successful~%int32 PREEMPTED                   =  0   # Preempted by another action goal~%int32 CANCELLED                   = -1   # We canceled our own request~%int32 INVALID_PIPELINE            = -2   # Not a valid pipeline in command~%int32 INVALID_COMMAND             = -3   # Not a valid command type~%int32 FILTER_NOT_IN_USE           = -4   # Reset/bias requires filter~%int32 OPTICAL_FLOW_FAILED         = -5   # Optical flow could not be toggled~%int32 PIPELINE_TOGGLE_FAILED      = -6   # Pipeline could not be toggled~%int32 PIPELINE_USE_FAILED         = -7   # Pipeline could not be used~%int32 PIPELINE_UNSTABLE           = -8   # Pipeline did not go stable~%int32 SET_INPUT_FAILED            = -9   # EKF could not be set to new mode~%int32 ESTIMATE_BIAS_FAILED        = -10  # Estimate bias service call failed~%int32 RESET_FAILED                = -11  # Reset service call failed~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalizationResult)))
  "Returns full string definition for message of type 'LocalizationResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%int32 response~%int32 PIPELINE_ALREADY_ACTIVE     =  2   # We are already on this mode~%int32 SUCCESS                     =  1   # The switch was successful~%int32 PREEMPTED                   =  0   # Preempted by another action goal~%int32 CANCELLED                   = -1   # We canceled our own request~%int32 INVALID_PIPELINE            = -2   # Not a valid pipeline in command~%int32 INVALID_COMMAND             = -3   # Not a valid command type~%int32 FILTER_NOT_IN_USE           = -4   # Reset/bias requires filter~%int32 OPTICAL_FLOW_FAILED         = -5   # Optical flow could not be toggled~%int32 PIPELINE_TOGGLE_FAILED      = -6   # Pipeline could not be toggled~%int32 PIPELINE_USE_FAILED         = -7   # Pipeline could not be used~%int32 PIPELINE_UNSTABLE           = -8   # Pipeline did not go stable~%int32 SET_INPUT_FAILED            = -9   # EKF could not be set to new mode~%int32 ESTIMATE_BIAS_FAILED        = -10  # Estimate bias service call failed~%int32 RESET_FAILED                = -11  # Reset service call failed~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalizationResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'fsm_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalizationResult>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalizationResult
    (cl:cons ':response (response msg))
    (cl:cons ':fsm_result (fsm_result msg))
))
