; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmResult.msg.html

(cl:defclass <ArmResult> (roslisp-msg-protocol:ros-message)
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

(cl:defclass ArmResult (<ArmResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmResult> is deprecated: use ff_msgs-msg:ArmResult instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <ArmResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:response-val is deprecated.  Use ff_msgs-msg:response instead.")
  (response m))

(cl:ensure-generic-function 'fsm_result-val :lambda-list '(m))
(cl:defmethod fsm_result-val ((m <ArmResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_result-val is deprecated.  Use ff_msgs-msg:fsm_result instead.")
  (fsm_result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmResult>)))
    "Constants for message type '<ArmResult>"
  '((:SUCCESS . 1)
    (:PREEMPTED . 0)
    (:INVALID_COMMAND . -1)
    (:BAD_TILT_VALUE . -2)
    (:BAD_PAN_VALUE . -3)
    (:BAD_GRIPPER_VALUE . -4)
    (:NOT_ALLOWED . -5)
    (:TILT_FAILED . -6)
    (:PAN_FAILED . -7)
    (:GRIPPER_FAILED . -8)
    (:COMMUNICATION_ERROR . -9)
    (:COLLISION_AVOIDED . -10)
    (:ENABLE_FAILED . -11)
    (:DISABLE_FAILED . -12)
    (:CALIBRATE_FAILED . -13)
    (:NO_GOAL . -14))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmResult)))
    "Constants for message type 'ArmResult"
  '((:SUCCESS . 1)
    (:PREEMPTED . 0)
    (:INVALID_COMMAND . -1)
    (:BAD_TILT_VALUE . -2)
    (:BAD_PAN_VALUE . -3)
    (:BAD_GRIPPER_VALUE . -4)
    (:NOT_ALLOWED . -5)
    (:TILT_FAILED . -6)
    (:PAN_FAILED . -7)
    (:GRIPPER_FAILED . -8)
    (:COMMUNICATION_ERROR . -9)
    (:COLLISION_AVOIDED . -10)
    (:ENABLE_FAILED . -11)
    (:DISABLE_FAILED . -12)
    (:CALIBRATE_FAILED . -13)
    (:NO_GOAL . -14))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmResult>) ostream)
  "Serializes a message object of type '<ArmResult>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmResult>) istream)
  "Deserializes a message object of type '<ArmResult>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmResult>)))
  "Returns string type for a message object of type '<ArmResult>"
  "ff_msgs/ArmResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmResult)))
  "Returns string type for a message object of type 'ArmResult"
  "ff_msgs/ArmResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmResult>)))
  "Returns md5sum for a message object of type '<ArmResult>"
  "5c229b93f1064b9f1f7e8f3320eff359")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmResult)))
  "Returns md5sum for a message object of type 'ArmResult"
  "5c229b93f1064b9f1f7e8f3320eff359")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmResult>)))
  "Returns full string definition for message of type '<ArmResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Machine-readable reseult code~%int32 response~%int32 SUCCESS             =  1                # Successfully completed~%int32 PREEMPTED           =  0                # Action was preempted~%int32 INVALID_COMMAND     = -1                # Invalid command~%int32 BAD_TILT_VALUE      = -2                # Invalid value for tilt~%int32 BAD_PAN_VALUE       = -3                # Invalid value for pan~%int32 BAD_GRIPPER_VALUE   = -4                # Invalid value for gripper~%int32 NOT_ALLOWED         = -5                # Not allowed~%int32 TILT_FAILED         = -6                # Tilt command failed~%int32 PAN_FAILED          = -7                # Pan command failed~%int32 GRIPPER_FAILED      = -8                # Gripper command failed~%int32 COMMUNICATION_ERROR = -9                # Cannot communicate with arm~%int32 COLLISION_AVOIDED   = -10               # No panning when tilt < 90~%int32 ENABLE_FAILED       = -11               # Cannot enable the servos~%int32 DISABLE_FAILED      = -12               # Cannot disable the servos~%int32 CALIBRATE_FAILED    = -13               # Cannot calibrate the gripper~%int32 NO_GOAL             = -14               # Unknown call to calibration~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmResult)))
  "Returns full string definition for message of type 'ArmResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Machine-readable reseult code~%int32 response~%int32 SUCCESS             =  1                # Successfully completed~%int32 PREEMPTED           =  0                # Action was preempted~%int32 INVALID_COMMAND     = -1                # Invalid command~%int32 BAD_TILT_VALUE      = -2                # Invalid value for tilt~%int32 BAD_PAN_VALUE       = -3                # Invalid value for pan~%int32 BAD_GRIPPER_VALUE   = -4                # Invalid value for gripper~%int32 NOT_ALLOWED         = -5                # Not allowed~%int32 TILT_FAILED         = -6                # Tilt command failed~%int32 PAN_FAILED          = -7                # Pan command failed~%int32 GRIPPER_FAILED      = -8                # Gripper command failed~%int32 COMMUNICATION_ERROR = -9                # Cannot communicate with arm~%int32 COLLISION_AVOIDED   = -10               # No panning when tilt < 90~%int32 ENABLE_FAILED       = -11               # Cannot enable the servos~%int32 DISABLE_FAILED      = -12               # Cannot disable the servos~%int32 CALIBRATE_FAILED    = -13               # Cannot calibrate the gripper~%int32 NO_GOAL             = -14               # Unknown call to calibration~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'fsm_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmResult
    (cl:cons ':response (response msg))
    (cl:cons ':fsm_result (fsm_result msg))
))
