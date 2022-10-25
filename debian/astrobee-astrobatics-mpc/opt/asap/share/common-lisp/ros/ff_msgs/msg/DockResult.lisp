; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DockResult.msg.html

(cl:defclass <DockResult> (roslisp-msg-protocol:ros-message)
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

(cl:defclass DockResult (<DockResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DockResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DockResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DockResult> is deprecated: use ff_msgs-msg:DockResult instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <DockResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:response-val is deprecated.  Use ff_msgs-msg:response instead.")
  (response m))

(cl:ensure-generic-function 'fsm_result-val :lambda-list '(m))
(cl:defmethod fsm_result-val ((m <DockResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:fsm_result-val is deprecated.  Use ff_msgs-msg:fsm_result instead.")
  (fsm_result m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DockResult>)))
    "Constants for message type '<DockResult>"
  '((:CANCELLED . 5)
    (:ALREADY_DOCKED . 4)
    (:ALREADY_UNDOCKED . 3)
    (:UNDOCKED . 2)
    (:DOCKED . 1)
    (:PREEMPTED . 0)
    (:INVALID_COMMAND . -1)
    (:INVALID_BERTH . -2)
    (:NOT_IN_UNDOCKED_STATE . -3)
    (:NOT_IN_DOCKED_STATE . -4)
    (:SWITCH_TO_ML_FAILED . -5)
    (:SWITCH_TO_AR_FAILED . -6)
    (:SWITCH_TO_NO_FAILED . -7)
    (:PREP_DISABLE_FAILED . -8)
    (:PREP_ENABLE_FAILED . -9)
    (:MOTION_APPROACH_FAILED . -10)
    (:MOTION_COMPLETE_FAILED . -11)
    (:MOTION_ATTACHED_FAILED . -12)
    (:EPS_UNDOCK_FAILED . -13)
    (:EPS_DOCK_FAILED . -14)
    (:TOO_FAR_AWAY_FROM_APPROACH . -15))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DockResult)))
    "Constants for message type 'DockResult"
  '((:CANCELLED . 5)
    (:ALREADY_DOCKED . 4)
    (:ALREADY_UNDOCKED . 3)
    (:UNDOCKED . 2)
    (:DOCKED . 1)
    (:PREEMPTED . 0)
    (:INVALID_COMMAND . -1)
    (:INVALID_BERTH . -2)
    (:NOT_IN_UNDOCKED_STATE . -3)
    (:NOT_IN_DOCKED_STATE . -4)
    (:SWITCH_TO_ML_FAILED . -5)
    (:SWITCH_TO_AR_FAILED . -6)
    (:SWITCH_TO_NO_FAILED . -7)
    (:PREP_DISABLE_FAILED . -8)
    (:PREP_ENABLE_FAILED . -9)
    (:MOTION_APPROACH_FAILED . -10)
    (:MOTION_COMPLETE_FAILED . -11)
    (:MOTION_ATTACHED_FAILED . -12)
    (:EPS_UNDOCK_FAILED . -13)
    (:EPS_DOCK_FAILED . -14)
    (:TOO_FAR_AWAY_FROM_APPROACH . -15))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DockResult>) ostream)
  "Serializes a message object of type '<DockResult>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DockResult>) istream)
  "Deserializes a message object of type '<DockResult>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DockResult>)))
  "Returns string type for a message object of type '<DockResult>"
  "ff_msgs/DockResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DockResult)))
  "Returns string type for a message object of type 'DockResult"
  "ff_msgs/DockResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DockResult>)))
  "Returns md5sum for a message object of type '<DockResult>"
  "0cc69ac3a301c7996578d2ee3e9b92a6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DockResult)))
  "Returns md5sum for a message object of type 'DockResult"
  "0cc69ac3a301c7996578d2ee3e9b92a6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DockResult>)))
  "Returns full string definition for message of type '<DockResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Result~%int32 response~%int32 CANCELLED                          = 5~%int32 ALREADY_DOCKED                     = 4~%int32 ALREADY_UNDOCKED                   = 3~%int32 UNDOCKED                           = 2~%int32 DOCKED                             = 1~%int32 PREEMPTED                          = 0~%int32 INVALID_COMMAND                    = -1~%int32 INVALID_BERTH                      = -2~%int32 NOT_IN_UNDOCKED_STATE              = -3~%int32 NOT_IN_DOCKED_STATE                = -4~%int32 SWITCH_TO_ML_FAILED                = -5~%int32 SWITCH_TO_AR_FAILED                = -6~%int32 SWITCH_TO_NO_FAILED                = -7~%int32 PREP_DISABLE_FAILED                = -8~%int32 PREP_ENABLE_FAILED                 = -9~%int32 MOTION_APPROACH_FAILED             = -10~%int32 MOTION_COMPLETE_FAILED             = -11~%int32 MOTION_ATTACHED_FAILED             = -12~%int32 EPS_UNDOCK_FAILED                  = -13~%int32 EPS_DOCK_FAILED                    = -14~%int32 TOO_FAR_AWAY_FROM_APPROACH         = -15~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DockResult)))
  "Returns full string definition for message of type 'DockResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Result~%int32 response~%int32 CANCELLED                          = 5~%int32 ALREADY_DOCKED                     = 4~%int32 ALREADY_UNDOCKED                   = 3~%int32 UNDOCKED                           = 2~%int32 DOCKED                             = 1~%int32 PREEMPTED                          = 0~%int32 INVALID_COMMAND                    = -1~%int32 INVALID_BERTH                      = -2~%int32 NOT_IN_UNDOCKED_STATE              = -3~%int32 NOT_IN_DOCKED_STATE                = -4~%int32 SWITCH_TO_ML_FAILED                = -5~%int32 SWITCH_TO_AR_FAILED                = -6~%int32 SWITCH_TO_NO_FAILED                = -7~%int32 PREP_DISABLE_FAILED                = -8~%int32 PREP_ENABLE_FAILED                 = -9~%int32 MOTION_APPROACH_FAILED             = -10~%int32 MOTION_COMPLETE_FAILED             = -11~%int32 MOTION_ATTACHED_FAILED             = -12~%int32 EPS_UNDOCK_FAILED                  = -13~%int32 EPS_DOCK_FAILED                    = -14~%int32 TOO_FAR_AWAY_FROM_APPROACH         = -15~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DockResult>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'fsm_result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DockResult>))
  "Converts a ROS message object to a list"
  (cl:list 'DockResult
    (cl:cons ':response (response msg))
    (cl:cons ':fsm_result (fsm_result msg))
))
