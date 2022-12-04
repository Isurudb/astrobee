; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmActionResult.msg.html

(cl:defclass <ArmActionResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type actionlib_msgs-msg:GoalStatus
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalStatus))
   (result
    :reader result
    :initarg :result
    :type ff_msgs-msg:ArmResult
    :initform (cl:make-instance 'ff_msgs-msg:ArmResult)))
)

(cl:defclass ArmActionResult (<ArmActionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmActionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmActionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmActionResult> is deprecated: use ff_msgs-msg:ArmActionResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArmActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ArmActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:status-val is deprecated.  Use ff_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ArmActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:result-val is deprecated.  Use ff_msgs-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmActionResult>) ostream)
  "Serializes a message object of type '<ArmActionResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmActionResult>) istream)
  "Deserializes a message object of type '<ArmActionResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmActionResult>)))
  "Returns string type for a message object of type '<ArmActionResult>"
  "ff_msgs/ArmActionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmActionResult)))
  "Returns string type for a message object of type 'ArmActionResult"
  "ff_msgs/ArmActionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmActionResult>)))
  "Returns md5sum for a message object of type '<ArmActionResult>"
  "b079a56e922752b036e0c53789575ff3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmActionResult)))
  "Returns md5sum for a message object of type 'ArmActionResult"
  "b079a56e922752b036e0c53789575ff3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmActionResult>)))
  "Returns full string definition for message of type '<ArmActionResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ArmResult result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ff_msgs/ArmResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Machine-readable reseult code~%int32 response~%int32 SUCCESS             =  1                # Successfully completed~%int32 PREEMPTED           =  0                # Action was preempted~%int32 INVALID_COMMAND     = -1                # Invalid command~%int32 BAD_TILT_VALUE      = -2                # Invalid value for tilt~%int32 BAD_PAN_VALUE       = -3                # Invalid value for pan~%int32 BAD_GRIPPER_VALUE   = -4                # Invalid value for gripper~%int32 NOT_ALLOWED         = -5                # Not allowed~%int32 TILT_FAILED         = -6                # Tilt command failed~%int32 PAN_FAILED          = -7                # Pan command failed~%int32 GRIPPER_FAILED      = -8                # Gripper command failed~%int32 COMMUNICATION_ERROR = -9                # Cannot communicate with arm~%int32 COLLISION_AVOIDED   = -10               # No panning when tilt < 90~%int32 ENABLE_FAILED       = -11               # Cannot enable the servos~%int32 DISABLE_FAILED      = -12               # Cannot disable the servos~%int32 CALIBRATE_FAILED    = -13               # Cannot calibrate the gripper~%int32 NO_GOAL             = -14               # Unknown call to calibration~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmActionResult)))
  "Returns full string definition for message of type 'ArmActionResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ArmResult result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ff_msgs/ArmResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Machine-readable reseult code~%int32 response~%int32 SUCCESS             =  1                # Successfully completed~%int32 PREEMPTED           =  0                # Action was preempted~%int32 INVALID_COMMAND     = -1                # Invalid command~%int32 BAD_TILT_VALUE      = -2                # Invalid value for tilt~%int32 BAD_PAN_VALUE       = -3                # Invalid value for pan~%int32 BAD_GRIPPER_VALUE   = -4                # Invalid value for gripper~%int32 NOT_ALLOWED         = -5                # Not allowed~%int32 TILT_FAILED         = -6                # Tilt command failed~%int32 PAN_FAILED          = -7                # Pan command failed~%int32 GRIPPER_FAILED      = -8                # Gripper command failed~%int32 COMMUNICATION_ERROR = -9                # Cannot communicate with arm~%int32 COLLISION_AVOIDED   = -10               # No panning when tilt < 90~%int32 ENABLE_FAILED       = -11               # Cannot enable the servos~%int32 DISABLE_FAILED      = -12               # Cannot disable the servos~%int32 CALIBRATE_FAILED    = -13               # Cannot calibrate the gripper~%int32 NO_GOAL             = -14               # Unknown call to calibration~%~%# Human readable FSM result for debugging~%string fsm_result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmActionResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmActionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmActionResult
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':result (result msg))
))