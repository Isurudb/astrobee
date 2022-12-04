; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude MotionActionResult.msg.html

(cl:defclass <MotionActionResult> (roslisp-msg-protocol:ros-message)
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
    :type ff_msgs-msg:MotionResult
    :initform (cl:make-instance 'ff_msgs-msg:MotionResult)))
)

(cl:defclass MotionActionResult (<MotionActionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotionActionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotionActionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<MotionActionResult> is deprecated: use ff_msgs-msg:MotionActionResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MotionActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <MotionActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:status-val is deprecated.  Use ff_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <MotionActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:result-val is deprecated.  Use ff_msgs-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotionActionResult>) ostream)
  "Serializes a message object of type '<MotionActionResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotionActionResult>) istream)
  "Deserializes a message object of type '<MotionActionResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotionActionResult>)))
  "Returns string type for a message object of type '<MotionActionResult>"
  "ff_msgs/MotionActionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotionActionResult)))
  "Returns string type for a message object of type 'MotionActionResult"
  "ff_msgs/MotionActionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotionActionResult>)))
  "Returns md5sum for a message object of type '<MotionActionResult>"
  "f293c504a9826fc31b42f590718e0dec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotionActionResult)))
  "Returns md5sum for a message object of type 'MotionActionResult"
  "f293c504a9826fc31b42f590718e0dec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotionActionResult>)))
  "Returns full string definition for message of type '<MotionActionResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%MotionResult result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ff_msgs/MotionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Motion result~%int32 response                            # Motion action response~%int32 ALREADY_THERE                         =   2  # MOVE: We are already at the location~%int32 SUCCESS                               =   1  # ALL: Motion succeeded~%int32 PREEMPTED                             =   0  # ALL: Motion preempted by thirdparty~%int32 PLAN_FAILED                           =  -1  # MOVE/EXEC: Plan/bootstrap failed~%int32 VALIDATE_FAILED                       =  -2  # MOVE/EXEC: No comms with mapper~%int32 PMC_FAILED                            =  -3  # MOVE/EXEC: PMC failed~%int32 CONTROL_FAILED                        =  -4  # ALL: Control failed~%int32 OBSTACLE_DETECTED                     =  -5  # ALL: Obstacle / replan disabled~%int32 REPLAN_NOT_ENOUGH_TIME                =  -6  # MOVE/EXEC: Not enough time to replan~%int32 REPLAN_FAILED                         =  -7  # MOVE/EXEC: Replanning failed~%int32 REVALIDATE_FAILED                     =  -8  # MOVE/EXEC: Revalidating failed~%int32 NOT_IN_WAITING_MODE                   =  -9  # ALL: Internal failure~%int32 INVALID_FLIGHT_MODE                   =  -10 # ALL: No flight mode specified~%int32 UNEXPECTED_EMPTY_SEGMENT              =  -11 # EXEC: Segment empty~%int32 COULD_NOT_RESAMPLE                    =  -12 # EXEC: Could not resample segment~%int32 UNEXPECTED_EMPTY_STATES               =  -13 # MOVE: State vector empty~%int32 INVALID_COMMAND                       =  -14 # Command rejected~%int32 CANNOT_QUERY_ROBOT_POSE               =  -15 # TF2 failed to find the current pose~%int32 NOT_ON_FIRST_POSE                     =  -16 # EXEC: Not on first pose of exec~%int32 BAD_DESIRED_VELOCITY                  =  -17 # Requested vel too high~%int32 BAD_DESIRED_ACCELERATION              =  -18 # Requested accel too high~%int32 BAD_DESIRED_OMEGA                     =  -19 # Requested omega too high~%int32 BAD_DESIRED_ALPHA                     =  -20 # Requested alpha too high~%int32 BAD_DESIRED_RATE                      =  -21 # Requested rate too low~%int32 TOLERANCE_VIOLATION_POSITION_ENDPOINT =  -22 # Position tolerance violated~%int32 TOLERANCE_VIOLATION_POSITION          =  -23 # Position tolerance violated~%int32 TOLERANCE_VIOLATION_ATTITUDE          =  -24 # Attitude tolerance violated~%int32 TOLERANCE_VIOLATION_VELOCITY          =  -25 # Velocity tolerance violated~%int32 TOLERANCE_VIOLATION_OMEGA             =  -26 # Omega tolerance violated~%int32 VIOLATES_RESAMPLING                   =  -27 # Validation: could not resample@10Hz~%int32 VIOLATES_KEEP_OUT                     =  -28 # Validation: Keep out violation~%int32 VIOLATES_KEEP_IN                      =  -29 # Validation: Keep in violation~%int32 VIOLATES_MINIMUM_FREQUENCY            =  -30 # Validation: Sample frequency too low~%int32 VIOLATES_STATIONARY_ENDPOINT          =  -31 # Validation: Last setpoint not static~%int32 VIOLATES_FIRST_IN_PAST                =  -32 # Validation: First timestamp in past~%int32 VIOLATES_MINIMUM_SETPOINTS            =  -33 # Validation: Not enough setpoints~%int32 VIOLATES_HARD_LIMIT_VEL               =  -34 # Validation: Velocity too high~%int32 VIOLATES_HARD_LIMIT_ACCEL             =  -35 # Validation: Acceleration too high~%int32 VIOLATES_HARD_LIMIT_OMEGA             =  -36 # Validation: Omega too high~%int32 VIOLATES_HARD_LIMIT_ALPHA             =  -37 # Validation: Alpha too high~%int32 CANCELLED                             =  -38 # ALL: Motion cancelled by callee~%int32 INVALID_REFERENCE_FRAME               =  -39 # ALL: Unknown reference frame~%~%# Human readable FSM result for debugging~%string fsm_result~%~%# The flight mode parameters used~%ff_msgs/FlightMode flight_mode~%~%# The final segment that was flown~%ff_msgs/ControlState[] segment~%~%~%================================================================================~%MSG: ff_msgs/FlightMode~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message captures all information in a flight mode~%~%Header header                     # Metadata~%~%string name                       # Name of the flight mode~%~%bool control_enabled              # Is control enabled?~%~%# Tolerances (all in SI units)~%float32 tolerance_pos_endpoint    # Endpoint position tolerance in m~%float32 tolerance_pos             # Position tolerance in m~%float32 tolerance_vel             # Velocity tolerance in m/s~%float32 tolerance_att             # Attitude tolerance in rads~%float32 tolerance_omega           # Angular acceleration tolerance in rad/s~%float32 tolerance_time            # Acceptable lag betwee TX and RX of control~%~%# Controller gains~%geometry_msgs/Vector3 att_kp      # Positional proportional constant~%geometry_msgs/Vector3 att_ki      # Positional integrative constant~%geometry_msgs/Vector3 omega_kd    # Attidue derivative constant~%geometry_msgs/Vector3 pos_kp      # Positional proportional contant~%geometry_msgs/Vector3 pos_ki      # Positional integrative constant~%geometry_msgs/Vector3 vel_kd      # Positional derivative constant~%~%# Hard limit on planning~%float32 hard_limit_vel            # Position tolerance in m/s~%float32 hard_limit_accel          # Position tolerance in m/s^2~%float32 hard_limit_omega          # Position tolerance in rads/s~%float32 hard_limit_alpha          # Position tolerance in rads/s^2~%~%# Impeller speed~%uint8 speed                       # Current speed gain~%uint8 SPEED_MIN        = 0        # Min acceptable gain~%uint8 SPEED_OFF        = 0        # Blowers off~%uint8 SPEED_QUIET      = 1        # Quiet mode~%uint8 SPEED_NOMINAL    = 2        # Nomainal mode~%uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode~%uint8 SPEED_MAX        = 3        # Max acceptable gain~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotionActionResult)))
  "Returns full string definition for message of type 'MotionActionResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%MotionResult result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: ff_msgs/MotionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# Motion result~%int32 response                            # Motion action response~%int32 ALREADY_THERE                         =   2  # MOVE: We are already at the location~%int32 SUCCESS                               =   1  # ALL: Motion succeeded~%int32 PREEMPTED                             =   0  # ALL: Motion preempted by thirdparty~%int32 PLAN_FAILED                           =  -1  # MOVE/EXEC: Plan/bootstrap failed~%int32 VALIDATE_FAILED                       =  -2  # MOVE/EXEC: No comms with mapper~%int32 PMC_FAILED                            =  -3  # MOVE/EXEC: PMC failed~%int32 CONTROL_FAILED                        =  -4  # ALL: Control failed~%int32 OBSTACLE_DETECTED                     =  -5  # ALL: Obstacle / replan disabled~%int32 REPLAN_NOT_ENOUGH_TIME                =  -6  # MOVE/EXEC: Not enough time to replan~%int32 REPLAN_FAILED                         =  -7  # MOVE/EXEC: Replanning failed~%int32 REVALIDATE_FAILED                     =  -8  # MOVE/EXEC: Revalidating failed~%int32 NOT_IN_WAITING_MODE                   =  -9  # ALL: Internal failure~%int32 INVALID_FLIGHT_MODE                   =  -10 # ALL: No flight mode specified~%int32 UNEXPECTED_EMPTY_SEGMENT              =  -11 # EXEC: Segment empty~%int32 COULD_NOT_RESAMPLE                    =  -12 # EXEC: Could not resample segment~%int32 UNEXPECTED_EMPTY_STATES               =  -13 # MOVE: State vector empty~%int32 INVALID_COMMAND                       =  -14 # Command rejected~%int32 CANNOT_QUERY_ROBOT_POSE               =  -15 # TF2 failed to find the current pose~%int32 NOT_ON_FIRST_POSE                     =  -16 # EXEC: Not on first pose of exec~%int32 BAD_DESIRED_VELOCITY                  =  -17 # Requested vel too high~%int32 BAD_DESIRED_ACCELERATION              =  -18 # Requested accel too high~%int32 BAD_DESIRED_OMEGA                     =  -19 # Requested omega too high~%int32 BAD_DESIRED_ALPHA                     =  -20 # Requested alpha too high~%int32 BAD_DESIRED_RATE                      =  -21 # Requested rate too low~%int32 TOLERANCE_VIOLATION_POSITION_ENDPOINT =  -22 # Position tolerance violated~%int32 TOLERANCE_VIOLATION_POSITION          =  -23 # Position tolerance violated~%int32 TOLERANCE_VIOLATION_ATTITUDE          =  -24 # Attitude tolerance violated~%int32 TOLERANCE_VIOLATION_VELOCITY          =  -25 # Velocity tolerance violated~%int32 TOLERANCE_VIOLATION_OMEGA             =  -26 # Omega tolerance violated~%int32 VIOLATES_RESAMPLING                   =  -27 # Validation: could not resample@10Hz~%int32 VIOLATES_KEEP_OUT                     =  -28 # Validation: Keep out violation~%int32 VIOLATES_KEEP_IN                      =  -29 # Validation: Keep in violation~%int32 VIOLATES_MINIMUM_FREQUENCY            =  -30 # Validation: Sample frequency too low~%int32 VIOLATES_STATIONARY_ENDPOINT          =  -31 # Validation: Last setpoint not static~%int32 VIOLATES_FIRST_IN_PAST                =  -32 # Validation: First timestamp in past~%int32 VIOLATES_MINIMUM_SETPOINTS            =  -33 # Validation: Not enough setpoints~%int32 VIOLATES_HARD_LIMIT_VEL               =  -34 # Validation: Velocity too high~%int32 VIOLATES_HARD_LIMIT_ACCEL             =  -35 # Validation: Acceleration too high~%int32 VIOLATES_HARD_LIMIT_OMEGA             =  -36 # Validation: Omega too high~%int32 VIOLATES_HARD_LIMIT_ALPHA             =  -37 # Validation: Alpha too high~%int32 CANCELLED                             =  -38 # ALL: Motion cancelled by callee~%int32 INVALID_REFERENCE_FRAME               =  -39 # ALL: Unknown reference frame~%~%# Human readable FSM result for debugging~%string fsm_result~%~%# The flight mode parameters used~%ff_msgs/FlightMode flight_mode~%~%# The final segment that was flown~%ff_msgs/ControlState[] segment~%~%~%================================================================================~%MSG: ff_msgs/FlightMode~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message captures all information in a flight mode~%~%Header header                     # Metadata~%~%string name                       # Name of the flight mode~%~%bool control_enabled              # Is control enabled?~%~%# Tolerances (all in SI units)~%float32 tolerance_pos_endpoint    # Endpoint position tolerance in m~%float32 tolerance_pos             # Position tolerance in m~%float32 tolerance_vel             # Velocity tolerance in m/s~%float32 tolerance_att             # Attitude tolerance in rads~%float32 tolerance_omega           # Angular acceleration tolerance in rad/s~%float32 tolerance_time            # Acceptable lag betwee TX and RX of control~%~%# Controller gains~%geometry_msgs/Vector3 att_kp      # Positional proportional constant~%geometry_msgs/Vector3 att_ki      # Positional integrative constant~%geometry_msgs/Vector3 omega_kd    # Attidue derivative constant~%geometry_msgs/Vector3 pos_kp      # Positional proportional contant~%geometry_msgs/Vector3 pos_ki      # Positional integrative constant~%geometry_msgs/Vector3 vel_kd      # Positional derivative constant~%~%# Hard limit on planning~%float32 hard_limit_vel            # Position tolerance in m/s~%float32 hard_limit_accel          # Position tolerance in m/s^2~%float32 hard_limit_omega          # Position tolerance in rads/s~%float32 hard_limit_alpha          # Position tolerance in rads/s^2~%~%# Impeller speed~%uint8 speed                       # Current speed gain~%uint8 SPEED_MIN        = 0        # Min acceptable gain~%uint8 SPEED_OFF        = 0        # Blowers off~%uint8 SPEED_QUIET      = 1        # Quiet mode~%uint8 SPEED_NOMINAL    = 2        # Nomainal mode~%uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode~%uint8 SPEED_MAX        = 3        # Max acceptable gain~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotionActionResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotionActionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'MotionActionResult
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':result (result msg))
))