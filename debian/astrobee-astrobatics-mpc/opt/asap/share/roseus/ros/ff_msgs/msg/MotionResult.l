;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::MotionResult)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'MotionResult (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::MOTIONRESULT")
  (make-package "FF_MSGS::MOTIONRESULT"))

(in-package "ROS")
;;//! \htmlinclude MotionResult.msg.html


(intern "*ALREADY_THERE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*ALREADY_THERE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*ALREADY_THERE* 2)
(intern "*SUCCESS*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*SUCCESS* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*SUCCESS* 1)
(intern "*PREEMPTED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*PREEMPTED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*PREEMPTED* 0)
(intern "*PLAN_FAILED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*PLAN_FAILED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*PLAN_FAILED* -1)
(intern "*VALIDATE_FAILED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VALIDATE_FAILED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VALIDATE_FAILED* -2)
(intern "*PMC_FAILED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*PMC_FAILED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*PMC_FAILED* -3)
(intern "*CONTROL_FAILED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*CONTROL_FAILED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*CONTROL_FAILED* -4)
(intern "*OBSTACLE_DETECTED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*OBSTACLE_DETECTED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*OBSTACLE_DETECTED* -5)
(intern "*REPLAN_NOT_ENOUGH_TIME*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*REPLAN_NOT_ENOUGH_TIME* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*REPLAN_NOT_ENOUGH_TIME* -6)
(intern "*REPLAN_FAILED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*REPLAN_FAILED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*REPLAN_FAILED* -7)
(intern "*REVALIDATE_FAILED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*REVALIDATE_FAILED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*REVALIDATE_FAILED* -8)
(intern "*NOT_IN_WAITING_MODE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*NOT_IN_WAITING_MODE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*NOT_IN_WAITING_MODE* -9)
(intern "*INVALID_FLIGHT_MODE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*INVALID_FLIGHT_MODE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*INVALID_FLIGHT_MODE* -10)
(intern "*UNEXPECTED_EMPTY_SEGMENT*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*UNEXPECTED_EMPTY_SEGMENT* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*UNEXPECTED_EMPTY_SEGMENT* -11)
(intern "*COULD_NOT_RESAMPLE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*COULD_NOT_RESAMPLE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*COULD_NOT_RESAMPLE* -12)
(intern "*UNEXPECTED_EMPTY_STATES*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*UNEXPECTED_EMPTY_STATES* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*UNEXPECTED_EMPTY_STATES* -13)
(intern "*INVALID_COMMAND*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*INVALID_COMMAND* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*INVALID_COMMAND* -14)
(intern "*CANNOT_QUERY_ROBOT_POSE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*CANNOT_QUERY_ROBOT_POSE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*CANNOT_QUERY_ROBOT_POSE* -15)
(intern "*NOT_ON_FIRST_POSE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*NOT_ON_FIRST_POSE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*NOT_ON_FIRST_POSE* -16)
(intern "*BAD_DESIRED_VELOCITY*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*BAD_DESIRED_VELOCITY* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*BAD_DESIRED_VELOCITY* -17)
(intern "*BAD_DESIRED_ACCELERATION*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*BAD_DESIRED_ACCELERATION* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*BAD_DESIRED_ACCELERATION* -18)
(intern "*BAD_DESIRED_OMEGA*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*BAD_DESIRED_OMEGA* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*BAD_DESIRED_OMEGA* -19)
(intern "*BAD_DESIRED_ALPHA*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*BAD_DESIRED_ALPHA* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*BAD_DESIRED_ALPHA* -20)
(intern "*BAD_DESIRED_RATE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*BAD_DESIRED_RATE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*BAD_DESIRED_RATE* -21)
(intern "*TOLERANCE_VIOLATION_POSITION_ENDPOINT*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*TOLERANCE_VIOLATION_POSITION_ENDPOINT* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*TOLERANCE_VIOLATION_POSITION_ENDPOINT* -22)
(intern "*TOLERANCE_VIOLATION_POSITION*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*TOLERANCE_VIOLATION_POSITION* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*TOLERANCE_VIOLATION_POSITION* -23)
(intern "*TOLERANCE_VIOLATION_ATTITUDE*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*TOLERANCE_VIOLATION_ATTITUDE* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*TOLERANCE_VIOLATION_ATTITUDE* -24)
(intern "*TOLERANCE_VIOLATION_VELOCITY*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*TOLERANCE_VIOLATION_VELOCITY* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*TOLERANCE_VIOLATION_VELOCITY* -25)
(intern "*TOLERANCE_VIOLATION_OMEGA*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*TOLERANCE_VIOLATION_OMEGA* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*TOLERANCE_VIOLATION_OMEGA* -26)
(intern "*VIOLATES_RESAMPLING*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_RESAMPLING* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_RESAMPLING* -27)
(intern "*VIOLATES_KEEP_OUT*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_KEEP_OUT* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_KEEP_OUT* -28)
(intern "*VIOLATES_KEEP_IN*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_KEEP_IN* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_KEEP_IN* -29)
(intern "*VIOLATES_MINIMUM_FREQUENCY*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_MINIMUM_FREQUENCY* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_MINIMUM_FREQUENCY* -30)
(intern "*VIOLATES_STATIONARY_ENDPOINT*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_STATIONARY_ENDPOINT* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_STATIONARY_ENDPOINT* -31)
(intern "*VIOLATES_FIRST_IN_PAST*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_FIRST_IN_PAST* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_FIRST_IN_PAST* -32)
(intern "*VIOLATES_MINIMUM_SETPOINTS*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_MINIMUM_SETPOINTS* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_MINIMUM_SETPOINTS* -33)
(intern "*VIOLATES_HARD_LIMIT_VEL*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_HARD_LIMIT_VEL* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_HARD_LIMIT_VEL* -34)
(intern "*VIOLATES_HARD_LIMIT_ACCEL*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_HARD_LIMIT_ACCEL* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_HARD_LIMIT_ACCEL* -35)
(intern "*VIOLATES_HARD_LIMIT_OMEGA*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_HARD_LIMIT_OMEGA* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_HARD_LIMIT_OMEGA* -36)
(intern "*VIOLATES_HARD_LIMIT_ALPHA*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*VIOLATES_HARD_LIMIT_ALPHA* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*VIOLATES_HARD_LIMIT_ALPHA* -37)
(intern "*CANCELLED*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*CANCELLED* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*CANCELLED* -38)
(intern "*INVALID_REFERENCE_FRAME*" (find-package "FF_MSGS::MOTIONRESULT"))
(shadow '*INVALID_REFERENCE_FRAME* (find-package "FF_MSGS::MOTIONRESULT"))
(defconstant ff_msgs::MotionResult::*INVALID_REFERENCE_FRAME* -39)
(defclass ff_msgs::MotionResult
  :super ros::object
  :slots (_response _fsm_result _flight_mode _segment ))

(defmethod ff_msgs::MotionResult
  (:init
   (&key
    ((:response __response) 0)
    ((:fsm_result __fsm_result) "")
    ((:flight_mode __flight_mode) (instance ff_msgs::FlightMode :init))
    ((:segment __segment) (let (r) (dotimes (i 0) (push (instance ff_msgs::ControlState :init) r)) r))
    )
   (send-super :init)
   (setq _response (round __response))
   (setq _fsm_result (string __fsm_result))
   (setq _flight_mode __flight_mode)
   (setq _segment __segment)
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:fsm_result
   (&optional __fsm_result)
   (if __fsm_result (setq _fsm_result __fsm_result)) _fsm_result)
  (:flight_mode
   (&rest __flight_mode)
   (if (keywordp (car __flight_mode))
       (send* _flight_mode __flight_mode)
     (progn
       (if __flight_mode (setq _flight_mode (car __flight_mode)))
       _flight_mode)))
  (:segment
   (&rest __segment)
   (if (keywordp (car __segment))
       (send* _segment __segment)
     (progn
       (if __segment (setq _segment (car __segment)))
       _segment)))
  (:serialization-length
   ()
   (+
    ;; int32 _response
    4
    ;; string _fsm_result
    4 (length _fsm_result)
    ;; ff_msgs/FlightMode _flight_mode
    (send _flight_mode :serialization-length)
    ;; ff_msgs/ControlState[] _segment
    (apply #'+ (send-all _segment :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _response
       (write-long _response s)
     ;; string _fsm_result
       (write-long (length _fsm_result) s) (princ _fsm_result s)
     ;; ff_msgs/FlightMode _flight_mode
       (send _flight_mode :serialize s)
     ;; ff_msgs/ControlState[] _segment
     (write-long (length _segment) s)
     (dolist (elem _segment)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _response
     (setq _response (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _fsm_result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fsm_result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/FlightMode _flight_mode
     (send _flight_mode :deserialize buf ptr-) (incf ptr- (send _flight_mode :serialization-length))
   ;; ff_msgs/ControlState[] _segment
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _segment (let (r) (dotimes (i n) (push (instance ff_msgs::ControlState :init) r)) r))
     (dolist (elem- _segment)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::MotionResult :md5sum-) "d9085ddc2b12a9c56a85552d7ec1a05e")
(setf (get ff_msgs::MotionResult :datatype-) "ff_msgs/MotionResult")
(setf (get ff_msgs::MotionResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

# Motion result
int32 response                            # Motion action response
int32 ALREADY_THERE                         =   2  # MOVE: We are already at the location
int32 SUCCESS                               =   1  # ALL: Motion succeeded
int32 PREEMPTED                             =   0  # ALL: Motion preempted by thirdparty
int32 PLAN_FAILED                           =  -1  # MOVE/EXEC: Plan/bootstrap failed
int32 VALIDATE_FAILED                       =  -2  # MOVE/EXEC: No comms with mapper
int32 PMC_FAILED                            =  -3  # MOVE/EXEC: PMC failed
int32 CONTROL_FAILED                        =  -4  # ALL: Control failed
int32 OBSTACLE_DETECTED                     =  -5  # ALL: Obstacle / replan disabled
int32 REPLAN_NOT_ENOUGH_TIME                =  -6  # MOVE/EXEC: Not enough time to replan
int32 REPLAN_FAILED                         =  -7  # MOVE/EXEC: Replanning failed
int32 REVALIDATE_FAILED                     =  -8  # MOVE/EXEC: Revalidating failed
int32 NOT_IN_WAITING_MODE                   =  -9  # ALL: Internal failure
int32 INVALID_FLIGHT_MODE                   =  -10 # ALL: No flight mode specified
int32 UNEXPECTED_EMPTY_SEGMENT              =  -11 # EXEC: Segment empty
int32 COULD_NOT_RESAMPLE                    =  -12 # EXEC: Could not resample segment
int32 UNEXPECTED_EMPTY_STATES               =  -13 # MOVE: State vector empty
int32 INVALID_COMMAND                       =  -14 # Command rejected
int32 CANNOT_QUERY_ROBOT_POSE               =  -15 # TF2 failed to find the current pose
int32 NOT_ON_FIRST_POSE                     =  -16 # EXEC: Not on first pose of exec
int32 BAD_DESIRED_VELOCITY                  =  -17 # Requested vel too high
int32 BAD_DESIRED_ACCELERATION              =  -18 # Requested accel too high
int32 BAD_DESIRED_OMEGA                     =  -19 # Requested omega too high
int32 BAD_DESIRED_ALPHA                     =  -20 # Requested alpha too high
int32 BAD_DESIRED_RATE                      =  -21 # Requested rate too low
int32 TOLERANCE_VIOLATION_POSITION_ENDPOINT =  -22 # Position tolerance violated
int32 TOLERANCE_VIOLATION_POSITION          =  -23 # Position tolerance violated
int32 TOLERANCE_VIOLATION_ATTITUDE          =  -24 # Attitude tolerance violated
int32 TOLERANCE_VIOLATION_VELOCITY          =  -25 # Velocity tolerance violated
int32 TOLERANCE_VIOLATION_OMEGA             =  -26 # Omega tolerance violated
int32 VIOLATES_RESAMPLING                   =  -27 # Validation: could not resample@10Hz
int32 VIOLATES_KEEP_OUT                     =  -28 # Validation: Keep out violation
int32 VIOLATES_KEEP_IN                      =  -29 # Validation: Keep in violation
int32 VIOLATES_MINIMUM_FREQUENCY            =  -30 # Validation: Sample frequency too low
int32 VIOLATES_STATIONARY_ENDPOINT          =  -31 # Validation: Last setpoint not static
int32 VIOLATES_FIRST_IN_PAST                =  -32 # Validation: First timestamp in past
int32 VIOLATES_MINIMUM_SETPOINTS            =  -33 # Validation: Not enough setpoints
int32 VIOLATES_HARD_LIMIT_VEL               =  -34 # Validation: Velocity too high
int32 VIOLATES_HARD_LIMIT_ACCEL             =  -35 # Validation: Acceleration too high
int32 VIOLATES_HARD_LIMIT_OMEGA             =  -36 # Validation: Omega too high
int32 VIOLATES_HARD_LIMIT_ALPHA             =  -37 # Validation: Alpha too high
int32 CANCELLED                             =  -38 # ALL: Motion cancelled by callee
int32 INVALID_REFERENCE_FRAME               =  -39 # ALL: Unknown reference frame

# Human readable FSM result for debugging
string fsm_result

# The flight mode parameters used
ff_msgs/FlightMode flight_mode

# The final segment that was flown
ff_msgs/ControlState[] segment


================================================================================
MSG: ff_msgs/FlightMode
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# This message captures all information in a flight mode

Header header                     # Metadata

string name                       # Name of the flight mode

bool control_enabled              # Is control enabled?

# Tolerances (all in SI units)
float32 tolerance_pos_endpoint    # Endpoint position tolerance in m
float32 tolerance_pos             # Position tolerance in m
float32 tolerance_vel             # Velocity tolerance in m/s
float32 tolerance_att             # Attitude tolerance in rads
float32 tolerance_omega           # Angular acceleration tolerance in rad/s
float32 tolerance_time            # Acceptable lag betwee TX and RX of control

# Controller gains
geometry_msgs/Vector3 att_kp      # Positional proportional constant
geometry_msgs/Vector3 att_ki      # Positional integrative constant
geometry_msgs/Vector3 omega_kd    # Attidue derivative constant
geometry_msgs/Vector3 pos_kp      # Positional proportional contant
geometry_msgs/Vector3 pos_ki      # Positional integrative constant
geometry_msgs/Vector3 vel_kd      # Positional derivative constant

# Hard limit on planning
float32 hard_limit_vel            # Position tolerance in m/s
float32 hard_limit_accel          # Position tolerance in m/s^2
float32 hard_limit_omega          # Position tolerance in rads/s
float32 hard_limit_alpha          # Position tolerance in rads/s^2

# Impeller speed
uint8 speed                       # Current speed gain
uint8 SPEED_MIN        = 0        # Min acceptable gain
uint8 SPEED_OFF        = 0        # Blowers off
uint8 SPEED_QUIET      = 1        # Quiet mode
uint8 SPEED_NOMINAL    = 2        # Nomainal mode
uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode
uint8 SPEED_MAX        = 3        # Max acceptable gain

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: ff_msgs/ControlState
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Full state vector containing Time, Pose, Vel, and Accel
# 
# when {time}
# flight_mode {string} - disctates, gains, tolerances, etc.
# pose {Point position, Quaternion orientation}
# twist {Vector3 linear, Vector3 angular}
# accel {Vector3 linear, Vector3 angular}

time when
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Twist accel

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

")



(provide :ff_msgs/MotionResult "d9085ddc2b12a9c56a85552d7ec1a05e")


