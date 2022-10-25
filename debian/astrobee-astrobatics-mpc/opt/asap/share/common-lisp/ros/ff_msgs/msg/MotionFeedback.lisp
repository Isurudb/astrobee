; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude MotionFeedback.msg.html

(cl:defclass <MotionFeedback> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type ff_msgs-msg:MotionState
    :initform (cl:make-instance 'ff_msgs-msg:MotionState))
   (progress
    :reader progress
    :initarg :progress
    :type ff_msgs-msg:ControlFeedback
    :initform (cl:make-instance 'ff_msgs-msg:ControlFeedback))
   (perc_complete
    :reader perc_complete
    :initarg :perc_complete
    :type cl:float
    :initform 0.0)
   (secs_remaining
    :reader secs_remaining
    :initarg :secs_remaining
    :type cl:float
    :initform 0.0))
)

(cl:defclass MotionFeedback (<MotionFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotionFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotionFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<MotionFeedback> is deprecated: use ff_msgs-msg:MotionFeedback instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <MotionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:state-val is deprecated.  Use ff_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <MotionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:progress-val is deprecated.  Use ff_msgs-msg:progress instead.")
  (progress m))

(cl:ensure-generic-function 'perc_complete-val :lambda-list '(m))
(cl:defmethod perc_complete-val ((m <MotionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:perc_complete-val is deprecated.  Use ff_msgs-msg:perc_complete instead.")
  (perc_complete m))

(cl:ensure-generic-function 'secs_remaining-val :lambda-list '(m))
(cl:defmethod secs_remaining-val ((m <MotionFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:secs_remaining-val is deprecated.  Use ff_msgs-msg:secs_remaining instead.")
  (secs_remaining m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotionFeedback>) ostream)
  "Serializes a message object of type '<MotionFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'progress) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'perc_complete))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'secs_remaining))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotionFeedback>) istream)
  "Deserializes a message object of type '<MotionFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'progress) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'perc_complete) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'secs_remaining) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotionFeedback>)))
  "Returns string type for a message object of type '<MotionFeedback>"
  "ff_msgs/MotionFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotionFeedback)))
  "Returns string type for a message object of type 'MotionFeedback"
  "ff_msgs/MotionFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotionFeedback>)))
  "Returns md5sum for a message object of type '<MotionFeedback>"
  "a70f7ffb0db5b74cf7c14491a515651c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotionFeedback)))
  "Returns md5sum for a message object of type 'MotionFeedback"
  "a70f7ffb0db5b74cf7c14491a515651c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotionFeedback>)))
  "Returns full string definition for message of type '<MotionFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# The state of the teleop command~%ff_msgs/MotionState state~%~%# Control progress~%ff_msgs/ControlFeedback progress~%~%# Planner progress~%float32 perc_complete~%float32 secs_remaining~%~%~%================================================================================~%MSG: ff_msgs/MotionState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Locked topic that registers updates to the internal dock state~%~%# Header with timestamp~%std_msgs/Header header~%~%# The state of the mobility subsystem~%int8 state~%int8 INITIALIZING        = 0~%int8 IDLE                = 1~%int8 STOPPED             = 2~%int8 IDLING              = 3~%int8 STOPPING            = 4~%int8 PREPPING            = 5~%int8 BOOTSTRAPPING       = 6~%int8 PLANNING            = 7~%int8 PREPARING           = 8~%int8 CONTROLLING         = 9~%int8 REPLANNING          = 10~%int8 HALTING             = 11~%int8 REPLAN_WAIT         = 12~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/ControlFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%uint32 index                                # Index being processed~%~%ff_msgs/ControlState setpoint               # Current setpoint~%~%float32 error_position                      # Position error~%float32 error_attitude                      # Attitude error~%float32 error_velocity                      # Velocity error~%float32 error_omega                         # Omega error~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotionFeedback)))
  "Returns full string definition for message of type 'MotionFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%# The state of the teleop command~%ff_msgs/MotionState state~%~%# Control progress~%ff_msgs/ControlFeedback progress~%~%# Planner progress~%float32 perc_complete~%float32 secs_remaining~%~%~%================================================================================~%MSG: ff_msgs/MotionState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Locked topic that registers updates to the internal dock state~%~%# Header with timestamp~%std_msgs/Header header~%~%# The state of the mobility subsystem~%int8 state~%int8 INITIALIZING        = 0~%int8 IDLE                = 1~%int8 STOPPED             = 2~%int8 IDLING              = 3~%int8 STOPPING            = 4~%int8 PREPPING            = 5~%int8 BOOTSTRAPPING       = 6~%int8 PLANNING            = 7~%int8 PREPARING           = 8~%int8 CONTROLLING         = 9~%int8 REPLANNING          = 10~%int8 HALTING             = 11~%int8 REPLAN_WAIT         = 12~%~%# A human readble version of the (event) -> [state] transition~%string fsm_event~%string fsm_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/ControlFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%uint32 index                                # Index being processed~%~%ff_msgs/ControlState setpoint               # Current setpoint~%~%float32 error_position                      # Position error~%float32 error_attitude                      # Attitude error~%float32 error_velocity                      # Velocity error~%float32 error_omega                         # Omega error~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotionFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'progress))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotionFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'MotionFeedback
    (cl:cons ':state (state msg))
    (cl:cons ':progress (progress msg))
    (cl:cons ':perc_complete (perc_complete msg))
    (cl:cons ':secs_remaining (secs_remaining msg))
))
