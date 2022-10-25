; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude MotionGoal.msg.html

(cl:defclass <MotionGoal> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (flight_mode
    :reader flight_mode
    :initarg :flight_mode
    :type cl:string
    :initform "")
   (segment
    :reader segment
    :initarg :segment
    :type (cl:vector ff_msgs-msg:ControlState)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:ControlState :initial-element (cl:make-instance 'ff_msgs-msg:ControlState)))
   (states
    :reader states
    :initarg :states
    :type (cl:vector geometry_msgs-msg:PoseStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:PoseStamped :initial-element (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
   (reference_frame
    :reader reference_frame
    :initarg :reference_frame
    :type cl:string
    :initform ""))
)

(cl:defclass MotionGoal (<MotionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<MotionGoal> is deprecated: use ff_msgs-msg:MotionGoal instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <MotionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:command-val is deprecated.  Use ff_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'flight_mode-val :lambda-list '(m))
(cl:defmethod flight_mode-val ((m <MotionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:flight_mode-val is deprecated.  Use ff_msgs-msg:flight_mode instead.")
  (flight_mode m))

(cl:ensure-generic-function 'segment-val :lambda-list '(m))
(cl:defmethod segment-val ((m <MotionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:segment-val is deprecated.  Use ff_msgs-msg:segment instead.")
  (segment m))

(cl:ensure-generic-function 'states-val :lambda-list '(m))
(cl:defmethod states-val ((m <MotionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:states-val is deprecated.  Use ff_msgs-msg:states instead.")
  (states m))

(cl:ensure-generic-function 'reference_frame-val :lambda-list '(m))
(cl:defmethod reference_frame-val ((m <MotionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:reference_frame-val is deprecated.  Use ff_msgs-msg:reference_frame instead.")
  (reference_frame m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<MotionGoal>)))
    "Constants for message type '<MotionGoal>"
  '((:STOP . 0)
    (:IDLE . 1)
    (:EXEC . 2)
    (:MOVE . 3)
    (:PREP . 4)
    (:OFF . off)
    (:NOMINAL . nominal)
    (:DIFFICULT . difficult)
    (:QUIET . quiet)
    (:PERCHING . perching)
    (:UNPERCHING . unperching)
    (:DOCKING . docking)
    (:UNDOCKING . undocking)
    (:PRECISION . precision))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'MotionGoal)))
    "Constants for message type 'MotionGoal"
  '((:STOP . 0)
    (:IDLE . 1)
    (:EXEC . 2)
    (:MOVE . 3)
    (:PREP . 4)
    (:OFF . off)
    (:NOMINAL . nominal)
    (:DIFFICULT . difficult)
    (:QUIET . quiet)
    (:PERCHING . perching)
    (:UNPERCHING . unperching)
    (:DOCKING . docking)
    (:UNDOCKING . undocking)
    (:PRECISION . precision))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotionGoal>) ostream)
  "Serializes a message object of type '<MotionGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'flight_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'flight_mode))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'segment))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'states))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'states))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reference_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reference_frame))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotionGoal>) istream)
  "Deserializes a message object of type '<MotionGoal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'flight_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'flight_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'segment) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'segment)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:ControlState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'states) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'states)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:PoseStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reference_frame) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reference_frame) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotionGoal>)))
  "Returns string type for a message object of type '<MotionGoal>"
  "ff_msgs/MotionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotionGoal)))
  "Returns string type for a message object of type 'MotionGoal"
  "ff_msgs/MotionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotionGoal>)))
  "Returns md5sum for a message object of type '<MotionGoal>"
  "1f5494c5016459c2ad2035dda3ef7acb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotionGoal)))
  "Returns md5sum for a message object of type 'MotionGoal"
  "1f5494c5016459c2ad2035dda3ef7acb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotionGoal>)))
  "Returns full string definition for message of type '<MotionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the MOTION action offered by the CHOREOGRAPHER~%~%# Desired command~%uint8 command~%uint8 STOP        = 0  # Stop immediately~%uint8 IDLE        = 1  # Idle immediately~%uint8 EXEC        = 2  # Execute a given segment~%uint8 MOVE        = 3  # Move through a given set of poses~%uint8 PREP        = 4  # Prepare the system for a given flight mode~%~%# ALL COMMANDS: flight mode. If left empty, the default will be used.~%string flight_mode~%string OFF        = off~%string NOMINAL    = nominal~%string DIFFICULT  = difficult~%string QUIET      = quiet~%~%# These four values no longer used (2020/10). Keep for backward~%# compatibility with archived ISS telemetry bags.~%string PERCHING   = perching~%string UNPERCHING = unperching~%string DOCKING    = docking~%string UNDOCKING  = undocking~%~%string PRECISION  = precision~%~%# EXECUTE ONLY : The segment~%ff_msgs/ControlState[] segment~%~%# MOVE ONLY: Desired state sequence~%geometry_msgs/PoseStamped[] states~%~%string reference_frame~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotionGoal)))
  "Returns full string definition for message of type 'MotionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the MOTION action offered by the CHOREOGRAPHER~%~%# Desired command~%uint8 command~%uint8 STOP        = 0  # Stop immediately~%uint8 IDLE        = 1  # Idle immediately~%uint8 EXEC        = 2  # Execute a given segment~%uint8 MOVE        = 3  # Move through a given set of poses~%uint8 PREP        = 4  # Prepare the system for a given flight mode~%~%# ALL COMMANDS: flight mode. If left empty, the default will be used.~%string flight_mode~%string OFF        = off~%string NOMINAL    = nominal~%string DIFFICULT  = difficult~%string QUIET      = quiet~%~%# These four values no longer used (2020/10). Keep for backward~%# compatibility with archived ISS telemetry bags.~%string PERCHING   = perching~%string UNPERCHING = unperching~%string DOCKING    = docking~%string UNDOCKING  = undocking~%~%string PRECISION  = precision~%~%# EXECUTE ONLY : The segment~%ff_msgs/ControlState[] segment~%~%# MOVE ONLY: Desired state sequence~%geometry_msgs/PoseStamped[] states~%~%string reference_frame~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotionGoal>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'flight_mode))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segment) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'states) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'reference_frame))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'MotionGoal
    (cl:cons ':command (command msg))
    (cl:cons ':flight_mode (flight_mode msg))
    (cl:cons ':segment (segment msg))
    (cl:cons ':states (states msg))
    (cl:cons ':reference_frame (reference_frame msg))
))
