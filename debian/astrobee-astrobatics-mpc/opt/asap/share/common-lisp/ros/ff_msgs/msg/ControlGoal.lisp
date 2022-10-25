; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ControlGoal.msg.html

(cl:defclass <ControlGoal> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (segment
    :reader segment
    :initarg :segment
    :type (cl:vector ff_msgs-msg:ControlState)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:ControlState :initial-element (cl:make-instance 'ff_msgs-msg:ControlState))))
)

(cl:defclass ControlGoal (<ControlGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ControlGoal> is deprecated: use ff_msgs-msg:ControlGoal instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <ControlGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:command-val is deprecated.  Use ff_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'segment-val :lambda-list '(m))
(cl:defmethod segment-val ((m <ControlGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:segment-val is deprecated.  Use ff_msgs-msg:segment instead.")
  (segment m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ControlGoal>)))
    "Constants for message type '<ControlGoal>"
  '((:STOP . 0)
    (:IDLE . 1)
    (:NOMINAL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ControlGoal)))
    "Constants for message type 'ControlGoal"
  '((:STOP . 0)
    (:IDLE . 1)
    (:NOMINAL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlGoal>) ostream)
  "Serializes a message object of type '<ControlGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'segment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'segment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlGoal>) istream)
  "Deserializes a message object of type '<ControlGoal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlGoal>)))
  "Returns string type for a message object of type '<ControlGoal>"
  "ff_msgs/ControlGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlGoal)))
  "Returns string type for a message object of type 'ControlGoal"
  "ff_msgs/ControlGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlGoal>)))
  "Returns md5sum for a message object of type '<ControlGoal>"
  "cb33d16997b599aafbc0b0932c171b92")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlGoal)))
  "Returns md5sum for a message object of type 'ControlGoal"
  "cb33d16997b599aafbc0b0932c171b92")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlGoal>)))
  "Returns full string definition for message of type '<ControlGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the CONTROL action offered by GNC::WRAPPER~%~%uint8 command                               # STOP, IDLE, NOMINAL~%uint8 STOP    = 0~%uint8 IDLE    = 1~%uint8 NOMINAL = 2~%~%ff_msgs/ControlState[] segment              # NOMINIAL ONLY: Segment~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlGoal)))
  "Returns full string definition for message of type 'ControlGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the CONTROL action offered by GNC::WRAPPER~%~%uint8 command                               # STOP, IDLE, NOMINAL~%uint8 STOP    = 0~%uint8 IDLE    = 1~%uint8 NOMINAL = 2~%~%ff_msgs/ControlState[] segment              # NOMINIAL ONLY: Segment~%~%~%================================================================================~%MSG: ff_msgs/ControlState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Full state vector containing Time, Pose, Vel, and Accel~%# ~%# when {time}~%# flight_mode {string} - disctates, gains, tolerances, etc.~%# pose {Point position, Quaternion orientation}~%# twist {Vector3 linear, Vector3 angular}~%# accel {Vector3 linear, Vector3 angular}~%~%time when~%geometry_msgs/Pose pose~%geometry_msgs/Twist twist~%geometry_msgs/Twist accel~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlGoal>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'segment) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlGoal
    (cl:cons ':command (command msg))
    (cl:cons ':segment (segment msg))
))
