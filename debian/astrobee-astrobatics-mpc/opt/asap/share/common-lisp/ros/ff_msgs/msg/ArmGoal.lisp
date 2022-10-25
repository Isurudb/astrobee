; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmGoal.msg.html

(cl:defclass <ArmGoal> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (pan
    :reader pan
    :initarg :pan
    :type cl:float
    :initform 0.0)
   (tilt
    :reader tilt
    :initarg :tilt
    :type cl:float
    :initform 0.0)
   (gripper
    :reader gripper
    :initarg :gripper
    :type cl:float
    :initform 0.0))
)

(cl:defclass ArmGoal (<ArmGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmGoal> is deprecated: use ff_msgs-msg:ArmGoal instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <ArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:command-val is deprecated.  Use ff_msgs-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'pan-val :lambda-list '(m))
(cl:defmethod pan-val ((m <ArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:pan-val is deprecated.  Use ff_msgs-msg:pan instead.")
  (pan m))

(cl:ensure-generic-function 'tilt-val :lambda-list '(m))
(cl:defmethod tilt-val ((m <ArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tilt-val is deprecated.  Use ff_msgs-msg:tilt instead.")
  (tilt m))

(cl:ensure-generic-function 'gripper-val :lambda-list '(m))
(cl:defmethod gripper-val ((m <ArmGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:gripper-val is deprecated.  Use ff_msgs-msg:gripper instead.")
  (gripper m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ArmGoal>)))
    "Constants for message type '<ArmGoal>"
  '((:ARM_STOP . 0)
    (:ARM_DEPLOY . 1)
    (:ARM_STOW . 2)
    (:ARM_PAN . 3)
    (:ARM_TILT . 4)
    (:ARM_MOVE . 5)
    (:GRIPPER_SET . 6)
    (:GRIPPER_OPEN . 7)
    (:GRIPPER_CLOSE . 8)
    (:DISABLE_SERVO . 9))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ArmGoal)))
    "Constants for message type 'ArmGoal"
  '((:ARM_STOP . 0)
    (:ARM_DEPLOY . 1)
    (:ARM_STOW . 2)
    (:ARM_PAN . 3)
    (:ARM_TILT . 4)
    (:ARM_MOVE . 5)
    (:GRIPPER_SET . 6)
    (:GRIPPER_OPEN . 7)
    (:GRIPPER_CLOSE . 8)
    (:DISABLE_SERVO . 9))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmGoal>) ostream)
  "Serializes a message object of type '<ArmGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tilt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gripper))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmGoal>) istream)
  "Deserializes a message object of type '<ArmGoal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pan) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tilt) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gripper) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmGoal>)))
  "Returns string type for a message object of type '<ArmGoal>"
  "ff_msgs/ArmGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmGoal)))
  "Returns string type for a message object of type 'ArmGoal"
  "ff_msgs/ArmGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmGoal>)))
  "Returns md5sum for a message object of type '<ArmGoal>"
  "2132b678d6b320fdf79bc08b99d42769")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmGoal)))
  "Returns md5sum for a message object of type 'ArmGoal"
  "2132b678d6b320fdf79bc08b99d42769")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmGoal>)))
  "Returns full string definition for message of type '<ArmGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the ARM action offered by the PERCHING ARM~%~%uint8 command                                 # What to do~%uint8 ARM_STOP            = 0                 # Stop the arm (vel = 0)~%uint8 ARM_DEPLOY          = 1                 # Deploy the arm~%uint8 ARM_STOW            = 2                 # Retract the arm~%uint8 ARM_PAN             = 3                 # Pan the arm~%uint8 ARM_TILT            = 4                 # Tilt the arm~%uint8 ARM_MOVE            = 5                 # Pan and tilt the~%uint8 GRIPPER_SET         = 6                 # Set the gripper value~%uint8 GRIPPER_OPEN        = 7                 # Open the gripper~%uint8 GRIPPER_CLOSE       = 8                 # Close the gripper~%uint8 DISABLE_SERVO       = 9                 # Disable the servos~%~%float32 pan                                   # PAN from -90 to +90~%float32 tilt                                  # TILT from -120 to +90~%float32 gripper                               # SET from 20 to 45~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmGoal)))
  "Returns full string definition for message of type 'ArmGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message describes the ARM action offered by the PERCHING ARM~%~%uint8 command                                 # What to do~%uint8 ARM_STOP            = 0                 # Stop the arm (vel = 0)~%uint8 ARM_DEPLOY          = 1                 # Deploy the arm~%uint8 ARM_STOW            = 2                 # Retract the arm~%uint8 ARM_PAN             = 3                 # Pan the arm~%uint8 ARM_TILT            = 4                 # Tilt the arm~%uint8 ARM_MOVE            = 5                 # Pan and tilt the~%uint8 GRIPPER_SET         = 6                 # Set the gripper value~%uint8 GRIPPER_OPEN        = 7                 # Open the gripper~%uint8 GRIPPER_CLOSE       = 8                 # Close the gripper~%uint8 DISABLE_SERVO       = 9                 # Disable the servos~%~%float32 pan                                   # PAN from -90 to +90~%float32 tilt                                  # TILT from -120 to +90~%float32 gripper                               # SET from 20 to 45~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmGoal>))
  (cl:+ 0
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmGoal
    (cl:cons ':command (command msg))
    (cl:cons ':pan (pan msg))
    (cl:cons ':tilt (tilt msg))
    (cl:cons ':gripper (gripper msg))
))
