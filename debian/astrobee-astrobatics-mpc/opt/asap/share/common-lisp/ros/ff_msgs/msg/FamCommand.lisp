; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude FamCommand.msg.html

(cl:defclass <FamCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (accel
    :reader accel
    :initarg :accel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (alpha
    :reader alpha
    :initarg :alpha
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (position_error
    :reader position_error
    :initarg :position_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (position_error_integrated
    :reader position_error_integrated
    :initarg :position_error_integrated
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (attitude_error
    :reader attitude_error
    :initarg :attitude_error
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (attitude_error_integrated
    :reader attitude_error_integrated
    :initarg :attitude_error_integrated
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (attitude_error_mag
    :reader attitude_error_mag
    :initarg :attitude_error_mag
    :type cl:float
    :initform 0.0)
   (control_mode
    :reader control_mode
    :initarg :control_mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FamCommand (<FamCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FamCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FamCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<FamCommand> is deprecated: use ff_msgs-msg:FamCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:wrench-val is deprecated.  Use ff_msgs-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:accel-val is deprecated.  Use ff_msgs-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'alpha-val :lambda-list '(m))
(cl:defmethod alpha-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:alpha-val is deprecated.  Use ff_msgs-msg:alpha instead.")
  (alpha m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:status-val is deprecated.  Use ff_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'position_error-val :lambda-list '(m))
(cl:defmethod position_error-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:position_error-val is deprecated.  Use ff_msgs-msg:position_error instead.")
  (position_error m))

(cl:ensure-generic-function 'position_error_integrated-val :lambda-list '(m))
(cl:defmethod position_error_integrated-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:position_error_integrated-val is deprecated.  Use ff_msgs-msg:position_error_integrated instead.")
  (position_error_integrated m))

(cl:ensure-generic-function 'attitude_error-val :lambda-list '(m))
(cl:defmethod attitude_error-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:attitude_error-val is deprecated.  Use ff_msgs-msg:attitude_error instead.")
  (attitude_error m))

(cl:ensure-generic-function 'attitude_error_integrated-val :lambda-list '(m))
(cl:defmethod attitude_error_integrated-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:attitude_error_integrated-val is deprecated.  Use ff_msgs-msg:attitude_error_integrated instead.")
  (attitude_error_integrated m))

(cl:ensure-generic-function 'attitude_error_mag-val :lambda-list '(m))
(cl:defmethod attitude_error_mag-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:attitude_error_mag-val is deprecated.  Use ff_msgs-msg:attitude_error_mag instead.")
  (attitude_error_mag m))

(cl:ensure-generic-function 'control_mode-val :lambda-list '(m))
(cl:defmethod control_mode-val ((m <FamCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:control_mode-val is deprecated.  Use ff_msgs-msg:control_mode instead.")
  (control_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FamCommand>) ostream)
  "Serializes a message object of type '<FamCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'alpha) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_error_integrated) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_error) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'attitude_error_integrated) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'attitude_error_mag))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FamCommand>) istream)
  "Deserializes a message object of type '<FamCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'alpha) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_error_integrated) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_error) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'attitude_error_integrated) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'attitude_error_mag) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'control_mode)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FamCommand>)))
  "Returns string type for a message object of type '<FamCommand>"
  "ff_msgs/FamCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FamCommand)))
  "Returns string type for a message object of type 'FamCommand"
  "ff_msgs/FamCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FamCommand>)))
  "Returns md5sum for a message object of type '<FamCommand>"
  "baf174131dee1a8b03d9d5feac8aa809")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FamCommand)))
  "Returns md5sum for a message object of type 'FamCommand"
  "baf174131dee1a8b03d9d5feac8aa809")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FamCommand>)))
  "Returns full string definition for message of type '<FamCommand>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Command sent from control to the FAM.~%~%std_msgs/Header header # header with time stamp~%~%# force and torque~%geometry_msgs/Wrench wrench~%# linear acceleration (wrench w/out estimated mass)~%geometry_msgs/Vector3 accel~%# angular accceleration (wrench w/out estimated mass)~%geometry_msgs/Vector3 alpha~%~%# status byte from GNC ICD~%uint8 status~%~%# position error~%geometry_msgs/Vector3 position_error~%# integrated position error~%geometry_msgs/Vector3 position_error_integrated~%~%# attitude error~%geometry_msgs/Vector3 attitude_error~%# integrated attitude error~%geometry_msgs/Vector3 attitude_error_integrated~%# magnitude of attitude error~%float32 attitude_error_mag~%~%# control mode from GNC ICD~%uint8 control_mode~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FamCommand)))
  "Returns full string definition for message of type 'FamCommand"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Command sent from control to the FAM.~%~%std_msgs/Header header # header with time stamp~%~%# force and torque~%geometry_msgs/Wrench wrench~%# linear acceleration (wrench w/out estimated mass)~%geometry_msgs/Vector3 accel~%# angular accceleration (wrench w/out estimated mass)~%geometry_msgs/Vector3 alpha~%~%# status byte from GNC ICD~%uint8 status~%~%# position error~%geometry_msgs/Vector3 position_error~%# integrated position error~%geometry_msgs/Vector3 position_error_integrated~%~%# attitude error~%geometry_msgs/Vector3 attitude_error~%# integrated attitude error~%geometry_msgs/Vector3 attitude_error_integrated~%# magnitude of attitude error~%float32 attitude_error_mag~%~%# control mode from GNC ICD~%uint8 control_mode~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FamCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'alpha))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_error_integrated))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_error))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'attitude_error_integrated))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FamCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'FamCommand
    (cl:cons ':header (header msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':accel (accel msg))
    (cl:cons ':alpha (alpha msg))
    (cl:cons ':status (status msg))
    (cl:cons ':position_error (position_error msg))
    (cl:cons ':position_error_integrated (position_error_integrated msg))
    (cl:cons ':attitude_error (attitude_error msg))
    (cl:cons ':attitude_error_integrated (attitude_error_integrated msg))
    (cl:cons ':attitude_error_mag (attitude_error_mag msg))
    (cl:cons ':control_mode (control_mode msg))
))
