; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude FlightMode.msg.html

(cl:defclass <FlightMode> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (control_enabled
    :reader control_enabled
    :initarg :control_enabled
    :type cl:boolean
    :initform cl:nil)
   (tolerance_pos_endpoint
    :reader tolerance_pos_endpoint
    :initarg :tolerance_pos_endpoint
    :type cl:float
    :initform 0.0)
   (tolerance_pos
    :reader tolerance_pos
    :initarg :tolerance_pos
    :type cl:float
    :initform 0.0)
   (tolerance_vel
    :reader tolerance_vel
    :initarg :tolerance_vel
    :type cl:float
    :initform 0.0)
   (tolerance_att
    :reader tolerance_att
    :initarg :tolerance_att
    :type cl:float
    :initform 0.0)
   (tolerance_omega
    :reader tolerance_omega
    :initarg :tolerance_omega
    :type cl:float
    :initform 0.0)
   (tolerance_time
    :reader tolerance_time
    :initarg :tolerance_time
    :type cl:float
    :initform 0.0)
   (att_kp
    :reader att_kp
    :initarg :att_kp
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (att_ki
    :reader att_ki
    :initarg :att_ki
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (omega_kd
    :reader omega_kd
    :initarg :omega_kd
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (pos_kp
    :reader pos_kp
    :initarg :pos_kp
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (pos_ki
    :reader pos_ki
    :initarg :pos_ki
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vel_kd
    :reader vel_kd
    :initarg :vel_kd
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (hard_limit_vel
    :reader hard_limit_vel
    :initarg :hard_limit_vel
    :type cl:float
    :initform 0.0)
   (hard_limit_accel
    :reader hard_limit_accel
    :initarg :hard_limit_accel
    :type cl:float
    :initform 0.0)
   (hard_limit_omega
    :reader hard_limit_omega
    :initarg :hard_limit_omega
    :type cl:float
    :initform 0.0)
   (hard_limit_alpha
    :reader hard_limit_alpha
    :initarg :hard_limit_alpha
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass FlightMode (<FlightMode>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FlightMode>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FlightMode)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<FlightMode> is deprecated: use ff_msgs-msg:FlightMode instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'control_enabled-val :lambda-list '(m))
(cl:defmethod control_enabled-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:control_enabled-val is deprecated.  Use ff_msgs-msg:control_enabled instead.")
  (control_enabled m))

(cl:ensure-generic-function 'tolerance_pos_endpoint-val :lambda-list '(m))
(cl:defmethod tolerance_pos_endpoint-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tolerance_pos_endpoint-val is deprecated.  Use ff_msgs-msg:tolerance_pos_endpoint instead.")
  (tolerance_pos_endpoint m))

(cl:ensure-generic-function 'tolerance_pos-val :lambda-list '(m))
(cl:defmethod tolerance_pos-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tolerance_pos-val is deprecated.  Use ff_msgs-msg:tolerance_pos instead.")
  (tolerance_pos m))

(cl:ensure-generic-function 'tolerance_vel-val :lambda-list '(m))
(cl:defmethod tolerance_vel-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tolerance_vel-val is deprecated.  Use ff_msgs-msg:tolerance_vel instead.")
  (tolerance_vel m))

(cl:ensure-generic-function 'tolerance_att-val :lambda-list '(m))
(cl:defmethod tolerance_att-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tolerance_att-val is deprecated.  Use ff_msgs-msg:tolerance_att instead.")
  (tolerance_att m))

(cl:ensure-generic-function 'tolerance_omega-val :lambda-list '(m))
(cl:defmethod tolerance_omega-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tolerance_omega-val is deprecated.  Use ff_msgs-msg:tolerance_omega instead.")
  (tolerance_omega m))

(cl:ensure-generic-function 'tolerance_time-val :lambda-list '(m))
(cl:defmethod tolerance_time-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tolerance_time-val is deprecated.  Use ff_msgs-msg:tolerance_time instead.")
  (tolerance_time m))

(cl:ensure-generic-function 'att_kp-val :lambda-list '(m))
(cl:defmethod att_kp-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:att_kp-val is deprecated.  Use ff_msgs-msg:att_kp instead.")
  (att_kp m))

(cl:ensure-generic-function 'att_ki-val :lambda-list '(m))
(cl:defmethod att_ki-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:att_ki-val is deprecated.  Use ff_msgs-msg:att_ki instead.")
  (att_ki m))

(cl:ensure-generic-function 'omega_kd-val :lambda-list '(m))
(cl:defmethod omega_kd-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:omega_kd-val is deprecated.  Use ff_msgs-msg:omega_kd instead.")
  (omega_kd m))

(cl:ensure-generic-function 'pos_kp-val :lambda-list '(m))
(cl:defmethod pos_kp-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:pos_kp-val is deprecated.  Use ff_msgs-msg:pos_kp instead.")
  (pos_kp m))

(cl:ensure-generic-function 'pos_ki-val :lambda-list '(m))
(cl:defmethod pos_ki-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:pos_ki-val is deprecated.  Use ff_msgs-msg:pos_ki instead.")
  (pos_ki m))

(cl:ensure-generic-function 'vel_kd-val :lambda-list '(m))
(cl:defmethod vel_kd-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:vel_kd-val is deprecated.  Use ff_msgs-msg:vel_kd instead.")
  (vel_kd m))

(cl:ensure-generic-function 'hard_limit_vel-val :lambda-list '(m))
(cl:defmethod hard_limit_vel-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:hard_limit_vel-val is deprecated.  Use ff_msgs-msg:hard_limit_vel instead.")
  (hard_limit_vel m))

(cl:ensure-generic-function 'hard_limit_accel-val :lambda-list '(m))
(cl:defmethod hard_limit_accel-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:hard_limit_accel-val is deprecated.  Use ff_msgs-msg:hard_limit_accel instead.")
  (hard_limit_accel m))

(cl:ensure-generic-function 'hard_limit_omega-val :lambda-list '(m))
(cl:defmethod hard_limit_omega-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:hard_limit_omega-val is deprecated.  Use ff_msgs-msg:hard_limit_omega instead.")
  (hard_limit_omega m))

(cl:ensure-generic-function 'hard_limit_alpha-val :lambda-list '(m))
(cl:defmethod hard_limit_alpha-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:hard_limit_alpha-val is deprecated.  Use ff_msgs-msg:hard_limit_alpha instead.")
  (hard_limit_alpha m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <FlightMode>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:speed-val is deprecated.  Use ff_msgs-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FlightMode>)))
    "Constants for message type '<FlightMode>"
  '((:SPEED_MIN . 0)
    (:SPEED_OFF . 0)
    (:SPEED_QUIET . 1)
    (:SPEED_NOMINAL . 2)
    (:SPEED_AGGRESSIVE . 3)
    (:SPEED_MAX . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FlightMode)))
    "Constants for message type 'FlightMode"
  '((:SPEED_MIN . 0)
    (:SPEED_OFF . 0)
    (:SPEED_QUIET . 1)
    (:SPEED_NOMINAL . 2)
    (:SPEED_AGGRESSIVE . 3)
    (:SPEED_MAX . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FlightMode>) ostream)
  "Serializes a message object of type '<FlightMode>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'control_enabled) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance_pos_endpoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance_pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance_att))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance_omega))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tolerance_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'att_kp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'att_ki) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'omega_kd) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_kp) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos_ki) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel_kd) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hard_limit_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hard_limit_accel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hard_limit_omega))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'hard_limit_alpha))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FlightMode>) istream)
  "Deserializes a message object of type '<FlightMode>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'control_enabled) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_pos_endpoint) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_pos) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_att) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_omega) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tolerance_time) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'att_kp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'att_ki) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'omega_kd) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_kp) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos_ki) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel_kd) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hard_limit_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hard_limit_accel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hard_limit_omega) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'hard_limit_alpha) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'speed)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FlightMode>)))
  "Returns string type for a message object of type '<FlightMode>"
  "ff_msgs/FlightMode")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FlightMode)))
  "Returns string type for a message object of type 'FlightMode"
  "ff_msgs/FlightMode")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FlightMode>)))
  "Returns md5sum for a message object of type '<FlightMode>"
  "0bb389101a5f30087bd644e6596d8e8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FlightMode)))
  "Returns md5sum for a message object of type 'FlightMode"
  "0bb389101a5f30087bd644e6596d8e8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FlightMode>)))
  "Returns full string definition for message of type '<FlightMode>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message captures all information in a flight mode~%~%Header header                     # Metadata~%~%string name                       # Name of the flight mode~%~%bool control_enabled              # Is control enabled?~%~%# Tolerances (all in SI units)~%float32 tolerance_pos_endpoint    # Endpoint position tolerance in m~%float32 tolerance_pos             # Position tolerance in m~%float32 tolerance_vel             # Velocity tolerance in m/s~%float32 tolerance_att             # Attitude tolerance in rads~%float32 tolerance_omega           # Angular acceleration tolerance in rad/s~%float32 tolerance_time            # Acceptable lag betwee TX and RX of control~%~%# Controller gains~%geometry_msgs/Vector3 att_kp      # Positional proportional constant~%geometry_msgs/Vector3 att_ki      # Positional integrative constant~%geometry_msgs/Vector3 omega_kd    # Attidue derivative constant~%geometry_msgs/Vector3 pos_kp      # Positional proportional contant~%geometry_msgs/Vector3 pos_ki      # Positional integrative constant~%geometry_msgs/Vector3 vel_kd      # Positional derivative constant~%~%# Hard limit on planning~%float32 hard_limit_vel            # Position tolerance in m/s~%float32 hard_limit_accel          # Position tolerance in m/s^2~%float32 hard_limit_omega          # Position tolerance in rads/s~%float32 hard_limit_alpha          # Position tolerance in rads/s^2~%~%# Impeller speed~%uint8 speed                       # Current speed gain~%uint8 SPEED_MIN        = 0        # Min acceptable gain~%uint8 SPEED_OFF        = 0        # Blowers off~%uint8 SPEED_QUIET      = 1        # Quiet mode~%uint8 SPEED_NOMINAL    = 2        # Nomainal mode~%uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode~%uint8 SPEED_MAX        = 3        # Max acceptable gain~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FlightMode)))
  "Returns full string definition for message of type 'FlightMode"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message captures all information in a flight mode~%~%Header header                     # Metadata~%~%string name                       # Name of the flight mode~%~%bool control_enabled              # Is control enabled?~%~%# Tolerances (all in SI units)~%float32 tolerance_pos_endpoint    # Endpoint position tolerance in m~%float32 tolerance_pos             # Position tolerance in m~%float32 tolerance_vel             # Velocity tolerance in m/s~%float32 tolerance_att             # Attitude tolerance in rads~%float32 tolerance_omega           # Angular acceleration tolerance in rad/s~%float32 tolerance_time            # Acceptable lag betwee TX and RX of control~%~%# Controller gains~%geometry_msgs/Vector3 att_kp      # Positional proportional constant~%geometry_msgs/Vector3 att_ki      # Positional integrative constant~%geometry_msgs/Vector3 omega_kd    # Attidue derivative constant~%geometry_msgs/Vector3 pos_kp      # Positional proportional contant~%geometry_msgs/Vector3 pos_ki      # Positional integrative constant~%geometry_msgs/Vector3 vel_kd      # Positional derivative constant~%~%# Hard limit on planning~%float32 hard_limit_vel            # Position tolerance in m/s~%float32 hard_limit_accel          # Position tolerance in m/s^2~%float32 hard_limit_omega          # Position tolerance in rads/s~%float32 hard_limit_alpha          # Position tolerance in rads/s^2~%~%# Impeller speed~%uint8 speed                       # Current speed gain~%uint8 SPEED_MIN        = 0        # Min acceptable gain~%uint8 SPEED_OFF        = 0        # Blowers off~%uint8 SPEED_QUIET      = 1        # Quiet mode~%uint8 SPEED_NOMINAL    = 2        # Nomainal mode~%uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode~%uint8 SPEED_MAX        = 3        # Max acceptable gain~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FlightMode>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'name))
     1
     4
     4
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'att_kp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'att_ki))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'omega_kd))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_kp))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos_ki))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel_kd))
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FlightMode>))
  "Converts a ROS message object to a list"
  (cl:list 'FlightMode
    (cl:cons ':header (header msg))
    (cl:cons ':name (name msg))
    (cl:cons ':control_enabled (control_enabled msg))
    (cl:cons ':tolerance_pos_endpoint (tolerance_pos_endpoint msg))
    (cl:cons ':tolerance_pos (tolerance_pos msg))
    (cl:cons ':tolerance_vel (tolerance_vel msg))
    (cl:cons ':tolerance_att (tolerance_att msg))
    (cl:cons ':tolerance_omega (tolerance_omega msg))
    (cl:cons ':tolerance_time (tolerance_time msg))
    (cl:cons ':att_kp (att_kp msg))
    (cl:cons ':att_ki (att_ki msg))
    (cl:cons ':omega_kd (omega_kd msg))
    (cl:cons ':pos_kp (pos_kp msg))
    (cl:cons ':pos_ki (pos_ki msg))
    (cl:cons ':vel_kd (vel_kd msg))
    (cl:cons ':hard_limit_vel (hard_limit_vel msg))
    (cl:cons ':hard_limit_accel (hard_limit_accel msg))
    (cl:cons ':hard_limit_omega (hard_limit_omega msg))
    (cl:cons ':hard_limit_alpha (hard_limit_alpha msg))
    (cl:cons ':speed (speed msg))
))
