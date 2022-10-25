; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude Odometry.msg.html

(cl:defclass <Odometry> (roslisp-msg-protocol:ros-message)
  ((source_time
    :reader source_time
    :initarg :source_time
    :type cl:real
    :initform 0)
   (target_time
    :reader target_time
    :initarg :target_time
    :type cl:real
    :initform 0)
   (sensor_F_source_T_target
    :reader sensor_F_source_T_target
    :initarg :sensor_F_source_T_target
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance))
   (body_F_source_T_target
    :reader body_F_source_T_target
    :initarg :body_F_source_T_target
    :type geometry_msgs-msg:PoseWithCovariance
    :initform (cl:make-instance 'geometry_msgs-msg:PoseWithCovariance)))
)

(cl:defclass Odometry (<Odometry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Odometry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Odometry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<Odometry> is deprecated: use ff_msgs-msg:Odometry instead.")))

(cl:ensure-generic-function 'source_time-val :lambda-list '(m))
(cl:defmethod source_time-val ((m <Odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:source_time-val is deprecated.  Use ff_msgs-msg:source_time instead.")
  (source_time m))

(cl:ensure-generic-function 'target_time-val :lambda-list '(m))
(cl:defmethod target_time-val ((m <Odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_time-val is deprecated.  Use ff_msgs-msg:target_time instead.")
  (target_time m))

(cl:ensure-generic-function 'sensor_F_source_T_target-val :lambda-list '(m))
(cl:defmethod sensor_F_source_T_target-val ((m <Odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:sensor_F_source_T_target-val is deprecated.  Use ff_msgs-msg:sensor_F_source_T_target instead.")
  (sensor_F_source_T_target m))

(cl:ensure-generic-function 'body_F_source_T_target-val :lambda-list '(m))
(cl:defmethod body_F_source_T_target-val ((m <Odometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:body_F_source_T_target-val is deprecated.  Use ff_msgs-msg:body_F_source_T_target instead.")
  (body_F_source_T_target m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Odometry>) ostream)
  "Serializes a message object of type '<Odometry>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'source_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'source_time) (cl:floor (cl:slot-value msg 'source_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'target_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'target_time) (cl:floor (cl:slot-value msg 'target_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sensor_F_source_T_target) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'body_F_source_T_target) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Odometry>) istream)
  "Deserializes a message object of type '<Odometry>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sensor_F_source_T_target) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'body_F_source_T_target) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Odometry>)))
  "Returns string type for a message object of type '<Odometry>"
  "ff_msgs/Odometry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Odometry)))
  "Returns string type for a message object of type 'Odometry"
  "ff_msgs/Odometry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Odometry>)))
  "Returns md5sum for a message object of type '<Odometry>"
  "422b88b6dc476361c3b1485e5b6113f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Odometry)))
  "Returns md5sum for a message object of type 'Odometry"
  "422b88b6dc476361c3b1485e5b6113f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Odometry>)))
  "Returns full string definition for message of type '<Odometry>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%~%time source_time ~%time target_time~%geometry_msgs/PoseWithCovariance sensor_F_source_T_target~%geometry_msgs/PoseWithCovariance body_F_source_T_target~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Odometry)))
  "Returns full string definition for message of type 'Odometry"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%~%time source_time ~%time target_time~%geometry_msgs/PoseWithCovariance sensor_F_source_T_target~%geometry_msgs/PoseWithCovariance body_F_source_T_target~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Odometry>))
  (cl:+ 0
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sensor_F_source_T_target))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'body_F_source_T_target))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Odometry>))
  "Converts a ROS message object to a list"
  (cl:list 'Odometry
    (cl:cons ':source_time (source_time msg))
    (cl:cons ':target_time (target_time msg))
    (cl:cons ':sensor_F_source_T_target (sensor_F_source_T_target msg))
    (cl:cons ':body_F_source_T_target (body_F_source_T_target msg))
))
