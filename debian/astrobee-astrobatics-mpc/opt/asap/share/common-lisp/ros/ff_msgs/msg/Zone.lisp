; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude Zone.msg.html

(cl:defclass <Zone> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (index
    :reader index
    :initarg :index
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0)
   (min
    :reader min
    :initarg :min
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (max
    :reader max
    :initarg :max
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Zone (<Zone>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Zone>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Zone)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<Zone> is deprecated: use ff_msgs-msg:Zone instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <Zone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:name-val is deprecated.  Use ff_msgs-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <Zone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:index-val is deprecated.  Use ff_msgs-msg:index instead.")
  (index m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <Zone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:type-val is deprecated.  Use ff_msgs-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <Zone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:min-val is deprecated.  Use ff_msgs-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <Zone>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:max-val is deprecated.  Use ff_msgs-msg:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Zone>)))
    "Constants for message type '<Zone>"
  '((:KEEPOUT . 0)
    (:KEEPIN . 1)
    (:CLUTTER . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Zone)))
    "Constants for message type 'Zone"
  '((:KEEPOUT . 0)
    (:KEEPIN . 1)
    (:CLUTTER . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Zone>) ostream)
  "Serializes a message object of type '<Zone>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let* ((signed (cl:slot-value msg 'index)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Zone>) istream)
  "Deserializes a message object of type '<Zone>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'index) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Zone>)))
  "Returns string type for a message object of type '<Zone>"
  "ff_msgs/Zone")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Zone)))
  "Returns string type for a message object of type 'Zone"
  "ff_msgs/Zone")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Zone>)))
  "Returns md5sum for a message object of type '<Zone>"
  "eb4a7bedb72c164486d2ac45ba0a7b8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Zone)))
  "Returns md5sum for a message object of type 'Zone"
  "eb4a7bedb72c164486d2ac45ba0a7b8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Zone>)))
  "Returns full string definition for message of type '<Zone>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message defines a zone, such as a Keepin or Keeoput.~%~%string name                   # Name of zone~%~%# A name can refer to multiple zones. This is the index of the zone with respect~%# to the zone name~%int32 index~%~%# Zone type~%uint8 KEEPOUT = 0       # An area the freeflyer should stay out of~%uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in ~%uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter~%~%uint8 type              # Whether the zone is a keepin, keepout, or clutter~%~%geometry_msgs/Vector3 min   # One corner of the zone~%geometry_msgs/Vector3 max   # The opposite corner of the zone~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Zone)))
  "Returns full string definition for message of type 'Zone"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message defines a zone, such as a Keepin or Keeoput.~%~%string name                   # Name of zone~%~%# A name can refer to multiple zones. This is the index of the zone with respect~%# to the zone name~%int32 index~%~%# Zone type~%uint8 KEEPOUT = 0       # An area the freeflyer should stay out of~%uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in ~%uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter~%~%uint8 type              # Whether the zone is a keepin, keepout, or clutter~%~%geometry_msgs/Vector3 min   # One corner of the zone~%geometry_msgs/Vector3 max   # The opposite corner of the zone~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Zone>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Zone>))
  "Converts a ROS message object to a list"
  (cl:list 'Zone
    (cl:cons ':name (name msg))
    (cl:cons ':index (index msg))
    (cl:cons ':type (type msg))
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
