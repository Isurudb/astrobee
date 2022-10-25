; Auto-generated. Do not edit!


(cl:in-package ff_msgs-srv)


;//! \htmlinclude SetZones-request.msg.html

(cl:defclass <SetZones-request> (roslisp-msg-protocol:ros-message)
  ((timestamp
    :reader timestamp
    :initarg :timestamp
    :type cl:real
    :initform 0)
   (zones
    :reader zones
    :initarg :zones
    :type (cl:vector ff_msgs-msg:Zone)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:Zone :initial-element (cl:make-instance 'ff_msgs-msg:Zone))))
)

(cl:defclass SetZones-request (<SetZones-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetZones-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetZones-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetZones-request> is deprecated: use ff_msgs-srv:SetZones-request instead.")))

(cl:ensure-generic-function 'timestamp-val :lambda-list '(m))
(cl:defmethod timestamp-val ((m <SetZones-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:timestamp-val is deprecated.  Use ff_msgs-srv:timestamp instead.")
  (timestamp m))

(cl:ensure-generic-function 'zones-val :lambda-list '(m))
(cl:defmethod zones-val ((m <SetZones-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:zones-val is deprecated.  Use ff_msgs-srv:zones instead.")
  (zones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetZones-request>) ostream)
  "Serializes a message object of type '<SetZones-request>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'timestamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'timestamp) (cl:floor (cl:slot-value msg 'timestamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'zones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'zones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetZones-request>) istream)
  "Deserializes a message object of type '<SetZones-request>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'timestamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'zones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'zones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:Zone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetZones-request>)))
  "Returns string type for a service object of type '<SetZones-request>"
  "ff_msgs/SetZonesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetZones-request)))
  "Returns string type for a service object of type 'SetZones-request"
  "ff_msgs/SetZonesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetZones-request>)))
  "Returns md5sum for a message object of type '<SetZones-request>"
  "409154f0ce409359cbde795e43243c62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetZones-request)))
  "Returns md5sum for a message object of type 'SetZones-request"
  "409154f0ce409359cbde795e43243c62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetZones-request>)))
  "Returns full string definition for message of type '<SetZones-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%time timestamp~%ff_msgs/Zone[] zones~%~%================================================================================~%MSG: ff_msgs/Zone~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message defines a zone, such as a Keepin or Keeoput.~%~%string name                   # Name of zone~%~%# A name can refer to multiple zones. This is the index of the zone with respect~%# to the zone name~%int32 index~%~%# Zone type~%uint8 KEEPOUT = 0       # An area the freeflyer should stay out of~%uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in ~%uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter~%~%uint8 type              # Whether the zone is a keepin, keepout, or clutter~%~%geometry_msgs/Vector3 min   # One corner of the zone~%geometry_msgs/Vector3 max   # The opposite corner of the zone~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetZones-request)))
  "Returns full string definition for message of type 'SetZones-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%time timestamp~%ff_msgs/Zone[] zones~%~%================================================================================~%MSG: ff_msgs/Zone~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# This message defines a zone, such as a Keepin or Keeoput.~%~%string name                   # Name of zone~%~%# A name can refer to multiple zones. This is the index of the zone with respect~%# to the zone name~%int32 index~%~%# Zone type~%uint8 KEEPOUT = 0       # An area the freeflyer should stay out of~%uint8 KEEPIN  = 1       # An area the freeflyer can fly freely in ~%uint8 CLUTTER = 2       # An area that the freeflyer should avoid due to clutter~%~%uint8 type              # Whether the zone is a keepin, keepout, or clutter~%~%geometry_msgs/Vector3 min   # One corner of the zone~%geometry_msgs/Vector3 max   # The opposite corner of the zone~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetZones-request>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'zones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetZones-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetZones-request
    (cl:cons ':timestamp (timestamp msg))
    (cl:cons ':zones (zones msg))
))
;//! \htmlinclude SetZones-response.msg.html

(cl:defclass <SetZones-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetZones-response (<SetZones-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetZones-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetZones-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-srv:<SetZones-response> is deprecated: use ff_msgs-srv:SetZones-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetZones-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-srv:success-val is deprecated.  Use ff_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetZones-response>) ostream)
  "Serializes a message object of type '<SetZones-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetZones-response>) istream)
  "Deserializes a message object of type '<SetZones-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetZones-response>)))
  "Returns string type for a service object of type '<SetZones-response>"
  "ff_msgs/SetZonesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetZones-response)))
  "Returns string type for a service object of type 'SetZones-response"
  "ff_msgs/SetZonesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetZones-response>)))
  "Returns md5sum for a message object of type '<SetZones-response>"
  "409154f0ce409359cbde795e43243c62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetZones-response)))
  "Returns md5sum for a message object of type 'SetZones-response"
  "409154f0ce409359cbde795e43243c62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetZones-response>)))
  "Returns full string definition for message of type '<SetZones-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetZones-response)))
  "Returns full string definition for message of type 'SetZones-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetZones-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetZones-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetZones-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetZones)))
  'SetZones-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetZones)))
  'SetZones-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetZones)))
  "Returns string type for a service object of type '<SetZones>"
  "ff_msgs/SetZones")