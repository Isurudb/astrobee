; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude VisualeyezData.msg.html

(cl:defclass <VisualeyezData> (roslisp-msg-protocol:ros-message)
  ((tcmid
    :reader tcmid
    :initarg :tcmid
    :type cl:fixnum
    :initform 0)
   (ledid
    :reader ledid
    :initarg :ledid
    :type cl:fixnum
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass VisualeyezData (<VisualeyezData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualeyezData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualeyezData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<VisualeyezData> is deprecated: use ff_msgs-msg:VisualeyezData instead.")))

(cl:ensure-generic-function 'tcmid-val :lambda-list '(m))
(cl:defmethod tcmid-val ((m <VisualeyezData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:tcmid-val is deprecated.  Use ff_msgs-msg:tcmid instead.")
  (tcmid m))

(cl:ensure-generic-function 'ledid-val :lambda-list '(m))
(cl:defmethod ledid-val ((m <VisualeyezData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:ledid-val is deprecated.  Use ff_msgs-msg:ledid instead.")
  (ledid m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <VisualeyezData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:position-val is deprecated.  Use ff_msgs-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualeyezData>) ostream)
  "Serializes a message object of type '<VisualeyezData>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tcmid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ledid)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualeyezData>) istream)
  "Deserializes a message object of type '<VisualeyezData>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'tcmid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ledid)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualeyezData>)))
  "Returns string type for a message object of type '<VisualeyezData>"
  "ff_msgs/VisualeyezData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualeyezData)))
  "Returns string type for a message object of type 'VisualeyezData"
  "ff_msgs/VisualeyezData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualeyezData>)))
  "Returns md5sum for a message object of type '<VisualeyezData>"
  "0a4f041324891dc34a11ad8b15af9d60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualeyezData)))
  "Returns md5sum for a message object of type 'VisualeyezData"
  "0a4f041324891dc34a11ad8b15af9d60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualeyezData>)))
  "Returns full string definition for message of type '<VisualeyezData>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Raw Visualeyez data.~%~%uint8 tcmid                         # Transmission control module ID~%uint8 ledid                         # Light emitting diode ID~%geometry_msgs/Vector3 position      # Coordinate ~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualeyezData)))
  "Returns full string definition for message of type 'VisualeyezData"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Raw Visualeyez data.~%~%uint8 tcmid                         # Transmission control module ID~%uint8 ledid                         # Light emitting diode ID~%geometry_msgs/Vector3 position      # Coordinate ~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualeyezData>))
  (cl:+ 0
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualeyezData>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualeyezData
    (cl:cons ':tcmid (tcmid msg))
    (cl:cons ':ledid (ledid msg))
    (cl:cons ':position (position msg))
))
