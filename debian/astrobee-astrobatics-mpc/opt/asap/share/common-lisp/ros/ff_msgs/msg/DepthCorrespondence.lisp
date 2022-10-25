; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DepthCorrespondence.msg.html

(cl:defclass <DepthCorrespondence> (roslisp-msg-protocol:ros-message)
  ((source_image_point
    :reader source_image_point
    :initarg :source_image_point
    :type ff_msgs-msg:ImagePoint
    :initform (cl:make-instance 'ff_msgs-msg:ImagePoint))
   (target_image_point
    :reader target_image_point
    :initarg :target_image_point
    :type ff_msgs-msg:ImagePoint
    :initform (cl:make-instance 'ff_msgs-msg:ImagePoint))
   (source_3d_point
    :reader source_3d_point
    :initarg :source_3d_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (target_3d_point
    :reader target_3d_point
    :initarg :target_3d_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass DepthCorrespondence (<DepthCorrespondence>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthCorrespondence>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthCorrespondence)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DepthCorrespondence> is deprecated: use ff_msgs-msg:DepthCorrespondence instead.")))

(cl:ensure-generic-function 'source_image_point-val :lambda-list '(m))
(cl:defmethod source_image_point-val ((m <DepthCorrespondence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:source_image_point-val is deprecated.  Use ff_msgs-msg:source_image_point instead.")
  (source_image_point m))

(cl:ensure-generic-function 'target_image_point-val :lambda-list '(m))
(cl:defmethod target_image_point-val ((m <DepthCorrespondence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_image_point-val is deprecated.  Use ff_msgs-msg:target_image_point instead.")
  (target_image_point m))

(cl:ensure-generic-function 'source_3d_point-val :lambda-list '(m))
(cl:defmethod source_3d_point-val ((m <DepthCorrespondence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:source_3d_point-val is deprecated.  Use ff_msgs-msg:source_3d_point instead.")
  (source_3d_point m))

(cl:ensure-generic-function 'target_3d_point-val :lambda-list '(m))
(cl:defmethod target_3d_point-val ((m <DepthCorrespondence>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:target_3d_point-val is deprecated.  Use ff_msgs-msg:target_3d_point instead.")
  (target_3d_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthCorrespondence>) ostream)
  "Serializes a message object of type '<DepthCorrespondence>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source_image_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_image_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source_3d_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_3d_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthCorrespondence>) istream)
  "Deserializes a message object of type '<DepthCorrespondence>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source_image_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_image_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source_3d_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_3d_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthCorrespondence>)))
  "Returns string type for a message object of type '<DepthCorrespondence>"
  "ff_msgs/DepthCorrespondence")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthCorrespondence)))
  "Returns string type for a message object of type 'DepthCorrespondence"
  "ff_msgs/DepthCorrespondence")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthCorrespondence>)))
  "Returns md5sum for a message object of type '<DepthCorrespondence>"
  "869ec9e0747cdb7e1fd25af7fca82639")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthCorrespondence)))
  "Returns md5sum for a message object of type 'DepthCorrespondence"
  "869ec9e0747cdb7e1fd25af7fca82639")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthCorrespondence>)))
  "Returns full string definition for message of type '<DepthCorrespondence>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%ImagePoint source_image_point~%ImagePoint target_image_point~%geometry_msgs/Point source_3d_point~%geometry_msgs/Point target_3d_point~%~%================================================================================~%MSG: ff_msgs/ImagePoint~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%~%float32 x~%float32 y~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthCorrespondence)))
  "Returns full string definition for message of type 'DepthCorrespondence"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%ImagePoint source_image_point~%ImagePoint target_image_point~%geometry_msgs/Point source_3d_point~%geometry_msgs/Point target_3d_point~%~%================================================================================~%MSG: ff_msgs/ImagePoint~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%~%float32 x~%float32 y~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthCorrespondence>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source_image_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_image_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source_3d_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_3d_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthCorrespondence>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthCorrespondence
    (cl:cons ':source_image_point (source_image_point msg))
    (cl:cons ':target_image_point (target_image_point msg))
    (cl:cons ':source_3d_point (source_3d_point msg))
    (cl:cons ':target_3d_point (target_3d_point msg))
))
