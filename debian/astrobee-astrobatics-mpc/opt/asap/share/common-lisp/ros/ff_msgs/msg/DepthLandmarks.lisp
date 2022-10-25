; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude DepthLandmarks.msg.html

(cl:defclass <DepthLandmarks> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (camera_id
    :reader camera_id
    :initarg :camera_id
    :type cl:integer
    :initform 0)
   (end_seen
    :reader end_seen
    :initarg :end_seen
    :type cl:fixnum
    :initform 0)
   (update_global_pose
    :reader update_global_pose
    :initarg :update_global_pose
    :type cl:fixnum
    :initform 0)
   (sensor_T_handrail
    :reader sensor_T_handrail
    :initarg :sensor_T_handrail
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (sensor_t_line_points
    :reader sensor_t_line_points
    :initarg :sensor_t_line_points
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32)))
   (sensor_t_line_endpoints
    :reader sensor_t_line_endpoints
    :initarg :sensor_t_line_endpoints
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (sensor_t_plane_points
    :reader sensor_t_plane_points
    :initarg :sensor_t_plane_points
    :type (cl:vector geometry_msgs-msg:Point32)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point32 :initial-element (cl:make-instance 'geometry_msgs-msg:Point32)))
   (landmarks
    :reader landmarks
    :initarg :landmarks
    :type (cl:vector ff_msgs-msg:DepthLandmark)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:DepthLandmark :initial-element (cl:make-instance 'ff_msgs-msg:DepthLandmark))))
)

(cl:defclass DepthLandmarks (<DepthLandmarks>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthLandmarks>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthLandmarks)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<DepthLandmarks> is deprecated: use ff_msgs-msg:DepthLandmarks instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'camera_id-val :lambda-list '(m))
(cl:defmethod camera_id-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:camera_id-val is deprecated.  Use ff_msgs-msg:camera_id instead.")
  (camera_id m))

(cl:ensure-generic-function 'end_seen-val :lambda-list '(m))
(cl:defmethod end_seen-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:end_seen-val is deprecated.  Use ff_msgs-msg:end_seen instead.")
  (end_seen m))

(cl:ensure-generic-function 'update_global_pose-val :lambda-list '(m))
(cl:defmethod update_global_pose-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:update_global_pose-val is deprecated.  Use ff_msgs-msg:update_global_pose instead.")
  (update_global_pose m))

(cl:ensure-generic-function 'sensor_T_handrail-val :lambda-list '(m))
(cl:defmethod sensor_T_handrail-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:sensor_T_handrail-val is deprecated.  Use ff_msgs-msg:sensor_T_handrail instead.")
  (sensor_T_handrail m))

(cl:ensure-generic-function 'sensor_t_line_points-val :lambda-list '(m))
(cl:defmethod sensor_t_line_points-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:sensor_t_line_points-val is deprecated.  Use ff_msgs-msg:sensor_t_line_points instead.")
  (sensor_t_line_points m))

(cl:ensure-generic-function 'sensor_t_line_endpoints-val :lambda-list '(m))
(cl:defmethod sensor_t_line_endpoints-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:sensor_t_line_endpoints-val is deprecated.  Use ff_msgs-msg:sensor_t_line_endpoints instead.")
  (sensor_t_line_endpoints m))

(cl:ensure-generic-function 'sensor_t_plane_points-val :lambda-list '(m))
(cl:defmethod sensor_t_plane_points-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:sensor_t_plane_points-val is deprecated.  Use ff_msgs-msg:sensor_t_plane_points instead.")
  (sensor_t_plane_points m))

(cl:ensure-generic-function 'landmarks-val :lambda-list '(m))
(cl:defmethod landmarks-val ((m <DepthLandmarks>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:landmarks-val is deprecated.  Use ff_msgs-msg:landmarks instead.")
  (landmarks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthLandmarks>) ostream)
  "Serializes a message object of type '<DepthLandmarks>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'camera_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_seen)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'update_global_pose)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sensor_T_handrail) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensor_t_line_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sensor_t_line_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensor_t_line_endpoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sensor_t_line_endpoints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensor_t_plane_points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sensor_t_plane_points))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'landmarks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'landmarks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthLandmarks>) istream)
  "Deserializes a message object of type '<DepthLandmarks>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'camera_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'end_seen)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'update_global_pose)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sensor_T_handrail) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensor_t_line_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensor_t_line_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensor_t_line_endpoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensor_t_line_endpoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensor_t_plane_points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensor_t_plane_points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point32))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'landmarks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'landmarks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:DepthLandmark))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthLandmarks>)))
  "Returns string type for a message object of type '<DepthLandmarks>"
  "ff_msgs/DepthLandmarks")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthLandmarks)))
  "Returns string type for a message object of type 'DepthLandmarks"
  "ff_msgs/DepthLandmarks")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthLandmarks>)))
  "Returns md5sum for a message object of type '<DepthLandmarks>"
  "7fc86a54f996c15d2798a19b023404dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthLandmarks)))
  "Returns md5sum for a message object of type 'DepthLandmarks"
  "7fc86a54f996c15d2798a19b023404dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthLandmarks>)))
  "Returns full string definition for message of type '<DepthLandmarks>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An observation of a handrail from a depth image.~%~%Header header                                  # Image header, with time stamp~%uint32 camera_id                               # Image ID, associated with registration~%uint8 end_seen                                 # Whether the handrail endpoint was detected~%uint8 update_global_pose                       # Whether to update the global pose~%geometry_msgs/Pose sensor_T_handrail           # Handrail center in the sensor frame~%geometry_msgs/Point32[] sensor_t_line_points   # Detected line points~%geometry_msgs/Point[] sensor_t_line_endpoints  # Detected line endpoints~%geometry_msgs/Point32[] sensor_t_plane_points  # Detected plane points~%ff_msgs/DepthLandmark[] landmarks              # List of landmarks seen TODO(rsoussan): This should be removed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: ff_msgs/DepthLandmark~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A landmark seen from a depth landmark~%~%float32 u     # First coordinate in the image plane~%float32 v     # Second coordinate in the image plane~%float32 w     # Depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthLandmarks)))
  "Returns full string definition for message of type 'DepthLandmarks"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# An observation of a handrail from a depth image.~%~%Header header                                  # Image header, with time stamp~%uint32 camera_id                               # Image ID, associated with registration~%uint8 end_seen                                 # Whether the handrail endpoint was detected~%uint8 update_global_pose                       # Whether to update the global pose~%geometry_msgs/Pose sensor_T_handrail           # Handrail center in the sensor frame~%geometry_msgs/Point32[] sensor_t_line_points   # Detected line points~%geometry_msgs/Point[] sensor_t_line_endpoints  # Detected line endpoints~%geometry_msgs/Point32[] sensor_t_plane_points  # Detected plane points~%ff_msgs/DepthLandmark[] landmarks              # List of landmarks seen TODO(rsoussan): This should be removed~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: ff_msgs/DepthLandmark~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%#~%# All rights reserved.~%#~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%#~%#     http://www.apache.org/licenses/LICENSE-2.0~%#~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# A landmark seen from a depth landmark~%~%float32 u     # First coordinate in the image plane~%float32 v     # Second coordinate in the image plane~%float32 w     # Depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthLandmarks>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sensor_T_handrail))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor_t_line_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor_t_line_endpoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor_t_plane_points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'landmarks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthLandmarks>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthLandmarks
    (cl:cons ':header (header msg))
    (cl:cons ':camera_id (camera_id msg))
    (cl:cons ':end_seen (end_seen msg))
    (cl:cons ':update_global_pose (update_global_pose msg))
    (cl:cons ':sensor_T_handrail (sensor_T_handrail msg))
    (cl:cons ':sensor_t_line_points (sensor_t_line_points msg))
    (cl:cons ':sensor_t_line_endpoints (sensor_t_line_endpoints msg))
    (cl:cons ':sensor_t_plane_points (sensor_t_plane_points msg))
    (cl:cons ':landmarks (landmarks msg))
))
