; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude GuestScienceConfig.msg.html

(cl:defclass <GuestScienceConfig> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (serial
    :reader serial
    :initarg :serial
    :type cl:integer
    :initform 0)
   (apks
    :reader apks
    :initarg :apks
    :type (cl:vector ff_msgs-msg:GuestScienceApk)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:GuestScienceApk :initial-element (cl:make-instance 'ff_msgs-msg:GuestScienceApk))))
)

(cl:defclass GuestScienceConfig (<GuestScienceConfig>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GuestScienceConfig>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GuestScienceConfig)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<GuestScienceConfig> is deprecated: use ff_msgs-msg:GuestScienceConfig instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GuestScienceConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'serial-val :lambda-list '(m))
(cl:defmethod serial-val ((m <GuestScienceConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:serial-val is deprecated.  Use ff_msgs-msg:serial instead.")
  (serial m))

(cl:ensure-generic-function 'apks-val :lambda-list '(m))
(cl:defmethod apks-val ((m <GuestScienceConfig>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:apks-val is deprecated.  Use ff_msgs-msg:apks instead.")
  (apks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GuestScienceConfig>) ostream)
  "Serializes a message object of type '<GuestScienceConfig>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'serial)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'apks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'apks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GuestScienceConfig>) istream)
  "Deserializes a message object of type '<GuestScienceConfig>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'serial) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'apks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'apks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:GuestScienceApk))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GuestScienceConfig>)))
  "Returns string type for a message object of type '<GuestScienceConfig>"
  "ff_msgs/GuestScienceConfig")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GuestScienceConfig)))
  "Returns string type for a message object of type 'GuestScienceConfig"
  "ff_msgs/GuestScienceConfig")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GuestScienceConfig>)))
  "Returns md5sum for a message object of type '<GuestScienceConfig>"
  "c6f632ed2411de4159494a3e7754794e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GuestScienceConfig)))
  "Returns md5sum for a message object of type 'GuestScienceConfig"
  "c6f632ed2411de4159494a3e7754794e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GuestScienceConfig>)))
  "Returns full string definition for message of type '<GuestScienceConfig>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to store a list of guest science APKs and information relevant to~%# the APK~%~%# Header with timestamp~%std_msgs/Header header~%~%# Used for guest science config and state message synchronization on the ground~%int64 serial~%~%ff_msgs/GuestScienceApk[] apks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/GuestScienceApk~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to contain information about a guest science apk~%~%# Full apk name~%string apk_name~%~%# Short (human readable) name of the apk~%string short_name~%~%# Whether the apk is primary or secondary~%bool primary~%~%# List of commands the apk will accept~%ff_msgs/GuestScienceCommand[] commands~%~%================================================================================~%MSG: ff_msgs/GuestScienceCommand~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to store guest science commands~%~%# Name of command~%string name~%~%# Syntax of the command~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GuestScienceConfig)))
  "Returns full string definition for message of type 'GuestScienceConfig"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to store a list of guest science APKs and information relevant to~%# the APK~%~%# Header with timestamp~%std_msgs/Header header~%~%# Used for guest science config and state message synchronization on the ground~%int64 serial~%~%ff_msgs/GuestScienceApk[] apks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/GuestScienceApk~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to contain information about a guest science apk~%~%# Full apk name~%string apk_name~%~%# Short (human readable) name of the apk~%string short_name~%~%# Whether the apk is primary or secondary~%bool primary~%~%# List of commands the apk will accept~%ff_msgs/GuestScienceCommand[] commands~%~%================================================================================~%MSG: ff_msgs/GuestScienceCommand~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to store guest science commands~%~%# Name of command~%string name~%~%# Syntax of the command~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GuestScienceConfig>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'apks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GuestScienceConfig>))
  "Converts a ROS message object to a list"
  (cl:list 'GuestScienceConfig
    (cl:cons ':header (header msg))
    (cl:cons ':serial (serial msg))
    (cl:cons ':apks (apks msg))
))