; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude GuestScienceApk.msg.html

(cl:defclass <GuestScienceApk> (roslisp-msg-protocol:ros-message)
  ((apk_name
    :reader apk_name
    :initarg :apk_name
    :type cl:string
    :initform "")
   (short_name
    :reader short_name
    :initarg :short_name
    :type cl:string
    :initform "")
   (primary
    :reader primary
    :initarg :primary
    :type cl:boolean
    :initform cl:nil)
   (commands
    :reader commands
    :initarg :commands
    :type (cl:vector ff_msgs-msg:GuestScienceCommand)
   :initform (cl:make-array 0 :element-type 'ff_msgs-msg:GuestScienceCommand :initial-element (cl:make-instance 'ff_msgs-msg:GuestScienceCommand))))
)

(cl:defclass GuestScienceApk (<GuestScienceApk>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GuestScienceApk>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GuestScienceApk)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<GuestScienceApk> is deprecated: use ff_msgs-msg:GuestScienceApk instead.")))

(cl:ensure-generic-function 'apk_name-val :lambda-list '(m))
(cl:defmethod apk_name-val ((m <GuestScienceApk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:apk_name-val is deprecated.  Use ff_msgs-msg:apk_name instead.")
  (apk_name m))

(cl:ensure-generic-function 'short_name-val :lambda-list '(m))
(cl:defmethod short_name-val ((m <GuestScienceApk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:short_name-val is deprecated.  Use ff_msgs-msg:short_name instead.")
  (short_name m))

(cl:ensure-generic-function 'primary-val :lambda-list '(m))
(cl:defmethod primary-val ((m <GuestScienceApk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:primary-val is deprecated.  Use ff_msgs-msg:primary instead.")
  (primary m))

(cl:ensure-generic-function 'commands-val :lambda-list '(m))
(cl:defmethod commands-val ((m <GuestScienceApk>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:commands-val is deprecated.  Use ff_msgs-msg:commands instead.")
  (commands m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GuestScienceApk>) ostream)
  "Serializes a message object of type '<GuestScienceApk>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'apk_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'apk_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'short_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'short_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'primary) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'commands))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'commands))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GuestScienceApk>) istream)
  "Deserializes a message object of type '<GuestScienceApk>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'apk_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'apk_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'short_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'short_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'primary) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'commands) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'commands)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ff_msgs-msg:GuestScienceCommand))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GuestScienceApk>)))
  "Returns string type for a message object of type '<GuestScienceApk>"
  "ff_msgs/GuestScienceApk")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GuestScienceApk)))
  "Returns string type for a message object of type 'GuestScienceApk"
  "ff_msgs/GuestScienceApk")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GuestScienceApk>)))
  "Returns md5sum for a message object of type '<GuestScienceApk>"
  "8ed1d23e09733f18dbf96d2f9cd798e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GuestScienceApk)))
  "Returns md5sum for a message object of type 'GuestScienceApk"
  "8ed1d23e09733f18dbf96d2f9cd798e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GuestScienceApk>)))
  "Returns full string definition for message of type '<GuestScienceApk>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to contain information about a guest science apk~%~%# Full apk name~%string apk_name~%~%# Short (human readable) name of the apk~%string short_name~%~%# Whether the apk is primary or secondary~%bool primary~%~%# List of commands the apk will accept~%ff_msgs/GuestScienceCommand[] commands~%~%================================================================================~%MSG: ff_msgs/GuestScienceCommand~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to store guest science commands~%~%# Name of command~%string name~%~%# Syntax of the command~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GuestScienceApk)))
  "Returns full string definition for message of type 'GuestScienceApk"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to contain information about a guest science apk~%~%# Full apk name~%string apk_name~%~%# Short (human readable) name of the apk~%string short_name~%~%# Whether the apk is primary or secondary~%bool primary~%~%# List of commands the apk will accept~%ff_msgs/GuestScienceCommand[] commands~%~%================================================================================~%MSG: ff_msgs/GuestScienceCommand~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Message used to store guest science commands~%~%# Name of command~%string name~%~%# Syntax of the command~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GuestScienceApk>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'apk_name))
     4 (cl:length (cl:slot-value msg 'short_name))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'commands) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GuestScienceApk>))
  "Converts a ROS message object to a list"
  (cl:list 'GuestScienceApk
    (cl:cons ':apk_name (apk_name msg))
    (cl:cons ':short_name (short_name msg))
    (cl:cons ':primary (primary msg))
    (cl:cons ':commands (commands msg))
))
