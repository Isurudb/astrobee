; Auto-generated. Do not edit!


(cl:in-package ff_msgs-msg)


;//! \htmlinclude ArmStateStamped.msg.html

(cl:defclass <ArmStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (joint_state
    :reader joint_state
    :initarg :joint_state
    :type ff_msgs-msg:ArmJointState
    :initform (cl:make-instance 'ff_msgs-msg:ArmJointState))
   (gripper_state
    :reader gripper_state
    :initarg :gripper_state
    :type ff_msgs-msg:ArmGripperState
    :initform (cl:make-instance 'ff_msgs-msg:ArmGripperState)))
)

(cl:defclass ArmStateStamped (<ArmStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ff_msgs-msg:<ArmStateStamped> is deprecated: use ff_msgs-msg:ArmStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ArmStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:header-val is deprecated.  Use ff_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_state-val :lambda-list '(m))
(cl:defmethod joint_state-val ((m <ArmStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:joint_state-val is deprecated.  Use ff_msgs-msg:joint_state instead.")
  (joint_state m))

(cl:ensure-generic-function 'gripper_state-val :lambda-list '(m))
(cl:defmethod gripper_state-val ((m <ArmStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ff_msgs-msg:gripper_state-val is deprecated.  Use ff_msgs-msg:gripper_state instead.")
  (gripper_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmStateStamped>) ostream)
  "Serializes a message object of type '<ArmStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joint_state) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gripper_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmStateStamped>) istream)
  "Deserializes a message object of type '<ArmStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joint_state) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gripper_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmStateStamped>)))
  "Returns string type for a message object of type '<ArmStateStamped>"
  "ff_msgs/ArmStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmStateStamped)))
  "Returns string type for a message object of type 'ArmStateStamped"
  "ff_msgs/ArmStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmStateStamped>)))
  "Returns md5sum for a message object of type '<ArmStateStamped>"
  "3861c96e90f30d3bd53dc5e09edfb937")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmStateStamped)))
  "Returns md5sum for a message object of type 'ArmStateStamped"
  "3861c96e90f30d3bd53dc5e09edfb937")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmStateStamped>)))
  "Returns full string definition for message of type '<ArmStateStamped>"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# ArmState message~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%std_msgs/Header header~%~%ff_msgs/ArmJointState joint_state~%ff_msgs/ArmGripperState  gripper_state~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/ArmJointState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Joint State enum.~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN   = 0~%uint8 STOWED    = 1~%uint8 DEPLOYING = 2~%uint8 STOPPED   = 3~%uint8 MOVING    = 4~%uint8 STOWING   = 5~%~%uint8 state~%~%================================================================================~%MSG: ff_msgs/ArmGripperState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Gripper State enum~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN      = 0~%uint8 UNCALIBRATED = 1~%uint8 CALIBRATING  = 2~%uint8 CLOSED       = 3~%uint8 OPEN         = 4~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmStateStamped)))
  "Returns full string definition for message of type 'ArmStateStamped"
  (cl:format cl:nil "# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# ArmState message~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%std_msgs/Header header~%~%ff_msgs/ArmJointState joint_state~%ff_msgs/ArmGripperState  gripper_state~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ff_msgs/ArmJointState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Joint State enum.~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN   = 0~%uint8 STOWED    = 1~%uint8 DEPLOYING = 2~%uint8 STOPPED   = 3~%uint8 MOVING    = 4~%uint8 STOWING   = 5~%~%uint8 state~%~%================================================================================~%MSG: ff_msgs/ArmGripperState~%# Copyright (c) 2017, United States Government, as represented by the~%# Administrator of the National Aeronautics and Space Administration.~%# ~%# All rights reserved.~%# ~%# The Astrobee platform is licensed under the Apache License, Version 2.0~%# (the \"License\"); you may not use this file except in compliance with the~%# License. You may obtain a copy of the License at~%# ~%#     http://www.apache.org/licenses/LICENSE-2.0~%# ~%# Unless required by applicable law or agreed to in writing, software~%# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT~%# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the~%# License for the specific language governing permissions and limitations~%# under the License.~%#~%# Arm Gripper State enum~%#~%# *MUST* be kept in sync with rapid::ext::astrobee::ArmState~%~%uint8 UNKNOWN      = 0~%uint8 UNCALIBRATED = 1~%uint8 CALIBRATING  = 2~%uint8 CLOSED       = 3~%uint8 OPEN         = 4~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joint_state))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gripper_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':joint_state (joint_state msg))
    (cl:cons ':gripper_state (gripper_state msg))
))
