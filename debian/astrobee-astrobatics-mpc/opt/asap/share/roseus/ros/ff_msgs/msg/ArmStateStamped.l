;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ArmStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ArmStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ARMSTATESTAMPED")
  (make-package "FF_MSGS::ARMSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude ArmStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::ArmStateStamped
  :super ros::object
  :slots (_header _joint_state _gripper_state ))

(defmethod ff_msgs::ArmStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_state __joint_state) (instance ff_msgs::ArmJointState :init))
    ((:gripper_state __gripper_state) (instance ff_msgs::ArmGripperState :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_state __joint_state)
   (setq _gripper_state __gripper_state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_state
   (&rest __joint_state)
   (if (keywordp (car __joint_state))
       (send* _joint_state __joint_state)
     (progn
       (if __joint_state (setq _joint_state (car __joint_state)))
       _joint_state)))
  (:gripper_state
   (&rest __gripper_state)
   (if (keywordp (car __gripper_state))
       (send* _gripper_state __gripper_state)
     (progn
       (if __gripper_state (setq _gripper_state (car __gripper_state)))
       _gripper_state)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/ArmJointState _joint_state
    (send _joint_state :serialization-length)
    ;; ff_msgs/ArmGripperState _gripper_state
    (send _gripper_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/ArmJointState _joint_state
       (send _joint_state :serialize s)
     ;; ff_msgs/ArmGripperState _gripper_state
       (send _gripper_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/ArmJointState _joint_state
     (send _joint_state :deserialize buf ptr-) (incf ptr- (send _joint_state :serialization-length))
   ;; ff_msgs/ArmGripperState _gripper_state
     (send _gripper_state :deserialize buf ptr-) (incf ptr- (send _gripper_state :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::ArmStateStamped :md5sum-) "3861c96e90f30d3bd53dc5e09edfb937")
(setf (get ff_msgs::ArmStateStamped :datatype-) "ff_msgs/ArmStateStamped")
(setf (get ff_msgs::ArmStateStamped :definition-)
      "# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# ArmState message
#
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState

std_msgs/Header header

ff_msgs/ArmJointState joint_state
ff_msgs/ArmGripperState  gripper_state


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: ff_msgs/ArmJointState
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Arm Joint State enum.
#
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState

uint8 UNKNOWN   = 0
uint8 STOWED    = 1
uint8 DEPLOYING = 2
uint8 STOPPED   = 3
uint8 MOVING    = 4
uint8 STOWING   = 5

uint8 state

================================================================================
MSG: ff_msgs/ArmGripperState
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# Arm Gripper State enum
#
# *MUST* be kept in sync with rapid::ext::astrobee::ArmState

uint8 UNKNOWN      = 0
uint8 UNCALIBRATED = 1
uint8 CALIBRATING  = 2
uint8 CLOSED       = 3
uint8 OPEN         = 4

uint8 state

")



(provide :ff_msgs/ArmStateStamped "3861c96e90f30d3bd53dc5e09edfb937")


