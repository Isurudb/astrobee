;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ControlCommand)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ControlCommand (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CONTROLCOMMAND")
  (make-package "FF_MSGS::CONTROLCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude ControlCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MODE_IDLE*" (find-package "FF_MSGS::CONTROLCOMMAND"))
(shadow '*MODE_IDLE* (find-package "FF_MSGS::CONTROLCOMMAND"))
(defconstant ff_msgs::ControlCommand::*MODE_IDLE* 0)
(intern "*MODE_STOP*" (find-package "FF_MSGS::CONTROLCOMMAND"))
(shadow '*MODE_STOP* (find-package "FF_MSGS::CONTROLCOMMAND"))
(defconstant ff_msgs::ControlCommand::*MODE_STOP* 1)
(intern "*MODE_NOMINAL*" (find-package "FF_MSGS::CONTROLCOMMAND"))
(shadow '*MODE_NOMINAL* (find-package "FF_MSGS::CONTROLCOMMAND"))
(defconstant ff_msgs::ControlCommand::*MODE_NOMINAL* 2)
(defclass ff_msgs::ControlCommand
  :super ros::object
  :slots (_header _mode _current _next ))

(defmethod ff_msgs::ControlCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mode __mode) 0)
    ((:current __current) (instance ff_msgs::ControlState :init))
    ((:next __next) (instance ff_msgs::ControlState :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _mode (round __mode))
   (setq _current __current)
   (setq _next __next)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:current
   (&rest __current)
   (if (keywordp (car __current))
       (send* _current __current)
     (progn
       (if __current (setq _current (car __current)))
       _current)))
  (:next
   (&rest __next)
   (if (keywordp (car __next))
       (send* _next __next)
     (progn
       (if __next (setq _next (car __next)))
       _next)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _mode
    1
    ;; ff_msgs/ControlState _current
    (send _current :serialization-length)
    ;; ff_msgs/ControlState _next
    (send _next :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; ff_msgs/ControlState _current
       (send _current :serialize s)
     ;; ff_msgs/ControlState _next
       (send _next :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; ff_msgs/ControlState _current
     (send _current :deserialize buf ptr-) (incf ptr- (send _current :serialization-length))
   ;; ff_msgs/ControlState _next
     (send _next :deserialize buf ptr-) (incf ptr- (send _next :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::ControlCommand :md5sum-) "d82f6c881b46d5890f70c1dd8fdcfd1a")
(setf (get ff_msgs::ControlCommand :datatype-) "ff_msgs/ControlCommand")
(setf (get ff_msgs::ControlCommand :definition-)
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
# two copies of a ControlState message, plus a header
# mode: the current mode we are in. only two states defined so far
# current: the current ControlState trajectory we should aim for
# next: the next ControlState trajectory, just in case

std_msgs/Header header
uint8 mode
uint8 MODE_IDLE = 0
uint8 MODE_STOP = 1
uint8 MODE_NOMINAL = 2
ff_msgs/ControlState current
ff_msgs/ControlState next

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
MSG: ff_msgs/ControlState
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
# Full state vector containing Time, Pose, Vel, and Accel
# 
# when {time}
# flight_mode {string} - disctates, gains, tolerances, etc.
# pose {Point position, Quaternion orientation}
# twist {Vector3 linear, Vector3 angular}
# accel {Vector3 linear, Vector3 angular}

time when
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Twist accel

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :ff_msgs/ControlCommand "d82f6c881b46d5890f70c1dd8fdcfd1a")


