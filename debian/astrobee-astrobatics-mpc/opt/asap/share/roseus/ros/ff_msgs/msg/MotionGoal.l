;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::MotionGoal)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'MotionGoal (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::MOTIONGOAL")
  (make-package "FF_MSGS::MOTIONGOAL"))

(in-package "ROS")
;;//! \htmlinclude MotionGoal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*STOP*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*STOP* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*STOP* 0)
(intern "*IDLE*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*IDLE* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*IDLE* 1)
(intern "*EXEC*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*EXEC* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*EXEC* 2)
(intern "*MOVE*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*MOVE* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*MOVE* 3)
(intern "*PREP*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*PREP* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*PREP* 4)
(intern "*OFF*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*OFF* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*OFF* "off")
(intern "*NOMINAL*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*NOMINAL* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*NOMINAL* "nominal")
(intern "*DIFFICULT*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*DIFFICULT* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*DIFFICULT* "difficult")
(intern "*QUIET*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*QUIET* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*QUIET* "quiet")
(intern "*PERCHING*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*PERCHING* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*PERCHING* "perching")
(intern "*UNPERCHING*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*UNPERCHING* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*UNPERCHING* "unperching")
(intern "*DOCKING*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*DOCKING* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*DOCKING* "docking")
(intern "*UNDOCKING*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*UNDOCKING* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*UNDOCKING* "undocking")
(intern "*PRECISION*" (find-package "FF_MSGS::MOTIONGOAL"))
(shadow '*PRECISION* (find-package "FF_MSGS::MOTIONGOAL"))
(defconstant ff_msgs::MotionGoal::*PRECISION* "precision")
(defclass ff_msgs::MotionGoal
  :super ros::object
  :slots (_command _flight_mode _segment _states _reference_frame ))

(defmethod ff_msgs::MotionGoal
  (:init
   (&key
    ((:command __command) 0)
    ((:flight_mode __flight_mode) "")
    ((:segment __segment) (let (r) (dotimes (i 0) (push (instance ff_msgs::ControlState :init) r)) r))
    ((:states __states) (let (r) (dotimes (i 0) (push (instance geometry_msgs::PoseStamped :init) r)) r))
    ((:reference_frame __reference_frame) "")
    )
   (send-super :init)
   (setq _command (round __command))
   (setq _flight_mode (string __flight_mode))
   (setq _segment __segment)
   (setq _states __states)
   (setq _reference_frame (string __reference_frame))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:flight_mode
   (&optional __flight_mode)
   (if __flight_mode (setq _flight_mode __flight_mode)) _flight_mode)
  (:segment
   (&rest __segment)
   (if (keywordp (car __segment))
       (send* _segment __segment)
     (progn
       (if __segment (setq _segment (car __segment)))
       _segment)))
  (:states
   (&rest __states)
   (if (keywordp (car __states))
       (send* _states __states)
     (progn
       (if __states (setq _states (car __states)))
       _states)))
  (:reference_frame
   (&optional __reference_frame)
   (if __reference_frame (setq _reference_frame __reference_frame)) _reference_frame)
  (:serialization-length
   ()
   (+
    ;; uint8 _command
    1
    ;; string _flight_mode
    4 (length _flight_mode)
    ;; ff_msgs/ControlState[] _segment
    (apply #'+ (send-all _segment :serialization-length)) 4
    ;; geometry_msgs/PoseStamped[] _states
    (apply #'+ (send-all _states :serialization-length)) 4
    ;; string _reference_frame
    4 (length _reference_frame)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _command
       (write-byte _command s)
     ;; string _flight_mode
       (write-long (length _flight_mode) s) (princ _flight_mode s)
     ;; ff_msgs/ControlState[] _segment
     (write-long (length _segment) s)
     (dolist (elem _segment)
       (send elem :serialize s)
       )
     ;; geometry_msgs/PoseStamped[] _states
     (write-long (length _states) s)
     (dolist (elem _states)
       (send elem :serialize s)
       )
     ;; string _reference_frame
       (write-long (length _reference_frame) s) (princ _reference_frame s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _flight_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flight_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/ControlState[] _segment
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _segment (let (r) (dotimes (i n) (push (instance ff_msgs::ControlState :init) r)) r))
     (dolist (elem- _segment)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/PoseStamped[] _states
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _states (let (r) (dotimes (i n) (push (instance geometry_msgs::PoseStamped :init) r)) r))
     (dolist (elem- _states)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string _reference_frame
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reference_frame (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::MotionGoal :md5sum-) "1f5494c5016459c2ad2035dda3ef7acb")
(setf (get ff_msgs::MotionGoal :datatype-) "ff_msgs/MotionGoal")
(setf (get ff_msgs::MotionGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
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
# This message describes the MOTION action offered by the CHOREOGRAPHER

# Desired command
uint8 command
uint8 STOP        = 0  # Stop immediately
uint8 IDLE        = 1  # Idle immediately
uint8 EXEC        = 2  # Execute a given segment
uint8 MOVE        = 3  # Move through a given set of poses
uint8 PREP        = 4  # Prepare the system for a given flight mode

# ALL COMMANDS: flight mode. If left empty, the default will be used.
string flight_mode
string OFF        = off
string NOMINAL    = nominal
string DIFFICULT  = difficult
string QUIET      = quiet

# These four values no longer used (2020/10). Keep for backward
# compatibility with archived ISS telemetry bags.
string PERCHING   = perching
string UNPERCHING = unperching
string DOCKING    = docking
string UNDOCKING  = undocking

string PRECISION  = precision

# EXECUTE ONLY : The segment
ff_msgs/ControlState[] segment

# MOVE ONLY: Desired state sequence
geometry_msgs/PoseStamped[] states

string reference_frame


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
================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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

")



(provide :ff_msgs/MotionGoal "1f5494c5016459c2ad2035dda3ef7acb")


