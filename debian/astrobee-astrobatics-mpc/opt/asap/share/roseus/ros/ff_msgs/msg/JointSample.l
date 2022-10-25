;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::JointSample)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'JointSample (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::JOINTSAMPLE")
  (make-package "FF_MSGS::JOINTSAMPLE"))

(in-package "ROS")
;;//! \htmlinclude JointSample.msg.html


(intern "*JOINT_ENABLED*" (find-package "FF_MSGS::JOINTSAMPLE"))
(shadow '*JOINT_ENABLED* (find-package "FF_MSGS::JOINTSAMPLE"))
(defconstant ff_msgs::JointSample::*JOINT_ENABLED* 0)
(intern "*JOINT_DISABLED*" (find-package "FF_MSGS::JOINTSAMPLE"))
(shadow '*JOINT_DISABLED* (find-package "FF_MSGS::JOINTSAMPLE"))
(defconstant ff_msgs::JointSample::*JOINT_DISABLED* 1)
(defclass ff_msgs::JointSample
  :super ros::object
  :slots (_angle_pos _angle_vel _angle_acc _current _torque _temperature _status _name ))

(defmethod ff_msgs::JointSample
  (:init
   (&key
    ((:angle_pos __angle_pos) 0.0)
    ((:angle_vel __angle_vel) 0.0)
    ((:angle_acc __angle_acc) 0.0)
    ((:current __current) 0.0)
    ((:torque __torque) 0.0)
    ((:temperature __temperature) 0.0)
    ((:status __status) 0)
    ((:name __name) "")
    )
   (send-super :init)
   (setq _angle_pos (float __angle_pos))
   (setq _angle_vel (float __angle_vel))
   (setq _angle_acc (float __angle_acc))
   (setq _current (float __current))
   (setq _torque (float __torque))
   (setq _temperature (float __temperature))
   (setq _status (round __status))
   (setq _name (string __name))
   self)
  (:angle_pos
   (&optional __angle_pos)
   (if __angle_pos (setq _angle_pos __angle_pos)) _angle_pos)
  (:angle_vel
   (&optional __angle_vel)
   (if __angle_vel (setq _angle_vel __angle_vel)) _angle_vel)
  (:angle_acc
   (&optional __angle_acc)
   (if __angle_acc (setq _angle_acc __angle_acc)) _angle_acc)
  (:current
   (&optional __current)
   (if __current (setq _current __current)) _current)
  (:torque
   (&optional __torque)
   (if __torque (setq _torque __torque)) _torque)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; float32 _angle_pos
    4
    ;; float32 _angle_vel
    4
    ;; float32 _angle_acc
    4
    ;; float32 _current
    4
    ;; float32 _torque
    4
    ;; float32 _temperature
    4
    ;; uint16 _status
    2
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle_pos
       (sys::poke _angle_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_vel
       (sys::poke _angle_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_acc
       (sys::poke _angle_acc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current
       (sys::poke _current (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _torque
       (sys::poke _torque (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _status
       (write-word _status s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle_pos
     (setq _angle_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_vel
     (setq _angle_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_acc
     (setq _angle_acc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current
     (setq _current (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _torque
     (setq _torque (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _status
     (setq _status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::JointSample :md5sum-) "fe238686c8b329629bd0aa9499404e2e")
(setf (get ff_msgs::JointSample :datatype-) "ff_msgs/JointSample")
(setf (get ff_msgs::JointSample :definition-)
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
# JointSample message, based off of rapid::JointSample

# Flag values for joint status. 
# Joint is enabled
uint8 JOINT_ENABLED    = 0      # Joint enabled
uint8 JOINT_DISABLED   = 1      # Joint disabled


# Angle position (in radians) of the joint
float32 angle_pos

# Angle velocity (in radians/sec) of the joint
float32 angle_vel

# Angle acceleration (in radians/sec^2) of the joint (not being used)
float32 angle_acc

# Current draw of joint motor
float32 current

# Torque sensed at the joint (not being used)
float32 torque

# Temperature of the joint (in Celsius)
float32 temperature

# Bit field representing the state of the joint
uint16 status

# Human-readable name
string name

")



(provide :ff_msgs/JointSample "fe238686c8b329629bd0aa9499404e2e")


