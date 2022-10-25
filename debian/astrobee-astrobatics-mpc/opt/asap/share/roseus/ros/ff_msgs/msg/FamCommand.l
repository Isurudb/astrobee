;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::FamCommand)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'FamCommand (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FAMCOMMAND")
  (make-package "FF_MSGS::FAMCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude FamCommand.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::FamCommand
  :super ros::object
  :slots (_header _wrench _accel _alpha _status _position_error _position_error_integrated _attitude_error _attitude_error_integrated _attitude_error_mag _control_mode ))

(defmethod ff_msgs::FamCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:wrench __wrench) (instance geometry_msgs::Wrench :init))
    ((:accel __accel) (instance geometry_msgs::Vector3 :init))
    ((:alpha __alpha) (instance geometry_msgs::Vector3 :init))
    ((:status __status) 0)
    ((:position_error __position_error) (instance geometry_msgs::Vector3 :init))
    ((:position_error_integrated __position_error_integrated) (instance geometry_msgs::Vector3 :init))
    ((:attitude_error __attitude_error) (instance geometry_msgs::Vector3 :init))
    ((:attitude_error_integrated __attitude_error_integrated) (instance geometry_msgs::Vector3 :init))
    ((:attitude_error_mag __attitude_error_mag) 0.0)
    ((:control_mode __control_mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _wrench __wrench)
   (setq _accel __accel)
   (setq _alpha __alpha)
   (setq _status (round __status))
   (setq _position_error __position_error)
   (setq _position_error_integrated __position_error_integrated)
   (setq _attitude_error __attitude_error)
   (setq _attitude_error_integrated __attitude_error_integrated)
   (setq _attitude_error_mag (float __attitude_error_mag))
   (setq _control_mode (round __control_mode))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:accel
   (&rest __accel)
   (if (keywordp (car __accel))
       (send* _accel __accel)
     (progn
       (if __accel (setq _accel (car __accel)))
       _accel)))
  (:alpha
   (&rest __alpha)
   (if (keywordp (car __alpha))
       (send* _alpha __alpha)
     (progn
       (if __alpha (setq _alpha (car __alpha)))
       _alpha)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:position_error
   (&rest __position_error)
   (if (keywordp (car __position_error))
       (send* _position_error __position_error)
     (progn
       (if __position_error (setq _position_error (car __position_error)))
       _position_error)))
  (:position_error_integrated
   (&rest __position_error_integrated)
   (if (keywordp (car __position_error_integrated))
       (send* _position_error_integrated __position_error_integrated)
     (progn
       (if __position_error_integrated (setq _position_error_integrated (car __position_error_integrated)))
       _position_error_integrated)))
  (:attitude_error
   (&rest __attitude_error)
   (if (keywordp (car __attitude_error))
       (send* _attitude_error __attitude_error)
     (progn
       (if __attitude_error (setq _attitude_error (car __attitude_error)))
       _attitude_error)))
  (:attitude_error_integrated
   (&rest __attitude_error_integrated)
   (if (keywordp (car __attitude_error_integrated))
       (send* _attitude_error_integrated __attitude_error_integrated)
     (progn
       (if __attitude_error_integrated (setq _attitude_error_integrated (car __attitude_error_integrated)))
       _attitude_error_integrated)))
  (:attitude_error_mag
   (&optional __attitude_error_mag)
   (if __attitude_error_mag (setq _attitude_error_mag __attitude_error_mag)) _attitude_error_mag)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Wrench _wrench
    (send _wrench :serialization-length)
    ;; geometry_msgs/Vector3 _accel
    (send _accel :serialization-length)
    ;; geometry_msgs/Vector3 _alpha
    (send _alpha :serialization-length)
    ;; uint8 _status
    1
    ;; geometry_msgs/Vector3 _position_error
    (send _position_error :serialization-length)
    ;; geometry_msgs/Vector3 _position_error_integrated
    (send _position_error_integrated :serialization-length)
    ;; geometry_msgs/Vector3 _attitude_error
    (send _attitude_error :serialization-length)
    ;; geometry_msgs/Vector3 _attitude_error_integrated
    (send _attitude_error_integrated :serialization-length)
    ;; float32 _attitude_error_mag
    4
    ;; uint8 _control_mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Wrench _wrench
       (send _wrench :serialize s)
     ;; geometry_msgs/Vector3 _accel
       (send _accel :serialize s)
     ;; geometry_msgs/Vector3 _alpha
       (send _alpha :serialize s)
     ;; uint8 _status
       (write-byte _status s)
     ;; geometry_msgs/Vector3 _position_error
       (send _position_error :serialize s)
     ;; geometry_msgs/Vector3 _position_error_integrated
       (send _position_error_integrated :serialize s)
     ;; geometry_msgs/Vector3 _attitude_error
       (send _attitude_error :serialize s)
     ;; geometry_msgs/Vector3 _attitude_error_integrated
       (send _attitude_error_integrated :serialize s)
     ;; float32 _attitude_error_mag
       (sys::poke _attitude_error_mag (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _control_mode
       (write-byte _control_mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Wrench _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; geometry_msgs/Vector3 _accel
     (send _accel :deserialize buf ptr-) (incf ptr- (send _accel :serialization-length))
   ;; geometry_msgs/Vector3 _alpha
     (send _alpha :deserialize buf ptr-) (incf ptr- (send _alpha :serialization-length))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _position_error
     (send _position_error :deserialize buf ptr-) (incf ptr- (send _position_error :serialization-length))
   ;; geometry_msgs/Vector3 _position_error_integrated
     (send _position_error_integrated :deserialize buf ptr-) (incf ptr- (send _position_error_integrated :serialization-length))
   ;; geometry_msgs/Vector3 _attitude_error
     (send _attitude_error :deserialize buf ptr-) (incf ptr- (send _attitude_error :serialization-length))
   ;; geometry_msgs/Vector3 _attitude_error_integrated
     (send _attitude_error_integrated :deserialize buf ptr-) (incf ptr- (send _attitude_error_integrated :serialization-length))
   ;; float32 _attitude_error_mag
     (setq _attitude_error_mag (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _control_mode
     (setq _control_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::FamCommand :md5sum-) "baf174131dee1a8b03d9d5feac8aa809")
(setf (get ff_msgs::FamCommand :datatype-) "ff_msgs/FamCommand")
(setf (get ff_msgs::FamCommand :definition-)
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
# Command sent from control to the FAM.

std_msgs/Header header # header with time stamp

# force and torque
geometry_msgs/Wrench wrench
# linear acceleration (wrench w/out estimated mass)
geometry_msgs/Vector3 accel
# angular accceleration (wrench w/out estimated mass)
geometry_msgs/Vector3 alpha

# status byte from GNC ICD
uint8 status

# position error
geometry_msgs/Vector3 position_error
# integrated position error
geometry_msgs/Vector3 position_error_integrated

# attitude error
geometry_msgs/Vector3 attitude_error
# integrated attitude error
geometry_msgs/Vector3 attitude_error_integrated
# magnitude of attitude error
float32 attitude_error_mag

# control mode from GNC ICD
uint8 control_mode


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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

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



(provide :ff_msgs/FamCommand "baf174131dee1a8b03d9d5feac8aa809")


