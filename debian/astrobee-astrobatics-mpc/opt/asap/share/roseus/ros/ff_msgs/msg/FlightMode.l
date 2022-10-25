;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::FlightMode)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'FlightMode (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FLIGHTMODE")
  (make-package "FF_MSGS::FLIGHTMODE"))

(in-package "ROS")
;;//! \htmlinclude FlightMode.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*SPEED_MIN*" (find-package "FF_MSGS::FLIGHTMODE"))
(shadow '*SPEED_MIN* (find-package "FF_MSGS::FLIGHTMODE"))
(defconstant ff_msgs::FlightMode::*SPEED_MIN* 0)
(intern "*SPEED_OFF*" (find-package "FF_MSGS::FLIGHTMODE"))
(shadow '*SPEED_OFF* (find-package "FF_MSGS::FLIGHTMODE"))
(defconstant ff_msgs::FlightMode::*SPEED_OFF* 0)
(intern "*SPEED_QUIET*" (find-package "FF_MSGS::FLIGHTMODE"))
(shadow '*SPEED_QUIET* (find-package "FF_MSGS::FLIGHTMODE"))
(defconstant ff_msgs::FlightMode::*SPEED_QUIET* 1)
(intern "*SPEED_NOMINAL*" (find-package "FF_MSGS::FLIGHTMODE"))
(shadow '*SPEED_NOMINAL* (find-package "FF_MSGS::FLIGHTMODE"))
(defconstant ff_msgs::FlightMode::*SPEED_NOMINAL* 2)
(intern "*SPEED_AGGRESSIVE*" (find-package "FF_MSGS::FLIGHTMODE"))
(shadow '*SPEED_AGGRESSIVE* (find-package "FF_MSGS::FLIGHTMODE"))
(defconstant ff_msgs::FlightMode::*SPEED_AGGRESSIVE* 3)
(intern "*SPEED_MAX*" (find-package "FF_MSGS::FLIGHTMODE"))
(shadow '*SPEED_MAX* (find-package "FF_MSGS::FLIGHTMODE"))
(defconstant ff_msgs::FlightMode::*SPEED_MAX* 3)
(defclass ff_msgs::FlightMode
  :super ros::object
  :slots (_header _name _control_enabled _tolerance_pos_endpoint _tolerance_pos _tolerance_vel _tolerance_att _tolerance_omega _tolerance_time _att_kp _att_ki _omega_kd _pos_kp _pos_ki _vel_kd _hard_limit_vel _hard_limit_accel _hard_limit_omega _hard_limit_alpha _speed ))

(defmethod ff_msgs::FlightMode
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:control_enabled __control_enabled) nil)
    ((:tolerance_pos_endpoint __tolerance_pos_endpoint) 0.0)
    ((:tolerance_pos __tolerance_pos) 0.0)
    ((:tolerance_vel __tolerance_vel) 0.0)
    ((:tolerance_att __tolerance_att) 0.0)
    ((:tolerance_omega __tolerance_omega) 0.0)
    ((:tolerance_time __tolerance_time) 0.0)
    ((:att_kp __att_kp) (instance geometry_msgs::Vector3 :init))
    ((:att_ki __att_ki) (instance geometry_msgs::Vector3 :init))
    ((:omega_kd __omega_kd) (instance geometry_msgs::Vector3 :init))
    ((:pos_kp __pos_kp) (instance geometry_msgs::Vector3 :init))
    ((:pos_ki __pos_ki) (instance geometry_msgs::Vector3 :init))
    ((:vel_kd __vel_kd) (instance geometry_msgs::Vector3 :init))
    ((:hard_limit_vel __hard_limit_vel) 0.0)
    ((:hard_limit_accel __hard_limit_accel) 0.0)
    ((:hard_limit_omega __hard_limit_omega) 0.0)
    ((:hard_limit_alpha __hard_limit_alpha) 0.0)
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _control_enabled __control_enabled)
   (setq _tolerance_pos_endpoint (float __tolerance_pos_endpoint))
   (setq _tolerance_pos (float __tolerance_pos))
   (setq _tolerance_vel (float __tolerance_vel))
   (setq _tolerance_att (float __tolerance_att))
   (setq _tolerance_omega (float __tolerance_omega))
   (setq _tolerance_time (float __tolerance_time))
   (setq _att_kp __att_kp)
   (setq _att_ki __att_ki)
   (setq _omega_kd __omega_kd)
   (setq _pos_kp __pos_kp)
   (setq _pos_ki __pos_ki)
   (setq _vel_kd __vel_kd)
   (setq _hard_limit_vel (float __hard_limit_vel))
   (setq _hard_limit_accel (float __hard_limit_accel))
   (setq _hard_limit_omega (float __hard_limit_omega))
   (setq _hard_limit_alpha (float __hard_limit_alpha))
   (setq _speed (round __speed))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:control_enabled
   (&optional __control_enabled)
   (if __control_enabled (setq _control_enabled __control_enabled)) _control_enabled)
  (:tolerance_pos_endpoint
   (&optional __tolerance_pos_endpoint)
   (if __tolerance_pos_endpoint (setq _tolerance_pos_endpoint __tolerance_pos_endpoint)) _tolerance_pos_endpoint)
  (:tolerance_pos
   (&optional __tolerance_pos)
   (if __tolerance_pos (setq _tolerance_pos __tolerance_pos)) _tolerance_pos)
  (:tolerance_vel
   (&optional __tolerance_vel)
   (if __tolerance_vel (setq _tolerance_vel __tolerance_vel)) _tolerance_vel)
  (:tolerance_att
   (&optional __tolerance_att)
   (if __tolerance_att (setq _tolerance_att __tolerance_att)) _tolerance_att)
  (:tolerance_omega
   (&optional __tolerance_omega)
   (if __tolerance_omega (setq _tolerance_omega __tolerance_omega)) _tolerance_omega)
  (:tolerance_time
   (&optional __tolerance_time)
   (if __tolerance_time (setq _tolerance_time __tolerance_time)) _tolerance_time)
  (:att_kp
   (&rest __att_kp)
   (if (keywordp (car __att_kp))
       (send* _att_kp __att_kp)
     (progn
       (if __att_kp (setq _att_kp (car __att_kp)))
       _att_kp)))
  (:att_ki
   (&rest __att_ki)
   (if (keywordp (car __att_ki))
       (send* _att_ki __att_ki)
     (progn
       (if __att_ki (setq _att_ki (car __att_ki)))
       _att_ki)))
  (:omega_kd
   (&rest __omega_kd)
   (if (keywordp (car __omega_kd))
       (send* _omega_kd __omega_kd)
     (progn
       (if __omega_kd (setq _omega_kd (car __omega_kd)))
       _omega_kd)))
  (:pos_kp
   (&rest __pos_kp)
   (if (keywordp (car __pos_kp))
       (send* _pos_kp __pos_kp)
     (progn
       (if __pos_kp (setq _pos_kp (car __pos_kp)))
       _pos_kp)))
  (:pos_ki
   (&rest __pos_ki)
   (if (keywordp (car __pos_ki))
       (send* _pos_ki __pos_ki)
     (progn
       (if __pos_ki (setq _pos_ki (car __pos_ki)))
       _pos_ki)))
  (:vel_kd
   (&rest __vel_kd)
   (if (keywordp (car __vel_kd))
       (send* _vel_kd __vel_kd)
     (progn
       (if __vel_kd (setq _vel_kd (car __vel_kd)))
       _vel_kd)))
  (:hard_limit_vel
   (&optional __hard_limit_vel)
   (if __hard_limit_vel (setq _hard_limit_vel __hard_limit_vel)) _hard_limit_vel)
  (:hard_limit_accel
   (&optional __hard_limit_accel)
   (if __hard_limit_accel (setq _hard_limit_accel __hard_limit_accel)) _hard_limit_accel)
  (:hard_limit_omega
   (&optional __hard_limit_omega)
   (if __hard_limit_omega (setq _hard_limit_omega __hard_limit_omega)) _hard_limit_omega)
  (:hard_limit_alpha
   (&optional __hard_limit_alpha)
   (if __hard_limit_alpha (setq _hard_limit_alpha __hard_limit_alpha)) _hard_limit_alpha)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; bool _control_enabled
    1
    ;; float32 _tolerance_pos_endpoint
    4
    ;; float32 _tolerance_pos
    4
    ;; float32 _tolerance_vel
    4
    ;; float32 _tolerance_att
    4
    ;; float32 _tolerance_omega
    4
    ;; float32 _tolerance_time
    4
    ;; geometry_msgs/Vector3 _att_kp
    (send _att_kp :serialization-length)
    ;; geometry_msgs/Vector3 _att_ki
    (send _att_ki :serialization-length)
    ;; geometry_msgs/Vector3 _omega_kd
    (send _omega_kd :serialization-length)
    ;; geometry_msgs/Vector3 _pos_kp
    (send _pos_kp :serialization-length)
    ;; geometry_msgs/Vector3 _pos_ki
    (send _pos_ki :serialization-length)
    ;; geometry_msgs/Vector3 _vel_kd
    (send _vel_kd :serialization-length)
    ;; float32 _hard_limit_vel
    4
    ;; float32 _hard_limit_accel
    4
    ;; float32 _hard_limit_omega
    4
    ;; float32 _hard_limit_alpha
    4
    ;; uint8 _speed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _control_enabled
       (if _control_enabled (write-byte -1 s) (write-byte 0 s))
     ;; float32 _tolerance_pos_endpoint
       (sys::poke _tolerance_pos_endpoint (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tolerance_pos
       (sys::poke _tolerance_pos (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tolerance_vel
       (sys::poke _tolerance_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tolerance_att
       (sys::poke _tolerance_att (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tolerance_omega
       (sys::poke _tolerance_omega (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tolerance_time
       (sys::poke _tolerance_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Vector3 _att_kp
       (send _att_kp :serialize s)
     ;; geometry_msgs/Vector3 _att_ki
       (send _att_ki :serialize s)
     ;; geometry_msgs/Vector3 _omega_kd
       (send _omega_kd :serialize s)
     ;; geometry_msgs/Vector3 _pos_kp
       (send _pos_kp :serialize s)
     ;; geometry_msgs/Vector3 _pos_ki
       (send _pos_ki :serialize s)
     ;; geometry_msgs/Vector3 _vel_kd
       (send _vel_kd :serialize s)
     ;; float32 _hard_limit_vel
       (sys::poke _hard_limit_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hard_limit_accel
       (sys::poke _hard_limit_accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hard_limit_omega
       (sys::poke _hard_limit_omega (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _hard_limit_alpha
       (sys::poke _hard_limit_alpha (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _speed
       (write-byte _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _control_enabled
     (setq _control_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _tolerance_pos_endpoint
     (setq _tolerance_pos_endpoint (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tolerance_pos
     (setq _tolerance_pos (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tolerance_vel
     (setq _tolerance_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tolerance_att
     (setq _tolerance_att (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tolerance_omega
     (setq _tolerance_omega (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tolerance_time
     (setq _tolerance_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _att_kp
     (send _att_kp :deserialize buf ptr-) (incf ptr- (send _att_kp :serialization-length))
   ;; geometry_msgs/Vector3 _att_ki
     (send _att_ki :deserialize buf ptr-) (incf ptr- (send _att_ki :serialization-length))
   ;; geometry_msgs/Vector3 _omega_kd
     (send _omega_kd :deserialize buf ptr-) (incf ptr- (send _omega_kd :serialization-length))
   ;; geometry_msgs/Vector3 _pos_kp
     (send _pos_kp :deserialize buf ptr-) (incf ptr- (send _pos_kp :serialization-length))
   ;; geometry_msgs/Vector3 _pos_ki
     (send _pos_ki :deserialize buf ptr-) (incf ptr- (send _pos_ki :serialization-length))
   ;; geometry_msgs/Vector3 _vel_kd
     (send _vel_kd :deserialize buf ptr-) (incf ptr- (send _vel_kd :serialization-length))
   ;; float32 _hard_limit_vel
     (setq _hard_limit_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hard_limit_accel
     (setq _hard_limit_accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hard_limit_omega
     (setq _hard_limit_omega (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _hard_limit_alpha
     (setq _hard_limit_alpha (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _speed
     (setq _speed (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::FlightMode :md5sum-) "0bb389101a5f30087bd644e6596d8e8e")
(setf (get ff_msgs::FlightMode :datatype-) "ff_msgs/FlightMode")
(setf (get ff_msgs::FlightMode :definition-)
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
# This message captures all information in a flight mode

Header header                     # Metadata

string name                       # Name of the flight mode

bool control_enabled              # Is control enabled?

# Tolerances (all in SI units)
float32 tolerance_pos_endpoint    # Endpoint position tolerance in m
float32 tolerance_pos             # Position tolerance in m
float32 tolerance_vel             # Velocity tolerance in m/s
float32 tolerance_att             # Attitude tolerance in rads
float32 tolerance_omega           # Angular acceleration tolerance in rad/s
float32 tolerance_time            # Acceptable lag betwee TX and RX of control

# Controller gains
geometry_msgs/Vector3 att_kp      # Positional proportional constant
geometry_msgs/Vector3 att_ki      # Positional integrative constant
geometry_msgs/Vector3 omega_kd    # Attidue derivative constant
geometry_msgs/Vector3 pos_kp      # Positional proportional contant
geometry_msgs/Vector3 pos_ki      # Positional integrative constant
geometry_msgs/Vector3 vel_kd      # Positional derivative constant

# Hard limit on planning
float32 hard_limit_vel            # Position tolerance in m/s
float32 hard_limit_accel          # Position tolerance in m/s^2
float32 hard_limit_omega          # Position tolerance in rads/s
float32 hard_limit_alpha          # Position tolerance in rads/s^2

# Impeller speed
uint8 speed                       # Current speed gain
uint8 SPEED_MIN        = 0        # Min acceptable gain
uint8 SPEED_OFF        = 0        # Blowers off
uint8 SPEED_QUIET      = 1        # Quiet mode
uint8 SPEED_NOMINAL    = 2        # Nomainal mode
uint8 SPEED_AGGRESSIVE = 3        # Aggressive mode
uint8 SPEED_MAX        = 3        # Max acceptable gain

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



(provide :ff_msgs/FlightMode "0bb389101a5f30087bd644e6596d8e8e")


