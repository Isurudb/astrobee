;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::AgentStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'AgentStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::AGENTSTATESTAMPED")
  (make-package "FF_MSGS::AGENTSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude AgentStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::AgentStateStamped
  :super ros::object
  :slots (_header _operating_state _plan_execution_state _guest_science_state _mobility_state _proximity _profile_name _flight_mode _target_linear_velocity _target_linear_accel _target_angular_velocity _target_angular_accel _collision_distance _holonomic_enabled _check_obstacles _check_zones _auto_return_enabled _immediate_enabled _planner _replanning_enabled _world _boot_time ))

(defmethod ff_msgs::AgentStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:operating_state __operating_state) (instance ff_msgs::OpState :init))
    ((:plan_execution_state __plan_execution_state) (instance ff_msgs::ExecState :init))
    ((:guest_science_state __guest_science_state) (instance ff_msgs::ExecState :init))
    ((:mobility_state __mobility_state) (instance ff_msgs::MobilityState :init))
    ((:proximity __proximity) 0.0)
    ((:profile_name __profile_name) "")
    ((:flight_mode __flight_mode) "")
    ((:target_linear_velocity __target_linear_velocity) 0.0)
    ((:target_linear_accel __target_linear_accel) 0.0)
    ((:target_angular_velocity __target_angular_velocity) 0.0)
    ((:target_angular_accel __target_angular_accel) 0.0)
    ((:collision_distance __collision_distance) 0.0)
    ((:holonomic_enabled __holonomic_enabled) nil)
    ((:check_obstacles __check_obstacles) nil)
    ((:check_zones __check_zones) nil)
    ((:auto_return_enabled __auto_return_enabled) nil)
    ((:immediate_enabled __immediate_enabled) nil)
    ((:planner __planner) "")
    ((:replanning_enabled __replanning_enabled) nil)
    ((:world __world) "")
    ((:boot_time __boot_time) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _operating_state __operating_state)
   (setq _plan_execution_state __plan_execution_state)
   (setq _guest_science_state __guest_science_state)
   (setq _mobility_state __mobility_state)
   (setq _proximity (float __proximity))
   (setq _profile_name (string __profile_name))
   (setq _flight_mode (string __flight_mode))
   (setq _target_linear_velocity (float __target_linear_velocity))
   (setq _target_linear_accel (float __target_linear_accel))
   (setq _target_angular_velocity (float __target_angular_velocity))
   (setq _target_angular_accel (float __target_angular_accel))
   (setq _collision_distance (float __collision_distance))
   (setq _holonomic_enabled __holonomic_enabled)
   (setq _check_obstacles __check_obstacles)
   (setq _check_zones __check_zones)
   (setq _auto_return_enabled __auto_return_enabled)
   (setq _immediate_enabled __immediate_enabled)
   (setq _planner (string __planner))
   (setq _replanning_enabled __replanning_enabled)
   (setq _world (string __world))
   (setq _boot_time (round __boot_time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:operating_state
   (&rest __operating_state)
   (if (keywordp (car __operating_state))
       (send* _operating_state __operating_state)
     (progn
       (if __operating_state (setq _operating_state (car __operating_state)))
       _operating_state)))
  (:plan_execution_state
   (&rest __plan_execution_state)
   (if (keywordp (car __plan_execution_state))
       (send* _plan_execution_state __plan_execution_state)
     (progn
       (if __plan_execution_state (setq _plan_execution_state (car __plan_execution_state)))
       _plan_execution_state)))
  (:guest_science_state
   (&rest __guest_science_state)
   (if (keywordp (car __guest_science_state))
       (send* _guest_science_state __guest_science_state)
     (progn
       (if __guest_science_state (setq _guest_science_state (car __guest_science_state)))
       _guest_science_state)))
  (:mobility_state
   (&rest __mobility_state)
   (if (keywordp (car __mobility_state))
       (send* _mobility_state __mobility_state)
     (progn
       (if __mobility_state (setq _mobility_state (car __mobility_state)))
       _mobility_state)))
  (:proximity
   (&optional __proximity)
   (if __proximity (setq _proximity __proximity)) _proximity)
  (:profile_name
   (&optional __profile_name)
   (if __profile_name (setq _profile_name __profile_name)) _profile_name)
  (:flight_mode
   (&optional __flight_mode)
   (if __flight_mode (setq _flight_mode __flight_mode)) _flight_mode)
  (:target_linear_velocity
   (&optional __target_linear_velocity)
   (if __target_linear_velocity (setq _target_linear_velocity __target_linear_velocity)) _target_linear_velocity)
  (:target_linear_accel
   (&optional __target_linear_accel)
   (if __target_linear_accel (setq _target_linear_accel __target_linear_accel)) _target_linear_accel)
  (:target_angular_velocity
   (&optional __target_angular_velocity)
   (if __target_angular_velocity (setq _target_angular_velocity __target_angular_velocity)) _target_angular_velocity)
  (:target_angular_accel
   (&optional __target_angular_accel)
   (if __target_angular_accel (setq _target_angular_accel __target_angular_accel)) _target_angular_accel)
  (:collision_distance
   (&optional __collision_distance)
   (if __collision_distance (setq _collision_distance __collision_distance)) _collision_distance)
  (:holonomic_enabled
   (&optional __holonomic_enabled)
   (if __holonomic_enabled (setq _holonomic_enabled __holonomic_enabled)) _holonomic_enabled)
  (:check_obstacles
   (&optional __check_obstacles)
   (if __check_obstacles (setq _check_obstacles __check_obstacles)) _check_obstacles)
  (:check_zones
   (&optional __check_zones)
   (if __check_zones (setq _check_zones __check_zones)) _check_zones)
  (:auto_return_enabled
   (&optional __auto_return_enabled)
   (if __auto_return_enabled (setq _auto_return_enabled __auto_return_enabled)) _auto_return_enabled)
  (:immediate_enabled
   (&optional __immediate_enabled)
   (if __immediate_enabled (setq _immediate_enabled __immediate_enabled)) _immediate_enabled)
  (:planner
   (&optional __planner)
   (if __planner (setq _planner __planner)) _planner)
  (:replanning_enabled
   (&optional __replanning_enabled)
   (if __replanning_enabled (setq _replanning_enabled __replanning_enabled)) _replanning_enabled)
  (:world
   (&optional __world)
   (if __world (setq _world __world)) _world)
  (:boot_time
   (&optional __boot_time)
   (if __boot_time (setq _boot_time __boot_time)) _boot_time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/OpState _operating_state
    (send _operating_state :serialization-length)
    ;; ff_msgs/ExecState _plan_execution_state
    (send _plan_execution_state :serialization-length)
    ;; ff_msgs/ExecState _guest_science_state
    (send _guest_science_state :serialization-length)
    ;; ff_msgs/MobilityState _mobility_state
    (send _mobility_state :serialization-length)
    ;; float32 _proximity
    4
    ;; string _profile_name
    4 (length _profile_name)
    ;; string _flight_mode
    4 (length _flight_mode)
    ;; float32 _target_linear_velocity
    4
    ;; float32 _target_linear_accel
    4
    ;; float32 _target_angular_velocity
    4
    ;; float32 _target_angular_accel
    4
    ;; float32 _collision_distance
    4
    ;; bool _holonomic_enabled
    1
    ;; bool _check_obstacles
    1
    ;; bool _check_zones
    1
    ;; bool _auto_return_enabled
    1
    ;; bool _immediate_enabled
    1
    ;; string _planner
    4 (length _planner)
    ;; bool _replanning_enabled
    1
    ;; string _world
    4 (length _world)
    ;; uint32 _boot_time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/OpState _operating_state
       (send _operating_state :serialize s)
     ;; ff_msgs/ExecState _plan_execution_state
       (send _plan_execution_state :serialize s)
     ;; ff_msgs/ExecState _guest_science_state
       (send _guest_science_state :serialize s)
     ;; ff_msgs/MobilityState _mobility_state
       (send _mobility_state :serialize s)
     ;; float32 _proximity
       (sys::poke _proximity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _profile_name
       (write-long (length _profile_name) s) (princ _profile_name s)
     ;; string _flight_mode
       (write-long (length _flight_mode) s) (princ _flight_mode s)
     ;; float32 _target_linear_velocity
       (sys::poke _target_linear_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_linear_accel
       (sys::poke _target_linear_accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_angular_velocity
       (sys::poke _target_angular_velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_angular_accel
       (sys::poke _target_angular_accel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _collision_distance
       (sys::poke _collision_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _holonomic_enabled
       (if _holonomic_enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _check_obstacles
       (if _check_obstacles (write-byte -1 s) (write-byte 0 s))
     ;; bool _check_zones
       (if _check_zones (write-byte -1 s) (write-byte 0 s))
     ;; bool _auto_return_enabled
       (if _auto_return_enabled (write-byte -1 s) (write-byte 0 s))
     ;; bool _immediate_enabled
       (if _immediate_enabled (write-byte -1 s) (write-byte 0 s))
     ;; string _planner
       (write-long (length _planner) s) (princ _planner s)
     ;; bool _replanning_enabled
       (if _replanning_enabled (write-byte -1 s) (write-byte 0 s))
     ;; string _world
       (write-long (length _world) s) (princ _world s)
     ;; uint32 _boot_time
       (write-long _boot_time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/OpState _operating_state
     (send _operating_state :deserialize buf ptr-) (incf ptr- (send _operating_state :serialization-length))
   ;; ff_msgs/ExecState _plan_execution_state
     (send _plan_execution_state :deserialize buf ptr-) (incf ptr- (send _plan_execution_state :serialization-length))
   ;; ff_msgs/ExecState _guest_science_state
     (send _guest_science_state :deserialize buf ptr-) (incf ptr- (send _guest_science_state :serialization-length))
   ;; ff_msgs/MobilityState _mobility_state
     (send _mobility_state :deserialize buf ptr-) (incf ptr- (send _mobility_state :serialization-length))
   ;; float32 _proximity
     (setq _proximity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _profile_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _profile_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _flight_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flight_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _target_linear_velocity
     (setq _target_linear_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_linear_accel
     (setq _target_linear_accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_angular_velocity
     (setq _target_angular_velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_angular_accel
     (setq _target_angular_accel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _collision_distance
     (setq _collision_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _holonomic_enabled
     (setq _holonomic_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _check_obstacles
     (setq _check_obstacles (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _check_zones
     (setq _check_zones (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _auto_return_enabled
     (setq _auto_return_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _immediate_enabled
     (setq _immediate_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _planner
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _planner (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _replanning_enabled
     (setq _replanning_enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _world
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _world (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _boot_time
     (setq _boot_time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::AgentStateStamped :md5sum-) "156487b23e377e3a1dc7ef079f0e327d")
(setf (get ff_msgs::AgentStateStamped :datatype-) "ff_msgs/AgentStateStamped")
(setf (get ff_msgs::AgentStateStamped :definition-)
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
# State of the Astrobee, based off of rapid::ext::astrobee::AgentState

# Header with timestamp
std_msgs/Header header

# Operating state of the Astrobee
ff_msgs/OpState operating_state

# Plan execution state. State is idle when there is no plan to be executed. Once
# a plan is uploaded, the state change to paused. Upon a run plan command, the
# state will change to executing. If a stop or pause plan command is received or
# a fault occurs, the state will be set to pause. Once the plan is completed,
# the state will go back to idle
ff_msgs/ExecState plan_execution_state

# Guest science state. If a primary guest science apk is started, the state
# will go from idle to executing. Once the primarty apk is finished, the state
# will go back to idle
ff_msgs/ExecState guest_science_state

# Mobility state of the Astrobee
ff_msgs/MobilityState mobility_state

# Proximity to the dock when docking and undocking. Proximity to a handrail when
# perching or unperching. 0 the rest of the time.
float32 proximity

# Name of profile configuration, i.e. Nominal, IgnoreObstacles, Faceforward,
# Quiet, etc. Profiles specify stuff like target velocity and acceleration,
# collision distance, etc.
string profile_name

#Defines GN&C gains, hard limits, tolerances, etc.
string flight_mode

# Maximum linear velocity to target while translating
float32 target_linear_velocity
# Maximum linear acceleration to target while translating
float32 target_linear_accel
# Maximum angular velocity to target while rotating
float32 target_angular_velocity
# Maximum angular acceleration to target while rotating
float32 target_angular_accel
# Minimum distance margin to maintain away from obstacles
float32 collision_distance

# Specifies whether the Astrobee is allowed to fly blind i.e. not faceforward
bool holonomic_enabled

# Specifies whether the Astrobee is checking for obstacles
bool check_obstacles

# Specifies whether the Astrobee is checking the keepin and keepout zones
bool check_zones

# Specifies whether the Astrobee is allowed to auto return. Please note,
# Astrobee will only use this flags when deciding if it should auto return. If
# the astrobee gets a command to auto return from the operator or guest science,
# it will auto return without checking this flag
bool auto_return_enabled

# Specifies whether the choreographer should execute a segment immediately or
# based on the time stamp in the segement
bool immediate_enabled

# Specifies the current planner being used
string planner

# Specifies whether re-planning is allowed
bool replanning_enabled

# Specifies the current world being used
string world

# Number of seconds since Unix Epoch
uint32 boot_time

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
MSG: ff_msgs/OpState
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
# Operating States, based off of the enumeration constants
# in rapid::ext::astrobee::AgentState.
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 READY            = 0  # Freeflyer is ready to accept commands
uint8 PLAN_EXECUTION   = 1  # Freeflyer is executing a plan
uint8 TELEOPERATION    = 2  # Freeflyer is executing a teleop command
uint8 AUTO_RETURN      = 3  # Freeflyer is autonomously returning to the dock
# The freeflyer is either executing a fault response or there is a fault
# occurring in the system that prevents the freeflyer from moving
uint8 FAULT            = 4

# Operating state
uint8 state

================================================================================
MSG: ff_msgs/ExecState
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
# Execution States, based off of the enumeration constants in
# rapid::ext::astrobee::AgentState
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 IDLE      = 0   # Process is idle
uint8 EXECUTING = 1   # Process is executing
uint8 PAUSED    = 2   # Process is paused
uint8 ERROR     = 3   # Process encountered an error

# Execution state
uint8 state

================================================================================
MSG: ff_msgs/MobilityState
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
# Mobility states, based off the enumeration constants in
# rapid::ext::astrobee::AgentState
#
# *MUST* be kept in sync with the DDS IDL file in astrobee_common

uint8 DRIFTING        = 0   # Astrobee's propulsion is off
uint8 STOPPING        = 1   # Astrobee is either stopping or stopped
uint8 FLYING          = 2   # Astrobee is flying
uint8 DOCKING         = 3   # Astrobee is either docking or undocking
uint8 PERCHING        = 4   # Astrobee is either perching or unperching

# Mobility state
uint8 state

# Specifies the progress of the action. For docking, this value can be N to -N
# where N through 1 specifies the progress of a docking action, 0 is docked, and
# -1 through -N specifies the process of an undocking action. For stopping, this
# value is either 1 or 0 where 1 means the robot is coming to a stop and 0 means
# the robot is stopped. For perching, this value can be N to -N where N through
# 1 specifies the progress of a perching action, 0 is perched, and -1 through
# -N specifies the process of an unperching action.
int32 sub_state

")



(provide :ff_msgs/AgentStateStamped "156487b23e377e3a1dc7ef079f0e327d")


