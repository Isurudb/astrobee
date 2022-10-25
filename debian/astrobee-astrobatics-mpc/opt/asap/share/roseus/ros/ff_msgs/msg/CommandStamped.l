;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CommandStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CommandStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::COMMANDSTAMPED")
  (make-package "FF_MSGS::COMMANDSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CommandStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::CommandStamped
  :super ros::object
  :slots (_header _cmd_name _cmd_id _cmd_src _cmd_origin _subsys_name _args ))

(defmethod ff_msgs::CommandStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cmd_name __cmd_name) "")
    ((:cmd_id __cmd_id) "")
    ((:cmd_src __cmd_src) "")
    ((:cmd_origin __cmd_origin) "")
    ((:subsys_name __subsys_name) "")
    ((:args __args) (let (r) (dotimes (i 0) (push (instance ff_msgs::CommandArg :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _cmd_name (string __cmd_name))
   (setq _cmd_id (string __cmd_id))
   (setq _cmd_src (string __cmd_src))
   (setq _cmd_origin (string __cmd_origin))
   (setq _subsys_name (string __subsys_name))
   (setq _args __args)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cmd_name
   (&optional __cmd_name)
   (if __cmd_name (setq _cmd_name __cmd_name)) _cmd_name)
  (:cmd_id
   (&optional __cmd_id)
   (if __cmd_id (setq _cmd_id __cmd_id)) _cmd_id)
  (:cmd_src
   (&optional __cmd_src)
   (if __cmd_src (setq _cmd_src __cmd_src)) _cmd_src)
  (:cmd_origin
   (&optional __cmd_origin)
   (if __cmd_origin (setq _cmd_origin __cmd_origin)) _cmd_origin)
  (:subsys_name
   (&optional __subsys_name)
   (if __subsys_name (setq _subsys_name __subsys_name)) _subsys_name)
  (:args
   (&rest __args)
   (if (keywordp (car __args))
       (send* _args __args)
     (progn
       (if __args (setq _args (car __args)))
       _args)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _cmd_name
    4 (length _cmd_name)
    ;; string _cmd_id
    4 (length _cmd_id)
    ;; string _cmd_src
    4 (length _cmd_src)
    ;; string _cmd_origin
    4 (length _cmd_origin)
    ;; string _subsys_name
    4 (length _subsys_name)
    ;; ff_msgs/CommandArg[] _args
    (apply #'+ (send-all _args :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _cmd_name
       (write-long (length _cmd_name) s) (princ _cmd_name s)
     ;; string _cmd_id
       (write-long (length _cmd_id) s) (princ _cmd_id s)
     ;; string _cmd_src
       (write-long (length _cmd_src) s) (princ _cmd_src s)
     ;; string _cmd_origin
       (write-long (length _cmd_origin) s) (princ _cmd_origin s)
     ;; string _subsys_name
       (write-long (length _subsys_name) s) (princ _subsys_name s)
     ;; ff_msgs/CommandArg[] _args
     (write-long (length _args) s)
     (dolist (elem _args)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _cmd_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cmd_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cmd_src
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd_src (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cmd_origin
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd_origin (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _subsys_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _subsys_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/CommandArg[] _args
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _args (let (r) (dotimes (i n) (push (instance ff_msgs::CommandArg :init) r)) r))
     (dolist (elem- _args)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::CommandStamped :md5sum-) "ac01350894b1be9e3a7b4f390a14812d")
(setf (get ff_msgs::CommandStamped :datatype-) "ff_msgs/CommandStamped")
(setf (get ff_msgs::CommandStamped :definition-)
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
# Command Message, loosely based off of the RAPID Command.idl

# Header with timestamp
std_msgs/Header header

# Command name
string cmd_name

# Unique identifier for command = unique counter + participant + timestamp
string cmd_id

# Source of the command, either operators, the system monitor or guest science
string cmd_src

# Origin of the command, ground for operators, astrobee for another astrobee,
# sys_monitor for fault responses, and guest_science for guest science
# commands
string cmd_origin

# Name of subsystem the command is going to (not used but kept to be consistant
# with the command idl)
string subsys_name

# Arguments for the command 
ff_msgs/CommandArg[] args

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
MSG: ff_msgs/CommandArg
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
# An argument to a command sent through RAPID
#
# Note that this is approximating a union in DDS. However, this is an
# inefficient union, and thus each instance will take up at least 89 bytes.
# However, even with the maximum of 16 arguments to a command, we only have
# about 1k extra data. I, tfmorse, am ok with that. Commands are rarely sent.

uint8 DATA_TYPE_BOOL     = 0
uint8 DATA_TYPE_DOUBLE   = 1
uint8 DATA_TYPE_FLOAT    = 2
uint8 DATA_TYPE_INT      = 3
uint8 DATA_TYPE_LONGLONG = 4
uint8 DATA_TYPE_STRING   = 5
uint8 DATA_TYPE_VEC3d    = 6
uint8 DATA_TYPE_MAT33f   = 7

uint8 data_type

bool b
float64 d
float32 f
int32 i
int64 ll
string s
float64[3] vec3d
float32[9] mat33f


")



(provide :ff_msgs/CommandStamped "ac01350894b1be9e3a7b4f390a14812d")


