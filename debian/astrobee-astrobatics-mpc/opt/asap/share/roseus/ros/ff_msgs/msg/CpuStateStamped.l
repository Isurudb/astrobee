;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CpuStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CpuStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CPUSTATESTAMPED")
  (make-package "FF_MSGS::CPUSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CpuStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*NICE*" (find-package "FF_MSGS::CPUSTATESTAMPED"))
(shadow '*NICE* (find-package "FF_MSGS::CPUSTATESTAMPED"))
(defconstant ff_msgs::CpuStateStamped::*NICE* "nice")
(intern "*USER*" (find-package "FF_MSGS::CPUSTATESTAMPED"))
(shadow '*USER* (find-package "FF_MSGS::CPUSTATESTAMPED"))
(defconstant ff_msgs::CpuStateStamped::*USER* "user")
(intern "*SYS*" (find-package "FF_MSGS::CPUSTATESTAMPED"))
(shadow '*SYS* (find-package "FF_MSGS::CPUSTATESTAMPED"))
(defconstant ff_msgs::CpuStateStamped::*SYS* "sys")
(intern "*VIRT*" (find-package "FF_MSGS::CPUSTATESTAMPED"))
(shadow '*VIRT* (find-package "FF_MSGS::CPUSTATESTAMPED"))
(defconstant ff_msgs::CpuStateStamped::*VIRT* "virt")
(intern "*TOTAL*" (find-package "FF_MSGS::CPUSTATESTAMPED"))
(shadow '*TOTAL* (find-package "FF_MSGS::CPUSTATESTAMPED"))
(defconstant ff_msgs::CpuStateStamped::*TOTAL* "total")
(defclass ff_msgs::CpuStateStamped
  :super ros::object
  :slots (_header _name _load_fields _avg_loads _temp _cpus _load_nodes ))

(defmethod ff_msgs::CpuStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:load_fields __load_fields) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:avg_loads __avg_loads) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:temp __temp) 0.0)
    ((:cpus __cpus) (let (r) (dotimes (i 0) (push (instance ff_msgs::CpuState :init) r)) r))
    ((:load_nodes __load_nodes) (let (r) (dotimes (i 0) (push (instance ff_msgs::CpuNodeState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _load_fields __load_fields)
   (setq _avg_loads __avg_loads)
   (setq _temp (float __temp))
   (setq _cpus __cpus)
   (setq _load_nodes __load_nodes)
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
  (:load_fields
   (&optional __load_fields)
   (if __load_fields (setq _load_fields __load_fields)) _load_fields)
  (:avg_loads
   (&optional __avg_loads)
   (if __avg_loads (setq _avg_loads __avg_loads)) _avg_loads)
  (:temp
   (&optional __temp)
   (if __temp (setq _temp __temp)) _temp)
  (:cpus
   (&rest __cpus)
   (if (keywordp (car __cpus))
       (send* _cpus __cpus)
     (progn
       (if __cpus (setq _cpus (car __cpus)))
       _cpus)))
  (:load_nodes
   (&rest __load_nodes)
   (if (keywordp (car __load_nodes))
       (send* _load_nodes __load_nodes)
     (progn
       (if __load_nodes (setq _load_nodes (car __load_nodes)))
       _load_nodes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; string[] _load_fields
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _load_fields)) 4
    ;; float32[] _avg_loads
    (* 4    (length _avg_loads)) 4
    ;; float32 _temp
    4
    ;; ff_msgs/CpuState[] _cpus
    (apply #'+ (send-all _cpus :serialization-length)) 4
    ;; ff_msgs/CpuNodeState[] _load_nodes
    (apply #'+ (send-all _load_nodes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string[] _load_fields
     (write-long (length _load_fields) s)
     (dolist (elem _load_fields)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float32[] _avg_loads
     (write-long (length _avg_loads) s)
     (dotimes (i (length _avg_loads))
       (sys::poke (elt _avg_loads i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _temp
       (sys::poke _temp (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; ff_msgs/CpuState[] _cpus
     (write-long (length _cpus) s)
     (dolist (elem _cpus)
       (send elem :serialize s)
       )
     ;; ff_msgs/CpuNodeState[] _load_nodes
     (write-long (length _load_nodes) s)
     (dolist (elem _load_nodes)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _load_fields
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _load_fields (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _load_fields i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float32[] _avg_loads
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _avg_loads (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _avg_loads i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _temp
     (setq _temp (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; ff_msgs/CpuState[] _cpus
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cpus (let (r) (dotimes (i n) (push (instance ff_msgs::CpuState :init) r)) r))
     (dolist (elem- _cpus)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; ff_msgs/CpuNodeState[] _load_nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _load_nodes (let (r) (dotimes (i n) (push (instance ff_msgs::CpuNodeState :init) r)) r))
     (dolist (elem- _load_nodes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::CpuStateStamped :md5sum-) "ecff381c2d5b2d38dda690920e947e86")
(setf (get ff_msgs::CpuStateStamped :datatype-) "ff_msgs/CpuStateStamped")
(setf (get ff_msgs::CpuStateStamped :definition-)
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
# Cpu state message with timestamp.

# Header with timestamp
std_msgs/Header header

# Machine name (llp, hlp, mlp, etc)
string name

# Load constants
string NICE=nice
string USER=user
string SYS=sys
string VIRT=virt
string TOTAL=total

# The available fields within the load values, mostly uses the constants
# defined above.
string[] load_fields

# Average loads for all processors combined
float32[] avg_loads

# Temperature for a cpu (average of all thermal zones)
float32 temp

# Information for each processor
# Size of the array specifies how many processors are on the board, whether
# or not all of them are enabled.
ff_msgs/CpuState[] cpus

# Load usage of individual ROS nodes
ff_msgs/CpuNodeState[] load_nodes
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
MSG: ff_msgs/CpuState
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
# State of a CPU.

# Processor is on (enabled) or not
bool enabled

# The load (in percentages) of the cpu, for the fields given in
# CpuStateStamped
float32[] loads 

# Current operating frequency in Hz
uint32 frequency

# Max frequency (may be less than theoretical limit of the processor)
uint32 max_frequency

================================================================================
MSG: ff_msgs/CpuNodeState
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
# State of a CPU load for a node.

# Node name
string name

# The load (in percentages) of the cpu
float32 load 

")



(provide :ff_msgs/CpuStateStamped "ecff381c2d5b2d38dda690920e947e86")


