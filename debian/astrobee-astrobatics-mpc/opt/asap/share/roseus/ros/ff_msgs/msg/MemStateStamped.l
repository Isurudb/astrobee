;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::MemStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'MemStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::MEMSTATESTAMPED")
  (make-package "FF_MSGS::MEMSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude MemStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::MemStateStamped
  :super ros::object
  :slots (_header _name _ram_total _ram_used _virt_total _virt_used _nodes ))

(defmethod ff_msgs::MemStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:ram_total __ram_total) 0)
    ((:ram_used __ram_used) 0)
    ((:virt_total __virt_total) 0)
    ((:virt_used __virt_used) 0)
    ((:nodes __nodes) (let (r) (dotimes (i 0) (push (instance ff_msgs::MemState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _ram_total (round __ram_total))
   (setq _ram_used (round __ram_used))
   (setq _virt_total (round __virt_total))
   (setq _virt_used (round __virt_used))
   (setq _nodes __nodes)
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
  (:ram_total
   (&optional __ram_total)
   (if __ram_total (setq _ram_total __ram_total)) _ram_total)
  (:ram_used
   (&optional __ram_used)
   (if __ram_used (setq _ram_used __ram_used)) _ram_used)
  (:virt_total
   (&optional __virt_total)
   (if __virt_total (setq _virt_total __virt_total)) _virt_total)
  (:virt_used
   (&optional __virt_used)
   (if __virt_used (setq _virt_used __virt_used)) _virt_used)
  (:nodes
   (&rest __nodes)
   (if (keywordp (car __nodes))
       (send* _nodes __nodes)
     (progn
       (if __nodes (setq _nodes (car __nodes)))
       _nodes)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; uint32 _ram_total
    4
    ;; uint32 _ram_used
    4
    ;; uint32 _virt_total
    4
    ;; uint32 _virt_used
    4
    ;; ff_msgs/MemState[] _nodes
    (apply #'+ (send-all _nodes :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint32 _ram_total
       (write-long _ram_total s)
     ;; uint32 _ram_used
       (write-long _ram_used s)
     ;; uint32 _virt_total
       (write-long _virt_total s)
     ;; uint32 _virt_used
       (write-long _virt_used s)
     ;; ff_msgs/MemState[] _nodes
     (write-long (length _nodes) s)
     (dolist (elem _nodes)
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
   ;; uint32 _ram_total
     (setq _ram_total (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ram_used
     (setq _ram_used (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _virt_total
     (setq _virt_total (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _virt_used
     (setq _virt_used (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ff_msgs/MemState[] _nodes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nodes (let (r) (dotimes (i n) (push (instance ff_msgs::MemState :init) r)) r))
     (dolist (elem- _nodes)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::MemStateStamped :md5sum-) "331992e4d9de11a301b654de12ecbac9")
(setf (get ff_msgs::MemStateStamped :datatype-) "ff_msgs/MemStateStamped")
(setf (get ff_msgs::MemStateStamped :definition-)
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
# Memory state message with timestamp.

# Header with timestamp
std_msgs/Header header

# Machine name (llp, hlp, mlp, etc)
string name

# Physical Memory (RAM)
uint32 ram_total   # total physical memeory in system Mb
uint32 ram_used    # totalphysical memeory used in Mb

# Virtual Memory
uint32 virt_total  # total virtual memeory in system in Mb
uint32 virt_used   # total virtual memeory used in Mb

# Individual nodes
ff_msgs/MemState[] nodes



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
MSG: ff_msgs/MemState
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
# State of the Memory.

# The memory load of the node, for the fields given in
string name
# Virtual Memory
uint32 virt        # virtual memeory used in Mb
uint32 virt_peak   # peak virtual memory used in Mb

# Physical Memory
uint32 ram        # physical memory used in Mb
uint32 ram_peak   # peak physical memory used in Mb
float32 ram_perc  # percentage of physical memory in %


")



(provide :ff_msgs/MemStateStamped "331992e4d9de11a301b654de12ecbac9")


