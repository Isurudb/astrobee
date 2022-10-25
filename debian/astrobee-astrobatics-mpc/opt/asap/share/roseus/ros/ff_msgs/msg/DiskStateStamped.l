;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DiskStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DiskStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DISKSTATESTAMPED")
  (make-package "FF_MSGS::DISKSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude DiskStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::DiskStateStamped
  :super ros::object
  :slots (_header _processor_name _disks ))

(defmethod ff_msgs::DiskStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:processor_name __processor_name) "")
    ((:disks __disks) (let (r) (dotimes (i 0) (push (instance ff_msgs::DiskState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _processor_name (string __processor_name))
   (setq _disks __disks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:processor_name
   (&optional __processor_name)
   (if __processor_name (setq _processor_name __processor_name)) _processor_name)
  (:disks
   (&rest __disks)
   (if (keywordp (car __disks))
       (send* _disks __disks)
     (progn
       (if __disks (setq _disks (car __disks)))
       _disks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _processor_name
    4 (length _processor_name)
    ;; ff_msgs/DiskState[] _disks
    (apply #'+ (send-all _disks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _processor_name
       (write-long (length _processor_name) s) (princ _processor_name s)
     ;; ff_msgs/DiskState[] _disks
     (write-long (length _disks) s)
     (dolist (elem _disks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _processor_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _processor_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/DiskState[] _disks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _disks (let (r) (dotimes (i n) (push (instance ff_msgs::DiskState :init) r)) r))
     (dolist (elem- _disks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::DiskStateStamped :md5sum-) "af262ed457fa1b453f85c47b9f5c607b")
(setf (get ff_msgs::DiskStateStamped :datatype-) "ff_msgs/DiskStateStamped")
(setf (get ff_msgs::DiskStateStamped :definition-)
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
# This message describes the state of a filesystem within astrobee
# Based off of DiskState from rapid::ext::astrobee

# Header with timestamp
std_msgs/Header header

string processor_name       # Processor name, either llp, mlp, or hlp

# Information on the mounted filesystem on the processor
ff_msgs/DiskState[] disks

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
MSG: ff_msgs/DiskState
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
# This message describes the state of a filesystem within astrobee
# Based off of DiskState from rapid::ext::astrobee

string path       # The pathname of the file within the mounted filesystem
uint64 capacity   # The size of the filesystem
uint64 used       # The amount of the filesystem being used

")



(provide :ff_msgs/DiskStateStamped "af262ed457fa1b453f85c47b9f5c607b")


