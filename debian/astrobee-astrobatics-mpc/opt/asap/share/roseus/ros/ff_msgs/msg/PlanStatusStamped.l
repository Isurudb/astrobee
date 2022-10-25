;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::PlanStatusStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'PlanStatusStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::PLANSTATUSSTAMPED")
  (make-package "FF_MSGS::PLANSTATUSSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude PlanStatusStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::PlanStatusStamped
  :super ros::object
  :slots (_header _name _point _command _status _history ))

(defmethod ff_msgs::PlanStatusStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:point __point) 0)
    ((:command __command) 0)
    ((:status __status) (instance ff_msgs::AckStatus :init))
    ((:history __history) (let (r) (dotimes (i 0) (push (instance ff_msgs::Status :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _point (round __point))
   (setq _command (round __command))
   (setq _status __status)
   (setq _history __history)
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
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:history
   (&rest __history)
   (if (keywordp (car __history))
       (send* _history __history)
     (progn
       (if __history (setq _history (car __history)))
       _history)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; int32 _point
    4
    ;; int32 _command
    4
    ;; ff_msgs/AckStatus _status
    (send _status :serialization-length)
    ;; ff_msgs/Status[] _history
    (apply #'+ (send-all _history :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _point
       (write-long _point s)
     ;; int32 _command
       (write-long _command s)
     ;; ff_msgs/AckStatus _status
       (send _status :serialize s)
     ;; ff_msgs/Status[] _history
     (write-long (length _history) s)
     (dolist (elem _history)
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
   ;; int32 _point
     (setq _point (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _command
     (setq _command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ff_msgs/AckStatus _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; ff_msgs/Status[] _history
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _history (let (r) (dotimes (i n) (push (instance ff_msgs::Status :init) r)) r))
     (dolist (elem- _history)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::PlanStatusStamped :md5sum-) "6b462b22b23dabb22643884c3aecd09b")
(setf (get ff_msgs::PlanStatusStamped :datatype-) "ff_msgs/PlanStatusStamped")
(setf (get ff_msgs::PlanStatusStamped :definition-)
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
# Plan status message. Based off of PlanStatus from DDS. Note that while in
# ROS we use an unbounded array, we are actually limited to ~64 previous
# status messages in the DDS type.

# Header with timestamp
std_msgs/Header header

# Name of plan
string name

int32 point                 # Current station or segment
int32 command               # Current subcommand within station/segment or -1
ff_msgs/AckStatus status    # Status of the currently executing plan element

ff_msgs/Status[] history    # Completion status of the last 64 plan elements

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
MSG: ff_msgs/AckStatus
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
# Command status. Based off AckStatus in RAPID DDS

uint8 QUEUED = 0      # Command is in a queue and waiting to be executed
uint8 EXECUTING = 1   # Command is being executed
uint8 REQUEUED = 2    # Command is paused and waiting to be restarted 
uint8 COMPLETED = 3   # Command is finished

uint8 status          # Command status

================================================================================
MSG: ff_msgs/Status
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
# Sub-type for a command's status in a PlanStatus' history.

int32 point                         # Station or segment
int32 command                       # Subcommand within station/segment or -1
int32 duration                      # How long it took
ff_msgs/AckCompletedStatus status   # The completion status

================================================================================
MSG: ff_msgs/AckCompletedStatus
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
# Completed command status. Based on AckCompletedStatus from RAPID DDS

uint8 NOT = 0           # Command not completed
uint8 OK = 1            # Command completed successfully
uint8 BAD_SYNTAX = 2    # Command not recognized, bad parameters, etc.
uint8 EXEC_FAILED = 3   # Command failed to execute
uint8 CANCELED = 4      # Command was canceled by operator

# Completed command status
uint8 status

")



(provide :ff_msgs/PlanStatusStamped "6b462b22b23dabb22643884c3aecd09b")


