;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::AckStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'AckStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ACKSTAMPED")
  (make-package "FF_MSGS::ACKSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude AckStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::AckStamped
  :super ros::object
  :slots (_header _cmd_id _status _completed_status _message ))

(defmethod ff_msgs::AckStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cmd_id __cmd_id) "")
    ((:status __status) (instance ff_msgs::AckStatus :init))
    ((:completed_status __completed_status) (instance ff_msgs::AckCompletedStatus :init))
    ((:message __message) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _cmd_id (string __cmd_id))
   (setq _status __status)
   (setq _completed_status __completed_status)
   (setq _message (string __message))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cmd_id
   (&optional __cmd_id)
   (if __cmd_id (setq _cmd_id __cmd_id)) _cmd_id)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:completed_status
   (&rest __completed_status)
   (if (keywordp (car __completed_status))
       (send* _completed_status __completed_status)
     (progn
       (if __completed_status (setq _completed_status (car __completed_status)))
       _completed_status)))
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _cmd_id
    4 (length _cmd_id)
    ;; ff_msgs/AckStatus _status
    (send _status :serialization-length)
    ;; ff_msgs/AckCompletedStatus _completed_status
    (send _completed_status :serialization-length)
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _cmd_id
       (write-long (length _cmd_id) s) (princ _cmd_id s)
     ;; ff_msgs/AckStatus _status
       (send _status :serialize s)
     ;; ff_msgs/AckCompletedStatus _completed_status
       (send _completed_status :serialize s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _cmd_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; ff_msgs/AckStatus _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; ff_msgs/AckCompletedStatus _completed_status
     (send _completed_status :deserialize buf ptr-) (incf ptr- (send _completed_status :serialization-length))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::AckStamped :md5sum-) "cebddec69fd770df54633be444a9187d")
(setf (get ff_msgs::AckStamped :datatype-) "ff_msgs/AckStamped")
(setf (get ff_msgs::AckStamped :definition-)
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
# Message used to send an acknowledgement for commands received. Based off of
# Ack in RAPID DDS

# Header with timestamp
std_msgs/Header header

# Id of the command being acknowledged
string cmd_id

# Status of the command
ff_msgs/AckStatus status

# Completed status of the command
ff_msgs/AckCompletedStatus completed_status

# If the command fails to execute, message will contain information on why it
# failed.
string message

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



(provide :ff_msgs/AckStamped "cebddec69fd770df54633be444a9187d")


