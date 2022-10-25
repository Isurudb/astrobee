;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Status)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Status (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::STATUS")
  (make-package "FF_MSGS::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html


(defclass ff_msgs::Status
  :super ros::object
  :slots (_point _command _duration _status ))

(defmethod ff_msgs::Status
  (:init
   (&key
    ((:point __point) 0)
    ((:command __command) 0)
    ((:duration __duration) 0)
    ((:status __status) (instance ff_msgs::AckCompletedStatus :init))
    )
   (send-super :init)
   (setq _point (round __point))
   (setq _command (round __command))
   (setq _duration (round __duration))
   (setq _status __status)
   self)
  (:point
   (&optional __point)
   (if __point (setq _point __point)) _point)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:duration
   (&optional __duration)
   (if __duration (setq _duration __duration)) _duration)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:serialization-length
   ()
   (+
    ;; int32 _point
    4
    ;; int32 _command
    4
    ;; int32 _duration
    4
    ;; ff_msgs/AckCompletedStatus _status
    (send _status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _point
       (write-long _point s)
     ;; int32 _command
       (write-long _command s)
     ;; int32 _duration
       (write-long _duration s)
     ;; ff_msgs/AckCompletedStatus _status
       (send _status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _point
     (setq _point (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _command
     (setq _command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _duration
     (setq _duration (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ff_msgs/AckCompletedStatus _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::Status :md5sum-) "ce2a77030078d6182709c37909b7659f")
(setf (get ff_msgs::Status :datatype-) "ff_msgs/Status")
(setf (get ff_msgs::Status :definition-)
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



(provide :ff_msgs/Status "ce2a77030078d6182709c37909b7659f")


