;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::AckStatus)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'AckStatus (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ACKSTATUS")
  (make-package "FF_MSGS::ACKSTATUS"))

(in-package "ROS")
;;//! \htmlinclude AckStatus.msg.html


(intern "*QUEUED*" (find-package "FF_MSGS::ACKSTATUS"))
(shadow '*QUEUED* (find-package "FF_MSGS::ACKSTATUS"))
(defconstant ff_msgs::AckStatus::*QUEUED* 0)
(intern "*EXECUTING*" (find-package "FF_MSGS::ACKSTATUS"))
(shadow '*EXECUTING* (find-package "FF_MSGS::ACKSTATUS"))
(defconstant ff_msgs::AckStatus::*EXECUTING* 1)
(intern "*REQUEUED*" (find-package "FF_MSGS::ACKSTATUS"))
(shadow '*REQUEUED* (find-package "FF_MSGS::ACKSTATUS"))
(defconstant ff_msgs::AckStatus::*REQUEUED* 2)
(intern "*COMPLETED*" (find-package "FF_MSGS::ACKSTATUS"))
(shadow '*COMPLETED* (find-package "FF_MSGS::ACKSTATUS"))
(defconstant ff_msgs::AckStatus::*COMPLETED* 3)
(defclass ff_msgs::AckStatus
  :super ros::object
  :slots (_status ))

(defmethod ff_msgs::AckStatus
  (:init
   (&key
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _status (round __status))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; uint8 _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _status
       (write-byte _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::AckStatus :md5sum-) "3a9fd9fcbdad61abda5990f950ce8aee")
(setf (get ff_msgs::AckStatus :datatype-) "ff_msgs/AckStatus")
(setf (get ff_msgs::AckStatus :definition-)
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
# Command status. Based off AckStatus in RAPID DDS

uint8 QUEUED = 0      # Command is in a queue and waiting to be executed
uint8 EXECUTING = 1   # Command is being executed
uint8 REQUEUED = 2    # Command is paused and waiting to be restarted 
uint8 COMPLETED = 3   # Command is finished

uint8 status          # Command status

")



(provide :ff_msgs/AckStatus "3a9fd9fcbdad61abda5990f950ce8aee")


