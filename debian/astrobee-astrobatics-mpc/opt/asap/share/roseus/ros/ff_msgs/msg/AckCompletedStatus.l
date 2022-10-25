;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::AckCompletedStatus)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'AckCompletedStatus (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ACKCOMPLETEDSTATUS")
  (make-package "FF_MSGS::ACKCOMPLETEDSTATUS"))

(in-package "ROS")
;;//! \htmlinclude AckCompletedStatus.msg.html


(intern "*NOT*" (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(shadow '*NOT* (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(defconstant ff_msgs::AckCompletedStatus::*NOT* 0)
(intern "*OK*" (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(shadow '*OK* (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(defconstant ff_msgs::AckCompletedStatus::*OK* 1)
(intern "*BAD_SYNTAX*" (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(shadow '*BAD_SYNTAX* (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(defconstant ff_msgs::AckCompletedStatus::*BAD_SYNTAX* 2)
(intern "*EXEC_FAILED*" (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(shadow '*EXEC_FAILED* (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(defconstant ff_msgs::AckCompletedStatus::*EXEC_FAILED* 3)
(intern "*CANCELED*" (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(shadow '*CANCELED* (find-package "FF_MSGS::ACKCOMPLETEDSTATUS"))
(defconstant ff_msgs::AckCompletedStatus::*CANCELED* 4)
(defclass ff_msgs::AckCompletedStatus
  :super ros::object
  :slots (_status ))

(defmethod ff_msgs::AckCompletedStatus
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

(setf (get ff_msgs::AckCompletedStatus :md5sum-) "06cb8fd59f3394d8700f8f7e1d9d75a3")
(setf (get ff_msgs::AckCompletedStatus :datatype-) "ff_msgs/AckCompletedStatus")
(setf (get ff_msgs::AckCompletedStatus :definition-)
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
# Completed command status. Based on AckCompletedStatus from RAPID DDS

uint8 NOT = 0           # Command not completed
uint8 OK = 1            # Command completed successfully
uint8 BAD_SYNTAX = 2    # Command not recognized, bad parameters, etc.
uint8 EXEC_FAILED = 3   # Command failed to execute
uint8 CANCELED = 4      # Command was canceled by operator

# Completed command status
uint8 status

")



(provide :ff_msgs/AckCompletedStatus "06cb8fd59f3394d8700f8f7e1d9d75a3")


