;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GuestScienceCommand)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GuestScienceCommand (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GUESTSCIENCECOMMAND")
  (make-package "FF_MSGS::GUESTSCIENCECOMMAND"))

(in-package "ROS")
;;//! \htmlinclude GuestScienceCommand.msg.html


(defclass ff_msgs::GuestScienceCommand
  :super ros::object
  :slots (_name _command ))

(defmethod ff_msgs::GuestScienceCommand
  (:init
   (&key
    ((:name __name) "")
    ((:command __command) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _command (string __command))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::GuestScienceCommand :md5sum-) "532fb0240cd4da843e0df16fb3fa75c4")
(setf (get ff_msgs::GuestScienceCommand :datatype-) "ff_msgs/GuestScienceCommand")
(setf (get ff_msgs::GuestScienceCommand :definition-)
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
# Message used to store guest science commands

# Name of command
string name

# Syntax of the command
string command

")



(provide :ff_msgs/GuestScienceCommand "532fb0240cd4da843e0df16fb3fa75c4")


