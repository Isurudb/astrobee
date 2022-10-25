;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GuestScienceApk)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GuestScienceApk (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GUESTSCIENCEAPK")
  (make-package "FF_MSGS::GUESTSCIENCEAPK"))

(in-package "ROS")
;;//! \htmlinclude GuestScienceApk.msg.html


(defclass ff_msgs::GuestScienceApk
  :super ros::object
  :slots (_apk_name _short_name _primary _commands ))

(defmethod ff_msgs::GuestScienceApk
  (:init
   (&key
    ((:apk_name __apk_name) "")
    ((:short_name __short_name) "")
    ((:primary __primary) nil)
    ((:commands __commands) (let (r) (dotimes (i 0) (push (instance ff_msgs::GuestScienceCommand :init) r)) r))
    )
   (send-super :init)
   (setq _apk_name (string __apk_name))
   (setq _short_name (string __short_name))
   (setq _primary __primary)
   (setq _commands __commands)
   self)
  (:apk_name
   (&optional __apk_name)
   (if __apk_name (setq _apk_name __apk_name)) _apk_name)
  (:short_name
   (&optional __short_name)
   (if __short_name (setq _short_name __short_name)) _short_name)
  (:primary
   (&optional __primary)
   (if __primary (setq _primary __primary)) _primary)
  (:commands
   (&rest __commands)
   (if (keywordp (car __commands))
       (send* _commands __commands)
     (progn
       (if __commands (setq _commands (car __commands)))
       _commands)))
  (:serialization-length
   ()
   (+
    ;; string _apk_name
    4 (length _apk_name)
    ;; string _short_name
    4 (length _short_name)
    ;; bool _primary
    1
    ;; ff_msgs/GuestScienceCommand[] _commands
    (apply #'+ (send-all _commands :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _apk_name
       (write-long (length _apk_name) s) (princ _apk_name s)
     ;; string _short_name
       (write-long (length _short_name) s) (princ _short_name s)
     ;; bool _primary
       (if _primary (write-byte -1 s) (write-byte 0 s))
     ;; ff_msgs/GuestScienceCommand[] _commands
     (write-long (length _commands) s)
     (dolist (elem _commands)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _apk_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _apk_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _short_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _short_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _primary
     (setq _primary (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; ff_msgs/GuestScienceCommand[] _commands
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _commands (let (r) (dotimes (i n) (push (instance ff_msgs::GuestScienceCommand :init) r)) r))
     (dolist (elem- _commands)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::GuestScienceApk :md5sum-) "8ed1d23e09733f18dbf96d2f9cd798e5")
(setf (get ff_msgs::GuestScienceApk :datatype-) "ff_msgs/GuestScienceApk")
(setf (get ff_msgs::GuestScienceApk :definition-)
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
# Message used to contain information about a guest science apk

# Full apk name
string apk_name

# Short (human readable) name of the apk
string short_name

# Whether the apk is primary or secondary
bool primary

# List of commands the apk will accept
ff_msgs/GuestScienceCommand[] commands

================================================================================
MSG: ff_msgs/GuestScienceCommand
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
# Message used to store guest science commands

# Name of command
string name

# Syntax of the command
string command

")



(provide :ff_msgs/GuestScienceApk "8ed1d23e09733f18dbf96d2f9cd798e5")


