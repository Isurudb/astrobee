;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GuestScienceConfig)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GuestScienceConfig (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GUESTSCIENCECONFIG")
  (make-package "FF_MSGS::GUESTSCIENCECONFIG"))

(in-package "ROS")
;;//! \htmlinclude GuestScienceConfig.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::GuestScienceConfig
  :super ros::object
  :slots (_header _serial _apks ))

(defmethod ff_msgs::GuestScienceConfig
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:serial __serial) 0)
    ((:apks __apks) (let (r) (dotimes (i 0) (push (instance ff_msgs::GuestScienceApk :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _serial (round __serial))
   (setq _apks __apks)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:serial
   (&optional __serial)
   (if __serial (setq _serial __serial)) _serial)
  (:apks
   (&rest __apks)
   (if (keywordp (car __apks))
       (send* _apks __apks)
     (progn
       (if __apks (setq _apks (car __apks)))
       _apks)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _serial
    8
    ;; ff_msgs/GuestScienceApk[] _apks
    (apply #'+ (send-all _apks :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _serial
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _serial (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _serial) (= (length (_serial . bv)) 2)) ;; bignum
              (write-long (ash (elt (_serial . bv) 0) 0) s)
              (write-long (ash (elt (_serial . bv) 1) -1) s))
             ((and (class _serial) (= (length (_serial . bv)) 1)) ;; big1
              (write-long (elt (_serial . bv) 0) s)
              (write-long (if (>= _serial 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _serial s)(write-long (if (>= _serial 0) 0 #xffffffff) s)))
     ;; ff_msgs/GuestScienceApk[] _apks
     (write-long (length _apks) s)
     (dolist (elem _apks)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _serial
#+(or :alpha :irix6 :x86_64)
      (setf _serial (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _serial (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; ff_msgs/GuestScienceApk[] _apks
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _apks (let (r) (dotimes (i n) (push (instance ff_msgs::GuestScienceApk :init) r)) r))
     (dolist (elem- _apks)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::GuestScienceConfig :md5sum-) "c6f632ed2411de4159494a3e7754794e")
(setf (get ff_msgs::GuestScienceConfig :datatype-) "ff_msgs/GuestScienceConfig")
(setf (get ff_msgs::GuestScienceConfig :definition-)
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
# Message used to store a list of guest science APKs and information relevant to
# the APK

# Header with timestamp
std_msgs/Header header

# Used for guest science config and state message synchronization on the ground
int64 serial

ff_msgs/GuestScienceApk[] apks

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
MSG: ff_msgs/GuestScienceApk
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



(provide :ff_msgs/GuestScienceConfig "c6f632ed2411de4159494a3e7754794e")


