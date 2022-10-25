;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GuestScienceData)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GuestScienceData (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GUESTSCIENCEDATA")
  (make-package "FF_MSGS::GUESTSCIENCEDATA"))

(in-package "ROS")
;;//! \htmlinclude GuestScienceData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STRING*" (find-package "FF_MSGS::GUESTSCIENCEDATA"))
(shadow '*STRING* (find-package "FF_MSGS::GUESTSCIENCEDATA"))
(defconstant ff_msgs::GuestScienceData::*STRING* 0)
(intern "*JSON*" (find-package "FF_MSGS::GUESTSCIENCEDATA"))
(shadow '*JSON* (find-package "FF_MSGS::GUESTSCIENCEDATA"))
(defconstant ff_msgs::GuestScienceData::*JSON* 1)
(intern "*BINARY*" (find-package "FF_MSGS::GUESTSCIENCEDATA"))
(shadow '*BINARY* (find-package "FF_MSGS::GUESTSCIENCEDATA"))
(defconstant ff_msgs::GuestScienceData::*BINARY* 2)
(defclass ff_msgs::GuestScienceData
  :super ros::object
  :slots (_header _apk_name _data_type _topic _data ))

(defmethod ff_msgs::GuestScienceData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:apk_name __apk_name) "")
    ((:data_type __data_type) 0)
    ((:topic __topic) "")
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _apk_name (string __apk_name))
   (setq _data_type (round __data_type))
   (setq _topic (string __topic))
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:apk_name
   (&optional __apk_name)
   (if __apk_name (setq _apk_name __apk_name)) _apk_name)
  (:data_type
   (&optional __data_type)
   (if __data_type (setq _data_type __data_type)) _data_type)
  (:topic
   (&optional __topic)
   (if __topic (setq _topic __topic)) _topic)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _apk_name
    4 (length _apk_name)
    ;; uint8 _data_type
    1
    ;; string _topic
    4 (length _topic)
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _apk_name
       (write-long (length _apk_name) s) (princ _apk_name s)
     ;; uint8 _data_type
       (write-byte _data_type s)
     ;; string _topic
       (write-long (length _topic) s) (princ _topic s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _apk_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _apk_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _data_type
     (setq _data_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _topic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::GuestScienceData :md5sum-) "edd2c791ea93ffa7cb0ec80c5bb00972")
(setf (get ff_msgs::GuestScienceData :datatype-) "ff_msgs/GuestScienceData")
(setf (get ff_msgs::GuestScienceData :definition-)
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
# Message used to send guest science data to the ground

# Header with timestamp
std_msgs/Header header

# Full name of apk
string apk_name

# Types of data stored in the data array
uint8 STRING  = 0
uint8 JSON    = 1
uint8 BINARY  = 2

# Type of data being sent, see above
uint8 data_type

# String to classify the kind of data
string topic

# Data from the apk, rapid type is an octet sequence where an octet is an 8-bit
# quantity
uint8[] data

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

")



(provide :ff_msgs/GuestScienceData "edd2c791ea93ffa7cb0ec80c5bb00972")


