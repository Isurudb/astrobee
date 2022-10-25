;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DataToDiskState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DataToDiskState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DATATODISKSTATE")
  (make-package "FF_MSGS::DATATODISKSTATE"))

(in-package "ROS")
;;//! \htmlinclude DataToDiskState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::DataToDiskState
  :super ros::object
  :slots (_header _name _recording _topic_save_settings ))

(defmethod ff_msgs::DataToDiskState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:name __name) "")
    ((:recording __recording) nil)
    ((:topic_save_settings __topic_save_settings) (let (r) (dotimes (i 0) (push (instance ff_msgs::SaveSettings :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _name (string __name))
   (setq _recording __recording)
   (setq _topic_save_settings __topic_save_settings)
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
  (:recording
   (&optional __recording)
   (if __recording (setq _recording __recording)) _recording)
  (:topic_save_settings
   (&rest __topic_save_settings)
   (if (keywordp (car __topic_save_settings))
       (send* _topic_save_settings __topic_save_settings)
     (progn
       (if __topic_save_settings (setq _topic_save_settings (car __topic_save_settings)))
       _topic_save_settings)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _name
    4 (length _name)
    ;; bool _recording
    1
    ;; ff_msgs/SaveSettings[] _topic_save_settings
    (apply #'+ (send-all _topic_save_settings :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _recording
       (if _recording (write-byte -1 s) (write-byte 0 s))
     ;; ff_msgs/SaveSettings[] _topic_save_settings
     (write-long (length _topic_save_settings) s)
     (dolist (elem _topic_save_settings)
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
   ;; bool _recording
     (setq _recording (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; ff_msgs/SaveSettings[] _topic_save_settings
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _topic_save_settings (let (r) (dotimes (i n) (push (instance ff_msgs::SaveSettings :init) r)) r))
     (dolist (elem- _topic_save_settings)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::DataToDiskState :md5sum-) "68d7ec16d4c7bc2b6e1a00776a76b4f7")
(setf (get ff_msgs::DataToDiskState :datatype-) "ff_msgs/DataToDiskState")
(setf (get ff_msgs::DataToDiskState :definition-)
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

# Data to disk state message used to let ground operators know which topics
# are currently being recorded.

# Header with timestamp
std_msgs/Header header

# Name of the latest data to disk file uploaded from the ground
string name

# Whether the data bagger is recording a bag or not
bool recording

# An array containing information about the topics being recorded
ff_msgs/SaveSettings[] topic_save_settings

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
MSG: ff_msgs/SaveSettings
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

# The save settings message contains information about the topics currently
# being recorded.

# Name of topic
string topic_name

# Topic saved to disk; upon docking it is downlinked
uint8 IMMEDIATE   = 0

# Topic saved to disk; upon docking it is transferred to ISS server for later
# downlink
uint8 DELAYED     = 1

# Downlink option indicates if and when the data in the rostopic is downlinked
uint8 downlinkOption

# Times per second to save the data (Hz)
float32 frequency

")



(provide :ff_msgs/DataToDiskState "68d7ec16d4c7bc2b6e1a00776a76b4f7")


