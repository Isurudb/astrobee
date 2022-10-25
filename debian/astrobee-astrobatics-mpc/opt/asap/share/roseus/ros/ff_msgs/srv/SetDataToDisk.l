;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetDataToDisk)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetDataToDisk (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETDATATODISK")
  (make-package "FF_MSGS::SETDATATODISK"))
(unless (find-package "FF_MSGS::SETDATATODISKREQUEST")
  (make-package "FF_MSGS::SETDATATODISKREQUEST"))
(unless (find-package "FF_MSGS::SETDATATODISKRESPONSE")
  (make-package "FF_MSGS::SETDATATODISKRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::SetDataToDiskRequest
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::SetDataToDiskRequest
  (:init
   (&key
    ((:state __state) (instance ff_msgs::DataToDiskState :init))
    )
   (send-super :init)
   (setq _state __state)
   self)
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:serialization-length
   ()
   (+
    ;; ff_msgs/DataToDiskState _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ff_msgs/DataToDiskState _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ff_msgs/DataToDiskState _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(defclass ff_msgs::SetDataToDiskResponse
  :super ros::object
  :slots (_success _status ))

(defmethod ff_msgs::SetDataToDiskResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status (string __status))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ff_msgs::SetDataToDisk
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetDataToDisk :md5sum-) "b4cc86540abfc221f4a113bbc42f4b5c")
(setf (get ff_msgs::SetDataToDisk :datatype-) "ff_msgs/SetDataToDisk")
(setf (get ff_msgs::SetDataToDisk :request) ff_msgs::SetDataToDiskRequest)
(setf (get ff_msgs::SetDataToDisk :response) ff_msgs::SetDataToDiskResponse)

(defmethod ff_msgs::SetDataToDiskRequest
  (:response () (instance ff_msgs::SetDataToDiskResponse :init)))

(setf (get ff_msgs::SetDataToDiskRequest :md5sum-) "b4cc86540abfc221f4a113bbc42f4b5c")
(setf (get ff_msgs::SetDataToDiskRequest :datatype-) "ff_msgs/SetDataToDiskRequest")
(setf (get ff_msgs::SetDataToDiskRequest :definition-)
      "
















ff_msgs/DataToDiskState state

================================================================================
MSG: ff_msgs/DataToDiskState
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
---
bool success
string status

")

(setf (get ff_msgs::SetDataToDiskResponse :md5sum-) "b4cc86540abfc221f4a113bbc42f4b5c")
(setf (get ff_msgs::SetDataToDiskResponse :datatype-) "ff_msgs/SetDataToDiskResponse")
(setf (get ff_msgs::SetDataToDiskResponse :definition-)
      "
















ff_msgs/DataToDiskState state

================================================================================
MSG: ff_msgs/DataToDiskState
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
---
bool success
string status

")



(provide :ff_msgs/SetDataToDisk "b4cc86540abfc221f4a113bbc42f4b5c")


