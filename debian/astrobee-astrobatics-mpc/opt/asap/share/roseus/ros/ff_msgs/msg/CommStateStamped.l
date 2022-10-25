;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CommStateStamped)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CommStateStamped (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::COMMSTATESTAMPED")
  (make-package "FF_MSGS::COMMSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude CommStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::CommStateStamped
  :super ros::object
  :slots (_header _wirelessConnected _apName _bssid _rssi _frequency _channel _lanConnected ))

(defmethod ff_msgs::CommStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:wirelessConnected __wirelessConnected) nil)
    ((:apName __apName) "")
    ((:bssid __bssid) "")
    ((:rssi __rssi) 0.0)
    ((:frequency __frequency) 0.0)
    ((:channel __channel) 0)
    ((:lanConnected __lanConnected) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _wirelessConnected __wirelessConnected)
   (setq _apName (string __apName))
   (setq _bssid (string __bssid))
   (setq _rssi (float __rssi))
   (setq _frequency (float __frequency))
   (setq _channel (round __channel))
   (setq _lanConnected __lanConnected)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:wirelessConnected
   (&optional __wirelessConnected)
   (if __wirelessConnected (setq _wirelessConnected __wirelessConnected)) _wirelessConnected)
  (:apName
   (&optional __apName)
   (if __apName (setq _apName __apName)) _apName)
  (:bssid
   (&optional __bssid)
   (if __bssid (setq _bssid __bssid)) _bssid)
  (:rssi
   (&optional __rssi)
   (if __rssi (setq _rssi __rssi)) _rssi)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:channel
   (&optional __channel)
   (if __channel (setq _channel __channel)) _channel)
  (:lanConnected
   (&optional __lanConnected)
   (if __lanConnected (setq _lanConnected __lanConnected)) _lanConnected)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _wirelessConnected
    1
    ;; string _apName
    4 (length _apName)
    ;; string _bssid
    4 (length _bssid)
    ;; float32 _rssi
    4
    ;; float32 _frequency
    4
    ;; uint16 _channel
    2
    ;; bool _lanConnected
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _wirelessConnected
       (if _wirelessConnected (write-byte -1 s) (write-byte 0 s))
     ;; string _apName
       (write-long (length _apName) s) (princ _apName s)
     ;; string _bssid
       (write-long (length _bssid) s) (princ _bssid s)
     ;; float32 _rssi
       (sys::poke _rssi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _frequency
       (sys::poke _frequency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _channel
       (write-word _channel s)
     ;; bool _lanConnected
       (if _lanConnected (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _wirelessConnected
     (setq _wirelessConnected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _apName
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _apName (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _bssid
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bssid (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _rssi
     (setq _rssi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _frequency
     (setq _frequency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _channel
     (setq _channel (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _lanConnected
     (setq _lanConnected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::CommStateStamped :md5sum-) "bfb1cf1d26d8d3be4813192cd797e19f")
(setf (get ff_msgs::CommStateStamped :datatype-) "ff_msgs/CommStateStamped")
(setf (get ff_msgs::CommStateStamped :definition-)
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
# CommState, based off of rapid::ext::astrobee::CommState

std_msgs/Header header

bool wirelessConnected
string apName
string bssid
float32 rssi
float32 frequency
uint16 channel

bool lanConnected

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



(provide :ff_msgs/CommStateStamped "bfb1cf1d26d8d3be4813192cd797e19f")


