;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SaveSettings)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SaveSettings (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SAVESETTINGS")
  (make-package "FF_MSGS::SAVESETTINGS"))

(in-package "ROS")
;;//! \htmlinclude SaveSettings.msg.html


(intern "*IMMEDIATE*" (find-package "FF_MSGS::SAVESETTINGS"))
(shadow '*IMMEDIATE* (find-package "FF_MSGS::SAVESETTINGS"))
(defconstant ff_msgs::SaveSettings::*IMMEDIATE* 0)
(intern "*DELAYED*" (find-package "FF_MSGS::SAVESETTINGS"))
(shadow '*DELAYED* (find-package "FF_MSGS::SAVESETTINGS"))
(defconstant ff_msgs::SaveSettings::*DELAYED* 1)
(defclass ff_msgs::SaveSettings
  :super ros::object
  :slots (_topic_name _downlinkOption _frequency ))

(defmethod ff_msgs::SaveSettings
  (:init
   (&key
    ((:topic_name __topic_name) "")
    ((:downlinkOption __downlinkOption) 0)
    ((:frequency __frequency) 0.0)
    )
   (send-super :init)
   (setq _topic_name (string __topic_name))
   (setq _downlinkOption (round __downlinkOption))
   (setq _frequency (float __frequency))
   self)
  (:topic_name
   (&optional __topic_name)
   (if __topic_name (setq _topic_name __topic_name)) _topic_name)
  (:downlinkOption
   (&optional __downlinkOption)
   (if __downlinkOption (setq _downlinkOption __downlinkOption)) _downlinkOption)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:serialization-length
   ()
   (+
    ;; string _topic_name
    4 (length _topic_name)
    ;; uint8 _downlinkOption
    1
    ;; float32 _frequency
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _topic_name
       (write-long (length _topic_name) s) (princ _topic_name s)
     ;; uint8 _downlinkOption
       (write-byte _downlinkOption s)
     ;; float32 _frequency
       (sys::poke _frequency (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _topic_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _topic_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _downlinkOption
     (setq _downlinkOption (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _frequency
     (setq _frequency (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::SaveSettings :md5sum-) "87300656673b0987cb5b546a70fa697f")
(setf (get ff_msgs::SaveSettings :datatype-) "ff_msgs/SaveSettings")
(setf (get ff_msgs::SaveSettings :definition-)
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



(provide :ff_msgs/SaveSettings "87300656673b0987cb5b546a70fa697f")


