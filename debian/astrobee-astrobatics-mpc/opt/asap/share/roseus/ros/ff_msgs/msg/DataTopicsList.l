;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DataTopicsList)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DataTopicsList (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DATATOPICSLIST")
  (make-package "FF_MSGS::DATATOPICSLIST"))

(in-package "ROS")
;;//! \htmlinclude DataTopicsList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::DataTopicsList
  :super ros::object
  :slots (_header _topic_names ))

(defmethod ff_msgs::DataTopicsList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:topic_names __topic_names) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _topic_names __topic_names)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:topic_names
   (&optional __topic_names)
   (if __topic_names (setq _topic_names __topic_names)) _topic_names)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _topic_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _topic_names)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _topic_names
     (write-long (length _topic_names) s)
     (dolist (elem _topic_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _topic_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _topic_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _topic_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::DataTopicsList :md5sum-) "59cc5f70481d7b043e1dca93df85302c")
(setf (get ff_msgs::DataTopicsList :datatype-) "ff_msgs/DataTopicsList")
(setf (get ff_msgs::DataTopicsList :definition-)
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

# This message contains the names of all the ros topics in the system.

# Header with timestamp
std_msgs/Header header

string[] topic_names         # An array of all the ros topics in the system

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



(provide :ff_msgs/DataTopicsList "59cc5f70481d7b043e1dca93df85302c")


