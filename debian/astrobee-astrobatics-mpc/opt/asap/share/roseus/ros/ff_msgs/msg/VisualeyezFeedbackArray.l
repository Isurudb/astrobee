;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::VisualeyezFeedbackArray)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'VisualeyezFeedbackArray (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::VISUALEYEZFEEDBACKARRAY")
  (make-package "FF_MSGS::VISUALEYEZFEEDBACKARRAY"))

(in-package "ROS")
;;//! \htmlinclude VisualeyezFeedbackArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::VisualeyezFeedbackArray
  :super ros::object
  :slots (_header _feedback ))

(defmethod ff_msgs::VisualeyezFeedbackArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:feedback __feedback) (let (r) (dotimes (i 0) (push (instance ff_msgs::VisualeyezFeedback :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _feedback __feedback)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:feedback
   (&rest __feedback)
   (if (keywordp (car __feedback))
       (send* _feedback __feedback)
     (progn
       (if __feedback (setq _feedback (car __feedback)))
       _feedback)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; ff_msgs/VisualeyezFeedback[] _feedback
    (apply #'+ (send-all _feedback :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; ff_msgs/VisualeyezFeedback[] _feedback
     (write-long (length _feedback) s)
     (dolist (elem _feedback)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; ff_msgs/VisualeyezFeedback[] _feedback
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _feedback (let (r) (dotimes (i n) (push (instance ff_msgs::VisualeyezFeedback :init) r)) r))
     (dolist (elem- _feedback)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::VisualeyezFeedbackArray :md5sum-) "50480fdca33dc36bf859801c972b691e")
(setf (get ff_msgs::VisualeyezFeedbackArray :datatype-) "ff_msgs/VisualeyezFeedbackArray")
(setf (get ff_msgs::VisualeyezFeedbackArray :definition-)
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
# Visualeyez feedback array with timestamp.

Header header                           # Header with timestamp
ff_msgs/VisualeyezFeedback[] feedback   # List of all measurements
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
MSG: ff_msgs/VisualeyezFeedback
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
# Visualeyez feedback.

uint8 tcmid                         # Transmission control module ID
uint8 ledid                         # Light emitting diode ID
uint32 count                        # Number of valid measurements
")



(provide :ff_msgs/VisualeyezFeedbackArray "50480fdca33dc36bf859801c972b691e")


