;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::LocalizationGraph)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'LocalizationGraph (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::LOCALIZATIONGRAPH")
  (make-package "FF_MSGS::LOCALIZATIONGRAPH"))

(in-package "ROS")
;;//! \htmlinclude LocalizationGraph.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::LocalizationGraph
  :super ros::object
  :slots (_header _child_frame_id _serialized_graph ))

(defmethod ff_msgs::LocalizationGraph
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:child_frame_id __child_frame_id) "")
    ((:serialized_graph __serialized_graph) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _child_frame_id (string __child_frame_id))
   (setq _serialized_graph (string __serialized_graph))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:child_frame_id
   (&optional __child_frame_id)
   (if __child_frame_id (setq _child_frame_id __child_frame_id)) _child_frame_id)
  (:serialized_graph
   (&optional __serialized_graph)
   (if __serialized_graph (setq _serialized_graph __serialized_graph)) _serialized_graph)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _child_frame_id
    4 (length _child_frame_id)
    ;; string _serialized_graph
    4 (length _serialized_graph)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _child_frame_id
       (write-long (length _child_frame_id) s) (princ _child_frame_id s)
     ;; string _serialized_graph
       (write-long (length _serialized_graph) s) (princ _serialized_graph s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _child_frame_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _child_frame_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _serialized_graph
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _serialized_graph (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::LocalizationGraph :md5sum-) "8f879946485aa13660736928ff10f870")
(setf (get ff_msgs::LocalizationGraph :datatype-) "ff_msgs/LocalizationGraph")
(setf (get ff_msgs::LocalizationGraph :definition-)
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
std_msgs/Header header # header with timestamp
string child_frame_id # frame ID

string serialized_graph 

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



(provide :ff_msgs/LocalizationGraph "8f879946485aa13660736928ff10f870")


