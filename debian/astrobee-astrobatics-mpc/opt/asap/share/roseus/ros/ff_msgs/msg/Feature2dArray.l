;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Feature2dArray)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Feature2dArray (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FEATURE2DARRAY")
  (make-package "FF_MSGS::FEATURE2DARRAY"))

(in-package "ROS")
;;//! \htmlinclude Feature2dArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::Feature2dArray
  :super ros::object
  :slots (_header _camera_id _feature_array ))

(defmethod ff_msgs::Feature2dArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera_id __camera_id) 0)
    ((:feature_array __feature_array) (let (r) (dotimes (i 0) (push (instance ff_msgs::Feature2d :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera_id (round __camera_id))
   (setq _feature_array __feature_array)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:camera_id
   (&optional __camera_id)
   (if __camera_id (setq _camera_id __camera_id)) _camera_id)
  (:feature_array
   (&rest __feature_array)
   (if (keywordp (car __feature_array))
       (send* _feature_array __feature_array)
     (progn
       (if __feature_array (setq _feature_array (car __feature_array)))
       _feature_array)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _camera_id
    4
    ;; ff_msgs/Feature2d[] _feature_array
    (apply #'+ (send-all _feature_array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _camera_id
       (write-long _camera_id s)
     ;; ff_msgs/Feature2d[] _feature_array
     (write-long (length _feature_array) s)
     (dolist (elem _feature_array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _camera_id
     (setq _camera_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; ff_msgs/Feature2d[] _feature_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _feature_array (let (r) (dotimes (i n) (push (instance ff_msgs::Feature2d :init) r)) r))
     (dolist (elem- _feature_array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ff_msgs::Feature2dArray :md5sum-) "579cb05879a7a1292a35750f014c3208")
(setf (get ff_msgs::Feature2dArray :datatype-) "ff_msgs/Feature2dArray")
(setf (get ff_msgs::Feature2dArray :definition-)
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
# An observation of image points and associated ids, for optical flow.

Header header # header with timestamp
uint32 camera_id # image ID, linked to registration pulse
Feature2d[] feature_array # list of observed features

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
MSG: ff_msgs/Feature2d
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
# A single observation of a feature, with an ID and coordinates.
# Used for an optical flow feature.

uint16 id # feature ID
float32 x # feature x coordinate
float32 y # feature y coordinate

")



(provide :ff_msgs/Feature2dArray "579cb05879a7a1292a35750f014c3208")


