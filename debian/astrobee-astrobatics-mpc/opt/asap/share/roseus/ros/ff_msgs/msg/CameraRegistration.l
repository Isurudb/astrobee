;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CameraRegistration)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CameraRegistration (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CAMERAREGISTRATION")
  (make-package "FF_MSGS::CAMERAREGISTRATION"))

(in-package "ROS")
;;//! \htmlinclude CameraRegistration.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ff_msgs::CameraRegistration
  :super ros::object
  :slots (_header _camera_id ))

(defmethod ff_msgs::CameraRegistration
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:camera_id __camera_id) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _camera_id (round __camera_id))
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
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _camera_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _camera_id
       (write-long _camera_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _camera_id
     (setq _camera_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::CameraRegistration :md5sum-) "39570521afd2f2ad2728a757c58cee99")
(setf (get ff_msgs::CameraRegistration :datatype-) "ff_msgs/CameraRegistration")
(setf (get ff_msgs::CameraRegistration :definition-)
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
# This message registers a camera pulse when an image is taken.

std_msgs/Header header # header, time stamp is important

uint32 camera_id # an id associated with the image

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



(provide :ff_msgs/CameraRegistration "39570521afd2f2ad2728a757c58cee99")


