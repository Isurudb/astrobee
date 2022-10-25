;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ImagePoint)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ImagePoint (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::IMAGEPOINT")
  (make-package "FF_MSGS::IMAGEPOINT"))

(in-package "ROS")
;;//! \htmlinclude ImagePoint.msg.html


(defclass ff_msgs::ImagePoint
  :super ros::object
  :slots (_x _y ))

(defmethod ff_msgs::ImagePoint
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::ImagePoint :md5sum-) "ff8d7d66dd3e4b731ef14a45d38888b6")
(setf (get ff_msgs::ImagePoint :datatype-) "ff_msgs/ImagePoint")
(setf (get ff_msgs::ImagePoint :definition-)
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

float32 x
float32 y

")



(provide :ff_msgs/ImagePoint "ff8d7d66dd3e4b731ef14a45d38888b6")


