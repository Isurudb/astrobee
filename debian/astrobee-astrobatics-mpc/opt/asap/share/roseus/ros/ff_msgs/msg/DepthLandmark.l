;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DepthLandmark)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DepthLandmark (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DEPTHLANDMARK")
  (make-package "FF_MSGS::DEPTHLANDMARK"))

(in-package "ROS")
;;//! \htmlinclude DepthLandmark.msg.html


(defclass ff_msgs::DepthLandmark
  :super ros::object
  :slots (_u _v _w ))

(defmethod ff_msgs::DepthLandmark
  (:init
   (&key
    ((:u __u) 0.0)
    ((:v __v) 0.0)
    ((:w __w) 0.0)
    )
   (send-super :init)
   (setq _u (float __u))
   (setq _v (float __v))
   (setq _w (float __w))
   self)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:serialization-length
   ()
   (+
    ;; float32 _u
    4
    ;; float32 _v
    4
    ;; float32 _w
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _u
       (sys::poke _u (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v
       (sys::poke _v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _w
       (sys::poke _w (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _u
     (setq _u (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v
     (setq _v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _w
     (setq _w (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::DepthLandmark :md5sum-) "3e053b7c204c5284962522efbf66bb32")
(setf (get ff_msgs::DepthLandmark :datatype-) "ff_msgs/DepthLandmark")
(setf (get ff_msgs::DepthLandmark :definition-)
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
# A landmark seen from a depth landmark

float32 u     # First coordinate in the image plane
float32 v     # Second coordinate in the image plane
float32 w     # Depth

")



(provide :ff_msgs/DepthLandmark "3e053b7c204c5284962522efbf66bb32")


