;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::DepthCorrespondence)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'DepthCorrespondence (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::DEPTHCORRESPONDENCE")
  (make-package "FF_MSGS::DEPTHCORRESPONDENCE"))

(in-package "ROS")
;;//! \htmlinclude DepthCorrespondence.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ff_msgs::DepthCorrespondence
  :super ros::object
  :slots (_source_image_point _target_image_point _source_3d_point _target_3d_point ))

(defmethod ff_msgs::DepthCorrespondence
  (:init
   (&key
    ((:source_image_point __source_image_point) (instance ff_msgs::ImagePoint :init))
    ((:target_image_point __target_image_point) (instance ff_msgs::ImagePoint :init))
    ((:source_3d_point __source_3d_point) (instance geometry_msgs::Point :init))
    ((:target_3d_point __target_3d_point) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _source_image_point __source_image_point)
   (setq _target_image_point __target_image_point)
   (setq _source_3d_point __source_3d_point)
   (setq _target_3d_point __target_3d_point)
   self)
  (:source_image_point
   (&rest __source_image_point)
   (if (keywordp (car __source_image_point))
       (send* _source_image_point __source_image_point)
     (progn
       (if __source_image_point (setq _source_image_point (car __source_image_point)))
       _source_image_point)))
  (:target_image_point
   (&rest __target_image_point)
   (if (keywordp (car __target_image_point))
       (send* _target_image_point __target_image_point)
     (progn
       (if __target_image_point (setq _target_image_point (car __target_image_point)))
       _target_image_point)))
  (:source_3d_point
   (&rest __source_3d_point)
   (if (keywordp (car __source_3d_point))
       (send* _source_3d_point __source_3d_point)
     (progn
       (if __source_3d_point (setq _source_3d_point (car __source_3d_point)))
       _source_3d_point)))
  (:target_3d_point
   (&rest __target_3d_point)
   (if (keywordp (car __target_3d_point))
       (send* _target_3d_point __target_3d_point)
     (progn
       (if __target_3d_point (setq _target_3d_point (car __target_3d_point)))
       _target_3d_point)))
  (:serialization-length
   ()
   (+
    ;; ff_msgs/ImagePoint _source_image_point
    (send _source_image_point :serialization-length)
    ;; ff_msgs/ImagePoint _target_image_point
    (send _target_image_point :serialization-length)
    ;; geometry_msgs/Point _source_3d_point
    (send _source_3d_point :serialization-length)
    ;; geometry_msgs/Point _target_3d_point
    (send _target_3d_point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ff_msgs/ImagePoint _source_image_point
       (send _source_image_point :serialize s)
     ;; ff_msgs/ImagePoint _target_image_point
       (send _target_image_point :serialize s)
     ;; geometry_msgs/Point _source_3d_point
       (send _source_3d_point :serialize s)
     ;; geometry_msgs/Point _target_3d_point
       (send _target_3d_point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ff_msgs/ImagePoint _source_image_point
     (send _source_image_point :deserialize buf ptr-) (incf ptr- (send _source_image_point :serialization-length))
   ;; ff_msgs/ImagePoint _target_image_point
     (send _target_image_point :deserialize buf ptr-) (incf ptr- (send _target_image_point :serialization-length))
   ;; geometry_msgs/Point _source_3d_point
     (send _source_3d_point :deserialize buf ptr-) (incf ptr- (send _source_3d_point :serialization-length))
   ;; geometry_msgs/Point _target_3d_point
     (send _target_3d_point :deserialize buf ptr-) (incf ptr- (send _target_3d_point :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::DepthCorrespondence :md5sum-) "869ec9e0747cdb7e1fd25af7fca82639")
(setf (get ff_msgs::DepthCorrespondence :datatype-) "ff_msgs/DepthCorrespondence")
(setf (get ff_msgs::DepthCorrespondence :definition-)
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
ImagePoint source_image_point
ImagePoint target_image_point
geometry_msgs/Point source_3d_point
geometry_msgs/Point target_3d_point

================================================================================
MSG: ff_msgs/ImagePoint
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

float32 x
float32 y

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :ff_msgs/DepthCorrespondence "869ec9e0747cdb7e1fd25af7fca82639")


