;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::VisualeyezData)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'VisualeyezData (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::VISUALEYEZDATA")
  (make-package "FF_MSGS::VISUALEYEZDATA"))

(in-package "ROS")
;;//! \htmlinclude VisualeyezData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass ff_msgs::VisualeyezData
  :super ros::object
  :slots (_tcmid _ledid _position ))

(defmethod ff_msgs::VisualeyezData
  (:init
   (&key
    ((:tcmid __tcmid) 0)
    ((:ledid __ledid) 0)
    ((:position __position) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _tcmid (round __tcmid))
   (setq _ledid (round __ledid))
   (setq _position __position)
   self)
  (:tcmid
   (&optional __tcmid)
   (if __tcmid (setq _tcmid __tcmid)) _tcmid)
  (:ledid
   (&optional __ledid)
   (if __ledid (setq _ledid __ledid)) _ledid)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:serialization-length
   ()
   (+
    ;; uint8 _tcmid
    1
    ;; uint8 _ledid
    1
    ;; geometry_msgs/Vector3 _position
    (send _position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _tcmid
       (write-byte _tcmid s)
     ;; uint8 _ledid
       (write-byte _ledid s)
     ;; geometry_msgs/Vector3 _position
       (send _position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _tcmid
     (setq _tcmid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _ledid
     (setq _ledid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;;
   self)
  )

(setf (get ff_msgs::VisualeyezData :md5sum-) "0a4f041324891dc34a11ad8b15af9d60")
(setf (get ff_msgs::VisualeyezData :datatype-) "ff_msgs/VisualeyezData")
(setf (get ff_msgs::VisualeyezData :definition-)
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
# Raw Visualeyez data.

uint8 tcmid                         # Transmission control module ID
uint8 ledid                         # Light emitting diode ID
geometry_msgs/Vector3 position      # Coordinate 
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :ff_msgs/VisualeyezData "0a4f041324891dc34a11ad8b15af9d60")


