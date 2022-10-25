;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::VisualeyezFeedback)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'VisualeyezFeedback (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::VISUALEYEZFEEDBACK")
  (make-package "FF_MSGS::VISUALEYEZFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude VisualeyezFeedback.msg.html


(defclass ff_msgs::VisualeyezFeedback
  :super ros::object
  :slots (_tcmid _ledid _count ))

(defmethod ff_msgs::VisualeyezFeedback
  (:init
   (&key
    ((:tcmid __tcmid) 0)
    ((:ledid __ledid) 0)
    ((:count __count) 0)
    )
   (send-super :init)
   (setq _tcmid (round __tcmid))
   (setq _ledid (round __ledid))
   (setq _count (round __count))
   self)
  (:tcmid
   (&optional __tcmid)
   (if __tcmid (setq _tcmid __tcmid)) _tcmid)
  (:ledid
   (&optional __ledid)
   (if __ledid (setq _ledid __ledid)) _ledid)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:serialization-length
   ()
   (+
    ;; uint8 _tcmid
    1
    ;; uint8 _ledid
    1
    ;; uint32 _count
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _tcmid
       (write-byte _tcmid s)
     ;; uint8 _ledid
       (write-byte _ledid s)
     ;; uint32 _count
       (write-long _count s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _tcmid
     (setq _tcmid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _ledid
     (setq _ledid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _count
     (setq _count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::VisualeyezFeedback :md5sum-) "2ce06a8bb76a1a67ecd18953ba9bbf84")
(setf (get ff_msgs::VisualeyezFeedback :datatype-) "ff_msgs/VisualeyezFeedback")
(setf (get ff_msgs::VisualeyezFeedback :definition-)
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
# Visualeyez feedback.

uint8 tcmid                         # Transmission control module ID
uint8 ledid                         # Light emitting diode ID
uint32 count                        # Number of valid measurements
")



(provide :ff_msgs/VisualeyezFeedback "2ce06a8bb76a1a67ecd18953ba9bbf84")


