;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::LocalizationPipeline)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'LocalizationPipeline (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::LOCALIZATIONPIPELINE")
  (make-package "FF_MSGS::LOCALIZATIONPIPELINE"))

(in-package "ROS")
;;//! \htmlinclude LocalizationPipeline.msg.html


(defclass ff_msgs::LocalizationPipeline
  :super ros::object
  :slots (_id _mode _name _requires_filter _requires_optical_flow ))

(defmethod ff_msgs::LocalizationPipeline
  (:init
   (&key
    ((:id __id) "")
    ((:mode __mode) 0)
    ((:name __name) "")
    ((:requires_filter __requires_filter) nil)
    ((:requires_optical_flow __requires_optical_flow) nil)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _mode (round __mode))
   (setq _name (string __name))
   (setq _requires_filter __requires_filter)
   (setq _requires_optical_flow __requires_optical_flow)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:requires_filter
   (&optional __requires_filter)
   (if __requires_filter (setq _requires_filter __requires_filter)) _requires_filter)
  (:requires_optical_flow
   (&optional __requires_optical_flow)
   (if __requires_optical_flow (setq _requires_optical_flow __requires_optical_flow)) _requires_optical_flow)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; uint8 _mode
    1
    ;; string _name
    4 (length _name)
    ;; bool _requires_filter
    1
    ;; bool _requires_optical_flow
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; uint8 _mode
       (write-byte _mode s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _requires_filter
       (if _requires_filter (write-byte -1 s) (write-byte 0 s))
     ;; bool _requires_optical_flow
       (if _requires_optical_flow (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _requires_filter
     (setq _requires_filter (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _requires_optical_flow
     (setq _requires_optical_flow (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ff_msgs::LocalizationPipeline :md5sum-) "19b78d22f6e82148b4ff1aec54ea7e06")
(setf (get ff_msgs::LocalizationPipeline :datatype-) "ff_msgs/LocalizationPipeline")
(setf (get ff_msgs::LocalizationPipeline :definition-)
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
# Information about a pipeline

string id                     # Short id for the pipeline
uint8 mode                    # EKF mode for the pipeline
string name                   # Long name for the pipe
bool requires_filter          # Does this pipeline require the EKF
bool requires_optical_flow    # Does this pipeline require optical flow

")



(provide :ff_msgs/LocalizationPipeline "19b78d22f6e82148b4ff1aec54ea7e06")


