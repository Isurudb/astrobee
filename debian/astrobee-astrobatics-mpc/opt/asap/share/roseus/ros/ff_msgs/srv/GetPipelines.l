;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::GetPipelines)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'GetPipelines (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::GETPIPELINES")
  (make-package "FF_MSGS::GETPIPELINES"))
(unless (find-package "FF_MSGS::GETPIPELINESREQUEST")
  (make-package "FF_MSGS::GETPIPELINESREQUEST"))
(unless (find-package "FF_MSGS::GETPIPELINESRESPONSE")
  (make-package "FF_MSGS::GETPIPELINESRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::GetPipelinesRequest
  :super ros::object
  :slots ())

(defmethod ff_msgs::GetPipelinesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ff_msgs::GetPipelinesResponse
  :super ros::object
  :slots (_pipelines ))

(defmethod ff_msgs::GetPipelinesResponse
  (:init
   (&key
    ((:pipelines __pipelines) (let (r) (dotimes (i 0) (push (instance ff_msgs::LocalizationPipeline :init) r)) r))
    )
   (send-super :init)
   (setq _pipelines __pipelines)
   self)
  (:pipelines
   (&rest __pipelines)
   (if (keywordp (car __pipelines))
       (send* _pipelines __pipelines)
     (progn
       (if __pipelines (setq _pipelines (car __pipelines)))
       _pipelines)))
  (:serialization-length
   ()
   (+
    ;; ff_msgs/LocalizationPipeline[] _pipelines
    (apply #'+ (send-all _pipelines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; ff_msgs/LocalizationPipeline[] _pipelines
     (write-long (length _pipelines) s)
     (dolist (elem _pipelines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; ff_msgs/LocalizationPipeline[] _pipelines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pipelines (let (r) (dotimes (i n) (push (instance ff_msgs::LocalizationPipeline :init) r)) r))
     (dolist (elem- _pipelines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass ff_msgs::GetPipelines
  :super ros::object
  :slots ())

(setf (get ff_msgs::GetPipelines :md5sum-) "4fb31d141d0f152e9301905ffcaa8f48")
(setf (get ff_msgs::GetPipelines :datatype-) "ff_msgs/GetPipelines")
(setf (get ff_msgs::GetPipelines :request) ff_msgs::GetPipelinesRequest)
(setf (get ff_msgs::GetPipelines :response) ff_msgs::GetPipelinesResponse)

(defmethod ff_msgs::GetPipelinesRequest
  (:response () (instance ff_msgs::GetPipelinesResponse :init)))

(setf (get ff_msgs::GetPipelinesRequest :md5sum-) "4fb31d141d0f152e9301905ffcaa8f48")
(setf (get ff_msgs::GetPipelinesRequest :datatype-) "ff_msgs/GetPipelinesRequest")
(setf (get ff_msgs::GetPipelinesRequest :definition-)
      "


















---

ff_msgs/LocalizationPipeline[] pipelines


================================================================================
MSG: ff_msgs/LocalizationPipeline
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
# Information about a pipeline

string id                     # Short id for the pipeline
uint8 mode                    # EKF mode for the pipeline
string name                   # Long name for the pipe
bool requires_filter          # Does this pipeline require the EKF
bool requires_optical_flow    # Does this pipeline require optical flow
")

(setf (get ff_msgs::GetPipelinesResponse :md5sum-) "4fb31d141d0f152e9301905ffcaa8f48")
(setf (get ff_msgs::GetPipelinesResponse :datatype-) "ff_msgs/GetPipelinesResponse")
(setf (get ff_msgs::GetPipelinesResponse :definition-)
      "


















---

ff_msgs/LocalizationPipeline[] pipelines


================================================================================
MSG: ff_msgs/LocalizationPipeline
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
# Information about a pipeline

string id                     # Short id for the pipeline
uint8 mode                    # EKF mode for the pipeline
string name                   # Long name for the pipe
bool requires_filter          # Does this pipeline require the EKF
bool requires_optical_flow    # Does this pipeline require optical flow
")



(provide :ff_msgs/GetPipelines "4fb31d141d0f152e9301905ffcaa8f48")


