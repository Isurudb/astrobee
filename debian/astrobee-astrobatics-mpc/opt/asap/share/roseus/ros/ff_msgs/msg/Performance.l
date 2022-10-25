;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::Performance)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'Performance (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::PERFORMANCE")
  (make-package "FF_MSGS::PERFORMANCE"))

(in-package "ROS")
;;//! \htmlinclude Performance.msg.html


(defclass ff_msgs::Performance
  :super ros::object
  :slots (_stamp _count _last _min _max _mean _stddev _var ))

(defmethod ff_msgs::Performance
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:count __count) 0.0)
    ((:last __last) 0.0)
    ((:min __min) 0.0)
    ((:max __max) 0.0)
    ((:mean __mean) 0.0)
    ((:stddev __stddev) 0.0)
    ((:var __var) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _count (float __count))
   (setq _last (float __last))
   (setq _min (float __min))
   (setq _max (float __max))
   (setq _mean (float __mean))
   (setq _stddev (float __stddev))
   (setq _var (float __var))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:count
   (&optional __count)
   (if __count (setq _count __count)) _count)
  (:last
   (&optional __last)
   (if __last (setq _last __last)) _last)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:max
   (&optional __max)
   (if __max (setq _max __max)) _max)
  (:mean
   (&optional __mean)
   (if __mean (setq _mean __mean)) _mean)
  (:stddev
   (&optional __stddev)
   (if __stddev (setq _stddev __stddev)) _stddev)
  (:var
   (&optional __var)
   (if __var (setq _var __var)) _var)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float32 _count
    4
    ;; float32 _last
    4
    ;; float32 _min
    4
    ;; float32 _max
    4
    ;; float32 _mean
    4
    ;; float32 _stddev
    4
    ;; float32 _var
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _count
       (sys::poke _count (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _last
       (sys::poke _last (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _min
       (sys::poke _min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max
       (sys::poke _max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _mean
       (sys::poke _mean (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _stddev
       (sys::poke _stddev (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _var
       (sys::poke _var (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _count
     (setq _count (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _last
     (setq _last (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _min
     (setq _min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max
     (setq _max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _mean
     (setq _mean (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _stddev
     (setq _stddev (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _var
     (setq _var (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::Performance :md5sum-) "04ada46adc18b617f396f9d156029c85")
(setf (get ff_msgs::Performance :datatype-) "ff_msgs/Performance")
(setf (get ff_msgs::Performance :definition-)
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
# Statistics used to measure performance.

time stamp
float32 count
float32 last
float32 min
float32 max
float32 mean
float32 stddev
float32 var

")



(provide :ff_msgs/Performance "04ada46adc18b617f396f9d156029c85")


