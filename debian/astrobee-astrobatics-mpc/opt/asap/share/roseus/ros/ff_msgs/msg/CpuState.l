;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CpuState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CpuState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CPUSTATE")
  (make-package "FF_MSGS::CPUSTATE"))

(in-package "ROS")
;;//! \htmlinclude CpuState.msg.html


(defclass ff_msgs::CpuState
  :super ros::object
  :slots (_enabled _loads _frequency _max_frequency ))

(defmethod ff_msgs::CpuState
  (:init
   (&key
    ((:enabled __enabled) nil)
    ((:loads __loads) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:frequency __frequency) 0)
    ((:max_frequency __max_frequency) 0)
    )
   (send-super :init)
   (setq _enabled __enabled)
   (setq _loads __loads)
   (setq _frequency (round __frequency))
   (setq _max_frequency (round __max_frequency))
   self)
  (:enabled
   (&optional __enabled)
   (if __enabled (setq _enabled __enabled)) _enabled)
  (:loads
   (&optional __loads)
   (if __loads (setq _loads __loads)) _loads)
  (:frequency
   (&optional __frequency)
   (if __frequency (setq _frequency __frequency)) _frequency)
  (:max_frequency
   (&optional __max_frequency)
   (if __max_frequency (setq _max_frequency __max_frequency)) _max_frequency)
  (:serialization-length
   ()
   (+
    ;; bool _enabled
    1
    ;; float32[] _loads
    (* 4    (length _loads)) 4
    ;; uint32 _frequency
    4
    ;; uint32 _max_frequency
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enabled
       (if _enabled (write-byte -1 s) (write-byte 0 s))
     ;; float32[] _loads
     (write-long (length _loads) s)
     (dotimes (i (length _loads))
       (sys::poke (elt _loads i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32 _frequency
       (write-long _frequency s)
     ;; uint32 _max_frequency
       (write-long _max_frequency s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enabled
     (setq _enabled (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32[] _loads
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _loads (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _loads i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint32 _frequency
     (setq _frequency (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _max_frequency
     (setq _max_frequency (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::CpuState :md5sum-) "3ff6c0a8b78ea1e9087461c1e42b6ca2")
(setf (get ff_msgs::CpuState :datatype-) "ff_msgs/CpuState")
(setf (get ff_msgs::CpuState :definition-)
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
# State of a CPU.

# Processor is on (enabled) or not
bool enabled

# The load (in percentages) of the cpu, for the fields given in
# CpuStateStamped
float32[] loads 

# Current operating frequency in Hz
uint32 frequency

# Max frequency (may be less than theoretical limit of the processor)
uint32 max_frequency

")



(provide :ff_msgs/CpuState "3ff6c0a8b78ea1e9087461c1e42b6ca2")


