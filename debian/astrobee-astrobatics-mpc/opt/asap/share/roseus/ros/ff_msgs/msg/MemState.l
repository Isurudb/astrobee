;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::MemState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'MemState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::MEMSTATE")
  (make-package "FF_MSGS::MEMSTATE"))

(in-package "ROS")
;;//! \htmlinclude MemState.msg.html


(defclass ff_msgs::MemState
  :super ros::object
  :slots (_name _virt _virt_peak _ram _ram_peak _ram_perc ))

(defmethod ff_msgs::MemState
  (:init
   (&key
    ((:name __name) "")
    ((:virt __virt) 0)
    ((:virt_peak __virt_peak) 0)
    ((:ram __ram) 0)
    ((:ram_peak __ram_peak) 0)
    ((:ram_perc __ram_perc) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _virt (round __virt))
   (setq _virt_peak (round __virt_peak))
   (setq _ram (round __ram))
   (setq _ram_peak (round __ram_peak))
   (setq _ram_perc (float __ram_perc))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:virt
   (&optional __virt)
   (if __virt (setq _virt __virt)) _virt)
  (:virt_peak
   (&optional __virt_peak)
   (if __virt_peak (setq _virt_peak __virt_peak)) _virt_peak)
  (:ram
   (&optional __ram)
   (if __ram (setq _ram __ram)) _ram)
  (:ram_peak
   (&optional __ram_peak)
   (if __ram_peak (setq _ram_peak __ram_peak)) _ram_peak)
  (:ram_perc
   (&optional __ram_perc)
   (if __ram_perc (setq _ram_perc __ram_perc)) _ram_perc)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; uint32 _virt
    4
    ;; uint32 _virt_peak
    4
    ;; uint32 _ram
    4
    ;; uint32 _ram_peak
    4
    ;; float32 _ram_perc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint32 _virt
       (write-long _virt s)
     ;; uint32 _virt_peak
       (write-long _virt_peak s)
     ;; uint32 _ram
       (write-long _ram s)
     ;; uint32 _ram_peak
       (write-long _ram_peak s)
     ;; float32 _ram_perc
       (sys::poke _ram_perc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _virt
     (setq _virt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _virt_peak
     (setq _virt_peak (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ram
     (setq _ram (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _ram_peak
     (setq _ram_peak (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _ram_perc
     (setq _ram_perc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::MemState :md5sum-) "35fa33fe0824ebd7cf296b7a82e3c26b")
(setf (get ff_msgs::MemState :datatype-) "ff_msgs/MemState")
(setf (get ff_msgs::MemState :definition-)
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
# State of the Memory.

# The memory load of the node, for the fields given in
string name
# Virtual Memory
uint32 virt        # virtual memeory used in Mb
uint32 virt_peak   # peak virtual memory used in Mb

# Physical Memory
uint32 ram        # physical memory used in Mb
uint32 ram_peak   # peak physical memory used in Mb
float32 ram_perc  # percentage of physical memory in %


")



(provide :ff_msgs/MemState "35fa33fe0824ebd7cf296b7a82e3c26b")


