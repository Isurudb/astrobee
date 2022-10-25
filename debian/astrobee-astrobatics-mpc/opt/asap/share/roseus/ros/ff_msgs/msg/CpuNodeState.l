;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CpuNodeState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CpuNodeState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CPUNODESTATE")
  (make-package "FF_MSGS::CPUNODESTATE"))

(in-package "ROS")
;;//! \htmlinclude CpuNodeState.msg.html


(defclass ff_msgs::CpuNodeState
  :super ros::object
  :slots (_name _load ))

(defmethod ff_msgs::CpuNodeState
  (:init
   (&key
    ((:name __name) "")
    ((:load __load) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _load (float __load))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float32 _load
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _load
       (sys::poke _load (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _load
     (setq _load (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::CpuNodeState :md5sum-) "f4103935420c1e1dc393e01ceae6eaff")
(setf (get ff_msgs::CpuNodeState :datatype-) "ff_msgs/CpuNodeState")
(setf (get ff_msgs::CpuNodeState :definition-)
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
# State of a CPU load for a node.

# Node name
string name

# The load (in percentages) of the cpu
float32 load 

")



(provide :ff_msgs/CpuNodeState "f4103935420c1e1dc393e01ceae6eaff")


