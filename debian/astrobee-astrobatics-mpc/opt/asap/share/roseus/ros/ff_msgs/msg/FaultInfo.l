;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::FaultInfo)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'FaultInfo (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::FAULTINFO")
  (make-package "FF_MSGS::FAULTINFO"))

(in-package "ROS")
;;//! \htmlinclude FaultInfo.msg.html


(defclass ff_msgs::FaultInfo
  :super ros::object
  :slots (_subsystem _node _id _warning _description ))

(defmethod ff_msgs::FaultInfo
  (:init
   (&key
    ((:subsystem __subsystem) 0)
    ((:node __node) 0)
    ((:id __id) 0)
    ((:warning __warning) nil)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _subsystem (round __subsystem))
   (setq _node (round __node))
   (setq _id (round __id))
   (setq _warning __warning)
   (setq _description (string __description))
   self)
  (:subsystem
   (&optional __subsystem)
   (if __subsystem (setq _subsystem __subsystem)) _subsystem)
  (:node
   (&optional __node)
   (if __node (setq _node __node)) _node)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:warning
   (&optional __warning)
   (if __warning (setq _warning __warning)) _warning)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; uint16 _subsystem
    2
    ;; uint16 _node
    2
    ;; uint32 _id
    4
    ;; bool _warning
    1
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _subsystem
       (write-word _subsystem s)
     ;; uint16 _node
       (write-word _node s)
     ;; uint32 _id
       (write-long _id s)
     ;; bool _warning
       (if _warning (write-byte -1 s) (write-byte 0 s))
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _subsystem
     (setq _subsystem (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _node
     (setq _node (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _warning
     (setq _warning (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ff_msgs::FaultInfo :md5sum-) "1f6014a9106a0f40b77f475f6f9592fa")
(setf (get ff_msgs::FaultInfo :datatype-) "ff_msgs/FaultInfo")
(setf (get ff_msgs::FaultInfo :definition-)
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
# Fault info message is used in the fault config message to contain all the 
# information GDS needs to know about a fault

uint16 subsystem    # index into subsystem names array found in fault config msg

uint16 node         # index into node names array found in fault config msg

uint32 id           # id corresponding to the fault

bool warning        # whether the fault is a warning or not

string description  # A short description of why the fault occurred

")



(provide :ff_msgs/FaultInfo "1f6014a9106a0f40b77f475f6f9592fa")


