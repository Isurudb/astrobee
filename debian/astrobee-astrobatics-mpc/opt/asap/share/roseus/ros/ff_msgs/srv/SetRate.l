;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetRate)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetRate (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETRATE")
  (make-package "FF_MSGS::SETRATE"))
(unless (find-package "FF_MSGS::SETRATEREQUEST")
  (make-package "FF_MSGS::SETRATEREQUEST"))
(unless (find-package "FF_MSGS::SETRATERESPONSE")
  (make-package "FF_MSGS::SETRATERESPONSE"))

(in-package "ROS")





(intern "*COMM_STATUS*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*COMM_STATUS* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*COMM_STATUS* 0)
(intern "*CPU_STATE*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*CPU_STATE* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*CPU_STATE* 1)
(intern "*DISK_STATE*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*DISK_STATE* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*DISK_STATE* 2)
(intern "*EKF_STATE*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*EKF_STATE* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*EKF_STATE* 3)
(intern "*GNC_STATE*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*GNC_STATE* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*GNC_STATE* 4)
(intern "*PMC_CMD_STATE*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*PMC_CMD_STATE* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*PMC_CMD_STATE* 5)
(intern "*POSITION*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*POSITION* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*POSITION* 6)
(intern "*SPARSE_MAPPING_POSE*" (find-package "FF_MSGS::SETRATEREQUEST"))
(shadow '*SPARSE_MAPPING_POSE* (find-package "FF_MSGS::SETRATEREQUEST"))
(defconstant ff_msgs::SetRateRequest::*SPARSE_MAPPING_POSE* 7)
(defclass ff_msgs::SetRateRequest
  :super ros::object
  :slots (_which _rate ))

(defmethod ff_msgs::SetRateRequest
  (:init
   (&key
    ((:which __which) 0)
    ((:rate __rate) 0.0)
    )
   (send-super :init)
   (setq _which (round __which))
   (setq _rate (float __rate))
   self)
  (:which
   (&optional __which)
   (if __which (setq _which __which)) _which)
  (:rate
   (&optional __rate)
   (if __rate (setq _rate __rate)) _rate)
  (:serialization-length
   ()
   (+
    ;; uint8 _which
    1
    ;; float32 _rate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _which
       (write-byte _which s)
     ;; float32 _rate
       (sys::poke _rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _which
     (setq _which (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _rate
     (setq _rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ff_msgs::SetRateResponse
  :super ros::object
  :slots (_success _status ))

(defmethod ff_msgs::SetRateResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _status (string __status))
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ff_msgs::SetRate
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetRate :md5sum-) "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(setf (get ff_msgs::SetRate :datatype-) "ff_msgs/SetRate")
(setf (get ff_msgs::SetRate :request) ff_msgs::SetRateRequest)
(setf (get ff_msgs::SetRate :response) ff_msgs::SetRateResponse)

(defmethod ff_msgs::SetRateRequest
  (:response () (instance ff_msgs::SetRateResponse :init)))

(setf (get ff_msgs::SetRateRequest :md5sum-) "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(setf (get ff_msgs::SetRateRequest :datatype-) "ff_msgs/SetRateRequest")
(setf (get ff_msgs::SetRateRequest :definition-)
      "


















uint8 COMM_STATUS         = 0
uint8 CPU_STATE           = 1
uint8 DISK_STATE          = 2
uint8 EKF_STATE           = 3
uint8 GNC_STATE           = 4
uint8 PMC_CMD_STATE       = 5
uint8 POSITION            = 6
uint8 SPARSE_MAPPING_POSE = 7

uint8 which
float32 rate
---
bool success
string status

")

(setf (get ff_msgs::SetRateResponse :md5sum-) "64286d5b70f59b1a6bc4f3b0e85ea5c5")
(setf (get ff_msgs::SetRateResponse :datatype-) "ff_msgs/SetRateResponse")
(setf (get ff_msgs::SetRateResponse :definition-)
      "


















uint8 COMM_STATUS         = 0
uint8 CPU_STATE           = 1
uint8 DISK_STATE          = 2
uint8 EKF_STATE           = 3
uint8 GNC_STATE           = 4
uint8 PMC_CMD_STATE       = 5
uint8 POSITION            = 6
uint8 SPARSE_MAPPING_POSE = 7

uint8 which
float32 rate
---
bool success
string status

")



(provide :ff_msgs/SetRate "64286d5b70f59b1a6bc4f3b0e85ea5c5")


