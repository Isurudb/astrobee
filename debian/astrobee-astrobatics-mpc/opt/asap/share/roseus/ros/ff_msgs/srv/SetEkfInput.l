;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetEkfInput)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetEkfInput (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETEKFINPUT")
  (make-package "FF_MSGS::SETEKFINPUT"))
(unless (find-package "FF_MSGS::SETEKFINPUTREQUEST")
  (make-package "FF_MSGS::SETEKFINPUTREQUEST"))
(unless (find-package "FF_MSGS::SETEKFINPUTRESPONSE")
  (make-package "FF_MSGS::SETEKFINPUTRESPONSE"))

(in-package "ROS")





(intern "*MODE_MAP_LANDMARKS*" (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(shadow '*MODE_MAP_LANDMARKS* (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(defconstant ff_msgs::SetEkfInputRequest::*MODE_MAP_LANDMARKS* 0)
(intern "*MODE_AR_TAGS*" (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(shadow '*MODE_AR_TAGS* (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(defconstant ff_msgs::SetEkfInputRequest::*MODE_AR_TAGS* 1)
(intern "*MODE_HANDRAIL*" (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(shadow '*MODE_HANDRAIL* (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(defconstant ff_msgs::SetEkfInputRequest::*MODE_HANDRAIL* 2)
(intern "*MODE_NONE*" (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(shadow '*MODE_NONE* (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(defconstant ff_msgs::SetEkfInputRequest::*MODE_NONE* 3)
(intern "*MODE_PERCH*" (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(shadow '*MODE_PERCH* (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(defconstant ff_msgs::SetEkfInputRequest::*MODE_PERCH* 4)
(intern "*MODE_TRUTH*" (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(shadow '*MODE_TRUTH* (find-package "FF_MSGS::SETEKFINPUTREQUEST"))
(defconstant ff_msgs::SetEkfInputRequest::*MODE_TRUTH* 5)
(defclass ff_msgs::SetEkfInputRequest
  :super ros::object
  :slots (_mode ))

(defmethod ff_msgs::SetEkfInputRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::SetEkfInputResponse
  :super ros::object
  :slots ())

(defmethod ff_msgs::SetEkfInputResponse
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

(defclass ff_msgs::SetEkfInput
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetEkfInput :md5sum-) "f21f65ebbf246d59f8a1b8f5fe25668b")
(setf (get ff_msgs::SetEkfInput :datatype-) "ff_msgs/SetEkfInput")
(setf (get ff_msgs::SetEkfInput :request) ff_msgs::SetEkfInputRequest)
(setf (get ff_msgs::SetEkfInput :response) ff_msgs::SetEkfInputResponse)

(defmethod ff_msgs::SetEkfInputRequest
  (:response () (instance ff_msgs::SetEkfInputResponse :init)))

(setf (get ff_msgs::SetEkfInputRequest :md5sum-) "f21f65ebbf246d59f8a1b8f5fe25668b")
(setf (get ff_msgs::SetEkfInputRequest :datatype-) "ff_msgs/SetEkfInputRequest")
(setf (get ff_msgs::SetEkfInputRequest :definition-)
      "


















uint8 MODE_MAP_LANDMARKS = 0
uint8 MODE_AR_TAGS       = 1
uint8 MODE_HANDRAIL      = 2
uint8 MODE_NONE          = 3
uint8 MODE_PERCH         = 4
uint8 MODE_TRUTH         = 5

uint8 mode
---


")

(setf (get ff_msgs::SetEkfInputResponse :md5sum-) "f21f65ebbf246d59f8a1b8f5fe25668b")
(setf (get ff_msgs::SetEkfInputResponse :datatype-) "ff_msgs/SetEkfInputResponse")
(setf (get ff_msgs::SetEkfInputResponse :definition-)
      "


















uint8 MODE_MAP_LANDMARKS = 0
uint8 MODE_AR_TAGS       = 1
uint8 MODE_HANDRAIL      = 2
uint8 MODE_NONE          = 3
uint8 MODE_PERCH         = 4
uint8 MODE_TRUTH         = 5

uint8 mode
---


")



(provide :ff_msgs/SetEkfInput "f21f65ebbf246d59f8a1b8f5fe25668b")


