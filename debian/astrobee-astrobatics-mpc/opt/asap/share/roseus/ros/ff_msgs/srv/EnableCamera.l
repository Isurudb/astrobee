;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::EnableCamera)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'EnableCamera (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::ENABLECAMERA")
  (make-package "FF_MSGS::ENABLECAMERA"))
(unless (find-package "FF_MSGS::ENABLECAMERAREQUEST")
  (make-package "FF_MSGS::ENABLECAMERAREQUEST"))
(unless (find-package "FF_MSGS::ENABLECAMERARESPONSE")
  (make-package "FF_MSGS::ENABLECAMERARESPONSE"))

(in-package "ROS")





(intern "*BOTH*" (find-package "FF_MSGS::ENABLECAMERAREQUEST"))
(shadow '*BOTH* (find-package "FF_MSGS::ENABLECAMERAREQUEST"))
(defconstant ff_msgs::EnableCameraRequest::*BOTH* 0)
(intern "*RECORDING*" (find-package "FF_MSGS::ENABLECAMERAREQUEST"))
(shadow '*RECORDING* (find-package "FF_MSGS::ENABLECAMERAREQUEST"))
(defconstant ff_msgs::EnableCameraRequest::*RECORDING* 1)
(intern "*STREAMING*" (find-package "FF_MSGS::ENABLECAMERAREQUEST"))
(shadow '*STREAMING* (find-package "FF_MSGS::ENABLECAMERAREQUEST"))
(defconstant ff_msgs::EnableCameraRequest::*STREAMING* 2)
(defclass ff_msgs::EnableCameraRequest
  :super ros::object
  :slots (_mode _enable ))

(defmethod ff_msgs::EnableCameraRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _enable __enable)
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::EnableCameraResponse
  :super ros::object
  :slots ())

(defmethod ff_msgs::EnableCameraResponse
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

(defclass ff_msgs::EnableCamera
  :super ros::object
  :slots ())

(setf (get ff_msgs::EnableCamera :md5sum-) "4180836a5b8bc96980a6bee8edb99cea")
(setf (get ff_msgs::EnableCamera :datatype-) "ff_msgs/EnableCamera")
(setf (get ff_msgs::EnableCamera :request) ff_msgs::EnableCameraRequest)
(setf (get ff_msgs::EnableCamera :response) ff_msgs::EnableCameraResponse)

(defmethod ff_msgs::EnableCameraRequest
  (:response () (instance ff_msgs::EnableCameraResponse :init)))

(setf (get ff_msgs::EnableCameraRequest :md5sum-) "4180836a5b8bc96980a6bee8edb99cea")
(setf (get ff_msgs::EnableCameraRequest :datatype-) "ff_msgs/EnableCameraRequest")
(setf (get ff_msgs::EnableCameraRequest :definition-)
      "


















uint8 BOTH      = 0
uint8 RECORDING = 1
uint8 STREAMING = 2

uint8 mode
bool enable
---

")

(setf (get ff_msgs::EnableCameraResponse :md5sum-) "4180836a5b8bc96980a6bee8edb99cea")
(setf (get ff_msgs::EnableCameraResponse :datatype-) "ff_msgs/EnableCameraResponse")
(setf (get ff_msgs::EnableCameraResponse :definition-)
      "


















uint8 BOTH      = 0
uint8 RECORDING = 1
uint8 STREAMING = 2

uint8 mode
bool enable
---

")



(provide :ff_msgs/EnableCamera "4180836a5b8bc96980a6bee8edb99cea")


