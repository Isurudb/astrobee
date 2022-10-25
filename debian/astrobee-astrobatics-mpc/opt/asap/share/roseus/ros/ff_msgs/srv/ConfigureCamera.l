;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ConfigureCamera)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ConfigureCamera (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CONFIGURECAMERA")
  (make-package "FF_MSGS::CONFIGURECAMERA"))
(unless (find-package "FF_MSGS::CONFIGURECAMERAREQUEST")
  (make-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(unless (find-package "FF_MSGS::CONFIGURECAMERARESPONSE")
  (make-package "FF_MSGS::CONFIGURECAMERARESPONSE"))

(in-package "ROS")





(intern "*BOTH*" (find-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(shadow '*BOTH* (find-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(defconstant ff_msgs::ConfigureCameraRequest::*BOTH* 0)
(intern "*RECORDING*" (find-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(shadow '*RECORDING* (find-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(defconstant ff_msgs::ConfigureCameraRequest::*RECORDING* 1)
(intern "*STREAMING*" (find-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(shadow '*STREAMING* (find-package "FF_MSGS::CONFIGURECAMERAREQUEST"))
(defconstant ff_msgs::ConfigureCameraRequest::*STREAMING* 2)
(defclass ff_msgs::ConfigureCameraRequest
  :super ros::object
  :slots (_mode _rate _width _height _bitrate ))

(defmethod ff_msgs::ConfigureCameraRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:rate __rate) 0.0)
    ((:width __width) 0)
    ((:height __height) 0)
    ((:bitrate __bitrate) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _rate (float __rate))
   (setq _width (round __width))
   (setq _height (round __height))
   (setq _bitrate (float __bitrate))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:rate
   (&optional __rate)
   (if __rate (setq _rate __rate)) _rate)
  (:width
   (&optional __width)
   (if __width (setq _width __width)) _width)
  (:height
   (&optional __height)
   (if __height (setq _height __height)) _height)
  (:bitrate
   (&optional __bitrate)
   (if __bitrate (setq _bitrate __bitrate)) _bitrate)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; float32 _rate
    4
    ;; uint32 _width
    4
    ;; uint32 _height
    4
    ;; float32 _bitrate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _rate
       (sys::poke _rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _width
       (write-long _width s)
     ;; uint32 _height
       (write-long _height s)
     ;; float32 _bitrate
       (sys::poke _bitrate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _rate
     (setq _rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _width
     (setq _width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _height
     (setq _height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _bitrate
     (setq _bitrate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ff_msgs::ConfigureCameraResponse
  :super ros::object
  :slots ())

(defmethod ff_msgs::ConfigureCameraResponse
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

(defclass ff_msgs::ConfigureCamera
  :super ros::object
  :slots ())

(setf (get ff_msgs::ConfigureCamera :md5sum-) "263cde84e0c4384e57b9ce048385281d")
(setf (get ff_msgs::ConfigureCamera :datatype-) "ff_msgs/ConfigureCamera")
(setf (get ff_msgs::ConfigureCamera :request) ff_msgs::ConfigureCameraRequest)
(setf (get ff_msgs::ConfigureCamera :response) ff_msgs::ConfigureCameraResponse)

(defmethod ff_msgs::ConfigureCameraRequest
  (:response () (instance ff_msgs::ConfigureCameraResponse :init)))

(setf (get ff_msgs::ConfigureCameraRequest :md5sum-) "263cde84e0c4384e57b9ce048385281d")
(setf (get ff_msgs::ConfigureCameraRequest :datatype-) "ff_msgs/ConfigureCameraRequest")
(setf (get ff_msgs::ConfigureCameraRequest :definition-)
      "


















uint8 BOTH      = 0
uint8 RECORDING = 1
uint8 STREAMING = 2

uint8 mode
float32 rate
uint32 width
uint32 height
float32 bitrate
---

")

(setf (get ff_msgs::ConfigureCameraResponse :md5sum-) "263cde84e0c4384e57b9ce048385281d")
(setf (get ff_msgs::ConfigureCameraResponse :datatype-) "ff_msgs/ConfigureCameraResponse")
(setf (get ff_msgs::ConfigureCameraResponse :definition-)
      "


















uint8 BOTH      = 0
uint8 RECORDING = 1
uint8 STREAMING = 2

uint8 mode
float32 rate
uint32 width
uint32 height
float32 bitrate
---

")



(provide :ff_msgs/ConfigureCamera "263cde84e0c4384e57b9ce048385281d")


