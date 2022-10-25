;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetBool)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetBool (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETBOOL")
  (make-package "FF_MSGS::SETBOOL"))
(unless (find-package "FF_MSGS::SETBOOLREQUEST")
  (make-package "FF_MSGS::SETBOOLREQUEST"))
(unless (find-package "FF_MSGS::SETBOOLRESPONSE")
  (make-package "FF_MSGS::SETBOOLRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::SetBoolRequest
  :super ros::object
  :slots (_enable ))

(defmethod ff_msgs::SetBoolRequest
  (:init
   (&key
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _enable __enable)
   self)
  (:enable
   (&optional __enable)
   (if __enable (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::SetBoolResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::SetBoolResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::SetBool
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetBool :md5sum-) "6a0b406242562fc416b2c9c8a3efb051")
(setf (get ff_msgs::SetBool :datatype-) "ff_msgs/SetBool")
(setf (get ff_msgs::SetBool :request) ff_msgs::SetBoolRequest)
(setf (get ff_msgs::SetBool :response) ff_msgs::SetBoolResponse)

(defmethod ff_msgs::SetBoolRequest
  (:response () (instance ff_msgs::SetBoolResponse :init)))

(setf (get ff_msgs::SetBoolRequest :md5sum-) "6a0b406242562fc416b2c9c8a3efb051")
(setf (get ff_msgs::SetBoolRequest :datatype-) "ff_msgs/SetBoolRequest")
(setf (get ff_msgs::SetBoolRequest :definition-)
      "


















bool enable
---
bool success

")

(setf (get ff_msgs::SetBoolResponse :md5sum-) "6a0b406242562fc416b2c9c8a3efb051")
(setf (get ff_msgs::SetBoolResponse :datatype-) "ff_msgs/SetBoolResponse")
(setf (get ff_msgs::SetBoolResponse :definition-)
      "


















bool enable
---
bool success

")



(provide :ff_msgs/SetBool "6a0b406242562fc416b2c9c8a3efb051")


