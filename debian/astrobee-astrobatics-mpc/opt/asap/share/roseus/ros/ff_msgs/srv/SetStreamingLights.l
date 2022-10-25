;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::SetStreamingLights)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'SetStreamingLights (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::SETSTREAMINGLIGHTS")
  (make-package "FF_MSGS::SETSTREAMINGLIGHTS"))
(unless (find-package "FF_MSGS::SETSTREAMINGLIGHTSREQUEST")
  (make-package "FF_MSGS::SETSTREAMINGLIGHTSREQUEST"))
(unless (find-package "FF_MSGS::SETSTREAMINGLIGHTSRESPONSE")
  (make-package "FF_MSGS::SETSTREAMINGLIGHTSRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::SetStreamingLightsRequest
  :super ros::object
  :slots (_state ))

(defmethod ff_msgs::SetStreamingLightsRequest
  (:init
   (&key
    ((:state __state) nil)
    )
   (send-super :init)
   (setq _state __state)
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; bool _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::SetStreamingLightsResponse
  :super ros::object
  :slots (_success ))

(defmethod ff_msgs::SetStreamingLightsResponse
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

(defclass ff_msgs::SetStreamingLights
  :super ros::object
  :slots ())

(setf (get ff_msgs::SetStreamingLights :md5sum-) "4581db74aae4efc6534413a8b210908c")
(setf (get ff_msgs::SetStreamingLights :datatype-) "ff_msgs/SetStreamingLights")
(setf (get ff_msgs::SetStreamingLights :request) ff_msgs::SetStreamingLightsRequest)
(setf (get ff_msgs::SetStreamingLights :response) ff_msgs::SetStreamingLightsResponse)

(defmethod ff_msgs::SetStreamingLightsRequest
  (:response () (instance ff_msgs::SetStreamingLightsResponse :init)))

(setf (get ff_msgs::SetStreamingLightsRequest :md5sum-) "4581db74aae4efc6534413a8b210908c")
(setf (get ff_msgs::SetStreamingLightsRequest :datatype-) "ff_msgs/SetStreamingLightsRequest")
(setf (get ff_msgs::SetStreamingLightsRequest :definition-)
      "




















bool state
---
bool success
")

(setf (get ff_msgs::SetStreamingLightsResponse :md5sum-) "4581db74aae4efc6534413a8b210908c")
(setf (get ff_msgs::SetStreamingLightsResponse :datatype-) "ff_msgs/SetStreamingLightsResponse")
(setf (get ff_msgs::SetStreamingLightsResponse :definition-)
      "




















bool state
---
bool success
")



(provide :ff_msgs/SetStreamingLights "4581db74aae4efc6534413a8b210908c")


