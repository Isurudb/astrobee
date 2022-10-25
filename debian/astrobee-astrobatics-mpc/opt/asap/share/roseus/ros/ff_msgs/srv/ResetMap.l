;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::ResetMap)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'ResetMap (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::RESETMAP")
  (make-package "FF_MSGS::RESETMAP"))
(unless (find-package "FF_MSGS::RESETMAPREQUEST")
  (make-package "FF_MSGS::RESETMAPREQUEST"))
(unless (find-package "FF_MSGS::RESETMAPRESPONSE")
  (make-package "FF_MSGS::RESETMAPRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::ResetMapRequest
  :super ros::object
  :slots (_map_file ))

(defmethod ff_msgs::ResetMapRequest
  (:init
   (&key
    ((:map_file __map_file) "")
    )
   (send-super :init)
   (setq _map_file (string __map_file))
   self)
  (:map_file
   (&optional __map_file)
   (if __map_file (setq _map_file __map_file)) _map_file)
  (:serialization-length
   ()
   (+
    ;; string _map_file
    4 (length _map_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _map_file
       (write-long (length _map_file) s) (princ _map_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _map_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _map_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ff_msgs::ResetMapResponse
  :super ros::object
  :slots ())

(defmethod ff_msgs::ResetMapResponse
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

(defclass ff_msgs::ResetMap
  :super ros::object
  :slots ())

(setf (get ff_msgs::ResetMap :md5sum-) "a377c8d7c4f71636969846ebf44e4df2")
(setf (get ff_msgs::ResetMap :datatype-) "ff_msgs/ResetMap")
(setf (get ff_msgs::ResetMap :request) ff_msgs::ResetMapRequest)
(setf (get ff_msgs::ResetMap :response) ff_msgs::ResetMapResponse)

(defmethod ff_msgs::ResetMapRequest
  (:response () (instance ff_msgs::ResetMapResponse :init)))

(setf (get ff_msgs::ResetMapRequest :md5sum-) "a377c8d7c4f71636969846ebf44e4df2")
(setf (get ff_msgs::ResetMapRequest :datatype-) "ff_msgs/ResetMapRequest")
(setf (get ff_msgs::ResetMapRequest :definition-)
      "



















string map_file
---

")

(setf (get ff_msgs::ResetMapResponse :md5sum-) "a377c8d7c4f71636969846ebf44e4df2")
(setf (get ff_msgs::ResetMapResponse :datatype-) "ff_msgs/ResetMapResponse")
(setf (get ff_msgs::ResetMapResponse :definition-)
      "



















string map_file
---

")



(provide :ff_msgs/ResetMap "a377c8d7c4f71636969846ebf44e4df2")


