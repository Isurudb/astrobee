;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::RegisterPlanner)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'RegisterPlanner (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::REGISTERPLANNER")
  (make-package "FF_MSGS::REGISTERPLANNER"))
(unless (find-package "FF_MSGS::REGISTERPLANNERREQUEST")
  (make-package "FF_MSGS::REGISTERPLANNERREQUEST"))
(unless (find-package "FF_MSGS::REGISTERPLANNERRESPONSE")
  (make-package "FF_MSGS::REGISTERPLANNERRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::RegisterPlannerRequest
  :super ros::object
  :slots (_name _description _unregister ))

(defmethod ff_msgs::RegisterPlannerRequest
  (:init
   (&key
    ((:name __name) "")
    ((:description __description) "")
    ((:unregister __unregister) nil)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _description (string __description))
   (setq _unregister __unregister)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:unregister
   (&optional __unregister)
   (if __unregister (setq _unregister __unregister)) _unregister)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; string _description
    4 (length _description)
    ;; bool _unregister
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;; bool _unregister
       (if _unregister (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _unregister
     (setq _unregister (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ff_msgs::RegisterPlannerResponse
  :super ros::object
  :slots ())

(defmethod ff_msgs::RegisterPlannerResponse
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

(defclass ff_msgs::RegisterPlanner
  :super ros::object
  :slots ())

(setf (get ff_msgs::RegisterPlanner :md5sum-) "e247f0a3c6e3085865c44afa8ad187df")
(setf (get ff_msgs::RegisterPlanner :datatype-) "ff_msgs/RegisterPlanner")
(setf (get ff_msgs::RegisterPlanner :request) ff_msgs::RegisterPlannerRequest)
(setf (get ff_msgs::RegisterPlanner :response) ff_msgs::RegisterPlannerResponse)

(defmethod ff_msgs::RegisterPlannerRequest
  (:response () (instance ff_msgs::RegisterPlannerResponse :init)))

(setf (get ff_msgs::RegisterPlannerRequest :md5sum-) "e247f0a3c6e3085865c44afa8ad187df")
(setf (get ff_msgs::RegisterPlannerRequest :datatype-) "ff_msgs/RegisterPlannerRequest")
(setf (get ff_msgs::RegisterPlannerRequest :definition-)
      "



















string name
string description
bool unregister
---
")

(setf (get ff_msgs::RegisterPlannerResponse :md5sum-) "e247f0a3c6e3085865c44afa8ad187df")
(setf (get ff_msgs::RegisterPlannerResponse :datatype-) "ff_msgs/RegisterPlannerResponse")
(setf (get ff_msgs::RegisterPlannerResponse :definition-)
      "



















string name
string description
bool unregister
---
")



(provide :ff_msgs/RegisterPlanner "e247f0a3c6e3085865c44afa8ad187df")


