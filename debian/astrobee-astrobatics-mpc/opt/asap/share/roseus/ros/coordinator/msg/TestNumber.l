;; Auto-generated. Do not edit!


(when (boundp 'coordinator::TestNumber)
  (if (not (find-package "COORDINATOR"))
    (make-package "COORDINATOR"))
  (shadow 'TestNumber (find-package "COORDINATOR")))
(unless (find-package "COORDINATOR::TESTNUMBER")
  (make-package "COORDINATOR::TESTNUMBER"))

(in-package "ROS")
;;//! \htmlinclude TestNumber.msg.html


(defclass coordinator::TestNumber
  :super ros::object
  :slots (_stamp _test_number _role ))

(defmethod coordinator::TestNumber
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:test_number __test_number) 0)
    ((:role __role) "")
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _test_number (round __test_number))
   (setq _role (string __role))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:test_number
   (&optional __test_number)
   (if __test_number (setq _test_number __test_number)) _test_number)
  (:role
   (&optional __role)
   (if __role (setq _role __role)) _role)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; int32 _test_number
    4
    ;; string _role
    4 (length _role)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _test_number
       (write-long _test_number s)
     ;; string _role
       (write-long (length _role) s) (princ _role s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _test_number
     (setq _test_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _role
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _role (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get coordinator::TestNumber :md5sum-) "a546cf58ee360e93604091100205de8f")
(setf (get coordinator::TestNumber :datatype-) "coordinator/TestNumber")
(setf (get coordinator::TestNumber :definition-)
      "time stamp
int32 test_number
string role

")



(provide :coordinator/TestNumber "a546cf58ee360e93604091100205de8f")


