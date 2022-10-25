;; Auto-generated. Do not edit!


(when (boundp 'coordinator::StatusSecondary)
  (if (not (find-package "COORDINATOR"))
    (make-package "COORDINATOR"))
  (shadow 'StatusSecondary (find-package "COORDINATOR")))
(unless (find-package "COORDINATOR::STATUSSECONDARY")
  (make-package "COORDINATOR::STATUSSECONDARY"))

(in-package "ROS")
;;//! \htmlinclude StatusSecondary.msg.html


(defclass coordinator::StatusSecondary
  :super ros::object
  :slots (_stamp _test_number _default_control _flight_mode _test_finished _coord_ok ))

(defmethod coordinator::StatusSecondary
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:test_number __test_number) 0)
    ((:default_control __default_control) nil)
    ((:flight_mode __flight_mode) "")
    ((:test_finished __test_finished) nil)
    ((:coord_ok __coord_ok) nil)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _test_number (round __test_number))
   (setq _default_control __default_control)
   (setq _flight_mode (string __flight_mode))
   (setq _test_finished __test_finished)
   (setq _coord_ok __coord_ok)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:test_number
   (&optional __test_number)
   (if __test_number (setq _test_number __test_number)) _test_number)
  (:default_control
   (&optional __default_control)
   (if __default_control (setq _default_control __default_control)) _default_control)
  (:flight_mode
   (&optional __flight_mode)
   (if __flight_mode (setq _flight_mode __flight_mode)) _flight_mode)
  (:test_finished
   (&optional __test_finished)
   (if __test_finished (setq _test_finished __test_finished)) _test_finished)
  (:coord_ok
   (&optional __coord_ok)
   (if __coord_ok (setq _coord_ok __coord_ok)) _coord_ok)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; int32 _test_number
    4
    ;; bool _default_control
    1
    ;; string _flight_mode
    4 (length _flight_mode)
    ;; bool _test_finished
    1
    ;; bool _coord_ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; int32 _test_number
       (write-long _test_number s)
     ;; bool _default_control
       (if _default_control (write-byte -1 s) (write-byte 0 s))
     ;; string _flight_mode
       (write-long (length _flight_mode) s) (princ _flight_mode s)
     ;; bool _test_finished
       (if _test_finished (write-byte -1 s) (write-byte 0 s))
     ;; bool _coord_ok
       (if _coord_ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _test_number
     (setq _test_number (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _default_control
     (setq _default_control (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _flight_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _flight_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _test_finished
     (setq _test_finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _coord_ok
     (setq _coord_ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get coordinator::StatusSecondary :md5sum-) "73edc0983b4cd83de63e2948253578db")
(setf (get coordinator::StatusSecondary :datatype-) "coordinator/StatusSecondary")
(setf (get coordinator::StatusSecondary :definition-)
      "time stamp

# base (shared) values
int32 test_number
bool default_control
string flight_mode
bool test_finished
bool coord_ok

# StatusSecondary
")



(provide :coordinator/StatusSecondary "73edc0983b4cd83de63e2948253578db")


