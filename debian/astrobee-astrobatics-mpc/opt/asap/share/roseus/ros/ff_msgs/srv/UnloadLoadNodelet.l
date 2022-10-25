;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::UnloadLoadNodelet)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'UnloadLoadNodelet (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::UNLOADLOADNODELET")
  (make-package "FF_MSGS::UNLOADLOADNODELET"))
(unless (find-package "FF_MSGS::UNLOADLOADNODELETREQUEST")
  (make-package "FF_MSGS::UNLOADLOADNODELETREQUEST"))
(unless (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE")
  (make-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))

(in-package "ROS")





(defclass ff_msgs::UnloadLoadNodeletRequest
  :super ros::object
  :slots (_load _name _type _manager_name _remap_source_args _remap_target_args _my_argv _bond_id ))

(defmethod ff_msgs::UnloadLoadNodeletRequest
  (:init
   (&key
    ((:load __load) nil)
    ((:name __name) "")
    ((:type __type) "")
    ((:manager_name __manager_name) "")
    ((:remap_source_args __remap_source_args) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:remap_target_args __remap_target_args) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:my_argv __my_argv) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:bond_id __bond_id) "")
    )
   (send-super :init)
   (setq _load __load)
   (setq _name (string __name))
   (setq _type (string __type))
   (setq _manager_name (string __manager_name))
   (setq _remap_source_args __remap_source_args)
   (setq _remap_target_args __remap_target_args)
   (setq _my_argv __my_argv)
   (setq _bond_id (string __bond_id))
   self)
  (:load
   (&optional __load)
   (if __load (setq _load __load)) _load)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:manager_name
   (&optional __manager_name)
   (if __manager_name (setq _manager_name __manager_name)) _manager_name)
  (:remap_source_args
   (&optional __remap_source_args)
   (if __remap_source_args (setq _remap_source_args __remap_source_args)) _remap_source_args)
  (:remap_target_args
   (&optional __remap_target_args)
   (if __remap_target_args (setq _remap_target_args __remap_target_args)) _remap_target_args)
  (:my_argv
   (&optional __my_argv)
   (if __my_argv (setq _my_argv __my_argv)) _my_argv)
  (:bond_id
   (&optional __bond_id)
   (if __bond_id (setq _bond_id __bond_id)) _bond_id)
  (:serialization-length
   ()
   (+
    ;; bool _load
    1
    ;; string _name
    4 (length _name)
    ;; string _type
    4 (length _type)
    ;; string _manager_name
    4 (length _manager_name)
    ;; string[] _remap_source_args
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _remap_source_args)) 4
    ;; string[] _remap_target_args
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _remap_target_args)) 4
    ;; string[] _my_argv
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _my_argv)) 4
    ;; string _bond_id
    4 (length _bond_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _load
       (if _load (write-byte -1 s) (write-byte 0 s))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string _manager_name
       (write-long (length _manager_name) s) (princ _manager_name s)
     ;; string[] _remap_source_args
     (write-long (length _remap_source_args) s)
     (dolist (elem _remap_source_args)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _remap_target_args
     (write-long (length _remap_target_args) s)
     (dolist (elem _remap_target_args)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _my_argv
     (write-long (length _my_argv) s)
     (dolist (elem _my_argv)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string _bond_id
       (write-long (length _bond_id) s) (princ _bond_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _load
     (setq _load (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _manager_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _manager_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _remap_source_args
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_source_args (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _remap_source_args i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _remap_target_args
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _remap_target_args (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _remap_target_args i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _my_argv
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _my_argv (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _my_argv i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string _bond_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _bond_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(intern "*SUCCESSFUL*" (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(shadow '*SUCCESSFUL* (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(defconstant ff_msgs::UnloadLoadNodeletResponse::*SUCCESSFUL* 1)
(intern "*ROS_SERVICE_FAILED*" (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(shadow '*ROS_SERVICE_FAILED* (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(defconstant ff_msgs::UnloadLoadNodeletResponse::*ROS_SERVICE_FAILED* 2)
(intern "*NODE_NOT_IN_MAP*" (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(shadow '*NODE_NOT_IN_MAP* (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(defconstant ff_msgs::UnloadLoadNodeletResponse::*NODE_NOT_IN_MAP* 3)
(intern "*MANAGER_NAME_MISSING*" (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(shadow '*MANAGER_NAME_MISSING* (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(defconstant ff_msgs::UnloadLoadNodeletResponse::*MANAGER_NAME_MISSING* 4)
(intern "*TYPE_MISSING*" (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(shadow '*TYPE_MISSING* (find-package "FF_MSGS::UNLOADLOADNODELETRESPONSE"))
(defconstant ff_msgs::UnloadLoadNodeletResponse::*TYPE_MISSING* 5)
(defclass ff_msgs::UnloadLoadNodeletResponse
  :super ros::object
  :slots (_result ))

(defmethod ff_msgs::UnloadLoadNodeletResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass ff_msgs::UnloadLoadNodelet
  :super ros::object
  :slots ())

(setf (get ff_msgs::UnloadLoadNodelet :md5sum-) "7f19eb1a2a34b5a95695a9d88b20e227")
(setf (get ff_msgs::UnloadLoadNodelet :datatype-) "ff_msgs/UnloadLoadNodelet")
(setf (get ff_msgs::UnloadLoadNodelet :request) ff_msgs::UnloadLoadNodeletRequest)
(setf (get ff_msgs::UnloadLoadNodelet :response) ff_msgs::UnloadLoadNodeletResponse)

(defmethod ff_msgs::UnloadLoadNodeletRequest
  (:response () (instance ff_msgs::UnloadLoadNodeletResponse :init)))

(setf (get ff_msgs::UnloadLoadNodeletRequest :md5sum-) "7f19eb1a2a34b5a95695a9d88b20e227")
(setf (get ff_msgs::UnloadLoadNodeletRequest :datatype-) "ff_msgs/UnloadLoadNodeletRequest")
(setf (get ff_msgs::UnloadLoadNodeletRequest :definition-)
      "





bool load

string name



string type



string manager_name

string[] remap_source_args
string[] remap_target_args
string[] my_argv
string bond_id
---
int32 result

int32 SUCCESSFUL            = 1

int32 ROS_SERVICE_FAILED    = 2



int32 NODE_NOT_IN_MAP       = 3


int32 MANAGER_NAME_MISSING  = 4


int32 TYPE_MISSING          = 5

")

(setf (get ff_msgs::UnloadLoadNodeletResponse :md5sum-) "7f19eb1a2a34b5a95695a9d88b20e227")
(setf (get ff_msgs::UnloadLoadNodeletResponse :datatype-) "ff_msgs/UnloadLoadNodeletResponse")
(setf (get ff_msgs::UnloadLoadNodeletResponse :definition-)
      "





bool load

string name



string type



string manager_name

string[] remap_source_args
string[] remap_target_args
string[] my_argv
string bond_id
---
int32 result

int32 SUCCESSFUL            = 1

int32 ROS_SERVICE_FAILED    = 2



int32 NODE_NOT_IN_MAP       = 3


int32 MANAGER_NAME_MISSING  = 4


int32 TYPE_MISSING          = 5

")



(provide :ff_msgs/UnloadLoadNodelet "7f19eb1a2a34b5a95695a9d88b20e227")


