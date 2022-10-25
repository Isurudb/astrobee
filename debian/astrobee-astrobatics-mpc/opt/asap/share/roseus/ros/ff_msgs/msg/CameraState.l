;; Auto-generated. Do not edit!


(when (boundp 'ff_msgs::CameraState)
  (if (not (find-package "FF_MSGS"))
    (make-package "FF_MSGS"))
  (shadow 'CameraState (find-package "FF_MSGS")))
(unless (find-package "FF_MSGS::CAMERASTATE")
  (make-package "FF_MSGS::CAMERASTATE"))

(in-package "ROS")
;;//! \htmlinclude CameraState.msg.html


(defclass ff_msgs::CameraState
  :super ros::object
  :slots (_camera_name _streaming _stream_width _stream_height _stream_rate _recording _record_width _record_height _record_rate _bandwidth ))

(defmethod ff_msgs::CameraState
  (:init
   (&key
    ((:camera_name __camera_name) "")
    ((:streaming __streaming) nil)
    ((:stream_width __stream_width) 0)
    ((:stream_height __stream_height) 0)
    ((:stream_rate __stream_rate) 0.0)
    ((:recording __recording) nil)
    ((:record_width __record_width) 0)
    ((:record_height __record_height) 0)
    ((:record_rate __record_rate) 0.0)
    ((:bandwidth __bandwidth) 0.0)
    )
   (send-super :init)
   (setq _camera_name (string __camera_name))
   (setq _streaming __streaming)
   (setq _stream_width (round __stream_width))
   (setq _stream_height (round __stream_height))
   (setq _stream_rate (float __stream_rate))
   (setq _recording __recording)
   (setq _record_width (round __record_width))
   (setq _record_height (round __record_height))
   (setq _record_rate (float __record_rate))
   (setq _bandwidth (float __bandwidth))
   self)
  (:camera_name
   (&optional __camera_name)
   (if __camera_name (setq _camera_name __camera_name)) _camera_name)
  (:streaming
   (&optional __streaming)
   (if __streaming (setq _streaming __streaming)) _streaming)
  (:stream_width
   (&optional __stream_width)
   (if __stream_width (setq _stream_width __stream_width)) _stream_width)
  (:stream_height
   (&optional __stream_height)
   (if __stream_height (setq _stream_height __stream_height)) _stream_height)
  (:stream_rate
   (&optional __stream_rate)
   (if __stream_rate (setq _stream_rate __stream_rate)) _stream_rate)
  (:recording
   (&optional __recording)
   (if __recording (setq _recording __recording)) _recording)
  (:record_width
   (&optional __record_width)
   (if __record_width (setq _record_width __record_width)) _record_width)
  (:record_height
   (&optional __record_height)
   (if __record_height (setq _record_height __record_height)) _record_height)
  (:record_rate
   (&optional __record_rate)
   (if __record_rate (setq _record_rate __record_rate)) _record_rate)
  (:bandwidth
   (&optional __bandwidth)
   (if __bandwidth (setq _bandwidth __bandwidth)) _bandwidth)
  (:serialization-length
   ()
   (+
    ;; string _camera_name
    4 (length _camera_name)
    ;; bool _streaming
    1
    ;; uint16 _stream_width
    2
    ;; uint16 _stream_height
    2
    ;; float32 _stream_rate
    4
    ;; bool _recording
    1
    ;; uint16 _record_width
    2
    ;; uint16 _record_height
    2
    ;; float32 _record_rate
    4
    ;; float32 _bandwidth
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _camera_name
       (write-long (length _camera_name) s) (princ _camera_name s)
     ;; bool _streaming
       (if _streaming (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _stream_width
       (write-word _stream_width s)
     ;; uint16 _stream_height
       (write-word _stream_height s)
     ;; float32 _stream_rate
       (sys::poke _stream_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _recording
       (if _recording (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _record_width
       (write-word _record_width s)
     ;; uint16 _record_height
       (write-word _record_height s)
     ;; float32 _record_rate
       (sys::poke _record_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bandwidth
       (sys::poke _bandwidth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _camera_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _camera_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _streaming
     (setq _streaming (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _stream_width
     (setq _stream_width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _stream_height
     (setq _stream_height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _stream_rate
     (setq _stream_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _recording
     (setq _recording (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _record_width
     (setq _record_width (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _record_height
     (setq _record_height (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _record_rate
     (setq _record_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bandwidth
     (setq _bandwidth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get ff_msgs::CameraState :md5sum-) "644cfd14384d17cf28911b625a446f53")
(setf (get ff_msgs::CameraState :datatype-) "ff_msgs/CameraState")
(setf (get ff_msgs::CameraState :definition-)
      "# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the \"License\"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an \"AS IS\" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
#
# CameraState message, *MUST* be kept in sync with camera portion of
# rapid::ext::astrobee::TelemetryState

# nav_cam, dock_cam, etc.
string camera_name

# streaming to ground
bool streaming

# image width
uint16 stream_width
# image height
uint16 stream_height
# Rate in Hz
float32 stream_rate

# recording to disk
bool recording

# image width
uint16 record_width
# image height
uint16 record_height
# Rate in Hz
float32 record_rate

# only for sci cam
float32 bandwidth

")



(provide :ff_msgs/CameraState "644cfd14384d17cf28911b625a446f53")


