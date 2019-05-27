;; Auto-generated. Do not edit!


(when (boundp 'controller::pose)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'pose (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::POSE")
  (make-package "CONTROLLER::POSE"))

(in-package "ROS")
;;//! \htmlinclude pose.msg.html


(defclass controller::pose
  :super ros::object
  :slots (_pose ))

(defmethod controller::pose
  (:init
   (&key
    ((:pose __pose) "")
    )
   (send-super :init)
   (setq _pose (string __pose))
   self)
  (:pose
   (&optional __pose)
   (if __pose (setq _pose __pose)) _pose)
  (:serialization-length
   ()
   (+
    ;; string _pose
    4 (length _pose)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _pose
       (write-long (length _pose) s) (princ _pose s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _pose
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pose (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get controller::pose :md5sum-) "0cbc29f91adcb71214b885e1c0073aa7")
(setf (get controller::pose :datatype-) "controller/pose")
(setf (get controller::pose :definition-)
      "string pose

")



(provide :controller/pose "0cbc29f91adcb71214b885e1c0073aa7")


