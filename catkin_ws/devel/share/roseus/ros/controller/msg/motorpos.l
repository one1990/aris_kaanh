;; Auto-generated. Do not edit!


(when (boundp 'controller::motorpos)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'motorpos (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::MOTORPOS")
  (make-package "CONTROLLER::MOTORPOS"))

(in-package "ROS")
;;//! \htmlinclude motorpos.msg.html


(defclass controller::motorpos
  :super ros::object
  :slots (_angular ))

(defmethod controller::motorpos
  (:init
   (&key
    ((:angular __angular) 0)
    )
   (send-super :init)
   (setq _angular (round __angular))
   self)
  (:angular
   (&optional __angular)
   (if __angular (setq _angular __angular)) _angular)
  (:serialization-length
   ()
   (+
    ;; uint16 _angular
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _angular
       (write-word _angular s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _angular
     (setq _angular (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get controller::motorpos :md5sum-) "620daad8e7540d60806fb147811e141d")
(setf (get controller::motorpos :datatype-) "controller/motorpos")
(setf (get controller::motorpos :definition-)
      "uint16 angular

")



(provide :controller/motorpos "620daad8e7540d60806fb147811e141d")


