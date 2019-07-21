;; Auto-generated. Do not edit!


(when (boundp 'controller::interface)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'interface (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::INTERFACE")
  (make-package "CONTROLLER::INTERFACE"))
(unless (find-package "CONTROLLER::INTERFACEREQUEST")
  (make-package "CONTROLLER::INTERFACEREQUEST"))
(unless (find-package "CONTROLLER::INTERFACERESPONSE")
  (make-package "CONTROLLER::INTERFACERESPONSE"))

(in-package "ROS")





(defclass controller::interfaceRequest
  :super ros::object
  :slots (_cmd ))

(defmethod controller::interfaceRequest
  (:init
   (&key
    ((:cmd __cmd) "")
    )
   (send-super :init)
   (setq _cmd (string __cmd))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:serialization-length
   ()
   (+
    ;; string _cmd
    4 (length _cmd)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass controller::interfaceResponse
  :super ros::object
  :slots (_return_code ))

(defmethod controller::interfaceResponse
  (:init
   (&key
    ((:return_code __return_code) 0)
    )
   (send-super :init)
   (setq _return_code (round __return_code))
   self)
  (:return_code
   (&optional __return_code)
   (if __return_code (setq _return_code __return_code)) _return_code)
  (:serialization-length
   ()
   (+
    ;; int64 _return_code
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _return_code
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _return_code (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _return_code) (= (length (_return_code . bv)) 2)) ;; bignum
              (write-long (ash (elt (_return_code . bv) 0) 0) s)
              (write-long (ash (elt (_return_code . bv) 1) -1) s))
             ((and (class _return_code) (= (length (_return_code . bv)) 1)) ;; big1
              (write-long (elt (_return_code . bv) 0) s)
              (write-long (if (>= _return_code 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _return_code s)(write-long (if (>= _return_code 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _return_code
#+(or :alpha :irix6 :x86_64)
      (setf _return_code (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _return_code (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass controller::interface
  :super ros::object
  :slots ())

(setf (get controller::interface :md5sum-) "5fd22aa1f159fb59b0d35e4ba0a4f558")
(setf (get controller::interface :datatype-) "controller/interface")
(setf (get controller::interface :request) controller::interfaceRequest)
(setf (get controller::interface :response) controller::interfaceResponse)

(defmethod controller::interfaceRequest
  (:response () (instance controller::interfaceResponse :init)))

(setf (get controller::interfaceRequest :md5sum-) "5fd22aa1f159fb59b0d35e4ba0a4f558")
(setf (get controller::interfaceRequest :datatype-) "controller/interfaceRequest")
(setf (get controller::interfaceRequest :definition-)
      "string cmd
---
int64 return_code

")

(setf (get controller::interfaceResponse :md5sum-) "5fd22aa1f159fb59b0d35e4ba0a4f558")
(setf (get controller::interfaceResponse :datatype-) "controller/interfaceResponse")
(setf (get controller::interfaceResponse :definition-)
      "string cmd
---
int64 return_code

")



(provide :controller/interface "5fd22aa1f159fb59b0d35e4ba0a4f558")


