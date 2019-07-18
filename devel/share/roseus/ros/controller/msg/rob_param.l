;; Auto-generated. Do not edit!


(when (boundp 'controller::rob_param)
  (if (not (find-package "CONTROLLER"))
    (make-package "CONTROLLER"))
  (shadow 'rob_param (find-package "CONTROLLER")))
(unless (find-package "CONTROLLER::ROB_PARAM")
  (make-package "CONTROLLER::ROB_PARAM"))

(in-package "ROS")
;;//! \htmlinclude rob_param.msg.html


(defclass controller::rob_param
  :super ros::object
  :slots (_x _y _z _rx _ry _rz _j1 _j2 _j3 _j4 _j5 _j6 _select_mode _start _forward_back _gear _rs_button _md_ds_button _rc_en_button _start_stop ))

(defmethod controller::rob_param
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:z __z) 0)
    ((:rx __rx) 0)
    ((:ry __ry) 0)
    ((:rz __rz) 0.0)
    ((:j1 __j1) 0)
    ((:j2 __j2) 0)
    ((:j3 __j3) 0)
    ((:j4 __j4) 0)
    ((:j5 __j5) 0)
    ((:j6 __j6) 0.0)
    ((:select_mode __select_mode) 0)
    ((:start __start) 0)
    ((:forward_back __forward_back) 0.0)
    ((:gear __gear) 0)
    ((:rs_button __rs_button) 0)
    ((:md_ds_button __md_ds_button) 0)
    ((:rc_en_button __rc_en_button) 0.0)
    ((:start_stop __start_stop) 0.0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _z (round __z))
   (setq _rx (round __rx))
   (setq _ry (round __ry))
   (setq _rz (float __rz))
   (setq _j1 (round __j1))
   (setq _j2 (round __j2))
   (setq _j3 (round __j3))
   (setq _j4 (round __j4))
   (setq _j5 (round __j5))
   (setq _j6 (float __j6))
   (setq _select_mode (round __select_mode))
   (setq _start (round __start))
   (setq _forward_back (float __forward_back))
   (setq _gear (round __gear))
   (setq _rs_button (round __rs_button))
   (setq _md_ds_button (round __md_ds_button))
   (setq _rc_en_button (float __rc_en_button))
   (setq _start_stop (float __start_stop))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:rx
   (&optional __rx)
   (if __rx (setq _rx __rx)) _rx)
  (:ry
   (&optional __ry)
   (if __ry (setq _ry __ry)) _ry)
  (:rz
   (&optional __rz)
   (if __rz (setq _rz __rz)) _rz)
  (:j1
   (&optional __j1)
   (if __j1 (setq _j1 __j1)) _j1)
  (:j2
   (&optional __j2)
   (if __j2 (setq _j2 __j2)) _j2)
  (:j3
   (&optional __j3)
   (if __j3 (setq _j3 __j3)) _j3)
  (:j4
   (&optional __j4)
   (if __j4 (setq _j4 __j4)) _j4)
  (:j5
   (&optional __j5)
   (if __j5 (setq _j5 __j5)) _j5)
  (:j6
   (&optional __j6)
   (if __j6 (setq _j6 __j6)) _j6)
  (:select_mode
   (&optional __select_mode)
   (if __select_mode (setq _select_mode __select_mode)) _select_mode)
  (:start
   (&optional __start)
   (if __start (setq _start __start)) _start)
  (:forward_back
   (&optional __forward_back)
   (if __forward_back (setq _forward_back __forward_back)) _forward_back)
  (:gear
   (&optional __gear)
   (if __gear (setq _gear __gear)) _gear)
  (:rs_button
   (&optional __rs_button)
   (if __rs_button (setq _rs_button __rs_button)) _rs_button)
  (:md_ds_button
   (&optional __md_ds_button)
   (if __md_ds_button (setq _md_ds_button __md_ds_button)) _md_ds_button)
  (:rc_en_button
   (&optional __rc_en_button)
   (if __rc_en_button (setq _rc_en_button __rc_en_button)) _rc_en_button)
  (:start_stop
   (&optional __start_stop)
   (if __start_stop (setq _start_stop __start_stop)) _start_stop)
  (:serialization-length
   ()
   (+
    ;; int64 _x
    8
    ;; int64 _y
    8
    ;; int64 _z
    8
    ;; int64 _rx
    8
    ;; int64 _ry
    8
    ;; float64 _rz
    8
    ;; int64 _j1
    8
    ;; int64 _j2
    8
    ;; int64 _j3
    8
    ;; int64 _j4
    8
    ;; int64 _j5
    8
    ;; float64 _j6
    8
    ;; int64 _select_mode
    8
    ;; int64 _start
    8
    ;; float64 _forward_back
    8
    ;; int64 _gear
    8
    ;; int64 _rs_button
    8
    ;; int64 _md_ds_button
    8
    ;; float64 _rc_en_button
    8
    ;; float64 _start_stop
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _x
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x) (= (length (_x . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x . bv) 0) 0) s)
              (write-long (ash (elt (_x . bv) 1) -1) s))
             ((and (class _x) (= (length (_x . bv)) 1)) ;; big1
              (write-long (elt (_x . bv) 0) s)
              (write-long (if (>= _x 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x s)(write-long (if (>= _x 0) 0 #xffffffff) s)))
     ;; int64 _y
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y) (= (length (_y . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y . bv) 0) 0) s)
              (write-long (ash (elt (_y . bv) 1) -1) s))
             ((and (class _y) (= (length (_y . bv)) 1)) ;; big1
              (write-long (elt (_y . bv) 0) s)
              (write-long (if (>= _y 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y s)(write-long (if (>= _y 0) 0 #xffffffff) s)))
     ;; int64 _z
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _z (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _z) (= (length (_z . bv)) 2)) ;; bignum
              (write-long (ash (elt (_z . bv) 0) 0) s)
              (write-long (ash (elt (_z . bv) 1) -1) s))
             ((and (class _z) (= (length (_z . bv)) 1)) ;; big1
              (write-long (elt (_z . bv) 0) s)
              (write-long (if (>= _z 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _z s)(write-long (if (>= _z 0) 0 #xffffffff) s)))
     ;; int64 _rx
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _rx (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _rx) (= (length (_rx . bv)) 2)) ;; bignum
              (write-long (ash (elt (_rx . bv) 0) 0) s)
              (write-long (ash (elt (_rx . bv) 1) -1) s))
             ((and (class _rx) (= (length (_rx . bv)) 1)) ;; big1
              (write-long (elt (_rx . bv) 0) s)
              (write-long (if (>= _rx 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _rx s)(write-long (if (>= _rx 0) 0 #xffffffff) s)))
     ;; int64 _ry
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _ry (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _ry) (= (length (_ry . bv)) 2)) ;; bignum
              (write-long (ash (elt (_ry . bv) 0) 0) s)
              (write-long (ash (elt (_ry . bv) 1) -1) s))
             ((and (class _ry) (= (length (_ry . bv)) 1)) ;; big1
              (write-long (elt (_ry . bv) 0) s)
              (write-long (if (>= _ry 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _ry s)(write-long (if (>= _ry 0) 0 #xffffffff) s)))
     ;; float64 _rz
       (sys::poke _rz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _j1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _j1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _j1) (= (length (_j1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_j1 . bv) 0) 0) s)
              (write-long (ash (elt (_j1 . bv) 1) -1) s))
             ((and (class _j1) (= (length (_j1 . bv)) 1)) ;; big1
              (write-long (elt (_j1 . bv) 0) s)
              (write-long (if (>= _j1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _j1 s)(write-long (if (>= _j1 0) 0 #xffffffff) s)))
     ;; int64 _j2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _j2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _j2) (= (length (_j2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_j2 . bv) 0) 0) s)
              (write-long (ash (elt (_j2 . bv) 1) -1) s))
             ((and (class _j2) (= (length (_j2 . bv)) 1)) ;; big1
              (write-long (elt (_j2 . bv) 0) s)
              (write-long (if (>= _j2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _j2 s)(write-long (if (>= _j2 0) 0 #xffffffff) s)))
     ;; int64 _j3
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _j3 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _j3) (= (length (_j3 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_j3 . bv) 0) 0) s)
              (write-long (ash (elt (_j3 . bv) 1) -1) s))
             ((and (class _j3) (= (length (_j3 . bv)) 1)) ;; big1
              (write-long (elt (_j3 . bv) 0) s)
              (write-long (if (>= _j3 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _j3 s)(write-long (if (>= _j3 0) 0 #xffffffff) s)))
     ;; int64 _j4
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _j4 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _j4) (= (length (_j4 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_j4 . bv) 0) 0) s)
              (write-long (ash (elt (_j4 . bv) 1) -1) s))
             ((and (class _j4) (= (length (_j4 . bv)) 1)) ;; big1
              (write-long (elt (_j4 . bv) 0) s)
              (write-long (if (>= _j4 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _j4 s)(write-long (if (>= _j4 0) 0 #xffffffff) s)))
     ;; int64 _j5
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _j5 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _j5) (= (length (_j5 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_j5 . bv) 0) 0) s)
              (write-long (ash (elt (_j5 . bv) 1) -1) s))
             ((and (class _j5) (= (length (_j5 . bv)) 1)) ;; big1
              (write-long (elt (_j5 . bv) 0) s)
              (write-long (if (>= _j5 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _j5 s)(write-long (if (>= _j5 0) 0 #xffffffff) s)))
     ;; float64 _j6
       (sys::poke _j6 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _select_mode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _select_mode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _select_mode) (= (length (_select_mode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_select_mode . bv) 0) 0) s)
              (write-long (ash (elt (_select_mode . bv) 1) -1) s))
             ((and (class _select_mode) (= (length (_select_mode . bv)) 1)) ;; big1
              (write-long (elt (_select_mode . bv) 0) s)
              (write-long (if (>= _select_mode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _select_mode s)(write-long (if (>= _select_mode 0) 0 #xffffffff) s)))
     ;; int64 _start
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _start (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _start) (= (length (_start . bv)) 2)) ;; bignum
              (write-long (ash (elt (_start . bv) 0) 0) s)
              (write-long (ash (elt (_start . bv) 1) -1) s))
             ((and (class _start) (= (length (_start . bv)) 1)) ;; big1
              (write-long (elt (_start . bv) 0) s)
              (write-long (if (>= _start 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _start s)(write-long (if (>= _start 0) 0 #xffffffff) s)))
     ;; float64 _forward_back
       (sys::poke _forward_back (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _gear
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _gear (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _gear) (= (length (_gear . bv)) 2)) ;; bignum
              (write-long (ash (elt (_gear . bv) 0) 0) s)
              (write-long (ash (elt (_gear . bv) 1) -1) s))
             ((and (class _gear) (= (length (_gear . bv)) 1)) ;; big1
              (write-long (elt (_gear . bv) 0) s)
              (write-long (if (>= _gear 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _gear s)(write-long (if (>= _gear 0) 0 #xffffffff) s)))
     ;; int64 _rs_button
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _rs_button (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _rs_button) (= (length (_rs_button . bv)) 2)) ;; bignum
              (write-long (ash (elt (_rs_button . bv) 0) 0) s)
              (write-long (ash (elt (_rs_button . bv) 1) -1) s))
             ((and (class _rs_button) (= (length (_rs_button . bv)) 1)) ;; big1
              (write-long (elt (_rs_button . bv) 0) s)
              (write-long (if (>= _rs_button 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _rs_button s)(write-long (if (>= _rs_button 0) 0 #xffffffff) s)))
     ;; int64 _md_ds_button
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _md_ds_button (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _md_ds_button) (= (length (_md_ds_button . bv)) 2)) ;; bignum
              (write-long (ash (elt (_md_ds_button . bv) 0) 0) s)
              (write-long (ash (elt (_md_ds_button . bv) 1) -1) s))
             ((and (class _md_ds_button) (= (length (_md_ds_button . bv)) 1)) ;; big1
              (write-long (elt (_md_ds_button . bv) 0) s)
              (write-long (if (>= _md_ds_button 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _md_ds_button s)(write-long (if (>= _md_ds_button 0) 0 #xffffffff) s)))
     ;; float64 _rc_en_button
       (sys::poke _rc_en_button (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _start_stop
       (sys::poke _start_stop (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _x
#+(or :alpha :irix6 :x86_64)
      (setf _x (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y
#+(or :alpha :irix6 :x86_64)
      (setf _y (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _z
#+(or :alpha :irix6 :x86_64)
      (setf _z (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _z (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _rx
#+(or :alpha :irix6 :x86_64)
      (setf _rx (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _rx (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _ry
#+(or :alpha :irix6 :x86_64)
      (setf _ry (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _ry (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _rz
     (setq _rz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _j1
#+(or :alpha :irix6 :x86_64)
      (setf _j1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _j1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _j2
#+(or :alpha :irix6 :x86_64)
      (setf _j2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _j2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _j3
#+(or :alpha :irix6 :x86_64)
      (setf _j3 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _j3 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _j4
#+(or :alpha :irix6 :x86_64)
      (setf _j4 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _j4 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _j5
#+(or :alpha :irix6 :x86_64)
      (setf _j5 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _j5 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _j6
     (setq _j6 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _select_mode
#+(or :alpha :irix6 :x86_64)
      (setf _select_mode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _select_mode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _start
#+(or :alpha :irix6 :x86_64)
      (setf _start (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _start (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _forward_back
     (setq _forward_back (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _gear
#+(or :alpha :irix6 :x86_64)
      (setf _gear (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _gear (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _rs_button
#+(or :alpha :irix6 :x86_64)
      (setf _rs_button (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _rs_button (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _md_ds_button
#+(or :alpha :irix6 :x86_64)
      (setf _md_ds_button (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _md_ds_button (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _rc_en_button
     (setq _rc_en_button (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _start_stop
     (setq _start_stop (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get controller::rob_param :md5sum-) "0491c34928b4803b4c4b4d889e27e443")
(setf (get controller::rob_param :datatype-) "controller/rob_param")
(setf (get controller::rob_param :definition-)
      "int64 x
int64 y 
int64 z
int64 rx
int64 ry
float64 rz
int64 j1
int64 j2
int64 j3
int64 j4
int64 j5
float64 j6
int64 select_mode
int64 start
float64 forward_back
int64 gear
int64 rs_button
int64 md_ds_button
float64 rc_en_button
float64 start_stop

")



(provide :controller/rob_param "0491c34928b4803b4c4b4d889e27e443")


