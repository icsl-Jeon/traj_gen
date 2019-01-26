;; Auto-generated. Do not edit!


(when (boundp 'traj_gen::PolySpline)
  (if (not (find-package "TRAJ_GEN"))
    (make-package "TRAJ_GEN"))
  (shadow 'PolySpline (find-package "TRAJ_GEN")))
(unless (find-package "TRAJ_GEN::POLYSPLINE")
  (make-package "TRAJ_GEN::POLYSPLINE"))

(in-package "ROS")
;;//! \htmlinclude PolySpline.msg.html


(defclass traj_gen::PolySpline
  :super ros::object
  :slots (_poly_coeff _knot_time _n_seg _is_valid ))

(defmethod traj_gen::PolySpline
  (:init
   (&key
    ((:poly_coeff __poly_coeff) (let (r) (dotimes (i 0) (push (instance traj_gen::PolyCoeff :init) r)) r))
    ((:knot_time __knot_time) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:n_seg __n_seg) 0)
    ((:is_valid __is_valid) nil)
    )
   (send-super :init)
   (setq _poly_coeff __poly_coeff)
   (setq _knot_time __knot_time)
   (setq _n_seg (round __n_seg))
   (setq _is_valid __is_valid)
   self)
  (:poly_coeff
   (&rest __poly_coeff)
   (if (keywordp (car __poly_coeff))
       (send* _poly_coeff __poly_coeff)
     (progn
       (if __poly_coeff (setq _poly_coeff (car __poly_coeff)))
       _poly_coeff)))
  (:knot_time
   (&optional __knot_time)
   (if __knot_time (setq _knot_time __knot_time)) _knot_time)
  (:n_seg
   (&optional __n_seg)
   (if __n_seg (setq _n_seg __n_seg)) _n_seg)
  (:is_valid
   (&optional __is_valid)
   (if __is_valid (setq _is_valid __is_valid)) _is_valid)
  (:serialization-length
   ()
   (+
    ;; traj_gen/PolyCoeff[] _poly_coeff
    (apply #'+ (send-all _poly_coeff :serialization-length)) 4
    ;; float64[] _knot_time
    (* 8    (length _knot_time)) 4
    ;; int8 _n_seg
    1
    ;; bool _is_valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; traj_gen/PolyCoeff[] _poly_coeff
     (write-long (length _poly_coeff) s)
     (dolist (elem _poly_coeff)
       (send elem :serialize s)
       )
     ;; float64[] _knot_time
     (write-long (length _knot_time) s)
     (dotimes (i (length _knot_time))
       (sys::poke (elt _knot_time i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _n_seg
       (write-byte _n_seg s)
     ;; bool _is_valid
       (if _is_valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; traj_gen/PolyCoeff[] _poly_coeff
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poly_coeff (let (r) (dotimes (i n) (push (instance traj_gen::PolyCoeff :init) r)) r))
     (dolist (elem- _poly_coeff)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float64[] _knot_time
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _knot_time (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _knot_time i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int8 _n_seg
     (setq _n_seg (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _n_seg 127) (setq _n_seg (- _n_seg 256)))
   ;; bool _is_valid
     (setq _is_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get traj_gen::PolySpline :md5sum-) "370db93bb3910f197622508a6a558d72")
(setf (get traj_gen::PolySpline :datatype-) "traj_gen/PolySpline")
(setf (get traj_gen::PolySpline :definition-)
      "traj_gen/PolyCoeff[] poly_coeff
float64[] knot_time 
int8 n_seg
bool is_valid



================================================================================
MSG: traj_gen/PolyCoeff
float64[] coeff
int8 poly_order



")



(provide :traj_gen/PolySpline "370db93bb3910f197622508a6a558d72")


