;; Auto-generated. Do not edit!


(when (boundp 'traj_gen::PolySplineXYZ)
  (if (not (find-package "TRAJ_GEN"))
    (make-package "TRAJ_GEN"))
  (shadow 'PolySplineXYZ (find-package "TRAJ_GEN")))
(unless (find-package "TRAJ_GEN::POLYSPLINEXYZ")
  (make-package "TRAJ_GEN::POLYSPLINEXYZ"))

(in-package "ROS")
;;//! \htmlinclude PolySplineXYZ.msg.html


(defclass traj_gen::PolySplineXYZ
  :super ros::object
  :slots (_is_valid _spline_x _spline_y _spline_z _n_seg _poly_order _knot_time ))

(defmethod traj_gen::PolySplineXYZ
  (:init
   (&key
    ((:is_valid __is_valid) nil)
    ((:spline_x __spline_x) (instance traj_gen::PolySpline :init))
    ((:spline_y __spline_y) (instance traj_gen::PolySpline :init))
    ((:spline_z __spline_z) (instance traj_gen::PolySpline :init))
    ((:n_seg __n_seg) 0)
    ((:poly_order __poly_order) 0)
    ((:knot_time __knot_time) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _is_valid __is_valid)
   (setq _spline_x __spline_x)
   (setq _spline_y __spline_y)
   (setq _spline_z __spline_z)
   (setq _n_seg (round __n_seg))
   (setq _poly_order (round __poly_order))
   (setq _knot_time __knot_time)
   self)
  (:is_valid
   (&optional __is_valid)
   (if __is_valid (setq _is_valid __is_valid)) _is_valid)
  (:spline_x
   (&rest __spline_x)
   (if (keywordp (car __spline_x))
       (send* _spline_x __spline_x)
     (progn
       (if __spline_x (setq _spline_x (car __spline_x)))
       _spline_x)))
  (:spline_y
   (&rest __spline_y)
   (if (keywordp (car __spline_y))
       (send* _spline_y __spline_y)
     (progn
       (if __spline_y (setq _spline_y (car __spline_y)))
       _spline_y)))
  (:spline_z
   (&rest __spline_z)
   (if (keywordp (car __spline_z))
       (send* _spline_z __spline_z)
     (progn
       (if __spline_z (setq _spline_z (car __spline_z)))
       _spline_z)))
  (:n_seg
   (&optional __n_seg)
   (if __n_seg (setq _n_seg __n_seg)) _n_seg)
  (:poly_order
   (&optional __poly_order)
   (if __poly_order (setq _poly_order __poly_order)) _poly_order)
  (:knot_time
   (&optional __knot_time)
   (if __knot_time (setq _knot_time __knot_time)) _knot_time)
  (:serialization-length
   ()
   (+
    ;; bool _is_valid
    1
    ;; traj_gen/PolySpline _spline_x
    (send _spline_x :serialization-length)
    ;; traj_gen/PolySpline _spline_y
    (send _spline_y :serialization-length)
    ;; traj_gen/PolySpline _spline_z
    (send _spline_z :serialization-length)
    ;; int8 _n_seg
    1
    ;; int8 _poly_order
    1
    ;; float64[] _knot_time
    (* 8    (length _knot_time)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_valid
       (if _is_valid (write-byte -1 s) (write-byte 0 s))
     ;; traj_gen/PolySpline _spline_x
       (send _spline_x :serialize s)
     ;; traj_gen/PolySpline _spline_y
       (send _spline_y :serialize s)
     ;; traj_gen/PolySpline _spline_z
       (send _spline_z :serialize s)
     ;; int8 _n_seg
       (write-byte _n_seg s)
     ;; int8 _poly_order
       (write-byte _poly_order s)
     ;; float64[] _knot_time
     (write-long (length _knot_time) s)
     (dotimes (i (length _knot_time))
       (sys::poke (elt _knot_time i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_valid
     (setq _is_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; traj_gen/PolySpline _spline_x
     (send _spline_x :deserialize buf ptr-) (incf ptr- (send _spline_x :serialization-length))
   ;; traj_gen/PolySpline _spline_y
     (send _spline_y :deserialize buf ptr-) (incf ptr- (send _spline_y :serialization-length))
   ;; traj_gen/PolySpline _spline_z
     (send _spline_z :deserialize buf ptr-) (incf ptr- (send _spline_z :serialization-length))
   ;; int8 _n_seg
     (setq _n_seg (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _n_seg 127) (setq _n_seg (- _n_seg 256)))
   ;; int8 _poly_order
     (setq _poly_order (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _poly_order 127) (setq _poly_order (- _poly_order 256)))
   ;; float64[] _knot_time
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _knot_time (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _knot_time i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get traj_gen::PolySplineXYZ :md5sum-) "cde8ea48996e4cbc3defb260ac40b943")
(setf (get traj_gen::PolySplineXYZ :datatype-) "traj_gen/PolySplineXYZ")
(setf (get traj_gen::PolySplineXYZ :definition-)
      "bool is_valid
traj_gen/PolySpline spline_x
traj_gen/PolySpline spline_y 
traj_gen/PolySpline spline_z
int8 n_seg
int8 poly_order
float64[] knot_time 

================================================================================
MSG: traj_gen/PolySpline
traj_gen/PolyCoeff[] poly_coeff
float64[] knot_time 
int8 n_seg
bool is_valid



================================================================================
MSG: traj_gen/PolyCoeff
float64[] coeff
int8 poly_order



")



(provide :traj_gen/PolySplineXYZ "cde8ea48996e4cbc3defb260ac40b943")


