;; Auto-generated. Do not edit!


(when (boundp 'traj_gen::PolyCoeff)
  (if (not (find-package "TRAJ_GEN"))
    (make-package "TRAJ_GEN"))
  (shadow 'PolyCoeff (find-package "TRAJ_GEN")))
(unless (find-package "TRAJ_GEN::POLYCOEFF")
  (make-package "TRAJ_GEN::POLYCOEFF"))

(in-package "ROS")
;;//! \htmlinclude PolyCoeff.msg.html


(defclass traj_gen::PolyCoeff
  :super ros::object
  :slots (_coeff _poly_order ))

(defmethod traj_gen::PolyCoeff
  (:init
   (&key
    ((:coeff __coeff) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:poly_order __poly_order) 0)
    )
   (send-super :init)
   (setq _coeff __coeff)
   (setq _poly_order (round __poly_order))
   self)
  (:coeff
   (&optional __coeff)
   (if __coeff (setq _coeff __coeff)) _coeff)
  (:poly_order
   (&optional __poly_order)
   (if __poly_order (setq _poly_order __poly_order)) _poly_order)
  (:serialization-length
   ()
   (+
    ;; float64[] _coeff
    (* 8    (length _coeff)) 4
    ;; int8 _poly_order
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _coeff
     (write-long (length _coeff) s)
     (dotimes (i (length _coeff))
       (sys::poke (elt _coeff i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int8 _poly_order
       (write-byte _poly_order s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _coeff
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coeff (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _coeff i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int8 _poly_order
     (setq _poly_order (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _poly_order 127) (setq _poly_order (- _poly_order 256)))
   ;;
   self)
  )

(setf (get traj_gen::PolyCoeff :md5sum-) "eced8596b6a1e1e44efd431704fe5562")
(setf (get traj_gen::PolyCoeff :datatype-) "traj_gen/PolyCoeff")
(setf (get traj_gen::PolyCoeff :definition-)
      "float64[] coeff
int8 poly_order



")



(provide :traj_gen/PolyCoeff "eced8596b6a1e1e44efd431704fe5562")


