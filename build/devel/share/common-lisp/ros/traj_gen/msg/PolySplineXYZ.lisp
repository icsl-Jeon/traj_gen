; Auto-generated. Do not edit!


(cl:in-package traj_gen-msg)


;//! \htmlinclude PolySplineXYZ.msg.html

(cl:defclass <PolySplineXYZ> (roslisp-msg-protocol:ros-message)
  ((is_valid
    :reader is_valid
    :initarg :is_valid
    :type cl:boolean
    :initform cl:nil)
   (spline_x
    :reader spline_x
    :initarg :spline_x
    :type traj_gen-msg:PolySpline
    :initform (cl:make-instance 'traj_gen-msg:PolySpline))
   (spline_y
    :reader spline_y
    :initarg :spline_y
    :type traj_gen-msg:PolySpline
    :initform (cl:make-instance 'traj_gen-msg:PolySpline))
   (spline_z
    :reader spline_z
    :initarg :spline_z
    :type traj_gen-msg:PolySpline
    :initform (cl:make-instance 'traj_gen-msg:PolySpline))
   (n_seg
    :reader n_seg
    :initarg :n_seg
    :type cl:fixnum
    :initform 0)
   (poly_order
    :reader poly_order
    :initarg :poly_order
    :type cl:fixnum
    :initform 0)
   (knot_time
    :reader knot_time
    :initarg :knot_time
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass PolySplineXYZ (<PolySplineXYZ>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolySplineXYZ>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolySplineXYZ)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_gen-msg:<PolySplineXYZ> is deprecated: use traj_gen-msg:PolySplineXYZ instead.")))

(cl:ensure-generic-function 'is_valid-val :lambda-list '(m))
(cl:defmethod is_valid-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:is_valid-val is deprecated.  Use traj_gen-msg:is_valid instead.")
  (is_valid m))

(cl:ensure-generic-function 'spline_x-val :lambda-list '(m))
(cl:defmethod spline_x-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:spline_x-val is deprecated.  Use traj_gen-msg:spline_x instead.")
  (spline_x m))

(cl:ensure-generic-function 'spline_y-val :lambda-list '(m))
(cl:defmethod spline_y-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:spline_y-val is deprecated.  Use traj_gen-msg:spline_y instead.")
  (spline_y m))

(cl:ensure-generic-function 'spline_z-val :lambda-list '(m))
(cl:defmethod spline_z-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:spline_z-val is deprecated.  Use traj_gen-msg:spline_z instead.")
  (spline_z m))

(cl:ensure-generic-function 'n_seg-val :lambda-list '(m))
(cl:defmethod n_seg-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:n_seg-val is deprecated.  Use traj_gen-msg:n_seg instead.")
  (n_seg m))

(cl:ensure-generic-function 'poly_order-val :lambda-list '(m))
(cl:defmethod poly_order-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:poly_order-val is deprecated.  Use traj_gen-msg:poly_order instead.")
  (poly_order m))

(cl:ensure-generic-function 'knot_time-val :lambda-list '(m))
(cl:defmethod knot_time-val ((m <PolySplineXYZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:knot_time-val is deprecated.  Use traj_gen-msg:knot_time instead.")
  (knot_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolySplineXYZ>) ostream)
  "Serializes a message object of type '<PolySplineXYZ>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_valid) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'spline_x) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'spline_y) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'spline_z) ostream)
  (cl:let* ((signed (cl:slot-value msg 'n_seg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'poly_order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'knot_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'knot_time))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolySplineXYZ>) istream)
  "Deserializes a message object of type '<PolySplineXYZ>"
    (cl:setf (cl:slot-value msg 'is_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'spline_x) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'spline_y) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'spline_z) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_seg) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'poly_order) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'knot_time) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'knot_time)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolySplineXYZ>)))
  "Returns string type for a message object of type '<PolySplineXYZ>"
  "traj_gen/PolySplineXYZ")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolySplineXYZ)))
  "Returns string type for a message object of type 'PolySplineXYZ"
  "traj_gen/PolySplineXYZ")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolySplineXYZ>)))
  "Returns md5sum for a message object of type '<PolySplineXYZ>"
  "cde8ea48996e4cbc3defb260ac40b943")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolySplineXYZ)))
  "Returns md5sum for a message object of type 'PolySplineXYZ"
  "cde8ea48996e4cbc3defb260ac40b943")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolySplineXYZ>)))
  "Returns full string definition for message of type '<PolySplineXYZ>"
  (cl:format cl:nil "bool is_valid~%traj_gen/PolySpline spline_x~%traj_gen/PolySpline spline_y ~%traj_gen/PolySpline spline_z~%int8 n_seg~%int8 poly_order~%float64[] knot_time ~%~%================================================================================~%MSG: traj_gen/PolySpline~%traj_gen/PolyCoeff[] poly_coeff~%float64[] knot_time ~%int8 n_seg~%bool is_valid~%~%~%~%================================================================================~%MSG: traj_gen/PolyCoeff~%float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolySplineXYZ)))
  "Returns full string definition for message of type 'PolySplineXYZ"
  (cl:format cl:nil "bool is_valid~%traj_gen/PolySpline spline_x~%traj_gen/PolySpline spline_y ~%traj_gen/PolySpline spline_z~%int8 n_seg~%int8 poly_order~%float64[] knot_time ~%~%================================================================================~%MSG: traj_gen/PolySpline~%traj_gen/PolyCoeff[] poly_coeff~%float64[] knot_time ~%int8 n_seg~%bool is_valid~%~%~%~%================================================================================~%MSG: traj_gen/PolyCoeff~%float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolySplineXYZ>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'spline_x))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'spline_y))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'spline_z))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'knot_time) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolySplineXYZ>))
  "Converts a ROS message object to a list"
  (cl:list 'PolySplineXYZ
    (cl:cons ':is_valid (is_valid msg))
    (cl:cons ':spline_x (spline_x msg))
    (cl:cons ':spline_y (spline_y msg))
    (cl:cons ':spline_z (spline_z msg))
    (cl:cons ':n_seg (n_seg msg))
    (cl:cons ':poly_order (poly_order msg))
    (cl:cons ':knot_time (knot_time msg))
))
