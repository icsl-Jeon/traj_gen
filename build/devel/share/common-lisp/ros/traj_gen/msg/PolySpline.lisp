; Auto-generated. Do not edit!


(cl:in-package traj_gen-msg)


;//! \htmlinclude PolySpline.msg.html

(cl:defclass <PolySpline> (roslisp-msg-protocol:ros-message)
  ((poly_coeff
    :reader poly_coeff
    :initarg :poly_coeff
    :type (cl:vector traj_gen-msg:PolyCoeff)
   :initform (cl:make-array 0 :element-type 'traj_gen-msg:PolyCoeff :initial-element (cl:make-instance 'traj_gen-msg:PolyCoeff)))
   (knot_time
    :reader knot_time
    :initarg :knot_time
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (n_seg
    :reader n_seg
    :initarg :n_seg
    :type cl:fixnum
    :initform 0)
   (is_valid
    :reader is_valid
    :initarg :is_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass PolySpline (<PolySpline>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolySpline>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolySpline)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_gen-msg:<PolySpline> is deprecated: use traj_gen-msg:PolySpline instead.")))

(cl:ensure-generic-function 'poly_coeff-val :lambda-list '(m))
(cl:defmethod poly_coeff-val ((m <PolySpline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:poly_coeff-val is deprecated.  Use traj_gen-msg:poly_coeff instead.")
  (poly_coeff m))

(cl:ensure-generic-function 'knot_time-val :lambda-list '(m))
(cl:defmethod knot_time-val ((m <PolySpline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:knot_time-val is deprecated.  Use traj_gen-msg:knot_time instead.")
  (knot_time m))

(cl:ensure-generic-function 'n_seg-val :lambda-list '(m))
(cl:defmethod n_seg-val ((m <PolySpline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:n_seg-val is deprecated.  Use traj_gen-msg:n_seg instead.")
  (n_seg m))

(cl:ensure-generic-function 'is_valid-val :lambda-list '(m))
(cl:defmethod is_valid-val ((m <PolySpline>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:is_valid-val is deprecated.  Use traj_gen-msg:is_valid instead.")
  (is_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolySpline>) ostream)
  "Serializes a message object of type '<PolySpline>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poly_coeff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poly_coeff))
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
  (cl:let* ((signed (cl:slot-value msg 'n_seg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolySpline>) istream)
  "Deserializes a message object of type '<PolySpline>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poly_coeff) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poly_coeff)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'traj_gen-msg:PolyCoeff))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_seg) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'is_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolySpline>)))
  "Returns string type for a message object of type '<PolySpline>"
  "traj_gen/PolySpline")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolySpline)))
  "Returns string type for a message object of type 'PolySpline"
  "traj_gen/PolySpline")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolySpline>)))
  "Returns md5sum for a message object of type '<PolySpline>"
  "370db93bb3910f197622508a6a558d72")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolySpline)))
  "Returns md5sum for a message object of type 'PolySpline"
  "370db93bb3910f197622508a6a558d72")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolySpline>)))
  "Returns full string definition for message of type '<PolySpline>"
  (cl:format cl:nil "traj_gen/PolyCoeff[] poly_coeff~%float64[] knot_time ~%int8 n_seg~%bool is_valid~%~%~%~%================================================================================~%MSG: traj_gen/PolyCoeff~%float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolySpline)))
  "Returns full string definition for message of type 'PolySpline"
  (cl:format cl:nil "traj_gen/PolyCoeff[] poly_coeff~%float64[] knot_time ~%int8 n_seg~%bool is_valid~%~%~%~%================================================================================~%MSG: traj_gen/PolyCoeff~%float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolySpline>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poly_coeff) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'knot_time) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolySpline>))
  "Converts a ROS message object to a list"
  (cl:list 'PolySpline
    (cl:cons ':poly_coeff (poly_coeff msg))
    (cl:cons ':knot_time (knot_time msg))
    (cl:cons ':n_seg (n_seg msg))
    (cl:cons ':is_valid (is_valid msg))
))
