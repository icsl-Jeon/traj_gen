; Auto-generated. Do not edit!


(cl:in-package traj_gen-msg)


;//! \htmlinclude PolyCoeff.msg.html

(cl:defclass <PolyCoeff> (roslisp-msg-protocol:ros-message)
  ((coeff
    :reader coeff
    :initarg :coeff
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (poly_order
    :reader poly_order
    :initarg :poly_order
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PolyCoeff (<PolyCoeff>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PolyCoeff>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PolyCoeff)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_gen-msg:<PolyCoeff> is deprecated: use traj_gen-msg:PolyCoeff instead.")))

(cl:ensure-generic-function 'coeff-val :lambda-list '(m))
(cl:defmethod coeff-val ((m <PolyCoeff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:coeff-val is deprecated.  Use traj_gen-msg:coeff instead.")
  (coeff m))

(cl:ensure-generic-function 'poly_order-val :lambda-list '(m))
(cl:defmethod poly_order-val ((m <PolyCoeff>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-msg:poly_order-val is deprecated.  Use traj_gen-msg:poly_order instead.")
  (poly_order m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PolyCoeff>) ostream)
  "Serializes a message object of type '<PolyCoeff>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coeff))))
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
   (cl:slot-value msg 'coeff))
  (cl:let* ((signed (cl:slot-value msg 'poly_order)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PolyCoeff>) istream)
  "Deserializes a message object of type '<PolyCoeff>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coeff) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coeff)))
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
      (cl:setf (cl:slot-value msg 'poly_order) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PolyCoeff>)))
  "Returns string type for a message object of type '<PolyCoeff>"
  "traj_gen/PolyCoeff")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PolyCoeff)))
  "Returns string type for a message object of type 'PolyCoeff"
  "traj_gen/PolyCoeff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PolyCoeff>)))
  "Returns md5sum for a message object of type '<PolyCoeff>"
  "eced8596b6a1e1e44efd431704fe5562")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PolyCoeff)))
  "Returns md5sum for a message object of type 'PolyCoeff"
  "eced8596b6a1e1e44efd431704fe5562")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PolyCoeff>)))
  "Returns full string definition for message of type '<PolyCoeff>"
  (cl:format cl:nil "float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PolyCoeff)))
  "Returns full string definition for message of type 'PolyCoeff"
  (cl:format cl:nil "float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PolyCoeff>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coeff) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PolyCoeff>))
  "Converts a ROS message object to a list"
  (cl:list 'PolyCoeff
    (cl:cons ':coeff (coeff msg))
    (cl:cons ':poly_order (poly_order msg))
))
