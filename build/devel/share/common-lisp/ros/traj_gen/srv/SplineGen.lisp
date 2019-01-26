; Auto-generated. Do not edit!


(cl:in-package traj_gen-srv)


;//! \htmlinclude SplineGen-request.msg.html

(cl:defclass <SplineGen-request> (roslisp-msg-protocol:ros-message)
  ((v0
    :reader v0
    :initarg :v0
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (knot
    :reader knot
    :initarg :knot
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path))
   (knot_t
    :reader knot_t
    :initarg :knot_t
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass SplineGen-request (<SplineGen-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SplineGen-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SplineGen-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_gen-srv:<SplineGen-request> is deprecated: use traj_gen-srv:SplineGen-request instead.")))

(cl:ensure-generic-function 'v0-val :lambda-list '(m))
(cl:defmethod v0-val ((m <SplineGen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-srv:v0-val is deprecated.  Use traj_gen-srv:v0 instead.")
  (v0 m))

(cl:ensure-generic-function 'knot-val :lambda-list '(m))
(cl:defmethod knot-val ((m <SplineGen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-srv:knot-val is deprecated.  Use traj_gen-srv:knot instead.")
  (knot m))

(cl:ensure-generic-function 'knot_t-val :lambda-list '(m))
(cl:defmethod knot_t-val ((m <SplineGen-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-srv:knot_t-val is deprecated.  Use traj_gen-srv:knot_t instead.")
  (knot_t m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SplineGen-request>) ostream)
  "Serializes a message object of type '<SplineGen-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'v0) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'knot) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'knot_t))))
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
   (cl:slot-value msg 'knot_t))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SplineGen-request>) istream)
  "Deserializes a message object of type '<SplineGen-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'v0) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'knot) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'knot_t) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'knot_t)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SplineGen-request>)))
  "Returns string type for a service object of type '<SplineGen-request>"
  "traj_gen/SplineGenRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SplineGen-request)))
  "Returns string type for a service object of type 'SplineGen-request"
  "traj_gen/SplineGenRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SplineGen-request>)))
  "Returns md5sum for a message object of type '<SplineGen-request>"
  "18f4637150c5208159e6299948387360")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SplineGen-request)))
  "Returns md5sum for a message object of type 'SplineGen-request"
  "18f4637150c5208159e6299948387360")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SplineGen-request>)))
  "Returns full string definition for message of type '<SplineGen-request>"
  (cl:format cl:nil "geometry_msgs/Twist v0~%nav_msgs/Path knot~%float64[] knot_t~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SplineGen-request)))
  "Returns full string definition for message of type 'SplineGen-request"
  (cl:format cl:nil "geometry_msgs/Twist v0~%nav_msgs/Path knot~%float64[] knot_t~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SplineGen-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'v0))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'knot))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'knot_t) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SplineGen-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SplineGen-request
    (cl:cons ':v0 (v0 msg))
    (cl:cons ':knot (knot msg))
    (cl:cons ':knot_t (knot_t msg))
))
;//! \htmlinclude SplineGen-response.msg.html

(cl:defclass <SplineGen-response> (roslisp-msg-protocol:ros-message)
  ((spline_xyz
    :reader spline_xyz
    :initarg :spline_xyz
    :type traj_gen-msg:PolySplineXYZ
    :initform (cl:make-instance 'traj_gen-msg:PolySplineXYZ)))
)

(cl:defclass SplineGen-response (<SplineGen-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SplineGen-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SplineGen-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name traj_gen-srv:<SplineGen-response> is deprecated: use traj_gen-srv:SplineGen-response instead.")))

(cl:ensure-generic-function 'spline_xyz-val :lambda-list '(m))
(cl:defmethod spline_xyz-val ((m <SplineGen-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader traj_gen-srv:spline_xyz-val is deprecated.  Use traj_gen-srv:spline_xyz instead.")
  (spline_xyz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SplineGen-response>) ostream)
  "Serializes a message object of type '<SplineGen-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'spline_xyz) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SplineGen-response>) istream)
  "Deserializes a message object of type '<SplineGen-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'spline_xyz) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SplineGen-response>)))
  "Returns string type for a service object of type '<SplineGen-response>"
  "traj_gen/SplineGenResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SplineGen-response)))
  "Returns string type for a service object of type 'SplineGen-response"
  "traj_gen/SplineGenResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SplineGen-response>)))
  "Returns md5sum for a message object of type '<SplineGen-response>"
  "18f4637150c5208159e6299948387360")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SplineGen-response)))
  "Returns md5sum for a message object of type 'SplineGen-response"
  "18f4637150c5208159e6299948387360")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SplineGen-response>)))
  "Returns full string definition for message of type '<SplineGen-response>"
  (cl:format cl:nil "traj_gen/PolySplineXYZ spline_xyz~%~%~%================================================================================~%MSG: traj_gen/PolySplineXYZ~%bool is_valid~%traj_gen/PolySpline spline_x~%traj_gen/PolySpline spline_y ~%traj_gen/PolySpline spline_z~%int8 n_seg~%int8 poly_order~%float64[] knot_time ~%~%================================================================================~%MSG: traj_gen/PolySpline~%traj_gen/PolyCoeff[] poly_coeff~%float64[] knot_time ~%int8 n_seg~%bool is_valid~%~%~%~%================================================================================~%MSG: traj_gen/PolyCoeff~%float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SplineGen-response)))
  "Returns full string definition for message of type 'SplineGen-response"
  (cl:format cl:nil "traj_gen/PolySplineXYZ spline_xyz~%~%~%================================================================================~%MSG: traj_gen/PolySplineXYZ~%bool is_valid~%traj_gen/PolySpline spline_x~%traj_gen/PolySpline spline_y ~%traj_gen/PolySpline spline_z~%int8 n_seg~%int8 poly_order~%float64[] knot_time ~%~%================================================================================~%MSG: traj_gen/PolySpline~%traj_gen/PolyCoeff[] poly_coeff~%float64[] knot_time ~%int8 n_seg~%bool is_valid~%~%~%~%================================================================================~%MSG: traj_gen/PolyCoeff~%float64[] coeff~%int8 poly_order~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SplineGen-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'spline_xyz))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SplineGen-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SplineGen-response
    (cl:cons ':spline_xyz (spline_xyz msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SplineGen)))
  'SplineGen-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SplineGen)))
  'SplineGen-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SplineGen)))
  "Returns string type for a service object of type '<SplineGen>"
  "traj_gen/SplineGen")