;; Auto-generated. Do not edit!


(when (boundp 'traj_gen::SplineGen)
  (if (not (find-package "TRAJ_GEN"))
    (make-package "TRAJ_GEN"))
  (shadow 'SplineGen (find-package "TRAJ_GEN")))
(unless (find-package "TRAJ_GEN::SPLINEGEN")
  (make-package "TRAJ_GEN::SPLINEGEN"))
(unless (find-package "TRAJ_GEN::SPLINEGENREQUEST")
  (make-package "TRAJ_GEN::SPLINEGENREQUEST"))
(unless (find-package "TRAJ_GEN::SPLINEGENRESPONSE")
  (make-package "TRAJ_GEN::SPLINEGENRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))




(defclass traj_gen::SplineGenRequest
  :super ros::object
  :slots (_v0 _knot _knot_t ))

(defmethod traj_gen::SplineGenRequest
  (:init
   (&key
    ((:v0 __v0) (instance geometry_msgs::Twist :init))
    ((:knot __knot) (instance nav_msgs::Path :init))
    ((:knot_t __knot_t) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _v0 __v0)
   (setq _knot __knot)
   (setq _knot_t __knot_t)
   self)
  (:v0
   (&rest __v0)
   (if (keywordp (car __v0))
       (send* _v0 __v0)
     (progn
       (if __v0 (setq _v0 (car __v0)))
       _v0)))
  (:knot
   (&rest __knot)
   (if (keywordp (car __knot))
       (send* _knot __knot)
     (progn
       (if __knot (setq _knot (car __knot)))
       _knot)))
  (:knot_t
   (&optional __knot_t)
   (if __knot_t (setq _knot_t __knot_t)) _knot_t)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Twist _v0
    (send _v0 :serialization-length)
    ;; nav_msgs/Path _knot
    (send _knot :serialization-length)
    ;; float64[] _knot_t
    (* 8    (length _knot_t)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Twist _v0
       (send _v0 :serialize s)
     ;; nav_msgs/Path _knot
       (send _knot :serialize s)
     ;; float64[] _knot_t
     (write-long (length _knot_t) s)
     (dotimes (i (length _knot_t))
       (sys::poke (elt _knot_t i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Twist _v0
     (send _v0 :deserialize buf ptr-) (incf ptr- (send _v0 :serialization-length))
   ;; nav_msgs/Path _knot
     (send _knot :deserialize buf ptr-) (incf ptr- (send _knot :serialization-length))
   ;; float64[] _knot_t
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _knot_t (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _knot_t i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass traj_gen::SplineGenResponse
  :super ros::object
  :slots (_spline_xyz ))

(defmethod traj_gen::SplineGenResponse
  (:init
   (&key
    ((:spline_xyz __spline_xyz) (instance traj_gen::PolySplineXYZ :init))
    )
   (send-super :init)
   (setq _spline_xyz __spline_xyz)
   self)
  (:spline_xyz
   (&rest __spline_xyz)
   (if (keywordp (car __spline_xyz))
       (send* _spline_xyz __spline_xyz)
     (progn
       (if __spline_xyz (setq _spline_xyz (car __spline_xyz)))
       _spline_xyz)))
  (:serialization-length
   ()
   (+
    ;; traj_gen/PolySplineXYZ _spline_xyz
    (send _spline_xyz :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; traj_gen/PolySplineXYZ _spline_xyz
       (send _spline_xyz :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; traj_gen/PolySplineXYZ _spline_xyz
     (send _spline_xyz :deserialize buf ptr-) (incf ptr- (send _spline_xyz :serialization-length))
   ;;
   self)
  )

(defclass traj_gen::SplineGen
  :super ros::object
  :slots ())

(setf (get traj_gen::SplineGen :md5sum-) "18f4637150c5208159e6299948387360")
(setf (get traj_gen::SplineGen :datatype-) "traj_gen/SplineGen")
(setf (get traj_gen::SplineGen :request) traj_gen::SplineGenRequest)
(setf (get traj_gen::SplineGen :response) traj_gen::SplineGenResponse)

(defmethod traj_gen::SplineGenRequest
  (:response () (instance traj_gen::SplineGenResponse :init)))

(setf (get traj_gen::SplineGenRequest :md5sum-) "18f4637150c5208159e6299948387360")
(setf (get traj_gen::SplineGenRequest :datatype-) "traj_gen/SplineGenRequest")
(setf (get traj_gen::SplineGenRequest :definition-)
      "geometry_msgs/Twist v0
nav_msgs/Path knot
float64[] knot_t

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
traj_gen/PolySplineXYZ spline_xyz


================================================================================
MSG: traj_gen/PolySplineXYZ
bool is_valid
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

(setf (get traj_gen::SplineGenResponse :md5sum-) "18f4637150c5208159e6299948387360")
(setf (get traj_gen::SplineGenResponse :datatype-) "traj_gen/SplineGenResponse")
(setf (get traj_gen::SplineGenResponse :definition-)
      "geometry_msgs/Twist v0
nav_msgs/Path knot
float64[] knot_t

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
traj_gen/PolySplineXYZ spline_xyz


================================================================================
MSG: traj_gen/PolySplineXYZ
bool is_valid
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



(provide :traj_gen/SplineGen "18f4637150c5208159e6299948387360")


