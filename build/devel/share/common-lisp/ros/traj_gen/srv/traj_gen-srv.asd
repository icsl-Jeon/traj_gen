
(cl:in-package :asdf)

(defsystem "traj_gen-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :nav_msgs-msg
               :traj_gen-msg
)
  :components ((:file "_package")
    (:file "SplineGen" :depends-on ("_package_SplineGen"))
    (:file "_package_SplineGen" :depends-on ("_package"))
  ))