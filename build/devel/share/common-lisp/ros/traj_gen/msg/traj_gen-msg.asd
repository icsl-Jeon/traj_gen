
(cl:in-package :asdf)

(defsystem "traj_gen-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PolyCoeff" :depends-on ("_package_PolyCoeff"))
    (:file "_package_PolyCoeff" :depends-on ("_package"))
    (:file "PolyCoeff" :depends-on ("_package_PolyCoeff"))
    (:file "_package_PolyCoeff" :depends-on ("_package"))
    (:file "PolySpline" :depends-on ("_package_PolySpline"))
    (:file "_package_PolySpline" :depends-on ("_package"))
    (:file "PolySpline" :depends-on ("_package_PolySpline"))
    (:file "_package_PolySpline" :depends-on ("_package"))
    (:file "PolySplineXYZ" :depends-on ("_package_PolySplineXYZ"))
    (:file "_package_PolySplineXYZ" :depends-on ("_package"))
    (:file "PolySplineXYZ" :depends-on ("_package_PolySplineXYZ"))
    (:file "_package_PolySplineXYZ" :depends-on ("_package"))
  ))