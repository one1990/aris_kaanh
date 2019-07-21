
(cl:in-package :asdf)

(defsystem "controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "motorpos" :depends-on ("_package_motorpos"))
    (:file "_package_motorpos" :depends-on ("_package"))
    (:file "pose" :depends-on ("_package_pose"))
    (:file "_package_pose" :depends-on ("_package"))
    (:file "rob_param" :depends-on ("_package_rob_param"))
    (:file "_package_rob_param" :depends-on ("_package"))
  ))