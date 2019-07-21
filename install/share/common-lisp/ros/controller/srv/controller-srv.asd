
(cl:in-package :asdf)

(defsystem "controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "interface" :depends-on ("_package_interface"))
    (:file "_package_interface" :depends-on ("_package"))
  ))