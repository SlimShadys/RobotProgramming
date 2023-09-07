
(cl:in-package :asdf)

(defsystem "my_first_package-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "multiplier" :depends-on ("_package_multiplier"))
    (:file "_package_multiplier" :depends-on ("_package"))
  ))