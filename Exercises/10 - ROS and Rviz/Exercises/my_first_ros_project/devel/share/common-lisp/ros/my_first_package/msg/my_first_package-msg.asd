
(cl:in-package :asdf)

(defsystem "my_first_package-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "position" :depends-on ("_package_position"))
    (:file "_package_position" :depends-on ("_package"))
    (:file "values" :depends-on ("_package_values"))
    (:file "_package_values" :depends-on ("_package"))
  ))