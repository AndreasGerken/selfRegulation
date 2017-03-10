
(cl:in-package :asdf)

(defsystem "tiny_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "tinyIMU" :depends-on ("_package_tinyIMU"))
    (:file "_package_tinyIMU" :depends-on ("_package"))
    (:file "tinyVector" :depends-on ("_package_tinyVector"))
    (:file "_package_tinyVector" :depends-on ("_package"))
  ))