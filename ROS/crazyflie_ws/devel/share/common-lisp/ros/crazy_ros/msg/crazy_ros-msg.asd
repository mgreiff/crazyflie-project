
(cl:in-package :asdf)

(defsystem "crazy_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Num" :depends-on ("_package_Num"))
    (:file "_package_Num" :depends-on ("_package"))
    (:file "NumpyArrayFloat64" :depends-on ("_package_NumpyArrayFloat64"))
    (:file "_package_NumpyArrayFloat64" :depends-on ("_package"))
  ))