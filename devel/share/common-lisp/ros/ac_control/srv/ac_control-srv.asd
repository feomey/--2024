
(cl:in-package :asdf)

(defsystem "ac_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "visino_fcu" :depends-on ("_package_visino_fcu"))
    (:file "_package_visino_fcu" :depends-on ("_package"))
  ))