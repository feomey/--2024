
(cl:in-package :asdf)

(defsystem "ac_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "qr_scanner" :depends-on ("_package_qr_scanner"))
    (:file "_package_qr_scanner" :depends-on ("_package"))
  ))