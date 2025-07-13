;; Auto-generated. Do not edit!


(when (boundp 'ac_control::qr_scanner)
  (if (not (find-package "AC_CONTROL"))
    (make-package "AC_CONTROL"))
  (shadow 'qr_scanner (find-package "AC_CONTROL")))
(unless (find-package "AC_CONTROL::QR_SCANNER")
  (make-package "AC_CONTROL::QR_SCANNER"))

(in-package "ROS")
;;//! \htmlinclude qr_scanner.msg.html


(defclass ac_control::qr_scanner
  :super ros::object
  :slots (_message _state _mode _position ))

(defmethod ac_control::qr_scanner
  (:init
   (&key
    ((:message __message) "")
    ((:state __state) nil)
    ((:mode __mode) 0)
    ((:position __position) "")
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _state __state)
   (setq _mode (round __mode))
   (setq _position (string __position))
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:state
   (&optional (__state :null))
   (if (not (eq __state :null)) (setq _state __state)) _state)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; bool _state
    1
    ;; int32 _mode
    4
    ;; string _position
    4 (length _position)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;; int32 _mode
       (write-long _mode s)
     ;; string _position
       (write-long (length _position) s) (princ _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _position
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _position (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ac_control::qr_scanner :md5sum-) "1f44d1ea4409fff8982b454001169518")
(setf (get ac_control::qr_scanner :datatype-) "ac_control/qr_scanner")
(setf (get ac_control::qr_scanner :definition-)
      "string message
bool state
int32 mode
string position
")



(provide :ac_control/qr_scanner "1f44d1ea4409fff8982b454001169518")


