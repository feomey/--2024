;; Auto-generated. Do not edit!


(when (boundp 'ac_control::visino_fcu)
  (if (not (find-package "AC_CONTROL"))
    (make-package "AC_CONTROL"))
  (shadow 'visino_fcu (find-package "AC_CONTROL")))
(unless (find-package "AC_CONTROL::VISINO_FCU")
  (make-package "AC_CONTROL::VISINO_FCU"))
(unless (find-package "AC_CONTROL::VISINO_FCUREQUEST")
  (make-package "AC_CONTROL::VISINO_FCUREQUEST"))
(unless (find-package "AC_CONTROL::VISINO_FCURESPONSE")
  (make-package "AC_CONTROL::VISINO_FCURESPONSE"))

(in-package "ROS")





(defclass ac_control::visino_fcuRequest
  :super ros::object
  :slots (_vision_flag _fcu_flag ))

(defmethod ac_control::visino_fcuRequest
  (:init
   (&key
    ((:vision_flag __vision_flag) nil)
    ((:fcu_flag __fcu_flag) nil)
    )
   (send-super :init)
   (setq _vision_flag __vision_flag)
   (setq _fcu_flag __fcu_flag)
   self)
  (:vision_flag
   (&optional (__vision_flag :null))
   (if (not (eq __vision_flag :null)) (setq _vision_flag __vision_flag)) _vision_flag)
  (:fcu_flag
   (&optional (__fcu_flag :null))
   (if (not (eq __fcu_flag :null)) (setq _fcu_flag __fcu_flag)) _fcu_flag)
  (:serialization-length
   ()
   (+
    ;; bool _vision_flag
    1
    ;; bool _fcu_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _vision_flag
       (if _vision_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _fcu_flag
       (if _fcu_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _vision_flag
     (setq _vision_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fcu_flag
     (setq _fcu_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ac_control::visino_fcuResponse
  :super ros::object
  :slots (_success _vision_flag _fcu_flag ))

(defmethod ac_control::visino_fcuResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:vision_flag __vision_flag) nil)
    ((:fcu_flag __fcu_flag) nil)
    )
   (send-super :init)
   (setq _success __success)
   (setq _vision_flag __vision_flag)
   (setq _fcu_flag __fcu_flag)
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:vision_flag
   (&optional (__vision_flag :null))
   (if (not (eq __vision_flag :null)) (setq _vision_flag __vision_flag)) _vision_flag)
  (:fcu_flag
   (&optional (__fcu_flag :null))
   (if (not (eq __fcu_flag :null)) (setq _fcu_flag __fcu_flag)) _fcu_flag)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; bool _vision_flag
    1
    ;; bool _fcu_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; bool _vision_flag
       (if _vision_flag (write-byte -1 s) (write-byte 0 s))
     ;; bool _fcu_flag
       (if _fcu_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _vision_flag
     (setq _vision_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _fcu_flag
     (setq _fcu_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ac_control::visino_fcu
  :super ros::object
  :slots ())

(setf (get ac_control::visino_fcu :md5sum-) "5e11117ad059078a9af4671921a90b5c")
(setf (get ac_control::visino_fcu :datatype-) "ac_control/visino_fcu")
(setf (get ac_control::visino_fcu :request) ac_control::visino_fcuRequest)
(setf (get ac_control::visino_fcu :response) ac_control::visino_fcuResponse)

(defmethod ac_control::visino_fcuRequest
  (:response () (instance ac_control::visino_fcuResponse :init)))

(setf (get ac_control::visino_fcuRequest :md5sum-) "5e11117ad059078a9af4671921a90b5c")
(setf (get ac_control::visino_fcuRequest :datatype-) "ac_control/visino_fcuRequest")
(setf (get ac_control::visino_fcuRequest :definition-)
      "# 客户端请求时发送的两个数字
bool vision_flag  # 请求中包含一个布尔值，表示视觉标志位
bool fcu_flag  # 请求中包含一个布尔值，表示飞控标志位
---
# 服务器响应发送的数据
bool success       # 响应中包含一个布尔值，表示操作是否成功
bool vision_flag  # 响应中包含一个布尔值，表示视觉标志位
bool fcu_flag  # 响应中包含一个布尔值，表示飞控标志位
")

(setf (get ac_control::visino_fcuResponse :md5sum-) "5e11117ad059078a9af4671921a90b5c")
(setf (get ac_control::visino_fcuResponse :datatype-) "ac_control/visino_fcuResponse")
(setf (get ac_control::visino_fcuResponse :definition-)
      "# 客户端请求时发送的两个数字
bool vision_flag  # 请求中包含一个布尔值，表示视觉标志位
bool fcu_flag  # 请求中包含一个布尔值，表示飞控标志位
---
# 服务器响应发送的数据
bool success       # 响应中包含一个布尔值，表示操作是否成功
bool vision_flag  # 响应中包含一个布尔值，表示视觉标志位
bool fcu_flag  # 响应中包含一个布尔值，表示飞控标志位
")



(provide :ac_control/visino_fcu "5e11117ad059078a9af4671921a90b5c")


