; Auto-generated. Do not edit!


(cl:in-package ac_control-srv)


;//! \htmlinclude visino_fcu-request.msg.html

(cl:defclass <visino_fcu-request> (roslisp-msg-protocol:ros-message)
  ((vision_flag
    :reader vision_flag
    :initarg :vision_flag
    :type cl:boolean
    :initform cl:nil)
   (fcu_flag
    :reader fcu_flag
    :initarg :fcu_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass visino_fcu-request (<visino_fcu-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <visino_fcu-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'visino_fcu-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ac_control-srv:<visino_fcu-request> is deprecated: use ac_control-srv:visino_fcu-request instead.")))

(cl:ensure-generic-function 'vision_flag-val :lambda-list '(m))
(cl:defmethod vision_flag-val ((m <visino_fcu-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-srv:vision_flag-val is deprecated.  Use ac_control-srv:vision_flag instead.")
  (vision_flag m))

(cl:ensure-generic-function 'fcu_flag-val :lambda-list '(m))
(cl:defmethod fcu_flag-val ((m <visino_fcu-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-srv:fcu_flag-val is deprecated.  Use ac_control-srv:fcu_flag instead.")
  (fcu_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <visino_fcu-request>) ostream)
  "Serializes a message object of type '<visino_fcu-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vision_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fcu_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <visino_fcu-request>) istream)
  "Deserializes a message object of type '<visino_fcu-request>"
    (cl:setf (cl:slot-value msg 'vision_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fcu_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<visino_fcu-request>)))
  "Returns string type for a service object of type '<visino_fcu-request>"
  "ac_control/visino_fcuRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'visino_fcu-request)))
  "Returns string type for a service object of type 'visino_fcu-request"
  "ac_control/visino_fcuRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<visino_fcu-request>)))
  "Returns md5sum for a message object of type '<visino_fcu-request>"
  "5e11117ad059078a9af4671921a90b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'visino_fcu-request)))
  "Returns md5sum for a message object of type 'visino_fcu-request"
  "5e11117ad059078a9af4671921a90b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<visino_fcu-request>)))
  "Returns full string definition for message of type '<visino_fcu-request>"
  (cl:format cl:nil "# 客户端请求时发送的两个数字~%bool vision_flag  # 请求中包含一个布尔值，表示视觉标志位~%bool fcu_flag  # 请求中包含一个布尔值，表示飞控标志位~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'visino_fcu-request)))
  "Returns full string definition for message of type 'visino_fcu-request"
  (cl:format cl:nil "# 客户端请求时发送的两个数字~%bool vision_flag  # 请求中包含一个布尔值，表示视觉标志位~%bool fcu_flag  # 请求中包含一个布尔值，表示飞控标志位~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <visino_fcu-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <visino_fcu-request>))
  "Converts a ROS message object to a list"
  (cl:list 'visino_fcu-request
    (cl:cons ':vision_flag (vision_flag msg))
    (cl:cons ':fcu_flag (fcu_flag msg))
))
;//! \htmlinclude visino_fcu-response.msg.html

(cl:defclass <visino_fcu-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (vision_flag
    :reader vision_flag
    :initarg :vision_flag
    :type cl:boolean
    :initform cl:nil)
   (fcu_flag
    :reader fcu_flag
    :initarg :fcu_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass visino_fcu-response (<visino_fcu-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <visino_fcu-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'visino_fcu-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ac_control-srv:<visino_fcu-response> is deprecated: use ac_control-srv:visino_fcu-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <visino_fcu-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-srv:success-val is deprecated.  Use ac_control-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'vision_flag-val :lambda-list '(m))
(cl:defmethod vision_flag-val ((m <visino_fcu-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-srv:vision_flag-val is deprecated.  Use ac_control-srv:vision_flag instead.")
  (vision_flag m))

(cl:ensure-generic-function 'fcu_flag-val :lambda-list '(m))
(cl:defmethod fcu_flag-val ((m <visino_fcu-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-srv:fcu_flag-val is deprecated.  Use ac_control-srv:fcu_flag instead.")
  (fcu_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <visino_fcu-response>) ostream)
  "Serializes a message object of type '<visino_fcu-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'vision_flag) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'fcu_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <visino_fcu-response>) istream)
  "Deserializes a message object of type '<visino_fcu-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'vision_flag) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'fcu_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<visino_fcu-response>)))
  "Returns string type for a service object of type '<visino_fcu-response>"
  "ac_control/visino_fcuResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'visino_fcu-response)))
  "Returns string type for a service object of type 'visino_fcu-response"
  "ac_control/visino_fcuResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<visino_fcu-response>)))
  "Returns md5sum for a message object of type '<visino_fcu-response>"
  "5e11117ad059078a9af4671921a90b5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'visino_fcu-response)))
  "Returns md5sum for a message object of type 'visino_fcu-response"
  "5e11117ad059078a9af4671921a90b5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<visino_fcu-response>)))
  "Returns full string definition for message of type '<visino_fcu-response>"
  (cl:format cl:nil "# 服务器响应发送的数据~%bool success       # 响应中包含一个布尔值，表示操作是否成功~%bool vision_flag  # 响应中包含一个布尔值，表示视觉标志位~%bool fcu_flag  # 响应中包含一个布尔值，表示飞控标志位~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'visino_fcu-response)))
  "Returns full string definition for message of type 'visino_fcu-response"
  (cl:format cl:nil "# 服务器响应发送的数据~%bool success       # 响应中包含一个布尔值，表示操作是否成功~%bool vision_flag  # 响应中包含一个布尔值，表示视觉标志位~%bool fcu_flag  # 响应中包含一个布尔值，表示飞控标志位~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <visino_fcu-response>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <visino_fcu-response>))
  "Converts a ROS message object to a list"
  (cl:list 'visino_fcu-response
    (cl:cons ':success (success msg))
    (cl:cons ':vision_flag (vision_flag msg))
    (cl:cons ':fcu_flag (fcu_flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'visino_fcu)))
  'visino_fcu-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'visino_fcu)))
  'visino_fcu-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'visino_fcu)))
  "Returns string type for a service object of type '<visino_fcu>"
  "ac_control/visino_fcu")