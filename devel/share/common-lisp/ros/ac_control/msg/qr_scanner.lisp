; Auto-generated. Do not edit!


(cl:in-package ac_control-msg)


;//! \htmlinclude qr_scanner.msg.html

(cl:defclass <qr_scanner> (roslisp-msg-protocol:ros-message)
  ((message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:string
    :initform ""))
)

(cl:defclass qr_scanner (<qr_scanner>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <qr_scanner>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'qr_scanner)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ac_control-msg:<qr_scanner> is deprecated: use ac_control-msg:qr_scanner instead.")))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <qr_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-msg:message-val is deprecated.  Use ac_control-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <qr_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-msg:state-val is deprecated.  Use ac_control-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <qr_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-msg:mode-val is deprecated.  Use ac_control-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <qr_scanner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ac_control-msg:position-val is deprecated.  Use ac_control-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <qr_scanner>) ostream)
  "Serializes a message object of type '<qr_scanner>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'position))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <qr_scanner>) istream)
  "Deserializes a message object of type '<qr_scanner>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'position) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<qr_scanner>)))
  "Returns string type for a message object of type '<qr_scanner>"
  "ac_control/qr_scanner")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'qr_scanner)))
  "Returns string type for a message object of type 'qr_scanner"
  "ac_control/qr_scanner")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<qr_scanner>)))
  "Returns md5sum for a message object of type '<qr_scanner>"
  "1f44d1ea4409fff8982b454001169518")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'qr_scanner)))
  "Returns md5sum for a message object of type 'qr_scanner"
  "1f44d1ea4409fff8982b454001169518")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<qr_scanner>)))
  "Returns full string definition for message of type '<qr_scanner>"
  (cl:format cl:nil "string message~%bool state~%int32 mode~%string position~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'qr_scanner)))
  "Returns full string definition for message of type 'qr_scanner"
  (cl:format cl:nil "string message~%bool state~%int32 mode~%string position~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <qr_scanner>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'message))
     1
     4
     4 (cl:length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <qr_scanner>))
  "Converts a ROS message object to a list"
  (cl:list 'qr_scanner
    (cl:cons ':message (message msg))
    (cl:cons ':state (state msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':position (position msg))
))
