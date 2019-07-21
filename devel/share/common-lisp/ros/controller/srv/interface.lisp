; Auto-generated. Do not edit!


(cl:in-package controller-srv)


;//! \htmlinclude interface-request.msg.html

(cl:defclass <interface-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform ""))
)

(cl:defclass interface-request (<interface-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <interface-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'interface-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<interface-request> is deprecated: use controller-srv:interface-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <interface-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:cmd-val is deprecated.  Use controller-srv:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <interface-request>) ostream)
  "Serializes a message object of type '<interface-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <interface-request>) istream)
  "Deserializes a message object of type '<interface-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<interface-request>)))
  "Returns string type for a service object of type '<interface-request>"
  "controller/interfaceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'interface-request)))
  "Returns string type for a service object of type 'interface-request"
  "controller/interfaceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<interface-request>)))
  "Returns md5sum for a message object of type '<interface-request>"
  "5fd22aa1f159fb59b0d35e4ba0a4f558")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'interface-request)))
  "Returns md5sum for a message object of type 'interface-request"
  "5fd22aa1f159fb59b0d35e4ba0a4f558")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<interface-request>)))
  "Returns full string definition for message of type '<interface-request>"
  (cl:format cl:nil "string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'interface-request)))
  "Returns full string definition for message of type 'interface-request"
  (cl:format cl:nil "string cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <interface-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <interface-request>))
  "Converts a ROS message object to a list"
  (cl:list 'interface-request
    (cl:cons ':cmd (cmd msg))
))
;//! \htmlinclude interface-response.msg.html

(cl:defclass <interface-response> (roslisp-msg-protocol:ros-message)
  ((return_code
    :reader return_code
    :initarg :return_code
    :type cl:integer
    :initform 0))
)

(cl:defclass interface-response (<interface-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <interface-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'interface-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<interface-response> is deprecated: use controller-srv:interface-response instead.")))

(cl:ensure-generic-function 'return_code-val :lambda-list '(m))
(cl:defmethod return_code-val ((m <interface-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:return_code-val is deprecated.  Use controller-srv:return_code instead.")
  (return_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <interface-response>) ostream)
  "Serializes a message object of type '<interface-response>"
  (cl:let* ((signed (cl:slot-value msg 'return_code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <interface-response>) istream)
  "Deserializes a message object of type '<interface-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'return_code) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<interface-response>)))
  "Returns string type for a service object of type '<interface-response>"
  "controller/interfaceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'interface-response)))
  "Returns string type for a service object of type 'interface-response"
  "controller/interfaceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<interface-response>)))
  "Returns md5sum for a message object of type '<interface-response>"
  "5fd22aa1f159fb59b0d35e4ba0a4f558")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'interface-response)))
  "Returns md5sum for a message object of type 'interface-response"
  "5fd22aa1f159fb59b0d35e4ba0a4f558")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<interface-response>)))
  "Returns full string definition for message of type '<interface-response>"
  (cl:format cl:nil "int64 return_code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'interface-response)))
  "Returns full string definition for message of type 'interface-response"
  (cl:format cl:nil "int64 return_code~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <interface-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <interface-response>))
  "Converts a ROS message object to a list"
  (cl:list 'interface-response
    (cl:cons ':return_code (return_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'interface)))
  'interface-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'interface)))
  'interface-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'interface)))
  "Returns string type for a service object of type '<interface>"
  "controller/interface")