; Auto-generated. Do not edit!


(cl:in-package controller-srv)


;//! \htmlinclude setpos-request.msg.html

(cl:defclass <setpos-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0))
)

(cl:defclass setpos-request (<setpos-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setpos-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setpos-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<setpos-request> is deprecated: use controller-srv:setpos-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <setpos-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:cmd-val is deprecated.  Use controller-srv:cmd instead.")
  (cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setpos-request>) ostream)
  "Serializes a message object of type '<setpos-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cmd)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setpos-request>) istream)
  "Deserializes a message object of type '<setpos-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cmd)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setpos-request>)))
  "Returns string type for a service object of type '<setpos-request>"
  "controller/setposRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setpos-request)))
  "Returns string type for a service object of type 'setpos-request"
  "controller/setposRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setpos-request>)))
  "Returns md5sum for a message object of type '<setpos-request>"
  "b7d0a4a753fe0e11a1400076e4574484")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setpos-request)))
  "Returns md5sum for a message object of type 'setpos-request"
  "b7d0a4a753fe0e11a1400076e4574484")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setpos-request>)))
  "Returns full string definition for message of type '<setpos-request>"
  (cl:format cl:nil "uint16 cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setpos-request)))
  "Returns full string definition for message of type 'setpos-request"
  (cl:format cl:nil "uint16 cmd~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setpos-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setpos-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setpos-request
    (cl:cons ':cmd (cmd msg))
))
;//! \htmlinclude setpos-response.msg.html

(cl:defclass <setpos-response> (roslisp-msg-protocol:ros-message)
  ((return_code
    :reader return_code
    :initarg :return_code
    :type cl:integer
    :initform 0))
)

(cl:defclass setpos-response (<setpos-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setpos-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setpos-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-srv:<setpos-response> is deprecated: use controller-srv:setpos-response instead.")))

(cl:ensure-generic-function 'return_code-val :lambda-list '(m))
(cl:defmethod return_code-val ((m <setpos-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-srv:return_code-val is deprecated.  Use controller-srv:return_code instead.")
  (return_code m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setpos-response>) ostream)
  "Serializes a message object of type '<setpos-response>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setpos-response>) istream)
  "Deserializes a message object of type '<setpos-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setpos-response>)))
  "Returns string type for a service object of type '<setpos-response>"
  "controller/setposResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setpos-response)))
  "Returns string type for a service object of type 'setpos-response"
  "controller/setposResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setpos-response>)))
  "Returns md5sum for a message object of type '<setpos-response>"
  "b7d0a4a753fe0e11a1400076e4574484")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setpos-response)))
  "Returns md5sum for a message object of type 'setpos-response"
  "b7d0a4a753fe0e11a1400076e4574484")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setpos-response>)))
  "Returns full string definition for message of type '<setpos-response>"
  (cl:format cl:nil "int64 return_code~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setpos-response)))
  "Returns full string definition for message of type 'setpos-response"
  (cl:format cl:nil "int64 return_code~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setpos-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setpos-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setpos-response
    (cl:cons ':return_code (return_code msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setpos)))
  'setpos-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setpos)))
  'setpos-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setpos)))
  "Returns string type for a service object of type '<setpos>"
  "controller/setpos")