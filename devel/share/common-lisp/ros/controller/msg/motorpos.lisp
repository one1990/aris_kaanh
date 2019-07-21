; Auto-generated. Do not edit!


(cl:in-package controller-msg)


;//! \htmlinclude motorpos.msg.html

(cl:defclass <motorpos> (roslisp-msg-protocol:ros-message)
  ((angular
    :reader angular
    :initarg :angular
    :type cl:fixnum
    :initform 0))
)

(cl:defclass motorpos (<motorpos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorpos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorpos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-msg:<motorpos> is deprecated: use controller-msg:motorpos instead.")))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <motorpos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-msg:angular-val is deprecated.  Use controller-msg:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorpos>) ostream)
  "Serializes a message object of type '<motorpos>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angular)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angular)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorpos>) istream)
  "Deserializes a message object of type '<motorpos>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'angular)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'angular)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorpos>)))
  "Returns string type for a message object of type '<motorpos>"
  "controller/motorpos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorpos)))
  "Returns string type for a message object of type 'motorpos"
  "controller/motorpos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorpos>)))
  "Returns md5sum for a message object of type '<motorpos>"
  "620daad8e7540d60806fb147811e141d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorpos)))
  "Returns md5sum for a message object of type 'motorpos"
  "620daad8e7540d60806fb147811e141d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorpos>)))
  "Returns full string definition for message of type '<motorpos>"
  (cl:format cl:nil "uint16 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorpos)))
  "Returns full string definition for message of type 'motorpos"
  (cl:format cl:nil "uint16 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorpos>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorpos>))
  "Converts a ROS message object to a list"
  (cl:list 'motorpos
    (cl:cons ':angular (angular msg))
))
