; Auto-generated. Do not edit!


(cl:in-package controller-msg)


;//! \htmlinclude pose.msg.html

(cl:defclass <pose> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type cl:string
    :initform ""))
)

(cl:defclass pose (<pose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name controller-msg:<pose> is deprecated: use controller-msg:pose instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <pose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader controller-msg:pose-val is deprecated.  Use controller-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pose>) ostream)
  "Serializes a message object of type '<pose>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pose))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pose))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pose>) istream)
  "Deserializes a message object of type '<pose>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pose) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pose) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pose>)))
  "Returns string type for a message object of type '<pose>"
  "controller/pose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pose)))
  "Returns string type for a message object of type 'pose"
  "controller/pose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pose>)))
  "Returns md5sum for a message object of type '<pose>"
  "0cbc29f91adcb71214b885e1c0073aa7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pose)))
  "Returns md5sum for a message object of type 'pose"
  "0cbc29f91adcb71214b885e1c0073aa7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pose>)))
  "Returns full string definition for message of type '<pose>"
  (cl:format cl:nil "string pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pose)))
  "Returns full string definition for message of type 'pose"
  (cl:format cl:nil "string pose~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pose>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pose>))
  "Converts a ROS message object to a list"
  (cl:list 'pose
    (cl:cons ':pose (pose msg))
))
