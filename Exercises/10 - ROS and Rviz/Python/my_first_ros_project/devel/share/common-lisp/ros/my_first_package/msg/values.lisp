; Auto-generated. Do not edit!


(cl:in-package my_first_package-msg)


;//! \htmlinclude values.msg.html

(cl:defclass <values> (roslisp-msg-protocol:ros-message)
  ((letter
    :reader letter
    :initarg :letter
    :type cl:integer
    :initform 0))
)

(cl:defclass values (<values>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <values>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'values)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_first_package-msg:<values> is deprecated: use my_first_package-msg:values instead.")))

(cl:ensure-generic-function 'letter-val :lambda-list '(m))
(cl:defmethod letter-val ((m <values>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_first_package-msg:letter-val is deprecated.  Use my_first_package-msg:letter instead.")
  (letter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <values>) ostream)
  "Serializes a message object of type '<values>"
  (cl:let* ((signed (cl:slot-value msg 'letter)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <values>) istream)
  "Deserializes a message object of type '<values>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'letter) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<values>)))
  "Returns string type for a message object of type '<values>"
  "my_first_package/values")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'values)))
  "Returns string type for a message object of type 'values"
  "my_first_package/values")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<values>)))
  "Returns md5sum for a message object of type '<values>"
  "0aff0824e905c6e4dad564004ff12fa0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'values)))
  "Returns md5sum for a message object of type 'values"
  "0aff0824e905c6e4dad564004ff12fa0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<values>)))
  "Returns full string definition for message of type '<values>"
  (cl:format cl:nil "int64 letter~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'values)))
  "Returns full string definition for message of type 'values"
  (cl:format cl:nil "int64 letter~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <values>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <values>))
  "Converts a ROS message object to a list"
  (cl:list 'values
    (cl:cons ':letter (letter msg))
))
