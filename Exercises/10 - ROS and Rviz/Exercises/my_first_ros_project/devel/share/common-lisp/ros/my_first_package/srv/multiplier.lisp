; Auto-generated. Do not edit!


(cl:in-package my_first_package-srv)


;//! \htmlinclude multiplier-request.msg.html

(cl:defclass <multiplier-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0)
   (b
    :reader b
    :initarg :b
    :type cl:integer
    :initform 0))
)

(cl:defclass multiplier-request (<multiplier-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <multiplier-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'multiplier-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_first_package-srv:<multiplier-request> is deprecated: use my_first_package-srv:multiplier-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <multiplier-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_first_package-srv:a-val is deprecated.  Use my_first_package-srv:a instead.")
  (a m))

(cl:ensure-generic-function 'b-val :lambda-list '(m))
(cl:defmethod b-val ((m <multiplier-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_first_package-srv:b-val is deprecated.  Use my_first_package-srv:b instead.")
  (b m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <multiplier-request>) ostream)
  "Serializes a message object of type '<multiplier-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'b)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <multiplier-request>) istream)
  "Deserializes a message object of type '<multiplier-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'b) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<multiplier-request>)))
  "Returns string type for a service object of type '<multiplier-request>"
  "my_first_package/multiplierRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'multiplier-request)))
  "Returns string type for a service object of type 'multiplier-request"
  "my_first_package/multiplierRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<multiplier-request>)))
  "Returns md5sum for a message object of type '<multiplier-request>"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'multiplier-request)))
  "Returns md5sum for a message object of type 'multiplier-request"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<multiplier-request>)))
  "Returns full string definition for message of type '<multiplier-request>"
  (cl:format cl:nil "# Inputs~%int32 a~%int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'multiplier-request)))
  "Returns full string definition for message of type 'multiplier-request"
  (cl:format cl:nil "# Inputs~%int32 a~%int32 b~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <multiplier-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <multiplier-request>))
  "Converts a ROS message object to a list"
  (cl:list 'multiplier-request
    (cl:cons ':a (a msg))
    (cl:cons ':b (b msg))
))
;//! \htmlinclude multiplier-response.msg.html

(cl:defclass <multiplier-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass multiplier-response (<multiplier-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <multiplier-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'multiplier-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_first_package-srv:<multiplier-response> is deprecated: use my_first_package-srv:multiplier-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <multiplier-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_first_package-srv:result-val is deprecated.  Use my_first_package-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <multiplier-response>) ostream)
  "Serializes a message object of type '<multiplier-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <multiplier-response>) istream)
  "Deserializes a message object of type '<multiplier-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<multiplier-response>)))
  "Returns string type for a service object of type '<multiplier-response>"
  "my_first_package/multiplierResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'multiplier-response)))
  "Returns string type for a service object of type 'multiplier-response"
  "my_first_package/multiplierResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<multiplier-response>)))
  "Returns md5sum for a message object of type '<multiplier-response>"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'multiplier-response)))
  "Returns md5sum for a message object of type 'multiplier-response"
  "a7d7d7065d45755acef7d4dcf908162a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<multiplier-response>)))
  "Returns full string definition for message of type '<multiplier-response>"
  (cl:format cl:nil "int32 result~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'multiplier-response)))
  "Returns full string definition for message of type 'multiplier-response"
  (cl:format cl:nil "int32 result~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <multiplier-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <multiplier-response>))
  "Converts a ROS message object to a list"
  (cl:list 'multiplier-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'multiplier)))
  'multiplier-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'multiplier)))
  'multiplier-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'multiplier)))
  "Returns string type for a service object of type '<multiplier>"
  "my_first_package/multiplier")