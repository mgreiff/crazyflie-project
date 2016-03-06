; Auto-generated. Do not edit!


(cl:in-package crazy_ros-msg)


;//! \htmlinclude NumpyArrayFloat64.msg.html

(cl:defclass <NumpyArrayFloat64> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass NumpyArrayFloat64 (<NumpyArrayFloat64>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NumpyArrayFloat64>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NumpyArrayFloat64)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name crazy_ros-msg:<NumpyArrayFloat64> is deprecated: use crazy_ros-msg:NumpyArrayFloat64 instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <NumpyArrayFloat64>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader crazy_ros-msg:data-val is deprecated.  Use crazy_ros-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NumpyArrayFloat64>) ostream)
  "Serializes a message object of type '<NumpyArrayFloat64>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NumpyArrayFloat64>) istream)
  "Deserializes a message object of type '<NumpyArrayFloat64>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NumpyArrayFloat64>)))
  "Returns string type for a message object of type '<NumpyArrayFloat64>"
  "crazy_ros/NumpyArrayFloat64")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NumpyArrayFloat64)))
  "Returns string type for a message object of type 'NumpyArrayFloat64"
  "crazy_ros/NumpyArrayFloat64")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NumpyArrayFloat64>)))
  "Returns md5sum for a message object of type '<NumpyArrayFloat64>"
  "788898178a3da2c3718461eecda8f714")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NumpyArrayFloat64)))
  "Returns md5sum for a message object of type 'NumpyArrayFloat64"
  "788898178a3da2c3718461eecda8f714")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NumpyArrayFloat64>)))
  "Returns full string definition for message of type '<NumpyArrayFloat64>"
  (cl:format cl:nil "float64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NumpyArrayFloat64)))
  "Returns full string definition for message of type 'NumpyArrayFloat64"
  (cl:format cl:nil "float64[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NumpyArrayFloat64>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NumpyArrayFloat64>))
  "Converts a ROS message object to a list"
  (cl:list 'NumpyArrayFloat64
    (cl:cons ':data (data msg))
))
