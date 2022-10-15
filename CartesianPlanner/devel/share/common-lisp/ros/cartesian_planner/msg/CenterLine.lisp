; Auto-generated. Do not edit!


(cl:in-package cartesian_planner-msg)


;//! \htmlinclude CenterLine.msg.html

(cl:defclass <CenterLine> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector cartesian_planner-msg:CenterLinePoint)
   :initform (cl:make-array 0 :element-type 'cartesian_planner-msg:CenterLinePoint :initial-element (cl:make-instance 'cartesian_planner-msg:CenterLinePoint))))
)

(cl:defclass CenterLine (<CenterLine>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CenterLine>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CenterLine)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartesian_planner-msg:<CenterLine> is deprecated: use cartesian_planner-msg:CenterLine instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <CenterLine>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartesian_planner-msg:points-val is deprecated.  Use cartesian_planner-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CenterLine>) ostream)
  "Serializes a message object of type '<CenterLine>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CenterLine>) istream)
  "Deserializes a message object of type '<CenterLine>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartesian_planner-msg:CenterLinePoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CenterLine>)))
  "Returns string type for a message object of type '<CenterLine>"
  "cartesian_planner/CenterLine")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CenterLine)))
  "Returns string type for a message object of type 'CenterLine"
  "cartesian_planner/CenterLine")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CenterLine>)))
  "Returns md5sum for a message object of type '<CenterLine>"
  "76f98b257c0b7d1218f95661429b5529")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CenterLine)))
  "Returns md5sum for a message object of type 'CenterLine"
  "76f98b257c0b7d1218f95661429b5529")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CenterLine>)))
  "Returns full string definition for message of type '<CenterLine>"
  (cl:format cl:nil "CenterLinePoint[] points~%~%================================================================================~%MSG: cartesian_planner/CenterLinePoint~%float64 s~%float64 x~%float64 y~%float64 theta~%float64 kappa~%float64 left_bound~%float64 right_bound~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CenterLine)))
  "Returns full string definition for message of type 'CenterLine"
  (cl:format cl:nil "CenterLinePoint[] points~%~%================================================================================~%MSG: cartesian_planner/CenterLinePoint~%float64 s~%float64 x~%float64 y~%float64 theta~%float64 kappa~%float64 left_bound~%float64 right_bound~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CenterLine>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CenterLine>))
  "Converts a ROS message object to a list"
  (cl:list 'CenterLine
    (cl:cons ':points (points msg))
))
