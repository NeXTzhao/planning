; Auto-generated. Do not edit!


(cl:in-package cartesian_planner-msg)


;//! \htmlinclude DynamicObstacle.msg.html

(cl:defclass <DynamicObstacle> (roslisp-msg-protocol:ros-message)
  ((polygon
    :reader polygon
    :initarg :polygon
    :type geometry_msgs-msg:Polygon
    :initform (cl:make-instance 'geometry_msgs-msg:Polygon))
   (trajectory
    :reader trajectory
    :initarg :trajectory
    :type (cl:vector cartesian_planner-msg:DynamicTrajectoryPoint)
   :initform (cl:make-array 0 :element-type 'cartesian_planner-msg:DynamicTrajectoryPoint :initial-element (cl:make-instance 'cartesian_planner-msg:DynamicTrajectoryPoint))))
)

(cl:defclass DynamicObstacle (<DynamicObstacle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicObstacle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicObstacle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartesian_planner-msg:<DynamicObstacle> is deprecated: use cartesian_planner-msg:DynamicObstacle instead.")))

(cl:ensure-generic-function 'polygon-val :lambda-list '(m))
(cl:defmethod polygon-val ((m <DynamicObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartesian_planner-msg:polygon-val is deprecated.  Use cartesian_planner-msg:polygon instead.")
  (polygon m))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <DynamicObstacle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartesian_planner-msg:trajectory-val is deprecated.  Use cartesian_planner-msg:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicObstacle>) ostream)
  "Serializes a message object of type '<DynamicObstacle>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'polygon) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'trajectory))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicObstacle>) istream)
  "Deserializes a message object of type '<DynamicObstacle>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'polygon) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartesian_planner-msg:DynamicTrajectoryPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicObstacle>)))
  "Returns string type for a message object of type '<DynamicObstacle>"
  "cartesian_planner/DynamicObstacle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicObstacle)))
  "Returns string type for a message object of type 'DynamicObstacle"
  "cartesian_planner/DynamicObstacle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicObstacle>)))
  "Returns md5sum for a message object of type '<DynamicObstacle>"
  "9cbe4735150eb164552f0f587e68e631")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicObstacle)))
  "Returns md5sum for a message object of type 'DynamicObstacle"
  "9cbe4735150eb164552f0f587e68e631")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicObstacle>)))
  "Returns full string definition for message of type '<DynamicObstacle>"
  (cl:format cl:nil "geometry_msgs/Polygon polygon~%DynamicTrajectoryPoint[] trajectory~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: cartesian_planner/DynamicTrajectoryPoint~%float64 time~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicObstacle)))
  "Returns full string definition for message of type 'DynamicObstacle"
  (cl:format cl:nil "geometry_msgs/Polygon polygon~%DynamicTrajectoryPoint[] trajectory~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: cartesian_planner/DynamicTrajectoryPoint~%float64 time~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicObstacle>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'polygon))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicObstacle>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicObstacle
    (cl:cons ':polygon (polygon msg))
    (cl:cons ':trajectory (trajectory msg))
))
