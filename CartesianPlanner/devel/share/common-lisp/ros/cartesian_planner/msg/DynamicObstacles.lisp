; Auto-generated. Do not edit!


(cl:in-package cartesian_planner-msg)


;//! \htmlinclude DynamicObstacles.msg.html

(cl:defclass <DynamicObstacles> (roslisp-msg-protocol:ros-message)
  ((obstacles
    :reader obstacles
    :initarg :obstacles
    :type (cl:vector cartesian_planner-msg:DynamicObstacle)
   :initform (cl:make-array 0 :element-type 'cartesian_planner-msg:DynamicObstacle :initial-element (cl:make-instance 'cartesian_planner-msg:DynamicObstacle))))
)

(cl:defclass DynamicObstacles (<DynamicObstacles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DynamicObstacles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DynamicObstacles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cartesian_planner-msg:<DynamicObstacles> is deprecated: use cartesian_planner-msg:DynamicObstacles instead.")))

(cl:ensure-generic-function 'obstacles-val :lambda-list '(m))
(cl:defmethod obstacles-val ((m <DynamicObstacles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cartesian_planner-msg:obstacles-val is deprecated.  Use cartesian_planner-msg:obstacles instead.")
  (obstacles m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DynamicObstacles>) ostream)
  "Serializes a message object of type '<DynamicObstacles>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'obstacles))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'obstacles))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DynamicObstacles>) istream)
  "Deserializes a message object of type '<DynamicObstacles>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'obstacles) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'obstacles)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'cartesian_planner-msg:DynamicObstacle))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DynamicObstacles>)))
  "Returns string type for a message object of type '<DynamicObstacles>"
  "cartesian_planner/DynamicObstacles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DynamicObstacles)))
  "Returns string type for a message object of type 'DynamicObstacles"
  "cartesian_planner/DynamicObstacles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DynamicObstacles>)))
  "Returns md5sum for a message object of type '<DynamicObstacles>"
  "30b207dd63b971b7a3a28ba13f187795")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DynamicObstacles)))
  "Returns md5sum for a message object of type 'DynamicObstacles"
  "30b207dd63b971b7a3a28ba13f187795")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DynamicObstacles>)))
  "Returns full string definition for message of type '<DynamicObstacles>"
  (cl:format cl:nil "DynamicObstacle[] obstacles~%~%================================================================================~%MSG: cartesian_planner/DynamicObstacle~%geometry_msgs/Polygon polygon~%DynamicTrajectoryPoint[] trajectory~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: cartesian_planner/DynamicTrajectoryPoint~%float64 time~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DynamicObstacles)))
  "Returns full string definition for message of type 'DynamicObstacles"
  (cl:format cl:nil "DynamicObstacle[] obstacles~%~%================================================================================~%MSG: cartesian_planner/DynamicObstacle~%geometry_msgs/Polygon polygon~%DynamicTrajectoryPoint[] trajectory~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: cartesian_planner/DynamicTrajectoryPoint~%float64 time~%float64 x~%float64 y~%float64 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DynamicObstacles>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'obstacles) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DynamicObstacles>))
  "Converts a ROS message object to a list"
  (cl:list 'DynamicObstacles
    (cl:cons ':obstacles (obstacles msg))
))
