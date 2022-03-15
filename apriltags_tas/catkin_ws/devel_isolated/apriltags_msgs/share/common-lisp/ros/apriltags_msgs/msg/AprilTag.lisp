; Auto-generated. Do not edit!


(cl:in-package apriltags_msgs-msg)


;//! \htmlinclude AprilTag.msg.html

(cl:defclass <AprilTag> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (corners_px
    :reader corners_px
    :initarg :corners_px
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point)))
   (pose_3d
    :reader pose_3d
    :initarg :pose_3d
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (pose_valid
    :reader pose_valid
    :initarg :pose_valid
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass AprilTag (<AprilTag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AprilTag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AprilTag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name apriltags_msgs-msg:<AprilTag> is deprecated: use apriltags_msgs-msg:AprilTag instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AprilTag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags_msgs-msg:id-val is deprecated.  Use apriltags_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'corners_px-val :lambda-list '(m))
(cl:defmethod corners_px-val ((m <AprilTag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags_msgs-msg:corners_px-val is deprecated.  Use apriltags_msgs-msg:corners_px instead.")
  (corners_px m))

(cl:ensure-generic-function 'pose_3d-val :lambda-list '(m))
(cl:defmethod pose_3d-val ((m <AprilTag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags_msgs-msg:pose_3d-val is deprecated.  Use apriltags_msgs-msg:pose_3d instead.")
  (pose_3d m))

(cl:ensure-generic-function 'pose_valid-val :lambda-list '(m))
(cl:defmethod pose_valid-val ((m <AprilTag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader apriltags_msgs-msg:pose_valid-val is deprecated.  Use apriltags_msgs-msg:pose_valid instead.")
  (pose_valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AprilTag>) ostream)
  "Serializes a message object of type '<AprilTag>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'corners_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'corners_px))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_3d) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose_valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AprilTag>) istream)
  "Deserializes a message object of type '<AprilTag>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'corners_px) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'corners_px)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_3d) istream)
    (cl:setf (cl:slot-value msg 'pose_valid) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AprilTag>)))
  "Returns string type for a message object of type '<AprilTag>"
  "apriltags_msgs/AprilTag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AprilTag)))
  "Returns string type for a message object of type 'AprilTag"
  "apriltags_msgs/AprilTag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AprilTag>)))
  "Returns md5sum for a message object of type '<AprilTag>"
  "568095c4db6d177cf3bc296bc284d97b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AprilTag)))
  "Returns md5sum for a message object of type 'AprilTag"
  "568095c4db6d177cf3bc296bc284d97b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AprilTag>)))
  "Returns full string definition for message of type '<AprilTag>"
  (cl:format cl:nil "string id~%geometry_msgs/Point[] corners_px~%geometry_msgs/Pose pose_3d~%bool pose_valid~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AprilTag)))
  "Returns full string definition for message of type 'AprilTag"
  (cl:format cl:nil "string id~%geometry_msgs/Point[] corners_px~%geometry_msgs/Pose pose_3d~%bool pose_valid~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AprilTag>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'corners_px) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_3d))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AprilTag>))
  "Converts a ROS message object to a list"
  (cl:list 'AprilTag
    (cl:cons ':id (id msg))
    (cl:cons ':corners_px (corners_px msg))
    (cl:cons ':pose_3d (pose_3d msg))
    (cl:cons ':pose_valid (pose_valid msg))
))
