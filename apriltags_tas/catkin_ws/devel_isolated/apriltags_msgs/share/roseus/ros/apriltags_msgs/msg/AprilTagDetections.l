;; Auto-generated. Do not edit!


(when (boundp 'apriltags_msgs::AprilTagDetections)
  (if (not (find-package "APRILTAGS_MSGS"))
    (make-package "APRILTAGS_MSGS"))
  (shadow 'AprilTagDetections (find-package "APRILTAGS_MSGS")))
(unless (find-package "APRILTAGS_MSGS::APRILTAGDETECTIONS")
  (make-package "APRILTAGS_MSGS::APRILTAGDETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude AprilTagDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass apriltags_msgs::AprilTagDetections
  :super ros::object
  :slots (_header _detections ))

(defmethod apriltags_msgs::AprilTagDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) (let (r) (dotimes (i 0) (push (instance apriltags_msgs::AprilTag :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; apriltags_msgs/AprilTag[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; apriltags_msgs/AprilTag[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; apriltags_msgs/AprilTag[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance apriltags_msgs::AprilTag :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get apriltags_msgs::AprilTagDetections :md5sum-) "d4bc35daf77e797dc95ce1f2fcc1ea5d")
(setf (get apriltags_msgs::AprilTagDetections :datatype-) "apriltags_msgs/AprilTagDetections")
(setf (get apriltags_msgs::AprilTagDetections :definition-)
      "Header header
AprilTag[] detections
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: apriltags_msgs/AprilTag
string id
geometry_msgs/Point[] corners_px
geometry_msgs/Pose pose_3d
bool pose_valid
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :apriltags_msgs/AprilTagDetections "d4bc35daf77e797dc95ce1f2fcc1ea5d")


