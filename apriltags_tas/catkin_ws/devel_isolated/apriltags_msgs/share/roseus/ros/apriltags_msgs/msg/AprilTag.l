;; Auto-generated. Do not edit!


(when (boundp 'apriltags_msgs::AprilTag)
  (if (not (find-package "APRILTAGS_MSGS"))
    (make-package "APRILTAGS_MSGS"))
  (shadow 'AprilTag (find-package "APRILTAGS_MSGS")))
(unless (find-package "APRILTAGS_MSGS::APRILTAG")
  (make-package "APRILTAGS_MSGS::APRILTAG"))

(in-package "ROS")
;;//! \htmlinclude AprilTag.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass apriltags_msgs::AprilTag
  :super ros::object
  :slots (_id _corners_px _pose_3d _pose_valid ))

(defmethod apriltags_msgs::AprilTag
  (:init
   (&key
    ((:id __id) "")
    ((:corners_px __corners_px) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:pose_3d __pose_3d) (instance geometry_msgs::Pose :init))
    ((:pose_valid __pose_valid) nil)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _corners_px __corners_px)
   (setq _pose_3d __pose_3d)
   (setq _pose_valid __pose_valid)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:corners_px
   (&rest __corners_px)
   (if (keywordp (car __corners_px))
       (send* _corners_px __corners_px)
     (progn
       (if __corners_px (setq _corners_px (car __corners_px)))
       _corners_px)))
  (:pose_3d
   (&rest __pose_3d)
   (if (keywordp (car __pose_3d))
       (send* _pose_3d __pose_3d)
     (progn
       (if __pose_3d (setq _pose_3d (car __pose_3d)))
       _pose_3d)))
  (:pose_valid
   (&optional __pose_valid)
   (if __pose_valid (setq _pose_valid __pose_valid)) _pose_valid)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; geometry_msgs/Point[] _corners_px
    (apply #'+ (send-all _corners_px :serialization-length)) 4
    ;; geometry_msgs/Pose _pose_3d
    (send _pose_3d :serialization-length)
    ;; bool _pose_valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; geometry_msgs/Point[] _corners_px
     (write-long (length _corners_px) s)
     (dolist (elem _corners_px)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Pose _pose_3d
       (send _pose_3d :serialize s)
     ;; bool _pose_valid
       (if _pose_valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point[] _corners_px
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _corners_px (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _corners_px)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Pose _pose_3d
     (send _pose_3d :deserialize buf ptr-) (incf ptr- (send _pose_3d :serialization-length))
   ;; bool _pose_valid
     (setq _pose_valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get apriltags_msgs::AprilTag :md5sum-) "568095c4db6d177cf3bc296bc284d97b")
(setf (get apriltags_msgs::AprilTag :datatype-) "apriltags_msgs/AprilTag")
(setf (get apriltags_msgs::AprilTag :definition-)
      "string id
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



(provide :apriltags_msgs/AprilTag "568095c4db6d177cf3bc296bc284d97b")


