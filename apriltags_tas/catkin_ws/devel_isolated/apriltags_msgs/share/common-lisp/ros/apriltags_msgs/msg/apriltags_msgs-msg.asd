
(cl:in-package :asdf)

(defsystem "apriltags_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AprilTag" :depends-on ("_package_AprilTag"))
    (:file "_package_AprilTag" :depends-on ("_package"))
    (:file "AprilTagDetections" :depends-on ("_package_AprilTagDetections"))
    (:file "_package_AprilTagDetections" :depends-on ("_package"))
  ))