;; Auto-generated. Do not edit!


(when (boundp 'tiny_msgs::tinyIMU)
  (if (not (find-package "TINY_MSGS"))
    (make-package "TINY_MSGS"))
  (shadow 'tinyIMU (find-package "TINY_MSGS")))
(unless (find-package "TINY_MSGS::TINYIMU")
  (make-package "TINY_MSGS::TINYIMU"))

(in-package "ROS")
;;//! \htmlinclude tinyIMU.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tiny_msgs::tinyIMU
  :super ros::object
  :slots (_header _accel _gyro ))

(defmethod tiny_msgs::tinyIMU
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:accel __accel) (instance tiny_msgs::tinyVector :init))
    ((:gyro __gyro) (instance tiny_msgs::tinyVector :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _accel __accel)
   (setq _gyro __gyro)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:accel
   (&rest __accel)
   (if (keywordp (car __accel))
       (send* _accel __accel)
     (progn
       (if __accel (setq _accel (car __accel)))
       _accel)))
  (:gyro
   (&rest __gyro)
   (if (keywordp (car __gyro))
       (send* _gyro __gyro)
     (progn
       (if __gyro (setq _gyro (car __gyro)))
       _gyro)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tiny_msgs/tinyVector _accel
    (send _accel :serialization-length)
    ;; tiny_msgs/tinyVector _gyro
    (send _gyro :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tiny_msgs/tinyVector _accel
       (send _accel :serialize s)
     ;; tiny_msgs/tinyVector _gyro
       (send _gyro :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tiny_msgs/tinyVector _accel
     (send _accel :deserialize buf ptr-) (incf ptr- (send _accel :serialization-length))
   ;; tiny_msgs/tinyVector _gyro
     (send _gyro :deserialize buf ptr-) (incf ptr- (send _gyro :serialization-length))
   ;;
   self)
  )

(setf (get tiny_msgs::tinyIMU :md5sum-) "53582bc8b7315f3bc7728d82df98bb24")
(setf (get tiny_msgs::tinyIMU :datatype-) "tiny_msgs/tinyIMU")
(setf (get tiny_msgs::tinyIMU :definition-)
      "# a Tiny IMU message

Header header
tinyVector accel
tinyVector gyro

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: tiny_msgs/tinyVector
# A Tiny Vector

int16 x
int16 y
int16 z

")



(provide :tiny_msgs/tinyIMU "53582bc8b7315f3bc7728d82df98bb24")


