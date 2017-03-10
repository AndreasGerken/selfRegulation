; Auto-generated. Do not edit!


(cl:in-package tiny_msgs-msg)


;//! \htmlinclude tinyIMU.msg.html

(cl:defclass <tinyIMU> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (accel
    :reader accel
    :initarg :accel
    :type tiny_msgs-msg:tinyVector
    :initform (cl:make-instance 'tiny_msgs-msg:tinyVector))
   (gyro
    :reader gyro
    :initarg :gyro
    :type tiny_msgs-msg:tinyVector
    :initform (cl:make-instance 'tiny_msgs-msg:tinyVector)))
)

(cl:defclass tinyIMU (<tinyIMU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tinyIMU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tinyIMU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tiny_msgs-msg:<tinyIMU> is deprecated: use tiny_msgs-msg:tinyIMU instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <tinyIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiny_msgs-msg:header-val is deprecated.  Use tiny_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <tinyIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiny_msgs-msg:accel-val is deprecated.  Use tiny_msgs-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'gyro-val :lambda-list '(m))
(cl:defmethod gyro-val ((m <tinyIMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tiny_msgs-msg:gyro-val is deprecated.  Use tiny_msgs-msg:gyro instead.")
  (gyro m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tinyIMU>) ostream)
  "Serializes a message object of type '<tinyIMU>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyro) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tinyIMU>) istream)
  "Deserializes a message object of type '<tinyIMU>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyro) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tinyIMU>)))
  "Returns string type for a message object of type '<tinyIMU>"
  "tiny_msgs/tinyIMU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tinyIMU)))
  "Returns string type for a message object of type 'tinyIMU"
  "tiny_msgs/tinyIMU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tinyIMU>)))
  "Returns md5sum for a message object of type '<tinyIMU>"
  "53582bc8b7315f3bc7728d82df98bb24")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tinyIMU)))
  "Returns md5sum for a message object of type 'tinyIMU"
  "53582bc8b7315f3bc7728d82df98bb24")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tinyIMU>)))
  "Returns full string definition for message of type '<tinyIMU>"
  (cl:format cl:nil "# a Tiny IMU message~%~%Header header~%tinyVector accel~%tinyVector gyro~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tiny_msgs/tinyVector~%# A Tiny Vector~%~%int16 x~%int16 y~%int16 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tinyIMU)))
  "Returns full string definition for message of type 'tinyIMU"
  (cl:format cl:nil "# a Tiny IMU message~%~%Header header~%tinyVector accel~%tinyVector gyro~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tiny_msgs/tinyVector~%# A Tiny Vector~%~%int16 x~%int16 y~%int16 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tinyIMU>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyro))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tinyIMU>))
  "Converts a ROS message object to a list"
  (cl:list 'tinyIMU
    (cl:cons ':header (header msg))
    (cl:cons ':accel (accel msg))
    (cl:cons ':gyro (gyro msg))
))
