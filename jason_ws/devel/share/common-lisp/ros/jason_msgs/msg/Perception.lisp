; Auto-generated. Do not edit!


(cl:in-package jason_msgs-msg)


;//! \htmlinclude Perception.msg.html

(cl:defclass <Perception> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Perception (<Perception>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Perception>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Perception)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jason_msgs-msg:<Perception> is deprecated: use jason_msgs-msg:Perception instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Perception>) ostream)
  "Serializes a message object of type '<Perception>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Perception>) istream)
  "Deserializes a message object of type '<Perception>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Perception>)))
  "Returns string type for a message object of type '<Perception>"
  "jason_msgs/Perception")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Perception)))
  "Returns string type for a message object of type 'Perception"
  "jason_msgs/Perception")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Perception>)))
  "Returns md5sum for a message object of type '<Perception>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Perception)))
  "Returns md5sum for a message object of type 'Perception"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Perception>)))
  "Returns full string definition for message of type '<Perception>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Perception)))
  "Returns full string definition for message of type 'Perception"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Perception>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Perception>))
  "Converts a ROS message object to a list"
  (cl:list 'Perception
))
