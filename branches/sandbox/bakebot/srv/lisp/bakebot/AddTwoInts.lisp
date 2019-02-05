; Auto-generated. Do not edit!


(in-package bakebot-srv)


;//! \htmlinclude AddTwoInts-request.msg.html

(defclass <AddTwoInts-request> (ros-message)
  ((a
    :reader a-val
    :initarg :a
    :type integer
    :initform 0)
   (b
    :reader b-val
    :initarg :b
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <AddTwoInts-request>) ostream)
  "Serializes a message object of type '<AddTwoInts-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'a)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'a)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'b)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'b)) ostream)
)
(defmethod deserialize ((msg <AddTwoInts-request>) istream)
  "Deserializes a message object of type '<AddTwoInts-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'a)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'b)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'b)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<AddTwoInts-request>)))
  "Returns string type for a service object of type '<AddTwoInts-request>"
  "bakebot/AddTwoIntsRequest")
(defmethod md5sum ((type (eql '<AddTwoInts-request>)))
  "Returns md5sum for a message object of type '<AddTwoInts-request>"
  "6a2e34150c00229791cc89ff309fff21")
(defmethod message-definition ((type (eql '<AddTwoInts-request>)))
  "Returns full string definition for message of type '<AddTwoInts-request>"
  (format nil "int64 a~%int64 b~%~%"))
(defmethod serialization-length ((msg <AddTwoInts-request>))
  (+ 0
     8
     8
))
(defmethod ros-message-to-list ((msg <AddTwoInts-request>))
  "Converts a ROS message object to a list"
  (list '<AddTwoInts-request>
    (cons ':a (a-val msg))
    (cons ':b (b-val msg))
))
;//! \htmlinclude AddTwoInts-response.msg.html

(defclass <AddTwoInts-response> (ros-message)
  ((sum
    :reader sum-val
    :initarg :sum
    :type integer
    :initform 0))
)
(defmethod serialize ((msg <AddTwoInts-response>) ostream)
  "Serializes a message object of type '<AddTwoInts-response>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'sum)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'sum)) ostream)
)
(defmethod deserialize ((msg <AddTwoInts-response>) istream)
  "Deserializes a message object of type '<AddTwoInts-response>"
  (setf (ldb (byte 8 0) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'sum)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'sum)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<AddTwoInts-response>)))
  "Returns string type for a service object of type '<AddTwoInts-response>"
  "bakebot/AddTwoIntsResponse")
(defmethod md5sum ((type (eql '<AddTwoInts-response>)))
  "Returns md5sum for a message object of type '<AddTwoInts-response>"
  "6a2e34150c00229791cc89ff309fff21")
(defmethod message-definition ((type (eql '<AddTwoInts-response>)))
  "Returns full string definition for message of type '<AddTwoInts-response>"
  (format nil "int64 sum~%~%~%"))
(defmethod serialization-length ((msg <AddTwoInts-response>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <AddTwoInts-response>))
  "Converts a ROS message object to a list"
  (list '<AddTwoInts-response>
    (cons ':sum (sum-val msg))
))
(defmethod service-request-type ((msg (eql 'AddTwoInts)))
  '<AddTwoInts-request>)
(defmethod service-response-type ((msg (eql 'AddTwoInts)))
  '<AddTwoInts-response>)
(defmethod ros-datatype ((msg (eql 'AddTwoInts)))
  "Returns string type for a service object of type '<AddTwoInts>"
  "bakebot/AddTwoInts")
