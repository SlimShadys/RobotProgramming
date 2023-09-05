;; Auto-generated. Do not edit!


(when (boundp 'my_first_package::position)
  (if (not (find-package "MY_FIRST_PACKAGE"))
    (make-package "MY_FIRST_PACKAGE"))
  (shadow 'position (find-package "MY_FIRST_PACKAGE")))
(unless (find-package "MY_FIRST_PACKAGE::POSITION")
  (make-package "MY_FIRST_PACKAGE::POSITION"))

(in-package "ROS")
;;//! \htmlinclude position.msg.html


(defclass my_first_package::position
  :super ros::object
  :slots (_message _x _y _even ))

(defmethod my_first_package::position
  (:init
   (&key
    ((:message __message) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:even __even) nil)
    )
   (send-super :init)
   (setq _message (string __message))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _even __even)
   self)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:even
   (&optional (__even :null))
   (if (not (eq __even :null)) (setq _even __even)) _even)
  (:serialization-length
   ()
   (+
    ;; string _message
    4 (length _message)
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; bool _even
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _even
       (if _even (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _even
     (setq _even (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get my_first_package::position :md5sum-) "1dc6b20bdff8e4b6f0ccfd6e4f6a991d")
(setf (get my_first_package::position :datatype-) "my_first_package/position")
(setf (get my_first_package::position :definition-)
      "string message
float32 x
float32 y
bool even
")



(provide :my_first_package/position "1dc6b20bdff8e4b6f0ccfd6e4f6a991d")


