;; Auto-generated. Do not edit!


(when (boundp 'my_first_package::values)
  (if (not (find-package "MY_FIRST_PACKAGE"))
    (make-package "MY_FIRST_PACKAGE"))
  (shadow 'values (find-package "MY_FIRST_PACKAGE")))
(unless (find-package "MY_FIRST_PACKAGE::VALUES")
  (make-package "MY_FIRST_PACKAGE::VALUES"))

(in-package "ROS")
;;//! \htmlinclude values.msg.html


(defclass my_first_package::values
  :super ros::object
  :slots (_letter ))

(defmethod my_first_package::values
  (:init
   (&key
    ((:letter __letter) 0)
    )
   (send-super :init)
   (setq _letter (round __letter))
   self)
  (:letter
   (&optional __letter)
   (if __letter (setq _letter __letter)) _letter)
  (:serialization-length
   ()
   (+
    ;; int64 _letter
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _letter
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _letter (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _letter) (= (length (_letter . bv)) 2)) ;; bignum
              (write-long (ash (elt (_letter . bv) 0) 0) s)
              (write-long (ash (elt (_letter . bv) 1) -1) s))
             ((and (class _letter) (= (length (_letter . bv)) 1)) ;; big1
              (write-long (elt (_letter . bv) 0) s)
              (write-long (if (>= _letter 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _letter s)(write-long (if (>= _letter 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _letter
#+(or :alpha :irix6 :x86_64)
      (setf _letter (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _letter (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get my_first_package::values :md5sum-) "0aff0824e905c6e4dad564004ff12fa0")
(setf (get my_first_package::values :datatype-) "my_first_package/values")
(setf (get my_first_package::values :definition-)
      "int64 letter

")



(provide :my_first_package/values "0aff0824e905c6e4dad564004ff12fa0")


