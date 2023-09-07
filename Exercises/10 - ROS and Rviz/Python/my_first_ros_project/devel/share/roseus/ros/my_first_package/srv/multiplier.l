;; Auto-generated. Do not edit!


(when (boundp 'my_first_package::multiplier)
  (if (not (find-package "MY_FIRST_PACKAGE"))
    (make-package "MY_FIRST_PACKAGE"))
  (shadow 'multiplier (find-package "MY_FIRST_PACKAGE")))
(unless (find-package "MY_FIRST_PACKAGE::MULTIPLIER")
  (make-package "MY_FIRST_PACKAGE::MULTIPLIER"))
(unless (find-package "MY_FIRST_PACKAGE::MULTIPLIERREQUEST")
  (make-package "MY_FIRST_PACKAGE::MULTIPLIERREQUEST"))
(unless (find-package "MY_FIRST_PACKAGE::MULTIPLIERRESPONSE")
  (make-package "MY_FIRST_PACKAGE::MULTIPLIERRESPONSE"))

(in-package "ROS")





(defclass my_first_package::multiplierRequest
  :super ros::object
  :slots (_a _b ))

(defmethod my_first_package::multiplierRequest
  (:init
   (&key
    ((:a __a) 0)
    ((:b __b) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   (setq _b (round __b))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:b
   (&optional __b)
   (if __b (setq _b __b)) _b)
  (:serialization-length
   ()
   (+
    ;; int32 _a
    4
    ;; int32 _b
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _a
       (write-long _a s)
     ;; int32 _b
       (write-long _b s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _a
     (setq _a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _b
     (setq _b (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass my_first_package::multiplierResponse
  :super ros::object
  :slots (_result ))

(defmethod my_first_package::multiplierResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass my_first_package::multiplier
  :super ros::object
  :slots ())

(setf (get my_first_package::multiplier :md5sum-) "a7d7d7065d45755acef7d4dcf908162a")
(setf (get my_first_package::multiplier :datatype-) "my_first_package/multiplier")
(setf (get my_first_package::multiplier :request) my_first_package::multiplierRequest)
(setf (get my_first_package::multiplier :response) my_first_package::multiplierResponse)

(defmethod my_first_package::multiplierRequest
  (:response () (instance my_first_package::multiplierResponse :init)))

(setf (get my_first_package::multiplierRequest :md5sum-) "a7d7d7065d45755acef7d4dcf908162a")
(setf (get my_first_package::multiplierRequest :datatype-) "my_first_package/multiplierRequest")
(setf (get my_first_package::multiplierRequest :definition-)
      "# Inputs
int32 a
int32 b
---
int32 result


")

(setf (get my_first_package::multiplierResponse :md5sum-) "a7d7d7065d45755acef7d4dcf908162a")
(setf (get my_first_package::multiplierResponse :datatype-) "my_first_package/multiplierResponse")
(setf (get my_first_package::multiplierResponse :definition-)
      "# Inputs
int32 a
int32 b
---
int32 result


")



(provide :my_first_package/multiplier "a7d7d7065d45755acef7d4dcf908162a")


