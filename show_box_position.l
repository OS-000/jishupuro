#!/usr/bin/env roseus

;;robotの初期化
;(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
;(dxl-armed-turtlebot-init)

;; jsk_pcl_rosのmsgを使うため、loadする
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "std_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
(defvar *topic-name* "/camera/depth_registered/boxes")
(defvar *bounding-box-list* nil)

;; ros::initする
(ros::roseus "box_info")

;;変更後のcam-coordsを定義
(setq *new* (make-coords))
(send *new* :translate (float-vector 12.5 -87 305.2))
(send *new* :rotate (/ -pi 2) :x)

;;クラスタリング結果で一番近いものの対象までの距離
(setq *dist* most-positive-fixnum)

;;クラスタリング結果で一番近いものの高さ
(setq *height* most-positive-fixnum)

;;クラスタリング結果で一番近いものの右端と左端
(setq *right* most-positive-fixnum)
(setq *left* most-positive-fixnum)

;;コールバック関数の設定
(defun bounding-box-array-cb (msg)
  ;;メッセージを受け取ったときに高さと距離を初期化
  (setq *dist* most-positive-fixnum)
  (setq *height* most-positive-fixnum)
  (setq *bounding-box-list* (send msg :boxes))
  ;;Boundingboxがあれば表示
  (when *bounding-box-list*
    (mapcar #'(lambda (b)
	    ;;寸法の抽出
		(let* ((dims (ros::tf-point->pos (send b :dimensions)))
		       (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2)))
		   (cam->objcoords (ros::tf-pose->coords (send b :pose)))
		   (cam-coords (send *new* :copy-worldcoords))
		   ;;*bounding_box_list*から取り出されたものの高さと距離を格納する
		   (height most-positive-fixnum)
		   (dist most-positive-fixnum)
		   (right most-positive-fixnum)
		   (left most-positive-fixnum))
		  
	      
	      (send bx :newcoords (send cam-coords :transform cam->objcoords))
	      (send bx :worldcoords)
	     
	      (setq height  (+ (elt (send (send bx :copy-worldcoords) :pos) 2) (/ (elt dims 1) 2)))
	      (setq dist (- (elt (send (send bx :copy-worldcoords) :pos) 1) (/ (elt dims 2) 2)))
	      (setq right (+ (elt (send (send bx :copy-worldcoords) :pos) 0) (/ (elt dims 0) 2)))
	      (setq left (- (elt (send (send bx :copy-worldcoords) :pos) 0) (/ (elt dims 0) 2)))
	      
	      (when (< dist *dist*)
		  (setq *dist* dist)
		  (setq *height* height)
		  (setq *right* right)
		  (setq *left* left))
	      bx))
	    *bounding-box-list*)
  
    ;;パブリッシュする
    (setq msg_h (instance std_msgs::float32 :init))
    (setq msg_d (instance std_msgs::float32 :init))
    (setq msg_r (instance std_msgs::float32 :init))
    (setq msg_l (instance std_msgs::float32 :init))
    (send msg_h :data *height*)
    (send msg_d :data *dist*)
    (send msg_r :data *right*)
    (send msg_l :data *left*)
    (ros::publish "kinect/box_height" msg_h)
    (ros::publish "kinect/box_distance" msg_d)
    (ros::publish "kinect/box_right" msg_r)
    (ros::publish "kinect/box_left" msg_l)
    )
  )
     
	     



;;サブスクライバの設定
(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

;;パブリッシャの設定
(ros::advertise "kinect/box_height" std_msgs::float32 10)
(ros::advertise "kinect/box_distance" std_msgs::float32 10)
(ros::advertise "kinect/box_right" std_msgs::float32 10)
(ros::advertise "kinect/box_left" std_msgs::float32 10)

(ros::rate 10)

(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )
