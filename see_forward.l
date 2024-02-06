#! /usr/bin/env roseus

;; robotの初期化
;(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
;(dxl-armed-turtlebot-init)

;; jsk_pcl_rosのmsgを使うため、loadする
;;(ros::load-ros-manifest "jsk_pcl_ros")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "std_msgs")

;; クラスタリング結果であるBoundingBoxのtopic名
;;(defvar *topic-name* "/camera/depth_registered/cluster_decomposer/boxes")
(defvar *topic-name* "/camera/depth_registered/boxes_use_pca")
(defvar *bounding-box-list* nil)

;; ros::initする
(ros::roseus "box_info_use_pca")

;;クラスタリング結果で一番近いものの対象までの距離
(setq *dist* most-positive-fixnum)

;;ヨー角の取得
(setq *yaw* nil)

;;コールバック関数の設定
(defun bounding-use-pca (msg)
  ;;メッセージを受け取ったときに高さと距離を初期化
  (setq *dist* most-positive-fixnum)
  (setq *quo_x* most-positive-fixnum)
  (setq *bounding-box-list* (send msg :boxes))

  (when *bounding-box-list*
    (mapcar #'(lambda (b)
		(let* ((dims (ros::tf-point->pos (send b :dimensions)))
		       (pos (ros::tf-pose->coords (send b :pose)))
		       (yaw nil)
		       (dist most-positive-fixnum))

		  (setq dist (- (elt (send pos :pos) 2) (/ (elt dims 2) 2)))
		  (setq yaw  (elt (elt (rpy-angle (send pos :rot)) 0) 1))
	
		  (when (< dist *dist*)
		    (setq *dist* dist)
		    (setq *yaw* yaw))
		  pos))
	    *bounding-box-list*)
    ;;パブリッシュする
    (setq msg_y (instance std_msgs::float32 :init))
    (send msg_y :data *yaw*)
    (ros::publish "kinect/box_yaw" msg_y)
    ))


(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-use-pca 1)

(ros::rate 10)

(ros::advertise "kinect/box_yaw" std_msgs::float32 10)

(do-until-key
 (ros::spin-once)
 (ros::sleep)
 )
