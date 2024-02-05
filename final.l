#!/usr/bin/env roseus

#|


;;セットアップ
(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init) ;; アーム＋台車ロボットの *ri* と *dxl-armed-turtlebot* を生成

(send *dxl-armed-turtlebot* :angle-vector #f(0 135 -90 -90 90 90))) ;; 関節角度を セット
(send *irtviewer* :draw-objects) ;; 描画
(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 3)

(print (list 'setup 'is 'finished!))
|#
;;メッセージ型のロード
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "std_msgs")


;;ros::initする
(ros::roseus "jishupuro")

;;測距センサーの位置
(setq *ard_pos* 130)


;;測距センサーから得られる距離
(setq *dist_ard* nil)

;;kinectから得られる箱の高さ、距離、右辺、左辺、箱の幅、ヨー角
(setq *box_height* nil)
(setq *box_dist* nil)
(setq *box_right* nil)
(setq *box_left* nil)
(setq *box_width* nil)
(setq *box_yaw* nil)

;;関節角度
(setq *first-angle-vector* nil)

;;ステップの状態を格納する変数
(setq *step_num* nil)

;;ステップの開始時刻を格納する変数
(setq *start_time* nil)


;;コールバック関数の設定
(defun arduino_distance_cb (msg)
  (setq *dist_ard* (* 10 (send msg :data))))

(defun kinect_height_cb (msg)
;;ステップ１で取得したらそれ以降一回も更新しない
  (when (= *step_num* 1)
    (if (null *box_height*) (setq *box_height* (send msg :data))
  (setq *box_height* (/ (+ *box_height* (send msg :data))2)))))
  

(defun kinect_distance_cb (msg)
  (setq *box_dist* (send msg :data)))

(defun kinect_right_cb (msg)
  (setq *box_right* (send msg :data)))

(defun kinect_left_cb(msg)
  (setq *box_left* (send msg :data))
  (when (or (= *step_num* 11) (= *step_num* 1))
    (setq *box_width* (- *box_right* *box_left*))))

(defun kinect_yaw_cb(msg)
  (setq *box_yaw* (send msg :data)))


;;サブスクライバの設定
;;測距センサについて
(ros::subscribe "arduino/distance" std_msgs::float64  #'arduino_distance_cb)
;;kinectについて
(ros::subscribe "kinect/box_height" std_msgs::float32 #'kinect_height_cb)
(ros::subscribe "kinect/box_distance" std_msgs::float32 #'kinect_distance_cb)
(ros::subscribe "kinect/box_right" std_msgs::float32 #'kinect_right_cb)
(ros::subscribe "kinect/box_left" std_msgs::float32 #'kinect_left_cb)
(ros::subscribe "kinect/box_yaw" std_msgs::float32 #'kinect_yaw_cb)


;;周波数の設定
(ros::rate 100)

;;実際の処理開始
(setq *step_num* 0)
(setq *start_time* (car (unix:ptimes)))
(print (list 'step *step_num* 'start!))

;;step0 箱に対して正面を向く
(print (list 'step *step_num* 'start!))


(while t
 (when (> (- (car (unix:ptimes)) *start_time*) 200) (return nil))
 (ros::sleep)
 (ros::spin-once)
)

(print (list 'ok))

(while t
  (let (vel pm)
    (setq vel (* *box_yaw* -50))
    (setq pm (/ vel (abs vel)))
    (print *box_yaw*)
    (when (< (abs *box_yaw*) 0.02)
      (return nil))
    (if (< (abs vel) 3)
	(send *ri* :go-velocity 0 0 vel) (send *ri* :go-velocity 0 0 (* pm 3))))
  (ros::sleep)
  (ros::spin-once))
     
(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))



;;step1　箱の高さ、幅の取得 (box_heightとbox_width を取得する)
;;一定時間rosを回して高さを取得する
(setq *start_time* (car (unix:ptimes)))
(print (list 'step *step_num* 'start!))

(while t
 (when (> (- (car (unix:ptimes)) *start_time*) 200) (return nil))
 (ros::sleep)
 (ros::spin-once)
)

(print (list 'height 'is *box_height*))
(print (list 'width 'is *box_width*))
(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))


;;step2 取得した高さをもとに逆運動学を解く
(print (list 'step *step_num* 'start!))

;;ポーズの初期化
(send *dxl-armed-turtlebot* :reset-pose)

;;取得した高さより少し高め位置で解く（重力を考慮＋このあと刃を刺す）
(send *dxl-armed-turtlebot* :inverse-kinematics
(make-coords :pos (float-vector -10 390 (+ *box_height* 25)) :rpy (list 1.394 1.571 -1.748)) :translation-axis :yz  :rotation-axis :y)

;;関節角度の保存
(setq *first-angle-vector* (send *dxl-armed-turtlebot* :angle-vector))

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))


;;step3 手先が箱の端に当たるように移動
(print (list 'step *step_num* 'start!))

;;逆運動学を説いた結果の手先の座標を取得 xが箱の左右方向　yが奥行き方向
(setq *arm_x_coords* (elt (send (send *dxl-armed-turtlebot* :arm :end-coords) :worldpos) 0))
(setq *arm_y_coords* (elt (send (send *dxl-armed-turtlebot* :arm :end-coords) :worldpos) 1))

(print (list 'arm_x_coords 'is *arm_x_coords*))
(print (list 'arm_y_coords 'is *arm_y_coords*))
;;手先が箱の左端に来るまで移動(手先が確実に刺さってほしいので、箱の左端の位置を右寄りに60mm補正)

;;箱の左端と手先位置の差が一定になるまで移動（P制御）
(while t
  (let (sub vel pm)
  (setq sub (- *arm_x_coords* (+ *box_left* 60)))
  (setq vel (* sub -0.001))
  (setq pm (/ vel (abs vel)))
  (print (list 'sub 'is sub))
  (print (list 'vel 'is vel))
  (print (list 'pm 'is pm))
    (when (< (abs sub) 5)
      (return nil))
    (if (< (abs vel) 0.05)
	(send *ri* :go-velocity vel 0 0) (send *ri* :go-velocity (* pm 0.05) 0 0)))
  (ros::sleep)
  (ros::spin-once))
	 

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))


 ;;step4 90度回転させて移動方向を箱に向ける
(print (list 'step *step_num* 'start!))



;;腕を下に向ける
(send *dxl-armed-turtlebot* :angle-vector #f(0 0 -90 0 0 90))

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

;;回転
(send *ri* :go-pos 0 0 90)

(unix:sleep 1)


(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step5測距センサにより、手先が箱に当たる位置まで前進(手先より前に刃があることを考慮した30mmの補正を入れる）
(print (list 'step *step_num* 'start!))

(while t
(let  (sub vel pm)
  (setq sub (- (+ *arm_y_coords* 30) (+ *ard_pos* *dist_ard*)))
  (setq vel (* sub -0.001))
  (setq pm (/ vel (abs vel)))
  (print (list 'sub 'is sub))
    (when (< (abs sub) 5)
      (return nil))
    (if (< (abs vel) 0.05)
	(send *ri* :go-velocity vel 0 0) (send *ri* :go-velocity (* pm 0.05) 0 0)))
  (ros::sleep)
  (ros::spin-once))

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step6 アームを正面に向けるために90度回転
(print (list 'step *step_num* 'start!))


(send *ri* :go-pos 0 0 -90)

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step7 アームを逆運動学の結果まで移動し、その後刃を差し込む
(print (list 'step *step_num* 'start!))

;;重力を考慮し、二番目の関節角度を調整する
(send *dxl-armed-turtlebot* :angle-vector *first-angle-vector*)
(send (send *dxl-armed-turtlebot* :arm_joint2) :joint-angle  (+ (elt (send *dxl-armed-turtlebot* :angle-vector) 1) 30))

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)
(unix:sleep 5)

;;刃を刺す

(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 20 0) :world
      :rotation-axis :y)

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)
(unix:sleep 2)

(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 0 -20) :world
      :rotation-axis :y)

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)
(unix:sleep 2)





(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step8 切りきるまで動く
(print (list 'step *step_num* 'start!))


(setq *start_time* (car (unix:ptimes)))

;;取得した幅をもとに切りきるまで動く(接触抵抗により指令値より進まないため補正を入れる）
(send *ri* :go-pos (* (/ *box_width* 1000) 1.3) 0 0)


(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step9 アームを外す
(print (list 'step *step_num* 'start!))

(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 0 30) :world
      :rotation-axis :y :translation-axis nil)

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 2)

(send *dxl-armed-turtlebot* :angle-vector #f(0 135 -90 -90 90 90))

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 5)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))



;;step10 次の面の正面まで移動
(print (list 'step *step_num* 'start!))

(send *ri* :go-pos 0 0 40)

(unix:sleep 1)

(send *ri* :go-pos 0.7 0 0)

(unix:sleep 1)

(send *ri* :go-pos 0 0 50)

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))


;;後半の蓋を開けるステップの始まり



;;step11 正面を向く step0と同じ
(print (list 'step *step_num* 'start!))


(while t
  (let (vel pm)
    (setq vel (* *box_yaw* -50))
    (setq pm (/ vel (abs vel)))
    (print *box_yaw*)
    (when (< (abs *box_yaw*) 0.02)
      (return nil))
    (if (< (abs vel) 5)
	(send *ri* :go-velocity 0 0 vel) (send *ri* :go-velocity 0 0 (* pm 5))))
  (ros::sleep)
  (ros::spin-once))

;;横幅を取得する時間を設ける
(setq *start_time* (car (unix:ptimes)))

(while t
 (when (> (- (car (unix:ptimes)) *start_time*) 200) (return nil))
 (ros::sleep)
 (ros::spin-once)
)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))



;;step12 逆運動学を解く
(print (list 'step *step_num* 'start!))

(send *dxl-armed-turtlebot* :reset-pose)
(send *dxl-armed-turtlebot* :inverse-kinematics
      (make-coords :pos (float-vector -10 400 (+ *box_height* 55)) :rpy (list (/ pi 2) 0 (/ -pi 2))) :translation-axis :xz)

;;関節角度の保存
(setq *second-angle-vector* (send *dxl-armed-turtlebot* :angle-vector))

(unix:sleep 1)
(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))


;;step13 手先が箱の左端に刺さるように横移動(step3を参考)
(print (list 'step *step_num* 'start!))

;;逆運動学を説いた結果の手先の座標を取得 xが箱の左右方向　yが奥行き方向
(setq *arm_x_coords* (elt (send (send *dxl-armed-turtlebot* :arm :end-coords) :worldpos) 0))
(setq *arm_y_coords* (elt (send (send *dxl-armed-turtlebot* :arm :end-coords) :worldpos) 1))

;;箱の左端と手先位置の差が一定になるまで移動（P制御）(手先全体が刺さるように箱の左端の位置を右方向に50mm補正)
(while t
  (let (sub vel pm)
  (setq sub (- *arm_x_coords* (+ *box_left* 50)))
  (setq vel (* sub -0.001))
  (setq pm (/ vel (abs vel)))
  (print (list 'sub 'is sub))
  (print (list 'vel 'is vel))
  (print (list 'pm 'is pm))
    (when (< (abs sub) 5)
      (return nil))
    (if (< (abs vel) 0.05)
	(send *ri* :go-velocity vel 0 0) (send *ri* :go-velocity (* pm 0.05) 0 0)))
  (ros::sleep)
  (ros::spin-once))

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))


;;step14 90度回転してからアームの手先が箱に当たるように箱に近づく(step4とstep5を参考）
(print (list 'step *step_num* 'start!))

;;90度回転
(send *ri* :go-pos 0 0 90)

(unix:sleep 1)

;;測距センサを利用して箱に近づく
(while t
(let  (sub vel pm)
  (setq sub (- (+ *arm_y_coords* -35)  (+ *ard_pos* *dist_ard*)))
  (setq vel (* sub -0.001))
  (setq pm (/ vel (abs vel)))
  (print (list 'sub 'is sub))
    (when (< (abs sub) 5)
      (return nil))
    (if (< (abs vel) 0.05)
	(send *ri* :go-velocity vel 0 0) (send *ri* :go-velocity (* pm 0.05) 0 0)))
  (ros::sleep)
  (ros::spin-once))

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step15　90度回転してアームを箱の方向に向けてから逆運動学の結果を適用
(print (list 'step *step_num* 'start!))

;;90度回転
(send *ri* :go-pos 0 0 -90)

(unix:sleep 1)

;;逆運動学の結果を適用

;;重力を考慮し、二番目の関節角度を調整する
(send *dxl-armed-turtlebot* :angle-vector *second-angle-vector*)
(send (send *dxl-armed-turtlebot* :arm_joint2) :joint-angle  (+ (elt (send *dxl-armed-turtlebot* :angle-vector) 1) 5))

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 5)

;;奥に移動して手先を箱に刺す

;;さげる
(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 0 -10) :world :rotation-axis :y)

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 3)

;;まえにだす
(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 15 0) :world :rotation-axis :y)

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

;;刺さった時点での手先の座標を保存
(setq *co* (send (send (send *dxl-armed-turtlebot* :arm :end-coords) :copy-worldcoords) :pos))

(unix:sleep 3)

;;移動
(send *ri* :go-pos 0.02 0 0)

;;まえにだす
(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 15 0) :world :rotation-axis :y)

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 3)


(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;step16 箱の開封
(print (list 'step *step_num* 'start!))

;;手先を傾ける
(send (send *dxl-armed-turtlebot* :arm_joint6) :joint-angle  (+ (elt (send *dxl-armed-turtlebot* :angle-vector) 5) 10))

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 2)

;;手先の角度を考慮してアームを上げる
(send *dxl-armed-turtlebot* :rarm :move-end-pos (float-vector 0 0 80) :world :rotation-axis :y)

(send (send *dxl-armed-turtlebot* :arm_joint6) :joint-angle  (+ (elt (send *dxl-armed-turtlebot* :angle-vector) 5) 15))

(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 6)


(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))

;;この時点で刺さっている
;;手先を回転させて箱を回転させる


(setq *co* (v+ *co* (float-vector (/ *box_width* 4) 0 (/ *box_width* 10))))
(print *co*)
(send *dxl-armed-turtlebot* :inverse-kinematics
      (make-coords :pos *co* :rpy (list 1.571 0 -0.785)) :translation-axis :z)
(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

(unix:sleep 1)

(print (list 'step *step_num* 'finished!))
(setq *step_num* (+ *step_num* 1))



