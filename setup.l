#! /usr/bin/env roseus 

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(dxl-armed-turtlebot-init) ;; アーム＋台車ロボットの *ri* と *dxl-armed-turtlebot* を生成
(send *dxl-armed-turtlebot* :angle-vector #f(0 135 -90 -90 90 90))) ;; 関節角度を セット
(send *irtviewer* :draw-objects) ;; 描画
(send *ri* :angle-vector (send *dxl-armed-turtlebot* :angle-vector) 4000)

