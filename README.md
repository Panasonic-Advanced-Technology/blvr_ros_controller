# blv_r_controller

> [!NOTE]
> 本ソフトウェアはパナソニック アドバンストテクノロジー株式会社が公開しています。ソフトウェアに関する問い合わせをオリエンタルモーター様へなさらないようにお願いします。

オリエンタルモーター BLVシリーズ Rタイプ用ロボットドライバ  
RS485を使ったmodbus-RTUでモータ制御用回路と通信する
IDシェアモードに対応

以下の台車に適用可能
- 二輪差動駆動
- 四輪差動駆動(メカナムホイール)

---
## 目次
<!-- @import "[TOC]" {cmd="toc" depthFrom=2 depthTo=3 orderedList=false} -->
<!-- code_chunk_output -->

- [目次](#目次)
- [コンフィグレーションファイルについて](#コンフィグレーションファイルについて)
- [ロボットドライバ起動ファイル (launchファイル) について](#ロボットドライバ起動ファイル-launchファイル-について)
- [Publishトピック](#publishトピック)
  - [~odom [nav_msgs/Odometry]](#odom-nav_msgsodometry)
  - [~emg_status [std_msgs/Bool]](#emg_status-std_msgsbool)
  - [~estop [robot_driver_msgs/MotorError]](#estop-robot_driver_msgsmotorerror)
  - [~status [robot_driver_msgs/MotorStatusComplex]](#status-robot_driver_msgsmotorstatuscomplex)
- [Subscribeトピック](#subscribeトピック)
  - [~cmd_vel [geometry_msgs/Twist]](#cmd_vel-geometry_msgstwist)
  - [~cmd_pos [geometry_msgs/Twist]](#cmd_pos-geometry_msgstwist)
- [Services](#services)
  - [~config_set_pos [robot_driver_msgs/ConfigSetPos]](#config_set_pos-robot_driver_msgsconfigsetpos)
  - [~get_status [robot_driver_msgs/GetMotorStatus]](#get_status-robot_driver_msgsgetmotorstatus)
  - [~reset [std_srvs/Empty]](#reset-std_srvsempty)
  - [~control_excitation [std_srvs/SetBool]](#control_excitation-std_srvssetbool)
- [Parameters](#parameters)
- [使用方法](#使用方法)
  - [build方法](#build方法)
  - [起動方法](#起動方法)
  - [操作方法](#操作方法)
  - [駆動確認および車輪回転方向の設定](#駆動確認および車輪回転方向の設定)

<!-- /code_chunk_output -->

---

## コンフィグレーションファイルについて

  configディレクトリにBLVシリーズ Rタイプ用コンフィグファイルを格納しています。
対象の台車に応じて、以下のように対応するコンフィグファイルを専用ソフト(MEXE02 ver.4.15以降)を用いて、各モータドライバに適用してください。MEXE02の使用方法についてはMEXE02付属のヘルプを参照してください。

- 二輪差動駆動台車

|ファイル名|説明|
|-|-|
|BLVD-KRD_Left.mxex|左車輪モータ用BLVD-KRDパラメータファイル(MODBUS軸番号2)|
|BLVD-KRD_Right.mxex|右車輪モータ用BLVD-KRDパラメータファイル(MODBUS軸番号1)|

- 四輪差動駆動台車

|ファイル名|説明|
|-|-|
|BLVD-KBRD_FL.mxex|前方左車輪モータ用BLVD-KBRDパラメータファイル(MODBUS軸番号1)|
|BLVD-KBRD_FR.mxex|前方右車輪モータ用BLVD-KBRDパラメータファイル(MODBUS軸番号2)|
|BLVD-KBRD_RL.mxex|後方左車輪モータ用BLVD-KBRDパラメータファイル(MODBUS軸番号3)|
|BLVD-KBRD_RR.mxex|後方右車輪モータ用BLVD-KBRDパラメータファイル(MODBUS軸番号4)|

## ロボットドライバ起動ファイル (launchファイル) について

  launchディレクトリに適用可能な台車の起動ファイルを格納しています。各起動ファイルは、前述のコンフィグレーションファイルが各モータドライバに適用されている想定で記載されています。

  |ファイル名|説明|
  |-|-|
  |blv_r.launch|二輪差動駆動台車用ロボットドライバ起動ファイル|
  |blv_r_mecanum.launch|四輪差動駆動(メカナムホイール)用ロボットドライバ起動ファイル|

> モータドライバの入れ替えや、MODBUS軸番号の変更を行う場合は、起動ファイルの修正を実施してください。

#### 各パラメータの詳細情報

|パラメータ|説明|
|-|-|
|device_path|RS485通信デバイスパス|
|baudrate|MODBUSに使用するRS485通信のボーレート設定[bps]|
|parity|MODBUSに使用するRS485通信のパリティ設定(none｜even｜odd)|
|stop_bit|MODBUSに使用するRS485通信のストップビット設定(1｜2)|
|left_id|左(前輪)MODBUS軸番号|
|right_id|右(前輪)MODBUS軸番号|
|rear_left_id|左後輪MODBUS軸番号 (四輪差動駆動台車の場合のみ必要)|
|rear_right_id|右後輪MODBUS軸番号 (四輪差動駆動台車の場合のみ必要)|
|global_id|シェアグループID|
|set_rpm_timeout|速度更新タイムアウト[sec]|
|auto_excitation_off|自動励磁OFF機能設定(true:有効｜false:無効)|
|excitation_off_sec|モータがアイドル状態になってから励磁OFFされるまでの時間[sec]|
|left_forward_rotation_is_positive|左モータ回転方向(true:前進が正値｜false:前進が負値)|
|right_forward_rotation_is_positive|右モータ回転方向(true:前進が正値｜false:前進が負値)|
|torque_in_vel_controlling|トルク値[0.1%]|
|torque_in_positioning|位置指定動作時トルク[0.1%]|
|gear_ratio|ギア比(減速比)|
|wheel_size|車輪径[m]|
|tread_width|トレッド幅[m]|
|wheel_base|ホイールベース[m] (四輪差動駆動台車の場合のみ必要)|
|velocity_updating_interval|速度指令更新間隔[sec]|
|encoder_polling_interval|エンコーダ情報取得間隔[sec]|
|status_polling_interval|モータステータス取得間隔[sec]|
|acc_in_vel_controlling|速度制御時に適用される加速度[m/sec^2]|
|dec_in_vel_controlling|速度制御時に適用される減速度[m/sec^2]|
|linear_vel_in_set_position|位置指定制御時に適用される並進速度[m/s]|
|linear_acc_in_set_position|位置指定制御時に適用される並進加速度[m/s^2]|
|linear_dec_in_set_position|位置指定制御時に適用される並進減速度[m/s^2]|
|angular_vel_in_set_position|位置指定制御時に適用される旋回角速度[rad/s]|
|angular_acc_in_set_position|位置指定制御時に適用される旋回角加速度[rad/s^2]|
|angular_dec_in_set_position|位置指定制御時に適用される旋回角減速度[rad/s^2]|

## Publishトピック
### ~odom [nav_msgs/Odometry]
オドメトリ。

### ~emg_status [std_msgs/Bool]
非常停止ボタンの押下状態の通知。
* true:非常停止モードON
* false:非常停止モードOFF

### ~estop [robot_driver_msgs/MotorError]
エラー停止状態の通知。

### ~status [robot_driver_msgs/MotorStatusComplex]
エラー詳細情報の通知。

本ドライバではrobot_driver_msgs/MotorStatus[] statusに以下を設定します。
* name
  * 二輪差動駆動台車用：left,right
  * 四輪差動駆動(メカナムホイール)用：left,right,rear_left,rear_right
* status
  * robot_driver_fw::RobotDriverStatus (robot_driver_fw/include/robot_driver_fw/robot_driver.h) を参照。
* sub_status
  * robot_driver_fw::RobotDriverErr (robot_driver_fw/include/robot_driver_fw/robot_status.h) の以下いずれかを通知。
    * RobotDriverErr::BUSY
    * RobotDriverErr::NOERR
* err_code
  * エラーコード(NET-ID 0x0040) [オリエンタルモーター BLVシリーズの仕様書参照]
* err_detail
  * name,err_codeを組み合わせた文字列。「(name)err_codeの16進数表示」。

## Subscribeトピック
### ~cmd_vel [geometry_msgs/Twist]
速度を指定し、ロボットを制御します。

### ~cmd_pos [geometry_msgs/Twist]
位置を指定し、ロボットを制御します。

## Services
### ~config_set_pos [robot_driver_msgs/ConfigSetPos]
位置指定制御時に適用されるパラメータを設定します。

### ~get_status [robot_driver_msgs/GetMotorStatus]
モータードライバの状態を取得します。

### ~reset [std_srvs/Empty]
モータードライバの再起動処理を行います。

### ~control_excitation [std_srvs/SetBool]
モーター励磁状態を操作します。
* 励磁ON/OFF(ON:true/OFF:false)

## Parameters
[ロボットドライバ起動ファイル (launchファイル) について](#ロボットドライバ起動ファイル-launchファイル-について) 参照。

## 使用方法
### build方法
下記いずれかの環境を事前にinstallする。
* Ubuntu18.04 + ROS melodic
* Ubuntu20.04 + ROS noetic

#### clone
以下のパッケージを```~/catkin_ws/src```以下に置きます。
* robot_driver_msgs
* robot_driver_fw
* blv_r_controller
```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Place the required packages here.
```

#### pre build
```shell
cd ~/catkin_ws
rosdep install --from-path src -ri -y
```

#### build
```shell
# set up your ROS. (melodic or noetic)
source /opt/ros/melodic/setup.bash
source /opt/ros/noetic/setup.bash

# config & build
catkin config --no-install --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

### 起動方法
```shell
source ~/catkin_ws/devel/setup.bash
roslaunch blv_r_controller blv_r.launch
```

* 次のようなエラーメッセージが表示されている場合、device_pathに設定したRS485通信デバイスにアクセスできるようになっているか確認してください。
  ```
  ERROR Can't open the device your_device_path (No such file or directory)
  ```
  RS485通信デバイスが存在しません。指定が正しいか確認してください。
  ```
  ERROR Can't open the device your_device_path (Permission denied)
  ```
  RS485通信デバイスに対してアクセス権がありません。read/writeできるように権限を設定してください。

### 操作方法
**以降の操作でモーターが回転します。安全を確保した状態で操作してください。** 

/cmd_vel [geometry_msgs/Twist] トピックを送信することで、ロボットを操作します。例として、teleop_twist_keyboardを使用します。

#### teleop_twist_keyboardのinstall
```shell
# install operation pkgs, according to your ROS.
sudo apt install ros-melodic-teleop-twist-keyboard
sudo apt install ros-noetic-teleop-twist-keyboard
```

#### teleop_twist_keyboardの起動
```shell
# set up your ROS. (melodic or noetic)
source /opt/ros/melodic/setup.bash
source /opt/ros/noetic/setup.bash
# launch.
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel
```

### 駆動確認および車輪回転方向の設定
teleop_twist_keyboardを少し操作し、モータが回転することを確認します。
モーターが回転することが確認できた場合、以降のモーターの回転方向の設定を行います。モーターに組み合わせる減速機の仕様によりモーター出力軸の回転方向と、車輪の回転方向が異なる場合があります。

#### 進行方向の確認

teleop_twist_keyboardで前進命令を入力します。車輪の回転方向を確認し、車輪が後進方向へ回転している場合、減速機によって回転方向が反転しています。設定ファイル```blv_r.launch```の下記パラメータを調整してください。
```
left_forward_rotation_is_positive: true
right_forward_rotation_is_positive: false
```
`true` -> `false`に変更することで、車輪の回転方向が逆転します。```blv_r.launch```の変更後、ロボットドライバを再起動して動作を確認してください。

#### 旋回方向の確認

teleop_twist_keyboardで右旋回（時計周り）命令を入力します。車体の回転方向が右旋回となっているか確認します。右旋回となっていない場合、設定ファイル```blv_r.launch```の下記パラメータを調整してください。
```
left_id: 2
right_id: 1
```

left/rightそれぞれのID `1` ↔ `2` を入れ替えてください。

編集後
```
left_id: 1
right_id: 2
```

四輪駆動型の車両の場合は、4つのモータの車両上の配置とそれぞれ対応するドライバに割り当てられたIDに一致するように設定ファイル```blv_r.launch```の下記パラメータを調整してください。
```
left_id: 1
right_id: 2
rear_left_id: 3
rear_right_id: 4
```

```blv_r.launch```の変更後、ロボットドライバを再起動して動作を確認してください。

