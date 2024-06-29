[English version](README.md)
# iRobot Create（Roomba）をROSで制御する
![Controlling a Roomba with ROS](img/Roomba_ps4.jpg)

# create_robot
[ROS](https://docs.ros.org)  ドライバ for iRobot Create 1 および 2.

<!--[](* Documentation: TODO)-->
* ROS wikiページ: http://wiki.ros.org/create_robot
* 作者: Takuto Yamauchi

## ビルドステータス
- ROS Noetic (ブランチ: `noetic`) ![](https://github.com/autonomylab/create_robot/workflows/Continuous%20Integration/badge.svg?branch=noetic)

## 対応ロボット

| モデル     | サポート状況    |
|-----------|------------|
| Roomba 500 シリーズ |  Yes  |

## インストール

#### 前提条件

* インターネット接続
* [ROS](http://wiki.ros.org/ROS/Installation)(noetic)
* NoeticはPython3のみをサポート
Ubuntuパッケージ: `python3-rosdep`, `python3-catkin-tools`

    ``` bash
    sudo apt-get install python3-rosdep python3-catkin-tools
    ```

#### コンパイル

1. catkinワークスペースを作成する  
    ``` bash
    cd ~
    mkdir -p create_ws/src  
    cd create_ws  
    catkin init  
    ```

2. このリポジトリをクローンする
    ``` bash
    cd ~/create_ws/src
    git clone https://github.com/autonomylab/create_robot.git --branch noetic
    ```
  
3. 依存関係をインストールする  
    ``` bash
    cd ~/create_ws
    rosdep update  
    rosdep install --from-paths src -i --rosdistro noetic 
    ```

4. ビルド
    ``` bash
    cd ~/create_ws
    catkin build
    ```

#### USBの権限設定
1. CreateをUSB経由で接続するために、ユーザーをdialoutグループに追加する
    ``` bash
    sudo usermod -a -G dialout $USER
    ```

2. 権限が反映されるようにログアウトして再ログインする

## ドライバの起動

### セットアップ

1. ソースからコンパイルした後、ワークスペースをソースするのを忘れないように:
    ``` bash
    cd
    source devel/setup.bash
    ```

2. シリアルポートをUSBに接続した後、
   ```bash
   cd
   sudo chmod +x /dev/ttyUSB0
   ```

### 起動ファイル

Create 2 (Roomba 500シリーズ)の場合:
``` bash
roslaunch create_bringup create_2.launch
```

### パブリッシャ

 トピック       | 説明  | 型
-------------|--------------|------
 `battery/capacity` | ロボットのバッテリーの推定充電容量 (Ah) | [std_msgs/Float32][float32]
 `battery/charge` | ロボットのバッテリーの現在の充電量 (Ah) | [std_msgs/Float32][float32]
 `battery/charge_ratio` | 充電量 / 容量 | [std_msgs/Float32][float32]
 `battery/charging_state` | バッテリーの充電状態 | [create_msgs/ChargingState][chargingstate_msg]
 `battery/current` | ロボットのバッテリーに流れる電流 (A)。正の電流は充電を意味する | [std_msgs/Float32][float32]
 `battery/temperature` | ロボットのバッテリーの温度 (摂氏) | [std_msgs/Int16][int16]
 `battery/voltage` | ロボットのバッテリーの電圧 (V) | [std_msgs/Float32][float32]
 `bumper` | バンパーの状態メッセージ（バンパー上の光センサーを含む） | [create_msgs/Bumper][bumper_msg]
 `clean_button` | 'clean'ボタンが押される（Create 1では'play'ボタン） | [std_msgs/Empty][empty]
 `day_button` |  'day'ボタンが押される | [std_msgs/Empty][empty]
 `hour_button` | 'hour'ボタンが押される | [std_msgs/Empty][empty]
 `minute_button` | 'minute'ボタンが押される | [std_msgs/Empty][empty]
 `dock_button` | 'dock'ボタンが押される（Create 1では'advance'ボタン） | [std_msgs/Empty][empty]
 `spot_button` | 'spot'ボタンが押される | [std_msgs/Empty][empty]
 `ir_omni` | 全方位受信機で現在読み取られているIR文字。値0は文字が受信されていないことを意味する | [std_msgs/UInt16][uint16]
 `joint_states` | ドライブホイールジョイントの状態（位置、速度） | [sensor_msgs/JointState][jointstate_msg]
 `mode` | ロボットの現在のモード（詳細は[OI Spec][oi_spec]を参照） | [create_msgs/Mode][mode_msg]
 `odom` | 車輪エンコーダに基づくロボットのオドメトリ | [nav_msgs/Odometry][odometry]
 `wheeldrop` | 少なくとも1つのドライブホイールが落ちた | [std_msgs/Empty][empty]
 `/tf` | `odom`フレームから`base_footprint`への変換。パラメータ`publish_tf`が`true`の場合のみ | [tf2_msgs/TFMessage](http://docs.ros.org/noetic/api/tf2_msgs/html/msg/TFMessage.html)
 `diagnostics` | バッテリーの充電、ホイールドロップ/クリフの状態、ロボットモード、およびシリアル接続に関する情報 | [diagnostic_msgs/DiagnosticArray](https://docs.ros.org/noetic/api/diagnostic_msgs/html/msg/DiagnosticArray.html)


### サブスクライバ

トピック       | 説明   | 型
------------|---------------|------
`cmd_vel` | 前方および角速度に従ってロボットのホイールを駆動する | [geometry_msgs/Twist][twist]
`debris_led` | 青い'デブリ'LEDを有効/無効にする | [std_msgs/Bool][bool]
`spot_led`   | 'スポット'LEDを有効/無効にする | [std_msgs/Bool][bool]
`dock_led`   | 'ドック'LEDを有効/無効にする | [std_msgs/Bool][bool]
`check_led`  | 'ロボットチェック'LEDを有効/無効にする | [std_msgs/Bool][bool]
`power_led`  | '電源'LEDの色と強度を設定する。1バイトまたは2バイトを受け入れ、最初のバイトは緑（0）から赤（255）の間の色を表し、2番目のバイト（オプション）はデフォルトで最も明るい設定（255）を表す強度を示す | [std_msgs/UInt8MultiArray][uint8multiarray]
`set_ascii` | 4桁のLEDを設定する。1〜4バイトを受け入れ、各バイトは左から右に表示されるASCII文字を表す | [std_msgs/UInt8MultiArray][uint8multiarray]
`dock` | デモのドッキング動作をアクティブにする。ロボットは_受動_モードに入り、ユーザーの制御が失われる（詳細は[OI Spec][oi_spec]を参照） | [std_msgs/Empty][empty]
`undock` | ロボットを_フル_モードに切り替え、ユーザーに制御を戻す | [std_msgs/Empty][empty]
`define_song` | 最大16音符の曲を定義する。各音符はMIDI音符番号と秒単位のfloat32持続時間で記述される。最長の持続時間は255/64秒。最大4曲を定義できる（詳細は[OI Spec][oi_spec]を参照） | [create_msgs/DefineSong][definesong_msg]
`play_song` | 事前定義された曲を再生する | [create_msgs/PlaySong][playsong_msg]
`side_brush_motor` | サイドブラシのデューティサイクルを設定する。-1.0から1.0の範囲を受け入れる | [create_msg/MotorSetpoint][motorsetpoint_msg]
`main_brush_motor` | メインブラシのデューティサイクルを設定する。-1.0

から1.0の範囲を受け入れる | [create_msg/MotorSetpoint][motorsetpoint_msg]
`vacuum_motor` | バキュームのデューティサイクルを設定する。0.0から1.0の範囲を受け入れる | [create_msg/MotorSetpoint][motorsetpoint_msg]

## Createの制御

ロボットを動かすためには、[geometry_msgs/Twist][twist]メッセージを`cmd_vel`トピックに送信する:

```
linear.x  (+)     前進 (m/s)
          (-)     後退 (m/s)
angular.z (+)     反時計回りに回転 (rad/s)
          (-)     時計回りに回転 (rad/s)
```

![Controlling a Roomba with cmd](img/1.png)

#### 速度制限

` -0.5 <= linear.x <= 0.5` および `-4.25 <= angular.z <= 4.25`

## テレオペレーション

#### 前提条件
1. PS4コントローラ用の必要なパッケージをインストールする
    ```bash
    sudo apt-get install ros-noetic-joy
    ```
2. PS4コントローラを接続する
    ```bash
    rosrun joy joy_node
    ```
3. PS4コントローラが接続されていることを確認する
    ```bash
    rostopic echo /joy
    ```

    ![Using a PS4 Controller to Operate ROS](img/ps4_ctrl1.png)

    ![Using a PS4 Controller](img/ps4_ctrl2.png)

次のPythonプログラムはPS4コントローラのみをサポートしています。
```bash
python3 ps4_talk.py
```

![Controlling a Roomba with Topic](img/Topic.png)

#### 以下のスクリプトをシェルスクリプトとしてRaspberry Piの起動時に実行するためにcronを使用することを推奨します。
```bash
#!/bin/bash

# ワークスペース
cd /path/to/roomba_ws

# ROSのセットアップ
source devel/setup.bash

# create_bringupノードの起動
roslaunch create_bringup create_2.launch &

# rosbridge_serverの起動
roslaunch rosbridge_server rosbridge_websocket.launch &

# Pythonスクリプトの起動
python3 /home/username/moongo4.py &

```
