# Gazeboシミュレータの紹介
このセクションはROSのロボットソフトウェア開発において一般的に用いられる**Gazeboシミュレータ**について紹介します。  
GazeboシミュレータはROSによって動作するロボットのための3Dモデルのバーチャルシミュレータです。Gazeboではシミュレーションできるロボットがすでにたくさんあります。Gazeboシミュレータの使用方法は大きく分けて2つ在ります。

1. すでに存在するロボットのシミュレーションモデルをインストールしてシミュレーション条件を変更して検証を行う。
- ロボットのモデル全て自分で定義してシミュレーションを行う

ここでは1においてturtlebotのシミュレーション環境の導入とシミュレータ起動の仕方を紹介します。

### 導入方法
以下のコマンドでシミュレーション環境を導入します。

```
sudo apt-get install ros-indigo-turtlebot-gazebo
sudo apt-get install ros-indigo-turtlebot-teleop
sudo apt-get install ros-indigo-turtlebot-rviz-launchers
```

### シミュレータの起動方法

```
source /opt/ros/indigo/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch
```
起動すると以下のような画面になります。

<img src="pic/gazebo_boot.png" width=500>

シミュレータ中のturtlebotを操作するため，操作ツールを起動します。  
新しいターミナルを開いて以下のコマンドを使用します。

```
source /opt/ros/indigo/setup.bash
roslaunch turtlebot_teleop keyboard_teleop.launch

...
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
```

turtlebotの操作の仕方は，以下のとおりです。キーボードで動かして見ましょう。  
シミュレータ画面のカメラの位置や角度はShift + マウスで調節できます。

```
   u：左前旋回  i：前進      o：右前旋回  
   j：左回転    k：ブレーキ  l：右回転  
   m：左後旋回  ,：後退      .：右後旋回  
```

### ロボットから得られる情報の可視化

シミュレータ起動中はロボットの仮想センサから得られる情報を視覚的に出力することができます。  
新しいターミナルを開いて以下のコマンドを使用します。

```
source /opt/ros/indigo/setup.bash
roslaunch turtlebot_rviz_launchers view_robot.launch
```
起動すると以下のような画面が起動します。

<img src="pic/rviz.png" width=500>

左横にあるリストの中でについて任意の要素についてチェックをつけるとセンサ情報が可視化され  
リアルタイムで出力されます。以下の例では**LaserScan**と**DepthCloud**のチェックを入れています。

<img src="pic/rviz_add.png" width=500>

### シミュレーション条件の変更

Gazeboシミュレータにおけるシミュレーション条件(障害物や仮想世界の環境)の変更は**worldファイル**の変更で行うことができます。シミュレーション条件の対象変更は環境変数を帰ることによって行えます。  
一度起動したシミュレータを全て停止させ，以下のコマンドを使用します。  
環境変数の`TURTLEBOT_GAZEBO_WORLD_FILE`を変更することによってシミュレーション条件を変更することができます。

```
echo $TURTLEBOT_GAZEBO_WORLD_FILE
/opt/ros/indigo/share/turtlebot_gazebo/worlds/playground.world
```
現在は**playground.world**というファイルが対象になっています。  
これを以下のコマンドで変更し，シミュレータを起動しなおします。

```
export TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/indigo/share/turtlebot_gazebo/worlds/corridor.world
roslaunch turtlebot_gazebo turtlebot_world.launch
```

起動すると先ほどとは違うシミュレーション条件になりました。

<img src="pic/gazebo_co.png" width=500>


## 参考
[Gazebo世界を探索してみる](http://wiki.ros.org/ja/turtlebot_simulator/Tutorials/hydro/Explore%20the%20Gazebo%20world)  
[第1回ROS勉強会発表資料 ROS+Gazeboではじめるロボットシミュレーション](http://www.slideshare.net/akio19937/1rosrosgazebo)