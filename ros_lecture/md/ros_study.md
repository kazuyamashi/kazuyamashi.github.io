# ROSの基本的な開発

## ROSのワークスペースを作る
ROSのワークスペースを作ります。以下のコマンドを入力してください。
また、ROSに関するツールを使えるようにするため、パスを通しておきます。

```
mkdir -p ros_ws/src
cd ros_ws
source /opt/ros/indigo/setup.bash
```

ここで一度makeしておきます。ROSでは***catkin_make***というコマンドを使ってワークスペースのプロジェクトをビルドします。また、初回でこのコマンドを行えばワークスペースに必要なファイルやディレクトリが生成されます。

```
catkin_make
```

catkin_makeしたら以下のようなファイル階層になっていると思います。

```
ros_ws
|--build
|--devel
|--src
```

- build
 - ビルドに関する設定やmakeのlogのファイルが入っています。
- devel
 - 実行ファイルやmakeによって生成されたものが入っています。
- src
 - ユーザがソースコードを保存する場所です。

これでワークスペースはできました。

## ROSパッケージを作る

ROSでは任意の機能をつめたソフトウェアの集合を**パッケージ**と呼びます。
パッケージの雛型は***catkin_create_pkg***で作成することができます。また***catkin_create_pkg***のコマンド引数は以下のとおりです。

```
catkin_create_pkg [package_name] [depend1] [depends2] ...
```
コンソールに以下のコマンドを入力してください。

```
cd src
catkin_create_pkg image_tutorial roscpp std_msgs sensor_msgs
```

この場合、パッケージの名前はimage_tutorial、また依存関係を持たせるパッケージはroscpp、std_msgs，sensor_msgsということになります。
ワークスペースでもう一度catkin_makeしましょう。

```
cd ~/ros_ws/
catkin_make
```

## ROSのコーディング
ここではROSの具体的なコーディングを行います。
###ROS message
ROSでは処理に必要なデータは基本的にメッセージとして通信を行ないます。
そのメッセージのデータ構造はさまざまなデータ型から任意のものを選択して独自のデータ構造にすることができます。
たとえば、データ型には以下のものがあります。

- int8, int16, int32, int64 (plus uint*)
- float32, float64
- string
- time, duration
- other msg files
- variable-length array[] and fixed-length array[C]

またメッセージに関するクラスが定義されているヘッダーは`/opt/ros/indigo/include`内にあります。  
簡単なメッセージファイルを作ってみましょう。

```
cd ~/ros_ws/src/image_tutorial
mkdir msg; cd msg
touch image.msg
emacs image.msg
```

メッセージファイル***image.msg***の中には以下の記述をしてください。
このmsgファイルでは32bitのsigned int型の変数を1個，sensor_msgs/Image型の変数を1個保有していることになります。

**image.msg**

```
int32 frameID
sensor_msgs/Image img
```

このメッセージファイルを元にメッセージ型を定義するヘッダファイルが生成されます。
ヘッダファイル生成の設定を行うため以下のファイルを編集してください。

```
 cd ~/ros_ws/src/image_tutorial
 emacs CMakeLists.txt
 
```

**CMakeLists.txt**

```diff
#該当意部分がはコメント解除して適宜修正
#7行目あたり
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
+ message_generation
)

#45行目あたり
## Generate messages in the 'msg' folder
add_message_files(
   FILES
- #   Message1.msg
- #   Message2.msg
+    image.msg
 )

#66行目あたり
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)
```
catkin_makeしましょう。

```
cd ~/ros_ws
catkin_make
```

catkin_makeに成功すると以下のディレクトリにメッセージを定義したヘッダファイルが生成されます。

```
less ~/ros_ws/devel/include/image_tutorial/image.h
…
namespace image_tutorial
{
template <class ContainerAllocator>
struct image_
{
  typedef image_<ContainerAllocator> Type;

  image_()
    : frameID(0)
    , img()  {
    }
  image_(const ContainerAllocator& _alloc)
    : frameID(0)
    , img(_alloc)  {
    }



   typedef int32_t _frameID_type;
  _frameID_type frameID;

   typedef  ::sensor_msgs::Image_<ContainerAllocator>  _img_type;
  _img_type img;
…
```
### ROSのノードを記述する
ROSのノードをC++言語で記述していきます。
```
cd ~/ros_ws/src/image_tutorial/src
touch publisher.cpp subscriber.cpp
```
- publisher.cpp：処理に必要な データをPublishするPublisher
- subscriber.cpp：publisherから受け取ったデータを表示するSubscriber

####Publisherを作る
以下に示すコードがPublisherとなります。
なお、コード中のAPIの説明などはコメントによって記しています。  
**publisher.cpp**

```cpp
// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// image_tutorial/image.h　image.msgから生成されたメッセージを定義しているヘッダ
#include "image_tutorial/image.h"

#include <stdlib.h>
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
  // 初期化宣言
  // このノードは"publisher"という名前であるという意味
  ros::init(argc, argv, "publisher");
  // ノードハンドラの宣言
  ros::NodeHandle n;
  //　Publisherとしての定義
  // n.advertise<image_tutorial::image>("image_data", 1000);
  // image_tutorial::image型のメッセージをimage_dataというトピックへ配信する
  //"1000"はトピックキューの最大値
  ros::Publisher pub = n.advertise<image_tutorial::image>("image_data", 1000);
  //1秒間に1回の間隔でループする
  ros::Rate loop_rate(1);

  //image_tutorial::image型のオブジェクトを定義
  //image.msgで定義したflameID,imgはメンバ変数としてアクセスできる
  image_tutorial::image msg;

  // 変数imgはsensor_msgs/Image型である
  // この型はもともとメンバ変数を持った型なので以下のような使い方でアクセスできる
  msg.img.height = 480;
  msg.img.width  = 640;
  msg.img.encoding = "rgb8"; 
  msg.img.step =  msg.img.width;

  // sensor_msgs/Image型のデータ部にデータをプッシュバックしている
  //　実際はOpenCVなどで画像のRGB情報を得たあと，データを格納する
  for(int i = 0; i <  msg.img.height; i++){
    for (int j = 0; j < msg.img.width; j++){
      msg.img.data.push_back(0xFF);
    }
  }

  int frameid = 0;
  //ノードが実行中は基本的にros::ok() = 1
  // Ctrl + Cなどのインタラプトが起こるとros::ok() = 0となる
  while (ros::ok())
  {
    msg.frameID = frameid;
    // Publishする関数
    pub.publish(msg);
    cout << "published !" << endl;
    ros::spinOnce();
    frameid++;

    loop_rate.sleep();
  }
  return 0;
}
```

#### Subscriberをつくる
以下に示すコードがSubscriberとなります。
なお、コード中のAPIの説明などはコメントによって記し、Publisherと同じ部分のコメントは省いています。

**subscriber.cpp**

```cpp
#include "ros/ros.h"
#include <stdio.h>
#include "image_tutorial/image.h"
#include <iostream>
using namespace std;

// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void chatterCallback(const image_tutorial::image msg)
{
  cout << "height = " << msg.img.height <<
          " width = " << msg.img.width <<
          " frameID = " << msg.frameID << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n;
  // Subscriberとしてimage_dataというトピックに対してSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = n.subscribe("image_data", 1000, chatterCallback);

  // トピック更新の待ちうけを行う関数
  ros::spin();

  return 0;
}
```

2つのファイルの記述が終わったら、パッケージをビルドするために
CMakeLists.txtを編集します。
以下のコマンドでCMakeLists.txtを編集してください。

```
cd ~/ros_ws/src/image_tutorial
emacs CMakeLists.txt
```

**CMakeLists.txt**

```diff
#ファイルの末尾に追加
+ add_executable(publisher src/publisher.cpp)
+ target_link_libraries(publisher ${catkin_LIBRARIES})
+ add_executable(subscriber src/subscriber.cpp)
+ target_link_libraries(subscriber ${catkin_LIBRARIES})
```

編集し終わったらビルドしましょう。

```
cd ~/ros_ws/
catkin_make
```

## ROSで作ったノードを実行してみる
ビルドが成功したら、さっそく実行してみましょう。
現在開いているコンソールｎほかにもう2つのコンソールを開き以下のコマンドを上からそれぞれ入力してください。

**1つ目のコンソール**  
ROSでは***roscore***というコマンドを始めに起動することでさまざまなソフトウェアをスタートすることができます。
具体的にはroscoreはネームサービスなどを行います。
```
cd ~/ros_ws
source devel/setup.bash
roscore
```

**2つ目のコンソール**  
ROSでは各ノードの実行は***rosrun***というコマンドによって実行されます。
ROSにおいて単体のノード実行は基本的にrosrunで行います。
**subscriber**を起動します。
```
cd ~/ros_ws
source devel/setup.bash
rosrun image_tutorial subscriber
```

**3つ目のコンソール**  
**publisher**を起動します。
```
cd ~/ros_ws
source devel/setup.bash
rosrun image_tutorial publisher
```

起動に成功したら以下のような結果が得られます。  
実行を止めたいときは**Ctrl + C**で止まります。

```
root@localhost:~/ros_ws# rosrun image_tutorial para_in
published !
published !
published !
published !
published !
published !

root@localhost:~/ros_ws# rosrun image_tutorial adder
height = 480 width = 640 flameID = 1
height = 480 width = 640 flameID = 2
height = 480 width = 640 flameID = 3
height = 480 width = 640 flameID = 4
height = 480 width = 640 flameID = 5
height = 480 width = 640 flameID = 6
```

## コマンドまとめ
- catkin_make
    - ワークスペース内のパッケージを一括ビルドするコマンド
- catkin_create_pkg
    - ROSにおけるパッケージの雛形を作るコマンド
- roscore
    - ROSのネームサービス、マスタ
- rosrun
    - ROSの単体ノードを起動する際に使用するコマンド

##課題
以下の仕様のパッケージadder_tutorialを作りましょう。  
パッケージ内に含まれるノードの数は2つとします。

|  作るもの  |                                                      機能                                                      |   ファイル名    |
|------------|----------------------------------------------------------------------------------------------------------------|-----------------|
| Subscriber | input_dataというTopicに対してSubscribeする<br>入力値を2つ受け取り，足し算をして標準出力する<br>ノード名：adder | adder.cpp       |
| Publisher  | input_dataというTopicに対してinput_value型のメッセージを1秒に1回Publishする<br>ノード名：para_in               | para_in.cpp     |
| message    | int32型の変数を2つ持つ<br>input_valueと称す。                                                                  | input_value.msg |

ヒント  

- パッケージを作るときは`~/ros_ws/src`内で`catkin_create_pkg adder_tutorial roscpp std_msgs`としましょう。
- わからない点がある場合は積極的に質問してください。