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
catkin_create_pkg ros_adder roscpp rospy std_msgs
```

この場合、パッケージの名前はros_adder、また依存関係を持たせるパッケージはroscpp、std_msgs，sensor_msgsということになります。
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
cd ~/ros_ws/src/ros_adder
mkdir msg; cd msg
touch Adder.msg
emacs Adder.msg
```

メッセージファイル***Adder.msg***の中には以下の記述をしてください。
このmsgファイルでは32bitのsigned int型の変数を1個，sensor_msgs/Image型の変数を1個保有していることになります。

**Adder.msg**

```
int32 arg_x
int32 arg_y
```

このメッセージファイルを元にメッセージ型を定義するヘッダファイルが生成されます。
ヘッダファイル生成の設定を行うため以下のファイルを編集してください。

```
 cd ~/ros_ws/src/ros_adder
 emacs CMakeLists.txt
```

**CMakeLists.txt**

```diff
#該当意部分がはコメント解除して適宜修正
#7行目あたり
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
+ message_generation
)

#45行目あたり
## Generate messages in the 'msg' folder
add_message_files(
	 FILES
- #   Message1.msg
- #   Message2.msg
+    Adder.msg
 )

#66行目あたり
generate_messages(
	 DEPENDENCIES
	 std_msgs
)

#104行目あたり
catkin_package(
#　 INCLUDE_DIRS include
	LIBRARIES ros_adder
	CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS system_lib
)

```
catkin_makeしましょう。

```
cd ~/ros_ws
catkin_make
```

catkin_makeに成功すると以下のディレクトリにメッセージを定義したPythonコードが生成されます。

```
ls ~/ros_ws/devel/lib/python2.7/dist-packages/ros_adder/msg/
_Adder.py  __init__.py
```
### ROSのノードを記述する
ROSのノードをPythonで記述していきます。
```
cd ~/ros_ws/src/ros_adder/
mkdir scripts; cd scripts
touch para_in.py adder.py
```
- para_in.py：処理に必要なデータをPublishするPublisher
- adder.py：publisherから受け取ったデータを表示するSubscriber

####Publisherを作る
以下に示すコードがPublisherとなります。
なお、コード中のAPIの説明などはコメントによって記しています。  
**para_in.py**

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from ros_adder.msg import Adder

def para_in():
	# 初期化宣言 : このソフトウェアは"para_in"という名前
	rospy.init_node('para_in', anonymous=True)

	# nodeの宣言 : publisherのインスタンスを作る
	# input_dataというtopicにAdder型のmessageを送るPublisherをつくった
	pub = rospy.Publisher('input_data', Adder, queue_size=100)

	# 1秒間にpublishする数の設定
	r = rospy.Rate(5)

	para_x = 0
	para_y = 2

	# Adder型のmessageのインスタンスを作る
	msg = Adder()

	# ctl +　Cで終了しない限りwhileループでpublishし続ける

	while not rospy.is_shutdown():

		msg.arg_x = para_x
		msg.arg_y = para_y

		# publishする関数
		pub.publish(msg)
		print "published arg_x=%d arg_y=%d"%(msg.arg_x,msg.arg_y)
		para_x += 1
		para_y += 1

		r.sleep()

if __name__ == '__main__':
	try:
			para_in()

	except rospy.ROSInterruptException: pass
```

#### Subscriberをつくる
以下に示すコードがSubscriberとなります。
なお、コード中のAPIの説明などはコメントによって記し、Publisherと同じ部分のコメントは省いています。

**adder.py**

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from ros_adder.msg import Adder

# Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
# 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
def callback(data):
	# 受けとったmessageの中身を足し算して出力
	print data.arg_x + data.arg_y

def adder():
	rospy.init_node('adder', anonymous=True)

	# Subscriberとしてimage_dataというトピックに対してSubscribeし、トピックが更新されたときは
  # callbackという名前のコールバック関数を実行する
	rospy.Subscriber('input_data', Adder, callback)

	# トピック更新の待ちうけを行う関数
	rospy.spin()

if __name__ == '__main__':
	adder()
```


2つのファイルの記述が終わったら、ビルドしましょう。

```
cd ~/ros_ws/
catkin_make
```

## ROSで作ったノードを実行してみる
ビルドが成功したら、さっそく実行してみましょう。
現在開いているコンソールほかにもう2つのコンソールを開き以下のコマンドを上からそれぞれ入力してください。

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
rosrun ros_adder adder.py
```

**3つ目のコンソール**  
**publisher**を起動します。
```
cd ~/ros_ws
source devel/setup.bash
rosrun ros_adder para_in.py
```

起動に成功したら以下のような結果が得られます。  
実行を止めたいときは**Ctrl + C**で止まります。

```
root@localhost:~/ros_ws# rosrun ros_adder para_in.py
published arg_x=0 arg_y=2
published arg_x=1 arg_y=3
published arg_x=2 arg_y=4
published arg_x=3 arg_y=5
published arg_x=4 arg_y=6
published arg_x=5 arg_y=7

root@localhost:~/ros_ws# rosrun ros_adder adder
2
4
6
8
10
12
14
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