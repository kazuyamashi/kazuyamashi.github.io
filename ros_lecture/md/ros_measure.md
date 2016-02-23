#ROSにおけるノード間通信の分析に<br>使うコマンド紹介とその実用例
このセクションではROSにおいて各パッケージの動作状況の分析やデバッグをするのに  
便利なコマンドの紹介と実用例を紹介します。  

###rosgraph
**rosgraph**は現在動作しているノードの情報を一挙に表示するためのコマンドです。  
実行すると以下のような出力がされます。  
以下の例では動作しているノードが**subscriber**，**rosout**，**publisher**の3つです。  
たとえば，**subscriber**であればInbound方向にPublisherからデータを受け取っていることになります。  

**コマンド書式**  
`rosgraph`  

**オプションなし**  

**実際の使い方例**  
```
rosgraph
Nodes:
  /subscriber :
    Inbound:
      /publisher
    Outbound:
      /rosout
  /rosout :
    Inbound:
      /publisher
      /subscriber
    Outbound:
  /publisher :
    Inbound:
    Outbound:
      /rosout
      /subscriber
```
またrosgraphと同じような使い方のできるrqt_graphもあります。  
このツールはGUI上でグラフィカルに動作中のノードやトピックの関係を表示できます。

<img src="pic/rosgraph.png">


###rostopic

**rostopic**は現在動作中の全てのTopicについてさまざまなことを調べることができます。  
任意のノード同士がどのように通信をしているか調べるときに非常に有効なコマンドです。  

**コマンド書式**  
`rostopic option topic_name`

**オプション一覧**
```
rostopic bw     任意のTopicのバンド幅を表示
rostopic echo   任意のTopicに配信されているメッセージ内容の表示
rostopic find   メッセージタイプでTopicを見つける
rostopic hz     任意のTopicの配信頻度を表示
rostopic info   任意のアクティブなTopicの情報を表示
rostopic list   アクティブなToipcの一覧を表示
rostopic pub    任意のノードにメッセージをpubulishする
rostopic type   Topicのメッセージタイプを表示
```

**実際の使い方例**  
アクティブなTopicを調べ，特定のTopicについてメッセージタイプとバンド幅を調べる。  
 `list→info→bw`
```
kazushi@degin:~$ rostopic list
/image_data
/rosout
/rosout_agg

kazushi@degin:~$ rostopic info image_data
Type: image_tutorial/image

Publishers:
 * /publisher (http://degin:42608/)

Subscribers:
 * /subscriber (http://degin:44576/)

kazushi@degin:~$ rostopic bw image_data
subscribed to [/image_data]
average: 504.44KB/s
        mean: 307.25KB min: 307.25KB max: 307.25KB window: 2
average: 415.31KB/s
        mean: 307.25KB min: 307.25KB max: 307.25KB window: 3
average: 381.60KB/s
        mean: 307.25KB min: 307.25KB max: 307.25KB window: 4
average: 324.38KB/s
        mean: 307.25KB min: 307.25KB max: 307.25KB window: 4
```


###rosnode
**rostopic**は現在動作中の全てのノードについてさまざまなことを調べることができます。  
ノードの単体デバッグやノード情報の取得などに便利です。

**コマンド書式**  
`rostopic option topic_name`

**オプション一覧**
```
rosnode ping    任意のノードが動作しているかどうか確認する(pingの送信)
rosnode list    アクティブなノード一覧を表示
rosnode info    任意のノードの情報を表示
rosnode machine ROSのノードが動作しているマシン名の一覧を表示
rosnode kill    任意のnodeの動作停止
rosnode cleanup nodeリストのクリーンアップ(動作していないノードの登録解除)
```

**実際の使い方例**  
アクティブなノード一覧を取得し，特定のノード情報について調べそのノードの動作を確認する  
`list→info→ping`
```
kazushi@degin:~$ rosnode list
/publisher
/rosout
/subscriber

kazushi@degin:~$ rosnode info publisher
--------------------------------------------------------------------------------
Node [/publisher]
Publications:
 * /rosout [rosgraph_msgs/Log]
 * /image_data [image_tutorial/image]

Subscriptions: None

Services:
 * /publisher/set_logger_level
 * /publisher/get_loggers


contacting node http://degin:57164/ ...
Pid: 12554
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /image_data
    * to: /subscriber
    * direction: outbound
    * transport: TCPROS

kazushi@degin:~$ rosnode ping publisher
rosnode: node is [/publisher]
pinging /publisher with a timeout of 3.0s
xmlrpc reply from http://degin:57164/   time=0.593901ms
xmlrpc reply from http://degin:57164/   time=0.894070ms
xmlrpc reply from http://degin:57164/   time=0.882864ms
```

###rospack

**rospack**は自分の作成したパッケージはもちろん，他の開発者が作成したパッケージについてのさまざまな情報を取得できます。  
オプションは大変多くあるので，今回はよく使用するオプションを紹介します。  

**コマンド書式**  
`rospack option package_name`

**オプション一覧**  
```
depends         直接、間接的なパッケージ依存を全て表示
depends1        直接的なパッケージ依存を表示
depends-indent  パッケージ依存をインデントがついた形で表示
list            インストールされているパッケージとインストールパスの一覧を表示
list-names      インストールされているパッケージの名前一覧を表示
```
**実際の使い方例**  
git cloneした他開発者のパッケージについての依存情報について調べ，足りないパッケージのインストールを行う。  

```
kazushi@degin:~/ros_ws/src$ git clone https://github.com/Kumikomi/openreroc_motion_sensor.git
Cloning into 'openreroc_motion_sensor'...
remote: Counting objects: 34, done.
remote: Total 34 (delta 0), reused 0 (delta 0), pack-reused 34
Unpacking objects: 100% (34/34), done.
Checking connectivity... done.

kazushi@degin:~/ros_ws/src$ rospack depends1 openreroc_motion_sensor
roscpp
std_msgs


kazushi@degin:~/ros_ws/src$ apt-cache search roscpp
ros-indigo-roscpp - roscpp is a C++ implementation of ROS. 
It provides a client library that enables C++ programmers to quickly interface with ROS Topics, 
Services, and Parameters. roscpp is the most widely used ROS client library 
and is designed to be the high-performance library for ROS.

sudo apt-get install ros-indigo-roscpp

```

###rosmsg
**コマンド書式**  
`rosmsg option package_name/message_file`

**オプション一覧**  

```
rosmsg show     メッセージに関する情報を表示
rosmsg list     インストールされているメッセージの型を全て表示
rosmsg md5      任意のメッセージタイプのmd5サムを表示
rosmsg package  任意のパッケージに含まれるメッセージタイプの一覧を表示
rosmsg packages インストールされているメッセージに関するパッケージを全て表示
```

**実際の使い方例**  
```
rosmsg package image_tutorial
image_tutorial/image
```

##課題
- 前のセクションで作成したimage_tutorialというパッケージのpublsherとsubscriberをそれぞれ起動しましょう。
- コマンドを使用してimage_tutorialについて調べてみましょう
    - image_tutorialの依存関係は？
    - publisherとsubscriberの使用しているトピック名とメッセージタイプは？
    - image_tutorialにおけるメッセージはどんな要素を持つ？
- rosnodeで起動中のpublsherとsubscriberを停止させましょう。