#ROS launchファイルの使い方
このセクションではプロジェクトが大きくなり，rosrunによるノードの単体起動が困難になってきた際に用いるlaunchファイルにについて学びます。

ROSのlaunchファイルの特徴

- 各パッケージ内にlaunchというディレクトリを作成し，拡張子が.launchというファイルで定義する
- roslaunchというコマンドで対象のlaunchファイルを指定することでいっせいにノードが起動する
- launchファイルはXML形式で記述し各タグを用いて記述する
- 複数のノードの起動設定を記述できる
- 起動対象のノードが等しいものでも，起動する際の名前空間を変えることで，異なるノードして起動がきる
- 引数の付与が可能
- Topicやノードのリネームが可能
- roscoreが起動していない場合は自動的にroscoreも起動する


まずは実際のlaunchファイルについて見てみましょう。以下のlaunchファイルに注目しながらもっとも基本的なタグについて解説します。

```xml
<launch>
  <group ns="image_tutorial1">
    <node pkg="image_tutorial" name="input_node" type="subscriber"/>
    <node pkg="image_tutorial" name="output_node" type="publisher"/>
  </group>
  <group ns="image_tutorial2">
    <node pkg="image_tutorial" name="input_node" type="subscriber"/>
    <node pkg="image_tutorial" name="output_node" type="publisher"/>
  </group>
  <node pkg="image_tutorial" name="output_node" type="subscriber">
    <remap from="image_data" to="data_stack"/>
  </node>
  <node pkg="image_tutorial" name="input_node" type="publisher">
    <remap from="image_data" to="data_stack"/>
  </node>
</launch>
```

###launchタグ
`<launch></launch>`というタグで一斉起動するノード群の定義を行います。
```xml
<launch>
	<!-- your_definition -->
</launch>
```

###nodeタグ

各ノードの定義は以下のような書式で行います。

```xml
<node pkg="package_name" name="node_name" type="node_type"/>
```
各属性の意味は以下のとおりです。

- package_name : パッケージ名
- node_name : ROSに登録するノード名
- node_type : ノードの実行ファイル名

###Topicのリネーム
ノードのトピックのリネームをします。

```xml
<node pkg="package_name" name="node_name" type="node_type">
  <remap from="original_name" to="new_name"/>
  ...
</node>
```
- original_name  ： 変更前の名前
- new_name       ： 変更後の名前

###groupタグ
ノードを起動する名前空間の指定をします。この設定を行うことによって，同じノード名でも名前空間が違うため1つのroscore上に存在できます。

```xml
<group ns="name_space">
  <!-- node_definition -->
</group>
```

- name_space : 名前空間の定義名

###launchファイルの実行

先に示したlaunchファイルを実行してみましょう。  
まず、以下のコマンドでパッケージ内にlaunchファイルを入れるためのディレクトリを作成し，ファイルを作成しましょう。

```
cd ~/ros_ws/src/image_tutorial
mkdir launch; cd launch
touch run_image_tutorial.launch
```

エディタを開き，`run_image_tutorial.launch`を上で示した内容をコピーしましょう。

roslaunchで起動します。

```
cd ~/ros_ws
source devel/setup.bash
roslaunch image_tutorial run_image_tutorial.launch
```

起動に成功する以下のような出力がされます。

```
... logging to /home/kazushi/.ros/log/0d3b03f8-da40-11e5-982b-00270e39a558/roslaunch-degin-13220.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://degin:39936/

SUMMARY
========

PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.13

NODES
  /
    input_node (image_tutorial/publisher)
    output_node (image_tutorial/subscriber)
  /image_tutorial1/
    input_node (image_tutorial/subscriber)
    output_node (image_tutorial/publisher)
  /image_tutorial2/
    input_node (image_tutorial/subscriber)
    output_node (image_tutorial/publisher)

ROS_MASTER_URI=http://localhost:11311

core service [/rosout] found
process[image_tutorial1/input_node-1]: started with pid [13238]
process[image_tutorial1/output_node-2]: started with pid [13254]
process[image_tutorial2/input_node-3]: started with pid [13265]
process[image_tutorial2/output_node-4]: started with pid [13281]
process[output_node-5]: started with pid [13292]
process[input_node-6]: started with pid [13308]
```

##課題

- rostopicやrosnodeで実際にノードやTopicなどがリネームされているか確認しましょう。

###参考
[複数のノードを同時に実行 (ROS Launch)](http://robotics.naist.jp/edu/text/?Robotics%2FROS%2FLunchMultiNodes)  
[ja/roslaunch/XML - ROS Wiki](http://wiki.ros.org/ja/roslaunch/XML)  
[大きなプロジェクトにおける roslaunch の tips](http://wiki.ros.org/ja/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects)