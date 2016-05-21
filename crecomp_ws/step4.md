#### Step4：追加課題

##### 課題1　ファイル入出力を行うROSパッケージを作る
以下の仕様のROSパッケージを作ってください。

| 作成するもの |  ファイル名  |                                            仕様                                           |
|--------------|--------------|-------------------------------------------------------------------------------------------|
| Publisher    | file_in.cpp  | input.txtからファイル入力(リダイレクト禁止)、`file_data`というトピックへメッセージをPublish |
| Subscriber   | file_out.cpp | `file_data`からデータを購読し，output.txtへ出力                                             |

- 作成するディレクトリは`~/workshop_crecomp/ros_ws/src/openreroc_motionsensor/src`内に作成してください。
- メッセージファイルはStep3で作成したsensor_data.msgを使用してください。
- sensor_data.msgを使用するには`#include openreroc_motionsensor/sensor_data.h`を定義する必要があります。
- ノードの記述が終了したらCMakeLists.txtの末尾に以下のように追加してください。

```diff
+ add_executable(openreroc_motionsensor src/openreroc_motionsensor.cpp)
+ target_link_libraries(openreroc_motionsensor ${catkin_LIBRARIES})
+ add_executable(sample_output src/sample_output.cpp)
+ target_link_libraries(sample_output ${catkin_LIBRARIES})
```

以下のコードはROSのライブラリの解説がされています。(課題の仕様にはなっていません)  
Publisherの例
```cpp
// ros/ros.h　ROSに関する基本的なAPIのためのヘッダ
#include "ros/ros.h"
// comp_tutrial/adder.h　adder.msgから生成されたメッセージを定義しているヘッダ
#include "comp_tutorial/adder.h"

int main(int argc, char **argv)
{
  // 初期化のためのAPI
  // このノードは"para_in"という名前であるという意味
  ros::init(argc, argv, "para_in");

  // ノードハンドラの宣言
  ros::NodeHandle n;

  //Publisherとしての定義
  // n.advertise<comp_tutorial::adder>("para_input", 1000);
  // comp_tutorial::adder型のメッセージをpara_inputというトピックへ配信する
  //"1000"はトピックキューの最大値
  ros::Publisher para_pub = n.advertise<comp_tutorial::adder>("para_input", 1000);

  //1秒間に1つのメッセージをPublishする
  ros::Rate loop_rate(1);

  //comp_tutrial::adder型のオブジェクトを定義
  //adder.msgで定義したa,bはメンバ変数としてアクセスできる
  comp_tutorial::adder msg;

  int count = 0;
  while (ros::ok())//ノードが実行中は基本的にros::ok()=1
  {
    msg.a = count;
    msg.b = count;
    para_pub.publish(msg);//PublishのAPI
    printf("a = %d b = %d \n",msg.a , msg.b );
    ros::spinOnce();
    loop_rate.sleep();
    count++;
  }
  return 0;
```
Subscriberの例
```cpp
#include "ros/ros.h"
#include "comp_tutorial/adder.h"

// Subscribeする対象のトピックが更新されたら呼び出されるコールバック関数
// 引数にはトピックにPublishされるメッセージの型と同じ型を定義する
void chatterCallback(const comp_tutorial::adder msg)
{
    int result;
    result = msg.a + msg.b;
    printf("a:%d + b:%d = %d\n",msg.a , msg.b, result );
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "adder");
  ros::NodeHandle n;

  // Subscriberとしてpara_inputというトピックがSubscribeし、トピックが更新されたときは
  // chatterCallbackという名前のコールバック関数を実行する
  ros::Subscriber sub = n.subscribe("para_input", 1000, chatterCallback);

  // トピック更新の待ちうけを行うAPI
  ros::spin();

  return 0;
}
```

##### 課題2　ソフトウェアインターフェイスを作る
cReCompではソフトウェアインターフェイスは自動生成でしたが，本課題では自作していただきます。  
ユーザロジックとのデータ通信のためにはFIFOへのデバイスファイルのファイルアクセスを行う必要があります。  
以下のヒントを元にソフトウェアインターフェイスを作成してください。  
なお、ハードウェア(超音波センサの回路)はFIFOへ1を書き込むとセンサのデータが1回分返ってきます。  
また、FIFOへの読み書きする際のデータ方はunsigned intとします。

インクルードするヘッダ

```cpp
#include <fcntl.h>
#include <termio.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
```

デバイスファイルのオープンの仕方

```cpp
int fd;
//書き込み用のFIFO
fd = open("/dev/xillybus_write_32", O_WRONLY);

//読み込み用のFIFO
fd = open("/dev/xillybus_read_32", O_RDONLY);
```

write関数
```
write(ファイルポインタ,FIFOへ書き込むデータ変数のアドレス,書き込むデータのサイズ)
```

read関数
```
read(ファイルポインタ,FIFOからデータを読み出し、保存する変数アドレス,読み込むデータのサイズ);
```