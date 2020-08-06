# toragi_ros_tutorial

CQ出版社 [トランジスタ技術2020年9月号](https://toragi.cqpub.co.jp/tabid/918/Default.aspx) **特集「ロボット1日開発 初めてのROS＆位置推定」** に掲載されている  
- [第3章: 作ってわかる！ROSの通信機能のしくみ](https://toragi.cqpub.co.jp/Portals/0/backnumber/2020/09/p050.pdf)
のサンプルのソースコードです．

ソースコードの差分やステップごとの進め方は [commit log](https://github.com/takasehideki/toragi_ros_tutorial/commits/master) をご参照ください．    
また，記事の解説ごとにブランチを作成しているので，適宜切り替えて参照してください．

- [step1: 1対1でノード間の通信を実行するパッケージを作成する](https://github.com/takasehideki/toragi_ros_tutorial/tree/step1)
- [step2: 複数のトピックに対して出版する](https://github.com/takasehideki/toragi_ros_tutorial/tree/step2)
- [step3: 独自定義の型を利用する](https://github.com/takasehideki/toragi_ros_tutorial/tree/step3)
- [step4: 出版と購読の両方を行うノードを実装する](https://github.com/takasehideki/toragi_ros_tutorial/tree/step4)
- [step5: すべてのノードを同時に起動する](https://github.com/takasehideki/toragi_ros_tutorial/tree/step5)

## 事前の準備と前提条件

Ubuntu 18.04 / [ROS Melodic Morenia](http://wiki.ros.org/melodic) の環境が構築済みであり，作業用のROSワークスペースは `~/catkin_ws` としています．

- ROSワークスペースの作成

```bash
$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

- 設定の確認

```bash
$ env | grep ROS
ROS_ETC_DIR=/opt/ros/melodic/etc/ros
ROS_ROOT=/opt/ros/melodic/share/ros
ROS_MASTER_URI=http://localhost:11311
ROS_VERSION=1
ROS_PYTHON_VERSION=2
ROS_PACKAGE_PATH=/opt/ros/melodic/share
ROSLISP_PACKAGE_DIRECTORIES=
ROS_DISTRO=melodic
```

- リポジトリのClone(任意)

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/takasehideki/toragi_ros_tutorial
```

## ライセンス

[MIT License](https://github.com/takasehideki/toragi_ros_tutorial/blob/master/LICENSE)

## 質問など

記事や本リポジトリの内容に質問がありましたら，[本リポジトリのIssues](https://github.com/takasehideki/toragi_ros_tutorial/issues)でお知らせください．些細なことでもなんでも対応いたします．
