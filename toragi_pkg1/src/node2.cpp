/*
 * toragi_pkg1パッケージのnode2の実装
 *   node2を購読者として生成し，topic1からのデータを受信する
 *   topic1は文字列型String
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

/*
 * データの購読時に実行されるコールバック関数．
 * 自身のノード名を取得して，ノード名と購読した文字列を出力
 */
void topic1Callback(const std_msgs::String::ConstPtr &msg)
{
  std::string node_name = ros::this_node::getName();
  ROS_INFO("%s heard: [%s]", node_name.c_str(), msg->data.c_str());
}

int main(int argc, char **argv)
{
  /* ROS環境の初期化 */
  /* コマンドライン引数を引き継ぎ，第3引数でノード名を指定．node2が生成される */
  ros::init(argc, argv, "node2");

  /* ノード管理のためのオブジェクトnhを用意 */
  ros::NodeHandle nh;
  /* topic2という名前のトピックを作成し， */
  /* その購読者としてノードをros masterに登録． */
  /* データの購読時に実行されるコールバック関数も登録 */
  ros::Subscriber sub = nh.subscribe("topic1", 1000, topic1Callback);

  /* ROSのイベントの発生やLinuxのシグナル発行などを無限待ち */
  ros::spin();

  return 0;
}