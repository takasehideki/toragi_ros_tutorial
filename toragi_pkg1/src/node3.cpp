/*
 * toragi_pkg1パッケージのnode3の実装
 *   node3を出版者として生成し，topic1とtopic2にデータを送信する
 *   topic1,topic2は文字列型String
 *   コマンドライン引数で送信回数を指定できる
 */
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  /* ノード名をnode3に指定 */
  ros::init(argc, argv, "node3");

  ros::NodeHandle nh;

  ros::Publisher pub1 = nh.advertise<std_msgs::String>("topic1", 1000);
  /* topic2の登録を追加 */
  ros::Publisher pub2 = nh.advertise<std_msgs::String>("topic2", 1000);

  ros::Rate loop_rate(0.2);
  int count = 0;
  std::string node_name = ros::this_node::getName();

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello from " << node_name << " count=" << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    pub1.publish(msg);
    /* topic2の出版を追加 */
    pub2.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();

    if ((argc >= 2) && (count >= atoi(argv[1])))
    {
      pub1.shutdown();
      /* topic2の出版を終了 */
      pub2.shutdown();
      ros::shutdown();
    }
    else
    {
      ++count;
    }
  }

  return 0;
}