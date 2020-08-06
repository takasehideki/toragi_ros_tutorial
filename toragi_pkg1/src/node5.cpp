/*
 * toragi_pkg1パッケージのnode5の実装
 *   node5を出版者として生成し，topic3にデータを送信する
 *   topic3は独自定義のHuman型
 */
#include "ros/ros.h"
/* 今回定義したHuman型のヘッダ・ファイルをインクルード */
#include "toragi_pkg1/Human.h"

#include <iostream>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node5");

  ros::NodeHandle nh;
  /* Human型のtopic3に対して出版者としての登録を行う */
  ros::Publisher pub = nh.advertise<toragi_pkg1::Human>("topic3", 1000);

  ros::Rate loop_rate(0.2);

  while (ros::ok())
  {
    /* Human型の各データを標準入力から取得 */
    toragi_pkg1::Human msg;
    cout << "Enter Name [str]: " << endl;
    cin >> msg.name;
    cout << "Enter Height [int/cm]: " << endl;
    cin >> msg.height;
    cout << "Enter Weight [float/kg]: " << endl;
    cin >> msg.weight;

    /* デバッグ用の出力 */
    ROS_INFO("published info:");
    ROS_INFO("  name: %s height: %d weight: %.2f", msg.name.c_str(), msg.height, msg.weight);

    /* topic3に対してHuman型のデータを出版 */
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}