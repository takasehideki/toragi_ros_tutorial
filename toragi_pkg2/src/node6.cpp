/*
 * toragi_pkg2パッケージのnode6の実装
 *   node6を，topic1の出版者およびtopic3の購読者として生成する
 *   topic1はString型
 *   topic3は独自定義のHuman型
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
/* Human型のヘッダ・ファイルをインクルード */
#include "toragi_pkg1/Human.h"

/* main関数とコールバック関数でやりとりするデータを大域変数として用意 */
std::string human_name;
float bmi = 0.0;
int flag = 0;

/* BMI値を計算するコールバック関数 */
void topic3Callback(const toragi_pkg1::Human::ConstPtr &msg)
{
  human_name = msg->name;
  ROS_INFO("I heard %s's height and weight", human_name.c_str());
  bmi = msg->weight / ((msg->height / 100.0) * (msg->height / 100.0));
  /* 購読と計算が完了できたことを示すフラグを立てる */
  flag = 1;
}

int main(int argc, char **argv)
{
  /* ROS環境の初期化を行ってnode6を生成 */
  ros::init(argc, argv, "node6");

  ros::NodeHandle nh;
  /* topicを生成するとともに，node6を出版者および購読者としてros masterに登録 */
  ros::Subscriber sub = nh.subscribe("topic3", 1000, topic3Callback);
  ros::Publisher pub = nh.advertise<std_msgs::String>("topic1", 1000);

  /* スリープの設定時間5秒 */
  ros::Rate loop_rate(0.2);

  while (ros::ok())
  {
    /* フラグが立っていたらString型のメッセージを作成 */
    if (flag)
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss << human_name << "'s BMI is " << bmi;
      msg.data = ss.str();

      /* デバッグ用の出力 */
      ROS_INFO("%s", msg.data.c_str());

      /* topic1に出版 */
      pub.publish(msg);

      /* フラグを下ろす */
      flag = 0;
    }

    /* ROSのイベントの発生やLinuxのシグナル発行などがあるかを確認． */
    /* なにもなければ処理を続ける */
    ros::spinOnce();
    /* 設定時間が経過するまでスリープ */
    loop_rate.sleep();
  }

  return 0;
}