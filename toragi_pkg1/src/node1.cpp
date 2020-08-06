/*
 * toragi_pkg1パッケージのnode1の実装
 *   node1を出版者として生成し，topic1にデータを送信する
 *   topic1は文字列型String
 *   コマンドライン引数で送信回数を指定できる
 */
/* 最も標準的な機能が含まれるヘッダ・ファイルをインクルード */
#include "ros/ros.h"
/* ROSの標準的な型のヘッダ・ファイルをインっクルード */
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  /* ROS環境の初期化 */
  /* 第1引数と第2引数はコマンドライン引数を引き継ぎ，第3引数ではノード名をnode1に指定 */
  ros::init(argc, argv, "node1");

  /* ノード管理のためのオブジェクトnhを用意 */
  ros::NodeHandle nh;
  /* topic1という名前のトピックを作成． */
  /* その出版者としてノードをros masterに登録 */
  ros::Publisher pub = nh.advertise<std_msgs::String>("topic1", 1000);

  /* ※設定ループの実行頻度を設定．単位はHzで，この例だと5秒． */
  ros::Rate loop_rate(0.2);
  /* ループ内のカウント変数 */
  int count = 0;
  /* 自身のノード名を取得 */
  std::string node_name = ros::this_node::getName();

  while (ros::ok())
  {
    /* 出版するデータの文字列を作成 */
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello from " << node_name << " count=" << count;
    msg.data = ss.str();

    /* デバッグ用出力 */
    ROS_INFO("%s", msg.data.c_str());

    /* 登録したtopic1に対してnode1が作成した文字列を出版 */
    pub.publish(msg);

    /* ROSのイベントの発生やLinuxのシグナル発行などがあるかを確認． */
    /* なにもなければ処理を続ける */
    ros::spinOnce();
    /* この行が実行されてから上記※の行(27行目)で設定された時間が経過するまでスリープ */
    loop_rate.sleep();

    /* コマンドライン引数が与えられているとき，その回数だけループを繰り返したら， */
    /* ノードの出版処理を終了．ROSの実行も終了． */
    if ((argc >= 2) && (count >= atoi(argv[1])))
    {
      pub.shutdown();
      ros::shutdown();
    }
    else
    {
      ++count;
    }
  }

  return 0;
}