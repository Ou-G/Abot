#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
    // ROSノードの初期化
    ros::init(argc, argv, "goal_publisher");
    ros::NodeHandle nh;

    // パブリッシャーの作成
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_0/goal", 10);

    // メッセージの作成
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";  // フレームIDを指定（例: "map"）
    goal_msg.header.stamp = ros::Time::now();  // タイムスタンプを現在時刻に設定
    goal_msg.pose.position.x = 2.0;  // x座標
    goal_msg.pose.position.y = 6.0;  // y座標
    goal_msg.pose.position.z = 0.0;  // z座標（平面上なので0）
    goal_msg.pose.orientation.x = 0.0;  // 回転は初期値（向きは必要に応じて設定）
    goal_msg.pose.orientation.y = 0.0;
    goal_msg.pose.orientation.z = 0.707;
    goal_msg.pose.orientation.w = 0.707;

    // パブリッシュのループ
    ros::Rate loop_rate(1);  // 1 Hzで送信
    while (ros::ok())
    {
        goal_msg.header.stamp = ros::Time::now();  // タイムスタンプを更新
        ROS_INFO("Publishing goal: x=%.2f, y=%.2f", goal_msg.pose.position.x, goal_msg.pose.position.y);
        goal_pub.publish(goal_msg);  // メッセージの送信
        loop_rate.sleep();  // 待機
    }

    return 0;
}
