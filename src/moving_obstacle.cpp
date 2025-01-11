#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    // ROSノードの初期化
    ros::init(argc, argv, "move_straight");
    ros::NodeHandle nh("~");

    // /cmd_velトピックのPublisherを作成
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

    // 前進速度と移動距離を設定
    double linear_speed = 0.2;  // 前進速度 (m/s)
    double distance = 6.0;     // 移動距離 (m)
    double time_to_travel = distance / linear_speed;  // 移動に必要な時間 (秒)

    geometry_msgs::Twist move_cmd;
    move_cmd.linear.x = linear_speed;  // 前進速度を設定
    move_cmd.angular.z = 0.0;         // 回転速度を0に設定

    ros::Rate rate(10);  // 制御ループの周期 (10Hz)

    // 現在時刻を取得して開始時間を記録
    ros::Time start_time = ros::Time::now();

    // 移動指令を一定時間送信
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < time_to_travel) {
        cmd_vel_pub.publish(move_cmd);  // 速度指令を送信
        rate.sleep();                   // 制御ループを待機
    }

    // 停止指令を送信
    move_cmd.linear.x = 0.0;  // 前進速度を0に設定
    cmd_vel_pub.publish(move_cmd);

    ROS_INFO("TurtleBot3 has moved 6 meters and stopped.");
    return 0;
}
