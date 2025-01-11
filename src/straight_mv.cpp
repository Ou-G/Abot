#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include <cmath>

class StraightMove {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber goal_sub_;

    double current_x_, current_y_, current_yaw_;
    double goal_x_, goal_y_;
    bool odom_received_, goal_received_;

public:
    StraightMove() 
        : current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
          goal_x_(0.0), goal_y_(0.0),
          odom_received_(false), goal_received_(false) {
        
        // cmd_velパブリッシャー
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);

        // オドメトリサブスクライバー
        odom_sub_ = nh_.subscribe("/robot_1/odom", 10, &StraightMove::odomCallback, this);

        // ゴールサブスクライバー
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &StraightMove::goalCallback, this);
    }

    // オドメトリのコールバック
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch;
        tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

        odom_received_ = true;
    }

    // ゴールのコールバック
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;

        ROS_INFO("Goal received: x = %.2f, y = %.2f", goal_x_, goal_y_);
        goal_received_ = true;
    }

    // ゴールに移動する
    void moveToGoal() {
        ros::Rate rate(10);

        while (ros::ok() && !odom_received_) {
            ROS_INFO("Waiting for odometry data...");
            ros::spinOnce();
            rate.sleep();
        }

        while (ros::ok()) {
            if (!goal_received_) {
                ROS_INFO("Waiting for goal...");
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            double distance = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
            double angle_to_goal = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
            double angle_error = angle_to_goal - current_yaw_;

            geometry_msgs::Twist cmd_msg;

            if (distance < 0.1) { // ゴールに到達
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = 0.0;
                cmd_vel_pub_.publish(cmd_msg);
                ROS_INFO("Goal reached!");
                goal_received_ = false; // 次のゴール待ちに戻る
                continue;
            }

            if (std::abs(angle_error) > 0.1) { // 回転が必要
                cmd_msg.angular.z = 0.5 * angle_error;
            } else { // 直進
                cmd_msg.linear.x = 0.2;
            }

            cmd_vel_pub_.publish(cmd_msg);
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "straight_move");

    StraightMove mover;
    mover.moveToGoal();

    return 0;
}


// #include "ros/ros.h"
// #include "geometry_msgs/Twist.h"
// #include "nav_msgs/Odometry.h"
// #include "tf/tf.h"
// #include <cmath>

// class StraightMove {
// private:
//     ros::NodeHandle nh_;
//     ros::Publisher cmd_vel_pub_;
//     ros::Subscriber odom_sub_;

//     double current_x_, current_y_, current_yaw_;
//     bool odom_received_;

// public:
//     StraightMove() : current_x_(0.0), current_y_(0.0), current_yaw_(0.0), odom_received_(false) {
//         cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);
//         odom_sub_ = nh_.subscribe("/robot_0/odom", 10, &StraightMove::odomCallback, this);
//     }

//     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
//         current_x_ = msg->pose.pose.position.x;
//         current_y_ = msg->pose.pose.position.y;

//         tf::Quaternion q(
//             msg->pose.pose.orientation.x,
//             msg->pose.pose.orientation.y,
//             msg->pose.pose.orientation.z,
//             msg->pose.pose.orientation.w
//         );
//         double roll, pitch;
//         tf::Matrix3x3(q).getRPY(roll, pitch, current_yaw_);

//         odom_received_ = true;
//     }

//     void moveToGoal(double goal_x, double goal_y) {
//         ros::Rate rate(10);
//         while (ros::ok() && !odom_received_) {
//             ROS_INFO("Waiting for odometry data...");
//             ros::spinOnce();
//             rate.sleep();
//         }

//         while (ros::ok()) {
//             double distance = std::sqrt(std::pow(goal_x - current_x_, 2) + std::pow(goal_y - current_y_, 2));
//             double angle_to_goal = std::atan2(goal_y - current_y_, goal_x - current_x_);
//             double angle_error = angle_to_goal - current_yaw_;

//             geometry_msgs::Twist cmd_msg;

//             if (distance < 0.1) { // ゴールに到達
//                 cmd_msg.linear.x = 0.0;
//                 cmd_msg.angular.z = 0.0;
//                 cmd_vel_pub_.publish(cmd_msg);
//                 ROS_INFO("Goal reached!");
//                 break;
//             }

//             if (std::abs(angle_error) > 0.1) { // 回転が必要
//                 cmd_msg.angular.z = 0.5 * angle_error;
//             } else { // 直進
//                 cmd_msg.linear.x = 0.2;
//             }

//             cmd_vel_pub_.publish(cmd_msg);
//             ros::spinOnce();
//             rate.sleep();
//         }
//     }
// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "straight_move");

//     StraightMove mover;

//     ros::NodeHandle nh("~");

//     double goal_x = 2.0;
//     double goal_y = 2.0;
//     nh.getParam("goal_x", goal_x);
//     nh.getParam("goal_y", goal_y);
//     mover.moveToGoal(goal_x, goal_y); // 目標位置 (x=2.0, y=2.0)

//     return 0;
// }
