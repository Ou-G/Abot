#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& clock_msg) {
  static ros::NodeHandle nh("~");
  static ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/robot_0/cmd_vel", 10);
  float t = clock_msg->clock.toSec();
  ROS_INFO("clockCallback: %f", t);
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0.2;
  float amplitude = 2;
  float period = 5;
  nh.getParam("amplitude", amplitude);
  nh.getParam("period", period);
  vel_msg.angular.z = amplitude * sin(2 * M_PI * t / period);
  ROS_INFO("publish: %f", vel_msg.linear.x);
  cmd_pub.publish(vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_simple_talker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber clock_sub = nh.subscribe("/clock", 1, clockCallback);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}