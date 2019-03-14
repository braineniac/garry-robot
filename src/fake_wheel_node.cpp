#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

class FakeWheelNode {
  public:
    FakeWheelNode();

  private:
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher  fake_wheel_twist_pub;

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& twist_msg);
};

FakeWheelNode::FakeWheelNode() {
    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/arduino/cmd_vel", 1, &FakeWheelNode::cmd_vel_cb, this);
    fake_wheel_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/fake_wheel/twist", 1);
}

void FakeWheelNode::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& twist_msg) {

    geometry_msgs::TwistStamped fake_wheel_twist;

    fake_wheel_twist.header.stamp    = ros::Time::now();
    fake_wheel_twist.header.frame_id = "base_link";

    fake_wheel_twist.twist.linear.x  = twist_msg->linear.x * 0.837758;     // in m/s
    fake_wheel_twist.twist.angular.z = twist_msg->angular.z * 5.150152;    // in rad/s

    fake_wheel_twist_pub.publish(fake_wheel_twist);
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "fake_wheel_odom_node");

    FakeWheelNode fake_wheel_odom_node;

    while(ros::ok()) { ros::spin(); }
}
