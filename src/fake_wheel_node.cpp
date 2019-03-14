#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

class FakeWheelNode {
  public:
    FakeWheelNode();
    void fake_wheel_pub();
  private:
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher  fake_wheel_twist_pub;

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& twist_msg);
		geometry_msgs::TwistWithCovarianceStamped current_msg;
};

FakeWheelNode::FakeWheelNode() {
    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/arduino/cmd_vel", 1, &FakeWheelNode::cmd_vel_cb, this);
    fake_wheel_twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/fake_wheel/twist", 1);
}

void FakeWheelNode::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& twist_msg) {

    geometry_msgs::TwistWithCovarianceStamped fake_wheel_twist;

    fake_wheel_twist.header.stamp    = ros::Time::now();
    fake_wheel_twist.header.frame_id = "base_link";

    fake_wheel_twist.twist.twist.linear.x  = twist_msg->linear.x * 0.352941176;    // in m/s
    fake_wheel_twist.twist.twist.angular.z = twist_msg->angular.z * 5.150152;    // in rad/s

    fake_wheel_twist.twist.covariance[0] = 0.0;

    current_msg = fake_wheel_twist;
}

void FakeWheelNode::fake_wheel_pub() {
    fake_wheel_twist_pub.publish(current_msg);
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "fake_wheel_odom_node");

    FakeWheelNode fake_wheel_odom_node;
		ros::Rate r(10);

    while(ros::ok()) { 
				fake_wheel_odom_node.fake_wheel_pub();
				ros::spinOnce();
			  r.sleep();	
		}
}
