#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

class FakeEncoderNode {
  public:
    FakeEncoderNode();
    void fake_encoder_pub();
  private:
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher  fake_encoder_pub;

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& twist_msg);
		geometry_msgs::TwistWithCovarianceStamped current_msg;
};

FakeWheelNode::FakeWheelNode() {
    cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/arduino/cmd_vel", 1, &FakeWheelNode::cmd_vel_cb, this);
    fake_encoder_twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/fake_encoder/twist", 1);
}

void FakeEncoderNode::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& twist_msg) {

    geometry_msgs::TwistWithCovarianceStamped fake_encoder_twist;

    fake_encoder_twist.header.stamp    = ros::Time::now();
    fake_encoder_twist.header.frame_id = "base_link";

    fake_encoder_twist.twist.twist.linear.x  = twist_msg->linear.x;
    fake_encoder_twist.twist.twist.angular.z = twist_msg->angular.z;

    current_msg = fake_encoder_twist;
}

void FakeEncoderNode::fake_encoder_pub() {
    fake_encoder_twist_pub.publish(current_msg);
}

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "fake_encoder_node");

    FakeEncoderNode fake_encoder_node;
    ros::Rate r(10);

    while(ros::ok()) {
        fake_encoder_node.fake_encoder_pub();
        ros::spinOnce();
        r.sleep();
    }
}
