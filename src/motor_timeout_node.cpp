#include <unistd.h>
#include <csignal>

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/Joy.h>

class MotorTimeoutNode {
  public:
    MotorTimeoutNode();
    void watch_motor_timeout();

  private:
    ros::NodeHandle nh;
    ros::Publisher  vel_pub;
    ros::Subscriber joy_sub;

    void joy_cb(const sensor_msgs::Joy::ConstPtr& joy);

    int timeout;    // in milisec

    ros::Time last_time;
};

MotorTimeoutNode::MotorTimeoutNode() {
    nh.getParam("/motor_timeout_node/timeout", timeout);
    vel_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/fake_wheel/twist",1);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &MotorTimeoutNode::joy_cb, this);
}

void MotorTimeoutNode::joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {

    last_time = ros::Time::now();
}

void MotorTimeoutNode::watch_motor_timeout(){

    ros::Duration motor_timeout(timeout);
    ros::Time now = ros::Time::now();

    if(last_time + motor_timeout < now) {

        geometry_msgs::TwistWithCovarianceStamped twist;

        twist.header.stamp = now;
        twist.header.frame_id = "base_link";

        twist.twist.twist.angular.z = 0.0;
        twist.twist.twist.linear.x = 0.0;
        twist.twist.covariance[0] = 0.0;
        vel_pub.publish(twist);
    }
}

int main (int argc, char * argv[]) {

    ros::init(argc,argv, "teleop_node");
    MotorTimeoutNode teleop_node;

    ros::Rate r(10);
    while(ros::ok()) {
        teleop_node.watch_motor_timeout();
        ros::spinOnce();
        r.sleep();
    }
}
