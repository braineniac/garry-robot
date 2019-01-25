#include <unistd.h>
#include <csignal>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

void it_is_done_my_lord(int);
std::sig_atomic_t volatile done = 0;

class TeleopNode {
    public:
        TeleopNode();
    private:
        ros::NodeHandle nh;

        ros::Publisher vel_pub;
        ros::Subscriber joy_sub;

        void joy_cb(const sensor_msgs::Joy::ConstPtr& joy);

        int linear, angular;
        double l_scale, a_scale;
        int timeout;
};

TeleopNode::TeleopNode() {

    nh.getParam("/teleop_node/scale_angular", a_scale);
    nh.getParam("/teleop_node/scale_linear", l_scale);
    nh.getParam("/teleop_node/timeout", timeout);

    linear = 1;
    angular = 2;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/arduino/cmd_vel",1);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopNode::joy_cb, this);
}

void TeleopNode::joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {

    geometry_msgs::Twist msg;

    msg.angular.z = a_scale * joy->axes[angular];
    msg.linear.x = l_scale * joy->axes[linear];

    ualarm(timeout,0);

    while(!done) {
        vel_pub.publish(msg);
        usleep(30000);
    }
    done = 0;
}

void it_is_done_my_lord(int) { done = 1; }

int main (int argc, char * argv[]) {

    std::signal(SIGALRM, it_is_done_my_lord);

    ros::init(argc,argv, "teleop_node");
    TeleopNode teleop_node;

    ros::spin();
}
