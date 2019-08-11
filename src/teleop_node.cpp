#include <unistd.h>
#include <csignal>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopNode {
    public:
        TeleopNode();
        void publish_joy();
    private:
        ros::NodeHandle nh;

        ros::Publisher vel_pub;
        ros::Subscriber joy_sub;
        ros::Publisher left_eye_pub;
        ros::Publisher right_eye_pub;

        void joy_cb(const sensor_msgs::Joy::ConstPtr& joy);

        int linear, angular;
        double l_scale=0.0; 
				double a_scale=0.0;
				double l_offset=0.0;
				double a_offset=0.0;
        bool left_eye{false};
        bool right_eye{false};

        geometry_msgs::Twist current_twist;
};

TeleopNode::TeleopNode() {

    nh.getParam("/teleop_node/scale_angular", a_scale);
    nh.getParam("/teleop_node/scale_linear", l_scale);
		nh.getParam("/teleop_node/l_offset", l_offset);
		nh.getParam("/teleop_node/a_offset", a_offset);

    linear = 3;
    angular = 2;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/arduino/cmd_vel",1);
    left_eye_pub = nh.advertise<std_msgs::Bool>("/arduino/led_left_eye",1);
    right_eye_pub = nh.advertise<std_msgs::Bool>("/arduino/led_right_eye",1);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopNode::joy_cb, this);
}

void TeleopNode::joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {

    geometry_msgs::Twist msg;

		if (joy->axes[linear]>0) {
		
    msg.linear.x = l_scale * joy->axes[linear] + l_offset;
		}
		else if (joy->axes[angular]>0) {

    msg.angular.z = a_scale * joy->axes[angular] + a_offset;
		}
		else if (joy->axes[linear]<0) {
    msg.linear.x = l_scale * joy->axes[linear] + l_offset;
		}
		else if (joy->axes[angular]<0) {

		msg.angular.z = a_scale * joy->axes[angular] -  a_offset;
		}
    // left eye toggle
    if(joy->buttons[1]) {
        std_msgs::Bool msg;
        if(left_eye){
            msg.data = 0;
            left_eye = false;
        }
        else {
            msg.data = 1;
            left_eye = true;
        }
        left_eye_pub.publish(msg);
    }

    // right eye toggle
    if(joy->buttons[2]) {
        std_msgs::Bool msg;
        if(right_eye){
            msg.data = 0;
            right_eye = false;
        }
        else {
            msg.data = 1;
            right_eye = true;
        }
        right_eye_pub.publish(msg);
    }

    current_twist = msg;

}

void TeleopNode::publish_joy() {
    vel_pub.publish(current_twist);
}

int main (int argc, char * argv[]) {

    ros::init(argc,argv, "teleop_node");
    TeleopNode teleop_node;
    ros::Rate r(50);
    while(ros::ok()){
        teleop_node.publish_joy();
           ros::spinOnce();
           r.sleep();
       }
}
