#include "ros/ros.h"
#include "std_msgs/Bool.h"

class EyeNode {
    public:
        EyeNode();

        void lights_on();
        void lights_off();
    private:
        ros::NodeHandle nh;
        ros::Publisher led_left_eye_pub;
        ros::Publisher led_right_eye_pub;

};

EyeNode::EyeNode () {
    led_left_eye_pub = nh.advertise<std_msgs::Bool>("/arduino/led_right_eye", 1);
    led_right_eye_pub = nh.advertise<std_msgs::Bool>("/arduino/led_left_eye", 1);
}

void EyeNode::lights_on() {
    std_msgs::Bool msg;
    msg.data = 1;
    EyeNode::led_left_eye_pub.publish(msg);
    EyeNode::led_right_eye_pub.publish(msg);
}

void EyeNode::lights_off () {
    std_msgs::Bool msg;
    msg.data = 0;
    EyeNode::led_left_eye_pub.publish(msg);
    EyeNode::led_right_eye_pub.publish(msg);
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "eyes_node");
    EyeNode eye_node;
    ros::Rate loop_rate(10000);

    while (ros::ok()) {

        eye_node.lights_on();
        loop_rate.sleep();
    }

    ros::spin();

}
