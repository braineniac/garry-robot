#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

class ImuNode {
    public:
        ImuNode();

    private:
        ros::NodeHandle nh;
        ros::Publisher pos_pub;
        ros::Subscriber imu_sub;

        void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
        void update_position();
        void publish_position();

        sensor_msgs::Imu imu_old;
        sensor_msgs::Imu imu_new;
        int imu_cb_count;

        std::vector<double> linear_position  = std::vector<double>(3);
        std::vector<double> angular_position = std::vector<double>(3);
        std::vector<double> linear_offset    = std::vector<double>(3);
        std::vector<double> angular_offset   = std::vector<double>(3);
        std::vector<double> angular_velocity = std::vector<double>(3);
};

ImuNode::ImuNode()
{
    pos_pub = nh.advertise<geometry_msgs::TwistStamped>("imu_pos", 1);
    imu_sub = nh.subscribe("/imu/data", 10, &ImuNode::imu_cb, this);
    imu_cb_count = 0;
}

void ImuNode::imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {

    ImuNode::imu_new.header = msg->header;
    ImuNode::imu_new.linear_acceleration = msg->linear_acceleration;
    ImuNode::imu_new.angular_velocity = msg->angular_velocity;
    ImuNode::imu_cb_count++;

    if(ImuNode::imu_cb_count == 5) {

        ImuNode::linear_offset[0] = msg->linear_acceleration.x;
        ImuNode::linear_offset[1] = msg->linear_acceleration.y;
        ImuNode::linear_offset[2] = msg->linear_acceleration.z;
        ImuNode::angular_offset[0] = msg->angular_velocity.x;
        ImuNode::angular_offset[1] = msg->angular_velocity.y;
        ImuNode::angular_offset[2] = msg->angular_velocity.z;
    }
    else if(ImuNode::imu_cb_count > 6) {
        ImuNode::update_position();
    }

}

void ImuNode::update_position() {

    double new_t = ImuNode::imu_new.header.stamp.sec + ImuNode::imu_new.header.stamp.nsec/1000000000.0;
    double old_t = ImuNode::imu_old.header.stamp.sec + ImuNode::imu_old.header.stamp.nsec/1000000000.0;
    double delta_t = new_t - old_t;

    std::vector<double> delta_a = std::vector<double>(3);
    delta_a[0] = ImuNode::imu_new.linear_acceleration.x - ImuNode::imu_old.linear_acceleration.x - ImuNode::linear_offset[0];
    delta_a[1] = ImuNode::imu_new.linear_acceleration.y - ImuNode::imu_old.linear_acceleration.x - ImuNode::linear_offset[1];
    delta_a[2] = ImuNode::imu_new.linear_acceleration.z - ImuNode::imu_old.linear_acceleration.x - ImuNode::linear_offset[2];

    std::vector<double> linear_path = std::vector<double>(3);
    linear_path[0] = (ImuNode::imu_old.linear_acceleration.x-ImuNode::linear_offset[0])*delta_t;
    linear_path[1] = (ImuNode::imu_old.linear_acceleration.y-ImuNode::linear_offset[1])*delta_t;
    linear_path[2] = (ImuNode::imu_old.linear_acceleration.z-ImuNode::linear_offset[2])*delta_t;

    ImuNode::linear_position[0] =  0.5*delta_a[0]*delta_t*delta_t + linear_path[0];
    ImuNode::linear_position[1] =  0.5*delta_a[1]*delta_t*delta_t + linear_path[1];
    ImuNode::linear_position[2] =  0.5*delta_a[2]*delta_t*delta_t + linear_path[2];

    ImuNode::angular_velocity[0] = (ImuNode::imu_old.angular_velocity.x - ImuNode::angular_offset[0]) * delta_t;
    ImuNode::angular_velocity[1] = (ImuNode::imu_old.angular_velocity.y - ImuNode::angular_offset[1]) * delta_t;
    ImuNode::angular_velocity[2] = (ImuNode::imu_old.angular_velocity.z - ImuNode::angular_offset[2]) * delta_t;

    ImuNode::publish_position();
}

void ImuNode::publish_position() {

    // sanity check
    if (ImuNode::linear_position[0] > 0.03 || ImuNode::linear_position[1] > 0.03)
        if (ImuNode::linear_position[0] < 0.5 || ImuNode::linear_position[2] < 0.5) {

            geometry_msgs::TwistStamped twist_stamped;
            twist_stamped.header = ImuNode::imu_old.header;
            twist_stamped.twist.linear.x = ImuNode::linear_position[0];
            twist_stamped.twist.linear.y = ImuNode::linear_position[1];
            twist_stamped.twist.linear.z = ImuNode::linear_position[2];
            twist_stamped.twist.angular.x = ImuNode::angular_velocity[0];
            twist_stamped.twist.angular.y = ImuNode::angular_velocity[1];
            twist_stamped.twist.angular.z = ImuNode::angular_velocity[2];

            ImuNode::pos_pub.publish(twist_stamped);
        }

    ImuNode::imu_old = ImuNode::imu_new;

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "imu_node");
    ImuNode imu_node;

    ros::spin();

}
