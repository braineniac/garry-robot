#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <array>

#define AVG_BUF_SIZE 200

class OdomNode {

  public:
    OdomNode();

  private:
    ros::NodeHandle nh;
    ros::Publisher  odom_pub;
    ros::Subscriber imu_sub;

    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);

    tf::TransformBroadcaster odom_broadcaster;

    void publish_position();
    void publish_transform();
    void update_position();

    sensor_msgs::Imu imu_old;
    sensor_msgs::Imu imu_new;

    int imu_cb_count;

    std::vector<double> linear_position  = std::vector<double>(3);
    std::vector<double> angular_position = std::vector<double>(3);
    std::vector<double> angular_velocity = std::vector<double>(3);
    std::vector<double> linear_velocity  = std::vector<double>(3);

    std::array<std::vector<double>, AVG_BUF_SIZE> linear_offset_array;
    std::array<std::vector<double>, AVG_BUF_SIZE> angular_offset_array;

    std::vector<double> linear_offset  = std::vector<double>(3);
    std::vector<double> angular_offset = std::vector<double>(3);
};

OdomNode::OdomNode() {

    odom_pub     = nh.advertise<nav_msgs::Odometry>("odom", 50);
    imu_sub      = nh.subscribe("/imu/data", 10, &OdomNode::imu_cb, this);
    imu_cb_count = 0;

    linear_offset_array.fill(std::vector<double>(3, 0.0));
    angular_offset_array.fill(std::vector<double>(3, 0.0));

    std::fill(linear_position.begin(), linear_position.end(), 0.0);
    std::fill(angular_position.begin(), angular_position.end(), 0.0);
    std::fill(angular_velocity.begin(), angular_velocity.end(), 0.0);
    std::fill(linear_offset.begin(), linear_offset.end(), 0.0);
    std::fill(angular_offset.begin(), angular_offset.end(), 0.0);
    std::fill(linear_velocity.begin(), linear_velocity.end(), 0.0);
}

void OdomNode::imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {

    OdomNode::imu_new.header              = msg->header;
    OdomNode::imu_new.linear_acceleration = msg->linear_acceleration;
    OdomNode::imu_new.angular_velocity    = msg->angular_velocity;
    OdomNode::imu_cb_count++;

    if(OdomNode::imu_cb_count == 5) {

        OdomNode::linear_offset[0]  = msg->linear_acceleration.x;
        OdomNode::linear_offset[1]  = msg->linear_acceleration.y;
        OdomNode::linear_offset[2]  = msg->linear_acceleration.z;
        OdomNode::angular_offset[0] = msg->angular_velocity.x;
        OdomNode::angular_offset[1] = msg->angular_velocity.y;
        OdomNode::angular_offset[2] = msg->angular_velocity.z;
    }
    else if(OdomNode::imu_cb_count > 100 &&
            (OdomNode::imu_cb_count < linear_offset_array.size() + 100)) {
        for(int i = 0; i < linear_offset_array.size(); i++) {
            if(OdomNode::linear_offset_array[i][0] == 0.0) {
                OdomNode::linear_offset_array[i][0]  = msg->linear_acceleration.x;
                OdomNode::linear_offset_array[i][1]  = msg->linear_acceleration.y;
                OdomNode::linear_offset_array[i][2]  = msg->linear_acceleration.z;
                OdomNode::angular_offset_array[i][0] = msg->angular_velocity.x;
                OdomNode::angular_offset_array[i][1] = msg->angular_velocity.y;
                OdomNode::angular_offset_array[i][2] = msg->angular_velocity.z;
            }
        }
    }
    else {
        OdomNode::linear_offset_array[OdomNode::linear_offset_array.size() - 1][0] =
          msg->linear_acceleration.x;
        OdomNode::linear_offset_array[OdomNode::linear_offset_array.size() - 1][1] =
          msg->linear_acceleration.y;
        OdomNode::linear_offset_array[OdomNode::linear_offset_array.size() - 1][2] =
          msg->linear_acceleration.z;
        OdomNode::angular_offset_array[OdomNode::linear_offset_array.size() - 1][0] =
          msg->angular_velocity.x;
        OdomNode::angular_offset_array[OdomNode::linear_offset_array.size() - 1][1] =
          msg->angular_velocity.y;
        OdomNode::angular_offset_array[OdomNode::linear_offset_array.size() - 1][2] =
          msg->angular_velocity.z;


        std::array<double, 6> avg;
        std::fill(std::begin(avg), std::end(avg), 0.0);
        for(int i = 0; i < 3; i++) {
            for(int j = 0; j < OdomNode::linear_offset_array.size(); j++) {
                avg[i] += OdomNode::linear_offset_array[j][i];
                avg[i + 3] += OdomNode::angular_offset_array[j][i];
            }
            avg[i] = avg[i] / OdomNode::linear_offset_array.size();
        }
        // ROS_INFO_STREAM(std::to_string((double) avg[0]));

        OdomNode::linear_offset[0]  = avg[0];
        OdomNode::linear_offset[1]  = avg[1];
        OdomNode::linear_offset[2]  = avg[2];
        OdomNode::angular_offset[0] = avg[3];
        OdomNode::angular_offset[1] = avg[4];
        OdomNode::angular_offset[2] = avg[5];

        for(int k = 0; k < 4; k++) {
            OdomNode::linear_offset_array[k]  = OdomNode::linear_offset_array[k + 1];
            OdomNode::angular_offset_array[k] = OdomNode::angular_offset_array[k + 1];
        }

        OdomNode::update_position();
    }
}

void OdomNode::update_position() {

    double delta_t = (OdomNode::imu_new.header.stamp - OdomNode::imu_old.header.stamp).toSec();

    std::vector<double> delta_a = std::vector<double>(3);
    delta_a[0] = OdomNode::imu_new.linear_acceleration.x - OdomNode::imu_old.linear_acceleration.x -
                 OdomNode::linear_offset[0];
    delta_a[1] = OdomNode::imu_new.linear_acceleration.y - OdomNode::imu_old.linear_acceleration.x -
                 OdomNode::linear_offset[1];
    delta_a[2] = OdomNode::imu_new.linear_acceleration.z - OdomNode::imu_old.linear_acceleration.x -
                 OdomNode::linear_offset[2];

    std::vector<double> linear_pos_tmp  = std::vector<double>(3);
    std::vector<double> linear_vel_tmp  = std::vector<double>(3);
    std::vector<double> angular_pos_tmp = std::vector<double>(3);
    std::vector<double> angular_vel_tmp = std::vector<double>(3);

    linear_vel_tmp[0] =
      (OdomNode::imu_old.linear_acceleration.x - OdomNode::linear_offset[0]) * delta_t;
    linear_vel_tmp[1] =
      (OdomNode::imu_old.linear_acceleration.y - OdomNode::linear_offset[1]) * delta_t;
    linear_vel_tmp[2] =
      (OdomNode::imu_old.linear_acceleration.z - OdomNode::linear_offset[2]) * delta_t;

    linear_pos_tmp[0] =
      0.5 * delta_a[0] * delta_t * delta_t + OdomNode::linear_velocity[0] * delta_t;
    linear_pos_tmp[1] =
      0.5 * delta_a[1] * delta_t * delta_t + OdomNode::linear_velocity[1] * delta_t;
    linear_pos_tmp[2] =
      0.5 * delta_a[2] * delta_t * delta_t + OdomNode::linear_velocity[2] * delta_t;

    angular_vel_tmp[0] = (OdomNode::imu_new.angular_velocity.x - OdomNode::angular_offset[0]);
    angular_vel_tmp[1] = (OdomNode::imu_new.angular_velocity.y - OdomNode::angular_offset[1]);
    angular_vel_tmp[2] = (OdomNode::imu_new.angular_velocity.z - OdomNode::angular_offset[2]);

    angular_pos_tmp[0] =
      (OdomNode::imu_old.angular_velocity.x - OdomNode::angular_offset[0]) * delta_t;
    angular_pos_tmp[1] =
      (OdomNode::imu_old.angular_velocity.y - OdomNode::angular_offset[1]) * delta_t;
    angular_pos_tmp[2] =
      (OdomNode::imu_old.angular_velocity.z - OdomNode::angular_offset[2]) * delta_t;

    if(OdomNode::imu_cb_count % 100 == 0) {
        ROS_INFO("Linear global pos");
        ROS_INFO_STREAM(std::to_string((double) OdomNode::linear_position[0]));
        ROS_INFO_STREAM(std::to_string((double) OdomNode::linear_position[1]));
        ROS_INFO_STREAM(std::to_string((double) OdomNode::linear_position[2]));

        ROS_INFO("angular global pos");
        ROS_INFO_STREAM(std::to_string((double) OdomNode::angular_position[0]));
        ROS_INFO_STREAM(std::to_string((double) OdomNode::angular_position[1]));
        ROS_INFO_STREAM(std::to_string((double) OdomNode::angular_position[2]));
    }

    // data validation
    if((fabs(linear_pos_tmp[0]) > 0.005 || fabs(linear_pos_tmp[1]) > 0.005) &&
       (fabs(linear_pos_tmp[0]) < 0.5 || fabs(linear_pos_tmp[1]) < 0.5) && (delta_t < 0.2) &&
       (OdomNode::imu_cb_count > 150)) {

        OdomNode::linear_velocity[0] = linear_vel_tmp[0];
        OdomNode::linear_velocity[1] = linear_vel_tmp[1];
        OdomNode::linear_velocity[2] = linear_vel_tmp[2];

        OdomNode::linear_position[0] += linear_pos_tmp[0];
        OdomNode::linear_position[1] += linear_pos_tmp[1];
        OdomNode::linear_position[2] += linear_pos_tmp[2];

        OdomNode::angular_velocity[0] = angular_vel_tmp[0];
        OdomNode::angular_velocity[1] = angular_vel_tmp[1];
        OdomNode::angular_velocity[2] = angular_vel_tmp[2];

        OdomNode::angular_position[0] += angular_pos_tmp[0];
        OdomNode::angular_position[1] += angular_pos_tmp[1];
        OdomNode::angular_position[2] += angular_pos_tmp[2];

        OdomNode::publish_transform();
        OdomNode::publish_position();
    }

    OdomNode::imu_old = OdomNode::imu_new;
}

void OdomNode::publish_position() {

    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp    = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link";

    odom_msg.pose.pose.position.x  = OdomNode::linear_position[0];
    odom_msg.pose.pose.position.y  = OdomNode::linear_position[1];
    odom_msg.pose.pose.position.z  = 0.0;
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(OdomNode::angular_position[2]);

    odom_msg.twist.twist.angular.x = OdomNode::angular_velocity[0];
    odom_msg.twist.twist.angular.y = OdomNode::angular_velocity[1];
    odom_msg.twist.twist.angular.z = OdomNode::angular_velocity[2];

    OdomNode::odom_pub.publish(odom_msg);
}

void OdomNode::publish_transform() {

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp    = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id  = "base_link";

    odom_trans.transform.translation.x = OdomNode::linear_position[0];
    odom_trans.transform.translation.x = OdomNode::linear_position[1];
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(OdomNode::angular_position[2]);

    OdomNode::odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "odom_node");
    OdomNode odom_node;
    ros::spin();
}
