#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include <sstream>


sensor_msgs::Imu imu_old;
sensor_msgs::Imu imu_new;
geometry_msgs::TwistStamped pos_new;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {

				imu_new.header = msg->header;
				imu_new.linear_acceleration = msg->linear_acceleration;
				imu_new.angular_velocity = msg->angular_velocity;

}


int main(int argc, char **argv) {

				ros::init(argc, argv, "imu_node");
				ros::NodeHandle n;

				ros::Publisher pos_pub = n.advertise<geometry_msgs::TwistStamped>("imu_pos", 100);
				ros::Subscriber imu_sub = n.subscribe("/imu/data", 1000, imu_cb);

				ros::Rate loop_rate(10);

				while(ros::ok()) {
								
								double delta_t_nsec = 0.0;
								double delta_t_sec= 0.0;
								delta_t_nsec = imu_new.header.stamp.nsec - imu_old.header.stamp.nsec;
								delta_t_sec = imu_new.header.stamp.sec - imu_old.header.stamp.sec;
								double delta_t = delta_t_sec + delta_t_nsec/1000000000.0;

								pos_new.header = imu_new.header;
								double delta_a_x = imu_new.linear_acceleration.x - imu_old.linear_acceleration.x;
								double delta_a_y = imu_new.linear_acceleration.y - imu_old.linear_acceleration.x;
								double delta_a_z = imu_new.linear_acceleration.z - imu_old.linear_acceleration.x;

			  				pos_new.twist.linear.x = 1/2*delta_a_x*delta_t*delta_t + imu_old.linear_acceleration.x*delta_t;
			  				pos_new.twist.linear.y = 1/2*delta_a_y*delta_t*delta_t + imu_old.linear_acceleration.y*delta_t;
	  	  				pos_new.twist.linear.z = 1/2*delta_a_z*delta_t*delta_t + imu_old.linear_acceleration.z*delta_t;

								pos_new.twist.angular.x = imu_old.angular_velocity.x * delta_t;
								pos_new.twist.angular.y = imu_old.angular_velocity.y * delta_t;
								pos_new.twist.angular.z = imu_old.angular_velocity.z * delta_t;
	
								imu_old.header = imu_new.header;
								imu_old.linear_acceleration = imu_new.linear_acceleration;
								imu_old.angular_velocity = imu_new.angular_velocity;

								pos_pub.publish(pos_new);

								ros::spinOnce();
								loop_rate.sleep();

				}

				return 0;

}
