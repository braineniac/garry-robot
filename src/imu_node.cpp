#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

#include <sstream>

char output[50];
sensor_msgs::Imu imu_old;
sensor_msgs::Imu imu_new;
geometry_msgs::TwistStamped pos_new;
double imu_lin_offset_x=0;
double imu_lin_offset_y=0;
double imu_lin_offset_z=0;
double imu_ang_offset_x=0;
double imu_ang_offset_y=0;
double imu_ang_offset_z=0;
int imu_cb_count=0;


void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {

				imu_new.header = msg->header;
				imu_new.linear_acceleration = msg->linear_acceleration;
				imu_new.angular_velocity = msg->angular_velocity;
				imu_cb_count++;
				if(imu_cb_count == 5) {
								imu_lin_offset_x = msg->linear_acceleration.x;
								imu_lin_offset_y = msg->linear_acceleration.y;
								imu_lin_offset_z = msg->linear_acceleration.z;
								imu_ang_offset_x = msg->angular_velocity.x;
								imu_ang_offset_y = msg->angular_velocity.y;
								imu_ang_offset_z = msg->angular_velocity.z;

								snprintf(output,50,"linear offset x=%f", imu_lin_offset_x);
								ROS_INFO(output);
								snprintf(output,50,"linear offset y=%f", imu_lin_offset_y);
								ROS_INFO(output);
								snprintf(output,50,"linear offset z=%f", imu_lin_offset_z);
								ROS_INFO(output);
								snprintf(output,50,"angular offset x=%f", imu_ang_offset_x);
								ROS_INFO(output);
								snprintf(output,50,"angular offset y=%f", imu_ang_offset_y);
								ROS_INFO(output);
								snprintf(output,50,"angular offset z=%f", imu_ang_offset_z);
								ROS_INFO(output);



				}

}


int main(int argc, char **argv) {

				ros::init(argc, argv, "imu_node");
				ros::NodeHandle n;

				ros::Publisher pos_pub = n.advertise<geometry_msgs::TwistStamped>("imu_pos", 100);
				ros::Subscriber imu_sub = n.subscribe("/imu/data", 1000, imu_cb);

				ros::Rate loop_rate(10);

				while(ros::ok()) {
								
								double new_t = imu_new.header.stamp.sec + imu_new.header.stamp.nsec/1000000000.0;
								double old_t = imu_old.header.stamp.sec + imu_old.header.stamp.nsec/1000000000.0;
								double delta_t = new_t - old_t;
								snprintf(output,50,"%f",delta_t);
								ROS_INFO(output);

								pos_new.header = imu_new.header;
								double delta_a_x = imu_new.linear_acceleration.x - imu_old.linear_acceleration.x - imu_lin_offset_x;
								double delta_a_y = imu_new.linear_acceleration.y - imu_old.linear_acceleration.x - imu_lin_offset_y;
								double delta_a_z = imu_new.linear_acceleration.z - imu_old.linear_acceleration.x - imu_lin_offset_z;
								
								double new_pos_x =  0.5*delta_a_x*delta_t*delta_t + (imu_old.linear_acceleration.x-imu_lin_offset_x)*delta_t;
								double new_pos_y =  0.5*delta_a_y*delta_t*delta_t + (imu_old.linear_acceleration.y-imu_lin_offset_y)*delta_t;
								double new_pos_z =  0.5*delta_a_z*delta_t*delta_t + (imu_old.linear_acceleration.z-imu_lin_offset_z)*delta_t;


			  				pos_new.twist.linear.x = new_pos_x;			  					  	  				
								pos_new.twist.linear.y = new_pos_y;
								pos_new.twist.linear.z = new_pos_z;
								
								pos_new.twist.angular.x = (imu_old.angular_velocity.x - imu_ang_offset_x) * delta_t;
								pos_new.twist.angular.y = (imu_old.angular_velocity.y - imu_ang_offset_y) * delta_t;
								pos_new.twist.angular.z = (imu_old.angular_velocity.z - imu_ang_offset_z) * delta_t;
	
								imu_old.header = imu_new.header;
								imu_old.linear_acceleration = imu_new.linear_acceleration;
								imu_old.angular_velocity = imu_new.angular_velocity;
								
								if ((new_pos_x > 0.03 || new_pos_y > 0.03) && (new_pos_x<1.0 || new_pos_y < 1.0))
										pos_pub.publish(pos_new);

								ros::spinOnce();
								loop_rate.sleep();

				}

				return 0;

}
