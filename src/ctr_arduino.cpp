#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sstream"

#include "ctr_arduino/Servo.h"
#include "ctr_arduino/Motor.h"
#include "ctr_arduino/LEDs.h"

#define ARDUINO_I2C_ADDR 0x68

int file_i2c;
unsigned char buffer[60] = {0};

/* I2C functions */
int open_i2c_bus() {

	char *filename = (char *)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0) {

		ROS_INFO("Failed to open I2C bus");
		return -1;
	}
	else {
		ROS_INFO("Opened I2C bus\n");
		return 0;
	}
}

int access_arduino() {

	if (ioctl(file_i2c, I2C_SLAVE, ARDUINO_I2C_ADDR) < 0) {

		ROS_INFO("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}
	else {
		ROS_INFO("Accessed I2C device\n");
		return 0;
	}
}

int read_arduino() {
	
	int length = 4;
	unsigned char buffer[60] = {0};
	if (open_i2c_bus() == 0) {
		if (access_arduino() == 0) {
			if (read(file_i2c,buffer,length) != length) {
				ROS_INFO("Failed to read from I2C bus\n");
				return -1;
			}
			else {
				ROS_INFO("Data read: %s\n",buffer);
				return 0;
			}
		}
	}
}

int write_arduino(int cmd_num,int cmd_param0, int cmd_param1, int cmd_param2, int cmd_param3) {

	int length = 6;

	buffer[0] = 42;
	buffer[1] = cmd_num;
	buffer[2] = cmd_param0;
	buffer[3] = cmd_param1;
	buffer[4] = cmd_param2;
	buffer[5] = cmd_param3;

	if( open_i2c_bus() == 0) {
		if (access_arduino() == 0) {
			if (write(file_i2c, buffer, length) != length) {
				ROS_INFO("Failed to write to I2C bus\n");
				return -1;
			}
			else {
	 			ROS_INFO("Written data to I2C bus\n");
				return 0;
			}
		}
	}
}


/* Handlers */
void motor_cb(const ctr_arduino::Motor& motor_msg) {
	
	int A_speed = motor_msg.A_speed;
	int B_speed = motor_msg.B_speed;
	float A_dur = motor_msg.A_dur;
	float B_dur = motor_msg.B_dur;
	bool A_dir = (A_speed>0);
	bool B_dir = (B_speed>0);
	
	int countA = round(A_dur/0.1);
	int countB = round(B_dur/0.1);
	int n;

	if(A_speed < -255 && A_speed > 255)
		ROS_ERROR("Invalid speed on channel A");
	else if (B_speed < -255 && B_speed > 255)
		ROS_ERROR("Invalid speed on channel B");
	else {
		ROS_INFO("Motor movement set");
		if (A_dur > B_dur) {
			for(n=0;n<countB; n++) {	
				write_arduino(1,A_dir,B_dir,abs(A_speed),abs(B_speed));
			}
			for(n=countB;n<countA;n++) {
				write_arduino(1,A_dir,B_dir,abs(A_speed),0);
			}
		}
		else {
			for(n=0;n<countA; n++) {        
                                write_arduino(1,A_dir,B_dir,abs(A_speed),abs(B_speed));
                        }
                        for(n=countA;n<countB;n++) {
                                write_arduino(1,A_dir,B_dir,0,abs(B_speed));
                        }
		}
	}
}

void servo_cb(const ctr_arduino::Servo& servo_msg) {
	
	int A_pos = servo_msg.A_pos;
	int B_pos = servo_msg.B_pos;
	
	if(A_pos < 9 && A_pos > 90)
		ROS_ERROR("Invalid A servo position");
	else if(B_pos < 9 && B_pos > 180)
		ROS_ERROR("Invalid B servo position");
	else {
		write_arduino(2,A_pos,B_pos,0,0);
		ROS_INFO("Servo movement set");
	}
}

void LEDs_cb(const ctr_arduino::LEDs& LEDs_msg) {

	bool led1 = LEDs_msg.led1;
	bool led2 = LEDs_msg.led2;
	
	write_arduino(3,led1,led2,0,0);
	ROS_INFO("Set LEDs");
}

void initialise_hardware() {

	//write_arduino(3,1,1,0,0); //enables LEDs
	//write_arduino(2,60,90,0,0); // sets servos to default position
	
	while(1) {
		read_arduino();
		sleep(1);
	}

}
/* main function */
int main(int argc, char **argv) {
	
	ros::init(argc,argv,"ctr_arduino");
	ros::NodeHandle nh;
	ros::Subscriber motor_sub = nh.subscribe("motor", 1, motor_cb);
	ros::Subscriber servo_sub = nh.subscribe("servo", 1, servo_cb);
	ros::Subscriber leds_sub = nh.subscribe("leds", 1 , LEDs_cb);
	ros::Rate loop_rate(10);

	initialise_hardware();
	while (ros::ok()) {
		
		ros::spin();
	}

	return 0;
}
