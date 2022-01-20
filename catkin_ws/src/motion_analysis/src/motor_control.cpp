
#include <ros/ros.h>
#include "motion_analysis/Driver_XM540W270R.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

class Motor_Control {
	private:
		
		const char* device_name = "/dev/ttyUSB0";
		int protocol_version = 2.0;
		int baudrate = 2000000;
		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;
		std::vector<int> goalPositions{-90, 0, 90, 0, 90};
		bool torque = true;

		X_Motor motor;

	public:
	Motor_Control(){
		portHandler = dynamixel::PortHandler::getPortHandler(device_name);
		packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
		
		
		// Open port
		if (portHandler->openPort())
		{
			ROS_INFO_STREAM("Succeeded to open the port!");
		}
		else
		{
			ROS_INFO_STREAM("Failed to open the port!");
			ROS_INFO_STREAM("Press any key to terminate...");
			ros::waitForShutdown();
		}

		// Set port baudrate
		if (portHandler->setBaudRate(baudrate))
		{
			ROS_INFO_STREAM("Succeeded to change the baudrate!");
		}
		else
		{
			ROS_INFO_STREAM("Failed to change the baudrate!");
			ROS_INFO_STREAM("Press any key to terminate...");
			ros::waitForShutdown();
		}

		motor.init(1, portHandler, packetHandler, 4);

		motor.moveToPos(0);

		motor.moveToPos(2048);

		motor.moveToPos(1024);

	}


};

int main(int argc, char *argv[]) 
{
	ros::init(argc, argv, "motor_control");
	ROS_INFO_STREAM("Starting motor_control node...");
    ros::NodeHandle node;

	Motor_Control mc;
	ros::spin();

  return 0;
}