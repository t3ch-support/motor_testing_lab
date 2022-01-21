
#include <ros/ros.h>
#include "motion_analysis/Driver_XM540W270R.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/JointState.h"


class Motor_Control {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Subscriber command_sub;
		ros::Publisher encoder_pub;
		ros::Publisher joint_pub;
		int encoder_pub_rate = 10;

		const char* device_name = "/dev/ttyUSB0";
		int protocol_version = 2.0;
		int baudrate = 2000000;
		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;	
		bool torque = true;
		
		std::vector<X_Motor> motors;
		std::vector<int> goalPositions{-90, 0, 90, 0, 90};

	public:

	Motor_Control(const ros::NodeHandle &node_handle,
				  const ros::NodeHandle &node_handle_private)
	:
	nh_(node_handle),
	pnh_(node_handle_private)
	{
		this->init();
	}

	
	void init(){

		encoder_pub = nh_.advertise<std_msgs::Int32MultiArray>("encoder_positions", 100);		
		joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
		command_sub = nh_.subscribe("command_positions", 100, &Motor_Control::CommandCallback, this);
		
		portHandler = dynamixel::PortHandler::getPortHandler(device_name);
		packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);	

		// Create motor objects based on config/motors.yaml
		int motor_count;
		pnh_.getParam("motor_count", motor_count);
		for(int i = 1; i<=motor_count; i++){
			int id;
			int op_mode;
			double gear_ratio;
			int reset_state;
			int offset;
			pnh_.getParam("motor_"+std::to_string(i)+"/id", id);
			pnh_.getParam("motor_"+std::to_string(i)+"/op_mode", op_mode);
			pnh_.getParam("motor_"+std::to_string(i)+"/gear_ratio", gear_ratio);
			pnh_.getParam("motor_"+std::to_string(i)+"/reset_state", reset_state);
			pnh_.getParam("motor_"+std::to_string(i)+"/offset", offset);


			ROS_INFO_STREAM("ID: " << id);
			X_Motor tempX;
			tempX.init(id, portHandler, packetHandler, op_mode, gear_ratio, reset_state, offset);
			motors.push_back(tempX);
		}
	
		SetupPort();
		Loop();
	}

	void SetupPort(){
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
	}
	
	void Loop(){
		ros::Rate loop_rate(encoder_pub_rate); // In Hz
		while (ros::ok()){

			// Read motor present positions (saves to motor.dxl_present_position)
			ReadAllJoints(motors);

			// Populate msgs with motor data
			std_msgs::Int32MultiArray encoder_data;
    		sensor_msgs::JointState latestState;
			latestState.header.stamp = ros::Time::now();

			for(int i = 0; i<motors.size(); i++){
				int raw = motors.at(i).dxl_present_position;
				encoder_data.data.push_back(raw);
				latestState.name.at(i) = std::to_string(motors.at(i).id);
				latestState.position.at(i) = EncoderToRadians(raw, motors.at(i).offset, motors.at(i).gear_ratio);			
			}

			encoder_pub.publish(encoder_data);
			joint_pub.publish(latestState);

			ros::spinOnce();
			loop_rate.sleep();

		}
	}

	void CommandCallback(const sensor_msgs::JointStateConstPtr& joint_goals){
		
	}

	void ReadAllJoints(std::vector<X_Motor> &motorGroup){

		dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, motorGroup[0].ADDR_PRO_PRESENT_POSITION, motorGroup[0].LEN_PRO_PRESENT_POSITION);
		
		// Add params for syncread
		for(X_Motor &m: motorGroup){
			bool dxl_addparam_result = groupSyncRead.addParam(m.id);
			if (dxl_addparam_result != true)
			{
				ROS_INFO("[ID:%03d] groupSyncRead addparam failed", m.id);
			}
		}

		// Perform syncread
		int dxl_comm_result = groupSyncRead.txRxPacket();

		if (dxl_comm_result != COMM_SUCCESS)
		{
			ROS_INFO("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		}

		// Check if syncread data is available
		for(X_Motor &m: motorGroup){
			bool dxl_getdata_result = groupSyncRead.isAvailable(m.id, m.ADDR_PRO_PRESENT_POSITION, m.LEN_PRO_PRESENT_POSITION);
			if (dxl_getdata_result != true)
			{
				ROS_INFO("[ID:%03d] groupSyncRead getdata failed", m.id);
			}
		}

		// Grab data
		for(int i = 0; i<motorGroup.size(); i++){
			motorGroup.at(i).dxl_present_position = groupSyncRead.getData(motorGroup.at(i).id, 
							motorGroup.at(i).ADDR_PRO_PRESENT_POSITION,
							motorGroup.at(i).LEN_PRO_PRESENT_POSITION);
		}

              
	}
	
	double EncoderToRadians(int raw, int offset, double gear_ratio){
		double radians = (raw-offset)/((4096*gear_ratio)/M_PI);
		return radians;
	}

	double toDegrees(double radians){
            return radians * 180 / M_PI;
    }

	double toRadians(double degrees){
            return degrees * M_PI / 180;
    }

	~Motor_Control(){
		portHandler->closePort();
	}

};

int main(int argc, char *argv[]) 
{
	std::string node_name = "motor_control";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh("");
	ros::NodeHandle nh_p("~");

	Motor_Control mc(nh, nh_p);

	ROS_INFO_STREAM("Initialized " << node_name << "node");

	ros::spin();

}