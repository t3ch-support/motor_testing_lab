
#include <ros/ros.h>
#include "motion_analysis/Driver_XM540W270R.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/JointState.h"


class Motor_Control {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Subscriber command_sub;
		ros::Publisher encoder_pub;
		ros::Publisher joint_pub;
		int encoder_pub_rate = 20;

		const char* device_name = "/dev/ttyUSB0";
		int protocol_version = 2.0;
		int baudrate = 4000000;
		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;	
		bool torque = true;
		
		std::vector<X_Motor> motors;
		sensor_msgs::JointState latestState;


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
		// Check if this node was launch with a launch file, otherwise throw error and shutdown
		
		nh_.getParam("/encoder_pub_rate", encoder_pub_rate);
		encoder_pub = nh_.advertise<std_msgs::Int32MultiArray>("encoder_positions", 100);		
		joint_pub = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
		command_sub = nh_.subscribe("cmd_positions", 100, &Motor_Control::CommandCallback, this);
		
		portHandler = dynamixel::PortHandler::getPortHandler(device_name);
		packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);	
		SetupPort();

		// Create motor objects based on config/motors.yaml
		int motor_count;
		nh_.getParam("/motor_count", motor_count);
		for(int i = 1; i<=motor_count; i++){
			X_Specs spec;
			
			nh_.getParam("/motor_"+std::to_string(i)+"/id", spec.id);
			nh_.getParam("/motor_"+std::to_string(i)+"/op_mode", spec.op_mode);
			nh_.getParam("/motor_"+std::to_string(i)+"/gear_ratio", spec.gear_ratio);
			nh_.getParam("/motor_"+std::to_string(i)+"/reset_state", spec.reset_state);
			nh_.getParam("/motor_"+std::to_string(i)+"/offset", spec.offset);
			nh_.getParam("/motor_"+std::to_string(i)+"/high_limit", spec.high_limit);
			nh_.getParam("/motor_"+std::to_string(i)+"/low_limit", spec.low_limit);

			// Create a motor with the specified parameters	
			X_Motor tempX;
			tempX.init(spec, portHandler, packetHandler);
			motors.push_back(tempX);

			// Initialize the JointState message according to yaml specified motors
			latestState.position.push_back(-1.0); 
			latestState.velocity.push_back(-1.0); 
			latestState.effort.push_back(-1.0); 
			latestState.name.push_back(std::to_string(tempX.id));
		}
        ROS_INFO_STREAM("Motors: " << motors.size());
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

			// // Read joint data
			ReadJointPositions(motors);
			ReadJointVelocities(motors);
			ReadJointCurrent(motors);
			
			// // Populate msgs with motor data
			std_msgs::Int32MultiArray encoder_data;
    		// sensor_msgs::JointState latestState;
			latestState.header.stamp = ros::Time::now();

			for(int i = 0; i<motors.size(); i++){
				int raw = motors.at(i).dxl_present_position;
				encoder_data.data.push_back(raw);
				latestState.effort.at(i) = motors.at(i).dxl_present_current;
				latestState.velocity.at(i) = motors.at(i).dxl_present_velocity;
				latestState.position.at(i) = toDegrees(EncoderToRadians(raw, motors.at(i).offset, motors.at(i).gear_ratio));
			}

			encoder_pub.publish(encoder_data);
			joint_pub.publish(latestState);

			ros::spinOnce();
			loop_rate.sleep();

		}
	}

	void CommandCallback(const sensor_msgs::JointStateConstPtr& joint_goals){
		// Goals are received as degrees!
		std::vector<double> goalsRadians;
		std::vector<int> goalsRaw;
		bool exceededLimit = false;
		for(int i = 0; i<joint_goals->position.size(); i++){
			double goalDegree = joint_goals->position.at(i);
			// Check to see if commands are beyond the allowable limit
			if(goalDegree > motors.at(i).high_limit){
				goalDegree = motors.at(i).high_limit;
				exceededLimit = true;
			}else if(goalDegree < motors.at(i).low_limit){
				goalDegree = motors.at(i).low_limit;
				exceededLimit = true;				
			}
			if(exceededLimit){
				ROS_INFO_STREAM("Motor w/ ID " << motors.at(i).id << " tried to go out of bounds!");
			}
			
			// Save goals in radians
			goalsRadians.push_back(toRadians(goalDegree));
			// Save goals as raw encoder values
			goalsRaw.push_back(RadiansToEncoder(goalsRadians.at(i), motors.at(i).offset, motors.at(i).gear_ratio));
			ROS_INFO_STREAM("D: " << goalDegree << ", R: " << goalsRadians.at(i) << ", E: " << goalsRaw.at(i));
		}

		MoveAllJoints(goalsRaw, false);
	}
	
	bool MoveAllJoints(std::vector<int> goalPositions, bool waitForGoal){
		if(waitForGoal) ROS_INFO("Starting group sync write...");

		uint8_t dxl_error = 0; 
		int index = 1;
		int dxl_comm_result_X = COMM_TX_FAIL;
		bool dxl_addparam_result = false;
		dynamixel::GroupBulkWrite groupBulkWrite(portHandler, packetHandler);

		// Create the bulk write message		
		for(int i = 0; i<goalPositions.size(); i++){
			uint8_t tempGoal[4];
			tempGoal[0] = DXL_LOBYTE(DXL_LOWORD(goalPositions[i]));
			tempGoal[1] = DXL_HIBYTE(DXL_LOWORD(goalPositions[i]));
			tempGoal[2] = DXL_LOBYTE(DXL_HIWORD(goalPositions[i]));
			tempGoal[3] = DXL_HIBYTE(DXL_HIWORD(goalPositions[i]));

			dxl_addparam_result = groupBulkWrite.addParam(motors.at(i).id, motors.at(i).ADDR_PRO_GOAL_POSITION, 
													  	  motors.at(i).LEN_PRO_GOAL_POSITION, tempGoal);
			if (dxl_addparam_result != true)
			{
				ROS_INFO_STREAM("Failed to add param!");
			}

		}

		// Perform bulk write
		dxl_comm_result_X = groupBulkWrite.txPacket();
		if (dxl_comm_result_X != COMM_SUCCESS){
			ROS_INFO("%s\n", packetHandler->getTxRxResult(dxl_comm_result_X));
		}

		groupBulkWrite.clearParam();
		if(waitForGoal){
			bool isDone = true;
			do {
					ReadJointPositions(motors);
					ROS_INFO_STREAM("Waiting for move to finish...");
					isDone = true;
					for(int i = 0; i < motors.size(); i++){
						if(abs(goalPositions[i] - motors.at(i).dxl_present_position) > motors.at(i).DXL_MOVING_STATUS_THRESHOLD){
							isDone = false;
						}
					}
				} while (!isDone);
		}
		//joint_timer.start();
		if(waitForGoal) ROS_INFO_STREAM("Finished...");
		return true;
	}

	// Combine Read- Poisition,Velocity,and Current into a GroupBulkRead function to speed up loop

	void ReadJointCurrent(std::vector<X_Motor> &motorGroup){

		dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, motorGroup[0].ADDR_PRO_PRESENT_CURRENT, motorGroup[0].LEN_PRO_PRESENT_CURRENT);
		
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
			bool dxl_getdata_result = groupSyncRead.isAvailable(m.id, m.ADDR_PRO_PRESENT_CURRENT, m.LEN_PRO_PRESENT_CURRENT);
			if (dxl_getdata_result != true)
			{
				ROS_INFO("[ID:%03d] groupSyncRead getdata failed", m.id);
			}
		}

		// Grab data
		for(int i = 0; i<motorGroup.size(); i++){
			motorGroup.at(i).dxl_present_current = groupSyncRead.getData(motorGroup.at(i).id, 
							motorGroup.at(i).ADDR_PRO_PRESENT_CURRENT,
							motorGroup.at(i).LEN_PRO_PRESENT_CURRENT);
		}

	}

	void ReadJointVelocities(std::vector<X_Motor> &motorGroup){

		dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, motorGroup[0].ADDR_PRO_PRESENT_VELOCITY, motorGroup[0].LEN_PRESENT_VELOCITY);
		
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
			bool dxl_getdata_result = groupSyncRead.isAvailable(m.id, m.ADDR_PRO_PRESENT_VELOCITY, m.LEN_PRESENT_VELOCITY);
			if (dxl_getdata_result != true)
			{
				ROS_INFO("[ID:%03d] groupSyncRead getdata failed", m.id);
			}
		}

		// Grab data
		for(int i = 0; i<motorGroup.size(); i++){
			motorGroup.at(i).dxl_present_velocity = groupSyncRead.getData(motorGroup.at(i).id, 
							motorGroup.at(i).ADDR_PRO_PRESENT_VELOCITY,
							motorGroup.at(i).LEN_PRESENT_VELOCITY);
		}

	}

	void ReadJointPositions(std::vector<X_Motor> &motorGroup){

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
		double radians = (raw-offset)/((4096*gear_ratio)/(2*M_PI));
		return radians;
	}
	
	int RadiansToEncoder(double radians, int offset, double gear_ratio){
		int raw = (radians*((4096*gear_ratio)/(2*M_PI)))+offset;
		return raw;
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

	ROS_INFO_STREAM("Initializing " << node_name << " node.");

	Motor_Control mc(nh, nh_p);

}