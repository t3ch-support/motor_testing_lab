#pragma once
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <ros/ros.h>

struct X_Specs{
    int id;
    int op_mode;
    double gear_ratio;
    int reset_state;
    int offset;
    int high_limit;
    int low_limit;
};

class X_Motor {
		
	public:

		const int ADDR_PRO_TORQUE_ENABLE = 64;
		const int ADDR_PRO_GOAL_POSITION = 116;
		const int ADDR_PRO_PRESENT_POSITION = 132;
        const int ADDR_PRO_PRESENT_VELOCITY = 128;
        const int ADDR_PRO_PRESENT_CURRENT = 126;

		const int ADDR_PRO_OPERATING_MODE = 11;
		const int ADDR_PRO_VEL_PROFILE = 112;
		const int ADDR_PRO_ACC_PROFILE = 108;
		const int ADDR_PRO_LOAD = 126;
		const int ADDR_GOAL_PWM = 100;

		const int DXL_MOVING_STATUS_THRESHOLD = 20;
		const int TORQUE_ENABLE = 1;
		const int TORQUE_DISABLE = 0;
		const int DXL_MINIMUM_POSITION_VALUE = 0;
		const int DXL_MAXIMUM_POSITION_VALUE = 4095;
		const int LEN_PRO_GOAL_POSITION = 4;
		const int LEN_PRO_PRESENT_POSITION = 4;
		const int LEN_PRO_PRESENT_CURRENT = 4;
        const int LEN_PRESENT_VELOCITY = 4;
		const int PROFILE_ACCELERATION = 300;
		const int PROFILE_VELOCITY = 300;

		bool isGripper = false;
		int id;
        double gear_ratio = 1;
        int reset_state = 0;
        int offset = 0;
        int high_limit = 0;
        int low_limit = 0;
		int DXL_OPERATING_MODE = 3;
		int dxl_comm_result = COMM_TX_FAIL;
		uint8_t dxl_error = 0;
		int16_t dxl_present_position = 0;
		int16_t dxl_present_velocity = 0;
		int16_t dxl_present_current = 0;

		int16_t dxl_present_load = 0;
        

		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;


		X_Motor();
		
		void init(X_Specs spec, dynamixel::PortHandler *_portHandler,
				  dynamixel::PacketHandler *_packetHandler);

		bool toggleTorque(bool enable);
		bool setVelProfiles();
		bool setAccProfiles();
		bool setOperatingMode();		
		bool setOperatingMode(int operating_mode);
		int16_t readLoad();
		int32_t readPosition();
		int32_t readVelocity();

		bool moveToPos(int goalPosition);		
		bool closeGripper();
        
};


// Implementation

X_Motor::X_Motor(){

}

void X_Motor::init(X_Specs spec, dynamixel::PortHandler *_portHandler,
				  dynamixel::PacketHandler *_packetHandler) {
			id = spec.id;
            gear_ratio = spec.gear_ratio;
            reset_state = spec.reset_state;
            offset = spec.offset;
			DXL_OPERATING_MODE = spec.op_mode;
			portHandler = _portHandler;
			packetHandler = _packetHandler;
            high_limit = spec.high_limit;
            low_limit = spec.low_limit;


			toggleTorque(false);
			setOperatingMode();
			toggleTorque(true);
			setVelProfiles();
			setAccProfiles();
}

bool X_Motor::toggleTorque(bool enable) {
    int torque;
    if (enable) {
        torque = TORQUE_ENABLE;
    }
    else {
        torque = TORQUE_DISABLE;
    }
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, torque, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com Result: " << error);
        return false;
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
        return false;
    }
    else {
        ROS_INFO_STREAM("Torque Toggled on Motor: " << id);
        return true;
    }
}

bool X_Motor::setVelProfiles() {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PRO_VEL_PROFILE, PROFILE_VELOCITY, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
        return false;
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Set Velocity Profile: " << id);
        return true;
    }
}

bool X_Motor::setAccProfiles() {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PRO_ACC_PROFILE, PROFILE_ACCELERATION, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
        return false;
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Set Acceleration Profile: " << id);
        return true;
    }
}

bool X_Motor::setOperatingMode() {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_OPERATING_MODE, DXL_OPERATING_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
        return false;
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Set Operating Mode on Motor: " << id);
        return true;
    }
}

bool X_Motor::setOperatingMode(int operating_mode) {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_OPERATING_MODE, operating_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
        return false;
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
        return false;
    }
    else
    {
        ROS_INFO_STREAM("Set Operating Mode on Motor: " << id);
        return true;
    }
}

int16_t X_Motor::readLoad(){
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRO_LOAD, (uint16_t*)&dxl_present_load, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
    }
    return dxl_present_load;
}

int32_t X_Motor::readPosition() {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
    }
    return dxl_present_position;
}

int32_t X_Motor::readVelocity() {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRO_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
    }
    else if (dxl_error != 0)
    {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
    }
    return dxl_present_velocity;
}

bool X_Motor::moveToPos(int goalPosition) {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PRO_GOAL_POSITION, goalPosition, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        ROS_INFO_STREAM("Com result: " << error);
        return false;
    }
    else if (dxl_error != 0) {
        std::string error = packetHandler->getRxPacketError(dxl_error);
        ROS_INFO_STREAM("Error result: " << error);
        return false;
    }
    if(false){				
        do {
            readPosition();
            ROS_INFO_STREAM("Load: " << readLoad());
            if(dxl_present_load > 200 || dxl_present_load < -200){
                ROS_INFO_STREAM("Tight: " << dxl_present_load);
                //dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, _id, ADDR_PRO_GOAL_POSITION, dxl_present_position-50, &dxl_error);
                break;
            } 				
        } while ((abs(goalPosition - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }else{
        do {
        readPosition();
        } while ((abs(goalPosition - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));
    }
    
    
    ROS_INFO_STREAM("ID " << id << ": " << dxl_present_position);
    return true;
}

bool X_Motor::closeGripper(){

        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_PWM, -350, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            std::string error = packetHandler->getTxRxResult(dxl_comm_result);
            ROS_INFO_STREAM("Com result: " << error);
            return false;
        }
        else if (dxl_error != 0)
        {
            std::string error = packetHandler->getRxPacketError(dxl_error);
            ROS_INFO_STREAM("Error result: " << error);
            return false;
        }		
        readLoad();
        while(dxl_present_load > -90){
            readLoad();
            ROS_INFO_STREAM("Load: " << dxl_present_load);
        }
        
        ROS_INFO_STREAM("Tight grip!");
		return true;
        // toggleTorque(false);
        // setOperatingMode(3);
        // toggleTorque(true);
        // readPosition();
        // moveToPos(dxl_present_position);

        // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, _id, ADDR_GOAL_PWM, -500, &dxl_error);
        // if (dxl_comm_result != COMM_SUCCESS)
        // {
        // 	std::string error = packetHandler->getTxRxResult(dxl_comm_result);
        // 	ROS_INFO_STREAM("Com result: " << error);
        // 	return false;
        // }
        // else if (dxl_error != 0)
        // {
        // 	std::string error = packetHandler->getRxPacketError(dxl_error);
        // 	ROS_INFO_STREAM("Error result: " << error);
        // 	return false;
        // }
}

