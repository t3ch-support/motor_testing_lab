#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "motion_analysis/Driver_XM540W270R.h"


class Cmd_Center {

    private:

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher goal_joint_pub;
    int cmd_pub_rate = 20;
    sensor_msgs::JointState goalState;
    std::vector<X_Specs> active_specs;


    public:

    Cmd_Center(const ros::NodeHandle &node_handle,
               const ros::NodeHandle &node_handle_private)
    :
    nh_(node_handle),
    pnh_(node_handle_private)
    {
        this->init();
    }

    void init(){
        nh_.getParam("/cmd_pub_rate", cmd_pub_rate);	
		goal_joint_pub = nh_.advertise<sensor_msgs::JointState>("cmd_positions", 100);	

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
            active_specs.push_back(spec);

            goalState.position.push_back(-1.0); 
			goalState.velocity.push_back(-1.0); 
			goalState.effort.push_back(-1.0); 
			goalState.name.push_back(std::to_string(spec.id));

        }
        ROS_INFO_STREAM("Motors: " << active_specs.size());
        SpawnPlotter();
        Loop();

    }

    void Loop(){
        ros::Rate loop_rate(cmd_pub_rate); // In Hz
        int t = 0;
        // char choice;
        // ROS_INFO_STREAM("Main Menu");
        // ROS_INFO_STREAM("a: tuning routine");
        // ROS_INFO_STREAM("b: run test");

        // std::cin >> choice;

        // if(choice == 'a'){
        //     sleep(1);
        //     // Turn off torque
        //     ROS_INFO_STREAM("1. Torque is off, press any key to continue.");
        //     std::cin.get();

        //     // Ask for joints to be arranged
        //     ROS_INFO_STREAM("2. Arrange joints to zero position, then press any key to continue.");
        //     std::cin.get();


        //     // Turn on torque, edit motors.yaml->offset
        //     ROS_INFO_STREAM("3. Torque is on, press any key to continue.");
        //     std::cin.get();


        //     // Ready
        //     ROS_INFO_STREAM("4. Ready for test, press any key to run test, or Ctrl-C to cancel.");
        //     std::cin.get();
            


        // }else
        // if(choice == 'b'){
        //     ROS_INFO_STREAM("Beginning test.");         
        // }else{
        //     ROS_INFO_STREAM("Invalid input. Try again.");
        //     Loop();            
        // }

		while (ros::ok()){
            // ROS_INFO_STREAM("Running test.");         
            
            goalState.header.stamp = ros::Time::now();
            for(int i = 0; i<active_specs.size(); i++){
                double amp = active_specs.at(i).high_limit - active_specs.at(i).reset_state;
                goalState.position.at(i) = amp*sin(0.04*t)+active_specs.at(i).reset_state;
            }
            t++;
            goal_joint_pub.publish(goalState);              
            ros::spinOnce();
			loop_rate.sleep();
        }
    }

    void SpawnPlotter(){
        std::stringstream ss;

        ss << "rosrun rqt_plot rqt_plot __name:=cmd_vs_pos ";
        for(int i = 0; i<active_specs.size(); i++){
            ss << "/joint_states/position[" << i << "] ";
            ss << "/joint_states/velocity[" << i << "] ";
            ss << "/joint_states/effort[" << i << "] ";
            ss << "/cmd_positions/position[" << i << "] ";

        }
        ss << "&";
        system(ss.str().c_str());
    }

};

int main(int argc, char *argv[]) 
{
	std::string node_name = "cmd_center";
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh("");
	ros::NodeHandle nh_p("~");

	ROS_INFO_STREAM("Initializing " << node_name << " node");

	Cmd_Center cc(nh, nh_p);

}