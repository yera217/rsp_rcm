
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>

#include <rtt/TaskContext.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/InputPort.hpp>

#include <ur5_rcm_msgs/OptRCM.h>

#include <math.h>




class rtt_rcm_depl: public RTT::TaskContext {
private:

	double x_des;
	double y_des;
	double z_des;

	double x_RCM;
	double y_RCM;
	double z_RCM;

	std::array<double,3> xyz_RCM;
	std::array<double,3> xyz_des;

	RTT::InputPort<sensor_msgs::JointState> port_jnt_state;                                                   
	RTT::OutputPort<std_msgs::Float64MultiArray> port_cmd_vel;

	ros::NodeHandle nh;                                          
	ros::ServiceClient cli;

	KDL::ChainFkSolverPos_recursive* fk_frame_solver;
	KDL::ChainFkSolverPos_recursive* fk_frame_cl_solver;

	KDL::ChainJntToJacSolver* J_tip_solver;
	KDL::ChainJntToJacSolver* J_cl_solver;

	KDL::Tree tree;
	KDL::Chain chain;
	KDL::Chain chain_cl;

	std::string robot_description_string;

public:
	rtt_rcm_depl( const std::string& name );
	~rtt_rcm_depl(){}

	virtual bool configureHook();

	virtual bool startHook();

	virtual void updateHook();

	virtual void stopHook();

	virtual void cleanupHook();

	//function for dot product                                                                                  
    double dot_prod(std::array<double,3> vector_a, std::array<double,3> vector_b) {
        double prod = 0.0;
        for (int i = 0; i < 3; i++)
                prod += vector_a[i] * vector_b[i];
        return prod;
    }

	//function for vector subtraction
	std::array<double,3> subtract(std::array<double,3> vector_a, std::array<double,3> vector_b) {
		std::array<double,3> res;
		for (int i = 0; i < 3; i++)
			res[i]=vector_a[i]-vector_b[i];
		return res;
	}
	//function for vector addition
	std::array<double,3>  add(std::array<double,3> vector_a, std::array<double,3> vector_b) {
		std::array<double,3> res;
		for (int i = 0; i < 3; i++)
			res[i]=vector_a[i]+vector_b[i];
		return res;
	}
	//function for vector scaling
	std::array<double,3>  scale(double k, std::array<double,3> vector_b) {
		std::array<double,3> res;
		for (int i = 0; i < 3; i++)
			res[i]=k*vector_b[i];
		return res;
	}
	//function norm
	double norm(std::array<double,3> vector_a) {
		double running_sum=0;
		for (int i = 0; i < 3; i++)
			running_sum+=vector_a[i]*vector_a[i];
		return sqrt(running_sum);
	}
};






