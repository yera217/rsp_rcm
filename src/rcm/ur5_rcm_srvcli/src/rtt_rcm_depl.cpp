#include <ur5_rcm_srvcli/rtt_rcm_depl.hpp>
#include <rtt/Component.hpp>


rtt_rcm_depl::rtt_rcm_depl (const std::string& name) :
  TaskContext(name),
  nh(nh),fk_frame_solver(NULL),J_tip_solver(NULL),
  fk_frame_cl_solver(NULL),J_cl_solver(NULL),
  port_jnt_state("Input Joint states"),
  port_cmd_vel("Command joint velocities"),
  x_des(-0.3),y_des(-0.3),z_des(0.25),
  x_RCM(-0.25),y_RCM(-0.25),z_RCM(0.3){
  
  std::cout << "rtt_rcm::rtt_rcm"  << std::endl;

  cli = nh.serviceClient<ur5_rcm_msgs::OptRCM>("opt_rcm");

  addPort("JntStates", port_jnt_state).doc("Input joint positions");
  addPort("CmdVels", port_cmd_vel).doc("Command velocities");

  addProperty("robot_description", robot_description_string);
  addProperty("x_des", x_des);
  addProperty("y_des", y_des);
  addProperty("z_des", z_des);
  addProperty("x_RCM", x_RCM);
  addProperty("y_RCM", y_RCM);
  addProperty("z_RCM", z_RCM);

}



bool rtt_rcm_depl::startHook(){

  std::cout << "rtt_rcm::startHook" << std::endl;
  return true;
}



bool rtt_rcm_depl::configureHook(){

  std::cout << "rtt_rcm::configureHook" << std::endl;
  
  std::string robot_description_value;

  nh.param( "robot_description" , robot_description_value, std::string() );
  
  if(kdl_parser::treeFromString(robot_description_value, tree)) {
    if(tree.getChain("base_link", "needle_driver_tip", chain)) {
      //std::cout << chain.getNrOfJoints() << std::endl;
      
      fk_frame_solver = new KDL::ChainFkSolverPos_recursive( chain );
      J_tip_solver = new KDL::ChainJntToJacSolver( chain );
    } else {
		std::cout << "Could not read chain" << std::endl;
    }
  }

  return true;
}





void rtt_rcm_depl::updateHook(){
	// std::cout << "rtt_rcm::UpdateHook" << std::endl;
	xyz_RCM[0]=x_RCM; xyz_RCM[1]=y_RCM; xyz_RCM[2]=z_RCM;
  xyz_des[0]=x_des; xyz_des[1]=y_des; xyz_des[2]=z_des;

	////////// Reading data from input port and converting to KDL type /////////
  sensor_msgs::JointState js;
	if(port_jnt_state.read(js)==RTT::NewData) {
    KDL::JntArray q_in(js.position.size() );
    for (int i=0; i<3; i++) {
      q_in(i)=js.position[2-i];
    }
    for (int i=3; i<6; i++) {
      q_in(i)=js.position[i];
    }
    // std::cout << "Q: " << q_in(0) << " " << q_in(1) << " " << q_in(2) <<" " << q_in(3) <<" " << q_in(4) <<"  " << q_in(5) <<std::endl;

    ////////////// Obtaining necessary KDL FK and Jacobians /////////////
    KDL::Frame kdl_fk_frame;


    fk_frame_solver -> JntToCart( q_in, kdl_fk_frame);


    KDL::Jacobian kdl_J_tip(6);
    J_tip_solver -> JntToJac(q_in, kdl_J_tip);

    // std::cout << "J_TIP [0,0] "<< kdl_J_tip.columns() << kdl_J_tip.rows() << std::endl;

    std::array<double,3> x_curr;   //Current xyz position of the tool tip
    x_curr[0]=kdl_fk_frame.p.x(); x_curr[1]=kdl_fk_frame.p.y(); x_curr[2]=kdl_fk_frame.p.z();
    //std::cout << "x_curr" << x_curr[0] << " " << x_curr[1] << " "<< x_curr[2] << " "<< std::endl;

    int size_Jtip = 3*js.position.size();
    double J_tip[size_Jtip];
    for (int i=0; i<3; i++) {
      for (int j=0; j<js.position.size(); j++) {
        J_tip[i*js.position.size()+j]=kdl_J_tip(i,j);
      }
    }


    //////////// Finding closest point on the shaft  /////////////////

    // Defining chain that needed to calculate J at the closest to RCM point on the shaft
    
    if(tree.getChain("base_link", "needle_driver_shaft", chain_cl)) {
      // std::cout << chain_cl.getNrOfJoints() << std::endl;
    } else {
      std::cout << "Could not read chain_cl" << std::endl;
    }


    // Position xyz of the tool base (tb)
    fk_frame_cl_solver = new KDL::ChainFkSolverPos_recursive( chain_cl );
    KDL::Frame kdl_fk_frame_tb;
    fk_frame_cl_solver -> JntToCart( q_in, kdl_fk_frame_tb);
    std::array<double,3> xyz_tb;   //xyz position of the tool base
    xyz_tb[0]=kdl_fk_frame_tb.p.x(); xyz_tb[1]=kdl_fk_frame_tb.p.y(); xyz_tb[2]=kdl_fk_frame_tb.p.z();

    std::array<double,3> xyz_tt = x_curr;



    double lamda = rtt_rcm_depl::dot_prod( rtt_rcm_depl::subtract(xyz_RCM,xyz_tb), rtt_rcm_depl::subtract(xyz_tt,xyz_tb)) / rtt_rcm_depl::dot_prod( rtt_rcm_depl::subtract(xyz_tt,xyz_tb), rtt_rcm_depl::subtract(xyz_tt,xyz_tb));
    // std::cout << "xyz_tb" << xyz_tb[0] << " " << xyz_tb[1] << " "<< xyz_tb[2] << " "<< std::endl;
    lamda = std::max(0.0, std::min(lamda,1.0));

    // Finding xyz coordinates of the point on the shaft closest to RCM
    std::array<double,3> xyz_cl = rtt_rcm_depl::add(xyz_tb, rtt_rcm_depl::scale(lamda,rtt_rcm_depl::subtract(xyz_tt,xyz_tb) ));



    // Calculating Jacobian at the closest point
    double dist_shaft_cl = rtt_rcm_depl::norm(rtt_rcm_depl::subtract(xyz_cl,xyz_tb));

    chain_cl.addSegment(KDL::Segment( KDL::Joint(KDL::Joint::None), KDL::Frame(KDL::Vector(dist_shaft_cl,0,0))));

    
    J_cl_solver = new KDL::ChainJntToJacSolver( chain_cl );
    KDL::Jacobian kdl_J_cl(6);

    J_cl_solver -> JntToJac(q_in, kdl_J_cl);


    int size_Jcl = 3*js.position.size();
    double J_cl[size_Jtip];
    for (int i=0; i<3; i++) {
      for (int j=0; j<js.position.size(); j++) {
        J_cl[i*js.position.size()+j]=kdl_J_cl(i,j);
      }
    }


    // Converting to srv format
    std_msgs::Float64MultiArray fk, jac_tip, jac_cl;
    fk.layout.dim.push_back(std_msgs::MultiArrayDimension());
    jac_tip.layout.dim.push_back(std_msgs::MultiArrayDimension());
    jac_cl.layout.dim.push_back(std_msgs::MultiArrayDimension());


    fk.layout.dim[0].size=12;
    for (int i=0; i<3; i++) {
      fk.data.push_back( x_curr[i] );
    }

    for (int i=0; i<3; i++) {
      fk.data.push_back( xyz_cl[i] );
    }

    for (int i=0; i<3; i++) {
      fk.data.push_back( xyz_RCM[i] );
    }

    for (int i=0; i<3; i++) {
      fk.data.push_back( xyz_des[i] );
    }
    
    jac_tip.layout.dim[0].size=size_Jtip;
    for (int i=0; i<size_Jtip; i++) {
      jac_tip.data.push_back( J_tip[i] );
    }

    jac_cl.layout.dim[0].size=size_Jtip;
    for (int i=0; i<size_Jtip; i++) {
      jac_cl.data.push_back( J_cl[i] );
    }

    

     // Calling opt_rcm service (pyOpt based optimization for RCM)
    ur5_rcm_msgs::OptRCM srv;
    srv.request.fk=fk;
    srv.request.jac_tip=jac_tip;
    srv.request.jac_cl=jac_cl;


    // std::cout << "&&&&&&&&&&" << srv.request.fk.data[0] << " " << srv.request.fk.data[1] << " " <<srv.request.fk.data[2] << " " <<
    // srv.request.fk.data[3] << srv.request.fk.data[4] << " " << srv.request.fk.data[5] << std::endl;

    // std::cout << "&&&&&&&&&&" << srv.request.jac_tip.data[0] << " " << srv.request.jac_tip.data[1] << " " <<srv.request.jac_tip.data[2] << " " <<
    // srv.request.jac_tip.data[3] << srv.request.jac_tip.data[4] << " " << srv.request.jac_tip.data[5] << std::endl;



    std_msgs::Float64MultiArray cmd_vel;
    if(cli.call(srv)) {
      cmd_vel = srv.response.cmd_vel;

    } else {
      ROS_ERROR("Failed to call service.");
    }
    // std::cout << "CMD_VEL  " << cmd_vel.data[0] << cmd_vel.data[1] << cmd_vel.data[2] << std::endl;
    port_cmd_vel.write(cmd_vel);
  }
}
	






void rtt_rcm_depl::stopHook(){
  std::cout << "rtt_rcm::stopHook" << std::endl;
}
void rtt_rcm_depl::cleanupHook(){
  std::cout << "rtt_rcm::cleanupHook" << std::endl;
}







ORO_CREATE_COMPONENT(rtt_rcm_depl)


