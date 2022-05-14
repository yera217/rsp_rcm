#include <ur5_rcm_srvcli/rtt_rcm.hpp>

int main ( int argc, char** argv ) {

  ros::init( argc, argv, "rcm_cli_node" );
  ros::NodeHandle nh;

  rtt_rcm rcm_cli(nh);

  return 0;

}