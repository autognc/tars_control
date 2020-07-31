#include <ros/ros.h>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mg_msgs/follow_PolyPVA_XY_trajectoryAction.h"
#include "mg_msgs/minAccXYWpPVA.h"
#include "helper.h"


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_poly_control");

  actionlib::SimpleActionClient<mg_msgs::follow_PolyPVA_XY_trajectoryAction> ac("tars_polyControl", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action


/*

  // Fill dummy polynomials
  mg_msgs::PolyPVA polyX;
  mg_msgs::PolyPVA polyY;


  // float t0, tf;
  // uint order;
  // float PosCoeff, VelCoeff, AccCoeff;

  // t0 = 0;
  // tf = 10;
  // order = 3;
  // PosCoeff = -0.25;
  // VelCoeff = 0.005;
  // AccCoeff = 0.0005;

  polyX.t0 = 0;
  polyX.tf = 10;
  polyX.order = 3;
  polyX.PosCoeff = -0.25;
  polyX.VelCoeff = 0.005;
  polyX.AccCoeff = 0.0005;

  polyY.t0 = 0;
  polyY.tf = 10;
  polyY.order = 3;
  polyY.PosCoeff = -0.25;
  polyY.VelCoeff = 0.005;
  polyY.AccCoeff = 0.0005;

  // Set and send goal
  mg_msgs::follow_PolyPVA_XY_trajectoryGoal goal;

  goal.X = polyX;
  goal.Y = polyY;

  ac.sendGoal(goal);

  //exit
  return 0;
}

*/




  // This next stuff is writing out ideas using minAccXYWpPVA.srv

  double tf = 10.0;

  std::vector<mg_msgs::PolyPVA> polyX;
  std::vector<mg_msgs::PolyPVA> polyY;

  Eigen::Vector2d init_point = {0,0};
  Eigen::Vector2d final_point = {1,1};



  mg_msgs::PVA_request Wp0;
  Wp0.use_pos = false;
  Wp0.use_vel = false;
  Wp0.use_acc = false;
  Wp0.use_yaw = false;
  Wp0.use_yaw_dot = false;

  mg_msgs::PVA_request Wp1;
  Wp1.use_pos = false;
  Wp1.use_vel = false;
  Wp1.use_acc = false;
  Wp1.use_yaw = false;
  Wp1.use_yaw_dot = false;

  mg_msgs::PVA_request Wp_f;
  Wp_f.use_pos = false;
  Wp_f.use_vel = false;
  Wp_f.use_acc = false;
  Wp_f.use_yaw = false;
  Wp_f.use_yaw_dot = false;



  ros::NodeHandle nh;
  std::string service_name = "minAccSolver";
  ros::ServiceClient client = nh.serviceClient<mg_msgs::minAccXYWpPVA>(service_name);


  // Set first point
  Wp0.Pos = helper::eigenvec2d2rospoint(init_point);
  Wp0.Vel = helper::setvector3(0.0, 0.0, 0.0);
  Wp0.time = 0.0;
  Wp0.use_pos = true;
  Wp0.use_vel = true;

  // An intermediate point is needed for the minimum snap solver
  Wp1.Pos = helper::eigenvec2d2rospoint(0.5*(init_point + final_point));
  Wp1.time = 0.5*tf;
  Wp1.use_pos = true;

  // Set final point
  Wp_f.Pos = helper::eigenvec2d2rospoint(final_point);
  Wp_f.Vel = helper::setvector3(0.0, 0.0, 0.0);
  Wp_f.time = tf;
  Wp_f.use_pos = true;
  Wp_f.use_vel = true;

  // Push points into waypoint structure
  std::vector<mg_msgs::PVA_request> PVA_array;
  PVA_array.push_back(Wp0);
  PVA_array.push_back(Wp1);
  PVA_array.push_back(Wp_f);

  mg_msgs::minAccXYWpPVA srv;
  srv.request.PVA_array = PVA_array;
  srv.request.max_vel = 10000;  // No constraint on those in p2p trajectories
  srv.request.max_acc = 10000;
  srv.request.max_jerk = 10000;
  
  // if (client.call(srv)) {
  //     ROS_INFO("[%s mission_node] Service returned succesfully with point to point trajectory!", ns_.c_str());
  // } else {
  //     ROS_ERROR("[%s mission_node] Failed to call service ""%s"" for point to point trajectory...", ns_.c_str(),
  //               client.getService().c_str());
  //     return false;
  // }

  polyX = srv.response.X;
  polyY = srv.response.Y;


  // Set and send goal
  mg_msgs::follow_PolyPVA_XY_trajectoryGoal goal;

  goal.X = polyX;
  goal.Y = polyY;

  ac.sendGoal(goal);

  return 0;
}