#include <ros/ros.h>
#include <awe_nmpc.h>

namespace fw_nmpc {

FwNMPC::FwNMPC()
    : LOOP_RATE(20.0),  // MAGIC NUMBER
      TSTEP(0.1),  // MAGIC NUMBER
      FAKE_SIGNALS(0),
      t_lastctrl { 0 },
      bModeChanged(false),
      last_ctrl_mode(0),
      obctrl_en_(0),
      last_wp_idx_(-1),
      bYawReceived(false),
      last_yaw_msg_(0.0f),
      track_error_lat_(0.0f),
      track_error_lon_(0.0f),
      T_b_lat_(1.0),
      W_scale_ { 0 } {

  ROS_INFO("Instance of NMPC created");

  /* subscribers */
  /*  aslctrl_data_sub_ = nmpc_.subscribe("/mavros/aslctrl/data", 1,
   &FwNMPC::aslctrlDataCb, this); */
  /*  glob_pos_sub_ = nmpc_.subscribe("/mavros/global_position/global", 1,
   &FwNMPC::globPosCb, this); */  //TODO: switch to UTM - stop doing own ll2ne
  /*  odom_sub_ = nmpc_.subscribe("/mavros/global_position/local", 1,
   &FwNMPC::odomCb, this); */
  /*  ekf_ext_sub_ = nmpc_.subscribe("/mavros/aslekf_extended", 1,
   &FwNMPC::ekfExtCb, this); */
  /*  nmpc_params_sub_ = nmpc_.subscribe("/mavros/nmpc_params", 1,
   &FwNMPC::nmpcParamsCb, this); */
  /*  waypoint_list_sub_ = nmpc_.subscribe("/mavros/mission/waypoints", 1,
   &FwNMPC::waypointListCb, this); */
  /*  current_wp_sub_ = nmpc_.subscribe("/mavros/mission/current_wp", 1,
   &FwNMPC::currentWpCb, this); */
  /*  home_wp_sub_ = nmpc_.subscribe("/mavros/home_position", 1, &FwNMPC::homeWpCb,
   this); */
  /*  aslctrl_debug_sub_ = nmpc_.subscribe("/mavros/aslctrl/debug", 1,
   &FwNMPC::aslctrlDebugCb, this); */

  /* publishers */
  /*  obctrl_pub_ = nmpc_.advertise<mavros_msgs::AslObCtrl>("/nmpc/asl_obctrl", 10,
   true); */
  /*  nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NmpcInfo>("/nmpc/info", 10, true); */
  /*  acado_vars_pub_ = nmpc_.advertise<fw_ctrl::AcadoVars>("/nmpc/acado_vars", 10,
   true); */

  // Initialize NMPC Indexes //
  x_index = {0, 1, 2, 3, 4, 5};
  u_index = {0, 1, 2};
  p_index = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

  std::cout << "here 10";
  // Initialize NMPC Parameters //
  parameter[p_index.vw] = 10;
  parameter[p_index.r] = 220;
  parameter[p_index.r_dot] = 10 * 0.23;
  parameter[p_index.circle_azimut] = 0;
  parameter[p_index.circle_elevation] = 30.0 / 180 * M_PI;
  parameter[p_index.circle_angle] = atan(75 / 220);
  parameter[p_index.m] = 27.53;
  parameter[p_index.cla] = 0.9;
  parameter[p_index.cda] = 0.07;
  parameter[p_index.weight_tracking] = 1;
  parameter[p_index.weight_power] = 1;
  std::cout << "here 11";
  // Initialize Parameters from Launch File //
  // get loop rate
  nmpc_.getParam("/nmpc/loop_rate", LOOP_RATE);
  // get model discretization step
  nmpc_.getParam("/nmpc/time_step", TSTEP);
  // fake signals
  nmpc_.getParam("/nmpc/fake_signals", FAKE_SIGNALS);

}  // constructor


int FwNMPC::initNMPC() {
  // std::cout << LOOP_RATE;
  // Initialize ACADO variables
  initACADOVars();
  ROS_INFO("initNMPC: ACADO variables initialized");

  // Initialize the solver.
  int RET = 0;  //initializeSolver();

  return RET;
}


void FwNMPC::initACADOVars() {
  // TODO: maybe actually wait for all subscriptions to be filled here before initializing?
  std::cout << "here 1";
  // put something reasonable here.. NOT all zeros, solver is initialized from here //
  double X[NX] = { parameter[p_index.circle_azimut],
      parameter[p_index.circle_elevation]+parameter[p_index.circle_angle],
      -0.5*M_PI, 0.0, 50.0, 0.0 };
  double U[NU] = { 0.0, 0.0, 0.0 };
  //double OD[NOD];
  double Y[NY] = {0.0};  // Set all entries to 0
  double W[NY] = { 100.0, 20.0, 1.0, 100.0, 1.0};
  double WN[NY] = { 100.0, 0.0, 0.0 };

  // these set a constant value for each variable through the horizon //

  // states
  for (int i = 0; i < N + 1; ++i) {
    for (int j = 0; j < NX; ++j)
      acadoVariables.x[i * NX + j] = X[j];
  }

  // controls
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < NU; ++j)
      acadoVariables.u[i * NU + j] = U[j];
  }

  // online data
  for (int i = 0; i < N + 1; ++i) {
    for (int j = 0; j < NOD; ++j)
      acadoVariables.od[i * NOD + j] = parameter[j];
  }

  // references
  for (int i = 0; i < N + 1; ++i) {
    for (int j = 0; j < NY; ++j)
      acadoVariables.y[i * NY + j] = Y[j];
  }

  // weights
  memset(acadoVariables.W, 0, sizeof(acadoVariables.W));  // fill all with zero
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < NY; ++j) {
      acadoVariables.W[ NY * NY * i + NY * j + j] = W[j];  // fill diagonals
    }
  }
  memset(acadoVariables.WN, 0, sizeof(acadoVariables.WN));  // fill all with zero
  for (int i = 0; i < NYN; ++i) {
    acadoVariables.WN[i * NYN + i] = WN[i];  // fill diagonals
  }
}

double FwNMPC::getLoopRate() {
  return LOOP_RATE;
}

double FwNMPC::getTimeStep() {
  return TSTEP;
}

void FwNMPC::shutdown() {
  ROS_INFO("Shutting down NMPC...");
  ros::shutdown();
}

}
;
// namespace awe_nmpc

int main(int argc, char **argv) {
  // initialize node //
  ros::init(argc, argv, "awe_nmpc");
  std::cout << "here 0";
  fw_nmpc::FwNMPC nmpc;
  ros::spinOnce();

  // wait for required subscriptions //
  //nmpc.reqSubs();

  // initialize states, params, and solver //
  int ret = nmpc.initNMPC();

  if (ret != 0) {
    ROS_ERROR("initNMPC: error in qpOASES QP solver.");
    return 1;
  }
  /*
   //
   // NMPC loop
   //
   double loop_rate = nmpc.getLoopRate();
   ros::Rate nmpc_rate(loop_rate);
   ROS_ERROR("fw_nmpc: entering NMPC loop");
   while (ros::ok()) {

   // empty callback queues //
   ros::spinOnce();

   // nmpc iteration step //
   ret = nmpc.nmpcIteration();

   if (ret != 0) {
   ROS_ERROR("nmpc_iteration: error in qpOASES QP solver.");
   return 1;
   }

   // sleep //
   nmpc_rate.sleep();
   }

   */
  ROS_ERROR("awe_nmpc: closing...");

  return 0;
}  // end main

