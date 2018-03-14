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
                                  &FwNMPC::globPosCb, this); */ //TODO: switch to UTM - stop doing own ll2ne
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
}

int FwNMPC::initNMPC() {
  // get loop rate
  nmpc_.getParam("/nmpc/loop_rate", LOOP_RATE);

  // get model discretization step
  nmpc_.getParam("/nmpc/time_step", TSTEP);

  // fake signals
  nmpc_.getParam("/nmpc/fake_signals", FAKE_SIGNALS);

  // Initialize ACADO variables
  //initACADOVars();
  ROS_INFO("initNMPC: ACADO variables initialized");

  // Initialize the solver.
  int RET = 0;//initializeSolver();

  return RET;
}


void FwNMPC::initACADOVars() {

  //TODO: maybe actually wait for all subscriptions to be filled here before initializing?

  /*
  double y_uT_ref;
  nmpc_.getParam("/nmpc/y_ref/uT", y_uT_ref);
  double y_theta_ref;
  nmpc_.getParam("/nmpc/y_ref/theta_ref", y_theta_ref);
  double ddot_clmb;
  nmpc_.getParam("/nmpc/od/ddot_clmb", ddot_clmb);
  double ddot_sink;
  nmpc_.getParam("/nmpc/od/ddot_sink", ddot_sink);
  */

  /*
  // put something reasonable here.. NOT all zeros, solver is initialized from here //
  double X[NX] = { 0.0, 0.0, 0.0, 13.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      y_uT_ref, 0.0 };
  double U[NU] = { y_uT_ref, 0.0, y_theta_ref };
  double OD[NOD] = { 2.0, 0.0, 0.0, -100.0, 100.0, 0.0, 0.0, 2.0, 0.0, 0.0,
      -100.0, 100.0, 0.0, 0.0, 30.0, 0.8, 0.0, 0.0, 0.0, 0.1396, -0.0524,
      0.0349, 1.0, 0.5, ddot_clmb, ddot_sink };

  double Y[NY] = { 0.0, 0.0, 13.5, 0.0, 0.0, 0.0, 0.0, 0.0, y_uT_ref, 0.0,
      y_theta_ref };
  double W[NY] = { 400.0, 400.0, 150.0, 60.0, 60.0, 5.0, 120.0, 100.0, 200.0,
      100.0, 60.0 };

  nmpc_.getParam("nmpc/w_scale/q1", W_scale_[0]);
  nmpc_.getParam("nmpc/w_scale/q2", W_scale_[1]);
  nmpc_.getParam("nmpc/w_scale/q3", W_scale_[2]);
  nmpc_.getParam("nmpc/w_scale/q4", W_scale_[3]);
  nmpc_.getParam("nmpc/w_scale/q5", W_scale_[4]);
  nmpc_.getParam("nmpc/w_scale/q6", W_scale_[5]);
  nmpc_.getParam("nmpc/w_scale/q7", W_scale_[6]);
  nmpc_.getParam("nmpc/w_scale/q8", W_scale_[7]);
  nmpc_.getParam("nmpc/w_scale/q9", W_scale_[8]);
  nmpc_.getParam("nmpc/w_scale/q10", W_scale_[9]);
  nmpc_.getParam("nmpc/w_scale/q11", W_scale_[10]);

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
      acadoVariables.od[i * NOD + j] = OD[j];
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
      acadoVariables.W[ NY * NY * i + NY * j + j] = W[j] / W_scale_[j];  // fill diagonals
    }
  }
  memset(acadoVariables.WN, 0, sizeof(acadoVariables.WN));  // fill all with zero
  for (int i = 0; i < NYN; ++i) {
    acadoVariables.WN[i * NYN + i] = W[i] / W_scale_[i];  // fill diagonals
  }
  */
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


};  // namespace awe_nmpc




int main(int argc, char **argv) {
  // initialize node //
  ros::init(argc, argv, "awe_nmpc");
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


