#include <ros/ros.h>
#include <ros/package.h>
#include <awe_nmpc.h>

namespace fw_nmpc {

FwNMPC::FwNMPC()
    : LOOP_RATE(10.0),  // MAGIC NUMBER
      TSTEP(0.1),  // MAGIC NUMBER
      FAKE_SIGNALS(0),
      t_lastctrl { ros::Time::now() },
      bModeChanged(false),
      last_ctrl_mode(0),
      obctrl_en_(0),
      bYawReceived(false),
      last_yaw_msg_(0.0f) {
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
  path_pub_ = nmpc_.advertise<nav_msgs::Path>("nmpc_predicted", 1);
  /*  obctrl_pub_ = nmpc_.advertise<mavros_msgs::AslObCtrl>("/nmpc/asl_obctrl", 10,
   true); */
  /*  nmpc_info_pub_ = nmpc_.advertise<fw_ctrl::NmpcInfo>("/nmpc/info", 10, true); */
  /*  acado_vars_pub_ = nmpc_.advertise<fw_ctrl::AcadoVars>("/nmpc/acado_vars", 10,
   true); */

  // Initialize NMPC Indexes //

  x_index.psi = 0;  // states
  x_index.theta = 1;
  x_index.gamma = 2;
  x_index.phi = 3;
  x_index.vt = 4;
  x_index.phi_des = 5;
  u_index.dphi = 0;  // control inputs
  u_index.phi_slack = 1;
  u_index.theta_slack = 2;
  p_index.vw = 0;  // parameters
  p_index.r = 1;
  p_index.r_dot = 2;
  p_index.circle_azimut = 3;
  p_index.circle_elevation = 4;
  p_index.circle_angle = 5;
  p_index.m = 6;
  p_index.cla = 7;
  p_index.cda = 8;
  p_index.weight_tracking = 9;
  p_index.weight_power = 10;

  // Initialize NMPC Parameters //
  parameter[p_index.vw] = 10.0;
  parameter[p_index.r] = 220.0;
  parameter[p_index.r_dot] = 10.0 * 0.23;
  parameter[p_index.circle_azimut] = 0.0;
  parameter[p_index.circle_elevation] = 30.0 / 180 * M_PI;
  parameter[p_index.circle_angle] = atan(75.0 / 220.0);
  parameter[p_index.m] = 27.53;
  parameter[p_index.cla] = 0.9;
  parameter[p_index.cda] = 0.07;
  parameter[p_index.weight_tracking] = 1.0;
  parameter[p_index.weight_power] = 1.0;

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
  int RET = acado_initializeSolver();
  // Initialize all nodes with a forward simulation with default (0) control input
  acado_initializeNodesByForwardSimulation();

  return RET;
}


void FwNMPC::initACADOVars() {
  // TODO: maybe actually wait for all subscriptions to be filled here before initializing?
  // put something reasonable here.. NOT all zeros, solver is initialized from here //
  double X[NX] = { parameter[p_index.circle_azimut],
      parameter[p_index.circle_elevation]+parameter[p_index.circle_angle],
      -0.5*M_PI, 0.0, 50.0, 0.0 };
  double U[NU] = { 0.0, 0.0, 0.0 };
  //double OD[NOD];
  double Y[NY] = {0.0};  // Set all entries to 0
  double W[NY] = { 100.0, 0*20.0, 1.0, 100.0, 0.01*200.0};
  double WN[NY] = { 100.0, 0.0, 0*20.0 };

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
  /*for (int i = 0; i < N; ++i) {
    for (int j = 0; j < NY; ++j) {
      acadoVariables.W[ NY * NY * i + NY * j + j] = W[j];  // fill diagonals
      std::cout << "loop i " << i << " j " << j << std::endl;
    }
  }*/
  for (int i = 0; i < NY; ++i) {
    acadoVariables.W[i * NY + i] = W[i];  // fill diagonals
  }

  memset(acadoVariables.WN, 0, sizeof(acadoVariables.WN));  // fill all with zero
  for (int i = 0; i < NYN; ++i) {
    acadoVariables.WN[i * NYN + i] = WN[i];  // fill diagonals
  }
  // ROS_INFO_STREAM("State set to: vt: " << acadoVariables.x[4]);
  ROS_INFO_STREAM("State set to: Psi: " << X[0] << " Theta: " << X[1] << " gamma: " << X[2] << " psi: " << X[3] << " vt: " << X[4] << " psi_des: " << X[5]);
}


int FwNMPC::nmpcIteration() {
  // elapsed time reusable var //
  ros::Duration t_elapsed;
  // start nmpc iteration timer --> //
  ros::Time t_iter_start = ros::Time::now();
  // various timer initializations //
  uint64_t t_ctrl = 0;  // time elapsed since last control action was published (stays zero if not in auto mode)
  uint64_t t_solve = 0; // time elapsed during nmpc preparation and feedback step (solve time)
  uint64_t t_update = 0;  // time elapsed during array updates
  uint64_t t_wp_man = 0;  // time elapsed during waypoint management

  // initialize returns //
  int RET[2] = { 0, 0 };

  // check mode //  //TODO: should include some checking to make sure not on ground/ other singularity ridden cases //TODO: this does not cover RC loss..
  /*
  if (last_ctrl_mode != 5)
    bModeChanged = true;
  last_ctrl_mode = subs_.aslctrl_data.aslctrl_mode;
  */
  if (true) {  // subs_.aslctrl_data.aslctrl_mode == 5

    // start update timer --> //
    ros::Time t_update_start = ros::Time::now();

    int obctrl_status = 0;

    if (false) {  // bModeChanged
      // first time in loop //

      // initHorizon BEFORE Y, this is to make sure controls and prev_horiz are reinitialized before reaching Y update.
      // initHorizon();
      // updateACADO_Y();  // not needed for awe_nmpc
      updateACADO_W();

      bModeChanged = false;
    } else {
      // regular update

      // update ACADO states/references/weights //
      updateACADO_X0();  // note this shifts augmented states
      // updateACADO_Y();  // not needed for awe_nmpc
      updateACADO_W();
    }

    // update time in us <-- //
    t_elapsed = ros::Time::now() - t_update_start;
    t_update = t_elapsed.toNSec() / 1000;

    // update ACADO online data //
    updateACADO_OD();

    // start nmpc solve timer --> //
    ros::Time t_solve_start = ros::Time::now();

    // ROS_INFO_STREAM("State before preparation: vt: " << acadoVariables.x[4]);
    // Prepare first step //
    RET[0] = acado_preparationStep();
    if (RET[0] != 0) {
      ROS_ERROR("nmpcIteration: preparation step error, code %d", RET[0]);
      obctrl_status = RET[0];
    }
    // ROS_INFO_STREAM("State before feedback: vt: " << acadoVariables.x[4]);

    // Perform the feedback step. //
    RET[1] = acado_feedbackStep();
    if (RET[1] != 0) {
      ROS_ERROR("nmpcIteration: feedback step error, code %d", RET[1]);
      obctrl_status = RET[1];  //TODO: find a way that doesnt overwrite the other one...
    }
    // ROS_INFO_STREAM("State after feedback: vt: " << acadoVariables.x[4]);

    // solve time in us <-- //
    t_elapsed = ros::Time::now() - t_solve_start;
    t_solve = t_elapsed.toNSec() / 1000;

    // nmpc iteration time in us (approximate for publishing to pixhawk) <-- //
    t_elapsed = ros::Time::now() - t_iter_start;
    uint64_t t_iter_approx = t_elapsed.toNSec() / 1000;
    ROS_INFO_STREAM("NMPC Step completed took: " << t_iter_approx*0.001 << " ms");

    // publish control action //
    publishControls();

    //TODO: this should be interpolated in the event of Tnmpc ~= Tfcall //TODO: should this be before the feedback step?
    // Optional: shift the initialization (look in acado_common.h). //
    acado_shiftStates(2, 0, 0);
    acado_shiftControls(0);
  } else {
    // send "alive" status
    publishControls();  // zero control input
  }

  // publish ACADO variables //  // should this go outside?
  // publishAcadoVars();

  // publish nmpc info //
  // publishNmpcInfo(t_iter_start, t_ctrl, t_solve, t_update, t_wp_man);

  // return status //
  return (RET[0] != 0 || RET[1] != 0) ? 1 : 0;  // perhaps a better reporting method?
}


void FwNMPC::updateACADO_X0() {

  double X0[NX] = { parameter[p_index.circle_azimut],
      parameter[p_index.circle_elevation]+parameter[p_index.circle_angle],
      -0.5*M_PI, 0.0, 50.0, 0.0 };
  // internal horizon propagation:
  for (int i = 0; i < NX; ++i) {
    X0[i] = acadoVariables.x[i+NX];
  }

  /*
  paths_.ll2NE(X0[0], X0[1], (double) subs_.glob_pos.latitude,
               (double) subs_.glob_pos.longitude);        // n, e
  X0[2] = -((double) subs_.glob_pos.altitude - paths_.getHomeAlt());        // d
  X0[3] = (double) subs_.ekf_ext.airspeed;                                  // V
  X0[4] = -asin(
      (double) ((subs_.odom.twist.twist.linear.z - subs_.ekf_ext.windZ)
          / ((subs_.ekf_ext.airspeed < 1.0) ? 1.0 : subs_.ekf_ext.airspeed)));  // gamma
  X0[5] = (double) subs_.aslctrl_data.yawAngle;  // NOTE: this is actually xi, just using the message as vessel
  X0[6] = (double) subs_.aslctrl_data.rollAngle;                          // phi
  X0[7] = (double) subs_.aslctrl_data.pitchAngle;                       // theta
  X0[8] = (double) subs_.aslctrl_data.p;                                  // p
  X0[9] = (double) subs_.aslctrl_data.q;                                  // q
  X0[10] = (double) subs_.aslctrl_data.r;                                   // r
  if (FAKE_SIGNALS == 1) {
    // no shifting internal states when faking signals
    X0[11] = acadoVariables.x[11];
    X0[12] = acadoVariables.x[12];
  } else {
    // NOTE: yes.. this is shifting the state. Only for these internal states, however.--linear interpolation
    X0[11] = acadoVariables.x[11]
        + (acadoVariables.x[NX + 11] - acadoVariables.x[11]) / getLoopRate()
            / getTimeStep();  // NEXT throt state in horizon.
    X0[12] = acadoVariables.x[12]
        + (acadoVariables.x[NX + 12] - acadoVariables.x[12]) / getLoopRate()
            / getTimeStep();  // NEXT xsw state in horizon.
  }
  */
  for (int i = 0; i < NX; ++i) {
    acadoVariables.x0[i] = X0[i];
  }
}


void FwNMPC::updateACADO_OD() {
  // update online data //
  /*
  double OD[NOD];
  OD[0] = paths_.path_current.pparam1;
  OD[1] = paths_.path_current.pparam2;
  OD[2] = paths_.path_current.pparam3;
  OD[3] = paths_.path_current.pparam4;
  OD[4] = paths_.path_current.pparam5;
  OD[5] = paths_.path_current.pparam6;
  OD[6] = paths_.path_current.pparam7;
  OD[7] = paths_.path_next.pparam1;
  OD[8] = paths_.path_next.pparam2;
  OD[9] = paths_.path_next.pparam3;
  OD[10] = paths_.path_next.pparam4;
  OD[11] = paths_.path_next.pparam5;

  for (int i = 0; i < N + 1; ++i) {
    for (int j = 0; j < NOD; ++j) {
      acadoVariables.od[i * NOD + j] = OD[j];
    }
  }
  */
}


void FwNMPC::updateACADO_W() {
  // update objective gains //
  /*
  double W[NY];
  for (int i = 0; i < NY; i++)
    W[i] = (double) subs_.nmpc_params.Qdiag[i];

  // only update diagonal terms
  for (int i = 0; i < NY; ++i) {
    acadoVariables.W[i * NY + i] = W[i];  // fill diagonals
  }

  for (int i = 0; i < NYN; ++i) {
    acadoVariables.WN[i * NYN + i] = W[i];
  }
  */
}


void FwNMPC::publishControls() {
  double ctrl[NU];
  for (int i = 0; i < NU; ++i)
    ctrl[i] = acadoVariables.u[i];

  ROS_INFO_STREAM("Control_Publish: vt at 0: " << acadoVariables.x[4] << " vt at 10: " << acadoVariables.x[4+6*10]);
  ROS_INFO_STREAM("Control_Publish: rr at 0: " << ctrl[0]);

  double kkt = (double) acado_getKKT();
  double obj = (double) acado_getObjective();
  ROS_INFO("KKT: %f Obj: %f, ", kkt, obj);


  path_predicted_.header.frame_id = "world";
  path_predicted_.poses = std::vector<geometry_msgs::PoseStamped>(N);

  for (int i = 0; i < N; i++) {
    double psi = acadoVariables.x[x_index.psi+NX*i];
    double theta = acadoVariables.x[x_index.theta+NX*i];
    double r = acadoVariables.od[p_index.r+NOD*i];
    path_predicted_.poses[i].pose.position.x = r*cos(psi)*cos(theta);
    path_predicted_.poses[i].pose.position.y = r*sin(psi)*cos(theta);
    path_predicted_.poses[i].pose.position.z = r*sin(theta);
    path_predicted_.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  }
  path_pub_.publish(path_predicted_);


  /*
  double ctrl[NU];
  for (int i = 0; i < NU; ++i)
    ctrl[i] = acadoVariables.u[i];

  // saturate controls for internal model constraints //
  for (int i = 0; i < NU; ++i) {
    if (ctrl[i] < CTRL_SATURATION[i][0])
      ctrl[i] = CTRL_SATURATION[i][0];
    if (ctrl[i] > CTRL_SATURATION[i][1])
      ctrl[i] = CTRL_SATURATION[i][1];
  }

  // publish obctrl msg //
  mavros_msgs::AslObCtrl obctrl_msg;
  obctrl_msg.timestamp = t_iter_approx;
  obctrl_msg.uThrot = (isnan((float) ctrl[0])) ? 0.0f : (float) ctrl[0];
  obctrl_msg.uThrot2 = (float) track_error_lon_;  // for monitoring on QGC
  obctrl_msg.uAilR = (isnan((float) ctrl[1])) ? 0.0f : (float) ctrl[1];
  obctrl_msg.uAilL = (float) track_error_lat_;  // for monitoring on QGC
  obctrl_msg.uElev = (isnan((float) ctrl[2])) ? 0.0f : (float) ctrl[2];
  obctrl_msg.obctrl_status =
      (isnan((float) ctrl[0]) || isnan((float) ctrl[1])
          || isnan((float) ctrl[2])) ? 11 : (uint8_t) obctrl_status;  // status=11 for nan detection

  obctrl_pub_.publish(obctrl_msg);
  */

  // update last control timestamp / publish elapsed ctrl loop time //
  ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
  // t_ctrl = t_elapsed.toNSec() / 1000;  //us
  t_lastctrl = ros::Time::now();
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

  //
  // NMPC loop
  //

  double loop_rate = nmpc.getLoopRate();
  ros::Rate nmpc_rate(loop_rate);
  ROS_INFO("awe_nmpc: entering NMPC loop");
  while (ros::ok()) {

    // empty callback queues //
    ros::spinOnce();

    // nmpc iteration step //
    ret = nmpc.nmpcIteration();
    // ROS_INFO("Looping");
    if (ret != 0) {
      ROS_ERROR("nmpc_iteration: error in qpOASES QP solver.");
      // return 1;
    }

    // sleep //
    nmpc_rate.sleep();
  }



  ROS_ERROR("awe_nmpc: closing...");

  return 0;
}  // end main

