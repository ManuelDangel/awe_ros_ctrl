#include <ros/ros.h>
#include <ros/package.h>
#include <awe_nmpc.h>

namespace fw_nmpc {

FwNMPC::FwNMPC() {  // CONSTRUCTOR
  ROS_INFO("Instance of NMPC created");

  /* subscribers */
  position_sub_ = nmpc_.subscribe("/mavros/local_position/pose", 1,
                                  &FwNMPC::positionCb, this);
  velocity_sub_ = nmpc_.subscribe("/mavros/local_position/velocity", 1,
                                  &FwNMPC::velocityCb, this);
  state_sub_ = nmpc_.subscribe("/mavros/state", 1, &FwNMPC::stateCb, this);
  vfr_hud_sub_ = nmpc_.subscribe("/mavros/vfr_hud", 1, &FwNMPC::vfrHudCb, this);

  /* publishers */
  setpoint_attitude_attitude_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_attitude/attitude", 1);
  path_pub_ = nmpc_.advertise<nav_msgs::Path>("/nmpc/nmpc_predicted", 1);
  pose_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>(
      "/nmpc/aircraft_pose", 1);
  reference_pub_ = nmpc_.advertise<nav_msgs::Path>("/nmpc/reference", 1);
  state_pub_ = nmpc_.advertise<geometry_msgs::PoseStamped>("/nmpc/state", 1);
  thrust_pub_ = nmpc_.advertise<std_msgs::Float64>(
      "/mavros/setpoint_attitude/thrust", 1);

  /* Initialize NMPC Indexes */
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
  p_index.phi_freq = 9;
  p_index.wind_azimut = 10;
  p_index.thrust_power = 11;
  p_index.weight_tracking = 12;
  p_index.weight_power = 13;

  /* Initialize NMPC Parameters from launch file */
  nmpc_.param<double>("/nmpc/param_vw",
                      parameter[p_index.vw], 10.0);
  nmpc_.param<double>("/nmpc/param_r",
                      parameter[p_index.r], 220.0);
  nmpc_.param<double>("/nmpc/param_r_dot",
                      parameter[p_index.r_dot], 10.0*0.23);
  nmpc_.param<double>("/nmpc/param_circle_azimut",
                      parameter[p_index.circle_azimut], 0.0);
  nmpc_.param<double>("/nmpc/param_circle_elevation",
                      parameter[p_index.circle_elevation], 30.0 / 180 * M_PI);
  nmpc_.param<double>("/nmpc/param_circle_angle",
                      parameter[p_index.circle_angle], atan(75.0 / 220.0));
  nmpc_.param<double>("/nmpc/param_m",
                      parameter[p_index.m], 27.53);
  nmpc_.param<double>("/nmpc/param_cla",
                      parameter[p_index.cla], 0.9);
  nmpc_.param<double>("/nmpc/param_cda",
                      parameter[p_index.cda], 0.07);
  nmpc_.param<double>("/nmpc/param_phi_freq",
                      parameter[p_index.phi_freq], 2.7);
  nmpc_.param<double>("/nmpc/param_wind_azimut",
                      parameter[p_index.wind_azimut], 0.0);
  nmpc_.param<double>("/nmpc/param_thrust_power",
                      parameter[p_index.thrust_power], 0.0);
  nmpc_.param<double>("/nmpc/param_weight_tracking",
                      parameter[p_index.weight_tracking], 1.0);
  nmpc_.param<double>("/nmpc/param_weight_power",
                      parameter[p_index.weight_power], 1.0);

  // Initialize Settings from Launch File //
  nmpc_.param<double>("/nmpc/loop_rate", LOOP_RATE, 10.0);
  nmpc_.param<double>("/nmpc/time_step", TSTEP, 0.1);
  nmpc_.param<bool>("/nmpc/fake_signals", FAKE_SIGNALS, false);
  nmpc_.param<bool>("/nmpc/coordinate_flip", COORDINATE_FLIP, false);
  nmpc_.param<double>("/nmpc/angle_of_attack_deg", angle_of_attack_deg, 0.0);

  // Initialize Cost Function Parameters from Launch File
  nmpc_.param<double>("/nmpc/cost_control", cost_control, 1.0);
  nmpc_.param<double>("/nmpc/cost_tracking", cost_tracking, 100.0);
  nmpc_.param<double>("/nmpc/cost_power", cost_power, 0.0);

  // Initialize Thrust Controller
  nmpc_.param<double>("/nmpc/thrust_max", thrust_max, 0.0);
  nmpc_.param<double>("/nmpc/thrust_0_speed", thrust_0_speed, 20.0);
  nmpc_.param<double>("/nmpc/thrust_p", thrust_p, 0.1);

  // Initialize Settings
  t_lastctrl = ros::Time::now();
  loop_counter = 0;

  // Initialize State
  current_state[0] = parameter[p_index.circle_azimut];
  current_state[1] = parameter[p_index.circle_elevation]
      + parameter[p_index.circle_angle];
  current_state[2] = -0.5 * M_PI;
  current_state[3] = 0.0;
  current_state[4] = 50;
  current_state[5] = 0.0;
  current_radius = parameter[p_index.r];  // initialize radius
  airspeed = 0;

  // Initialize Resets
  reset_control_failure = false;
  reset_solution_bad = false;
  reset_no_offboard_mode = false;
}  // constructor

int FwNMPC::initNMPC() {
  // Initialize ACADO variables
  initACADOVars();
  ROS_INFO("initNMPC: ACADO variables initialized");

  // Initialize the solver.
  int RET = acado_initializeSolver();
  // Initialize all nodes with a forward sim with default (0) control input
  acado_initializeNodesByForwardSimulation();

  return RET;
}  // initNMPC

void FwNMPC::initACADOVars() {
  double X[NX] = { parameter[p_index.circle_azimut], parameter[p_index
      .circle_elevation] + parameter[p_index.circle_angle], -0.5 * M_PI, 0.0,
      50.0, 0.0 };
  double U[NU] = { 0.0, 0.0, 0.0 };
  // double OD[NOD];
  double Y[NY] = { 0.0 };  // Set all entries to 0
  double W[NY] = { cost_tracking, cost_power, cost_control, 100.0, 200.0 };
  double WN[NY] = { cost_tracking, 0.0, cost_power };

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
  memset(acadoVariables.W, 0, sizeof(acadoVariables.W));  // fill 0
  for (int i = 0; i < NY; ++i) {
    acadoVariables.W[i * NY + i] = W[i];  // fill diagonals
  }

  memset(acadoVariables.WN, 0, sizeof(acadoVariables.WN));  // fill 0
  for (int i = 0; i < NYN; ++i) {
    acadoVariables.WN[i * NYN + i] = WN[i];  // fill diagonals
  }
  // ROS_INFO_STREAM("State set to: vt: " << acadoVariables.x[4]);
  ROS_INFO_STREAM("State set to:"
      " Psi: " << X[0] << " Theta: " << X[1] << " gamma: " << X[2] <<
      " phi: " << X[3] << " vt: " << X[4] << " phi_des: " << X[5]);
}  // initACADOVars

int FwNMPC::nmpcIteration() {
  ROS_INFO_STREAM("Start NMPC Iteration with State: "
      "\n Psi: " << current_state[0] << "\n Theta: " << current_state[1] <<
      "\n gamma: " << current_state[2] << "\n psi: " << current_state[3] <<
      "\n vt: " << current_state[4] << "\n psi_des: " << current_state[5]);
  // elapsed time reusable var //
  ros::Duration t_elapsed;
  // start nmpc iteration timer --> //
  ros::Time t_iter_start = ros::Time::now();

  // initialize returns //
  int RET[2] = { 0, 0 };

  int obctrl_status = 0;

  // regular update
  // update ACADO states/references/weights //
  updateACADO_X0();
  updateACADO_W();

  // update ACADO online data //
  updateACADO_OD();

  // start nmpc solve timer --> //
  ros::Time t_solve_start = ros::Time::now();

  // Prepare first step //
  RET[0] = acado_preparationStep();
  if (RET[0] != 0) {
    ROS_ERROR("nmpcIteration: preparation step error, code %d", RET[0]);
    obctrl_status = RET[0];
  }

  // Perform the feedback step. //
  RET[1] = acado_feedbackStep();
  if (RET[1] != 0) {
    ROS_ERROR("nmpcIteration: feedback step error, code %d", RET[1]);
    obctrl_status = RET[1];
  }

  // solve time in us <-- //
  t_elapsed = ros::Time::now() - t_solve_start;

  // nmpc iteration time in us (approximate for publishing to pixhawk) <-- //
  t_elapsed = ros::Time::now() - t_iter_start;
  uint64_t t_iter_approx = t_elapsed.toNSec() / 1000;
  ROS_INFO_STREAM("NMPC Step completed took: " << t_iter_approx*0.001 << " ms");

  // publish control action //
  publishControls();

  // Shift the initialization (look in acado_common.h). //
  acado_shiftStates(2, 0, 0);
  acado_shiftControls(0);

  publishReference();

  // publish nmpc info //
  // publishNmpcInfo(t_iter_start, t_ctrl, t_solve, t_update, t_wp_man);

  // return status //
  return (RET[0] != 0 || RET[1] != 0) ? 1 : 0;
}  // nmpcIteration

void FwNMPC::updateACADO_X0() {
  if (FAKE_SIGNALS) {  // If we "simulate" the state is set to MPC belief
    for (int i = 0; i < NX; ++i) {  // internal horizon propagation:
      current_state[i] = acadoVariables.x[i + NX];
    }
  }

  // internal horizon propagation of roll states
  current_state[x_index.phi] = acadoVariables.x[x_index.phi + NX];
  current_state[x_index.phi_des] = acadoVariables.x[x_index.phi_des + NX];

  // catch and safe controller failure NaN's and extreme high KKT
  reset_control_failure = isnan(current_state[x_index.phi_des]);
  reset_solution_bad = (acado_getKKT() > 1000000.0);
  if (reset_control_failure || reset_solution_bad || reset_no_offboard_mode) {
    // catch controller failure
    if (reset_control_failure) {
      ROS_ERROR("Controller Failure, NaN's in Solution!");
    } else if (reset_solution_bad) {
      ROS_ERROR("Controller Failure, KKT too high!");
    } else {
      ROS_WARN("Reinitializing Controller "
          "because Offboard control mode is not switched on");
      // This is to try to get out of local minimas in the solution
      // Typically the Horizon used to get stuck in a solution running
      // the opposite direction of the circle
    }
    reset_no_offboard_mode = false;
    ROS_WARN("NMPC restarting...");

    current_state[x_index.phi_des] = 0;
    current_state[x_index.phi] = 0;
    // controls
    double U[NU] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < N; ++i) {  // set all control back to 0
      for (int j = 0; j < NU; ++j)
        acadoVariables.u[i * NU + j] = U[j];
    }
    for (int i = 0; i < N + 1; ++i) {  // set states to ok state
      for (int j = 0; j < NX; ++j)
        acadoVariables.x[i * NX + j] = current_state[j];
    }
  }

  for (int i = 0; i < NX; ++i) {  // set State
    acadoVariables.x0[i] = current_state[i];  // X0[i];
  }

  if (reset_control_failure) {  // restart after controller failure
    int RET = acado_initializeSolver();
    acado_initializeNodesByForwardSimulation();
  }
}  // updateACADO_X0


void FwNMPC::updateACADO_OD() {
  // update online data
  for (int i = 0; i < N + 1; ++i) {
    for (int j = 0; j < NOD; ++j)
      if (j == p_index.r) {
        acadoVariables.od[i * NOD + j] =
            current_radius+parameter[p_index.r_dot]*j*TSTEP;
      } else if (j == p_index.circle_angle) {  // sqrt cone shaped reference
        acadoVariables.od[i * NOD + j] = parameter[j] *
            sqrt(parameter[p_index.r]/acadoVariables.od[i*NOD+p_index.r]);
      } else if (j == p_index.thrust_power) {
        // Thrust calculation!
        // Set Thrust full at vt<=10 and none at vt>=20
        acadoVariables.od[i * NOD + j] =
            parameter[j]* std::min(thrust_max, std::max(0.0,  // saturation
            (thrust_0_speed-acadoVariables.x[i * NX + x_index.vt])*thrust_p));
      } else {
        acadoVariables.od[i * NOD + j] = parameter[j];
      }
  }
}  // updateACADO_OD


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
}  // updateACADO_W


void FwNMPC::publishControls() {
  double ctrl[NU];
  for (int i = 0; i < NU; ++i)
    ctrl[i] = acadoVariables.u[i];

  ROS_INFO_STREAM("Control_Publish: vt at 0: " << acadoVariables.x[4] <<
                  " vt at 10: " << acadoVariables.x[4+6*10]);
  ROS_INFO_STREAM("Control_Publish: rr at 0: " << ctrl[0]);

  double kkt = static_cast<double>(acado_getKKT());
  double obj = static_cast<double>(acado_getObjective());
  ROS_INFO("KKT: %f Obj: %f, ", kkt, obj);

  path_predicted_.header.frame_id = "world";
  path_predicted_.poses = std::vector<geometry_msgs::PoseStamped>(N + 1);
  // using the first one to draw a line to visualize the Tether
  tf::quaternionTFToMsg(tf::Quaternion(0.0, 0.0, 0.0, 1.0),
                        path_predicted_.poses[0].pose.orientation);

  tf::Matrix3x3 attitude_local;
  attitude_local.setEulerYPR(0.0, angle_of_attack_deg/180*M_PI, 0.0);
  tf::Quaternion attitude_quat_enu;
  attitude_local.getRotation(attitude_quat_enu);
  tf::quaternionTFToMsg(attitude_quat_enu,
                        path_predicted_.poses[0].pose.orientation);

  for (int i = 0; i < N; i++) {
    // Calculate Planned Aircraft Positions
    const double & psi = acadoVariables.x[x_index.psi+NX*i];
    const double & theta = acadoVariables.x[x_index.theta+NX*i];
    const double & r = acadoVariables.od[p_index.r+NOD*i];
    if (COORDINATE_FLIP) {
      path_predicted_.poses[i+1].pose.position.x = r*cos(psi)*cos(theta);
      path_predicted_.poses[i+1].pose.position.y = -r*sin(theta);
      path_predicted_.poses[i+1].pose.position.z = r*sin(psi)*cos(theta);
    } else {
      path_predicted_.poses[i+1].pose.position.x = r*cos(psi)*cos(theta);
      path_predicted_.poses[i+1].pose.position.y = r*sin(psi)*cos(theta);
      path_predicted_.poses[i+1].pose.position.z = r*sin(theta);
    }
    // Calculate Planned Aircraft Attitudes
    const double & gamma = acadoVariables.x[x_index.gamma+NX*i];
    const double & vt = acadoVariables.x[x_index.vt+NX*i];
    const double & phi = acadoVariables.x[x_index.phi+NX*i];
    const double & r_dot = acadoVariables.od[p_index.r_dot+NOD*i];
    tf::Matrix3x3 enu2local_rot(  // Transformation Matrix
          -sin(theta)*cos(psi)*cos(gamma)-sin(psi)*sin(gamma),
           sin(theta)*cos(psi)*sin(gamma)-sin(psi)*cos(gamma),
          -cos(theta)*cos(psi),
          -sin(theta)*sin(psi)*cos(gamma)+cos(psi)*sin(gamma),
           sin(theta)*sin(psi)*sin(gamma)+cos(psi)*cos(gamma),
          -cos(theta)*sin(psi),
           cos(theta)*cos(gamma),
          -cos(theta)*sin(gamma),
          -sin(theta));
    tf::Vector3 wind_enu(
        acadoVariables.od[p_index.vw + NOD * i]
            * cos(acadoVariables.od[p_index.wind_azimut + NOD * i]),
        acadoVariables.od[p_index.vw + NOD * i]
            * sin(acadoVariables.od[p_index.wind_azimut + NOD * i]),
        0.0);
    tf::Vector3 wind_local;
    wind_local = enu2local_rot.transpose() * wind_enu;  // Tf wind to local
    tf::Vector3 groundspeed_local(vt, 0, -r_dot);
    tf::Vector3 airspeed_local;
    airspeed_local = groundspeed_local - wind_local;
    double airspeed_abs = sqrt(pow(airspeed_local.x(), 2)+
                               pow(airspeed_local.y(), 2)+
                               pow(airspeed_local.z(), 2));
    tf::Matrix3x3 attitude_local;
    attitude_local.setEulerYPR(atan(airspeed_local.y()/vt),
                               -asin(airspeed_local.z()/airspeed_abs
                                     - angle_of_attack_deg/180.0*M_PI),
                               phi+M_PI);
    tf::Matrix3x3 attitude_enu;
    if (COORDINATE_FLIP) {
      tf::Matrix3x3 flip_rot(1.0, 0.0, 0.0,
                            0.0, 0.0, -1.0,
                            0.0, 1.0, 0.0);
      attitude_enu = flip_rot * enu2local_rot * attitude_local;
    } else {
      attitude_enu = enu2local_rot * attitude_local;
    }
    tf::Quaternion attitude_quat_enu;
    attitude_enu.getRotation(attitude_quat_enu);
    tf::quaternionTFToMsg(attitude_quat_enu,
                          path_predicted_.poses[i+1].pose.orientation);
    if (i == 1) {  // Publish setpoint for low level control
      // Publish also in ENU
      setpoint_attitude_attitude_.header.frame_id = "world";
      setpoint_attitude_attitude_.pose.position.x =
          path_predicted_.poses[i+1].pose.position.x;
      setpoint_attitude_attitude_.pose.position.y =
          path_predicted_.poses[i+1].pose.position.y;
      setpoint_attitude_attitude_.pose.position.z =
          path_predicted_.poses[i+1].pose.position.z;
      tf::quaternionTFToMsg(attitude_quat_enu,
                            setpoint_attitude_attitude_.pose.orientation);
    }
  }
  path_pub_.publish(path_predicted_);  // Pubish NMPC Results
  setpoint_attitude_attitude_pub_.publish(setpoint_attitude_attitude_);

  // publish state for logging
  state_.header.frame_id = "world";
  state_.pose.position.x = current_state[x_index.psi];
  state_.pose.position.y = current_state[x_index.theta];
  state_.pose.position.z = current_radius;
  state_.pose.orientation.x = current_state[x_index.gamma];
  state_.pose.orientation.y = current_state[x_index.phi];
  state_.pose.orientation.z = current_state[x_index.vt];
  state_.pose.orientation.w = current_state[x_index.phi_des];
  state_pub_.publish(state_);  // Pubish state

  // update last control timestamp / publish elapsed ctrl loop time //
  ros::Duration t_elapsed = ros::Time::now() - t_lastctrl;
  t_lastctrl = ros::Time::now();
}  // publishControls

void FwNMPC::publishReference() {
  reference_.header.frame_id = "world";
  reference_.poses = std::vector<geometry_msgs::PoseStamped>(50);
  double psi, theta;
  for (int i=0; i < 50; ++i) {
    theta = parameter[p_index.circle_elevation] +
        parameter[p_index.circle_angle] *
        sqrt(parameter[p_index.r]/current_radius)*cos((0.02+0.04*i)*M_PI);
    psi =  acos((cos(parameter[p_index.circle_angle] *
                     sqrt(parameter[p_index.r]/current_radius))
        -sin(parameter[p_index.circle_elevation])*sin(theta))
                /(cos(parameter[p_index.circle_elevation])*cos(theta)));
    if (i < 25) {
      psi = psi + parameter[p_index.circle_azimut];
    } else {
      psi = -psi + parameter[p_index.circle_azimut];
    }
    if (COORDINATE_FLIP) {
      reference_.poses[i].pose.position.x = current_radius*cos(psi)*cos(theta);
      reference_.poses[i].pose.position.y = -current_radius*sin(theta);
      reference_.poses[i].pose.position.z = current_radius*sin(psi)*cos(theta);
    } else {
      reference_.poses[i].pose.position.x = current_radius*cos(psi)*cos(theta);
      reference_.poses[i].pose.position.y = current_radius*sin(psi)*cos(theta);
      reference_.poses[i].pose.position.z = current_radius*sin(theta);
    }
  }
  reference_pub_.publish(reference_);  // Publish NMPC Reference
}  // publishReference

void FwNMPC::publishThrust() {  // Thrust Controller
  // This function is called directly from the VFR_HUD (airspeed) Callback
  thrust_.data = std::min(thrust_max, std::max(0.0,  // control saturation
      (thrust_0_speed-airspeed)*thrust_p));  // Airspeed P Controller
  thrust_pub_.publish(thrust_);
}  // publishThrust

void FwNMPC::positionCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // MAVROS OUTPUTS ENU
  const double & x = msg->pose.position.x;
  const double & y = msg->pose.position.y;
  const double & z = msg->pose.position.z;
  double r = sqrt(x*x+y*y+z*z);
  double psi, theta;
  if (COORDINATE_FLIP) {
    psi = atan2(z, x);   // set Azimut Angle
    theta = asin(-y/r);   // set Elevation Angle
  } else {
    psi = atan2(y, x);   // set Azimut Angle
    theta = asin(z/r);   // set Elevation Angle
  }

  // Wrapping of Horizon for psi
  if (psi > current_state[x_index.psi] + M_PI) {
    for (int i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + x_index.psi] += 2*M_PI;
    }
  } else if (psi < current_state[x_index.psi] - M_PI) {
    for (int i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + x_index.psi] -= 2*M_PI;
    }
  }
  current_state[x_index.psi] = psi;
  current_state[x_index.theta] = theta;
  current_radius = r;

  // Publish current Pose in enu
  aircraft_pose_.header.frame_id = "world";
  aircraft_pose_.pose = msg->pose;
  pose_pub_.publish(aircraft_pose_);  // Publish current aircraft pose in enu
}  // positionCb

void FwNMPC::velocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  const double & vx = msg->twist.linear.x;
  const double & vy = msg->twist.linear.y;
  const double & vz = msg->twist.linear.z;
  tf::Vector3 velocity_enu(vx, vy, vz);
  if (COORDINATE_FLIP) {  // flip coordinate frame
    velocity_enu.setY(vz);
    velocity_enu.setZ(-vy);
  }
  const double & psi = current_state[x_index.psi];
  const double & theta = current_state[x_index.theta];
  tf::Matrix3x3 enu2sphere_rot(
      -sin(theta)*cos(psi), -sin(psi), -cos(theta)*cos(psi),
      -sin(theta)*cos(psi), cos(psi), -cos(theta)*sin(psi),
      cos(theta), 0.0, -sin(theta));
  tf::Vector3 velocity_sphere;
  velocity_sphere = enu2sphere_rot.transpose() * velocity_enu;

  // Calculate Heading Angle
  double gamma = atan2(velocity_sphere.y(), velocity_sphere.x());
  // Wrapping of Horizon
  if (gamma > current_state[x_index.gamma] + M_PI) {
    for (int i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + x_index.gamma] += 2*M_PI;
    }
  } else if (gamma < current_state[x_index.gamma] - M_PI) {
    for (int i = 0; i < N + 1; ++i) {
      acadoVariables.x[i * NX + x_index.gamma] -= 2*M_PI;
    }
  }
  current_state[x_index.gamma] = gamma;  // set Heading Angle

  double vt = std::sqrt(std::pow(velocity_sphere.x(), 2)+
                        std::pow(velocity_sphere.y(), 2));
  ROS_INFO_STREAM("Set Velocity State: vx " <<  vx << " vy " << vy <<
                  " vx_sphere " << velocity_sphere.x() <<
                  " vy_sphere " << velocity_sphere.y());
  // set sphere velocity with lower bound to prevent infeasibility in solver
  current_state[x_index.vt] = std::max(vt, 5.0);
}  // velocityCb

void FwNMPC::stateCb(const mavros_msgs::State::ConstPtr& msg) {
  // this message comes at 1 Hz and resets the NMPC if not in OFFBOARD mode
  ROS_INFO_STREAM("Received Pixhawk Mode: " <<  msg->mode);
  if (msg->mode == "OFFBOARD") {
    reset_no_offboard_mode = false;
  } else {
    if (static_cast<double>(acado_getObjective()) > 100.0) {
      // Reset only if we are not already tracking the circle well
      reset_no_offboard_mode = true;
    }
  }
}  // stateCb

void FwNMPC::vfrHudCb(const mavros_msgs::VFR_HUD::ConstPtr& msg) {
  airspeed = msg->airspeed;
  publishThrust();
}  // vfrHudCb

double FwNMPC::getLoopRate() {
  return LOOP_RATE;
}  // getLoopRate

double FwNMPC::getTimeStep() {
  return TSTEP;
}  // getTimeStep

void FwNMPC::shutdown() {
  ROS_INFO("Shutting down NMPC...");
  ros::shutdown();
}  // shutdown

};  // namespace fw_nmpc


int main(int argc, char **argv) {
  // initialize node //
  ros::init(argc, argv, "awe_nmpc");
  fw_nmpc::FwNMPC nmpc;
  ros::spinOnce();

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
    ++nmpc.loop_counter;

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
}  // main

