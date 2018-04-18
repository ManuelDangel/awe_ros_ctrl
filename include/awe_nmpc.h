#ifndef _FW_NMPC_H
#define _FW_NMPC_H

// INCLUDES for ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <tf/tf.h>

#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

// general includes
#include "math.h"

// INCLUDES for ACADO
#include "acado_common.h"
#include "acado_auxiliary_functions.h"

/* some convenient definitions */
#define NX  ACADO_NX  /* Number of differential state variables.  */
#define NU  ACADO_NU  /* Number of control inputs. */
#define NOD ACADO_NOD /* Number of online data values. */
#define NY  ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN ACADO_NYN /* Number of measurements/references on node N. */
#define N   ACADO_N     /* Number of intervals in the horizon. */
// #define NX_AUGM 2     /* Number of augmented differential state variables. */

/* global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace fw_nmpc {

struct Xindex {  // Lists the indexes of the different states
  int psi, theta, gamma, phi, vt, phi_des;
};

struct Uindex {  // Lists the indexes of the different control inputs
  int dphi, phi_slack, theta_slack;
};

struct Pindex {  // Lists the indexes of the different online parameters
  int vw, r, r_dot, circle_azimut, circle_elevation, circle_angle, m,
  cla, cda, phi_freq, wind_azimut, weight_tracking, weight_power;
};


/*
 * @brief fw_nmpc class
 *
 * class that implements fixed-wing nmpc
 */
class FwNMPC {

 public:

  FwNMPC();

  /* callbacks */
  void positionCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void velocityCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
  // void aslctrlDataCb(const mavros_msgs::AslCtrlData::ConstPtr& msg);
  //void globPosCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
  //void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  // void ekfExtCb(const mavros_msgs::AslEkfExt::ConstPtr& msg);
  // void nmpcParamsCb(const mavros_msgs::AslNmpcParams::ConstPtr& msg);
  //void waypointListCb(const mavros_msgs::WaypointList::ConstPtr& msg);
  //void currentWpCb(const std_msgs::Int32::ConstPtr& msg);
  //void homeWpCb(const mavros_msgs::HomePosition::ConstPtr& msg);
  // void aslctrlDebugCb(const mavros_msgs::AslCtrlDebug::ConstPtr& msg);
  /* initializations */
  int initNMPC();
  void initACADOVars();
  //void initHorizon();
  /* sets */
  void updateACADO_X0();
  void updateACADO_OD();
  // void updateACADO_Y();  // Not necessary for AWE_NMPC
  void updateACADO_W();
  /* gets */
  double getLoopRate();
  double getTimeStep();
  /* functions */
  //void update();
  int nmpcIteration();
  /* publishing encapsulation */
  void publishControls();
  void publishReference();

  double LOOP_RATE;
  double TSTEP;
  int FAKE_SIGNALS;
  int loop_counter;

 private:
  /* node handles */
  ros::NodeHandle nmpc_;

  // NMPC Indexes
  Xindex x_index;
  Uindex u_index;
  Pindex p_index;
  // NMPC Parameter
  double parameter[NOD];
  // state
  double current_state[NX];
  double current_radius;

  /* subscribers */
  ros::Subscriber position_sub_;
  ros::Subscriber velocity_sub_;
  /* publishers */
  ros::Publisher setpoint_attitude_attitude_pub_;  // control
  geometry_msgs::PoseStamped setpoint_attitude_attitude_;
  ros::Publisher path_pub_;  // for visualization
  nav_msgs::Path path_predicted_;
  ros::Publisher pose_pub_;  // for visualization
  geometry_msgs::PoseStamped aircraft_pose_;
  ros::Publisher reference_pub_;  // for visualization
  nav_msgs::Path reference_;
  // ros::Publisher obctrl_pub_;
  // ros::Publisher nmpc_info_pub_;
  // ros::Publisher acado_vars_pub_;
  /* time keeping */
  ros::Time t_lastctrl;

  /* controller switch */
  bool bModeChanged;
  int last_ctrl_mode;
  int obctrl_en_;

  /* continuity */
  bool bYawReceived;
  float last_yaw_msg_;

  /* node functions */
  void shutdown();

};

}  // namespace fw_nmpc

#endif
