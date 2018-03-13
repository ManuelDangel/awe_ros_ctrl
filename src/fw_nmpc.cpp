#include <ros/ros.h>
#include <ros/package.h>

#include <fw_nmpc.h>

#include <math.h>
#include <string.h>

#include <std_msgs/UInt8.h>

// INCLUDES for mavros
#include <mavros_msgs/WaypointPull.h>

// INCLUDES for awe_ctrl
// #include <fw_ctrl/AcadoVars.h>
// #include <fw_ctrl/NmpcInfo.h>

namespace fw_nmpc {

FwNMPC::FwNMPC() :
    LOOP_RATE(20.0),  // MAGIC NUMBER
    TSTEP(0.1),  // MAGIC NUMBER
    FAKE_SIGNALS(0),
    t_lastctrl{0},
    bModeChanged(false),
    last_ctrl_mode(0),
    obctrl_en_(0),
    last_wp_idx_(-1),
    bYawReceived(false),
    last_yaw_msg_(0.0f),
    track_error_lat_(0.0f),
    track_error_lon_(0.0f),
    T_b_lat_(1.0),
    W_scale_{0}
{
  ROS_INFO("Instance of NMPC created");
}




int main(int argc, char **argv) {
  /* initialize node */
  ros::init(argc, argv, "fw_nmpc");
  fw_nmpc::FwNMPC nmpc;


  return 0;
}

}  // close namespace fw_nmpc
