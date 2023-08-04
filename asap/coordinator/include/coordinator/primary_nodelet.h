#pragma once
#include "coordinator/coordinator.tpp"
#include "coordinator/controller1.h"
/* //mathlab code generation
#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include "controller1.h"               // Model's header file
#include "rtwtypes.h" */
// status struct for primary Astrobee
struct primary_status_struct {
  std::string control_mode = "inactive";
  std::string description = "";  // post-processing and telemetry helpers
};


class PrimaryNodelet : public CoordinatorBase<coordinator::StatusPrimary>, public ff_util::FreeFlyerNodelet{
 public:
  PrimaryNodelet(): ff_util::FreeFlyerNodelet(true) {}  // don't do anything ROS-related in the constructor! (call the Nodelet constructor)
  ~PrimaryNodelet() {};

 private:
  //static controller1ModelClass controller1_Obj;// Instance of model class
  primary_status_struct primary_status_;
  std::string CONTROL_MODE_TOPIC = "reswarm/primary/control_mode";
  std::string VIRTUAL_LEADER_TOPIC = "virtual_Leader";

  ros::NodeHandle *nh_;

  // Publishers and subscriber declarations go here!
  ros::Subscriber sub_status_;
  ros::Subscriber sub_control_mode_;

  ros::Rate sleep_rate{10.0};
  float reg_time_;  // how long to regulate
  int ACTIVATE_DEFAULT_REGULATION_ = 0;  // whether regulation is allowed

  // Parameters
  bool ground_ = false;  // whether or not this is a ground test
  bool sim_ = false;
  std::string controller_ = "default";  // controller to send commands to
  std::string flight_mode_check_;

  //Eigen::Vector3d x0_;
 // Eigen::Vector4d a0_;

  Eigen::Matrix<double, 7, 1> POINT_A_GRANITE;
  Eigen::Matrix<double, 7, 1> POINT_A_ISS;
  Eigen::Matrix<double, 7, 1> POINT_B_GRANITE;
  Eigen::Matrix<double, 7, 1> POINT_B_ISS;
  Eigen::Matrix<double, 7, 1> POINT_C_GRANITE;
  Eigen::Matrix<double, 7, 1> POINT_C_ISS;

  // Regulation thresholds
  float pos_reg_thresh_;
  float vel_reg_thresh_;
  float att_reg_thresh_;
  float omega_reg_thresh_;

  void get_status_msg(coordinator::StatusPrimary& msg) override;

  void Initialize(ros::NodeHandle* nh);
  void load_params();
  void Run(ros::NodeHandle *nh);
  //void PrimaryNodelet::test_num_callback(Int32 test_number);
  // quick checkout
  void RunTest0(ros::NodeHandle *nh) override;  // quick checkout

  // Test list (overrides coordinator empty tests---add as desired!)
  void RunTest1(ros::NodeHandle *nh) override;  // a very simple test
   
  // Isuru Basnayake
 /*  void RunTest2(ros::NodeHandle *nh) override;  

  // Isuru Basnayake Jan 5 23
  void RunTest3(ros::NodeHandle *nh) override; 

  // Isuru Basnayake Jan 5 23
  void RunTest4(ros::NodeHandle *nh) override;

  // Isuru Basnayake Jan 5 23
  void RunTest5(ros::NodeHandle *nh) override;

  // Isuru Basnayake Jan 5 23
  void RunTest6(ros::NodeHandle *nh) override; */

  void check_regulate();
  void publish_dummy_uc_bound();

  void set_info_test_regulate_params();
  void process_rattle_test_number(int test_number);

  void pub_reg_setpoint(Eigen::MatrixXd reg_setpoint_);
  void send_traj_to_controller(Eigen::MatrixXd eigen_x_des_traj);
  Eigen::Matrix3f q2dcm(const Eigen::Vector4f &q);

  // status callbacks
  void control_mode_callback(const std_msgs::String::ConstPtr msg);  // externally update the control mode
  void gnc_ctl_setpoint_callback(const ros::TimerEvent& );
};

PLUGINLIB_EXPORT_CLASS(PrimaryNodelet, nodelet::Nodelet); // this is a nodelet!
