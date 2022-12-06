#pragma once

/*
# coordinator

High-level logic coordinating test operation.

Every test has a test#() function available in case it is needed by asap.py
*/

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include "eigen_conversions/eigen_msg.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>

// msg includes
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <msg_conversions/msg_conversions.h>
#include <ff_msgs/FamCommand.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/Wrench.h>
#include <coordinator/StatusPrimary.h>
#include <coordinator/StatusSecondary.h>
#include <coordinator/TestNumber.h>
//#include <std_msgs/Int32.h>
// Actions
#include <ff_msgs/ControlAction.h>

// Service message
#include <std_srvs/SetBool.h>

// C++
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <chrono>
#include <string.h>
#include <sstream>
#include <math.h>

//mathlab code generation
// Include Files
//#include "MPC_Guidance_v3_sand.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>





static std::string TOPIC_ASAP_STATUS = "asap/status";
static std::string TOPIC_ASAP_TEST_NUMBER = "asap/test_number";
static std::string TOPIC_GNC_CTL_CMD = "gnc/ctl/command";




// base status struct (key information)
struct BaseStatus {
  int test_number = -2;
  std::string flight_mode = "nominal";
  bool test_finished = false;

  bool coord_ok = true;
  bool regulate_finished = false;

  bool default_control = true;  // {true, false}: allow the default controller to run?
};


template <typename T>  // for PrimaryStatus or SecondaryStatus 
class CoordinatorBase 
{
 public:
  CoordinatorBase() {}; // don't do anything ROS-related in the constructor!
  ~CoordinatorBase() {};
  
  // Base main functions
  void Run(ros::NodeHandle *nh);
 protected:
  BaseStatus base_status_;

  ros::NodeHandle MTNH;
  std::shared_ptr<std::thread> thread_;

  ros::Publisher pub_flight_mode_;
  ros::Publisher pub_status_;
  ros::Publisher pub_ctl_;

  ros::Subscriber sub_flight_mode_;
  ros::Subscriber sub_ekf_;
  ros::Subscriber sub_test_number_;

  ros::ServiceClient serv_ctl_enable_;

  ros::Timer status_timer_;
  ros::Timer ctl_disable_timer_;

  ff_msgs::FlightMode flight_mode_;
  ff_msgs::EkfState ekf_state_;
  ff_msgs::FamCommand gnc_setpoint;

  geometry_msgs::Wrench ctl_input;
  geometry_msgs::Quaternion attitude;
  geometry_msgs::Vector3 omega,velocity_, position_, position_error, position_ref;
  tf2::Quaternion attitude_,q_ref,q_e,q_ref_inv;

  // Parameters
  bool ground_ = false;  // whether or not this is a ground test
  bool sim_ = false;
  std::string flight_mode_name_;

  // Stored status parameters
  std::string stored_control_mode_ = "track";  // stored control_mode, set by parameter inputs

  // Ekf state
  Eigen::Matrix<double, 16, 1> x_real_complete_;

  // Test number processing
  void process_test_number();
  void get_flight_mode();
  
  void publish_status(const ros::TimerEvent&);  // templated for Primary or Secondary status
  virtual void get_status_msg(coordinator::StatusPrimary& msg) {};
  virtual void get_status_msg(coordinator::StatusSecondary& msg) {};

  void test_num_callback(const coordinator::TestNumber::ConstPtr msg);
  void flight_mode_callback(const ff_msgs::FlightMode::ConstPtr msg);
  void ekf_callback(const ff_msgs::EkfState::ConstPtr msg);

  void debug();

  // Astrobee GNC interface
  void disable_default_ctl_callback(const ros::TimerEvent&);
  void disable_default_ctl();
  void enable_default_ctl();
  

  // Virtual test list: to be replaced on each derived coordinator
  virtual void RunTest0(ros::NodeHandle *nh) {};
  virtual void RunTest1(ros::NodeHandle *nh) {};
  virtual void RunTest2(ros::NodeHandle *nh) {};


  //controller1ModelClass controller1_Obj;// Instance of model class

// '<Root>/x_e'
float arg_x_e = 0.0;

// '<Root>/y_e'
 float arg_y_e = 0.0;

// '<Root>/z_e'
 float arg_z_e = 0.0;

// '<Root>/vx'
 float arg_vx = 0.0;

// '<Root>/vy'
 float arg_vy = 0.0;

// '<Root>/vz'
 float arg_vz = 0.0;

// '<Root>/qx'
 float arg_qx = 0.0;

// '<Root>/qy'
 float arg_qy = 0.0;

// '<Root>/qz'
 float arg_qz = 0.0;

// '<Root>/qw'
 float arg_qw = 0.0;

// '<Root>/omegax'
 float arg_omegax = 0.0;

// '<Root>/omegay'
 float arg_omegay = 0.0;

// '<Root>/omegaz'
 float arg_omegaz = 0.0;

// '<Root>/fx'
 float arg_fx;

// '<Root>/fy'
 float arg_fy;

// '<Root>/fz'
 float arg_fz;

// '<Root>/tau_x'
 float arg_tau_x;

// '<Root>/tau_y'
 float arg_tau_y;

// '<Root>/tau_z'
 float arg_tau_z;

double x0[6];
double Fx;
double Fy;
double Fz;
double target_state[6];
double dock_flag;
double CollAvoid_flag;
double dock_complete;
double num_iter;
double X_QP[60]={0};
double pt_sel;
double dr;

double z_nominal[6];
double zp_nextNominal[6];
double v_mpc[3];
bool initial_run = true;
bool initialzation = false;

double kn_tilda[3];
double kN[3];
bool rotation_done = false;
void step_PID();

// Function Declarations
void main_MPC_Guidance_v3_sand();
void MPC_Guidance_v3_sand();
void mldivide(double A[3600], double B[60]);
bool rtIsNaN(double value);
void nominal_dynamics();
void tubing_mpc();
  // add test definitions as necessary here
  // you can add more tests as desired in primary.h and secondary.h
};


/* ************************************************************************** */
// Coordinator template implementation
/* ************************************************************************** */

/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::Run(ros::NodeHandle *nh) {
  /**
   * @brief Interpret the `/test_number` topic and run tests. High-level test management
   * takes place here, and is delegated out to other nodes. 
   * Each test function is intended to be run just ONCE per test number received.
   * This is the place to add new test numbers!
   * 
   */
  x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;

  // Status publisher in separate thread
  status_timer_ = MTNH.createTimer(ros::Duration(0.2),
    boost::bind(&CoordinatorBase::publish_status, this, _1));  // send commands (5 Hz)

  // Controller disabler in separate thread
  // ctl_disable_timer_ = MTNH.createTimer(ros::Duration(5.0),
  //   boost::bind(&CoordinatorBase::disable_default_ctl_callback, this, _1));  // send disable commands (0.2 Hz)

  // Check updates at 10Hz
  ros::Rate sleep_rate(10.0);

  // (1) Start up and wait for test_number_
  while (ros::ok() && base_status_.test_number == -2) {  // startup test number
    ros::spinOnce();
    sleep_rate.sleep();
  }

  // (2) Publish default flight mode so FAM will actually perform actuation
  get_flight_mode();
  pub_flight_mode_.publish(flight_mode_);  // TODO: should this be a more aggressive flight mode?
  ros::Duration(2.0).sleep();  // Pause so flight mode actually gets registered

  // (3) Execute test_number_ logic
  while (ros::ok()) { 
    if (!base_status_.test_finished) {
      // Tests go below...
      if (base_status_.test_number == 0) {
        RunTest0(nh);
      }
      else if (base_status_.test_number  == 1) {
        RunTest1(nh);
      }
      else if(base_status_.test_number  == 2) {
        RunTest2(nh);
      }
      // add additional test checks here

      base_status_.test_finished = true;
    }
    ros::spinOnce();
    sleep_rate.sleep();
  }
}


/* ************************************************************************** */
template <typename T>
void CoordinatorBase<T>::get_flight_mode() {
  /* Get a nominal flight mode for desired test.
  */
  if (base_status_.test_number != -1 ) {  // NOT shutdown test number or checkout test {and base_status_.test_number != 0}
    // create a nominal FlightMode
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, base_status_.flight_mode)) {
      return;
    } 
  }
  else { // -1 shutdown test number
    // Shutdown Astrobee (turn off impellers)
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, "off")) {
      return;
    }  
  }
}


/* ************************************************************************** */
template <typename T>
void CoordinatorBase<T>::publish_status(const ros::TimerEvent&) {
  /**
   * @brief Main coordinator of Base logic. Relies on get_status_msg, defined in derived class.
   * Uses either Primary or Secondary status logic.
   * 
   */
  T msg;
  get_status_msg(msg);
  pub_status_.publish(msg);
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::process_test_number() {
  /**
   * @brief Process test number logic for parameters
   * An example is provided here for processing individual digits of a test greater than 100.
   * 
   */

  if (base_status_.test_number > 100) {
    std::string test_number_str = std::to_string(base_status_.test_number);

    // Parameter settings xx(xxxxxx)
    // controller
    if (test_number_str[2] == '2') {  // standard MPC
      stored_control_mode_ = "track";
    }
    else if (test_number_str[2] == '3') {  // tube MPC
      stored_control_mode_ = "track_tube";
    }
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::test_num_callback(const coordinator::TestNumber::ConstPtr msg) {
  /**
   * @brief Updates test numbers received from exec_asap
   * 
   */
  base_status_.test_number = msg->test_number;
  if (base_status_.test_number == -1) {
    // Re-enable default controller
    enable_default_ctl();

    // Set flight mode to off
    base_status_.flight_mode = "off";
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, base_status_.flight_mode)) {
        return;
    }
    pub_flight_mode_.publish(flight_mode_);
  }
  // ROS_INFO("It is working bro dont be sad ;) ");
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::flight_mode_callback(const ff_msgs::FlightMode::ConstPtr msg) {
  /**
   * @brief Callback for flight mode.
   * 
   */
  flight_mode_name_ = msg->name;

  // kill ctl if it tries to turn on
  if (base_status_.default_control == false){
    auto serv_start = std::chrono::high_resolution_clock::now();

    // Disable the default controller so custom controller can run
    std_srvs::SetBool srv;
    srv.request.data = false;
    serv_ctl_enable_.call(srv);

    auto serv_finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> serv_elapsed = serv_finish - serv_start;
    std::string response = srv.response.message;

    std::cout << "[COORDINATOR]: Controller disable service time: " << serv_elapsed.count() << " seconds."<< std::endl;
    std::cout << "[COORDINATOR]: Controller disable service result: " << response << std::endl;
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::ekf_callback(const ff_msgs::EkfState::ConstPtr msg) {
  /**
   * @brief The `gnc/ekf` subscriber callback. Called at 62.5 Hz.
   * Used to check if regulation is finished.
   * 
   */
  float qx = msg->pose.orientation.x;
  float qy = msg->pose.orientation.y;
  float qz = msg->pose.orientation.z;
  float qw = msg->pose.orientation.w;

  float px = msg->pose.position.x;
  float py = msg->pose.position.y;
  float pz = msg->pose.position.z;

  float vx = msg->velocity.x;
  float vy = msg->velocity.y;
  float vz = msg->velocity.z;


  float wx = msg->omega.x;
  float wy = msg->omega.y;
  float wz = msg->omega.z;

  if (qx != 0 || qy != 0 || qz != 0 || qw != 0) {
    x_real_complete_(0) = msg->pose.position.x;
    x_real_complete_(1) = msg->pose.position.y;
    x_real_complete_(2) = msg->pose.position.z;
    x_real_complete_(3) = msg->pose.orientation.x;
    x_real_complete_(4) = msg->pose.orientation.y;
    x_real_complete_(5) = msg->pose.orientation.z;
    x_real_complete_(6) = msg->pose.orientation.w;
    x_real_complete_(7) = msg->velocity.x;
    x_real_complete_(8) = msg->velocity.y;
    x_real_complete_(9) = msg->velocity.z;
    x_real_complete_(10) = msg->omega.x;
    x_real_complete_(11) = msg->omega.y;
    x_real_complete_(12) = msg->omega.z;
    x_real_complete_(13) = 0.0;
    x_real_complete_(14) = 0.0;
    x_real_complete_(15) = 0.0;
    }
    attitude.x=qx;
    attitude.y=qy;
    attitude.z=qz;
    attitude.w=qw;
    omega.x=wx;
    omega.y=wy;
    omega.z=wz;
    geometry_msgs::Vector3 torque;
    double r=0, p=0, y=225;  // Rotate the previous pose by 45* about Z

        q_ref.setRPY(r, p, y);
        tf2::convert(attitude,attitude_);
        q_ref_inv=q_ref.inverse();//
  q_e= q_ref_inv*attitude_;  // Calculate the new orientation
  q_e.normalize();
        float R_11 = 2*(attitude.x*attitude.x + attitude.w*attitude.w)-1;
        float R_12 = 2*(attitude.x*attitude.y - attitude.w*attitude.z);
        float R_13 = 2*(attitude.x*attitude.z + attitude.w*attitude.y); 
        float R_21 = 2*(attitude.x*attitude.y + attitude.w*attitude.z);
        float R_22 = 2*(attitude.y*attitude.y + attitude.w*attitude.w)-1;
        float R_23 = 2*(attitude.y*attitude.z - attitude.w*attitude.x);
        float R_31 = 2*(attitude.x*attitude.z - attitude.w*attitude.y); 
        float R_32 = 2*(attitude.y*attitude.z + attitude.w*attitude.x);
        float R_33 = 2*(attitude.z*attitude.z + attitude.w*attitude.w)-1;
        
        float cm_x =-0.3;
        float cm_y =0.0;
        float cm_z =0.0;


    position_.x = px + (cm_x*R_11 + cm_y*R_21 + cm_z*R_31);
    position_.y = py + (cm_x*R_12 + cm_y*R_22 + cm_z*R_32);
    position_.z = pz + (cm_x*R_13 + cm_y*R_23 + cm_z*R_33);

    /* position_ref.x = 10.8333388725;
    position_ref.y = -9.41988714508+0.5;
    position_ref.z = 4.20110343832;  */

  if(initialzation){
    position_error.x = position_.x - position_ref.x;
    position_error.y = position_.y - position_ref.y;
    position_error.z = position_.z - position_ref.z;

    velocity_.x=vx;
    velocity_.y=vy;
    velocity_.z=vz;

  

     arg_x_e = position_error.x;
     arg_y_e = position_error.y;
     arg_z_e = position_error.z;
     arg_vx  = vx;
     arg_vy  = vy;
     arg_vz  = vz;
     arg_qx = q_e.getX();
     arg_qy = q_e.getY();
     arg_qz = q_e.getZ();
     arg_qw = q_e.getW();
     arg_omegax = wx;
     arg_omegay = wy;
     arg_omegaz = wz;
    /* double *fx;
    double *fy;
    double *fz;
    double *taux;
    double *tauy;
    double *tauz; */
    step_PID();
    x0[0]=position_error.x;
    x0[1]=position_error.y;
    x0[2]=position_error.z;
    x0[3]=vx;
    x0[4]=vy;
    x0[5]=vz;
    main_MPC_Guidance_v3_sand();
    if (sqrt(q_e.getX()*q_e.getX()+q_e.getY()*q_e.getY()+q_e.getZ()*q_e.getZ())>0.005){
      z_nominal[0]=x0[0];
      z_nominal[1]=x0[1];
      z_nominal[2]=x0[2];
      z_nominal[3]=x0[3];
      z_nominal[4]=x0[4];
      z_nominal[5]=x0[5];

      initial_run=false;


    }
    else{
      z_nominal[0]=zp_nextNominal[0];
      z_nominal[1]=zp_nextNominal[1];
      z_nominal[2]=zp_nextNominal[2];
      z_nominal[3]=zp_nextNominal[3];
      z_nominal[4]=zp_nextNominal[4];
      z_nominal[5]=zp_nextNominal[5];


    }

    v_mpc[0]=Fx;
    v_mpc[1]=Fy;
    v_mpc[2]=Fz;
    nominal_dynamics();
    kn_tilda[0]=Fx;
    kn_tilda[1]=Fy;
    kn_tilda[2]=Fz;

      double sx=x0[0]-zp_nextNominal[0];
      double sy=x0[1]-zp_nextNominal[1];
      double sz=x0[2]-zp_nextNominal[2];
      double svx=x0[3]-zp_nextNominal[3];
      double svy=x0[4]-zp_nextNominal[4];
      double svz=x0[5]-zp_nextNominal[5];

    tubing_mpc();
    //X_QP=X_Qp
   // rt_OneStep();
   //ROS_INFO("ex: [%f]  ey: [%f] ez: [%f] ev_x: [%f] ev_y: [%f] ev_z: [%f]", sx,sy,sz,svx,svy,svz);

   // ROS_INFO("fx: [%f]  fy: [%f] fz: [%f] tau_x: [%f] tau_y: [%f] tau_y: [%f]", Fx,Fy,Fz, arg_tau_x,arg_tau_y,arg_tau_z);
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::debug(){
  /**
   * @brief debug function call to test compatibility with other Bases
   * 
   */

}
/* *************************************************************************** */
template<typename T>
void CoordinatorBase<T>::tubing_mpc(){
  double a[18] = { 2.0202, 0.0, 0.0, 0.0, 2.0202, 0.0, 0.0, 0.0,
    2.0202, 4.0895, 0.0, 0.0, 0.0, 4.0895, 0.0, 0.0, 0.0, 4.0895 };
  double b_x[6];
  double d;

  if (((kn_tilda[0] == 0.0) && (kn_tilda[1] == 0.0)) && (kn_tilda[2] == 0.0)) {
    kN[0] = 0.0;
    kN[1] = 0.0;
    kN[2] = 0.0;
  } else {
    int i;
    for (i = 0; i < 6; i++) {
      b_x[i] = x0[i] - z_nominal[i];
    }

    for (i = 0; i < 3; i++) {
      d = 0.0;
      for (int i1 = 0; i1 < 6; i1++) {
        d += a[i + (3 * i1)] * b_x[i1];
      }

      kN[i] = kn_tilda[i] - d;
    }
  }



}
/* **************************************************************************** */
template<typename T>
void CoordinatorBase<T>::nominal_dynamics(){
   double b_a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0 };

   double a[18] = { 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.082557 };

   double d;

  // MPCParams.A;
  // MPCParams.B;
  for (int i = 0; i < 6; i++) {
    d = 0.0;
    for (int i1 = 0; i1 < 6; i1++) {
      d += b_a[i + (6 * i1)] * z_nominal[i1];
    }

    zp_nextNominal[i] = d + (((a[i] * v_mpc[0]) + (a[i + 6] * v_mpc[1])) + (a[i + 12] * v_mpc[2]));
  }



}

template<typename T>
void CoordinatorBase<T>::step_PID(){
  /**
   * @brief debug function call to test compatibility with other Bases
   * 
   */

    float a[18] = { -0.3832000000000001, -0.0, -0.0, -0.0,
    -0.3832000000000001, -0.0, -0.0, -0.0, -0.3832000000000001,
    -3.8281680000000002, -0.0, -0.0, -0.0, -3.8281680000000002, -0.0, -0.0, -0.0,
    -3.8281680000000002 };
    
    /* { -4.0, -0.0, -0.0, -0.0, -4.0, -0.0, -0.0, -0.0,
    -4.0, -2.828, -0.0, -0.0, -0.0, -2.828, -0.0, -0.0, -0.0, -2.828 }; */

   float a_0[18] ={ -0.0095625, -0.0, -0.0, -0.0, -0.0089375, -0.0,
    -0.0, -0.0, -0.0101875, -0.0764235, -0.0, -0.0, -0.0, -0.071428499999999992,
    -0.0, -0.0, -0.0, -0.0814185 };
   
    /* { -0.612, -0.0, -0.0, -0.0, -0.572, -0.0, -0.0,
    -0.0, -0.652, -0.43268399999999996, -0.0, -0.0, -0.0, -0.40440399999999993,
    -0.0, -0.0, -0.0, -0.460964 }; */

  float tmp[6];
  float u[3];
  float tau[3];
  int i;
  int i_0;
  //UNUSED_PARAMETER(arg_qw);

  // MATLAB Function: '<Root>/Position Controller' incorporates:
  //   Inport: '<Root>/vx'
  //   Inport: '<Root>/vy'
  //   Inport: '<Root>/vz'
  //   Inport: '<Root>/x_e'
  //   Inport: '<Root>/y_e'
  //   Inport: '<Root>/z_e'

  tmp[0] = arg_x_e;
  tmp[1] = arg_y_e;
  tmp[2] = arg_z_e;
  tmp[3] = arg_vx;
  tmp[4] = arg_vy;
  tmp[5] = arg_vz;
  for (i = 0; i < 3; i++) {
    u[i] = 0.0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      u[i] += a[3 * i_0 + i] * tmp[i_0];
    }
  }


 /*  tmp[0] = arg_qx;
  tmp[1] = arg_qy;
  tmp[2] = arg_qz;
  tmp[3] = arg_omegax;
  tmp[4] = arg_omegay;
  tmp[5] = arg_omegaz;
  for (i = 0; i < 3; i++) {
    tau[i] = 0.0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      tau[i] += a_0[3 * i_0 + i] * tmp[i_0];
    }
  } */
 arg_fx=u[0];
 arg_fy=u[1];
 arg_fz=u[2];


 double qw = arg_qx;
 double qx = arg_qx;
 double qy = arg_qy;
 double qz = arg_qz;
 double fx = Fx; 
 double fy = Fy;
 double fz = Fz;
 double Omega1 = arg_omegax;
 double Omega2 = arg_omegay;
 double Omega3 = arg_omegaz;

 static const real_T a1[18] = { -0.11816682123356198, -0.0, -0.0, -0.0,
    -0.43472762491780986, -0.0, -0.0, -0.0, -0.35981497968424792,
    -1.18166821233562, -0.0, -0.0, -0.0, -4.3472762491780985, -0.0, -0.0, -0.0,
    -3.5981497968424794 };

  static const real_T b_a[9] = { 0.0, -0.030686081370449678, 0.0,
    0.030686081370449678, 0.0, 0.061372162740899357, -0.0, -0.061372162740899357,
    0.0 };

  real_T dv[9];
  real_T b_qx[6];
  real_T U[3];
  real_T dv1[3];
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  real_T d4;
  //int32_T i;

  // best case
  // 5.58;%9.58;%=2*9.58;
  // 9.58;
  //  d=0.6; % Distance between two Astrobee com
  // d/5;
  //  M=2;%linear density of arm kg/m
  // J1=M*l^3/4
  // J2=M*l^3/4
  // 0.453105990000000;% =2*0.153427995+13/16 Ml^2 Ixx of compund object
  // 0.540000000000000;%3*M*l^2;
  // 10.236728099999999 ;%=190/16M*l^2+2*(J1+J2+0.14271405)+9*9.58*l^2 Iyy of compund object 
  // 9.960905517999999 ;%=162/16*M*l^2+2*(J1+J2+0.162302759)+9*9.58*l^2 Izz of compund object 
  //  Ix=0.306855990;% =2*0.153427995 Ixx of compund object
  //  Iy=7.183028100000000 ;%=2*0.14271405+m*d^2 Iyy of compund object
  //  Iz=7.222205518000000 ;%=2*0.162302759+m*d^2 Izz of compund object
  //  PD gains
  // 1
  b_qx[0] = qx;
  b_qx[1] = qy;
  b_qx[2] = qz;
  b_qx[3] = Omega1;
  b_qx[4] = Omega2;
  b_qx[5] = Omega3;
  d = qw * qw;
  dv[0] = (2.0 * (d + (qx * qx))) - 1.0;
  d1 = qx * qy;
  d2 = qw * qz;
  dv[3] = 2.0 * (d1 - d2);
  d3 = qx * qz;
  d4 = qw * qy;
  dv[6] = 2.0 * (d3 + d4);
  dv[1] = 2.0 * (d1 + d2);
  dv[4] = (2.0 * (d + (qy * qy))) - 1.0;
  d1 = qy * qz;
  d2 = qw * qx;
  dv[7] = 2.0 * (d1 - d2);
  dv[2] = 2.0 * (d3 - d4);
  dv[5] = 2.0 * (d1 + d2);
  dv[8] = (2.0 * (d + (qz * qz))) - 1.0;
  for (i = 0; i < 3; i++) {
    dv1[i] = ((dv[i] * fx) + (dv[i + 3] * fy)) + (dv[i + 6] * fz);
    d = 0.0;
    for (int32_T i1 = 0; i1 < 6; i1++) {
      d += a1[i + (3 * i1)] * b_qx[i1];
    }

    U[i] = d;
  }

  d = dv1[0];
  d1 = dv1[1];
  d2 = dv1[2];
  for (i = 0; i < 3; i++) {
    U[i] -= ((b_a[i] * d) + (b_a[i + 3] * d1)) + (b_a[i + 6] * d2);
  }

  arg_tau_x = U[0];
  arg_tau_y = U[1];
  arg_tau_z = U[2];


}


//void rt_OneStep(void);
//void rt_OneStep(void)
/* template<typename T>
void CoordinatorBase<T>::rt_OneStep()
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(controller1ModelClass::getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  controller1ModelClass::step(arg_x_e, arg_y_e, arg_z_e, arg_vx, arg_vy, arg_vz, arg_qx,
                       arg_qy, arg_qz, arg_qw, arg_omegax, arg_omegay,
                       arg_omegaz, &arg_fx, &arg_fy, &arg_fz, &arg_tau_x,
                       &arg_tau_y, &arg_tau_z);

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
} */


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::disable_default_ctl_callback(const ros::TimerEvent&) {
  /**
   * @brief Switch default controller off, repeatedly.
   * @param base_status.default_control is monitored for activation
   */
  if (base_status_.default_control == false){
    auto serv_start = std::chrono::high_resolution_clock::now();

    // Disable the default controller so custom controller can run
    std_srvs::SetBool srv;
    srv.request.data = false;
    serv_ctl_enable_.call(srv);

    auto serv_finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> serv_elapsed = serv_finish - serv_start;
    std::string response = srv.response.message;

    std::cout << "[COORDINATOR]: Controller disable service time: " << serv_elapsed.count() << " seconds."<< std::endl;
    std::cout << "[COORDINATOR]: Controller disable service result: " << response << std::endl;
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::disable_default_ctl() {
  /**
   * @brief Switch default controller off.
   * @param base_status.default_control is monitored for activation
   */
  base_status_.default_control = false;

  // Disable the default controller so custom controller can run
  std_srvs::SetBool srv;
  srv.request.data = false;
  serv_ctl_enable_.call(srv);
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::enable_default_ctl() {
  /**
   * @brief Switch default controller on.
   * 
   */
  ROS_DEBUG_STREAM("[COORDINATOR]: Enabling default controller...");

  // Disable the default controller so tube-MPC can run
  base_status_.default_control = true;

  std_srvs::SetBool srv;
  srv.request.data = true;
  serv_ctl_enable_.call(srv);

  ROS_DEBUG_STREAM("[COORDINATOR]: Ctl enable service result: " << srv.response.message);
}


template<typename T>
void CoordinatorBase<T>::main_MPC_Guidance_v3_sand()
{

  // Initialize function 'MPC_Guidance_v3_sand' input arguments.
  // Initialize function input argument 'x0'.
  // Call the entry-point 'MPC_Guidance_v3_sand'.
  
  MPC_Guidance_v3_sand();
}


template<typename T>
bool  CoordinatorBase<T>::rtIsNaN(double value){
  return ((value!=value) ? true : false);
}

template<typename T>
void CoordinatorBase<T>::MPC_Guidance_v3_sand()
{
   static const real_T b[14400] = { 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100000.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1605.5555, -1.5169E-12, -3.0491E-12, 10220.2817,
    4.1155E-11, 2.9213E-11, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.5169E-12, 1605.5555, 1.3995E-11, 1.4663E-11, 10220.2817, 9.1029E-11, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0491E-12, 1.3995E-11, 1605.5555,
    -7.439E-12, 1.8651E-10, 10220.2817, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 10220.2817, 1.4663E-11, -7.439E-12, 508614.8614, 2.5421E-9, 1.2778E-9,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.1155E-11, 10220.2817,
    1.8651E-10, 2.5421E-9, 508614.8614, 1.2463E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 2.9213E-11, 9.1029E-11, 10220.2817, 1.2778E-9, 1.2463E-9,
    508614.8614 };

  static const real_T b_b[7200] = { 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0,
    2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0,
    2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0, 0.0,
    2.8895, 0.0, 0.0, 0.082557, 0.0, 0.0, 3.0546, 0.0, 0.0, 0.082557, 0.0, 0.0,
    3.2197, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0, 0.0, 3.0546, 0.0, 0.0, 0.082557, 0.0,
    0.0, 3.2197, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557,
    0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557,
    0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244, 0.0, 0.0, 0.082557,
    0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0, 0.0, 3.0546, 0.0, 0.0, 0.082557,
    0.0, 0.0, 3.2197, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0, 0.0, 3.0546,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0, 0.0, 3.0546,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244,
    0.0, 0.0, 0.082557, 0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0, 0.0, 3.0546,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557,
    0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557,
    0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557,
    0.0, 0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.8895, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.7244, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.7244, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0,
    2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0,
    2.5593, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.5593, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0,
    0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0,
    2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.3942, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.229, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0,
    0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0,
    0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.229, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0,
    0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 2.0639,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.8988, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.7337, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.7337, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0, 0.0,
    1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.5686, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732,
    0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.4035,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557,
    0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.2384, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0,
    0.082557, 0.0, 0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0,
    0.0, 1.0732, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 1.0732, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.90813, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.90813, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.74301, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.5779, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.41279,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0,
    0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.24767, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.082557, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.082557, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.082557, 0.0, 0.0, 0.082557 };

  static const real_T b_H[3600] = { 105707.9487, 3.2876E-11, -3.4224E-11,
    81671.1176, 3.4089E-11, -3.0779E-11, 77638.1032, 3.5302E-11, -2.7335E-11,
    73612.7224, 3.6515E-11, -2.389E-11, 69598.7919, 3.7728E-11, -2.0445E-11,
    65600.1284, 3.8941E-11, -1.7E-11, 61620.5488, 4.0154E-11, -1.3556E-11,
    57663.8698, 4.1368E-11, -1.0111E-11, 53733.9082, 4.2581E-11, -6.6662E-12,
    49834.4808, 4.3794E-11, -3.2214E-12, 45969.4042, 4.5007E-11, 2.2332E-13,
    42142.4954, 4.622E-11, 3.6681E-12, 38357.571, 4.7433E-11, 7.1128E-12,
    34618.4478, 4.8646E-11, 1.0558E-11, 30928.9426, 4.9859E-11, 1.4002E-11,
    27292.8722, 5.1072E-11, 1.7447E-11, 23714.0533, 5.2285E-11, 2.0892E-11,
    20196.3027, 5.3498E-11, 2.4337E-11, 16743.4372, 5.4711E-11, 2.7781E-11,
    13359.2735, 5.5925E-11, 3.1226E-11, 3.2876E-11, 105707.9487, 4.547E-10,
    3.3367E-11, 81671.1176, 4.3473E-10, 3.3858E-11, 77638.1032, 4.1477E-10,
    3.4348E-11, 73612.7224, 3.948E-10, 3.4839E-11, 69598.7919, 3.7484E-10,
    3.533E-11, 65600.1284, 3.5487E-10, 3.5821E-11, 61620.5488, 3.3491E-10,
    3.6312E-11, 57663.8698, 3.1494E-10, 3.6803E-11, 53733.9082, 2.9498E-10,
    3.7294E-11, 49834.4808, 2.7501E-10, 3.7784E-11, 45969.4042, 2.5505E-10,
    3.8275E-11, 42142.4954, 2.3508E-10, 3.8766E-11, 38357.571, 2.1512E-10,
    3.9257E-11, 34618.4478, 1.9515E-10, 3.9748E-11, 30928.9426, 1.7519E-10,
    4.0239E-11, 27292.8722, 1.5522E-10, 4.0729E-11, 23714.0533, 1.3526E-10,
    4.122E-11, 20196.3027, 1.1529E-10, 4.1711E-11, 16743.4372, 9.5329E-11,
    4.2202E-11, 13359.2735, 7.5364E-11, -3.4224E-11, 4.547E-10, 105707.9487,
    -3.1779E-11, 4.3733E-10, 81671.1176, -2.9333E-11, 4.1997E-10, 77638.1032,
    -2.6888E-11, 4.0261E-10, 73612.7224, -2.4442E-11, 3.8525E-10, 69598.7919,
    -2.1997E-11, 3.6789E-10, 65600.1284, -1.9551E-11, 3.5053E-10, 61620.5488,
    -1.7106E-11, 3.3316E-10, 57663.8698, -1.466E-11, 3.158E-10, 53733.9082,
    -1.2214E-11, 2.9844E-10, 49834.4808, -9.769E-12, 2.8108E-10, 45969.4042,
    -7.3235E-12, 2.6372E-10, 42142.4954, -4.878E-12, 2.4635E-10, 38357.571,
    -2.4324E-12, 2.2899E-10, 34618.4478, 1.306E-14, 2.1163E-10, 30928.9426,
    2.4586E-12, 1.9427E-10, 27292.8722, 4.9041E-12, 1.7691E-10, 23714.0533,
    7.3496E-12, 1.5955E-10, 20196.3027, 9.7951E-12, 1.4218E-10, 16743.4372,
    1.2241E-11, 1.2482E-10, 13359.2735, 81671.1176, 3.3367E-11, -3.1779E-11,
    99154.6174, 3.4497E-11, -2.85E-11, 75275.9399, 3.5628E-11, -2.5222E-11,
    71401.0791, 3.6758E-11, -2.1943E-11, 67533.8518, 3.7888E-11, -1.8665E-11,
    63678.0749, 3.9019E-11, -1.5386E-11, 59837.565, 4.0149E-11, -1.2108E-11,
    56016.1389, 4.1279E-11, -8.8292E-12, 52217.6135, 4.241E-11, -5.5507E-12,
    48445.8055, 4.354E-11, -2.2722E-12, 44704.5316, 4.4671E-11, 1.0063E-12,
    40997.6086, 4.5801E-11, 4.2848E-12, 37328.8533, 4.6931E-11, 7.5633E-12,
    33702.0825, 4.8062E-11, 1.0842E-11, 30121.1129, 4.9192E-11, 1.412E-11,
    26589.7612, 5.0322E-11, 1.7399E-11, 23111.8444, 5.1453E-11, 2.0677E-11,
    19691.179, 5.2583E-11, 2.3956E-11, 16331.582, 5.3713E-11, 2.7234E-11,
    13036.87, 5.4844E-11, 3.0513E-11, 3.4089E-11, 81671.1176, 4.3733E-10,
    3.4497E-11, 99154.6174, 4.1813E-10, 3.4905E-11, 75275.9399, 3.9893E-10,
    3.5313E-11, 71401.0791, 3.7973E-10, 3.5722E-11, 67533.8518, 3.6053E-10,
    3.613E-11, 63678.0749, 3.4133E-10, 3.6538E-11, 59837.565, 3.2212E-10,
    3.6946E-11, 56016.1389, 3.0292E-10, 3.7354E-11, 52217.6135, 2.8372E-10,
    3.7762E-11, 48445.8055, 2.6452E-10, 3.817E-11, 44704.5316, 2.4532E-10,
    3.8578E-11, 40997.6086, 2.2611E-10, 3.8987E-11, 37328.8533, 2.0691E-10,
    3.9395E-11, 33702.0825, 1.8771E-10, 3.9803E-11, 30121.1129, 1.6851E-10,
    4.0211E-11, 26589.7612, 1.4931E-10, 4.0619E-11, 23111.8444, 1.3011E-10,
    4.1027E-11, 19691.179, 1.109E-10, 4.1435E-11, 16331.582, 9.1703E-11,
    4.1844E-11, 13036.87, 7.2501E-11, -3.0779E-11, 4.3473E-10, 81671.1176,
    -2.85E-11, 4.1813E-10, 99154.6174, -2.6221E-11, 4.0153E-10, 75275.9399,
    -2.3942E-11, 3.8494E-10, 71401.0791, -2.1662E-11, 3.6834E-10, 67533.8518,
    -1.9383E-11, 3.5174E-10, 63678.0749, -1.7104E-11, 3.3514E-10, 59837.565,
    -1.4825E-11, 3.1854E-10, 56016.1389, -1.2545E-11, 3.0194E-10, 52217.6135,
    -1.0266E-11, 2.8534E-10, 48445.8055, -7.9868E-12, 2.6874E-10, 44704.5316,
    -5.7075E-12, 2.5215E-10, 40997.6086, -3.4283E-12, 2.3555E-10, 37328.8533,
    -1.149E-12, 2.1895E-10, 33702.0825, 1.1303E-12, 2.0235E-10, 30121.1129,
    3.4095E-12, 1.8575E-10, 26589.7612, 5.6888E-12, 1.6915E-10, 23111.8444,
    7.968E-12, 1.5255E-10, 19691.179, 1.0247E-11, 1.3595E-10, 16331.582,
    1.2527E-11, 1.1936E-10, 13036.87, 77638.1032, 3.3858E-11, -2.9333E-11,
    75275.9399, 3.4905E-11, -2.6221E-11, 92913.7766, 3.5953E-11, -2.3109E-11,
    69189.4358, 3.7001E-11, -1.9996E-11, 65468.9118, 3.8048E-11, -1.6884E-11,
    61756.0213, 3.9096E-11, -1.3772E-11, 58054.5811, 4.0144E-11, -1.066E-11,
    54368.4081, 4.1191E-11, -7.5474E-12, 50701.3188, 4.2239E-11, -4.4352E-12,
    47057.1302, 4.3287E-11, -1.323E-12, 43439.6589, 4.4334E-11, 1.7893E-12,
    39852.7218, 4.5382E-11, 4.9015E-12, 36300.1356, 4.643E-11, 8.0137E-12,
    32785.7171, 4.7477E-11, 1.1126E-11, 29313.2831, 4.8525E-11, 1.4238E-11,
    25886.6503, 4.9573E-11, 1.735E-11, 22509.6354, 5.062E-11, 2.0463E-11,
    19186.0554, 5.1668E-11, 2.3575E-11, 15919.7268, 5.2716E-11, 2.6687E-11,
    12714.4666, 5.3763E-11, 2.9799E-11, 3.5302E-11, 77638.1032, 4.1997E-10,
    3.5628E-11, 75275.9399, 4.0153E-10, 3.5953E-11, 92913.7766, 3.831E-10,
    3.6278E-11, 69189.4358, 3.6466E-10, 3.6604E-11, 65468.9118, 3.4622E-10,
    3.6929E-11, 61756.0213, 3.2778E-10, 3.7255E-11, 58054.5811, 3.0934E-10,
    3.758E-11, 54368.4081, 2.909E-10, 3.7905E-11, 50701.3188, 2.7246E-10,
    3.8231E-11, 47057.1302, 2.5402E-10, 3.8556E-11, 43439.6589, 2.3559E-10,
    3.8882E-11, 39852.7218, 2.1715E-10, 3.9207E-11, 36300.1356, 1.9871E-10,
    3.9533E-11, 32785.7171, 1.8027E-10, 3.9858E-11, 29313.2831, 1.6183E-10,
    4.0183E-11, 25886.6503, 1.4339E-10, 4.0509E-11, 22509.6354, 1.2495E-10,
    4.0834E-11, 19186.0554, 1.0651E-10, 4.116E-11, 15919.7268, 8.8076E-11,
    4.1485E-11, 12714.4666, 6.9638E-11, -2.7335E-11, 4.1477E-10, 77638.1032,
    -2.5222E-11, 3.9893E-10, 75275.9399, -2.3109E-11, 3.831E-10, 92913.7766,
    -2.0996E-11, 3.6726E-10, 69189.4358, -1.8883E-11, 3.5142E-10, 65468.9118,
    -1.677E-11, 3.3559E-10, 61756.0213, -1.4657E-11, 3.1975E-10, 58054.5811,
    -1.2544E-11, 3.0392E-10, 54368.4081, -1.0431E-11, 2.8808E-10, 50701.3188,
    -8.3176E-12, 2.7225E-10, 47057.1302, -6.2046E-12, 2.5641E-10, 43439.6589,
    -4.0916E-12, 2.4057E-10, 39852.7218, -1.9786E-12, 2.2474E-10, 36300.1356,
    1.3445E-13, 2.089E-10, 32785.7171, 2.2475E-12, 1.9307E-10, 29313.2831,
    4.3605E-12, 1.7723E-10, 25886.6503, 6.4735E-12, 1.614E-10, 22509.6354,
    8.5865E-12, 1.4556E-10, 19186.0554, 1.0699E-11, 1.2973E-10, 15919.7268,
    1.2812E-11, 1.1389E-10, 12714.4666, 73612.7224, 3.4348E-11, -2.6888E-11,
    71401.0791, 3.5313E-11, -2.3942E-11, 69189.4358, 3.6278E-11, -2.0996E-11,
    86977.7925, 3.7243E-11, -1.805E-11, 63403.9717, 3.8208E-11, -1.5104E-11,
    59833.9678, 3.9173E-11, -1.2158E-11, 56271.5973, 4.0138E-11, -9.2117E-12,
    52720.6772, 4.1103E-11, -6.2657E-12, 49185.0241, 4.2068E-11, -3.3197E-12,
    45668.4549, 4.3033E-11, -3.7372E-13, 42174.7863, 4.3998E-11, 2.5723E-12,
    38707.835, 4.4963E-11, 5.5182E-12, 35271.418, 4.5928E-11, 8.4642E-12,
    31869.3518, 4.6893E-11, 1.141E-11, 28505.4533, 4.7858E-11, 1.4356E-11,
    25183.5393, 4.8823E-11, 1.7302E-11, 21907.4265, 4.9788E-11, 2.0248E-11,
    18680.9317, 5.0753E-11, 2.3194E-11, 15507.8716, 5.1718E-11, 2.614E-11,
    12392.0631, 5.2683E-11, 2.9086E-11, 3.6515E-11, 73612.7224, 4.0261E-10,
    3.6758E-11, 71401.0791, 3.8494E-10, 3.7001E-11, 69189.4358, 3.6726E-10,
    3.7243E-11, 86977.7925, 3.4958E-10, 3.7486E-11, 63403.9717, 3.3191E-10,
    3.7729E-11, 59833.9678, 3.1423E-10, 3.7971E-11, 56271.5973, 2.9656E-10,
    3.8214E-11, 52720.6772, 2.7888E-10, 3.8457E-11, 49185.0241, 2.6121E-10,
    3.87E-11, 45668.4549, 2.4353E-10, 3.8942E-11, 42174.7863, 2.2585E-10,
    3.9185E-11, 38707.835, 2.0818E-10, 3.9428E-11, 35271.418, 1.905E-10,
    3.967E-11, 31869.3518, 1.7283E-10, 3.9913E-11, 28505.4533, 1.5515E-10,
    4.0156E-11, 25183.5393, 1.3748E-10, 4.0399E-11, 21907.4265, 1.198E-10,
    4.0641E-11, 18680.9317, 1.0213E-10, 4.0884E-11, 15507.8716, 8.445E-11,
    4.1127E-11, 12392.0631, 6.6774E-11, -2.389E-11, 3.948E-10, 73612.7224,
    -2.1943E-11, 3.7973E-10, 71401.0791, -1.9996E-11, 3.6466E-10, 69189.4358,
    -1.805E-11, 3.4958E-10, 86977.7925, -1.6103E-11, 3.3451E-10, 63403.9717,
    -1.4156E-11, 3.1944E-10, 59833.9678, -1.2209E-11, 3.0437E-10, 56271.5973,
    -1.0263E-11, 2.8929E-10, 52720.6772, -8.3159E-12, 2.7422E-10, 49185.0241,
    -6.3691E-12, 2.5915E-10, 45668.4549, -4.4223E-12, 2.4408E-10, 42174.7863,
    -2.4756E-12, 2.29E-10, 38707.835, -5.2885E-13, 2.1393E-10, 35271.418,
    1.4179E-12, 1.9886E-10, 31869.3518, 3.3647E-12, 1.8379E-10, 28505.4533,
    5.3114E-12, 1.6871E-10, 25183.5393, 7.2582E-12, 1.5364E-10, 21907.4265,
    9.2049E-12, 1.3857E-10, 18680.9317, 1.1152E-11, 1.235E-10, 15507.8716,
    1.3098E-11, 1.0842E-10, 12392.0631, 69598.7919, 3.4839E-11, -2.4442E-11,
    67533.8518, 3.5722E-11, -2.1662E-11, 65468.9118, 3.6604E-11, -1.8883E-11,
    63403.9717, 3.7486E-11, -1.6103E-11, 81339.0317, 3.8368E-11, -1.3323E-11,
    57911.9142, 3.9251E-11, -1.0543E-11, 54488.6135, 4.0133E-11, -7.7637E-12,
    51072.9463, 4.1015E-11, -4.9839E-12, 47668.7294, 4.1897E-11, -2.2042E-12,
    44279.7796, 4.278E-11, 5.7551E-13, 40909.9136, 4.3662E-11, 3.3552E-12,
    37562.9483, 4.4544E-11, 6.135E-12, 34242.7003, 4.5426E-11, 8.9147E-12,
    30952.9865, 4.6308E-11, 1.1694E-11, 27697.6235, 4.7191E-11, 1.4474E-11,
    24480.4283, 4.8073E-11, 1.7254E-11, 21305.2176, 4.8955E-11, 2.0034E-11,
    18175.808, 4.9837E-11, 2.2813E-11, 15096.0165, 5.072E-11, 2.5593E-11,
    12069.6597, 5.1602E-11, 2.8373E-11, 3.7728E-11, 69598.7919, 3.8525E-10,
    3.7888E-11, 67533.8518, 3.6834E-10, 3.8048E-11, 65468.9118, 3.5142E-10,
    3.8208E-11, 63403.9717, 3.3451E-10, 3.8368E-11, 81339.0317, 3.176E-10,
    3.8528E-11, 57911.9142, 3.0069E-10, 3.8688E-11, 54488.6135, 2.8377E-10,
    3.8848E-11, 51072.9463, 2.6686E-10, 3.9008E-11, 47668.7294, 2.4995E-10,
    3.9168E-11, 44279.7796, 2.3304E-10, 3.9328E-11, 40909.9136, 2.1612E-10,
    3.9488E-11, 37562.9483, 1.9921E-10, 3.9648E-11, 34242.7003, 1.823E-10,
    3.9808E-11, 30952.9865, 1.6539E-10, 3.9968E-11, 27697.6235, 1.4847E-10,
    4.0128E-11, 24480.4283, 1.3156E-10, 4.0288E-11, 21305.2176, 1.1465E-10,
    4.0448E-11, 18175.808, 9.7736E-11, 4.0608E-11, 15096.0165, 8.0824E-11,
    4.0768E-11, 12069.6597, 6.3911E-11, -2.0445E-11, 3.7484E-10, 69598.7919,
    -1.8665E-11, 3.6053E-10, 67533.8518, -1.6884E-11, 3.4622E-10, 65468.9118,
    -1.5104E-11, 3.3191E-10, 63403.9717, -1.3323E-11, 3.176E-10, 81339.0317,
    -1.1543E-11, 3.0329E-10, 57911.9142, -9.7621E-12, 2.8898E-10, 54488.6135,
    -7.9816E-12, 2.7467E-10, 51072.9463, -6.2011E-12, 2.6036E-10, 47668.7294,
    -4.4206E-12, 2.4605E-10, 44279.7796, -2.6401E-12, 2.3174E-10, 40909.9136,
    -8.5964E-13, 2.1743E-10, 37562.9483, 9.2086E-13, 2.0312E-10, 34242.7003,
    2.7014E-12, 1.8881E-10, 30952.9865, 4.4819E-12, 1.745E-10, 27697.6235,
    6.2624E-12, 1.6019E-10, 24480.4283, 8.0428E-12, 1.4589E-10, 21305.2176,
    9.8233E-12, 1.3158E-10, 18175.808, 1.1604E-11, 1.1727E-10, 15096.0165,
    1.3384E-11, 1.0296E-10, 12069.6597, 65600.1284, 3.533E-11, -2.1997E-11,
    63678.0749, 3.613E-11, -1.9383E-11, 61756.0213, 3.6929E-11, -1.677E-11,
    59833.9678, 3.7729E-11, -1.4156E-11, 57911.9142, 3.8528E-11, -1.1543E-11,
    75989.8606, 3.9328E-11, -8.9292E-12, 52705.6296, 4.0127E-11, -6.3157E-12,
    49425.2154, 4.0927E-11, -3.7022E-12, 46152.4347, 4.1726E-11, -1.0887E-12,
    42891.1043, 4.2526E-11, 1.5247E-12, 39645.041, 4.3326E-11, 4.1382E-12,
    36418.0615, 4.4125E-11, 6.7517E-12, 33213.9826, 4.4925E-11, 9.3652E-12,
    30036.6211, 4.5724E-11, 1.1979E-11, 26889.7938, 4.6524E-11, 1.4592E-11,
    23777.3174, 4.7323E-11, 1.7206E-11, 20703.0086, 4.8123E-11, 1.9819E-11,
    17670.6843, 4.8922E-11, 2.2433E-11, 14684.1613, 4.9722E-11, 2.5046E-11,
    11747.2562, 5.0521E-11, 2.7659E-11, 3.8941E-11, 65600.1284, 3.6789E-10,
    3.9019E-11, 63678.0749, 3.5174E-10, 3.9096E-11, 61756.0213, 3.3559E-10,
    3.9173E-11, 59833.9678, 3.1944E-10, 3.9251E-11, 57911.9142, 3.0329E-10,
    3.9328E-11, 75989.8606, 2.8714E-10, 3.9405E-11, 52705.6296, 2.7099E-10,
    3.9482E-11, 49425.2154, 2.5484E-10, 3.956E-11, 46152.4347, 2.3869E-10,
    3.9637E-11, 42891.1043, 2.2254E-10, 3.9714E-11, 39645.041, 2.0639E-10,
    3.9792E-11, 36418.0615, 1.9024E-10, 3.9869E-11, 33213.9826, 1.7409E-10,
    3.9946E-11, 30036.6211, 1.5794E-10, 4.0023E-11, 26889.7938, 1.418E-10,
    4.0101E-11, 23777.3174, 1.2565E-10, 4.0178E-11, 20703.0086, 1.095E-10,
    4.0255E-11, 17670.6843, 9.3347E-11, 4.0333E-11, 14684.1613, 7.7197E-11,
    4.041E-11, 11747.2562, 6.1048E-11, -1.7E-11, 3.5487E-10, 65600.1284,
    -1.5386E-11, 3.4133E-10, 63678.0749, -1.3772E-11, 3.2778E-10, 61756.0213,
    -1.2158E-11, 3.1423E-10, 59833.9678, -1.0543E-11, 3.0069E-10, 57911.9142,
    -8.9292E-12, 2.8714E-10, 75989.8606, -7.3149E-12, 2.7359E-10, 52705.6296,
    -5.7007E-12, 2.6005E-10, 49425.2154, -4.0864E-12, 2.465E-10, 46152.4347,
    -2.4722E-12, 2.3295E-10, 42891.1043, -8.5793E-13, 2.1941E-10, 39645.041,
    7.5632E-13, 2.0586E-10, 36418.0615, 2.3706E-12, 1.9232E-10, 33213.9826,
    3.9848E-12, 1.7877E-10, 30036.6211, 5.5991E-12, 1.6522E-10, 26889.7938,
    7.2133E-12, 1.5168E-10, 23777.3174, 8.8275E-12, 1.3813E-10, 20703.0086,
    1.0442E-11, 1.2458E-10, 17670.6843, 1.2056E-11, 1.1104E-10, 14684.1613,
    1.367E-11, 9.7491E-11, 11747.2562, 61620.5488, 3.5821E-11, -1.9551E-11,
    59837.565, 3.6538E-11, -1.7104E-11, 58054.5811, 3.7255E-11, -1.4657E-11,
    56271.5973, 3.7971E-11, -1.2209E-11, 54488.6135, 3.8688E-11, -9.7621E-12,
    52705.6296, 3.9405E-11, -7.3149E-12, 70922.6458, 4.0122E-11, -4.8677E-12,
    47777.4845, 4.0839E-11, -2.4205E-12, 44636.14, 4.1556E-11, 2.6757E-14,
    41502.429, 4.2272E-11, 2.474E-12, 38380.1683, 4.2989E-11, 4.9212E-12,
    35273.1747, 4.3706E-11, 7.3684E-12, 32185.2649, 4.4423E-11, 9.8156E-12,
    29120.2558, 4.514E-11, 1.2263E-11, 26081.964, 4.5857E-11, 1.471E-11,
    23074.2064, 4.6573E-11, 1.7157E-11, 20100.7997, 4.729E-11, 1.9605E-11,
    17165.5607, 4.8007E-11, 2.2052E-11, 14272.3061, 4.8724E-11, 2.4499E-11,
    11424.8527, 4.9441E-11, 2.6946E-11, 4.0154E-11, 61620.5488, 3.5053E-10,
    4.0149E-11, 59837.565, 3.3514E-10, 4.0144E-11, 58054.5811, 3.1975E-10,
    4.0138E-11, 56271.5973, 3.0437E-10, 4.0133E-11, 54488.6135, 2.8898E-10,
    4.0127E-11, 52705.6296, 2.7359E-10, 4.0122E-11, 70922.6458, 2.5821E-10,
    4.0117E-11, 47777.4845, 2.4282E-10, 4.0111E-11, 44636.14, 2.2743E-10,
    4.0106E-11, 41502.429, 2.1205E-10, 4.01E-11, 38380.1683, 1.9666E-10,
    4.0095E-11, 35273.1747, 1.8128E-10, 4.0089E-11, 32185.2649, 1.6589E-10,
    4.0084E-11, 29120.2558, 1.505E-10, 4.0079E-11, 26081.964, 1.3512E-10,
    4.0073E-11, 23074.2064, 1.1973E-10, 4.0068E-11, 20100.7997, 1.0434E-10,
    4.0062E-11, 17165.5607, 8.8957E-11, 4.0057E-11, 14272.3061, 7.3571E-11,
    4.0052E-11, 11424.8527, 5.8185E-11, -1.3556E-11, 3.3491E-10, 61620.5488,
    -1.2108E-11, 3.2212E-10, 59837.565, -1.066E-11, 3.0934E-10, 58054.5811,
    -9.2117E-12, 2.9656E-10, 56271.5973, -7.7637E-12, 2.8377E-10, 54488.6135,
    -6.3157E-12, 2.7099E-10, 52705.6296, -4.8677E-12, 2.5821E-10, 70922.6458,
    -3.4197E-12, 2.4542E-10, 47777.4845, -1.9717E-12, 2.3264E-10, 44636.14,
    -5.2371E-13, 2.1986E-10, 41502.429, 9.2428E-13, 2.0707E-10, 38380.1683,
    2.3723E-12, 1.9429E-10, 35273.1747, 3.8203E-12, 1.8151E-10, 32185.2649,
    5.2683E-12, 1.6872E-10, 29120.2558, 6.7162E-12, 1.5594E-10, 26081.964,
    8.1642E-12, 1.4316E-10, 23074.2064, 9.6122E-12, 1.3037E-10, 20100.7997,
    1.106E-11, 1.1759E-10, 17165.5607, 1.2508E-11, 1.0481E-10, 14272.3061,
    1.3956E-11, 9.2025E-11, 11424.8527, 57663.8698, 3.6312E-11, -1.7106E-11,
    56016.1389, 3.6946E-11, -1.4825E-11, 54368.4081, 3.758E-11, -1.2544E-11,
    52720.6772, 3.8214E-11, -1.0263E-11, 51072.9463, 3.8848E-11, -7.9816E-12,
    49425.2154, 3.9482E-11, -5.7007E-12, 47777.4845, 4.0117E-11, -3.4197E-12,
    66129.7536, 4.0751E-11, -1.1387E-12, 43119.8453, 4.1385E-11, 1.1422E-12,
    40113.7537, 4.2019E-11, 3.4232E-12, 37115.2957, 4.2653E-11, 5.7042E-12,
    34128.2879, 4.3287E-11, 7.9852E-12, 31156.5473, 4.3921E-11, 1.0266E-11,
    28203.8904, 4.4555E-11, 1.2547E-11, 25274.1342, 4.5189E-11, 1.4828E-11,
    22371.0954, 4.5824E-11, 1.7109E-11, 19498.5907, 4.6458E-11, 1.939E-11,
    16660.437, 4.7092E-11, 2.1671E-11, 13860.4509, 4.7726E-11, 2.3952E-11,
    11102.4493, 4.836E-11, 2.6233E-11, 4.1368E-11, 57663.8698, 3.3316E-10,
    4.1279E-11, 56016.1389, 3.1854E-10, 4.1191E-11, 54368.4081, 3.0392E-10,
    4.1103E-11, 52720.6772, 2.8929E-10, 4.1015E-11, 51072.9463, 2.7467E-10,
    4.0927E-11, 49425.2154, 2.6005E-10, 4.0839E-11, 47777.4845, 2.4542E-10,
    4.0751E-11, 66129.7536, 2.308E-10, 4.0663E-11, 43119.8453, 2.1618E-10,
    4.0574E-11, 40113.7537, 2.0155E-10, 4.0486E-11, 37115.2957, 1.8693E-10,
    4.0398E-11, 34128.2879, 1.7231E-10, 4.031E-11, 31156.5473, 1.5768E-10,
    4.0222E-11, 28203.8904, 1.4306E-10, 4.0134E-11, 25274.1342, 1.2844E-10,
    4.0046E-11, 22371.0954, 1.1381E-10, 3.9958E-11, 19498.5907, 9.9191E-11,
    3.9869E-11, 16660.437, 8.4568E-11, 3.9781E-11, 13860.4509, 6.9945E-11,
    3.9693E-11, 11102.4493, 5.5321E-11, -1.0111E-11, 3.1494E-10, 57663.8698,
    -8.8292E-12, 3.0292E-10, 56016.1389, -7.5474E-12, 2.909E-10, 54368.4081,
    -6.2657E-12, 2.7888E-10, 52720.6772, -4.9839E-12, 2.6686E-10, 51072.9463,
    -3.7022E-12, 2.5484E-10, 49425.2154, -2.4205E-12, 2.4282E-10, 47777.4845,
    -1.1387E-12, 2.308E-10, 66129.7536, 1.4301E-13, 2.1878E-10, 43119.8453,
    1.4248E-12, 2.0676E-10, 40113.7537, 2.7065E-12, 1.9474E-10, 37115.2957,
    3.9882E-12, 1.8272E-10, 34128.2879, 5.27E-12, 1.707E-10, 31156.5473,
    6.5517E-12, 1.5868E-10, 28203.8904, 7.8334E-12, 1.4666E-10, 25274.1342,
    9.1152E-12, 1.3464E-10, 22371.0954, 1.0397E-11, 1.2262E-10, 19498.5907,
    1.1679E-11, 1.106E-10, 16660.437, 1.296E-11, 9.8578E-11, 13860.4509,
    1.4242E-11, 8.6558E-11, 11102.4493, 53733.9082, 3.6803E-11, -1.466E-11,
    52217.6135, 3.7354E-11, -1.2545E-11, 50701.3188, 3.7905E-11, -1.0431E-11,
    49185.0241, 3.8457E-11, -8.3159E-12, 47668.7294, 3.9008E-11, -6.2011E-12,
    46152.4347, 3.956E-11, -4.0864E-12, 44636.14, 4.0111E-11, -1.9717E-12,
    43119.8453, 4.0663E-11, 1.4301E-13, 61603.5506, 4.1214E-11, 2.2577E-12,
    38725.0784, 4.1765E-11, 4.3724E-12, 35850.423, 4.2317E-11, 6.4872E-12,
    32983.4011, 4.2868E-11, 8.6019E-12, 30127.8296, 4.342E-11, 1.0717E-11,
    27287.5251, 4.3971E-11, 1.2831E-11, 24466.3045, 4.4522E-11, 1.4946E-11,
    21667.9844, 4.5074E-11, 1.7061E-11, 18896.3818, 4.5625E-11, 1.9175E-11,
    16155.3133, 4.6177E-11, 2.129E-11, 13448.5957, 4.6728E-11, 2.3405E-11,
    10780.0458, 4.7279E-11, 2.552E-11, 4.2581E-11, 53733.9082, 3.158E-10,
    4.241E-11, 52217.6135, 3.0194E-10, 4.2239E-11, 50701.3188, 2.8808E-10,
    4.2068E-11, 49185.0241, 2.7422E-10, 4.1897E-11, 47668.7294, 2.6036E-10,
    4.1726E-11, 46152.4347, 2.465E-10, 4.1556E-11, 44636.14, 2.3264E-10,
    4.1385E-11, 43119.8453, 2.1878E-10, 4.1214E-11, 61603.5506, 2.0492E-10,
    4.1043E-11, 38725.0784, 1.9106E-10, 4.0872E-11, 35850.423, 1.772E-10,
    4.0701E-11, 32983.4011, 1.6334E-10, 4.0531E-11, 30127.8296, 1.4948E-10,
    4.036E-11, 27287.5251, 1.3562E-10, 4.0189E-11, 24466.3045, 1.2176E-10,
    4.0018E-11, 21667.9844, 1.079E-10, 3.9847E-11, 18896.3818, 9.4039E-11,
    3.9676E-11, 16155.3133, 8.0179E-11, 3.9506E-11, 13448.5957, 6.6318E-11,
    3.9335E-11, 10780.0458, 5.2458E-11, -6.6662E-12, 2.9498E-10, 53733.9082,
    -5.5507E-12, 2.8372E-10, 52217.6135, -4.4352E-12, 2.7246E-10, 50701.3188,
    -3.3197E-12, 2.6121E-10, 49185.0241, -2.2042E-12, 2.4995E-10, 47668.7294,
    -1.0887E-12, 2.3869E-10, 46152.4347, 2.6757E-14, 2.2743E-10, 44636.14,
    1.1422E-12, 2.1618E-10, 43119.8453, 2.2577E-12, 2.0492E-10, 61603.5506,
    3.3732E-12, 1.9366E-10, 38725.0784, 4.4887E-12, 1.8241E-10, 35850.423,
    5.6042E-12, 1.7115E-10, 32983.4011, 6.7197E-12, 1.5989E-10, 30127.8296,
    7.8352E-12, 1.4863E-10, 27287.5251, 8.9506E-12, 1.3738E-10, 24466.3045,
    1.0066E-11, 1.2612E-10, 21667.9844, 1.1182E-11, 1.1486E-10, 18896.3818,
    1.2297E-11, 1.0361E-10, 16155.3133, 1.3413E-11, 9.2349E-11, 13448.5957,
    1.4528E-11, 8.1092E-11, 10780.0458, 49834.4808, 3.7294E-11, -1.2214E-11,
    48445.8055, 3.7762E-11, -1.0266E-11, 47057.1302, 3.8231E-11, -8.3176E-12,
    45668.4549, 3.87E-11, -6.3691E-12, 44279.7796, 3.9168E-11, -4.4206E-12,
    42891.1043, 3.9637E-11, -2.4722E-12, 41502.429, 4.0106E-11, -5.2371E-13,
    40113.7537, 4.0574E-11, 1.4248E-12, 38725.0784, 4.1043E-11, 3.3732E-12,
    57336.4031, 4.1512E-11, 5.3217E-12, 34585.5503, 4.198E-11, 7.2701E-12,
    31838.5144, 4.2449E-11, 9.2186E-12, 29099.1119, 4.2918E-11, 1.1167E-11,
    26371.1598, 4.3387E-11, 1.3116E-11, 23658.4747, 4.3855E-11, 1.5064E-11,
    20964.8735, 4.4324E-11, 1.7012E-11, 18294.1729, 4.4793E-11, 1.8961E-11,
    15650.1896, 4.5261E-11, 2.0909E-11, 13036.7405, 4.573E-11, 2.2858E-11,
    10457.6424, 4.6199E-11, 2.4806E-11, 4.3794E-11, 49834.4808, 2.9844E-10,
    4.354E-11, 48445.8055, 2.8534E-10, 4.3287E-11, 47057.1302, 2.7225E-10,
    4.3033E-11, 45668.4549, 2.5915E-10, 4.278E-11, 44279.7796, 2.4605E-10,
    4.2526E-11, 42891.1043, 2.3295E-10, 4.2272E-11, 41502.429, 2.1986E-10,
    4.2019E-11, 40113.7537, 2.0676E-10, 4.1765E-11, 38725.0784, 1.9366E-10,
    4.1512E-11, 57336.4031, 1.8057E-10, 4.1258E-11, 34585.5503, 1.6747E-10,
    4.1005E-11, 31838.5144, 1.5437E-10, 4.0751E-11, 29099.1119, 1.4127E-10,
    4.0498E-11, 26371.1598, 1.2818E-10, 4.0244E-11, 23658.4747, 1.1508E-10,
    3.9991E-11, 20964.8735, 1.0198E-10, 3.9737E-11, 18294.1729, 8.8886E-11,
    3.9483E-11, 15650.1896, 7.5789E-11, 3.923E-11, 13036.7405, 6.2692E-11,
    3.8976E-11, 10457.6424, 4.9595E-11, -3.2214E-12, 2.7501E-10, 49834.4808,
    -2.2722E-12, 2.6452E-10, 48445.8055, -1.323E-12, 2.5402E-10, 47057.1302,
    -3.7372E-13, 2.4353E-10, 45668.4549, 5.7551E-13, 2.3304E-10, 44279.7796,
    1.5247E-12, 2.2254E-10, 42891.1043, 2.474E-12, 2.1205E-10, 41502.429,
    3.4232E-12, 2.0155E-10, 40113.7537, 4.3724E-12, 1.9106E-10, 38725.0784,
    5.3217E-12, 1.8057E-10, 57336.4031, 6.2709E-12, 1.7007E-10, 34585.5503,
    7.2201E-12, 1.5958E-10, 31838.5144, 8.1694E-12, 1.4908E-10, 29099.1119,
    9.1186E-12, 1.3859E-10, 26371.1598, 1.0068E-11, 1.281E-10, 23658.4747,
    1.1017E-11, 1.176E-10, 20964.8735, 1.1966E-11, 1.0711E-10, 18294.1729,
    1.2916E-11, 9.6614E-11, 15650.1896, 1.3865E-11, 8.612E-11, 13036.7405,
    1.4814E-11, 7.5626E-11, 10457.6424, 45969.4042, 3.7784E-11, -9.769E-12,
    44704.5316, 3.817E-11, -7.9868E-12, 43439.6589, 3.8556E-11, -6.2046E-12,
    42174.7863, 3.8942E-11, -4.4223E-12, 40909.9136, 3.9328E-11, -2.6401E-12,
    39645.041, 3.9714E-11, -8.5793E-13, 38380.1683, 4.01E-11, 9.2428E-13,
    37115.2957, 4.0486E-11, 2.7065E-12, 35850.423, 4.0872E-11, 4.4887E-12,
    34585.5503, 4.1258E-11, 6.2709E-12, 53320.6777, 4.1644E-11, 8.0531E-12,
    30693.6276, 4.203E-11, 9.8353E-12, 28070.3942, 4.2416E-11, 1.1618E-11,
    25454.7944, 4.2802E-11, 1.34E-11, 22850.6449, 4.3188E-11, 1.5182E-11,
    20261.7625, 4.3574E-11, 1.6964E-11, 17691.9639, 4.396E-11, 1.8746E-11,
    15145.0659, 4.4346E-11, 2.0529E-11, 12624.8854, 4.4732E-11, 2.2311E-11,
    10135.2389, 4.5118E-11, 2.4093E-11, 4.5007E-11, 45969.4042, 2.8108E-10,
    4.4671E-11, 44704.5316, 2.6874E-10, 4.4334E-11, 43439.6589, 2.5641E-10,
    4.3998E-11, 42174.7863, 2.4408E-10, 4.3662E-11, 40909.9136, 2.3174E-10,
    4.3326E-11, 39645.041, 2.1941E-10, 4.2989E-11, 38380.1683, 2.0707E-10,
    4.2653E-11, 37115.2957, 1.9474E-10, 4.2317E-11, 35850.423, 1.8241E-10,
    4.198E-11, 34585.5503, 1.7007E-10, 4.1644E-11, 53320.6777, 1.5774E-10,
    4.1308E-11, 30693.6276, 1.454E-10, 4.0972E-11, 28070.3942, 1.3307E-10,
    4.0635E-11, 25454.7944, 1.2074E-10, 4.0299E-11, 22850.6449, 1.084E-10,
    3.9963E-11, 20261.7625, 9.6068E-11, 3.9627E-11, 17691.9639, 8.3734E-11,
    3.929E-11, 15145.0659, 7.14E-11, 3.8954E-11, 12624.8854, 5.9066E-11,
    3.8618E-11, 10135.2389, 4.6732E-11, 2.2332E-13, 2.5505E-10, 45969.4042,
    1.0063E-12, 2.4532E-10, 44704.5316, 1.7893E-12, 2.3559E-10, 43439.6589,
    2.5723E-12, 2.2585E-10, 42174.7863, 3.3552E-12, 2.1612E-10, 40909.9136,
    4.1382E-12, 2.0639E-10, 39645.041, 4.9212E-12, 1.9666E-10, 38380.1683,
    5.7042E-12, 1.8693E-10, 37115.2957, 6.4872E-12, 1.772E-10, 35850.423,
    7.2701E-12, 1.6747E-10, 34585.5503, 8.0531E-12, 1.5774E-10, 53320.6777,
    8.8361E-12, 1.4801E-10, 30693.6276, 9.6191E-12, 1.3828E-10, 28070.3942,
    1.0402E-11, 1.2855E-10, 25454.7944, 1.1185E-11, 1.1881E-10, 22850.6449,
    1.1968E-11, 1.0908E-10, 20261.7625, 1.2751E-11, 9.9352E-11, 17691.9639,
    1.3534E-11, 8.9621E-11, 15145.0659, 1.4317E-11, 7.989E-11, 12624.8854,
    1.51E-11, 7.0159E-11, 10135.2389, 42142.4954, 3.8275E-11, -7.3235E-12,
    40997.6086, 3.8578E-11, -5.7075E-12, 39852.7218, 3.8882E-11, -4.0916E-12,
    38707.835, 3.9185E-11, -2.4756E-12, 37562.9483, 3.9488E-11, -8.5964E-13,
    36418.0615, 3.9792E-11, 7.5632E-13, 35273.1747, 4.0095E-11, 2.3723E-12,
    34128.2879, 4.0398E-11, 3.9882E-12, 32983.4011, 4.0701E-11, 5.6042E-12,
    31838.5144, 4.1005E-11, 7.2201E-12, 30693.6276, 4.1308E-11, 8.8361E-12,
    49548.7408, 4.1611E-11, 1.0452E-11, 27041.6766, 4.1915E-11, 1.2068E-11,
    24538.4291, 4.2218E-11, 1.3684E-11, 22042.8152, 4.2521E-11, 1.53E-11,
    19558.6515, 4.2824E-11, 1.6916E-11, 17089.755, 4.3128E-11, 1.8532E-11,
    14639.9423, 4.3431E-11, 2.0148E-11, 12213.0302, 4.3734E-11, 2.1764E-11,
    9812.8355, 4.4037E-11, 2.338E-11, 4.622E-11, 42142.4954, 2.6372E-10,
    4.5801E-11, 40997.6086, 2.5215E-10, 4.5382E-11, 39852.7218, 2.4057E-10,
    4.4963E-11, 38707.835, 2.29E-10, 4.4544E-11, 37562.9483, 2.1743E-10,
    4.4125E-11, 36418.0615, 2.0586E-10, 4.3706E-11, 35273.1747, 1.9429E-10,
    4.3287E-11, 34128.2879, 1.8272E-10, 4.2868E-11, 32983.4011, 1.7115E-10,
    4.2449E-11, 31838.5144, 1.5958E-10, 4.203E-11, 30693.6276, 1.4801E-10,
    4.1611E-11, 49548.7408, 1.3644E-10, 4.1192E-11, 27041.6766, 1.2487E-10,
    4.0773E-11, 24538.4291, 1.1329E-10, 4.0354E-11, 22042.8152, 1.0172E-10,
    3.9935E-11, 19558.6515, 9.0152E-11, 3.9516E-11, 17089.755, 7.8581E-11,
    3.9097E-11, 14639.9423, 6.701E-11, 3.8679E-11, 12213.0302, 5.5439E-11,
    3.826E-11, 9812.8355, 4.3869E-11, 3.6681E-12, 2.3508E-10, 42142.4954,
    4.2848E-12, 2.2611E-10, 40997.6086, 4.9015E-12, 2.1715E-10, 39852.7218,
    5.5182E-12, 2.0818E-10, 38707.835, 6.135E-12, 1.9921E-10, 37562.9483,
    6.7517E-12, 1.9024E-10, 36418.0615, 7.3684E-12, 1.8128E-10, 35273.1747,
    7.9852E-12, 1.7231E-10, 34128.2879, 8.6019E-12, 1.6334E-10, 32983.4011,
    9.2186E-12, 1.5437E-10, 31838.5144, 9.8353E-12, 1.454E-10, 30693.6276,
    1.0452E-11, 1.3644E-10, 49548.7408, 1.1069E-11, 1.2747E-10, 27041.6766,
    1.1686E-11, 1.185E-10, 24538.4291, 1.2302E-11, 1.0953E-10, 22042.8152,
    1.2919E-11, 1.0056E-10, 19558.6515, 1.3536E-11, 9.1597E-11, 17089.755,
    1.4152E-11, 8.2629E-11, 14639.9423, 1.4769E-11, 7.3661E-11, 12213.0302,
    1.5386E-11, 6.4693E-11, 9812.8355, 38357.571, 3.8766E-11, -4.878E-12,
    37328.8533, 3.8987E-11, -3.4283E-12, 36300.1356, 3.9207E-11, -1.9786E-12,
    35271.418, 3.9428E-11, -5.2885E-13, 34242.7003, 3.9648E-11, 9.2086E-13,
    33213.9826, 3.9869E-11, 2.3706E-12, 32185.2649, 4.0089E-11, 3.8203E-12,
    31156.5473, 4.031E-11, 5.27E-12, 30127.8296, 4.0531E-11, 6.7197E-12,
    29099.1119, 4.0751E-11, 8.1694E-12, 28070.3942, 4.0972E-11, 9.6191E-12,
    27041.6766, 4.1192E-11, 1.1069E-11, 46012.9589, 4.1413E-11, 1.2518E-11,
    23622.0637, 4.1633E-11, 1.3968E-11, 21234.9854, 4.1854E-11, 1.5418E-11,
    18855.5406, 4.2075E-11, 1.6868E-11, 16487.546, 4.2295E-11, 1.8317E-11,
    14134.8186, 4.2516E-11, 1.9767E-11, 11801.175, 4.2736E-11, 2.1217E-11,
    9490.432, 4.2957E-11, 2.2666E-11, 4.7433E-11, 38357.571, 2.4635E-10,
    4.6931E-11, 37328.8533, 2.3555E-10, 4.643E-11, 36300.1356, 2.2474E-10,
    4.5928E-11, 35271.418, 2.1393E-10, 4.5426E-11, 34242.7003, 2.0312E-10,
    4.4925E-11, 33213.9826, 1.9232E-10, 4.4423E-11, 32185.2649, 1.8151E-10,
    4.3921E-11, 31156.5473, 1.707E-10, 4.342E-11, 30127.8296, 1.5989E-10,
    4.2918E-11, 29099.1119, 1.4908E-10, 4.2416E-11, 28070.3942, 1.3828E-10,
    4.1915E-11, 27041.6766, 1.2747E-10, 4.1413E-11, 46012.9589, 1.1666E-10,
    4.0911E-11, 23622.0637, 1.0585E-10, 4.041E-11, 21234.9854, 9.5045E-11,
    3.9908E-11, 18855.5406, 8.4237E-11, 3.9406E-11, 16487.546, 7.3429E-11,
    3.8904E-11, 14134.8186, 6.2621E-11, 3.8403E-11, 11801.175, 5.1813E-11,
    3.7901E-11, 9490.432, 4.1005E-11, 7.1128E-12, 2.1512E-10, 38357.571,
    7.5633E-12, 2.0691E-10, 37328.8533, 8.0137E-12, 1.9871E-10, 36300.1356,
    8.4642E-12, 1.905E-10, 35271.418, 8.9147E-12, 1.823E-10, 34242.7003,
    9.3652E-12, 1.7409E-10, 33213.9826, 9.8156E-12, 1.6589E-10, 32185.2649,
    1.0266E-11, 1.5768E-10, 31156.5473, 1.0717E-11, 1.4948E-10, 30127.8296,
    1.1167E-11, 1.4127E-10, 29099.1119, 1.1618E-11, 1.3307E-10, 28070.3942,
    1.2068E-11, 1.2487E-10, 27041.6766, 1.2518E-11, 1.1666E-10, 46012.9589,
    1.2969E-11, 1.0846E-10, 23622.0637, 1.3419E-11, 1.0025E-10, 21234.9854,
    1.387E-11, 9.2046E-11, 18855.5406, 1.432E-11, 8.3841E-11, 16487.546,
    1.4771E-11, 7.5636E-11, 14134.8186, 1.5221E-11, 6.7432E-11, 11801.175,
    1.5672E-11, 5.9227E-11, 9490.432, 34618.4478, 3.9257E-11, -2.4324E-12,
    33702.0825, 3.9395E-11, -1.149E-12, 32785.7171, 3.9533E-11, 1.3445E-13,
    31869.3518, 3.967E-11, 1.4179E-12, 30952.9865, 3.9808E-11, 2.7014E-12,
    30036.6211, 3.9946E-11, 3.9848E-12, 29120.2558, 4.0084E-11, 5.2683E-12,
    28203.8904, 4.0222E-11, 6.5517E-12, 27287.5251, 4.036E-11, 7.8352E-12,
    26371.1598, 4.0498E-11, 9.1186E-12, 25454.7944, 4.0635E-11, 1.0402E-11,
    24538.4291, 4.0773E-11, 1.1686E-11, 23622.0637, 4.0911E-11, 1.2969E-11,
    42705.6984, 4.1049E-11, 1.4252E-11, 20427.1556, 4.1187E-11, 1.5536E-11,
    18152.4296, 4.1325E-11, 1.6819E-11, 15885.3371, 4.1463E-11, 1.8103E-11,
    13629.6949, 4.16E-11, 1.9386E-11, 11389.3198, 4.1738E-11, 2.067E-11,
    9168.0285, 4.1876E-11, 2.1953E-11, 4.8646E-11, 34618.4478, 2.2899E-10,
    4.8062E-11, 33702.0825, 2.1895E-10, 4.7477E-11, 32785.7171, 2.089E-10,
    4.6893E-11, 31869.3518, 1.9886E-10, 4.6308E-11, 30952.9865, 1.8881E-10,
    4.5724E-11, 30036.6211, 1.7877E-10, 4.514E-11, 29120.2558, 1.6872E-10,
    4.4555E-11, 28203.8904, 1.5868E-10, 4.3971E-11, 27287.5251, 1.4863E-10,
    4.3387E-11, 26371.1598, 1.3859E-10, 4.2802E-11, 25454.7944, 1.2855E-10,
    4.2218E-11, 24538.4291, 1.185E-10, 4.1633E-11, 23622.0637, 1.0846E-10,
    4.1049E-11, 42705.6984, 9.8411E-11, 4.0465E-11, 20427.1556, 8.8366E-11,
    3.988E-11, 18152.4296, 7.8321E-11, 3.9296E-11, 15885.3371, 6.8276E-11,
    3.8712E-11, 13629.6949, 5.8232E-11, 3.8127E-11, 11389.3198, 4.8187E-11,
    3.7543E-11, 9168.0285, 3.8142E-11, 1.0558E-11, 1.9515E-10, 34618.4478,
    1.0842E-11, 1.8771E-10, 33702.0825, 1.1126E-11, 1.8027E-10, 32785.7171,
    1.141E-11, 1.7283E-10, 31869.3518, 1.1694E-11, 1.6539E-10, 30952.9865,
    1.1979E-11, 1.5794E-10, 30036.6211, 1.2263E-11, 1.505E-10, 29120.2558,
    1.2547E-11, 1.4306E-10, 28203.8904, 1.2831E-11, 1.3562E-10, 27287.5251,
    1.3116E-11, 1.2818E-10, 26371.1598, 1.34E-11, 1.2074E-10, 25454.7944,
    1.3684E-11, 1.1329E-10, 24538.4291, 1.3968E-11, 1.0585E-10, 23622.0637,
    1.4252E-11, 9.8411E-11, 42705.6984, 1.4537E-11, 9.0969E-11, 20427.1556,
    1.4821E-11, 8.3527E-11, 18152.4296, 1.5105E-11, 7.6086E-11, 15885.3371,
    1.5389E-11, 6.8644E-11, 13629.6949, 1.5674E-11, 6.1202E-11, 11389.3198,
    1.5958E-11, 5.376E-11, 9168.0285, 30928.9426, 3.9748E-11, 1.306E-14,
    30121.1129, 3.9803E-11, 1.1303E-12, 29313.2831, 3.9858E-11, 2.2475E-12,
    28505.4533, 3.9913E-11, 3.3647E-12, 27697.6235, 3.9968E-11, 4.4819E-12,
    26889.7938, 4.0023E-11, 5.5991E-12, 26081.964, 4.0079E-11, 6.7162E-12,
    25274.1342, 4.0134E-11, 7.8334E-12, 24466.3045, 4.0189E-11, 8.9506E-12,
    23658.4747, 4.0244E-11, 1.0068E-11, 22850.6449, 4.0299E-11, 1.1185E-11,
    22042.8152, 4.0354E-11, 1.2302E-11, 21234.9854, 4.041E-11, 1.3419E-11,
    20427.1556, 4.0465E-11, 1.4537E-11, 39619.3258, 4.052E-11, 1.5654E-11,
    17449.3186, 4.0575E-11, 1.6771E-11, 15283.1282, 4.063E-11, 1.7888E-11,
    13124.5712, 4.0685E-11, 1.9005E-11, 10977.4646, 4.074E-11, 2.0123E-11,
    8845.6251, 4.0796E-11, 2.124E-11, 4.9859E-11, 30928.9426, 2.1163E-10,
    4.9192E-11, 30121.1129, 2.0235E-10, 4.8525E-11, 29313.2831, 1.9307E-10,
    4.7858E-11, 28505.4533, 1.8379E-10, 4.7191E-11, 27697.6235, 1.745E-10,
    4.6524E-11, 26889.7938, 1.6522E-10, 4.5857E-11, 26081.964, 1.5594E-10,
    4.5189E-11, 25274.1342, 1.4666E-10, 4.4522E-11, 24466.3045, 1.3738E-10,
    4.3855E-11, 23658.4747, 1.281E-10, 4.3188E-11, 22850.6449, 1.1881E-10,
    4.2521E-11, 22042.8152, 1.0953E-10, 4.1854E-11, 21234.9854, 1.0025E-10,
    4.1187E-11, 20427.1556, 9.0969E-11, 4.052E-11, 39619.3258, 8.1687E-11,
    3.9853E-11, 17449.3186, 7.2406E-11, 3.9186E-11, 15283.1282, 6.3124E-11,
    3.8519E-11, 13124.5712, 5.3842E-11, 3.7851E-11, 10977.4646, 4.4561E-11,
    3.7184E-11, 8845.6251, 3.5279E-11, 1.4002E-11, 1.7519E-10, 30928.9426,
    1.412E-11, 1.6851E-10, 30121.1129, 1.4238E-11, 1.6183E-10, 29313.2831,
    1.4356E-11, 1.5515E-10, 28505.4533, 1.4474E-11, 1.4847E-10, 27697.6235,
    1.4592E-11, 1.418E-10, 26889.7938, 1.471E-11, 1.3512E-10, 26081.964,
    1.4828E-11, 1.2844E-10, 25274.1342, 1.4946E-11, 1.2176E-10, 24466.3045,
    1.5064E-11, 1.1508E-10, 23658.4747, 1.5182E-11, 1.084E-10, 22850.6449,
    1.53E-11, 1.0172E-10, 22042.8152, 1.5418E-11, 9.5045E-11, 21234.9854,
    1.5536E-11, 8.8366E-11, 20427.1556, 1.5654E-11, 8.1687E-11, 39619.3258,
    1.5772E-11, 7.5009E-11, 17449.3186, 1.589E-11, 6.833E-11, 15283.1282,
    1.6008E-11, 6.1651E-11, 13124.5712, 1.6126E-11, 5.4973E-11, 10977.4646,
    1.6244E-11, 4.8294E-11, 8845.6251, 27292.8722, 4.0239E-11, 2.4586E-12,
    26589.7612, 4.0211E-11, 3.4095E-12, 25886.6503, 4.0183E-11, 4.3605E-12,
    25183.5393, 4.0156E-11, 5.3114E-12, 24480.4283, 4.0128E-11, 6.2624E-12,
    23777.3174, 4.0101E-11, 7.2133E-12, 23074.2064, 4.0073E-11, 8.1642E-12,
    22371.0954, 4.0046E-11, 9.1152E-12, 21667.9844, 4.0018E-11, 1.0066E-11,
    20964.8735, 3.9991E-11, 1.1017E-11, 20261.7625, 3.9963E-11, 1.1968E-11,
    19558.6515, 3.9935E-11, 1.2919E-11, 18855.5406, 3.9908E-11, 1.387E-11,
    18152.4296, 3.988E-11, 1.4821E-11, 17449.3186, 3.9853E-11, 1.5772E-11,
    36746.2076, 3.9825E-11, 1.6723E-11, 14680.9192, 3.9798E-11, 1.7674E-11,
    12619.4476, 3.977E-11, 1.8625E-11, 10565.6094, 3.9742E-11, 1.9576E-11,
    8523.2216, 3.9715E-11, 2.0527E-11, 5.1072E-11, 27292.8722, 1.9427E-10,
    5.0322E-11, 26589.7612, 1.8575E-10, 4.9573E-11, 25886.6503, 1.7723E-10,
    4.8823E-11, 25183.5393, 1.6871E-10, 4.8073E-11, 24480.4283, 1.6019E-10,
    4.7323E-11, 23777.3174, 1.5168E-10, 4.6573E-11, 23074.2064, 1.4316E-10,
    4.5824E-11, 22371.0954, 1.3464E-10, 4.5074E-11, 21667.9844, 1.2612E-10,
    4.4324E-11, 20964.8735, 1.176E-10, 4.3574E-11, 20261.7625, 1.0908E-10,
    4.2824E-11, 19558.6515, 1.0056E-10, 4.2075E-11, 18855.5406, 9.2046E-11,
    4.1325E-11, 18152.4296, 8.3527E-11, 4.0575E-11, 17449.3186, 7.5009E-11,
    3.9825E-11, 36746.2076, 6.649E-11, 3.9075E-11, 14680.9192, 5.7971E-11,
    3.8326E-11, 12619.4476, 4.9453E-11, 3.7576E-11, 10565.6094, 4.0934E-11,
    3.6826E-11, 8523.2216, 3.2416E-11, 1.7447E-11, 1.5522E-10, 27292.8722,
    1.7399E-11, 1.4931E-10, 26589.7612, 1.735E-11, 1.4339E-10, 25886.6503,
    1.7302E-11, 1.3748E-10, 25183.5393, 1.7254E-11, 1.3156E-10, 24480.4283,
    1.7206E-11, 1.2565E-10, 23777.3174, 1.7157E-11, 1.1973E-10, 23074.2064,
    1.7109E-11, 1.1381E-10, 22371.0954, 1.7061E-11, 1.079E-10, 21667.9844,
    1.7012E-11, 1.0198E-10, 20964.8735, 1.6964E-11, 9.6068E-11, 20261.7625,
    1.6916E-11, 9.0152E-11, 19558.6515, 1.6868E-11, 8.4237E-11, 18855.5406,
    1.6819E-11, 7.8321E-11, 18152.4296, 1.6771E-11, 7.2406E-11, 17449.3186,
    1.6723E-11, 6.649E-11, 36746.2076, 1.6674E-11, 6.0575E-11, 14680.9192,
    1.6626E-11, 5.4659E-11, 12619.4476, 1.6578E-11, 4.8743E-11, 10565.6094,
    1.653E-11, 4.2828E-11, 8523.2216, 23714.0533, 4.0729E-11, 4.9041E-12,
    23111.8444, 4.0619E-11, 5.6888E-12, 22509.6354, 4.0509E-11, 6.4735E-12,
    21907.4265, 4.0399E-11, 7.2582E-12, 21305.2176, 4.0288E-11, 8.0428E-12,
    20703.0086, 4.0178E-11, 8.8275E-12, 20100.7997, 4.0068E-11, 9.6122E-12,
    19498.5907, 3.9958E-11, 1.0397E-11, 18896.3818, 3.9847E-11, 1.1182E-11,
    18294.1729, 3.9737E-11, 1.1966E-11, 17691.9639, 3.9627E-11, 1.2751E-11,
    17089.755, 3.9516E-11, 1.3536E-11, 16487.546, 3.9406E-11, 1.432E-11,
    15885.3371, 3.9296E-11, 1.5105E-11, 15283.1282, 3.9186E-11, 1.589E-11,
    14680.9192, 3.9075E-11, 1.6674E-11, 34078.7103, 3.8965E-11, 1.7459E-11,
    12114.3239, 3.8855E-11, 1.8244E-11, 10153.7542, 3.8745E-11, 1.9029E-11,
    8200.8182, 3.8634E-11, 1.9813E-11, 5.2285E-11, 23714.0533, 1.7691E-10,
    5.1453E-11, 23111.8444, 1.6915E-10, 5.062E-11, 22509.6354, 1.614E-10,
    4.9788E-11, 21907.4265, 1.5364E-10, 4.8955E-11, 21305.2176, 1.4589E-10,
    4.8123E-11, 20703.0086, 1.3813E-10, 4.729E-11, 20100.7997, 1.3037E-10,
    4.6458E-11, 19498.5907, 1.2262E-10, 4.5625E-11, 18896.3818, 1.1486E-10,
    4.4793E-11, 18294.1729, 1.0711E-10, 4.396E-11, 17691.9639, 9.9352E-11,
    4.3128E-11, 17089.755, 9.1597E-11, 4.2295E-11, 16487.546, 8.3841E-11,
    4.1463E-11, 15885.3371, 7.6086E-11, 4.063E-11, 15283.1282, 6.833E-11,
    3.9798E-11, 14680.9192, 6.0575E-11, 3.8965E-11, 34078.7103, 5.2819E-11,
    3.8133E-11, 12114.3239, 4.5063E-11, 3.73E-11, 10153.7542, 3.7308E-11,
    3.6468E-11, 8200.8182, 2.9552E-11, 2.0892E-11, 1.3526E-10, 23714.0533,
    2.0677E-11, 1.3011E-10, 23111.8444, 2.0463E-11, 1.2495E-10, 22509.6354,
    2.0248E-11, 1.198E-10, 21907.4265, 2.0034E-11, 1.1465E-10, 21305.2176,
    1.9819E-11, 1.095E-10, 20703.0086, 1.9605E-11, 1.0434E-10, 20100.7997,
    1.939E-11, 9.9191E-11, 19498.5907, 1.9175E-11, 9.4039E-11, 18896.3818,
    1.8961E-11, 8.8886E-11, 18294.1729, 1.8746E-11, 8.3734E-11, 17691.9639,
    1.8532E-11, 7.8581E-11, 17089.755, 1.8317E-11, 7.3429E-11, 16487.546,
    1.8103E-11, 6.8276E-11, 15885.3371, 1.7888E-11, 6.3124E-11, 15283.1282,
    1.7674E-11, 5.7971E-11, 14680.9192, 1.7459E-11, 5.2819E-11, 34078.7103,
    1.7245E-11, 4.7667E-11, 12114.3239, 1.703E-11, 4.2514E-11, 10153.7542,
    1.6816E-11, 3.7362E-11, 8200.8182, 20196.3027, 4.122E-11, 7.3496E-12,
    19691.179, 4.1027E-11, 7.968E-12, 19186.0554, 4.0834E-11, 8.5865E-12,
    18680.9317, 4.0641E-11, 9.2049E-12, 18175.808, 4.0448E-11, 9.8233E-12,
    17670.6843, 4.0255E-11, 1.0442E-11, 17165.5607, 4.0062E-11, 1.106E-11,
    16660.437, 3.9869E-11, 1.1679E-11, 16155.3133, 3.9676E-11, 1.2297E-11,
    15650.1896, 3.9483E-11, 1.2916E-11, 15145.0659, 3.929E-11, 1.3534E-11,
    14639.9423, 3.9097E-11, 1.4152E-11, 14134.8186, 3.8904E-11, 1.4771E-11,
    13629.6949, 3.8712E-11, 1.5389E-11, 13124.5712, 3.8519E-11, 1.6008E-11,
    12619.4476, 3.8326E-11, 1.6626E-11, 12114.3239, 3.8133E-11, 1.7245E-11,
    31609.2002, 3.794E-11, 1.7863E-11, 9741.8991, 3.7747E-11, 1.8481E-11,
    7878.4147, 3.7554E-11, 1.91E-11, 5.3498E-11, 20196.3027, 1.5955E-10,
    5.2583E-11, 19691.179, 1.5255E-10, 5.1668E-11, 19186.0554, 1.4556E-10,
    5.0753E-11, 18680.9317, 1.3857E-10, 4.9837E-11, 18175.808, 1.3158E-10,
    4.8922E-11, 17670.6843, 1.2458E-10, 4.8007E-11, 17165.5607, 1.1759E-10,
    4.7092E-11, 16660.437, 1.106E-10, 4.6177E-11, 16155.3133, 1.0361E-10,
    4.5261E-11, 15650.1896, 9.6614E-11, 4.4346E-11, 15145.0659, 8.9621E-11,
    4.3431E-11, 14639.9423, 8.2629E-11, 4.2516E-11, 14134.8186, 7.5636E-11,
    4.16E-11, 13629.6949, 6.8644E-11, 4.0685E-11, 13124.5712, 6.1651E-11,
    3.977E-11, 12619.4476, 5.4659E-11, 3.8855E-11, 12114.3239, 4.7667E-11,
    3.794E-11, 31609.2002, 4.0674E-11, 3.7024E-11, 9741.8991, 3.3682E-11,
    3.6109E-11, 7878.4147, 2.6689E-11, 2.4337E-11, 1.1529E-10, 20196.3027,
    2.3956E-11, 1.109E-10, 19691.179, 2.3575E-11, 1.0651E-10, 19186.0554,
    2.3194E-11, 1.0213E-10, 18680.9317, 2.2813E-11, 9.7736E-11, 18175.808,
    2.2433E-11, 9.3347E-11, 17670.6843, 2.2052E-11, 8.8957E-11, 17165.5607,
    2.1671E-11, 8.4568E-11, 16660.437, 2.129E-11, 8.0179E-11, 16155.3133,
    2.0909E-11, 7.5789E-11, 15650.1896, 2.0529E-11, 7.14E-11, 15145.0659,
    2.0148E-11, 6.701E-11, 14639.9423, 1.9767E-11, 6.2621E-11, 14134.8186,
    1.9386E-11, 5.8232E-11, 13629.6949, 1.9005E-11, 5.3842E-11, 13124.5712,
    1.8625E-11, 4.9453E-11, 12619.4476, 1.8244E-11, 4.5063E-11, 12114.3239,
    1.7863E-11, 4.0674E-11, 31609.2002, 1.7482E-11, 3.6285E-11, 9741.8991,
    1.7101E-11, 3.1895E-11, 7878.4147, 16743.4372, 4.1711E-11, 9.7951E-12,
    16331.582, 4.1435E-11, 1.0247E-11, 15919.7268, 4.116E-11, 1.0699E-11,
    15507.8716, 4.0884E-11, 1.1152E-11, 15096.0165, 4.0608E-11, 1.1604E-11,
    14684.1613, 4.0333E-11, 1.2056E-11, 14272.3061, 4.0057E-11, 1.2508E-11,
    13860.4509, 3.9781E-11, 1.296E-11, 13448.5957, 3.9506E-11, 1.3413E-11,
    13036.7405, 3.923E-11, 1.3865E-11, 12624.8854, 3.8954E-11, 1.4317E-11,
    12213.0302, 3.8679E-11, 1.4769E-11, 11801.175, 3.8403E-11, 1.5221E-11,
    11389.3198, 3.8127E-11, 1.5674E-11, 10977.4646, 3.7851E-11, 1.6126E-11,
    10565.6094, 3.7576E-11, 1.6578E-11, 10153.7542, 3.73E-11, 1.703E-11,
    9741.8991, 3.7024E-11, 1.7482E-11, 29330.0439, 3.6749E-11, 1.7934E-11,
    7556.0112, 3.6473E-11, 1.8387E-11, 5.4711E-11, 16743.4372, 1.4218E-10,
    5.3713E-11, 16331.582, 1.3595E-10, 5.2716E-11, 15919.7268, 1.2973E-10,
    5.1718E-11, 15507.8716, 1.235E-10, 5.072E-11, 15096.0165, 1.1727E-10,
    4.9722E-11, 14684.1613, 1.1104E-10, 4.8724E-11, 14272.3061, 1.0481E-10,
    4.7726E-11, 13860.4509, 9.8578E-11, 4.6728E-11, 13448.5957, 9.2349E-11,
    4.573E-11, 13036.7405, 8.612E-11, 4.4732E-11, 12624.8854, 7.989E-11,
    4.3734E-11, 12213.0302, 7.3661E-11, 4.2736E-11, 11801.175, 6.7432E-11,
    4.1738E-11, 11389.3198, 6.1202E-11, 4.074E-11, 10977.4646, 5.4973E-11,
    3.9742E-11, 10565.6094, 4.8743E-11, 3.8745E-11, 10153.7542, 4.2514E-11,
    3.7747E-11, 9741.8991, 3.6285E-11, 3.6749E-11, 29330.0439, 3.0055E-11,
    3.5751E-11, 7556.0112, 2.3826E-11, 2.7781E-11, 9.5329E-11, 16743.4372,
    2.7234E-11, 9.1703E-11, 16331.582, 2.6687E-11, 8.8076E-11, 15919.7268,
    2.614E-11, 8.445E-11, 15507.8716, 2.5593E-11, 8.0824E-11, 15096.0165,
    2.5046E-11, 7.7197E-11, 14684.1613, 2.4499E-11, 7.3571E-11, 14272.3061,
    2.3952E-11, 6.9945E-11, 13860.4509, 2.3405E-11, 6.6318E-11, 13448.5957,
    2.2858E-11, 6.2692E-11, 13036.7405, 2.2311E-11, 5.9066E-11, 12624.8854,
    2.1764E-11, 5.5439E-11, 12213.0302, 2.1217E-11, 5.1813E-11, 11801.175,
    2.067E-11, 4.8187E-11, 11389.3198, 2.0123E-11, 4.4561E-11, 10977.4646,
    1.9576E-11, 4.0934E-11, 10565.6094, 1.9029E-11, 3.7308E-11, 10153.7542,
    1.8481E-11, 3.3682E-11, 9741.8991, 1.7934E-11, 3.0055E-11, 29330.0439,
    1.7387E-11, 2.6429E-11, 7556.0112, 13359.2735, 4.2202E-11, 1.2241E-11,
    13036.87, 4.1844E-11, 1.2527E-11, 12714.4666, 4.1485E-11, 1.2812E-11,
    12392.0631, 4.1127E-11, 1.3098E-11, 12069.6597, 4.0768E-11, 1.3384E-11,
    11747.2562, 4.041E-11, 1.367E-11, 11424.8527, 4.0052E-11, 1.3956E-11,
    11102.4493, 3.9693E-11, 1.4242E-11, 10780.0458, 3.9335E-11, 1.4528E-11,
    10457.6424, 3.8976E-11, 1.4814E-11, 10135.2389, 3.8618E-11, 1.51E-11,
    9812.8355, 3.826E-11, 1.5386E-11, 9490.432, 3.7901E-11, 1.5672E-11,
    9168.0285, 3.7543E-11, 1.5958E-11, 8845.6251, 3.7184E-11, 1.6244E-11,
    8523.2216, 3.6826E-11, 1.653E-11, 8200.8182, 3.6468E-11, 1.6816E-11,
    7878.4147, 3.6109E-11, 1.7101E-11, 7556.0112, 3.5751E-11, 1.7387E-11,
    27233.6078, 3.5392E-11, 1.7673E-11, 5.5925E-11, 13359.2735, 1.2482E-10,
    5.4844E-11, 13036.87, 1.1936E-10, 5.3763E-11, 12714.4666, 1.1389E-10,
    5.2683E-11, 12392.0631, 1.0842E-10, 5.1602E-11, 12069.6597, 1.0296E-10,
    5.0521E-11, 11747.2562, 9.7491E-11, 4.9441E-11, 11424.8527, 9.2025E-11,
    4.836E-11, 11102.4493, 8.6558E-11, 4.7279E-11, 10780.0458, 8.1092E-11,
    4.6199E-11, 10457.6424, 7.5626E-11, 4.5118E-11, 10135.2389, 7.0159E-11,
    4.4037E-11, 9812.8355, 6.4693E-11, 4.2957E-11, 9490.432, 5.9227E-11,
    4.1876E-11, 9168.0285, 5.376E-11, 4.0796E-11, 8845.6251, 4.8294E-11,
    3.9715E-11, 8523.2216, 4.2828E-11, 3.8634E-11, 8200.8182, 3.7362E-11,
    3.7554E-11, 7878.4147, 3.1895E-11, 3.6473E-11, 7556.0112, 2.6429E-11,
    3.5392E-11, 27233.6078, 2.0963E-11, 3.1226E-11, 7.5364E-11, 13359.2735,
    3.0513E-11, 7.2501E-11, 13036.87, 2.9799E-11, 6.9638E-11, 12714.4666,
    2.9086E-11, 6.6774E-11, 12392.0631, 2.8373E-11, 6.3911E-11, 12069.6597,
    2.7659E-11, 6.1048E-11, 11747.2562, 2.6946E-11, 5.8185E-11, 11424.8527,
    2.6233E-11, 5.5321E-11, 11102.4493, 2.552E-11, 5.2458E-11, 10780.0458,
    2.4806E-11, 4.9595E-11, 10457.6424, 2.4093E-11, 4.6732E-11, 10135.2389,
    2.338E-11, 4.3869E-11, 9812.8355, 2.2666E-11, 4.1005E-11, 9490.432,
    2.1953E-11, 3.8142E-11, 9168.0285, 2.124E-11, 3.5279E-11, 8845.6251,
    2.0527E-11, 3.2416E-11, 8523.2216, 1.9813E-11, 2.9552E-11, 8200.8182,
    1.91E-11, 2.6689E-11, 7878.4147, 1.8387E-11, 2.3826E-11, 7556.0112,
    1.7673E-11, 2.0963E-11, 27233.6078 };

  static const real_T b_a[3600] = { -105707.9487, -3.2876E-11, 3.4224E-11,
    -81671.1176, -3.4089E-11, 3.0779E-11, -77638.1032, -3.5302E-11, 2.7335E-11,
    -73612.7224, -3.6515E-11, 2.389E-11, -69598.7919, -3.7728E-11, 2.0445E-11,
    -65600.1284, -3.8941E-11, 1.7E-11, -61620.5488, -4.0154E-11, 1.3556E-11,
    -57663.8698, -4.1368E-11, 1.0111E-11, -53733.9082, -4.2581E-11, 6.6662E-12,
    -49834.4808, -4.3794E-11, 3.2214E-12, -45969.4042, -4.5007E-11, -2.2332E-13,
    -42142.4954, -4.622E-11, -3.6681E-12, -38357.571, -4.7433E-11, -7.1128E-12,
    -34618.4478, -4.8646E-11, -1.0558E-11, -30928.9426, -4.9859E-11, -1.4002E-11,
    -27292.8722, -5.1072E-11, -1.7447E-11, -23714.0533, -5.2285E-11, -2.0892E-11,
    -20196.3027, -5.3498E-11, -2.4337E-11, -16743.4372, -5.4711E-11, -2.7781E-11,
    -13359.2735, -5.5925E-11, -3.1226E-11, -3.2876E-11, -105707.9487, -4.547E-10,
    -3.3367E-11, -81671.1176, -4.3473E-10, -3.3858E-11, -77638.1032, -4.1477E-10,
    -3.4348E-11, -73612.7224, -3.948E-10, -3.4839E-11, -69598.7919, -3.7484E-10,
    -3.533E-11, -65600.1284, -3.5487E-10, -3.5821E-11, -61620.5488, -3.3491E-10,
    -3.6312E-11, -57663.8698, -3.1494E-10, -3.6803E-11, -53733.9082, -2.9498E-10,
    -3.7294E-11, -49834.4808, -2.7501E-10, -3.7784E-11, -45969.4042, -2.5505E-10,
    -3.8275E-11, -42142.4954, -2.3508E-10, -3.8766E-11, -38357.571, -2.1512E-10,
    -3.9257E-11, -34618.4478, -1.9515E-10, -3.9748E-11, -30928.9426, -1.7519E-10,
    -4.0239E-11, -27292.8722, -1.5522E-10, -4.0729E-11, -23714.0533, -1.3526E-10,
    -4.122E-11, -20196.3027, -1.1529E-10, -4.1711E-11, -16743.4372, -9.5329E-11,
    -4.2202E-11, -13359.2735, -7.5364E-11, 3.4224E-11, -4.547E-10, -105707.9487,
    3.1779E-11, -4.3733E-10, -81671.1176, 2.9333E-11, -4.1997E-10, -77638.1032,
    2.6888E-11, -4.0261E-10, -73612.7224, 2.4442E-11, -3.8525E-10, -69598.7919,
    2.1997E-11, -3.6789E-10, -65600.1284, 1.9551E-11, -3.5053E-10, -61620.5488,
    1.7106E-11, -3.3316E-10, -57663.8698, 1.466E-11, -3.158E-10, -53733.9082,
    1.2214E-11, -2.9844E-10, -49834.4808, 9.769E-12, -2.8108E-10, -45969.4042,
    7.3235E-12, -2.6372E-10, -42142.4954, 4.878E-12, -2.4635E-10, -38357.571,
    2.4324E-12, -2.2899E-10, -34618.4478, -1.306E-14, -2.1163E-10, -30928.9426,
    -2.4586E-12, -1.9427E-10, -27292.8722, -4.9041E-12, -1.7691E-10, -23714.0533,
    -7.3496E-12, -1.5955E-10, -20196.3027, -9.7951E-12, -1.4218E-10, -16743.4372,
    -1.2241E-11, -1.2482E-10, -13359.2735, -81671.1176, -3.3367E-11, 3.1779E-11,
    -99154.6174, -3.4497E-11, 2.85E-11, -75275.9399, -3.5628E-11, 2.5222E-11,
    -71401.0791, -3.6758E-11, 2.1943E-11, -67533.8518, -3.7888E-11, 1.8665E-11,
    -63678.0749, -3.9019E-11, 1.5386E-11, -59837.565, -4.0149E-11, 1.2108E-11,
    -56016.1389, -4.1279E-11, 8.8292E-12, -52217.6135, -4.241E-11, 5.5507E-12,
    -48445.8055, -4.354E-11, 2.2722E-12, -44704.5316, -4.4671E-11, -1.0063E-12,
    -40997.6086, -4.5801E-11, -4.2848E-12, -37328.8533, -4.6931E-11, -7.5633E-12,
    -33702.0825, -4.8062E-11, -1.0842E-11, -30121.1129, -4.9192E-11, -1.412E-11,
    -26589.7612, -5.0322E-11, -1.7399E-11, -23111.8444, -5.1453E-11, -2.0677E-11,
    -19691.179, -5.2583E-11, -2.3956E-11, -16331.582, -5.3713E-11, -2.7234E-11,
    -13036.87, -5.4844E-11, -3.0513E-11, -3.4089E-11, -81671.1176, -4.3733E-10,
    -3.4497E-11, -99154.6174, -4.1813E-10, -3.4905E-11, -75275.9399, -3.9893E-10,
    -3.5313E-11, -71401.0791, -3.7973E-10, -3.5722E-11, -67533.8518, -3.6053E-10,
    -3.613E-11, -63678.0749, -3.4133E-10, -3.6538E-11, -59837.565, -3.2212E-10,
    -3.6946E-11, -56016.1389, -3.0292E-10, -3.7354E-11, -52217.6135, -2.8372E-10,
    -3.7762E-11, -48445.8055, -2.6452E-10, -3.817E-11, -44704.5316, -2.4532E-10,
    -3.8578E-11, -40997.6086, -2.2611E-10, -3.8987E-11, -37328.8533, -2.0691E-10,
    -3.9395E-11, -33702.0825, -1.8771E-10, -3.9803E-11, -30121.1129, -1.6851E-10,
    -4.0211E-11, -26589.7612, -1.4931E-10, -4.0619E-11, -23111.8444, -1.3011E-10,
    -4.1027E-11, -19691.179, -1.109E-10, -4.1435E-11, -16331.582, -9.1703E-11,
    -4.1844E-11, -13036.87, -7.2501E-11, 3.0779E-11, -4.3473E-10, -81671.1176,
    2.85E-11, -4.1813E-10, -99154.6174, 2.6221E-11, -4.0153E-10, -75275.9399,
    2.3942E-11, -3.8494E-10, -71401.0791, 2.1662E-11, -3.6834E-10, -67533.8518,
    1.9383E-11, -3.5174E-10, -63678.0749, 1.7104E-11, -3.3514E-10, -59837.565,
    1.4825E-11, -3.1854E-10, -56016.1389, 1.2545E-11, -3.0194E-10, -52217.6135,
    1.0266E-11, -2.8534E-10, -48445.8055, 7.9868E-12, -2.6874E-10, -44704.5316,
    5.7075E-12, -2.5215E-10, -40997.6086, 3.4283E-12, -2.3555E-10, -37328.8533,
    1.149E-12, -2.1895E-10, -33702.0825, -1.1303E-12, -2.0235E-10, -30121.1129,
    -3.4095E-12, -1.8575E-10, -26589.7612, -5.6888E-12, -1.6915E-10, -23111.8444,
    -7.968E-12, -1.5255E-10, -19691.179, -1.0247E-11, -1.3595E-10, -16331.582,
    -1.2527E-11, -1.1936E-10, -13036.87, -77638.1032, -3.3858E-11, 2.9333E-11,
    -75275.9399, -3.4905E-11, 2.6221E-11, -92913.7766, -3.5953E-11, 2.3109E-11,
    -69189.4358, -3.7001E-11, 1.9996E-11, -65468.9118, -3.8048E-11, 1.6884E-11,
    -61756.0213, -3.9096E-11, 1.3772E-11, -58054.5811, -4.0144E-11, 1.066E-11,
    -54368.4081, -4.1191E-11, 7.5474E-12, -50701.3188, -4.2239E-11, 4.4352E-12,
    -47057.1302, -4.3287E-11, 1.323E-12, -43439.6589, -4.4334E-11, -1.7893E-12,
    -39852.7218, -4.5382E-11, -4.9015E-12, -36300.1356, -4.643E-11, -8.0137E-12,
    -32785.7171, -4.7477E-11, -1.1126E-11, -29313.2831, -4.8525E-11, -1.4238E-11,
    -25886.6503, -4.9573E-11, -1.735E-11, -22509.6354, -5.062E-11, -2.0463E-11,
    -19186.0554, -5.1668E-11, -2.3575E-11, -15919.7268, -5.2716E-11, -2.6687E-11,
    -12714.4666, -5.3763E-11, -2.9799E-11, -3.5302E-11, -77638.1032, -4.1997E-10,
    -3.5628E-11, -75275.9399, -4.0153E-10, -3.5953E-11, -92913.7766, -3.831E-10,
    -3.6278E-11, -69189.4358, -3.6466E-10, -3.6604E-11, -65468.9118, -3.4622E-10,
    -3.6929E-11, -61756.0213, -3.2778E-10, -3.7255E-11, -58054.5811, -3.0934E-10,
    -3.758E-11, -54368.4081, -2.909E-10, -3.7905E-11, -50701.3188, -2.7246E-10,
    -3.8231E-11, -47057.1302, -2.5402E-10, -3.8556E-11, -43439.6589, -2.3559E-10,
    -3.8882E-11, -39852.7218, -2.1715E-10, -3.9207E-11, -36300.1356, -1.9871E-10,
    -3.9533E-11, -32785.7171, -1.8027E-10, -3.9858E-11, -29313.2831, -1.6183E-10,
    -4.0183E-11, -25886.6503, -1.4339E-10, -4.0509E-11, -22509.6354, -1.2495E-10,
    -4.0834E-11, -19186.0554, -1.0651E-10, -4.116E-11, -15919.7268, -8.8076E-11,
    -4.1485E-11, -12714.4666, -6.9638E-11, 2.7335E-11, -4.1477E-10, -77638.1032,
    2.5222E-11, -3.9893E-10, -75275.9399, 2.3109E-11, -3.831E-10, -92913.7766,
    2.0996E-11, -3.6726E-10, -69189.4358, 1.8883E-11, -3.5142E-10, -65468.9118,
    1.677E-11, -3.3559E-10, -61756.0213, 1.4657E-11, -3.1975E-10, -58054.5811,
    1.2544E-11, -3.0392E-10, -54368.4081, 1.0431E-11, -2.8808E-10, -50701.3188,
    8.3176E-12, -2.7225E-10, -47057.1302, 6.2046E-12, -2.5641E-10, -43439.6589,
    4.0916E-12, -2.4057E-10, -39852.7218, 1.9786E-12, -2.2474E-10, -36300.1356,
    -1.3445E-13, -2.089E-10, -32785.7171, -2.2475E-12, -1.9307E-10, -29313.2831,
    -4.3605E-12, -1.7723E-10, -25886.6503, -6.4735E-12, -1.614E-10, -22509.6354,
    -8.5865E-12, -1.4556E-10, -19186.0554, -1.0699E-11, -1.2973E-10, -15919.7268,
    -1.2812E-11, -1.1389E-10, -12714.4666, -73612.7224, -3.4348E-11, 2.6888E-11,
    -71401.0791, -3.5313E-11, 2.3942E-11, -69189.4358, -3.6278E-11, 2.0996E-11,
    -86977.7925, -3.7243E-11, 1.805E-11, -63403.9717, -3.8208E-11, 1.5104E-11,
    -59833.9678, -3.9173E-11, 1.2158E-11, -56271.5973, -4.0138E-11, 9.2117E-12,
    -52720.6772, -4.1103E-11, 6.2657E-12, -49185.0241, -4.2068E-11, 3.3197E-12,
    -45668.4549, -4.3033E-11, 3.7372E-13, -42174.7863, -4.3998E-11, -2.5723E-12,
    -38707.835, -4.4963E-11, -5.5182E-12, -35271.418, -4.5928E-11, -8.4642E-12,
    -31869.3518, -4.6893E-11, -1.141E-11, -28505.4533, -4.7858E-11, -1.4356E-11,
    -25183.5393, -4.8823E-11, -1.7302E-11, -21907.4265, -4.9788E-11, -2.0248E-11,
    -18680.9317, -5.0753E-11, -2.3194E-11, -15507.8716, -5.1718E-11, -2.614E-11,
    -12392.0631, -5.2683E-11, -2.9086E-11, -3.6515E-11, -73612.7224, -4.0261E-10,
    -3.6758E-11, -71401.0791, -3.8494E-10, -3.7001E-11, -69189.4358, -3.6726E-10,
    -3.7243E-11, -86977.7925, -3.4958E-10, -3.7486E-11, -63403.9717, -3.3191E-10,
    -3.7729E-11, -59833.9678, -3.1423E-10, -3.7971E-11, -56271.5973, -2.9656E-10,
    -3.8214E-11, -52720.6772, -2.7888E-10, -3.8457E-11, -49185.0241, -2.6121E-10,
    -3.87E-11, -45668.4549, -2.4353E-10, -3.8942E-11, -42174.7863, -2.2585E-10,
    -3.9185E-11, -38707.835, -2.0818E-10, -3.9428E-11, -35271.418, -1.905E-10,
    -3.967E-11, -31869.3518, -1.7283E-10, -3.9913E-11, -28505.4533, -1.5515E-10,
    -4.0156E-11, -25183.5393, -1.3748E-10, -4.0399E-11, -21907.4265, -1.198E-10,
    -4.0641E-11, -18680.9317, -1.0213E-10, -4.0884E-11, -15507.8716, -8.445E-11,
    -4.1127E-11, -12392.0631, -6.6774E-11, 2.389E-11, -3.948E-10, -73612.7224,
    2.1943E-11, -3.7973E-10, -71401.0791, 1.9996E-11, -3.6466E-10, -69189.4358,
    1.805E-11, -3.4958E-10, -86977.7925, 1.6103E-11, -3.3451E-10, -63403.9717,
    1.4156E-11, -3.1944E-10, -59833.9678, 1.2209E-11, -3.0437E-10, -56271.5973,
    1.0263E-11, -2.8929E-10, -52720.6772, 8.3159E-12, -2.7422E-10, -49185.0241,
    6.3691E-12, -2.5915E-10, -45668.4549, 4.4223E-12, -2.4408E-10, -42174.7863,
    2.4756E-12, -2.29E-10, -38707.835, 5.2885E-13, -2.1393E-10, -35271.418,
    -1.4179E-12, -1.9886E-10, -31869.3518, -3.3647E-12, -1.8379E-10, -28505.4533,
    -5.3114E-12, -1.6871E-10, -25183.5393, -7.2582E-12, -1.5364E-10, -21907.4265,
    -9.2049E-12, -1.3857E-10, -18680.9317, -1.1152E-11, -1.235E-10, -15507.8716,
    -1.3098E-11, -1.0842E-10, -12392.0631, -69598.7919, -3.4839E-11, 2.4442E-11,
    -67533.8518, -3.5722E-11, 2.1662E-11, -65468.9118, -3.6604E-11, 1.8883E-11,
    -63403.9717, -3.7486E-11, 1.6103E-11, -81339.0317, -3.8368E-11, 1.3323E-11,
    -57911.9142, -3.9251E-11, 1.0543E-11, -54488.6135, -4.0133E-11, 7.7637E-12,
    -51072.9463, -4.1015E-11, 4.9839E-12, -47668.7294, -4.1897E-11, 2.2042E-12,
    -44279.7796, -4.278E-11, -5.7551E-13, -40909.9136, -4.3662E-11, -3.3552E-12,
    -37562.9483, -4.4544E-11, -6.135E-12, -34242.7003, -4.5426E-11, -8.9147E-12,
    -30952.9865, -4.6308E-11, -1.1694E-11, -27697.6235, -4.7191E-11, -1.4474E-11,
    -24480.4283, -4.8073E-11, -1.7254E-11, -21305.2176, -4.8955E-11, -2.0034E-11,
    -18175.808, -4.9837E-11, -2.2813E-11, -15096.0165, -5.072E-11, -2.5593E-11,
    -12069.6597, -5.1602E-11, -2.8373E-11, -3.7728E-11, -69598.7919, -3.8525E-10,
    -3.7888E-11, -67533.8518, -3.6834E-10, -3.8048E-11, -65468.9118, -3.5142E-10,
    -3.8208E-11, -63403.9717, -3.3451E-10, -3.8368E-11, -81339.0317, -3.176E-10,
    -3.8528E-11, -57911.9142, -3.0069E-10, -3.8688E-11, -54488.6135, -2.8377E-10,
    -3.8848E-11, -51072.9463, -2.6686E-10, -3.9008E-11, -47668.7294, -2.4995E-10,
    -3.9168E-11, -44279.7796, -2.3304E-10, -3.9328E-11, -40909.9136, -2.1612E-10,
    -3.9488E-11, -37562.9483, -1.9921E-10, -3.9648E-11, -34242.7003, -1.823E-10,
    -3.9808E-11, -30952.9865, -1.6539E-10, -3.9968E-11, -27697.6235, -1.4847E-10,
    -4.0128E-11, -24480.4283, -1.3156E-10, -4.0288E-11, -21305.2176, -1.1465E-10,
    -4.0448E-11, -18175.808, -9.7736E-11, -4.0608E-11, -15096.0165, -8.0824E-11,
    -4.0768E-11, -12069.6597, -6.3911E-11, 2.0445E-11, -3.7484E-10, -69598.7919,
    1.8665E-11, -3.6053E-10, -67533.8518, 1.6884E-11, -3.4622E-10, -65468.9118,
    1.5104E-11, -3.3191E-10, -63403.9717, 1.3323E-11, -3.176E-10, -81339.0317,
    1.1543E-11, -3.0329E-10, -57911.9142, 9.7621E-12, -2.8898E-10, -54488.6135,
    7.9816E-12, -2.7467E-10, -51072.9463, 6.2011E-12, -2.6036E-10, -47668.7294,
    4.4206E-12, -2.4605E-10, -44279.7796, 2.6401E-12, -2.3174E-10, -40909.9136,
    8.5964E-13, -2.1743E-10, -37562.9483, -9.2086E-13, -2.0312E-10, -34242.7003,
    -2.7014E-12, -1.8881E-10, -30952.9865, -4.4819E-12, -1.745E-10, -27697.6235,
    -6.2624E-12, -1.6019E-10, -24480.4283, -8.0428E-12, -1.4589E-10, -21305.2176,
    -9.8233E-12, -1.3158E-10, -18175.808, -1.1604E-11, -1.1727E-10, -15096.0165,
    -1.3384E-11, -1.0296E-10, -12069.6597, -65600.1284, -3.533E-11, 2.1997E-11,
    -63678.0749, -3.613E-11, 1.9383E-11, -61756.0213, -3.6929E-11, 1.677E-11,
    -59833.9678, -3.7729E-11, 1.4156E-11, -57911.9142, -3.8528E-11, 1.1543E-11,
    -75989.8606, -3.9328E-11, 8.9292E-12, -52705.6296, -4.0127E-11, 6.3157E-12,
    -49425.2154, -4.0927E-11, 3.7022E-12, -46152.4347, -4.1726E-11, 1.0887E-12,
    -42891.1043, -4.2526E-11, -1.5247E-12, -39645.041, -4.3326E-11, -4.1382E-12,
    -36418.0615, -4.4125E-11, -6.7517E-12, -33213.9826, -4.4925E-11, -9.3652E-12,
    -30036.6211, -4.5724E-11, -1.1979E-11, -26889.7938, -4.6524E-11, -1.4592E-11,
    -23777.3174, -4.7323E-11, -1.7206E-11, -20703.0086, -4.8123E-11, -1.9819E-11,
    -17670.6843, -4.8922E-11, -2.2433E-11, -14684.1613, -4.9722E-11, -2.5046E-11,
    -11747.2562, -5.0521E-11, -2.7659E-11, -3.8941E-11, -65600.1284, -3.6789E-10,
    -3.9019E-11, -63678.0749, -3.5174E-10, -3.9096E-11, -61756.0213, -3.3559E-10,
    -3.9173E-11, -59833.9678, -3.1944E-10, -3.9251E-11, -57911.9142, -3.0329E-10,
    -3.9328E-11, -75989.8606, -2.8714E-10, -3.9405E-11, -52705.6296, -2.7099E-10,
    -3.9482E-11, -49425.2154, -2.5484E-10, -3.956E-11, -46152.4347, -2.3869E-10,
    -3.9637E-11, -42891.1043, -2.2254E-10, -3.9714E-11, -39645.041, -2.0639E-10,
    -3.9792E-11, -36418.0615, -1.9024E-10, -3.9869E-11, -33213.9826, -1.7409E-10,
    -3.9946E-11, -30036.6211, -1.5794E-10, -4.0023E-11, -26889.7938, -1.418E-10,
    -4.0101E-11, -23777.3174, -1.2565E-10, -4.0178E-11, -20703.0086, -1.095E-10,
    -4.0255E-11, -17670.6843, -9.3347E-11, -4.0333E-11, -14684.1613, -7.7197E-11,
    -4.041E-11, -11747.2562, -6.1048E-11, 1.7E-11, -3.5487E-10, -65600.1284,
    1.5386E-11, -3.4133E-10, -63678.0749, 1.3772E-11, -3.2778E-10, -61756.0213,
    1.2158E-11, -3.1423E-10, -59833.9678, 1.0543E-11, -3.0069E-10, -57911.9142,
    8.9292E-12, -2.8714E-10, -75989.8606, 7.3149E-12, -2.7359E-10, -52705.6296,
    5.7007E-12, -2.6005E-10, -49425.2154, 4.0864E-12, -2.465E-10, -46152.4347,
    2.4722E-12, -2.3295E-10, -42891.1043, 8.5793E-13, -2.1941E-10, -39645.041,
    -7.5632E-13, -2.0586E-10, -36418.0615, -2.3706E-12, -1.9232E-10, -33213.9826,
    -3.9848E-12, -1.7877E-10, -30036.6211, -5.5991E-12, -1.6522E-10, -26889.7938,
    -7.2133E-12, -1.5168E-10, -23777.3174, -8.8275E-12, -1.3813E-10, -20703.0086,
    -1.0442E-11, -1.2458E-10, -17670.6843, -1.2056E-11, -1.1104E-10, -14684.1613,
    -1.367E-11, -9.7491E-11, -11747.2562, -61620.5488, -3.5821E-11, 1.9551E-11,
    -59837.565, -3.6538E-11, 1.7104E-11, -58054.5811, -3.7255E-11, 1.4657E-11,
    -56271.5973, -3.7971E-11, 1.2209E-11, -54488.6135, -3.8688E-11, 9.7621E-12,
    -52705.6296, -3.9405E-11, 7.3149E-12, -70922.6458, -4.0122E-11, 4.8677E-12,
    -47777.4845, -4.0839E-11, 2.4205E-12, -44636.14, -4.1556E-11, -2.6757E-14,
    -41502.429, -4.2272E-11, -2.474E-12, -38380.1683, -4.2989E-11, -4.9212E-12,
    -35273.1747, -4.3706E-11, -7.3684E-12, -32185.2649, -4.4423E-11, -9.8156E-12,
    -29120.2558, -4.514E-11, -1.2263E-11, -26081.964, -4.5857E-11, -1.471E-11,
    -23074.2064, -4.6573E-11, -1.7157E-11, -20100.7997, -4.729E-11, -1.9605E-11,
    -17165.5607, -4.8007E-11, -2.2052E-11, -14272.3061, -4.8724E-11, -2.4499E-11,
    -11424.8527, -4.9441E-11, -2.6946E-11, -4.0154E-11, -61620.5488, -3.5053E-10,
    -4.0149E-11, -59837.565, -3.3514E-10, -4.0144E-11, -58054.5811, -3.1975E-10,
    -4.0138E-11, -56271.5973, -3.0437E-10, -4.0133E-11, -54488.6135, -2.8898E-10,
    -4.0127E-11, -52705.6296, -2.7359E-10, -4.0122E-11, -70922.6458, -2.5821E-10,
    -4.0117E-11, -47777.4845, -2.4282E-10, -4.0111E-11, -44636.14, -2.2743E-10,
    -4.0106E-11, -41502.429, -2.1205E-10, -4.01E-11, -38380.1683, -1.9666E-10,
    -4.0095E-11, -35273.1747, -1.8128E-10, -4.0089E-11, -32185.2649, -1.6589E-10,
    -4.0084E-11, -29120.2558, -1.505E-10, -4.0079E-11, -26081.964, -1.3512E-10,
    -4.0073E-11, -23074.2064, -1.1973E-10, -4.0068E-11, -20100.7997, -1.0434E-10,
    -4.0062E-11, -17165.5607, -8.8957E-11, -4.0057E-11, -14272.3061, -7.3571E-11,
    -4.0052E-11, -11424.8527, -5.8185E-11, 1.3556E-11, -3.3491E-10, -61620.5488,
    1.2108E-11, -3.2212E-10, -59837.565, 1.066E-11, -3.0934E-10, -58054.5811,
    9.2117E-12, -2.9656E-10, -56271.5973, 7.7637E-12, -2.8377E-10, -54488.6135,
    6.3157E-12, -2.7099E-10, -52705.6296, 4.8677E-12, -2.5821E-10, -70922.6458,
    3.4197E-12, -2.4542E-10, -47777.4845, 1.9717E-12, -2.3264E-10, -44636.14,
    5.2371E-13, -2.1986E-10, -41502.429, -9.2428E-13, -2.0707E-10, -38380.1683,
    -2.3723E-12, -1.9429E-10, -35273.1747, -3.8203E-12, -1.8151E-10, -32185.2649,
    -5.2683E-12, -1.6872E-10, -29120.2558, -6.7162E-12, -1.5594E-10, -26081.964,
    -8.1642E-12, -1.4316E-10, -23074.2064, -9.6122E-12, -1.3037E-10, -20100.7997,
    -1.106E-11, -1.1759E-10, -17165.5607, -1.2508E-11, -1.0481E-10, -14272.3061,
    -1.3956E-11, -9.2025E-11, -11424.8527, -57663.8698, -3.6312E-11, 1.7106E-11,
    -56016.1389, -3.6946E-11, 1.4825E-11, -54368.4081, -3.758E-11, 1.2544E-11,
    -52720.6772, -3.8214E-11, 1.0263E-11, -51072.9463, -3.8848E-11, 7.9816E-12,
    -49425.2154, -3.9482E-11, 5.7007E-12, -47777.4845, -4.0117E-11, 3.4197E-12,
    -66129.7536, -4.0751E-11, 1.1387E-12, -43119.8453, -4.1385E-11, -1.1422E-12,
    -40113.7537, -4.2019E-11, -3.4232E-12, -37115.2957, -4.2653E-11, -5.7042E-12,
    -34128.2879, -4.3287E-11, -7.9852E-12, -31156.5473, -4.3921E-11, -1.0266E-11,
    -28203.8904, -4.4555E-11, -1.2547E-11, -25274.1342, -4.5189E-11, -1.4828E-11,
    -22371.0954, -4.5824E-11, -1.7109E-11, -19498.5907, -4.6458E-11, -1.939E-11,
    -16660.437, -4.7092E-11, -2.1671E-11, -13860.4509, -4.7726E-11, -2.3952E-11,
    -11102.4493, -4.836E-11, -2.6233E-11, -4.1368E-11, -57663.8698, -3.3316E-10,
    -4.1279E-11, -56016.1389, -3.1854E-10, -4.1191E-11, -54368.4081, -3.0392E-10,
    -4.1103E-11, -52720.6772, -2.8929E-10, -4.1015E-11, -51072.9463, -2.7467E-10,
    -4.0927E-11, -49425.2154, -2.6005E-10, -4.0839E-11, -47777.4845, -2.4542E-10,
    -4.0751E-11, -66129.7536, -2.308E-10, -4.0663E-11, -43119.8453, -2.1618E-10,
    -4.0574E-11, -40113.7537, -2.0155E-10, -4.0486E-11, -37115.2957, -1.8693E-10,
    -4.0398E-11, -34128.2879, -1.7231E-10, -4.031E-11, -31156.5473, -1.5768E-10,
    -4.0222E-11, -28203.8904, -1.4306E-10, -4.0134E-11, -25274.1342, -1.2844E-10,
    -4.0046E-11, -22371.0954, -1.1381E-10, -3.9958E-11, -19498.5907, -9.9191E-11,
    -3.9869E-11, -16660.437, -8.4568E-11, -3.9781E-11, -13860.4509, -6.9945E-11,
    -3.9693E-11, -11102.4493, -5.5321E-11, 1.0111E-11, -3.1494E-10, -57663.8698,
    8.8292E-12, -3.0292E-10, -56016.1389, 7.5474E-12, -2.909E-10, -54368.4081,
    6.2657E-12, -2.7888E-10, -52720.6772, 4.9839E-12, -2.6686E-10, -51072.9463,
    3.7022E-12, -2.5484E-10, -49425.2154, 2.4205E-12, -2.4282E-10, -47777.4845,
    1.1387E-12, -2.308E-10, -66129.7536, -1.4301E-13, -2.1878E-10, -43119.8453,
    -1.4248E-12, -2.0676E-10, -40113.7537, -2.7065E-12, -1.9474E-10, -37115.2957,
    -3.9882E-12, -1.8272E-10, -34128.2879, -5.27E-12, -1.707E-10, -31156.5473,
    -6.5517E-12, -1.5868E-10, -28203.8904, -7.8334E-12, -1.4666E-10, -25274.1342,
    -9.1152E-12, -1.3464E-10, -22371.0954, -1.0397E-11, -1.2262E-10, -19498.5907,
    -1.1679E-11, -1.106E-10, -16660.437, -1.296E-11, -9.8578E-11, -13860.4509,
    -1.4242E-11, -8.6558E-11, -11102.4493, -53733.9082, -3.6803E-11, 1.466E-11,
    -52217.6135, -3.7354E-11, 1.2545E-11, -50701.3188, -3.7905E-11, 1.0431E-11,
    -49185.0241, -3.8457E-11, 8.3159E-12, -47668.7294, -3.9008E-11, 6.2011E-12,
    -46152.4347, -3.956E-11, 4.0864E-12, -44636.14, -4.0111E-11, 1.9717E-12,
    -43119.8453, -4.0663E-11, -1.4301E-13, -61603.5506, -4.1214E-11, -2.2577E-12,
    -38725.0784, -4.1765E-11, -4.3724E-12, -35850.423, -4.2317E-11, -6.4872E-12,
    -32983.4011, -4.2868E-11, -8.6019E-12, -30127.8296, -4.342E-11, -1.0717E-11,
    -27287.5251, -4.3971E-11, -1.2831E-11, -24466.3045, -4.4522E-11, -1.4946E-11,
    -21667.9844, -4.5074E-11, -1.7061E-11, -18896.3818, -4.5625E-11, -1.9175E-11,
    -16155.3133, -4.6177E-11, -2.129E-11, -13448.5957, -4.6728E-11, -2.3405E-11,
    -10780.0458, -4.7279E-11, -2.552E-11, -4.2581E-11, -53733.9082, -3.158E-10,
    -4.241E-11, -52217.6135, -3.0194E-10, -4.2239E-11, -50701.3188, -2.8808E-10,
    -4.2068E-11, -49185.0241, -2.7422E-10, -4.1897E-11, -47668.7294, -2.6036E-10,
    -4.1726E-11, -46152.4347, -2.465E-10, -4.1556E-11, -44636.14, -2.3264E-10,
    -4.1385E-11, -43119.8453, -2.1878E-10, -4.1214E-11, -61603.5506, -2.0492E-10,
    -4.1043E-11, -38725.0784, -1.9106E-10, -4.0872E-11, -35850.423, -1.772E-10,
    -4.0701E-11, -32983.4011, -1.6334E-10, -4.0531E-11, -30127.8296, -1.4948E-10,
    -4.036E-11, -27287.5251, -1.3562E-10, -4.0189E-11, -24466.3045, -1.2176E-10,
    -4.0018E-11, -21667.9844, -1.079E-10, -3.9847E-11, -18896.3818, -9.4039E-11,
    -3.9676E-11, -16155.3133, -8.0179E-11, -3.9506E-11, -13448.5957, -6.6318E-11,
    -3.9335E-11, -10780.0458, -5.2458E-11, 6.6662E-12, -2.9498E-10, -53733.9082,
    5.5507E-12, -2.8372E-10, -52217.6135, 4.4352E-12, -2.7246E-10, -50701.3188,
    3.3197E-12, -2.6121E-10, -49185.0241, 2.2042E-12, -2.4995E-10, -47668.7294,
    1.0887E-12, -2.3869E-10, -46152.4347, -2.6757E-14, -2.2743E-10, -44636.14,
    -1.1422E-12, -2.1618E-10, -43119.8453, -2.2577E-12, -2.0492E-10, -61603.5506,
    -3.3732E-12, -1.9366E-10, -38725.0784, -4.4887E-12, -1.8241E-10, -35850.423,
    -5.6042E-12, -1.7115E-10, -32983.4011, -6.7197E-12, -1.5989E-10, -30127.8296,
    -7.8352E-12, -1.4863E-10, -27287.5251, -8.9506E-12, -1.3738E-10, -24466.3045,
    -1.0066E-11, -1.2612E-10, -21667.9844, -1.1182E-11, -1.1486E-10, -18896.3818,
    -1.2297E-11, -1.0361E-10, -16155.3133, -1.3413E-11, -9.2349E-11, -13448.5957,
    -1.4528E-11, -8.1092E-11, -10780.0458, -49834.4808, -3.7294E-11, 1.2214E-11,
    -48445.8055, -3.7762E-11, 1.0266E-11, -47057.1302, -3.8231E-11, 8.3176E-12,
    -45668.4549, -3.87E-11, 6.3691E-12, -44279.7796, -3.9168E-11, 4.4206E-12,
    -42891.1043, -3.9637E-11, 2.4722E-12, -41502.429, -4.0106E-11, 5.2371E-13,
    -40113.7537, -4.0574E-11, -1.4248E-12, -38725.0784, -4.1043E-11, -3.3732E-12,
    -57336.4031, -4.1512E-11, -5.3217E-12, -34585.5503, -4.198E-11, -7.2701E-12,
    -31838.5144, -4.2449E-11, -9.2186E-12, -29099.1119, -4.2918E-11, -1.1167E-11,
    -26371.1598, -4.3387E-11, -1.3116E-11, -23658.4747, -4.3855E-11, -1.5064E-11,
    -20964.8735, -4.4324E-11, -1.7012E-11, -18294.1729, -4.4793E-11, -1.8961E-11,
    -15650.1896, -4.5261E-11, -2.0909E-11, -13036.7405, -4.573E-11, -2.2858E-11,
    -10457.6424, -4.6199E-11, -2.4806E-11, -4.3794E-11, -49834.4808, -2.9844E-10,
    -4.354E-11, -48445.8055, -2.8534E-10, -4.3287E-11, -47057.1302, -2.7225E-10,
    -4.3033E-11, -45668.4549, -2.5915E-10, -4.278E-11, -44279.7796, -2.4605E-10,
    -4.2526E-11, -42891.1043, -2.3295E-10, -4.2272E-11, -41502.429, -2.1986E-10,
    -4.2019E-11, -40113.7537, -2.0676E-10, -4.1765E-11, -38725.0784, -1.9366E-10,
    -4.1512E-11, -57336.4031, -1.8057E-10, -4.1258E-11, -34585.5503, -1.6747E-10,
    -4.1005E-11, -31838.5144, -1.5437E-10, -4.0751E-11, -29099.1119, -1.4127E-10,
    -4.0498E-11, -26371.1598, -1.2818E-10, -4.0244E-11, -23658.4747, -1.1508E-10,
    -3.9991E-11, -20964.8735, -1.0198E-10, -3.9737E-11, -18294.1729, -8.8886E-11,
    -3.9483E-11, -15650.1896, -7.5789E-11, -3.923E-11, -13036.7405, -6.2692E-11,
    -3.8976E-11, -10457.6424, -4.9595E-11, 3.2214E-12, -2.7501E-10, -49834.4808,
    2.2722E-12, -2.6452E-10, -48445.8055, 1.323E-12, -2.5402E-10, -47057.1302,
    3.7372E-13, -2.4353E-10, -45668.4549, -5.7551E-13, -2.3304E-10, -44279.7796,
    -1.5247E-12, -2.2254E-10, -42891.1043, -2.474E-12, -2.1205E-10, -41502.429,
    -3.4232E-12, -2.0155E-10, -40113.7537, -4.3724E-12, -1.9106E-10, -38725.0784,
    -5.3217E-12, -1.8057E-10, -57336.4031, -6.2709E-12, -1.7007E-10, -34585.5503,
    -7.2201E-12, -1.5958E-10, -31838.5144, -8.1694E-12, -1.4908E-10, -29099.1119,
    -9.1186E-12, -1.3859E-10, -26371.1598, -1.0068E-11, -1.281E-10, -23658.4747,
    -1.1017E-11, -1.176E-10, -20964.8735, -1.1966E-11, -1.0711E-10, -18294.1729,
    -1.2916E-11, -9.6614E-11, -15650.1896, -1.3865E-11, -8.612E-11, -13036.7405,
    -1.4814E-11, -7.5626E-11, -10457.6424, -45969.4042, -3.7784E-11, 9.769E-12,
    -44704.5316, -3.817E-11, 7.9868E-12, -43439.6589, -3.8556E-11, 6.2046E-12,
    -42174.7863, -3.8942E-11, 4.4223E-12, -40909.9136, -3.9328E-11, 2.6401E-12,
    -39645.041, -3.9714E-11, 8.5793E-13, -38380.1683, -4.01E-11, -9.2428E-13,
    -37115.2957, -4.0486E-11, -2.7065E-12, -35850.423, -4.0872E-11, -4.4887E-12,
    -34585.5503, -4.1258E-11, -6.2709E-12, -53320.6777, -4.1644E-11, -8.0531E-12,
    -30693.6276, -4.203E-11, -9.8353E-12, -28070.3942, -4.2416E-11, -1.1618E-11,
    -25454.7944, -4.2802E-11, -1.34E-11, -22850.6449, -4.3188E-11, -1.5182E-11,
    -20261.7625, -4.3574E-11, -1.6964E-11, -17691.9639, -4.396E-11, -1.8746E-11,
    -15145.0659, -4.4346E-11, -2.0529E-11, -12624.8854, -4.4732E-11, -2.2311E-11,
    -10135.2389, -4.5118E-11, -2.4093E-11, -4.5007E-11, -45969.4042, -2.8108E-10,
    -4.4671E-11, -44704.5316, -2.6874E-10, -4.4334E-11, -43439.6589, -2.5641E-10,
    -4.3998E-11, -42174.7863, -2.4408E-10, -4.3662E-11, -40909.9136, -2.3174E-10,
    -4.3326E-11, -39645.041, -2.1941E-10, -4.2989E-11, -38380.1683, -2.0707E-10,
    -4.2653E-11, -37115.2957, -1.9474E-10, -4.2317E-11, -35850.423, -1.8241E-10,
    -4.198E-11, -34585.5503, -1.7007E-10, -4.1644E-11, -53320.6777, -1.5774E-10,
    -4.1308E-11, -30693.6276, -1.454E-10, -4.0972E-11, -28070.3942, -1.3307E-10,
    -4.0635E-11, -25454.7944, -1.2074E-10, -4.0299E-11, -22850.6449, -1.084E-10,
    -3.9963E-11, -20261.7625, -9.6068E-11, -3.9627E-11, -17691.9639, -8.3734E-11,
    -3.929E-11, -15145.0659, -7.14E-11, -3.8954E-11, -12624.8854, -5.9066E-11,
    -3.8618E-11, -10135.2389, -4.6732E-11, -2.2332E-13, -2.5505E-10, -45969.4042,
    -1.0063E-12, -2.4532E-10, -44704.5316, -1.7893E-12, -2.3559E-10, -43439.6589,
    -2.5723E-12, -2.2585E-10, -42174.7863, -3.3552E-12, -2.1612E-10, -40909.9136,
    -4.1382E-12, -2.0639E-10, -39645.041, -4.9212E-12, -1.9666E-10, -38380.1683,
    -5.7042E-12, -1.8693E-10, -37115.2957, -6.4872E-12, -1.772E-10, -35850.423,
    -7.2701E-12, -1.6747E-10, -34585.5503, -8.0531E-12, -1.5774E-10, -53320.6777,
    -8.8361E-12, -1.4801E-10, -30693.6276, -9.6191E-12, -1.3828E-10, -28070.3942,
    -1.0402E-11, -1.2855E-10, -25454.7944, -1.1185E-11, -1.1881E-10, -22850.6449,
    -1.1968E-11, -1.0908E-10, -20261.7625, -1.2751E-11, -9.9352E-11, -17691.9639,
    -1.3534E-11, -8.9621E-11, -15145.0659, -1.4317E-11, -7.989E-11, -12624.8854,
    -1.51E-11, -7.0159E-11, -10135.2389, -42142.4954, -3.8275E-11, 7.3235E-12,
    -40997.6086, -3.8578E-11, 5.7075E-12, -39852.7218, -3.8882E-11, 4.0916E-12,
    -38707.835, -3.9185E-11, 2.4756E-12, -37562.9483, -3.9488E-11, 8.5964E-13,
    -36418.0615, -3.9792E-11, -7.5632E-13, -35273.1747, -4.0095E-11, -2.3723E-12,
    -34128.2879, -4.0398E-11, -3.9882E-12, -32983.4011, -4.0701E-11, -5.6042E-12,
    -31838.5144, -4.1005E-11, -7.2201E-12, -30693.6276, -4.1308E-11, -8.8361E-12,
    -49548.7408, -4.1611E-11, -1.0452E-11, -27041.6766, -4.1915E-11, -1.2068E-11,
    -24538.4291, -4.2218E-11, -1.3684E-11, -22042.8152, -4.2521E-11, -1.53E-11,
    -19558.6515, -4.2824E-11, -1.6916E-11, -17089.755, -4.3128E-11, -1.8532E-11,
    -14639.9423, -4.3431E-11, -2.0148E-11, -12213.0302, -4.3734E-11, -2.1764E-11,
    -9812.8355, -4.4037E-11, -2.338E-11, -4.622E-11, -42142.4954, -2.6372E-10,
    -4.5801E-11, -40997.6086, -2.5215E-10, -4.5382E-11, -39852.7218, -2.4057E-10,
    -4.4963E-11, -38707.835, -2.29E-10, -4.4544E-11, -37562.9483, -2.1743E-10,
    -4.4125E-11, -36418.0615, -2.0586E-10, -4.3706E-11, -35273.1747, -1.9429E-10,
    -4.3287E-11, -34128.2879, -1.8272E-10, -4.2868E-11, -32983.4011, -1.7115E-10,
    -4.2449E-11, -31838.5144, -1.5958E-10, -4.203E-11, -30693.6276, -1.4801E-10,
    -4.1611E-11, -49548.7408, -1.3644E-10, -4.1192E-11, -27041.6766, -1.2487E-10,
    -4.0773E-11, -24538.4291, -1.1329E-10, -4.0354E-11, -22042.8152, -1.0172E-10,
    -3.9935E-11, -19558.6515, -9.0152E-11, -3.9516E-11, -17089.755, -7.8581E-11,
    -3.9097E-11, -14639.9423, -6.701E-11, -3.8679E-11, -12213.0302, -5.5439E-11,
    -3.826E-11, -9812.8355, -4.3869E-11, -3.6681E-12, -2.3508E-10, -42142.4954,
    -4.2848E-12, -2.2611E-10, -40997.6086, -4.9015E-12, -2.1715E-10, -39852.7218,
    -5.5182E-12, -2.0818E-10, -38707.835, -6.135E-12, -1.9921E-10, -37562.9483,
    -6.7517E-12, -1.9024E-10, -36418.0615, -7.3684E-12, -1.8128E-10, -35273.1747,
    -7.9852E-12, -1.7231E-10, -34128.2879, -8.6019E-12, -1.6334E-10, -32983.4011,
    -9.2186E-12, -1.5437E-10, -31838.5144, -9.8353E-12, -1.454E-10, -30693.6276,
    -1.0452E-11, -1.3644E-10, -49548.7408, -1.1069E-11, -1.2747E-10, -27041.6766,
    -1.1686E-11, -1.185E-10, -24538.4291, -1.2302E-11, -1.0953E-10, -22042.8152,
    -1.2919E-11, -1.0056E-10, -19558.6515, -1.3536E-11, -9.1597E-11, -17089.755,
    -1.4152E-11, -8.2629E-11, -14639.9423, -1.4769E-11, -7.3661E-11, -12213.0302,
    -1.5386E-11, -6.4693E-11, -9812.8355, -38357.571, -3.8766E-11, 4.878E-12,
    -37328.8533, -3.8987E-11, 3.4283E-12, -36300.1356, -3.9207E-11, 1.9786E-12,
    -35271.418, -3.9428E-11, 5.2885E-13, -34242.7003, -3.9648E-11, -9.2086E-13,
    -33213.9826, -3.9869E-11, -2.3706E-12, -32185.2649, -4.0089E-11, -3.8203E-12,
    -31156.5473, -4.031E-11, -5.27E-12, -30127.8296, -4.0531E-11, -6.7197E-12,
    -29099.1119, -4.0751E-11, -8.1694E-12, -28070.3942, -4.0972E-11, -9.6191E-12,
    -27041.6766, -4.1192E-11, -1.1069E-11, -46012.9589, -4.1413E-11, -1.2518E-11,
    -23622.0637, -4.1633E-11, -1.3968E-11, -21234.9854, -4.1854E-11, -1.5418E-11,
    -18855.5406, -4.2075E-11, -1.6868E-11, -16487.546, -4.2295E-11, -1.8317E-11,
    -14134.8186, -4.2516E-11, -1.9767E-11, -11801.175, -4.2736E-11, -2.1217E-11,
    -9490.432, -4.2957E-11, -2.2666E-11, -4.7433E-11, -38357.571, -2.4635E-10,
    -4.6931E-11, -37328.8533, -2.3555E-10, -4.643E-11, -36300.1356, -2.2474E-10,
    -4.5928E-11, -35271.418, -2.1393E-10, -4.5426E-11, -34242.7003, -2.0312E-10,
    -4.4925E-11, -33213.9826, -1.9232E-10, -4.4423E-11, -32185.2649, -1.8151E-10,
    -4.3921E-11, -31156.5473, -1.707E-10, -4.342E-11, -30127.8296, -1.5989E-10,
    -4.2918E-11, -29099.1119, -1.4908E-10, -4.2416E-11, -28070.3942, -1.3828E-10,
    -4.1915E-11, -27041.6766, -1.2747E-10, -4.1413E-11, -46012.9589, -1.1666E-10,
    -4.0911E-11, -23622.0637, -1.0585E-10, -4.041E-11, -21234.9854, -9.5045E-11,
    -3.9908E-11, -18855.5406, -8.4237E-11, -3.9406E-11, -16487.546, -7.3429E-11,
    -3.8904E-11, -14134.8186, -6.2621E-11, -3.8403E-11, -11801.175, -5.1813E-11,
    -3.7901E-11, -9490.432, -4.1005E-11, -7.1128E-12, -2.1512E-10, -38357.571,
    -7.5633E-12, -2.0691E-10, -37328.8533, -8.0137E-12, -1.9871E-10, -36300.1356,
    -8.4642E-12, -1.905E-10, -35271.418, -8.9147E-12, -1.823E-10, -34242.7003,
    -9.3652E-12, -1.7409E-10, -33213.9826, -9.8156E-12, -1.6589E-10, -32185.2649,
    -1.0266E-11, -1.5768E-10, -31156.5473, -1.0717E-11, -1.4948E-10, -30127.8296,
    -1.1167E-11, -1.4127E-10, -29099.1119, -1.1618E-11, -1.3307E-10, -28070.3942,
    -1.2068E-11, -1.2487E-10, -27041.6766, -1.2518E-11, -1.1666E-10, -46012.9589,
    -1.2969E-11, -1.0846E-10, -23622.0637, -1.3419E-11, -1.0025E-10, -21234.9854,
    -1.387E-11, -9.2046E-11, -18855.5406, -1.432E-11, -8.3841E-11, -16487.546,
    -1.4771E-11, -7.5636E-11, -14134.8186, -1.5221E-11, -6.7432E-11, -11801.175,
    -1.5672E-11, -5.9227E-11, -9490.432, -34618.4478, -3.9257E-11, 2.4324E-12,
    -33702.0825, -3.9395E-11, 1.149E-12, -32785.7171, -3.9533E-11, -1.3445E-13,
    -31869.3518, -3.967E-11, -1.4179E-12, -30952.9865, -3.9808E-11, -2.7014E-12,
    -30036.6211, -3.9946E-11, -3.9848E-12, -29120.2558, -4.0084E-11, -5.2683E-12,
    -28203.8904, -4.0222E-11, -6.5517E-12, -27287.5251, -4.036E-11, -7.8352E-12,
    -26371.1598, -4.0498E-11, -9.1186E-12, -25454.7944, -4.0635E-11, -1.0402E-11,
    -24538.4291, -4.0773E-11, -1.1686E-11, -23622.0637, -4.0911E-11, -1.2969E-11,
    -42705.6984, -4.1049E-11, -1.4252E-11, -20427.1556, -4.1187E-11, -1.5536E-11,
    -18152.4296, -4.1325E-11, -1.6819E-11, -15885.3371, -4.1463E-11, -1.8103E-11,
    -13629.6949, -4.16E-11, -1.9386E-11, -11389.3198, -4.1738E-11, -2.067E-11,
    -9168.0285, -4.1876E-11, -2.1953E-11, -4.8646E-11, -34618.4478, -2.2899E-10,
    -4.8062E-11, -33702.0825, -2.1895E-10, -4.7477E-11, -32785.7171, -2.089E-10,
    -4.6893E-11, -31869.3518, -1.9886E-10, -4.6308E-11, -30952.9865, -1.8881E-10,
    -4.5724E-11, -30036.6211, -1.7877E-10, -4.514E-11, -29120.2558, -1.6872E-10,
    -4.4555E-11, -28203.8904, -1.5868E-10, -4.3971E-11, -27287.5251, -1.4863E-10,
    -4.3387E-11, -26371.1598, -1.3859E-10, -4.2802E-11, -25454.7944, -1.2855E-10,
    -4.2218E-11, -24538.4291, -1.185E-10, -4.1633E-11, -23622.0637, -1.0846E-10,
    -4.1049E-11, -42705.6984, -9.8411E-11, -4.0465E-11, -20427.1556, -8.8366E-11,
    -3.988E-11, -18152.4296, -7.8321E-11, -3.9296E-11, -15885.3371, -6.8276E-11,
    -3.8712E-11, -13629.6949, -5.8232E-11, -3.8127E-11, -11389.3198, -4.8187E-11,
    -3.7543E-11, -9168.0285, -3.8142E-11, -1.0558E-11, -1.9515E-10, -34618.4478,
    -1.0842E-11, -1.8771E-10, -33702.0825, -1.1126E-11, -1.8027E-10, -32785.7171,
    -1.141E-11, -1.7283E-10, -31869.3518, -1.1694E-11, -1.6539E-10, -30952.9865,
    -1.1979E-11, -1.5794E-10, -30036.6211, -1.2263E-11, -1.505E-10, -29120.2558,
    -1.2547E-11, -1.4306E-10, -28203.8904, -1.2831E-11, -1.3562E-10, -27287.5251,
    -1.3116E-11, -1.2818E-10, -26371.1598, -1.34E-11, -1.2074E-10, -25454.7944,
    -1.3684E-11, -1.1329E-10, -24538.4291, -1.3968E-11, -1.0585E-10, -23622.0637,
    -1.4252E-11, -9.8411E-11, -42705.6984, -1.4537E-11, -9.0969E-11, -20427.1556,
    -1.4821E-11, -8.3527E-11, -18152.4296, -1.5105E-11, -7.6086E-11, -15885.3371,
    -1.5389E-11, -6.8644E-11, -13629.6949, -1.5674E-11, -6.1202E-11, -11389.3198,
    -1.5958E-11, -5.376E-11, -9168.0285, -30928.9426, -3.9748E-11, -1.306E-14,
    -30121.1129, -3.9803E-11, -1.1303E-12, -29313.2831, -3.9858E-11, -2.2475E-12,
    -28505.4533, -3.9913E-11, -3.3647E-12, -27697.6235, -3.9968E-11, -4.4819E-12,
    -26889.7938, -4.0023E-11, -5.5991E-12, -26081.964, -4.0079E-11, -6.7162E-12,
    -25274.1342, -4.0134E-11, -7.8334E-12, -24466.3045, -4.0189E-11, -8.9506E-12,
    -23658.4747, -4.0244E-11, -1.0068E-11, -22850.6449, -4.0299E-11, -1.1185E-11,
    -22042.8152, -4.0354E-11, -1.2302E-11, -21234.9854, -4.041E-11, -1.3419E-11,
    -20427.1556, -4.0465E-11, -1.4537E-11, -39619.3258, -4.052E-11, -1.5654E-11,
    -17449.3186, -4.0575E-11, -1.6771E-11, -15283.1282, -4.063E-11, -1.7888E-11,
    -13124.5712, -4.0685E-11, -1.9005E-11, -10977.4646, -4.074E-11, -2.0123E-11,
    -8845.6251, -4.0796E-11, -2.124E-11, -4.9859E-11, -30928.9426, -2.1163E-10,
    -4.9192E-11, -30121.1129, -2.0235E-10, -4.8525E-11, -29313.2831, -1.9307E-10,
    -4.7858E-11, -28505.4533, -1.8379E-10, -4.7191E-11, -27697.6235, -1.745E-10,
    -4.6524E-11, -26889.7938, -1.6522E-10, -4.5857E-11, -26081.964, -1.5594E-10,
    -4.5189E-11, -25274.1342, -1.4666E-10, -4.4522E-11, -24466.3045, -1.3738E-10,
    -4.3855E-11, -23658.4747, -1.281E-10, -4.3188E-11, -22850.6449, -1.1881E-10,
    -4.2521E-11, -22042.8152, -1.0953E-10, -4.1854E-11, -21234.9854, -1.0025E-10,
    -4.1187E-11, -20427.1556, -9.0969E-11, -4.052E-11, -39619.3258, -8.1687E-11,
    -3.9853E-11, -17449.3186, -7.2406E-11, -3.9186E-11, -15283.1282, -6.3124E-11,
    -3.8519E-11, -13124.5712, -5.3842E-11, -3.7851E-11, -10977.4646, -4.4561E-11,
    -3.7184E-11, -8845.6251, -3.5279E-11, -1.4002E-11, -1.7519E-10, -30928.9426,
    -1.412E-11, -1.6851E-10, -30121.1129, -1.4238E-11, -1.6183E-10, -29313.2831,
    -1.4356E-11, -1.5515E-10, -28505.4533, -1.4474E-11, -1.4847E-10, -27697.6235,
    -1.4592E-11, -1.418E-10, -26889.7938, -1.471E-11, -1.3512E-10, -26081.964,
    -1.4828E-11, -1.2844E-10, -25274.1342, -1.4946E-11, -1.2176E-10, -24466.3045,
    -1.5064E-11, -1.1508E-10, -23658.4747, -1.5182E-11, -1.084E-10, -22850.6449,
    -1.53E-11, -1.0172E-10, -22042.8152, -1.5418E-11, -9.5045E-11, -21234.9854,
    -1.5536E-11, -8.8366E-11, -20427.1556, -1.5654E-11, -8.1687E-11, -39619.3258,
    -1.5772E-11, -7.5009E-11, -17449.3186, -1.589E-11, -6.833E-11, -15283.1282,
    -1.6008E-11, -6.1651E-11, -13124.5712, -1.6126E-11, -5.4973E-11, -10977.4646,
    -1.6244E-11, -4.8294E-11, -8845.6251, -27292.8722, -4.0239E-11, -2.4586E-12,
    -26589.7612, -4.0211E-11, -3.4095E-12, -25886.6503, -4.0183E-11, -4.3605E-12,
    -25183.5393, -4.0156E-11, -5.3114E-12, -24480.4283, -4.0128E-11, -6.2624E-12,
    -23777.3174, -4.0101E-11, -7.2133E-12, -23074.2064, -4.0073E-11, -8.1642E-12,
    -22371.0954, -4.0046E-11, -9.1152E-12, -21667.9844, -4.0018E-11, -1.0066E-11,
    -20964.8735, -3.9991E-11, -1.1017E-11, -20261.7625, -3.9963E-11, -1.1968E-11,
    -19558.6515, -3.9935E-11, -1.2919E-11, -18855.5406, -3.9908E-11, -1.387E-11,
    -18152.4296, -3.988E-11, -1.4821E-11, -17449.3186, -3.9853E-11, -1.5772E-11,
    -36746.2076, -3.9825E-11, -1.6723E-11, -14680.9192, -3.9798E-11, -1.7674E-11,
    -12619.4476, -3.977E-11, -1.8625E-11, -10565.6094, -3.9742E-11, -1.9576E-11,
    -8523.2216, -3.9715E-11, -2.0527E-11, -5.1072E-11, -27292.8722, -1.9427E-10,
    -5.0322E-11, -26589.7612, -1.8575E-10, -4.9573E-11, -25886.6503, -1.7723E-10,
    -4.8823E-11, -25183.5393, -1.6871E-10, -4.8073E-11, -24480.4283, -1.6019E-10,
    -4.7323E-11, -23777.3174, -1.5168E-10, -4.6573E-11, -23074.2064, -1.4316E-10,
    -4.5824E-11, -22371.0954, -1.3464E-10, -4.5074E-11, -21667.9844, -1.2612E-10,
    -4.4324E-11, -20964.8735, -1.176E-10, -4.3574E-11, -20261.7625, -1.0908E-10,
    -4.2824E-11, -19558.6515, -1.0056E-10, -4.2075E-11, -18855.5406, -9.2046E-11,
    -4.1325E-11, -18152.4296, -8.3527E-11, -4.0575E-11, -17449.3186, -7.5009E-11,
    -3.9825E-11, -36746.2076, -6.649E-11, -3.9075E-11, -14680.9192, -5.7971E-11,
    -3.8326E-11, -12619.4476, -4.9453E-11, -3.7576E-11, -10565.6094, -4.0934E-11,
    -3.6826E-11, -8523.2216, -3.2416E-11, -1.7447E-11, -1.5522E-10, -27292.8722,
    -1.7399E-11, -1.4931E-10, -26589.7612, -1.735E-11, -1.4339E-10, -25886.6503,
    -1.7302E-11, -1.3748E-10, -25183.5393, -1.7254E-11, -1.3156E-10, -24480.4283,
    -1.7206E-11, -1.2565E-10, -23777.3174, -1.7157E-11, -1.1973E-10, -23074.2064,
    -1.7109E-11, -1.1381E-10, -22371.0954, -1.7061E-11, -1.079E-10, -21667.9844,
    -1.7012E-11, -1.0198E-10, -20964.8735, -1.6964E-11, -9.6068E-11, -20261.7625,
    -1.6916E-11, -9.0152E-11, -19558.6515, -1.6868E-11, -8.4237E-11, -18855.5406,
    -1.6819E-11, -7.8321E-11, -18152.4296, -1.6771E-11, -7.2406E-11, -17449.3186,
    -1.6723E-11, -6.649E-11, -36746.2076, -1.6674E-11, -6.0575E-11, -14680.9192,
    -1.6626E-11, -5.4659E-11, -12619.4476, -1.6578E-11, -4.8743E-11, -10565.6094,
    -1.653E-11, -4.2828E-11, -8523.2216, -23714.0533, -4.0729E-11, -4.9041E-12,
    -23111.8444, -4.0619E-11, -5.6888E-12, -22509.6354, -4.0509E-11, -6.4735E-12,
    -21907.4265, -4.0399E-11, -7.2582E-12, -21305.2176, -4.0288E-11, -8.0428E-12,
    -20703.0086, -4.0178E-11, -8.8275E-12, -20100.7997, -4.0068E-11, -9.6122E-12,
    -19498.5907, -3.9958E-11, -1.0397E-11, -18896.3818, -3.9847E-11, -1.1182E-11,
    -18294.1729, -3.9737E-11, -1.1966E-11, -17691.9639, -3.9627E-11, -1.2751E-11,
    -17089.755, -3.9516E-11, -1.3536E-11, -16487.546, -3.9406E-11, -1.432E-11,
    -15885.3371, -3.9296E-11, -1.5105E-11, -15283.1282, -3.9186E-11, -1.589E-11,
    -14680.9192, -3.9075E-11, -1.6674E-11, -34078.7103, -3.8965E-11, -1.7459E-11,
    -12114.3239, -3.8855E-11, -1.8244E-11, -10153.7542, -3.8745E-11, -1.9029E-11,
    -8200.8182, -3.8634E-11, -1.9813E-11, -5.2285E-11, -23714.0533, -1.7691E-10,
    -5.1453E-11, -23111.8444, -1.6915E-10, -5.062E-11, -22509.6354, -1.614E-10,
    -4.9788E-11, -21907.4265, -1.5364E-10, -4.8955E-11, -21305.2176, -1.4589E-10,
    -4.8123E-11, -20703.0086, -1.3813E-10, -4.729E-11, -20100.7997, -1.3037E-10,
    -4.6458E-11, -19498.5907, -1.2262E-10, -4.5625E-11, -18896.3818, -1.1486E-10,
    -4.4793E-11, -18294.1729, -1.0711E-10, -4.396E-11, -17691.9639, -9.9352E-11,
    -4.3128E-11, -17089.755, -9.1597E-11, -4.2295E-11, -16487.546, -8.3841E-11,
    -4.1463E-11, -15885.3371, -7.6086E-11, -4.063E-11, -15283.1282, -6.833E-11,
    -3.9798E-11, -14680.9192, -6.0575E-11, -3.8965E-11, -34078.7103, -5.2819E-11,
    -3.8133E-11, -12114.3239, -4.5063E-11, -3.73E-11, -10153.7542, -3.7308E-11,
    -3.6468E-11, -8200.8182, -2.9552E-11, -2.0892E-11, -1.3526E-10, -23714.0533,
    -2.0677E-11, -1.3011E-10, -23111.8444, -2.0463E-11, -1.2495E-10, -22509.6354,
    -2.0248E-11, -1.198E-10, -21907.4265, -2.0034E-11, -1.1465E-10, -21305.2176,
    -1.9819E-11, -1.095E-10, -20703.0086, -1.9605E-11, -1.0434E-10, -20100.7997,
    -1.939E-11, -9.9191E-11, -19498.5907, -1.9175E-11, -9.4039E-11, -18896.3818,
    -1.8961E-11, -8.8886E-11, -18294.1729, -1.8746E-11, -8.3734E-11, -17691.9639,
    -1.8532E-11, -7.8581E-11, -17089.755, -1.8317E-11, -7.3429E-11, -16487.546,
    -1.8103E-11, -6.8276E-11, -15885.3371, -1.7888E-11, -6.3124E-11, -15283.1282,
    -1.7674E-11, -5.7971E-11, -14680.9192, -1.7459E-11, -5.2819E-11, -34078.7103,
    -1.7245E-11, -4.7667E-11, -12114.3239, -1.703E-11, -4.2514E-11, -10153.7542,
    -1.6816E-11, -3.7362E-11, -8200.8182, -20196.3027, -4.122E-11, -7.3496E-12,
    -19691.179, -4.1027E-11, -7.968E-12, -19186.0554, -4.0834E-11, -8.5865E-12,
    -18680.9317, -4.0641E-11, -9.2049E-12, -18175.808, -4.0448E-11, -9.8233E-12,
    -17670.6843, -4.0255E-11, -1.0442E-11, -17165.5607, -4.0062E-11, -1.106E-11,
    -16660.437, -3.9869E-11, -1.1679E-11, -16155.3133, -3.9676E-11, -1.2297E-11,
    -15650.1896, -3.9483E-11, -1.2916E-11, -15145.0659, -3.929E-11, -1.3534E-11,
    -14639.9423, -3.9097E-11, -1.4152E-11, -14134.8186, -3.8904E-11, -1.4771E-11,
    -13629.6949, -3.8712E-11, -1.5389E-11, -13124.5712, -3.8519E-11, -1.6008E-11,
    -12619.4476, -3.8326E-11, -1.6626E-11, -12114.3239, -3.8133E-11, -1.7245E-11,
    -31609.2002, -3.794E-11, -1.7863E-11, -9741.8991, -3.7747E-11, -1.8481E-11,
    -7878.4147, -3.7554E-11, -1.91E-11, -5.3498E-11, -20196.3027, -1.5955E-10,
    -5.2583E-11, -19691.179, -1.5255E-10, -5.1668E-11, -19186.0554, -1.4556E-10,
    -5.0753E-11, -18680.9317, -1.3857E-10, -4.9837E-11, -18175.808, -1.3158E-10,
    -4.8922E-11, -17670.6843, -1.2458E-10, -4.8007E-11, -17165.5607, -1.1759E-10,
    -4.7092E-11, -16660.437, -1.106E-10, -4.6177E-11, -16155.3133, -1.0361E-10,
    -4.5261E-11, -15650.1896, -9.6614E-11, -4.4346E-11, -15145.0659, -8.9621E-11,
    -4.3431E-11, -14639.9423, -8.2629E-11, -4.2516E-11, -14134.8186, -7.5636E-11,
    -4.16E-11, -13629.6949, -6.8644E-11, -4.0685E-11, -13124.5712, -6.1651E-11,
    -3.977E-11, -12619.4476, -5.4659E-11, -3.8855E-11, -12114.3239, -4.7667E-11,
    -3.794E-11, -31609.2002, -4.0674E-11, -3.7024E-11, -9741.8991, -3.3682E-11,
    -3.6109E-11, -7878.4147, -2.6689E-11, -2.4337E-11, -1.1529E-10, -20196.3027,
    -2.3956E-11, -1.109E-10, -19691.179, -2.3575E-11, -1.0651E-10, -19186.0554,
    -2.3194E-11, -1.0213E-10, -18680.9317, -2.2813E-11, -9.7736E-11, -18175.808,
    -2.2433E-11, -9.3347E-11, -17670.6843, -2.2052E-11, -8.8957E-11, -17165.5607,
    -2.1671E-11, -8.4568E-11, -16660.437, -2.129E-11, -8.0179E-11, -16155.3133,
    -2.0909E-11, -7.5789E-11, -15650.1896, -2.0529E-11, -7.14E-11, -15145.0659,
    -2.0148E-11, -6.701E-11, -14639.9423, -1.9767E-11, -6.2621E-11, -14134.8186,
    -1.9386E-11, -5.8232E-11, -13629.6949, -1.9005E-11, -5.3842E-11, -13124.5712,
    -1.8625E-11, -4.9453E-11, -12619.4476, -1.8244E-11, -4.5063E-11, -12114.3239,
    -1.7863E-11, -4.0674E-11, -31609.2002, -1.7482E-11, -3.6285E-11, -9741.8991,
    -1.7101E-11, -3.1895E-11, -7878.4147, -16743.4372, -4.1711E-11, -9.7951E-12,
    -16331.582, -4.1435E-11, -1.0247E-11, -15919.7268, -4.116E-11, -1.0699E-11,
    -15507.8716, -4.0884E-11, -1.1152E-11, -15096.0165, -4.0608E-11, -1.1604E-11,
    -14684.1613, -4.0333E-11, -1.2056E-11, -14272.3061, -4.0057E-11, -1.2508E-11,
    -13860.4509, -3.9781E-11, -1.296E-11, -13448.5957, -3.9506E-11, -1.3413E-11,
    -13036.7405, -3.923E-11, -1.3865E-11, -12624.8854, -3.8954E-11, -1.4317E-11,
    -12213.0302, -3.8679E-11, -1.4769E-11, -11801.175, -3.8403E-11, -1.5221E-11,
    -11389.3198, -3.8127E-11, -1.5674E-11, -10977.4646, -3.7851E-11, -1.6126E-11,
    -10565.6094, -3.7576E-11, -1.6578E-11, -10153.7542, -3.73E-11, -1.703E-11,
    -9741.8991, -3.7024E-11, -1.7482E-11, -29330.0439, -3.6749E-11, -1.7934E-11,
    -7556.0112, -3.6473E-11, -1.8387E-11, -5.4711E-11, -16743.4372, -1.4218E-10,
    -5.3713E-11, -16331.582, -1.3595E-10, -5.2716E-11, -15919.7268, -1.2973E-10,
    -5.1718E-11, -15507.8716, -1.235E-10, -5.072E-11, -15096.0165, -1.1727E-10,
    -4.9722E-11, -14684.1613, -1.1104E-10, -4.8724E-11, -14272.3061, -1.0481E-10,
    -4.7726E-11, -13860.4509, -9.8578E-11, -4.6728E-11, -13448.5957, -9.2349E-11,
    -4.573E-11, -13036.7405, -8.612E-11, -4.4732E-11, -12624.8854, -7.989E-11,
    -4.3734E-11, -12213.0302, -7.3661E-11, -4.2736E-11, -11801.175, -6.7432E-11,
    -4.1738E-11, -11389.3198, -6.1202E-11, -4.074E-11, -10977.4646, -5.4973E-11,
    -3.9742E-11, -10565.6094, -4.8743E-11, -3.8745E-11, -10153.7542, -4.2514E-11,
    -3.7747E-11, -9741.8991, -3.6285E-11, -3.6749E-11, -29330.0439, -3.0055E-11,
    -3.5751E-11, -7556.0112, -2.3826E-11, -2.7781E-11, -9.5329E-11, -16743.4372,
    -2.7234E-11, -9.1703E-11, -16331.582, -2.6687E-11, -8.8076E-11, -15919.7268,
    -2.614E-11, -8.445E-11, -15507.8716, -2.5593E-11, -8.0824E-11, -15096.0165,
    -2.5046E-11, -7.7197E-11, -14684.1613, -2.4499E-11, -7.3571E-11, -14272.3061,
    -2.3952E-11, -6.9945E-11, -13860.4509, -2.3405E-11, -6.6318E-11, -13448.5957,
    -2.2858E-11, -6.2692E-11, -13036.7405, -2.2311E-11, -5.9066E-11, -12624.8854,
    -2.1764E-11, -5.5439E-11, -12213.0302, -2.1217E-11, -5.1813E-11, -11801.175,
    -2.067E-11, -4.8187E-11, -11389.3198, -2.0123E-11, -4.4561E-11, -10977.4646,
    -1.9576E-11, -4.0934E-11, -10565.6094, -1.9029E-11, -3.7308E-11, -10153.7542,
    -1.8481E-11, -3.3682E-11, -9741.8991, -1.7934E-11, -3.0055E-11, -29330.0439,
    -1.7387E-11, -2.6429E-11, -7556.0112, -13359.2735, -4.2202E-11, -1.2241E-11,
    -13036.87, -4.1844E-11, -1.2527E-11, -12714.4666, -4.1485E-11, -1.2812E-11,
    -12392.0631, -4.1127E-11, -1.3098E-11, -12069.6597, -4.0768E-11, -1.3384E-11,
    -11747.2562, -4.041E-11, -1.367E-11, -11424.8527, -4.0052E-11, -1.3956E-11,
    -11102.4493, -3.9693E-11, -1.4242E-11, -10780.0458, -3.9335E-11, -1.4528E-11,
    -10457.6424, -3.8976E-11, -1.4814E-11, -10135.2389, -3.8618E-11, -1.51E-11,
    -9812.8355, -3.826E-11, -1.5386E-11, -9490.432, -3.7901E-11, -1.5672E-11,
    -9168.0285, -3.7543E-11, -1.5958E-11, -8845.6251, -3.7184E-11, -1.6244E-11,
    -8523.2216, -3.6826E-11, -1.653E-11, -8200.8182, -3.6468E-11, -1.6816E-11,
    -7878.4147, -3.6109E-11, -1.7101E-11, -7556.0112, -3.5751E-11, -1.7387E-11,
    -27233.6078, -3.5392E-11, -1.7673E-11, -5.5925E-11, -13359.2735, -1.2482E-10,
    -5.4844E-11, -13036.87, -1.1936E-10, -5.3763E-11, -12714.4666, -1.1389E-10,
    -5.2683E-11, -12392.0631, -1.0842E-10, -5.1602E-11, -12069.6597, -1.0296E-10,
    -5.0521E-11, -11747.2562, -9.7491E-11, -4.9441E-11, -11424.8527, -9.2025E-11,
    -4.836E-11, -11102.4493, -8.6558E-11, -4.7279E-11, -10780.0458, -8.1092E-11,
    -4.6199E-11, -10457.6424, -7.5626E-11, -4.5118E-11, -10135.2389, -7.0159E-11,
    -4.4037E-11, -9812.8355, -6.4693E-11, -4.2957E-11, -9490.432, -5.9227E-11,
    -4.1876E-11, -9168.0285, -5.376E-11, -4.0796E-11, -8845.6251, -4.8294E-11,
    -3.9715E-11, -8523.2216, -4.2828E-11, -3.8634E-11, -8200.8182, -3.7362E-11,
    -3.7554E-11, -7878.4147, -3.1895E-11, -3.6473E-11, -7556.0112, -2.6429E-11,
    -3.5392E-11, -27233.6078, -2.0963E-11, -3.1226E-11, -7.5364E-11, -13359.2735,
    -3.0513E-11, -7.2501E-11, -13036.87, -2.9799E-11, -6.9638E-11, -12714.4666,
    -2.9086E-11, -6.6774E-11, -12392.0631, -2.8373E-11, -6.3911E-11, -12069.6597,
    -2.7659E-11, -6.1048E-11, -11747.2562, -2.6946E-11, -5.8185E-11, -11424.8527,
    -2.6233E-11, -5.5321E-11, -11102.4493, -2.552E-11, -5.2458E-11, -10780.0458,
    -2.4806E-11, -4.9595E-11, -10457.6424, -2.4093E-11, -4.6732E-11, -10135.2389,
    -2.338E-11, -4.3869E-11, -9812.8355, -2.2666E-11, -4.1005E-11, -9490.432,
    -2.1953E-11, -3.8142E-11, -9168.0285, -2.124E-11, -3.5279E-11, -8845.6251,
    -2.0527E-11, -3.2416E-11, -8523.2216, -1.9813E-11, -2.9552E-11, -8200.8182,
    -1.91E-11, -2.6689E-11, -7878.4147, -1.8387E-11, -2.3826E-11, -7556.0112,
    -1.7673E-11, -2.0963E-11, -27233.6078 };

  static const real_T dv[60] = { 0.071699, -9.2684E-17, -0.071699, 0.051806,
    -7.2586E-17, -0.051806, 0.036583, -5.7412E-17, -0.036583, 0.024971,
    -4.61E-17, -0.024971, 0.016148, -3.7835E-17, -0.016148, 0.0094766,
    -3.2006E-17, -0.0094766, 0.0044653, -2.8155E-17, -0.0044653, 0.00073225,
    -2.5955E-17, -0.00073225, -0.0020179, -2.5186E-17, 0.0020179, -0.0040137,
    -2.5723E-17, 0.0040137, -0.0054319, -2.752E-17, 0.0054319, -0.0064091,
    -3.0616E-17, 0.0064091, -0.0070509, -3.5132E-17, 0.0070509, -0.0074387,
    -4.1276E-17, 0.0074387, -0.0076355, -4.9375E-17, 0.0076355, -0.0076896,
    -5.9859E-17, 0.0076896, -0.0076382, -7.3328E-17, 0.0076382, -0.0075101,
    -9.0564E-17, 0.0075101, -0.0073272, -1.126E-16, 0.0073272, -0.007106,
    -1.4077E-16, 0.007106 };

  static const int8_T A[7200] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T a[7200] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1 };

  static const int8_T c_a[7200] = { -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const int8_T iv[720] = { 1, 0, 0, 2, 0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 1, 0,
    0, 2, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 4, 0, 0,
    0, 1, 0, 0, 4, 0, 0, 0, 1, 0, 0, 4, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 1, 0, 0, 6, 0, 0, 0, 1, 0, 0, 6, 0, 0, 0, 1, 0, 0, 6, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 8, 0, 0, 0, 1, 0, 0, 8, 0,
    0, 0, 1, 0, 0, 8, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0,
    0, 10, 0, 0, 0, 1, 0, 0, 10, 0, 0, 0, 1, 0, 0, 10, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 12, 0, 0, 0, 1, 0, 0, 12, 0, 0, 0, 1, 0,
    0, 12, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 14, 0,
    0, 0, 1, 0, 0, 14, 0, 0, 0, 1, 0, 0, 14, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 1, 0, 0, 16, 0, 0, 0, 1, 0, 0, 16, 0, 0, 0, 1, 0, 0, 16, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 18, 0, 0, 0, 1,
    0, 0, 18, 0, 0, 0, 1, 0, 0, 18, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 1, 1, 0, 0, 20, 0, 0, 0, 1, 0, 0, 20, 0, 0, 0, 1, 0, 0, 20, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 22, 0, 0, 0, 1, 0, 0, 22,
    0, 0, 0, 1, 0, 0, 22, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 24, 0, 0, 0, 1, 0, 0, 24, 0, 0, 0, 1, 0, 0, 24, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 26, 0, 0, 0, 1, 0, 0, 26, 0, 0, 0,
    1, 0, 0, 26, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    28, 0, 0, 0, 1, 0, 0, 28, 0, 0, 0, 1, 0, 0, 28, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 30, 0, 0, 0, 1, 0, 0, 30, 0, 0, 0, 1, 0, 0,
    30, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 32, 0, 0,
    0, 1, 0, 0, 32, 0, 0, 0, 1, 0, 0, 32, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 1, 0, 0, 34, 0, 0, 0, 1, 0, 0, 34, 0, 0, 0, 1, 0, 0, 34, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 36, 0, 0, 0, 1, 0,
    0, 36, 0, 0, 0, 1, 0, 0, 36, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 1, 1, 0, 0, 38, 0, 0, 0, 1, 0, 0, 38, 0, 0, 0, 1, 0, 0, 38, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 40, 0, 0, 0, 1, 0, 0, 40, 0,
    0, 0, 1, 0, 0, 40, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 };

  real_T IGA[7200];
  real_T A_data[3600];
  double y_tmp[3600];
  real_T b_del_lam[240];
  real_T result[240];
  real_T varargin_1_data[240];
  real_T del_lam[120];
  real_T esig[120];
  real_T ftol[120];
  real_T igr2[120];
  real_T ilam[120];
  real_T lam[120];
  real_T mesil[120];
  real_T nlamt[120];
  real_T H[60];
  double del_z[60];
  real_T r1[60];
  real_T d;
  real_T mu;
  real_T mu_old;
  real_T ssq;
  int32_T b_i;
  int32_T exitflag;
  int32_T i;
  int32_T i1;
  int32_T idx;
  int32_T iy;
  int32_T unusedU0;
  uint8_T ii_data[240];
  boolean_T x[240];

  //  A=[1 0 0 2 0 0 ;0 1 0 0 2 0 ;0 0 1 0 0 2 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ]; 
  //  B=[0.20877 0 0 ;0 0.20877 0 ;0 0 0.20877 ;0.20877 0 0 ;0 0.20877 0 ;0 0 0.20877]; 
  //  Q_MPC=[100 0 0 0 0 0 ;0 100 0 0 0 0 ;0 0 100 0 0 0 ;0 0 0 100000 0 0 ;0 0 0 0 100000 0 ;0 0 0 0 0 100000]; 
  //  % P_MPC=[1567.5765 1.5888e-12 2.7019e-12 8536.6551 1.0896e-11 3.5199e-11 ;1.5888e-12 1567.5765 9.8891e-12 6.7324e-11 8536.6551 6.1982e-11 ;2.7019e-12 9.8891e-12 1567.5765 1.4677e-11 9.7557e-11 8536.6551 ;8536.6551 6.7324e-11 1.4677e-11 423802.2092 1.1408e-09 6.5756e-10 ;1.0896e-11 8536.6551 9.7557e-11 1.1408e-09 423802.2092 6.8455e-10 ;3.5199e-11 6.1982e-11 8536.6551 6.5756e-10 6.8455e-10 423802.2092 ;]; 
  //  R_MPC=[100 0 0 ;0 100 0 ;0 0 100];
  //  V_TRMPC=[0.25752 ;0.25752 ;0.25752 ;0.25752 ;0.25752 ;0.25752];
  //  Hp = 20;
  //  %  X_QP=[0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ]; 
  //  X_QP = zeros(3*Hp,1);
  //  solution matrices x_traj = A_tilde x_0 + B_tilde u_traj
  //  n=6;
  //  n = size(A,1);
  //  m=3;
  //  m= size(B,2);
  //  A_tilde = zeros((Hp)*n,n);
  //  % A_tilde = [1 0 0 2 0 0 ;0 1 0 0 2 0 ;0 0 1 0 0 2 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 4 0 0 ;0 1 0 0 4 0 ;0 0 1 0 0 4 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 6 0 0 ;0 1 0 0 6 0 ;0 0 1 0 0 6 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 8 0 0 ;0 1 0 0 8 0 ;0 0 1 0 0 8 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 10 0 0 ;0 1 0 0 10 0 ;0 0 1 0 0 10 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 12 0 0 ;0 1 0 0 12 0 ;0 0 1 0 0 12 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 14 0 0 ;0 1 0 0 14 0 ;0 0 1 0 0 14 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 16 0 0 ;0 1 0 0 16 0 ;0 0 1 0 0 16 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 18 0 0 ;0 1 0 0 18 0 ;0 0 1 0 0 18 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 20 0 0 ;0 1 0 0 20 0 ;0 0 1 0 0 20 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 22 0 0 ;0 1 0 0 22 0 ;0 0 1 0 0 22 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 24 0 0 ;0 1 0 0 24 0 ;0 0 1 0 0 24 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 26 0 0 ;0 1 0 0 26 0 ;0 0 1 0 0 26 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 28 0 0 ;0 1 0 0 28 0 ;0 0 1 0 0 28 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 30 0 0 ;0 1 0 0 30 0 ;0 0 1 0 0 30 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 32 0 0 ;0 1 0 0 32 0 ;0 0 1 0 0 32 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 34 0 0 ;0 1 0 0 34 0 ;0 0 1 0 0 34 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 36 0 0 ;0 1 0 0 36 0 ;0 0 1 0 0 36 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 38 0 0 ;0 1 0 0 38 0 ;0 0 1 0 0 38 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;1 0 0 40 0 0 ;0 1 0 0 40 0 ;0 0 1 0 0 40 ;0 0 0 1 0 0 ;0 0 0 0 1 0 ;0 0 0 0 0 1 ;]; 
  //  A_tilde(1:n,:) = A;
  //  for tt=1:Hp-1
  //    A_tilde(n*tt+1:n*(tt+1),:) = A * A_tilde(n*(tt-1)+1:n*tt,:);
  //  end
  //  B_tilde = zeros(length(A)*Hp,size(B,2)*Hp);
  //
  //  for ii = 1:Hp
  //      for jj = 1:ii
  //              B_tilde(n*(ii-1)+1:n*ii,m*(jj-1)+1:m*(jj)) = (A^(ii-jj))*B;
  //      end
  //  end
  //  % B_tilde=[0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 0 ;3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 ;0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 ;0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 0 ;3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 0 ;0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 0 ;0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 0 ;3.6706 0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 0 ;0 3.6706 0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 0 ;0 0 3.6706 0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 0 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0 ;3.869 0 0 3.6706 0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 0 ;0 3.869 0 0 3.6706 0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 0 ;0 0 3.869 0 0 3.6706 0 0 3.4722 0 0 3.2738 0 0 3.0754 0 0 2.877 0 0 2.6786 0 0 2.4802 0 0 2.2817 0 0 2.0833 0 0 1.8849 0 0 1.6865 0 0 1.4881 0 0 1.2897 0 0 1.0913 0 0 0.89285 0 0 0.69444 0 0 0.49603 0 0 0.29762 0 0 0.099206 ;0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 ;0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 ;0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 0 0 0.099206 ;]; 
  //  cost
  //  Q_tilde = blkdiag(kron(eye(Hp-1),Q_MPC),P_MPC);
  //  % Q_tilde = [70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1567.5765 1.5888e-12 2.7019e-12 8536.6551 1.0896e-11 3.5199e-11 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1.5888e-12 1567.5765 9.8891e-12 6.7324e-11 8536.6551 6.1982e-11 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 2.7019e-12 9.8891e-12 1567.5765 1.4677e-11 9.7557e-11 8536.6551 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8536.6551 6.7324e-11 1.4677e-11 423802.2092 1.1408e-09 6.5756e-10 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1.0896e-11 8536.6551 9.7557e-11 1.1408e-09 423802.2092 6.8455e-10 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 3.5199e-11 6.1982e-11 8536.6551 6.5756e-10 6.8455e-10 423802.2092 ;]; 
  //  R_tilde = kron(eye(Hp),R_MPC);
  //  R_tilde=[100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100 ]; 
  //  F_tilde = kron([ones(m-1,1);zeros(m*(Hp-1)+1,1)],F_MPC);
  //  L3 = blkdiag(kron(eye(Hp-1),Q_MPC),zeros(n,n));
  //  % L3=[70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 70 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 100000 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;]; 
  //  L4 = zeros(n,Hp*n);
  //  L4(:,end-(n-1):end) = eye(n);
  //  % L4 = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ;]; 
  //  PHI = blkdiag(kron(eye(Hp),eye(6,6)));
  //  % PHI = [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ;]; 
  //  constraints
  //  CONTROL INPUTS
  //  Define and Solve QP
  //  GAMMA = zeros(n*Hp,1);%Dimension of GAMMA is n*horizon x n
  //  for i = 1:Hp
  //      GAMMA(i*n-5:i*n,1) = xfinal;
  //  end
  //  % GAMMA = [0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;0 ;]; 
  //  QQ = 2*R_tilde + 2*OMEGA'*Q_tilde*OMEGA;
  //  QQ = [488965.6068 0 0 462030.3957 0 0 435330.0526 0 0 408699.4453 0 0 382173.4419 0 0 355786.9101 0 0 329574.718 0 0 303571.7335 0 0 277812.8244 0 0 252332.8589 0 0 227166.7047 0 0 202349.2298 0 0 177915.3021 0 0 153899.7896 0 0 130337.5602 0 0 107263.4818 0 0 84712.4224 0 0 62719.2499 0 0 41318.8321 0 0 20546.0372 0 0 ;0 488965.6068 0 0 462030.3957 0 0 435330.0526 0 0 408699.4453 0 0 382173.4419 0 0 355786.9101 0 0 329574.718 0 0 303571.7335 0 0 277812.8244 0 0 252332.8589 0 0 227166.7047 0 0 202349.2298 0 0 177915.3021 0 0 153899.7896 0 0 130337.5602 0 0 107263.4818 0 0 84712.4224 0 0 62719.2499 0 0 41318.8321 0 0 20546.0372 0 ;0 0 488965.6068 0 0 462030.3957 0 0 435330.0526 0 0 408699.4453 0 0 382173.4419 0 0 355786.9101 0 0 329574.718 0 0 303571.7335 0 0 277812.8244 0 0 252332.8589 0 0 227166.7047 0 0 202349.2298 0 0 177915.3021 0 0 153899.7896 0 0 130337.5602 0 0 107263.4818 0 0 84712.4224 0 0 62719.2499 0 0 41318.8321 0 0 20546.0372 ;462030.3957 0 0 445435.4433 0 0 419732.2252 0 0 394263.875 0 0 368865.2607 0 0 343571.2502 0 0 318416.7114 0 0 293436.5123 0 0 268665.5207 0 0 244138.6047 0 0 219890.6321 0 0 195956.4709 0 0 172370.989 0 0 149169.0543 0 0 126385.5347 0 0 104055.2983 0 0 82213.2129 0 0 60894.1465 0 0 40132.9669 0 0 19964.5421 0 0 ;0 462030.3957 0 0 445435.4433 0 0 419732.2252 0 0 394263.875 0 0 368865.2607 0 0 343571.2502 0 0 318416.7114 0 0 293436.5123 0 0 268665.5207 0 0 244138.6047 0 0 219890.6321 0 0 195956.4709 0 0 172370.989 0 0 149169.0543 0 0 126385.5347 0 0 104055.2983 0 0 82213.2129 0 0 60894.1465 0 0 40132.9669 0 0 19964.5421 0 ;0 0 462030.3957 0 0 445435.4433 0 0 419732.2252 0 0 394263.875 0 0 368865.2607 0 0 343571.2502 0 0 318416.7114 0 0 293436.5123 0 0 268665.5207 0 0 244138.6047 0 0 219890.6321 0 0 195956.4709 0 0 172370.989 0 0 149169.0543 0 0 126385.5347 0 0 104055.2983 0 0 82213.2129 0 0 60894.1465 0 0 40132.9669 0 0 19964.5421 ;435330.0526 0 0 419732.2252 0 0 404334.3977 0 0 379828.3047 0 0 355557.0796 0 0 331355.5903 0 0 307258.7049 0 0 283301.2911 0 0 259518.217 0 0 235944.3505 0 0 212614.5595 0 0 189563.712 0 0 166826.6758 0 0 144438.3189 0 0 122433.5093 0 0 100847.1148 0 0 79714.0034 0 0 59069.0431 0 0 38947.1016 0 0 19383.0471 0 0 ;0 435330.0526 0 0 419732.2252 0 0 404334.3977 0 0 379828.3047 0 0 355557.0796 0 0 331355.5903 0 0 307258.7049 0 0 283301.2911 0 0 259518.217 0 0 235944.3505 0 0 212614.5595 0 0 189563.712 0 0 166826.6758 0 0 144438.3189 0 0 122433.5093 0 0 100847.1148 0 0 79714.0034 0 0 59069.0431 0 0 38947.1016 0 0 19383.0471 0 ;0 0 435330.0526 0 0 419732.2252 0 0 404334.3977 0 0 379828.3047 0 0 355557.0796 0 0 331355.5903 0 0 307258.7049 0 0 283301.2911 0 0 259518.217 0 0 235944.3505 0 0 212614.5595 0 0 189563.712 0 0 166826.6758 0 0 144438.3189 0 0 122433.5093 0 0 100847.1148 0 0 79714.0034 0 0 59069.0431 0 0 38947.1016 0 0 19383.0471 ;408699.4453 0 0 394263.875 0 0 379828.3047 0 0 365592.7343 0 0 342248.8984 0 0 319139.9304 0 0 296100.6983 0 0 273166.0699 0 0 250370.9133 0 0 227750.0963 0 0 205338.4869 0 0 183170.9531 0 0 161282.3626 0 0 139707.5836 0 0 118481.4838 0 0 97638.9313 0 0 77214.7939 0 0 57243.9396 0 0 37761.2364 0 0 18801.5521 0 0 ;0 408699.4453 0 0 394263.875 0 0 379828.3047 0 0 365592.7343 0 0 342248.8984 0 0 319139.9304 0 0 296100.6983 0 0 273166.0699 0 0 250370.9133 0 0 227750.0963 0 0 205338.4869 0 0 183170.9531 0 0 161282.3626 0 0 139707.5836 0 0 118481.4838 0 0 97638.9313 0 0 77214.7939 0 0 57243.9396 0 0 37761.2364 0 0 18801.5521 0 ;0 0 408699.4453 0 0 394263.875 0 0 379828.3047 0 0 365592.7343 0 0 342248.8984 0 0 319139.9304 0 0 296100.6983 0 0 273166.0699 0 0 250370.9133 0 0 227750.0963 0 0 205338.4869 0 0 183170.9531 0 0 161282.3626 0 0 139707.5836 0 0 118481.4838 0 0 97638.9313 0 0 77214.7939 0 0 57243.9396 0 0 37761.2364 0 0 18801.5521 ;382173.4419 0 0 368865.2607 0 0 355557.0796 0 0 342248.8984 0 0 329140.7173 0 0 306924.2705 0 0 284942.6917 0 0 263030.8488 0 0 241223.6096 0 0 219555.8422 0 0 198062.4144 0 0 176778.1942 0 0 155738.0495 0 0 134976.8482 0 0 114529.4583 0 0 94430.7478 0 0 74715.5844 0 0 55418.8362 0 0 36575.3711 0 0 18220.0571 0 0 ;0 382173.4419 0 0 368865.2607 0 0 355557.0796 0 0 342248.8984 0 0 329140.7173 0 0 306924.2705 0 0 284942.6917 0 0 263030.8488 0 0 241223.6096 0 0 219555.8422 0 0 198062.4144 0 0 176778.1942 0 0 155738.0495 0 0 134976.8482 0 0 114529.4583 0 0 94430.7478 0 0 74715.5844 0 0 55418.8362 0 0 36575.3711 0 0 18220.0571 0 ;0 0 382173.4419 0 0 368865.2607 0 0 355557.0796 0 0 342248.8984 0 0 329140.7173 0 0 306924.2705 0 0 284942.6917 0 0 263030.8488 0 0 241223.6096 0 0 219555.8422 0 0 198062.4144 0 0 176778.1942 0 0 155738.0495 0 0 134976.8482 0 0 114529.4583 0 0 94430.7478 0 0 74715.5844 0 0 55418.8362 0 0 36575.3711 0 0 18220.0571 ;355786.9101 0 0 343571.2502 0 0 331355.5903 0 0 319139.9304 0 0 306924.2705 0 0 294908.6106 0 0 273784.6851 0 0 252895.6276 0 0 232076.3059 0 0 211361.588 0 0 190786.3418 0 0 170385.4353 0 0 150193.7363 0 0 130246.1129 0 0 110577.4329 0 0 91222.5642 0 0 72216.3749 0 0 53593.7328 0 0 35389.5059 0 0 17638.5621 0 0 ;0 355786.9101 0 0 343571.2502 0 0 331355.5903 0 0 319139.9304 0 0 306924.2705 0 0 294908.6106 0 0 273784.6851 0 0 252895.6276 0 0 232076.3059 0 0 211361.588 0 0 190786.3418 0 0 170385.4353 0 0 150193.7363 0 0 130246.1129 0 0 110577.4329 0 0 91222.5642 0 0 72216.3749 0 0 53593.7328 0 0 35389.5059 0 0 17638.5621 0 ;0 0 355786.9101 0 0 343571.2502 0 0 331355.5903 0 0 319139.9304 0 0 306924.2705 0 0 294908.6106 0 0 273784.6851 0 0 252895.6276 0 0 232076.3059 0 0 211361.588 0 0 190786.3418 0 0 170385.4353 0 0 150193.7363 0 0 130246.1129 0 0 110577.4329 0 0 91222.5642 0 0 72216.3749 0 0 53593.7328 0 0 35389.5059 0 0 17638.5621 ;329574.718 0 0 318416.7114 0 0 307258.7049 0 0 296100.6983 0 0 284942.6917 0 0 273784.6851 0 0 262826.6786 0 0 242760.4064 0 0 222929.0022 0 0 203167.3338 0 0 183510.2692 0 0 163992.6764 0 0 144649.4231 0 0 125515.3775 0 0 106625.4074 0 0 88014.3807 0 0 69717.1654 0 0 51768.6294 0 0 34203.6407 0 0 17057.067 0 0 ;0 329574.718 0 0 318416.7114 0 0 307258.7049 0 0 296100.6983 0 0 284942.6917 0 0 273784.6851 0 0 262826.6786 0 0 242760.4064 0 0 222929.0022 0 0 203167.3338 0 0 183510.2692 0 0 163992.6764 0 0 144649.4231 0 0 125515.3775 0 0 106625.4074 0 0 88014.3807 0 0 69717.1654 0 0 51768.6294 0 0 34203.6407 0 0 17057.067 0 ;0 0 329574.718 0 0 318416.7114 0 0 307258.7049 0 0 296100.6983 0 0 284942.6917 0 0 273784.6851 0 0 262826.6786 0 0 242760.4064 0 0 222929.0022 0 0 203167.3338 0 0 183510.2692 0 0 163992.6764 0 0 144649.4231 0 0 125515.3775 0 0 106625.4074 0 0 88014.3807 0 0 69717.1654 0 0 51768.6294 0 0 34203.6407 0 0 17057.067 ;303571.7335 0 0 293436.5123 0 0 283301.2911 0 0 273166.0699 0 0 263030.8488 0 0 252895.6276 0 0 242760.4064 0 0 232825.1852 0 0 213781.6985 0 0 194973.0796 0 0 176234.1966 0 0 157599.9174 0 0 139105.11 0 0 120784.6422 0 0 102673.3819 0 0 84806.1972 0 0 67217.9559 0 0 49943.526 0 0 33017.7754 0 0 16475.572 0 0 ;0 303571.7335 0 0 293436.5123 0 0 283301.2911 0 0 273166.0699 0 0 263030.8488 0 0 252895.6276 0 0 242760.4064 0 0 232825.1852 0 0 213781.6985 0 0 194973.0796 0 0 176234.1966 0 0 157599.9174 0 0 139105.11 0 0 120784.6422 0 0 102673.3819 0 0 84806.1972 0 0 67217.9559 0 0 49943.526 0 0 33017.7754 0 0 16475.572 0 ;0 0 303571.7335 0 0 293436.5123 0 0 283301.2911 0 0 273166.0699 0 0 263030.8488 0 0 252895.6276 0 0 242760.4064 0 0 232825.1852 0 0 213781.6985 0 0 194973.0796 0 0 176234.1966 0 0 157599.9174 0 0 139105.11 0 0 120784.6422 0 0 102673.3819 0 0 84806.1972 0 0 67217.9559 0 0 49943.526 0 0 33017.7754 0 0 16475.572 ;277812.8244 0 0 268665.5207 0 0 259518.217 0 0 250370.9133 0 0 241223.6096 0 0 232076.3059 0 0 222929.0022 0 0 213781.6985 0 0 204834.3947 0 0 186778.8254 0 0 168958.1241 0 0 151207.1585 0 0 133560.7968 0 0 116053.9068 0 0 98721.3565 0 0 81598.0137 0 0 64718.7464 0 0 48118.4226 0 0 31831.9102 0 0 15894.077 0 0 ;0 277812.8244 0 0 268665.5207 0 0 259518.217 0 0 250370.9133 0 0 241223.6096 0 0 232076.3059 0 0 222929.0022 0 0 213781.6985 0 0 204834.3947 0 0 186778.8254 0 0 168958.1241 0 0 151207.1585 0 0 133560.7968 0 0 116053.9068 0 0 98721.3565 0 0 81598.0137 0 0 64718.7464 0 0 48118.4226 0 0 31831.9102 0 0 15894.077 0 ;0 0 277812.8244 0 0 268665.5207 0 0 259518.217 0 0 250370.9133 0 0 241223.6096 0 0 232076.3059 0 0 222929.0022 0 0 213781.6985 0 0 204834.3947 0 0 186778.8254 0 0 168958.1241 0 0 151207.1585 0 0 133560.7968 0 0 116053.9068 0 0 98721.3565 0 0 81598.0137 0 0 64718.7464 0 0 48118.4226 0 0 31831.9102 0 0 15894.077 ;252332.8589 0 0 244138.6047 0 0 235944.3505 0 0 227750.0963 0 0 219555.8422 0 0 211361.588 0 0 203167.3338 0 0 194973.0796 0 0 186778.8254 0 0 178784.5713 0 0 161682.0515 0 0 144814.3996 0 0 128016.4836 0 0 111323.1715 0 0 94769.331 0 0 78389.8302 0 0 62219.5369 0 0 46293.3192 0 0 30646.0449 0 0 15312.582 0 0 ;0 252332.8589 0 0 244138.6047 0 0 235944.3505 0 0 227750.0963 0 0 219555.8422 0 0 211361.588 0 0 203167.3338 0 0 194973.0796 0 0 186778.8254 0 0 178784.5713 0 0 161682.0515 0 0 144814.3996 0 0 128016.4836 0 0 111323.1715 0 0 94769.331 0 0 78389.8302 0 0 62219.5369 0 0 46293.3192 0 0 30646.0449 0 0 15312.582 0 ;0 0 252332.8589 0 0 244138.6047 0 0 235944.3505 0 0 227750.0963 0 0 219555.8422 0 0 211361.588 0 0 203167.3338 0 0 194973.0796 0 0 186778.8254 0 0 178784.5713 0 0 161682.0515 0 0 144814.3996 0 0 128016.4836 0 0 111323.1715 0 0 94769.331 0 0 78389.8302 0 0 62219.5369 0 0 46293.3192 0 0 30646.0449 0 0 15312.582 ;227166.7047 0 0 219890.6321 0 0 212614.5595 0 0 205338.4869 0 0 198062.4144 0 0 190786.3418 0 0 183510.2692 0 0 176234.1966 0 0 168958.1241 0 0 161682.0515 0 0 154605.9789 0 0 138421.6407 0 0 122472.1705 0 0 106592.4361 0 0 90817.3055 0 0 75181.6466 0 0 59720.3274 0 0 44468.2158 0 0 29460.1797 0 0 14731.087 0 0 ;0 227166.7047 0 0 219890.6321 0 0 212614.5595 0 0 205338.4869 0 0 198062.4144 0 0 190786.3418 0 0 183510.2692 0 0 176234.1966 0 0 168958.1241 0 0 161682.0515 0 0 154605.9789 0 0 138421.6407 0 0 122472.1705 0 0 106592.4361 0 0 90817.3055 0 0 75181.6466 0 0 59720.3274 0 0 44468.2158 0 0 29460.1797 0 0 14731.087 0 ;0 0 227166.7047 0 0 219890.6321 0 0 212614.5595 0 0 205338.4869 0 0 198062.4144 0 0 190786.3418 0 0 183510.2692 0 0 176234.1966 0 0 168958.1241 0 0 161682.0515 0 0 154605.9789 0 0 138421.6407 0 0 122472.1705 0 0 106592.4361 0 0 90817.3055 0 0 75181.6466 0 0 59720.3274 0 0 44468.2158 0 0 29460.1797 0 0 14731.087 ;202349.2298 0 0 195956.4709 0 0 189563.712 0 0 183170.9531 0 0 176778.1942 0 0 170385.4353 0 0 163992.6764 0 0 157599.9174 0 0 151207.1585 0 0 144814.3996 0 0 138421.6407 0 0 132228.8818 0 0 116927.8573 0 0 101861.7008 0 0 86865.28 0 0 71973.4631 0 0 57221.1179 0 0 42643.1124 0 0 28274.3144 0 0 14149.592 0 0 ;0 202349.2298 0 0 195956.4709 0 0 189563.712 0 0 183170.9531 0 0 176778.1942 0 0 170385.4353 0 0 163992.6764 0 0 157599.9174 0 0 151207.1585 0 0 144814.3996 0 0 138421.6407 0 0 132228.8818 0 0 116927.8573 0 0 101861.7008 0 0 86865.28 0 0 71973.4631 0 0 57221.1179 0 0 42643.1124 0 0 28274.3144 0 0 14149.592 0 ;0 0 202349.2298 0 0 195956.4709 0 0 189563.712 0 0 183170.9531 0 0 176778.1942 0 0 170385.4353 0 0 163992.6764 0 0 157599.9174 0 0 151207.1585 0 0 144814.3996 0 0 138421.6407 0 0 132228.8818 0 0 116927.8573 0 0 101861.7008 0 0 86865.28 0 0 71973.4631 0 0 57221.1179 0 0 42643.1124 0 0 28274.3144 0 0 14149.592 ;177915.3021 0 0 172370.989 0 0 166826.6758 0 0 161282.3626 0 0 155738.0495 0 0 150193.7363 0 0 144649.4231 0 0 139105.11 0 0 133560.7968 0 0 128016.4836 0 0 122472.1705 0 0 116927.8573 0 0 111583.5442 0 0 97130.9654 0 0 82913.2546 0 0 68765.2796 0 0 54721.9084 0 0 40818.009 0 0 27088.4492 0 0 13568.0969 0 0 ;0 177915.3021 0 0 172370.989 0 0 166826.6758 0 0 161282.3626 0 0 155738.0495 0 0 150193.7363 0 0 144649.4231 0 0 139105.11 0 0 133560.7968 0 0 128016.4836 0 0 122472.1705 0 0 116927.8573 0 0 111583.5442 0 0 97130.9654 0 0 82913.2546 0 0 68765.2796 0 0 54721.9084 0 0 40818.009 0 0 27088.4492 0 0 13568.0969 0 ;0 0 177915.3021 0 0 172370.989 0 0 166826.6758 0 0 161282.3626 0 0 155738.0495 0 0 150193.7363 0 0 144649.4231 0 0 139105.11 0 0 133560.7968 0 0 128016.4836 0 0 122472.1705 0 0 116927.8573 0 0 111583.5442 0 0 97130.9654 0 0 82913.2546 0 0 68765.2796 0 0 54721.9084 0 0 40818.009 0 0 27088.4492 0 0 13568.0969 ;153899.7896 0 0 149169.0543 0 0 144438.3189 0 0 139707.5836 0 0 134976.8482 0 0 130246.1129 0 0 125515.3775 0 0 120784.6422 0 0 116053.9068 0 0 111323.1715 0 0 106592.4361 0 0 101861.7008 0 0 97130.9654 0 0 92600.23 0 0 78961.2291 0 0 65557.0961 0 0 52222.6989 0 0 38992.9056 0 0 25902.5839 0 0 12986.6019 0 0 ;0 153899.7896 0 0 149169.0543 0 0 144438.3189 0 0 139707.5836 0 0 134976.8482 0 0 130246.1129 0 0 125515.3775 0 0 120784.6422 0 0 116053.9068 0 0 111323.1715 0 0 106592.4361 0 0 101861.7008 0 0 97130.9654 0 0 92600.23 0 0 78961.2291 0 0 65557.0961 0 0 52222.6989 0 0 38992.9056 0 0 25902.5839 0 0 12986.6019 0 ;0 0 153899.7896 0 0 149169.0543 0 0 144438.3189 0 0 139707.5836 0 0 134976.8482 0 0 130246.1129 0 0 125515.3775 0 0 120784.6422 0 0 116053.9068 0 0 111323.1715 0 0 106592.4361 0 0 101861.7008 0 0 97130.9654 0 0 92600.23 0 0 78961.2291 0 0 65557.0961 0 0 52222.6989 0 0 38992.9056 0 0 25902.5839 0 0 12986.6019 ;130337.5602 0 0 126385.5347 0 0 122433.5093 0 0 118481.4838 0 0 114529.4583 0 0 110577.4329 0 0 106625.4074 0 0 102673.3819 0 0 98721.3565 0 0 94769.331 0 0 90817.3055 0 0 86865.28 0 0 82913.2546 0 0 78961.2291 0 0 75209.2036 0 0 62348.9126 0 0 49723.4894 0 0 37167.8021 0 0 24716.7187 0 0 12405.1069 0 0 ;0 130337.5602 0 0 126385.5347 0 0 122433.5093 0 0 118481.4838 0 0 114529.4583 0 0 110577.4329 0 0 106625.4074 0 0 102673.3819 0 0 98721.3565 0 0 94769.331 0 0 90817.3055 0 0 86865.28 0 0 82913.2546 0 0 78961.2291 0 0 75209.2036 0 0 62348.9126 0 0 49723.4894 0 0 37167.8021 0 0 24716.7187 0 0 12405.1069 0 ;0 0 130337.5602 0 0 126385.5347 0 0 122433.5093 0 0 118481.4838 0 0 114529.4583 0 0 110577.4329 0 0 106625.4074 0 0 102673.3819 0 0 98721.3565 0 0 94769.331 0 0 90817.3055 0 0 86865.28 0 0 82913.2546 0 0 78961.2291 0 0 75209.2036 0 0 62348.9126 0 0 49723.4894 0 0 37167.8021 0 0 24716.7187 0 0 12405.1069 ;107263.4818 0 0 104055.2983 0 0 100847.1148 0 0 97638.9313 0 0 94430.7478 0 0 91222.5642 0 0 88014.3807 0 0 84806.1972 0 0 81598.0137 0 0 78389.8302 0 0 75181.6466 0 0 71973.4631 0 0 68765.2796 0 0 65557.0961 0 0 62348.9126 0 0 59340.729 0 0 47224.2799 0 0 35342.6987 0 0 23530.8534 0 0 11823.6119 0 0 ;0 107263.4818 0 0 104055.2983 0 0 100847.1148 0 0 97638.9313 0 0 94430.7478 0 0 91222.5642 0 0 88014.3807 0 0 84806.1972 0 0 81598.0137 0 0 78389.8302 0 0 75181.6466 0 0 71973.4631 0 0 68765.2796 0 0 65557.0961 0 0 62348.9126 0 0 59340.729 0 0 47224.2799 0 0 35342.6987 0 0 23530.8534 0 0 11823.6119 0 ;0 0 107263.4818 0 0 104055.2983 0 0 100847.1148 0 0 97638.9313 0 0 94430.7478 0 0 91222.5642 0 0 88014.3807 0 0 84806.1972 0 0 81598.0137 0 0 78389.8302 0 0 75181.6466 0 0 71973.4631 0 0 68765.2796 0 0 65557.0961 0 0 62348.9126 0 0 59340.729 0 0 47224.2799 0 0 35342.6987 0 0 23530.8534 0 0 11823.6119 ;84712.4224 0 0 82213.2129 0 0 79714.0034 0 0 77214.7939 0 0 74715.5844 0 0 72216.3749 0 0 69717.1654 0 0 67217.9559 0 0 64718.7464 0 0 62219.5369 0 0 59720.3274 0 0 57221.1179 0 0 54721.9084 0 0 52222.6989 0 0 49723.4894 0 0 47224.2799 0 0 44925.0704 0 0 33517.5953 0 0 22344.9882 0 0 11242.1169 0 0 ;0 84712.4224 0 0 82213.2129 0 0 79714.0034 0 0 77214.7939 0 0 74715.5844 0 0 72216.3749 0 0 69717.1654 0 0 67217.9559 0 0 64718.7464 0 0 62219.5369 0 0 59720.3274 0 0 57221.1179 0 0 54721.9084 0 0 52222.6989 0 0 49723.4894 0 0 47224.2799 0 0 44925.0704 0 0 33517.5953 0 0 22344.9882 0 0 11242.1169 0 ;0 0 84712.4224 0 0 82213.2129 0 0 79714.0034 0 0 77214.7939 0 0 74715.5844 0 0 72216.3749 0 0 69717.1654 0 0 67217.9559 0 0 64718.7464 0 0 62219.5369 0 0 59720.3274 0 0 57221.1179 0 0 54721.9084 0 0 52222.6989 0 0 49723.4894 0 0 47224.2799 0 0 44925.0704 0 0 33517.5953 0 0 22344.9882 0 0 11242.1169 ;62719.2499 0 0 60894.1465 0 0 59069.0431 0 0 57243.9396 0 0 55418.8362 0 0 53593.7328 0 0 51768.6294 0 0 49943.526 0 0 48118.4226 0 0 46293.3192 0 0 44468.2158 0 0 42643.1124 0 0 40818.009 0 0 38992.9056 0 0 37167.8021 0 0 35342.6987 0 0 33517.5953 0 0 31892.4919 0 0 21159.1229 0 0 10660.6218 0 0 ;0 62719.2499 0 0 60894.1465 0 0 59069.0431 0 0 57243.9396 0 0 55418.8362 0 0 53593.7328 0 0 51768.6294 0 0 49943.526 0 0 48118.4226 0 0 46293.3192 0 0 44468.2158 0 0 42643.1124 0 0 40818.009 0 0 38992.9056 0 0 37167.8021 0 0 35342.6987 0 0 33517.5953 0 0 31892.4919 0 0 21159.1229 0 0 10660.6218 0 ;0 0 62719.2499 0 0 60894.1465 0 0 59069.0431 0 0 57243.9396 0 0 55418.8362 0 0 53593.7328 0 0 51768.6294 0 0 49943.526 0 0 48118.4226 0 0 46293.3192 0 0 44468.2158 0 0 42643.1124 0 0 40818.009 0 0 38992.9056 0 0 37167.8021 0 0 35342.6987 0 0 33517.5953 0 0 31892.4919 0 0 21159.1229 0 0 10660.6218 ;41318.8321 0 0 40132.9669 0 0 38947.1016 0 0 37761.2364 0 0 36575.3711 0 0 35389.5059 0 0 34203.6407 0 0 33017.7754 0 0 31831.9102 0 0 30646.0449 0 0 29460.1797 0 0 28274.3144 0 0 27088.4492 0 0 25902.5839 0 0 24716.7187 0 0 23530.8534 0 0 22344.9882 0 0 21159.1229 0 0 20173.2577 0 0 10079.1268 0 0 ;0 41318.8321 0 0 40132.9669 0 0 38947.1016 0 0 37761.2364 0 0 36575.3711 0 0 35389.5059 0 0 34203.6407 0 0 33017.7754 0 0 31831.9102 0 0 30646.0449 0 0 29460.1797 0 0 28274.3144 0 0 27088.4492 0 0 25902.5839 0 0 24716.7187 0 0 23530.8534 0 0 22344.9882 0 0 21159.1229 0 0 20173.2577 0 0 10079.1268 0 ;0 0 41318.8321 0 0 40132.9669 0 0 38947.1016 0 0 37761.2364 0 0 36575.3711 0 0 35389.5059 0 0 34203.6407 0 0 33017.7754 0 0 31831.9102 0 0 30646.0449 0 0 29460.1797 0 0 28274.3144 0 0 27088.4492 0 0 25902.5839 0 0 24716.7187 0 0 23530.8534 0 0 22344.9882 0 0 21159.1229 0 0 20173.2577 0 0 10079.1268 ;20546.0372 0 0 19964.5421 0 0 19383.0471 0 0 18801.5521 0 0 18220.0571 0 0 17638.5621 0 0 17057.067 0 0 16475.572 0 0 15894.077 0 0 15312.582 0 0 14731.087 0 0 14149.592 0 0 13568.0969 0 0 12986.6019 0 0 12405.1069 0 0 11823.6119 0 0 11242.1169 0 0 10660.6218 0 0 10079.1268 0 0 9697.6318 0 0 ;0 20546.0372 0 0 19964.5421 0 0 19383.0471 0 0 18801.5521 0 0 18220.0571 0 0 17638.5621 0 0 17057.067 0 0 16475.572 0 0 15894.077 0 0 15312.582 0 0 14731.087 0 0 14149.592 0 0 13568.0969 0 0 12986.6019 0 0 12405.1069 0 0 11823.6119 0 0 11242.1169 0 0 10660.6218 0 0 10079.1268 0 0 9697.6318 0 ;0 0 20546.0372 0 0 19964.5421 0 0 19383.0471 0 0 18801.5521 0 0 18220.0571 0 0 17638.5621 0 0 17057.067 0 0 16475.572 0 0 15894.077 0 0 15312.582 0 0 14731.087 0 0 14149.592 0 0 13568.0969 0 0 12986.6019 0 0 12405.1069 0 0 11823.6119 0 0 11242.1169 0 0 10660.6218 0 0 10079.1268 0 0 9697.6318]; 
  for (i = 0; i < 120; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      d += (2.0 * x0[i1]) * (static_cast<real_T>(iv[i1 + (6 * i)]));
    }

    lam[i] = d;
  }

  for (i = 0; i < 120; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 120; i1++) {
      d += lam[i1] * b[i1 + (120 * i)];
    }

    ftol[i] = d;
  }

  //  + F_tilde';
  //  L = [-eye(m*Hp);eye(m*Hp)];
  //  % L = [-1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -1 ;1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 ;0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 ;]; 
  //  b = [kron(ones(Hp,1), V_TRMPC(1:m,1)); kron(ones(Hp,1), V_TRMPC(m+1:end,1))]; 
  //  % b=[0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;0.22901 ;]; 
  //  Call QP Solver
  //  Call QP Solver
  //  QQ = (QQ+QQ')/2;
  //  % QQ = [138371.1425 1.3007e-10 1.3212e-10 112666.5366 1.2498e-10 1.274e-10 106967.442 1.1989e-10 1.2267e-10 101279.3704 1.148e-10 1.1794e-10 95607.833 1.0971e-10 1.1322e-10 89958.3413 1.0462e-10 1.0849e-10 84336.4067 9.953e-11 1.0377e-10 78747.5407 9.4441e-11 9.904e-11 73197.2546 8.9351e-11 9.4314e-11 67691.06 8.4261e-11 8.9588e-11 62234.4682 7.9172e-11 8.4862e-11 56832.9906 7.4082e-11 8.0136e-11 51492.1387 6.8992e-11 7.541e-11 46217.4239 6.3903e-11 7.0684e-11 41014.3576 5.8813e-11 6.5958e-11 35888.4513 5.3723e-11 6.1232e-11 30845.2164 4.8633e-11 5.6505e-11 25890.1642 4.3544e-11 5.1779e-11 21028.8063 3.8454e-11 4.7053e-11 16266.654 3.3364e-11 4.2327e-11 ;1.3007e-10 138371.1425 4.3201e-10 1.272e-10 112666.5366 4.1299e-10 1.2433e-10 106967.442 3.9397e-10 1.2146e-10 101279.3704 3.7494e-10 1.186e-10 95607.833 3.5592e-10 1.1573e-10 89958.3413 3.369e-10 1.1286e-10 84336.4067 3.1787e-10 1.0999e-10 78747.5407 2.9885e-10 1.0712e-10 73197.2546 2.7983e-10 1.0425e-10 67691.06 2.608e-10 1.0139e-10 62234.4682 2.4178e-10 9.8518e-11 56832.9906 2.2276e-10 9.5649e-11 51492.1387 2.0373e-10 9.2781e-11 46217.4239 1.8471e-10 8.9913e-11 41014.3576 1.6569e-10 8.7044e-11 35888.4513 1.4666e-10 8.4176e-11 30845.2164 1.2764e-10 8.1308e-11 25890.1642 1.0861e-10 7.844e-11 21028.8063 8.9591e-11 7.5571e-11 16266.654 7.0567e-11 ;1.3212e-10 4.3201e-10 138371.1425 1.2659e-10 4.1439e-10 112666.5366 1.2105e-10 3.9677e-10 106967.442 1.1552e-10 3.7915e-10 101279.3704 1.0999e-10 3.6152e-10 95607.833 1.0445e-10 3.439e-10 89958.3413 9.8919e-11 3.2628e-10 84336.4067 9.3385e-11 3.0865e-10 78747.5407 8.7851e-11 2.9103e-10 73197.2546 8.2317e-11 2.7341e-10 67691.06 7.6783e-11 2.5578e-10 62234.4682 7.1249e-11 2.3816e-10 56832.9906 6.5715e-11 2.2054e-10 51492.1387 6.0181e-11 2.0292e-10 46217.4239 5.4647e-11 1.8529e-10 41014.3576 4.9113e-11 1.6767e-10 35888.4513 4.3579e-11 1.5005e-10 30845.2164 3.8045e-11 1.3242e-10 25890.1642 3.2511e-11 1.148e-10 21028.8063 2.6977e-11 9.7177e-11 16266.654 ;112666.5366 1.272e-10 1.2659e-10 129154.3028 1.2224e-10 1.2208e-10 103675.0808 1.1727e-10 1.1756e-10 98201.3702 1.1231e-10 1.1305e-10 92738.6825 1.0734e-10 1.0854e-10 87292.529 1.0238e-10 1.0402e-10 81868.4212 9.7413e-11 9.9509e-11 76471.8706 9.2448e-11 9.4995e-11 71108.3885 8.7483e-11 9.0482e-11 65783.4864 8.2519e-11 8.5969e-11 60502.6757 7.7554e-11 8.1455e-11 55271.4678 7.259e-11 7.6942e-11 50095.3742 6.7625e-11 7.2429e-11 44979.9063 6.2661e-11 6.7915e-11 39930.5754 5.7696e-11 6.3402e-11 34952.8931 5.2731e-11 5.8889e-11 30052.3707 4.7767e-11 5.4375e-11 25234.5197 4.2802e-11 4.9862e-11 20504.8515 3.7838e-11 4.5348e-11 15868.8775 3.2873e-11 4.0835e-11 ;1.2498e-10 112666.5366 4.1439e-10 1.2224e-10 129154.3028 3.9615e-10 1.1949e-10 103675.0808 3.779e-10 1.1675e-10 98201.3702 3.5966e-10 1.1401e-10 92738.6825 3.4141e-10 1.1126e-10 87292.529 3.2317e-10 1.0852e-10 81868.4212 3.0492e-10 1.0578e-10 76471.8706 2.8668e-10 1.0303e-10 71108.3885 2.6843e-10 1.0029e-10 65783.4864 2.5019e-10 9.7547e-11 60502.6757 2.3194e-10 9.4804e-11 55271.4678 2.137e-10 9.2061e-11 50095.3742 1.9545e-10 8.9318e-11 44979.9063 1.7721e-10 8.6574e-11 39930.5754 1.5896e-10 8.3831e-11 34952.8931 1.4072e-10 8.1088e-11 30052.3707 1.2247e-10 7.8345e-11 25234.5197 1.0423e-10 7.5602e-11 20504.8515 8.5983e-11 7.2858e-11 15868.8775 6.7738e-11 ;1.274e-10 4.1299e-10 112666.5366 1.2208e-10 3.9615e-10 129154.3028 1.1675e-10 3.793e-10 103675.0808 1.1143e-10 3.6246e-10 98201.3702 1.0611e-10 3.4561e-10 92738.6825 1.0079e-10 3.2877e-10 87292.529 9.5469e-11 3.1192e-10 81868.4212 9.0148e-11 2.9508e-10 76471.8706 8.4827e-11 2.7824e-10 71108.3885 7.9506e-11 2.6139e-10 65783.4864 7.4184e-11 2.4455e-10 60502.6757 6.8863e-11 2.277e-10 55271.4678 6.3542e-11 2.1086e-10 50095.3742 5.8221e-11 1.9401e-10 44979.9063 5.2899e-11 1.7717e-10 39930.5754 4.7578e-11 1.6032e-10 34952.8931 4.2257e-11 1.4348e-10 30052.3707 3.6936e-11 1.2664e-10 25234.5197 3.1614e-11 1.0979e-10 20504.8515 2.6293e-11 9.2947e-11 15868.8775 ;106967.442 1.2433e-10 1.2105e-10 103675.0808 1.1949e-10 1.1675e-10 120382.7195 1.1465e-10 1.1245e-10 95123.37 1.0981e-10 1.0815e-10 89869.532 1.0497e-10 1.0385e-10 84626.7167 1.0013e-10 9.9552e-11 79400.4358 9.5295e-11 9.5251e-11 74196.2006 9.0455e-11 9.0951e-11 69019.5224 8.5616e-11 8.665e-11 63875.9129 8.0776e-11 8.2349e-11 58770.8833 7.5937e-11 7.8049e-11 53709.9451 7.1097e-11 7.3748e-11 48698.6097 6.6258e-11 6.9447e-11 43742.3886 6.1418e-11 6.5147e-11 38846.7932 5.6579e-11 6.0846e-11 34017.3348 5.1739e-11 5.6546e-11 29259.525 4.69e-11 5.2245e-11 24578.8751 4.206e-11 4.7944e-11 19980.8967 3.7221e-11 4.3644e-11 15471.101 3.2381e-11 3.9343e-11 ;1.1989e-10 106967.442 3.9677e-10 1.1727e-10 103675.0808 3.793e-10 1.1465e-10 120382.7195 3.6184e-10 1.1203e-10 95123.37 3.4437e-10 1.0942e-10 89869.532 3.269e-10 1.068e-10 84626.7167 3.0944e-10 1.0418e-10 79400.4358 2.9197e-10 1.0156e-10 74196.2006 2.745e-10 9.8944e-11 69019.5224 2.5704e-10 9.6326e-11 63875.9129 2.3957e-10 9.3708e-11 58770.8833 2.2211e-10 9.109e-11 53709.9451 2.0464e-10 8.8472e-11 48698.6097 1.8717e-10 8.5854e-11 43742.3886 1.6971e-10 8.3236e-11 38846.7932 1.5224e-10 8.0618e-11 34017.3348 1.3477e-10 7.8e-11 29259.525 1.1731e-10 7.5382e-11 24578.8751 9.9841e-11 7.2764e-11 19980.8967 8.2375e-11 7.0146e-11 15471.101 6.4909e-11 ;1.2267e-10 3.9397e-10 106967.442 1.1756e-10 3.779e-10 103675.0808 1.1245e-10 3.6184e-10 120382.7195 1.0735e-10 3.4577e-10 95123.37 1.0224e-10 3.297e-10 89869.532 9.7128e-11 3.1364e-10 84626.7167 9.202e-11 2.9757e-10 79400.4358 8.6911e-11 2.8151e-10 74196.2006 8.1803e-11 2.6544e-10 69019.5224 7.6694e-11 2.4938e-10 63875.9129 7.1586e-11 2.3331e-10 58770.8833 6.6477e-11 2.1724e-10 53709.9451 6.1368e-11 2.0118e-10 48698.6097 5.626e-11 1.8511e-10 43742.3886 5.1151e-11 1.6905e-10 38846.7932 4.6043e-11 1.5298e-10 34017.3348 4.0934e-11 1.3691e-10 29259.525 3.5826e-11 1.2085e-10 24578.8751 3.0717e-11 1.0478e-10 19980.8967 2.5609e-11 8.8717e-11 15471.101 ;101279.3704 1.2146e-10 1.1552e-10 98201.3702 1.1675e-10 1.1143e-10 95123.37 1.1203e-10 1.0735e-10 112045.3699 1.0732e-10 1.0326e-10 87000.3814 1.0261e-10 9.9169e-11 81960.9045 9.7892e-11 9.5081e-11 76932.4503 9.3177e-11 9.0994e-11 71920.5305 8.8463e-11 8.6906e-11 66930.6563 8.3748e-11 8.2818e-11 61968.3393 7.9034e-11 7.873e-11 57039.0908 7.432e-11 7.4642e-11 52148.4223 6.9605e-11 7.0554e-11 47301.8452 6.4891e-11 6.6466e-11 42504.871 6.0176e-11 6.2378e-11 37763.0109 5.5462e-11 5.829e-11 33081.7766 5.0748e-11 5.4203e-11 28466.6793 4.6033e-11 5.0115e-11 23923.2306 4.1319e-11 4.6027e-11 19456.9418 3.6604e-11 4.1939e-11 15073.3244 3.189e-11 3.7851e-11 ;1.148e-10 101279.3704 3.7915e-10 1.1231e-10 98201.3702 3.6246e-10 1.0981e-10 95123.37 3.4577e-10 1.0732e-10 112045.3699 3.2908e-10 1.0483e-10 87000.3814 3.1239e-10 1.0233e-10 81960.9045 2.9571e-10 9.9842e-11 76932.4503 2.7902e-10 9.7349e-11 71920.5305 2.6233e-10 9.4856e-11 66930.6563 2.4564e-10 9.2363e-11 61968.3393 2.2896e-10 8.987e-11 57039.0908 2.1227e-10 8.7377e-11 52148.4223 1.9558e-10 8.4884e-11 47301.8452 1.7889e-10 8.2391e-11 42504.871 1.6221e-10 7.9898e-11 37763.0109 1.4552e-10 7.7405e-11 33081.7766 1.2883e-10 7.4912e-11 28466.6793 1.1214e-10 7.2419e-11 23923.2306 9.5455e-11 6.9926e-11 19456.9418 7.8767e-11 6.7433e-11 15073.3244 6.2079e-11 ;1.1794e-10 3.7494e-10 101279.3704 1.1305e-10 3.5966e-10 98201.3702 1.0815e-10 3.4437e-10 95123.37 1.0326e-10 3.2908e-10 112045.3699 9.8361e-11 3.138e-10 87000.3814 9.3466e-11 2.9851e-10 81960.9045 8.857e-11 2.8322e-10 76932.4503 8.3674e-11 2.6793e-10 71920.5305 7.8778e-11 2.5265e-10 66930.6563 7.3883e-11 2.3736e-10 61968.3393 6.8987e-11 2.2207e-10 57039.0908 6.4091e-11 2.0678e-10 52148.4223 5.9195e-11 1.915e-10 47301.8452 5.4299e-11 1.7621e-10 42504.871 4.9404e-11 1.6092e-10 37763.0109 4.4508e-11 1.4564e-10 33081.7766 3.9612e-11 1.3035e-10 28466.6793 3.4716e-11 1.1506e-10 23923.2306 2.982e-11 9.9774e-11 19456.9418 2.4925e-11 8.4487e-11 15073.3244 ;95607.833 1.186e-10 1.0999e-10 92738.6825 1.1401e-10 1.0611e-10 89869.532 1.0942e-10 1.0224e-10 87000.3814 1.0483e-10 9.8361e-11 104131.2309 1.0024e-10 9.4486e-11 79295.0922 9.5649e-11 9.0611e-11 74464.4649 9.106e-11 8.6736e-11 69644.8604 8.647e-11 8.2861e-11 64841.7902 8.1881e-11 7.8986e-11 60060.7658 7.7292e-11 7.5111e-11 55307.2984 7.2702e-11 7.1235e-11 50586.8996 6.8113e-11 6.736e-11 45905.0807 6.3524e-11 6.3485e-11 41267.3533 5.8934e-11 5.961e-11 36679.2287 5.4345e-11 5.5735e-11 32146.2183 4.9756e-11 5.186e-11 27673.8337 4.5166e-11 4.7984e-11 23267.5861 4.0577e-11 4.4109e-11 18932.987 3.5988e-11 4.0234e-11 14675.5479 3.1398e-11 3.6359e-11 ;1.0971e-10 95607.833 3.6152e-10 1.0734e-10 92738.6825 3.4561e-10 1.0497e-10 89869.532 3.297e-10 1.0261e-10 87000.3814 3.138e-10 1.0024e-10 104131.2309 2.9789e-10 9.787e-11 79295.0922 2.8198e-10 9.5502e-11 74464.4649 2.6607e-10 9.3135e-11 69644.8604 2.5016e-10 9.0767e-11 64841.7902 2.3425e-10 8.8399e-11 60060.7658 2.1834e-10 8.6031e-11 55307.2984 2.0243e-10 8.3663e-11 50586.8996 1.8652e-10 8.1295e-11 45905.0807 1.7061e-10 7.8927e-11 41267.3533 1.547e-10 7.6559e-11 36679.2287 1.388e-10 7.4191e-11 32146.2183 1.2289e-10 7.1823e-11 27673.8337 1.0698e-10 6.9456e-11 23267.5861 9.1068e-11 6.7088e-11 18932.987 7.5159e-11 6.472e-11 14675.5479 5.925e-11 ;1.1322e-10 3.5592e-10 95607.833 1.0854e-10 3.4141e-10 92738.6825 1.0385e-10 3.269e-10 89869.532 9.9169e-11 3.1239e-10 87000.3814 9.4486e-11 2.9789e-10 104131.2309 8.9803e-11 2.8338e-10 79295.0922 8.512e-11 2.6887e-10 74464.4649 8.0437e-11 2.5436e-10 69644.8604 7.5754e-11 2.3985e-10 64841.7902 7.1071e-11 2.2534e-10 60060.7658 6.6388e-11 2.1083e-10 55307.2984 6.1705e-11 1.9633e-10 50586.8996 5.7022e-11 1.8182e-10 45905.0807 5.2339e-11 1.6731e-10 41267.3533 4.7656e-11 1.528e-10 36679.2287 4.2973e-11 1.3829e-10 32146.2183 3.829e-11 1.2378e-10 27673.8337 3.3607e-11 1.0927e-10 23267.5861 2.8924e-11 9.4766e-11 18932.987 2.4241e-11 8.0257e-11 14675.5479 ;89958.3413 1.1573e-10 1.0445e-10 87292.529 1.1126e-10 1.0079e-10 84626.7167 1.068e-10 9.7128e-11 81960.9045 1.0233e-10 9.3466e-11 79295.0922 9.787e-11 8.9803e-11 96629.2799 9.3406e-11 8.6141e-11 71996.4794 8.8942e-11 8.2478e-11 67369.1904 8.4478e-11 7.8816e-11 62752.9241 8.0013e-11 7.5154e-11 58153.1922 7.5549e-11 7.1491e-11 53575.5059 7.1085e-11 6.7829e-11 49025.3768 6.6621e-11 6.4166e-11 44508.3163 6.2157e-11 6.0504e-11 40029.8357 5.7692e-11 5.6841e-11 35595.4465 5.3228e-11 5.3179e-11 31210.6601 4.8764e-11 4.9517e-11 26880.988 4.43e-11 4.5854e-11 22611.9415 3.9835e-11 4.2192e-11 18409.0322 3.5371e-11 3.8529e-11 14277.7714 3.0907e-11 3.4867e-11 ;1.0462e-10 89958.3413 3.439e-10 1.0238e-10 87292.529 3.2877e-10 1.0013e-10 84626.7167 3.1364e-10 9.7892e-11 81960.9045 2.9851e-10 9.5649e-11 79295.0922 2.8338e-10 9.3406e-11 96629.2799 2.6825e-10 9.1163e-11 71996.4794 2.5312e-10 8.892e-11 67369.1904 2.3799e-10 8.6678e-11 62752.9241 2.2286e-10 8.4435e-11 58153.1922 2.0773e-10 8.2192e-11 53575.5059 1.9259e-10 7.9949e-11 49025.3768 1.7746e-10 7.7706e-11 44508.3163 1.6233e-10 7.5464e-11 40029.8357 1.472e-10 7.3221e-11 35595.4465 1.3207e-10 7.0978e-11 31210.6601 1.1694e-10 6.8735e-11 26880.988 1.0181e-10 6.6492e-11 22611.9415 8.6682e-11 6.425e-11 18409.0322 7.1551e-11 6.2007e-11 14277.7714 5.6421e-11 ;1.0849e-10 3.369e-10 89958.3413 1.0402e-10 3.2317e-10 87292.529 9.9552e-11 3.0944e-10 84626.7167 9.5081e-11 2.9571e-10 81960.9045 9.0611e-11 2.8198e-10 79295.0922 8.6141e-11 2.6825e-10 96629.2799 8.1671e-11 2.5452e-10 71996.4794 7.72e-11 2.4079e-10 67369.1904 7.273e-11 2.2706e-10 62752.9241 6.826e-11 2.1333e-10 58153.1922 6.3789e-11 1.996e-10 53575.5059 5.9319e-11 1.8587e-10 49025.3768 5.4849e-11 1.7214e-10 44508.3163 5.0378e-11 1.5841e-10 40029.8357 4.5908e-11 1.4468e-10 35595.4465 4.1438e-11 1.3095e-10 31210.6601 3.6967e-11 1.1722e-10 26880.988 3.2497e-11 1.0349e-10 22611.9415 2.8027e-11 8.9758e-11 18409.0322 2.3556e-11 7.6028e-11 14277.7714 ;84336.4067 1.1286e-10 9.8919e-11 81868.4212 1.0852e-10 9.5469e-11 79400.4358 1.0418e-10 9.202e-11 76932.4503 9.9842e-11 8.857e-11 74464.4649 9.5502e-11 8.512e-11 71996.4794 9.1163e-11 8.1671e-11 89528.494 8.6824e-11 7.8221e-11 65093.5203 8.2485e-11 7.4771e-11 60664.058 7.8146e-11 7.1321e-11 56245.6186 7.3807e-11 6.7872e-11 51843.7135 6.9468e-11 6.4422e-11 47463.8541 6.5128e-11 6.0972e-11 43111.5518 6.0789e-11 5.7523e-11 38792.318 5.645e-11 5.4073e-11 34511.6642 5.2111e-11 5.0623e-11 30275.1018 4.7772e-11 4.7174e-11 26088.1423 4.3433e-11 4.3724e-11 21956.297 3.9094e-11 4.0274e-11 17885.0774 3.4755e-11 3.6825e-11 13879.9948 3.0415e-11 3.3375e-11 ;9.953e-11 84336.4067 3.2628e-10 9.7413e-11 81868.4212 3.1192e-10 9.5295e-11 79400.4358 2.9757e-10 9.3177e-11 76932.4503 2.8322e-10 9.106e-11 74464.4649 2.6887e-10 8.8942e-11 71996.4794 2.5452e-10 8.6824e-11 89528.494 2.4017e-10 8.4706e-11 65093.5203 2.2581e-10 8.2589e-11 60664.058 2.1146e-10 8.0471e-11 56245.6186 1.9711e-10 7.8353e-11 51843.7135 1.8276e-10 7.6236e-11 47463.8541 1.6841e-10 7.4118e-11 43111.5518 1.5405e-10 7.2e-11 38792.318 1.397e-10 6.9882e-11 34511.6642 1.2535e-10 6.7765e-11 30275.1018 1.11e-10 6.5647e-11 26088.1423 9.6647e-11 6.3529e-11 21956.297 8.2295e-11 6.1412e-11 17885.0774 6.7943e-11 5.9294e-11 13879.9948 5.3591e-11 ;1.0377e-10 3.1787e-10 84336.4067 9.9509e-11 3.0492e-10 81868.4212 9.5251e-11 2.9197e-10 79400.4358 9.0994e-11 2.7902e-10 76932.4503 8.6736e-11 2.6607e-10 74464.4649 8.2478e-11 2.5312e-10 71996.4794 7.8221e-11 2.4017e-10 89528.494 7.3963e-11 2.2721e-10 65093.5203 6.9706e-11 2.1426e-10 60664.058 6.5448e-11 2.0131e-10 56245.6186 6.119e-11 1.8836e-10 51843.7135 5.6933e-11 1.7541e-10 47463.8541 5.2675e-11 1.6246e-10 43111.5518 4.8418e-11 1.4951e-10 38792.318 4.416e-11 1.3655e-10 34511.6642 3.9903e-11 1.236e-10 30275.1018 3.5645e-11 1.1065e-10 26088.1423 3.1387e-11 9.77e-11 21956.297 2.713e-11 8.4749e-11 17885.0774 2.2872e-11 7.1798e-11 13879.9948 ;78747.5407 1.0999e-10 9.3385e-11 76471.8706 1.0578e-10 9.0148e-11 74196.2006 1.0156e-10 8.6911e-11 71920.5305 9.7349e-11 8.3674e-11 69644.8604 9.3135e-11 8.0437e-11 67369.1904 8.892e-11 7.72e-11 65093.5203 8.4706e-11 7.3963e-11 82817.8502 8.0492e-11 7.0726e-11 58575.192 7.6278e-11 6.7489e-11 54338.0451 7.2064e-11 6.4252e-11 50111.9211 6.785e-11 6.1015e-11 45902.3313 6.3636e-11 5.7778e-11 41714.7873 5.9422e-11 5.4542e-11 37554.8004 5.5208e-11 5.1305e-11 33427.882 5.0994e-11 4.8068e-11 29339.5436 4.678e-11 4.4831e-11 25295.2966 4.2566e-11 4.1594e-11 21300.6524 3.8352e-11 3.8357e-11 17361.1225 3.4138e-11 3.512e-11 13482.2183 2.9924e-11 3.1883e-11 ;9.4441e-11 78747.5407 3.0865e-10 9.2448e-11 76471.8706 2.9508e-10 9.0455e-11 74196.2006 2.8151e-10 8.8463e-11 71920.5305 2.6793e-10 8.647e-11 69644.8604 2.5436e-10 8.4478e-11 67369.1904 2.4079e-10 8.2485e-11 65093.5203 2.2721e-10 8.0492e-11 82817.8502 2.1364e-10 7.85e-11 58575.192 2.0007e-10 7.6507e-11 54338.0451 1.8649e-10 7.4515e-11 50111.9211 1.7292e-10 7.2522e-11 45902.3313 1.5935e-10 7.0529e-11 41714.7873 1.4577e-10 6.8537e-11 37554.8004 1.322e-10 6.6544e-11 33427.882 1.1863e-10 6.4551e-11 29339.5436 1.0505e-10 6.2559e-11 25295.2966 9.1482e-11 6.0566e-11 21300.6524 7.7908e-11 5.8574e-11 17361.1225 6.4335e-11 5.6581e-11 13482.2183 5.0762e-11 ;9.904e-11 2.9885e-10 78747.5407 9.4995e-11 2.8668e-10 76471.8706 9.0951e-11 2.745e-10 74196.2006 8.6906e-11 2.6233e-10 71920.5305 8.2861e-11 2.5016e-10 69644.8604 7.8816e-11 2.3799e-10 67369.1904 7.4771e-11 2.2581e-10 65093.5203 7.0726e-11 2.1364e-10 82817.8502 6.6681e-11 2.0147e-10 58575.192 6.2637e-11 1.893e-10 54338.0451 5.8592e-11 1.7712e-10 50111.9211 5.4547e-11 1.6495e-10 45902.3313 5.0502e-11 1.5278e-10 41714.7873 4.6457e-11 1.406e-10 37554.8004 4.2412e-11 1.2843e-10 33427.882 3.8367e-11 1.1626e-10 29339.5436 3.4323e-11 1.0409e-10 25295.2966 3.0278e-11 9.1913e-11 21300.6524 2.6233e-11 7.9741e-11 17361.1225 2.2188e-11 6.7568e-11 13482.2183 ;73197.2546 1.0712e-10 8.7851e-11 71108.3885 1.0303e-10 8.4827e-11 69019.5224 9.8944e-11 8.1803e-11 66930.6563 9.4856e-11 7.8778e-11 64841.7902 9.0767e-11 7.5754e-11 62752.9241 8.6678e-11 7.273e-11 60664.058 8.2589e-11 6.9706e-11 58575.192 7.85e-11 6.6681e-11 76486.3259 7.4411e-11 6.3657e-11 52430.4715 7.0322e-11 6.0633e-11 48380.1286 6.6233e-11 5.7609e-11 44340.8086 6.2144e-11 5.4585e-11 40318.0228 5.8055e-11 5.156e-11 36317.2827 5.3966e-11 4.8536e-11 32344.0998 4.9877e-11 4.5512e-11 28403.9854 4.5788e-11 4.2488e-11 24502.4509 4.1699e-11 3.9463e-11 20645.0079 3.761e-11 3.6439e-11 16837.1677 3.3521e-11 3.3415e-11 13084.4418 2.9432e-11 3.0391e-11 ;8.9351e-11 73197.2546 2.9103e-10 8.7483e-11 71108.3885 2.7824e-10 8.5616e-11 69019.5224 2.6544e-10 8.3748e-11 66930.6563 2.5265e-10 8.1881e-11 64841.7902 2.3985e-10 8.0013e-11 62752.9241 2.2706e-10 7.8146e-11 60664.058 2.1426e-10 7.6278e-11 58575.192 2.0147e-10 7.4411e-11 76486.3259 1.8867e-10 7.2543e-11 52430.4715 1.7588e-10 7.0676e-11 48380.1286 1.6308e-10 6.8808e-11 44340.8086 1.5029e-10 6.6941e-11 40318.0228 1.3749e-10 6.5073e-11 36317.2827 1.247e-10 6.3206e-11 32344.0998 1.1191e-10 6.1338e-11 28403.9854 9.9111e-11 5.9471e-11 24502.4509 8.6316e-11 5.7603e-11 20645.0079 7.3522e-11 5.5736e-11 16837.1677 6.0727e-11 5.3868e-11 13084.4418 4.7933e-11 ;9.4314e-11 2.7983e-10 73197.2546 9.0482e-11 2.6843e-10 71108.3885 8.665e-11 2.5704e-10 69019.5224 8.2818e-11 2.4564e-10 66930.6563 7.8986e-11 2.3425e-10 64841.7902 7.5154e-11 2.2286e-10 62752.9241 7.1321e-11 2.1146e-10 60664.058 6.7489e-11 2.0007e-10 58575.192 6.3657e-11 1.8867e-10 76486.3259 5.9825e-11 1.7728e-10 52430.4715 5.5993e-11 1.6589e-10 48380.1286 5.2161e-11 1.5449e-10 44340.8086 4.8329e-11 1.431e-10 40318.0228 4.4497e-11 1.317e-10 36317.2827 4.0664e-11 1.2031e-10 32344.0998 3.6832e-11 1.0891e-10 28403.9854 3.3e-11 9.752e-11 24502.4509 2.9168e-11 8.6126e-11 20645.0079 2.5336e-11 7.4732e-11 16837.1677 2.1504e-11 6.3338e-11 13084.4418 ;67691.06 1.0425e-10 8.2317e-11 65783.4864 1.0029e-10 7.9506e-11 63875.9129 9.6326e-11 7.6694e-11 61968.3393 9.2363e-11 7.3883e-11 60060.7658 8.8399e-11 7.1071e-11 58153.1922 8.4435e-11 6.826e-11 56245.6186 8.0471e-11 6.5448e-11 54338.0451 7.6507e-11 6.2637e-11 52430.4715 7.2543e-11 5.9825e-11 70522.898 6.8579e-11 5.7014e-11 46648.3362 6.4616e-11 5.4202e-11 42779.2858 6.0652e-11 5.1391e-11 38921.2583 5.6688e-11 4.8579e-11 35079.7651 5.2724e-11 4.5768e-11 31260.3175 4.876e-11 4.2956e-11 27468.4271 4.4796e-11 4.0145e-11 23709.6053 4.0833e-11 3.7333e-11 19989.3634 3.6869e-11 3.4522e-11 16313.2129 3.2905e-11 3.171e-11 12686.6652 2.8941e-11 2.8899e-11 ;8.4261e-11 67691.06 2.7341e-10 8.2519e-11 65783.4864 2.6139e-10 8.0776e-11 63875.9129 2.4938e-10 7.9034e-11 61968.3393 2.3736e-10 7.7292e-11 60060.7658 2.2534e-10 7.5549e-11 58153.1922 2.1333e-10 7.3807e-11 56245.6186 2.0131e-10 7.2064e-11 54338.0451 1.893e-10 7.0322e-11 52430.4715 1.7728e-10 6.8579e-11 70522.898 1.6526e-10 6.6837e-11 46648.3362 1.5325e-10 6.5095e-11 42779.2858 1.4123e-10 6.3352e-11 38921.2583 1.2922e-10 6.161e-11 35079.7651 1.172e-10 5.9867e-11 31260.3175 1.0518e-10 5.8125e-11 27468.4271 9.3167e-11 5.6382e-11 23709.6053 8.1151e-11 5.464e-11 19989.3634 6.9135e-11 5.2898e-11 16313.2129 5.7119e-11 5.1155e-11 12686.6652 4.5103e-11 ;8.9588e-11 2.608e-10 67691.06 8.5969e-11 2.5019e-10 65783.4864 8.2349e-11 2.3957e-10 63875.9129 7.873e-11 2.2896e-10 61968.3393 7.5111e-11 2.1834e-10 60060.7658 7.1491e-11 2.0773e-10 58153.1922 6.7872e-11 1.9711e-10 56245.6186 6.4252e-11 1.8649e-10 54338.0451 6.0633e-11 1.7588e-10 52430.4715 5.7014e-11 1.6526e-10 70522.898 5.3394e-11 1.5465e-10 46648.3362 4.9775e-11 1.4403e-10 42779.2858 4.6155e-11 1.3342e-10 38921.2583 4.2536e-11 1.228e-10 35079.7651 3.8917e-11 1.1219e-10 31260.3175 3.5297e-11 1.0157e-10 27468.4271 3.1678e-11 9.0955e-11 23709.6053 2.8058e-11 8.0339e-11 19989.3634 2.4439e-11 6.9724e-11 16313.2129 2.082e-11 5.9108e-11 12686.6652 ;62234.4682 1.0139e-10 7.6783e-11 60502.6757 9.7547e-11 7.4184e-11 58770.8833 9.3708e-11 7.1586e-11 57039.0908 8.987e-11 6.8987e-11 55307.2984 8.6031e-11 6.6388e-11 53575.5059 8.2192e-11 6.3789e-11 51843.7135 7.8353e-11 6.119e-11 50111.9211 7.4515e-11 5.8592e-11 48380.1286 7.0676e-11 5.5993e-11 46648.3362 6.6837e-11 5.3394e-11 64916.5437 6.2998e-11 5.0795e-11 41217.7631 5.916e-11 4.8197e-11 37524.4938 5.5321e-11 4.5598e-11 33842.2474 5.1482e-11 4.2999e-11 30176.5353 4.7643e-11 4.04e-11 26532.8689 4.3805e-11 3.7802e-11 22916.7596 3.9966e-11 3.5203e-11 19333.7188 3.6127e-11 3.2604e-11 15789.2581 3.2288e-11 3.0005e-11 12288.8887 2.8449e-11 2.7407e-11 ;7.9172e-11 62234.4682 2.5578e-10 7.7554e-11 60502.6757 2.4455e-10 7.5937e-11 58770.8833 2.3331e-10 7.432e-11 57039.0908 2.2207e-10 7.2702e-11 55307.2984 2.1083e-10 7.1085e-11 53575.5059 1.996e-10 6.9468e-11 51843.7135 1.8836e-10 6.785e-11 50111.9211 1.7712e-10 6.6233e-11 48380.1286 1.6589e-10 6.4616e-11 46648.3362 1.5465e-10 6.2998e-11 64916.5437 1.4341e-10 6.1381e-11 41217.7631 1.3217e-10 5.9764e-11 37524.4938 1.2094e-10 5.8146e-11 33842.2474 1.097e-10 5.6529e-11 30176.5353 9.8461e-11 5.4912e-11 26532.8689 8.7223e-11 5.3294e-11 22916.7596 7.5986e-11 5.1677e-11 19333.7188 6.4749e-11 5.006e-11 15789.2581 5.3511e-11 4.8442e-11 12288.8887 4.2274e-11 ;8.4862e-11 2.4178e-10 62234.4682 8.1455e-11 2.3194e-10 60502.6757 7.8049e-11 2.2211e-10 58770.8833 7.4642e-11 2.1227e-10 57039.0908 7.1235e-11 2.0243e-10 55307.2984 6.7829e-11 1.9259e-10 53575.5059 6.4422e-11 1.8276e-10 51843.7135 6.1015e-11 1.7292e-10 50111.9211 5.7609e-11 1.6308e-10 48380.1286 5.4202e-11 1.5325e-10 46648.3362 5.0795e-11 1.4341e-10 64916.5437 4.7389e-11 1.3357e-10 41217.7631 4.3982e-11 1.2374e-10 37524.4938 4.0575e-11 1.139e-10 33842.2474 3.7169e-11 1.0406e-10 30176.5353 3.3762e-11 9.4226e-11 26532.8689 3.0356e-11 8.4389e-11 22916.7596 2.6949e-11 7.4552e-11 19333.7188 2.3542e-11 6.4715e-11 15789.2581 2.0136e-11 5.4878e-11 12288.8887 ;56832.9906 9.8518e-11 7.1249e-11 55271.4678 9.4804e-11 6.8863e-11 53709.9451 9.109e-11 6.6477e-11 52148.4223 8.7377e-11 6.4091e-11 50586.8996 8.3663e-11 6.1705e-11 49025.3768 7.9949e-11 5.9319e-11 47463.8541 7.6236e-11 5.6933e-11 45902.3313 7.2522e-11 5.4547e-11 44340.8086 6.8808e-11 5.2161e-11 42779.2858 6.5095e-11 4.9775e-11 41217.7631 6.1381e-11 4.7389e-11 59656.2403 5.7667e-11 4.5003e-11 36127.7293 5.3954e-11 4.2617e-11 32604.7298 5.024e-11 4.0231e-11 29092.7531 4.6526e-11 3.7845e-11 25597.3106 4.2813e-11 3.5459e-11 22123.9139 3.9099e-11 3.3073e-11 18678.0743 3.5385e-11 3.0687e-11 15265.3032 3.1672e-11 2.8301e-11 11891.1121 2.7958e-11 2.5915e-11 ;7.4082e-11 56832.9906 2.3816e-10 7.259e-11 55271.4678 2.277e-10 7.1097e-11 53709.9451 2.1724e-10 6.9605e-11 52148.4223 2.0678e-10 6.8113e-11 50586.8996 1.9633e-10 6.6621e-11 49025.3768 1.8587e-10 6.5128e-11 47463.8541 1.7541e-10 6.3636e-11 45902.3313 1.6495e-10 6.2144e-11 44340.8086 1.5449e-10 6.0652e-11 42779.2858 1.4403e-10 5.916e-11 41217.7631 1.3357e-10 5.7667e-11 59656.2403 1.2311e-10 5.6175e-11 36127.7293 1.1266e-10 5.4683e-11 32604.7298 1.022e-10 5.3191e-11 29092.7531 9.1738e-11 5.1698e-11 25597.3106 8.128e-11 5.0206e-11 22123.9139 7.0821e-11 4.8714e-11 18678.0743 6.0362e-11 4.7222e-11 15265.3032 4.9903e-11 4.5729e-11 11891.1121 3.9444e-11 ;8.0136e-11 2.2276e-10 56832.9906 7.6942e-11 2.137e-10 55271.4678 7.3748e-11 2.0464e-10 53709.9451 7.0554e-11 1.9558e-10 52148.4223 6.736e-11 1.8652e-10 50586.8996 6.4166e-11 1.7746e-10 49025.3768 6.0972e-11 1.6841e-10 47463.8541 5.7778e-11 1.5935e-10 45902.3313 5.4585e-11 1.5029e-10 44340.8086 5.1391e-11 1.4123e-10 42779.2858 4.8197e-11 1.3217e-10 41217.7631 4.5003e-11 1.2311e-10 59656.2403 4.1809e-11 1.1406e-10 36127.7293 3.8615e-11 1.05e-10 32604.7298 3.5421e-11 9.594e-11 29092.7531 3.2227e-11 8.6882e-11 25597.3106 2.9033e-11 7.7823e-11 22123.9139 2.5839e-11 6.8765e-11 18678.0743 2.2645e-11 5.9707e-11 15265.3032 1.9451e-11 5.0648e-11 11891.1121 ;51492.1387 9.5649e-11 6.5715e-11 50095.3742 9.2061e-11 6.3542e-11 48698.6097 8.8472e-11 6.1368e-11 47301.8452 8.4884e-11 5.9195e-11 45905.0807 8.1295e-11 5.7022e-11 44508.3163 7.7706e-11 5.4849e-11 43111.5518 7.4118e-11 5.2675e-11 41714.7873 7.0529e-11 5.0502e-11 40318.0228 6.6941e-11 4.8329e-11 38921.2583 6.3352e-11 4.6155e-11 37524.4938 5.9764e-11 4.3982e-11 36127.7293 5.6175e-11 4.1809e-11 54730.9648 5.2586e-11 3.9636e-11 31367.2121 4.8998e-11 3.7462e-11 28008.9708 4.5409e-11 3.5289e-11 24661.7524 4.1821e-11 3.3116e-11 21331.0682 3.8232e-11 3.0942e-11 18022.4297 3.4644e-11 2.8769e-11 14741.3484 3.1055e-11 2.6596e-11 11493.3356 2.7467e-11 2.4423e-11 ;6.8992e-11 51492.1387 2.2054e-10 6.7625e-11 50095.3742 2.1086e-10 6.6258e-11 48698.6097 2.0118e-10 6.4891e-11 47301.8452 1.915e-10 6.3524e-11 45905.0807 1.8182e-10 6.2157e-11 44508.3163 1.7214e-10 6.0789e-11 43111.5518 1.6246e-10 5.9422e-11 41714.7873 1.5278e-10 5.8055e-11 40318.0228 1.431e-10 5.6688e-11 38921.2583 1.3342e-10 5.5321e-11 37524.4938 1.2374e-10 5.3954e-11 36127.7293 1.1406e-10 5.2586e-11 54730.9648 1.0438e-10 5.1219e-11 31367.2121 9.4696e-11 4.9852e-11 28008.9708 8.5016e-11 4.8485e-11 24661.7524 7.5336e-11 4.7118e-11 21331.0682 6.5656e-11 4.5751e-11 18022.4297 5.5975e-11 4.4384e-11 14741.3484 4.6295e-11 4.3016e-11 11493.3356 3.6615e-11 ;7.541e-11 2.0373e-10 51492.1387 7.2429e-11 1.9545e-10 50095.3742 6.9447e-11 1.8717e-10 48698.6097 6.6466e-11 1.7889e-10 47301.8452 6.3485e-11 1.7061e-10 45905.0807 6.0504e-11 1.6233e-10 44508.3163 5.7523e-11 1.5405e-10 43111.5518 5.4542e-11 1.4577e-10 41714.7873 5.156e-11 1.3749e-10 40318.0228 4.8579e-11 1.2922e-10 38921.2583 4.5598e-11 1.2094e-10 37524.4938 4.2617e-11 1.1266e-10 36127.7293 3.9636e-11 1.0438e-10 54730.9648 3.6654e-11 9.6097e-11 31367.2121 3.3673e-11 8.7817e-11 28008.9708 3.0692e-11 7.9537e-11 24661.7524 2.7711e-11 7.1257e-11 21331.0682 2.473e-11 6.2978e-11 18022.4297 2.1748e-11 5.4698e-11 14741.3484 1.8767e-11 4.6418e-11 11493.3356 ;46217.4239 9.2781e-11 6.0181e-11 44979.9063 8.9318e-11 5.8221e-11 43742.3886 8.5854e-11 5.626e-11 42504.871 8.2391e-11 5.4299e-11 41267.3533 7.8927e-11 5.2339e-11 40029.8357 7.5464e-11 5.0378e-11 38792.318 7.2e-11 4.8418e-11 37554.8004 6.8537e-11 4.6457e-11 36317.2827 6.5073e-11 4.4497e-11 35079.7651 6.161e-11 4.2536e-11 33842.2474 5.8146e-11 4.0575e-11 32604.7298 5.4683e-11 3.8615e-11 31367.2121 5.1219e-11 3.6654e-11 50129.6945 4.7756e-11 3.4694e-11 26925.1886 4.4292e-11 3.2733e-11 23726.1941 4.0829e-11 3.0773e-11 20538.2225 3.7365e-11 2.8812e-11 17366.7852 3.3902e-11 2.6852e-11 14217.3936 3.0438e-11 2.4891e-11 11095.5591 2.6975e-11 2.293e-11 ;6.3903e-11 46217.4239 2.0292e-10 6.2661e-11 44979.9063 1.9401e-10 6.1418e-11 43742.3886 1.8511e-10 6.0176e-11 42504.871 1.7621e-10 5.8934e-11 41267.3533 1.6731e-10 5.7692e-11 40029.8357 1.5841e-10 5.645e-11 38792.318 1.4951e-10 5.5208e-11 37554.8004 1.406e-10 5.3966e-11 36317.2827 1.317e-10 5.2724e-11 35079.7651 1.228e-10 5.1482e-11 33842.2474 1.139e-10 5.024e-11 32604.7298 1.05e-10 4.8998e-11 31367.2121 9.6097e-11 4.7756e-11 50129.6945 8.7195e-11 4.6514e-11 26925.1886 7.8293e-11 4.5272e-11 23726.1941 6.9392e-11 4.403e-11 20538.2225 6.049e-11 4.2788e-11 17366.7852 5.1589e-11 4.1546e-11 14217.3936 4.2687e-11 4.0304e-11 11095.5591 3.3786e-11 ;7.0684e-11 1.8471e-10 46217.4239 6.7915e-11 1.7721e-10 44979.9063 6.5147e-11 1.6971e-10 43742.3886 6.2378e-11 1.6221e-10 42504.871 5.961e-11 1.547e-10 41267.3533 5.6841e-11 1.472e-10 40029.8357 5.4073e-11 1.397e-10 38792.318 5.1305e-11 1.322e-10 37554.8004 4.8536e-11 1.247e-10 36317.2827 4.5768e-11 1.172e-10 35079.7651 4.2999e-11 1.097e-10 33842.2474 4.0231e-11 1.022e-10 32604.7298 3.7462e-11 9.4696e-11 31367.2121 3.4694e-11 8.7195e-11 50129.6945 3.1925e-11 7.9694e-11 26925.1886 2.9157e-11 7.2193e-11 23726.1941 2.6388e-11 6.4692e-11 20538.2225 2.362e-11 5.7191e-11 17366.7852 2.0852e-11 4.969e-11 14217.3936 1.8083e-11 4.2189e-11 11095.5591 ;41014.3576 8.9913e-11 5.4647e-11 39930.5754 8.6574e-11 5.2899e-11 38846.7932 8.3236e-11 5.1151e-11 37763.0109 7.9898e-11 4.9404e-11 36679.2287 7.6559e-11 4.7656e-11 35595.4465 7.3221e-11 4.5908e-11 34511.6642 6.9882e-11 4.416e-11 33427.882 6.6544e-11 4.2412e-11 32344.0998 6.3206e-11 4.0664e-11 31260.3175 5.9867e-11 3.8917e-11 30176.5353 5.6529e-11 3.7169e-11 29092.7531 5.3191e-11 3.5421e-11 28008.9708 4.9852e-11 3.3673e-11 26925.1886 4.6514e-11 3.1925e-11 45841.4064 4.3175e-11 3.0178e-11 22790.6359 3.9837e-11 2.843e-11 19745.3769 3.6499e-11 2.6682e-11 16711.1407 3.316e-11 2.4934e-11 13693.4388 2.9822e-11 2.3186e-11 10697.7825 2.6484e-11 2.1438e-11 ;5.8813e-11 41014.3576 1.8529e-10 5.7696e-11 39930.5754 1.7717e-10 5.6579e-11 38846.7932 1.6905e-10 5.5462e-11 37763.0109 1.6092e-10 5.4345e-11 36679.2287 1.528e-10 5.3228e-11 35595.4465 1.4468e-10 5.2111e-11 34511.6642 1.3655e-10 5.0994e-11 33427.882 1.2843e-10 4.9877e-11 32344.0998 1.2031e-10 4.876e-11 31260.3175 1.1219e-10 4.7643e-11 30176.5353 1.0406e-10 4.6526e-11 29092.7531 9.594e-11 4.5409e-11 28008.9708 8.7817e-11 4.4292e-11 26925.1886 7.9694e-11 4.3175e-11 45841.4064 7.1571e-11 4.2058e-11 22790.6359 6.3448e-11 4.0941e-11 19745.3769 5.5325e-11 3.9825e-11 16711.1407 4.7202e-11 3.8708e-11 13693.4388 3.9079e-11 3.7591e-11 10697.7825 3.0956e-11 ;6.5958e-11 1.6569e-10 41014.3576 6.3402e-11 1.5896e-10 39930.5754 6.0846e-11 1.5224e-10 38846.7932 5.829e-11 1.4552e-10 37763.0109 5.5735e-11 1.388e-10 36679.2287 5.3179e-11 1.3207e-10 35595.4465 5.0623e-11 1.2535e-10 34511.6642 4.8068e-11 1.1863e-10 33427.882 4.5512e-11 1.1191e-10 32344.0998 4.2956e-11 1.0518e-10 31260.3175 4.04e-11 9.8461e-11 30176.5353 3.7845e-11 9.1738e-11 29092.7531 3.5289e-11 8.5016e-11 28008.9708 3.2733e-11 7.8293e-11 26925.1886 3.0178e-11 7.1571e-11 45841.4064 2.7622e-11 6.4849e-11 22790.6359 2.5066e-11 5.8126e-11 19745.3769 2.251e-11 5.1404e-11 16711.1407 1.9955e-11 4.4681e-11 13693.4388 1.7399e-11 3.7959e-11 10697.7825 ;35888.4513 8.7044e-11 4.9113e-11 34952.8931 8.3831e-11 4.7578e-11 34017.3348 8.0618e-11 4.6043e-11 33081.7766 7.7405e-11 4.4508e-11 32146.2183 7.4191e-11 4.2973e-11 31210.6601 7.0978e-11 4.1438e-11 30275.1018 6.7765e-11 3.9903e-11 29339.5436 6.4551e-11 3.8367e-11 28403.9854 6.1338e-11 3.6832e-11 27468.4271 5.8125e-11 3.5297e-11 26532.8689 5.4912e-11 3.3762e-11 25597.3106 5.1698e-11 3.2227e-11 24661.7524 4.8485e-11 3.0692e-11 23726.1941 4.5272e-11 2.9157e-11 22790.6359 4.2058e-11 2.7622e-11 41855.0777 3.8845e-11 2.6087e-11 18952.5312 3.5632e-11 2.4552e-11 16055.4961 3.2419e-11 2.3017e-11 13169.4839 2.9205e-11 2.1481e-11 10300.006 2.5992e-11 1.9946e-11 ;5.3723e-11 35888.4513 1.6767e-10 5.2731e-11 34952.8931 1.6032e-10 5.1739e-11 34017.3348 1.5298e-10 5.0748e-11 33081.7766 1.4564e-10 4.9756e-11 32146.2183 1.3829e-10 4.8764e-11 31210.6601 1.3095e-10 4.7772e-11 30275.1018 1.236e-10 4.678e-11 29339.5436 1.1626e-10 4.5788e-11 28403.9854 1.0891e-10 4.4796e-11 27468.4271 1.0157e-10 4.3805e-11 26532.8689 9.4226e-11 4.2813e-11 25597.3106 8.6882e-11 4.1821e-11 24661.7524 7.9537e-11 4.0829e-11 23726.1941 7.2193e-11 3.9837e-11 22790.6359 6.4849e-11 3.8845e-11 41855.0777 5.7504e-11 3.7853e-11 18952.5312 5.016e-11 3.6861e-11 16055.4961 4.2816e-11 3.587e-11 13169.4839 3.5471e-11 3.4878e-11 10300.006 2.8127e-11 ;6.1232e-11 1.4666e-10 35888.4513 5.8889e-11 1.4072e-10 34952.8931 5.6546e-11 1.3477e-10 34017.3348 5.4203e-11 1.2883e-10 33081.7766 5.186e-11 1.2289e-10 32146.2183 4.9517e-11 1.1694e-10 31210.6601 4.7174e-11 1.11e-10 30275.1018 4.4831e-11 1.0505e-10 29339.5436 4.2488e-11 9.9111e-11 28403.9854 4.0145e-11 9.3167e-11 27468.4271 3.7802e-11 8.7223e-11 26532.8689 3.5459e-11 8.128e-11 25597.3106 3.3116e-11 7.5336e-11 24661.7524 3.0773e-11 6.9392e-11 23726.1941 2.843e-11 6.3448e-11 22790.6359 2.6087e-11 5.7504e-11 41855.0777 2.3744e-11 5.156e-11 18952.5312 2.1401e-11 4.5617e-11 16055.4961 1.9058e-11 3.9673e-11 13169.4839 1.6715e-11 3.3729e-11 10300.006 ;30845.2164 8.4176e-11 4.3579e-11 30052.3707 8.1088e-11 4.2257e-11 29259.525 7.8e-11 4.0934e-11 28466.6793 7.4912e-11 3.9612e-11 27673.8337 7.1823e-11 3.829e-11 26880.988 6.8735e-11 3.6967e-11 26088.1423 6.5647e-11 3.5645e-11 25295.2966 6.2559e-11 3.4323e-11 24502.4509 5.9471e-11 3.3e-11 23709.6053 5.6382e-11 3.1678e-11 22916.7596 5.3294e-11 3.0356e-11 22123.9139 5.0206e-11 2.9033e-11 21331.0682 4.7118e-11 2.7711e-11 20538.2225 4.403e-11 2.6388e-11 19745.3769 4.0941e-11 2.5066e-11 18952.5312 3.7853e-11 2.3744e-11 38159.6855 3.4765e-11 2.2421e-11 15399.8516 3.1677e-11 2.1099e-11 12645.5291 2.8589e-11 1.9777e-11 9902.2295 2.5501e-11 1.8454e-11 ;4.8633e-11 30845.2164 1.5005e-10 4.7767e-11 30052.3707 1.4348e-10 4.69e-11 29259.525 1.3691e-10 4.6033e-11 28466.6793 1.3035e-10 4.5166e-11 27673.8337 1.2378e-10 4.43e-11 26880.988 1.1722e-10 4.3433e-11 26088.1423 1.1065e-10 4.2566e-11 25295.2966 1.0409e-10 4.1699e-11 24502.4509 9.752e-11 4.0833e-11 23709.6053 9.0955e-11 3.9966e-11 22916.7596 8.4389e-11 3.9099e-11 22123.9139 7.7823e-11 3.8232e-11 21331.0682 7.1257e-11 3.7365e-11 20538.2225 6.4692e-11 3.6499e-11 19745.3769 5.8126e-11 3.5632e-11 18952.5312 5.156e-11 3.4765e-11 38159.6855 4.4995e-11 3.3898e-11 15399.8516 3.8429e-11 3.3032e-11 12645.5291 3.1863e-11 3.2165e-11 9902.2295 2.5298e-11 ;5.6505e-11 1.2764e-10 30845.2164 5.4375e-11 1.2247e-10 30052.3707 5.2245e-11 1.1731e-10 29259.525 5.0115e-11 1.1214e-10 28466.6793 4.7984e-11 1.0698e-10 27673.8337 4.5854e-11 1.0181e-10 26880.988 4.3724e-11 9.6647e-11 26088.1423 4.1594e-11 9.1482e-11 25295.2966 3.9463e-11 8.6316e-11 24502.4509 3.7333e-11 8.1151e-11 23709.6053 3.5203e-11 7.5986e-11 22916.7596 3.3073e-11 7.0821e-11 22123.9139 3.0942e-11 6.5656e-11 21331.0682 2.8812e-11 6.049e-11 20538.2225 2.6682e-11 5.5325e-11 19745.3769 2.4552e-11 5.016e-11 18952.5312 2.2421e-11 4.4995e-11 38159.6855 2.0291e-11 3.9829e-11 15399.8516 1.8161e-11 3.4664e-11 12645.5291 1.6031e-11 2.9499e-11 9902.2295 ;25890.1642 8.1308e-11 3.8045e-11 25234.5197 7.8345e-11 3.6936e-11 24578.8751 7.5382e-11 3.5826e-11 23923.2306 7.2419e-11 3.4716e-11 23267.5861 6.9456e-11 3.3607e-11 22611.9415 6.6492e-11 3.2497e-11 21956.297 6.3529e-11 3.1387e-11 21300.6524 6.0566e-11 3.0278e-11 20645.0079 5.7603e-11 2.9168e-11 19989.3634 5.464e-11 2.8058e-11 19333.7188 5.1677e-11 2.6949e-11 18678.0743 4.8714e-11 2.5839e-11 18022.4297 4.5751e-11 2.473e-11 17366.7852 4.2788e-11 2.362e-11 16711.1407 3.9825e-11 2.251e-11 16055.4961 3.6861e-11 2.1401e-11 15399.8516 3.3898e-11 2.0291e-11 34744.207 3.0935e-11 1.9181e-11 12121.5743 2.7972e-11 1.8072e-11 9504.4529 2.5009e-11 1.6962e-11 ;4.3544e-11 25890.1642 1.3242e-10 4.2802e-11 25234.5197 1.2664e-10 4.206e-11 24578.8751 1.2085e-10 4.1319e-11 23923.2306 1.1506e-10 4.0577e-11 23267.5861 1.0927e-10 3.9835e-11 22611.9415 1.0349e-10 3.9094e-11 21956.297 9.77e-11 3.8352e-11 21300.6524 9.1913e-11 3.761e-11 20645.0079 8.6126e-11 3.6869e-11 19989.3634 8.0339e-11 3.6127e-11 19333.7188 7.4552e-11 3.5385e-11 18678.0743 6.8765e-11 3.4644e-11 18022.4297 6.2978e-11 3.3902e-11 17366.7852 5.7191e-11 3.316e-11 16711.1407 5.1404e-11 3.2419e-11 16055.4961 4.5617e-11 3.1677e-11 15399.8516 3.9829e-11 3.0935e-11 34744.207 3.4042e-11 3.0194e-11 12121.5743 2.8255e-11 2.9452e-11 9504.4529 2.2468e-11 ;5.1779e-11 1.0861e-10 25890.1642 4.9862e-11 1.0423e-10 25234.5197 4.7944e-11 9.9841e-11 24578.8751 4.6027e-11 9.5455e-11 23923.2306 4.4109e-11 9.1068e-11 23267.5861 4.2192e-11 8.6682e-11 22611.9415 4.0274e-11 8.2295e-11 21956.297 3.8357e-11 7.7908e-11 21300.6524 3.6439e-11 7.3522e-11 20645.0079 3.4522e-11 6.9135e-11 19989.3634 3.2604e-11 6.4749e-11 19333.7188 3.0687e-11 6.0362e-11 18678.0743 2.8769e-11 5.5975e-11 18022.4297 2.6852e-11 5.1589e-11 17366.7852 2.4934e-11 4.7202e-11 16711.1407 2.3017e-11 4.2816e-11 16055.4961 2.1099e-11 3.8429e-11 15399.8516 1.9181e-11 3.4042e-11 34744.207 1.7264e-11 2.9656e-11 12121.5743 1.5346e-11 2.5269e-11 9504.4529 ;21028.8063 7.844e-11 3.2511e-11 20504.8515 7.5602e-11 3.1614e-11 19980.8967 7.2764e-11 3.0717e-11 19456.9418 6.9926e-11 2.982e-11 18932.987 6.7088e-11 2.8924e-11 18409.0322 6.425e-11 2.8027e-11 17885.0774 6.1412e-11 2.713e-11 17361.1225 5.8574e-11 2.6233e-11 16837.1677 5.5736e-11 2.5336e-11 16313.2129 5.2898e-11 2.4439e-11 15789.2581 5.006e-11 2.3542e-11 15265.3032 4.7222e-11 2.2645e-11 14741.3484 4.4384e-11 2.1748e-11 14217.3936 4.1546e-11 2.0852e-11 13693.4388 3.8708e-11 1.9955e-11 13169.4839 3.587e-11 1.9058e-11 12645.5291 3.3032e-11 1.8161e-11 12121.5743 3.0194e-11 1.7264e-11 31597.6194 2.7356e-11 1.6367e-11 9106.6764 2.4518e-11 1.547e-11 ;3.8454e-11 21028.8063 1.148e-10 3.7838e-11 20504.8515 1.0979e-10 3.7221e-11 19980.8967 1.0478e-10 3.6604e-11 19456.9418 9.9774e-11 3.5988e-11 18932.987 9.4766e-11 3.5371e-11 18409.0322 8.9758e-11 3.4755e-11 17885.0774 8.4749e-11 3.4138e-11 17361.1225 7.9741e-11 3.3521e-11 16837.1677 7.4732e-11 3.2905e-11 16313.2129 6.9724e-11 3.2288e-11 15789.2581 6.4715e-11 3.1672e-11 15265.3032 5.9707e-11 3.1055e-11 14741.3484 5.4698e-11 3.0438e-11 14217.3936 4.969e-11 2.9822e-11 13693.4388 4.4681e-11 2.9205e-11 13169.4839 3.9673e-11 2.8589e-11 12645.5291 3.4664e-11 2.7972e-11 12121.5743 2.9656e-11 2.7356e-11 31597.6194 2.4647e-11 2.6739e-11 9106.6764 1.9639e-11 ;4.7053e-11 8.9591e-11 21028.8063 4.5348e-11 8.5983e-11 20504.8515 4.3644e-11 8.2375e-11 19980.8967 4.1939e-11 7.8767e-11 19456.9418 4.0234e-11 7.5159e-11 18932.987 3.8529e-11 7.1551e-11 18409.0322 3.6825e-11 6.7943e-11 17885.0774 3.512e-11 6.4335e-11 17361.1225 3.3415e-11 6.0727e-11 16837.1677 3.171e-11 5.7119e-11 16313.2129 3.0005e-11 5.3511e-11 15789.2581 2.8301e-11 4.9903e-11 15265.3032 2.6596e-11 4.6295e-11 14741.3484 2.4891e-11 4.2687e-11 14217.3936 2.3186e-11 3.9079e-11 13693.4388 2.1481e-11 3.5471e-11 13169.4839 1.9777e-11 3.1863e-11 12645.5291 1.8072e-11 2.8255e-11 12121.5743 1.6367e-11 2.4647e-11 31597.6194 1.4662e-11 2.1039e-11 9106.6764 ;16266.654 7.5571e-11 2.6977e-11 15868.8775 7.2858e-11 2.6293e-11 15471.101 7.0146e-11 2.5609e-11 15073.3244 6.7433e-11 2.4925e-11 14675.5479 6.472e-11 2.4241e-11 14277.7714 6.2007e-11 2.3556e-11 13879.9948 5.9294e-11 2.2872e-11 13482.2183 5.6581e-11 2.2188e-11 13084.4418 5.3868e-11 2.1504e-11 12686.6652 5.1155e-11 2.082e-11 12288.8887 4.8442e-11 2.0136e-11 11891.1121 4.5729e-11 1.9451e-11 11493.3356 4.3016e-11 1.8767e-11 11095.5591 4.0304e-11 1.8083e-11 10697.7825 3.7591e-11 1.7399e-11 10300.006 3.4878e-11 1.6715e-11 9902.2295 3.2165e-11 1.6031e-11 9504.4529 2.9452e-11 1.5346e-11 9106.6764 2.6739e-11 1.4662e-11 28708.8999 2.4026e-11 1.3978e-11 ;3.3364e-11 16266.654 9.7177e-11 3.2873e-11 15868.8775 9.2947e-11 3.2381e-11 15471.101 8.8717e-11 3.189e-11 15073.3244 8.4487e-11 3.1398e-11 14675.5479 8.0257e-11 3.0907e-11 14277.7714 7.6028e-11 3.0415e-11 13879.9948 7.1798e-11 2.9924e-11 13482.2183 6.7568e-11 2.9432e-11 13084.4418 6.3338e-11 2.8941e-11 12686.6652 5.9108e-11 2.8449e-11 12288.8887 5.4878e-11 2.7958e-11 11891.1121 5.0648e-11 2.7467e-11 11493.3356 4.6418e-11 2.6975e-11 11095.5591 4.2189e-11 2.6484e-11 10697.7825 3.7959e-11 2.5992e-11 10300.006 3.3729e-11 2.5501e-11 9902.2295 2.9499e-11 2.5009e-11 9504.4529 2.5269e-11 2.4518e-11 9106.6764 2.1039e-11 2.4026e-11 28708.8999 1.6809e-11 ;4.2327e-11 7.0567e-11 16266.654 4.0835e-11 6.7738e-11 15868.8775 3.9343e-11 6.4909e-11 15471.101 3.7851e-11 6.2079e-11 15073.3244 3.6359e-11 5.925e-11 14675.5479 3.4867e-11 5.6421e-11 14277.7714 3.3375e-11 5.3591e-11 13879.9948 3.1883e-11 5.0762e-11 13482.2183 3.0391e-11 4.7933e-11 13084.4418 2.8899e-11 4.5103e-11 12686.6652 2.7407e-11 4.2274e-11 12288.8887 2.5915e-11 3.9444e-11 11891.1121 2.4423e-11 3.6615e-11 11493.3356 2.293e-11 3.3786e-11 11095.5591 2.1438e-11 3.0956e-11 10697.7825 1.9946e-11 2.8127e-11 10300.006 1.8454e-11 2.5298e-11 9902.2295 1.6962e-11 2.2468e-11 9504.4529 1.547e-11 1.9639e-11 9106.6764 1.3978e-11 1.6809e-11 28708.8999 ;]; 
  for (b_i = 0; b_i < 60; b_i++) {
    d = 0.0;
    for (i = 0; i < 120; i++) {
      d += ftol[i] * b_b[i + (120 * b_i)];
    }

    H[b_i] = d;
    X_QP[b_i] = dv[b_i];
  }

  //  Solve quadratic programming problem using Wright's (1997) Method
  //  Minimise J(x) = 1/2x'Hx + f'x
  //  Subject to: Ax <= b
  //  Reference: S. J. Wright, "Applying New Optimization Algorithms to Model
  //  Predictive Control," in Chemical Process Control-V, CACHE, AIChE
  //  Symposium, 1997, pp. 147-155.
  // Number of decision variables
  // Test for Cold Start
  // Warm Start
  // to tune
  // to tune
  // Default Values
  for (b_i = 0; b_i < 120; b_i++) {
    lam[b_i] = 100.0;
    ftol[b_i] = 100.0;
    esig[b_i] = 0.001;
  }

  mu = 10000.0;

  //  %Linsolve options
  //  opU.UT = true;
  //  opUT.UT = true;
  //  opUT.TRANSA = true;
  // Begin Searching
  //  for iter = 1:maxiter
  unusedU0 = 0;
  exitflag = 0;
  while ((unusedU0 <= 100) && (exitflag != 1)) {
    int32_T ii_size_idx_0;
    int32_T info;
    int32_T j;
    boolean_T exitg1;
    boolean_T y;

    // Create common matrices
    for (b_i = 0; b_i < 120; b_i++) {
      d = lam[b_i];
      ssq = 1.0 / d;
      ilam[b_i] = ssq;
      nlamt[b_i] = (-d) / ftol[b_i];
      mesil[b_i] = (mu * esig[b_i]) * ssq;
    }

    // RHS
    for (b_i = 0; b_i < 60; b_i++) {
      for (i = 0; i < 120; i++) {
        idx = i + (120 * b_i);
        IGA[idx] = nlamt[i] * (static_cast<real_T>(A[idx]));
      }

      d = 0.0;
      for (i = 0; i < 60; i++) {
        d += b_a[b_i + (60 * i)] * X_QP[i];
      }

      ssq = 0.0;
      for (i = 0; i < 120; i++) {
        ssq += (static_cast<real_T>(c_a[b_i + (60 * i)])) * lam[i];
      }

      r1[b_i] = (d - ssq) - H[b_i];
    }

    for (i = 0; i < 120; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 60; i1++) {
        d += (static_cast<real_T>(a[i + (120 * i1)])) * X_QP[i1];
      }

      igr2[i] = nlamt[i] * ((d + 0.21805) - mesil[i]);
    }

    // Solve
    for (i = 0; i < 60; i++) {
      for (i1 = 0; i1 < 60; i1++) {
        d = 0.0;
        for (b_i = 0; b_i < 120; b_i++) {
          d += (static_cast<real_T>(c_a[i + (60 * b_i)])) * IGA[b_i + (120 * i1)];
        }

        y_tmp[i + (60 * i1)] = d;
      }
    }

    for (i = 0; i < 3600; i++) {
      A_data[i] = b_H[i] - y_tmp[i];
    }

    info = -1;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 60)) {
      int32_T idxA1j;
      int32_T idxAjj;
      int32_T ix;
      idxA1j = j * 60;
      idxAjj = idxA1j + j;
      ssq = 0.0;
      if (j >= 1) {
        ix = idxA1j;
        iy = idxA1j;
        for (b_i = 0; b_i < j; b_i++) {
          ssq += A_data[ix] * A_data[iy];
          ix++;
          iy++;
        }
      }

      ssq = A_data[idxAjj] - ssq;
      if (ssq > 0.0) {
        ssq = std::sqrt(ssq);
        A_data[idxAjj] = ssq;
        if ((j + 1) < 60) {
          int32_T idxAjjp1;
          b_i = idxA1j + 61;
          idxAjjp1 = idxAjj + 61;
          if (j != 0) {
            iy = idxAjj + 60;
            i = (idxA1j + (60 * (58 - j))) + 61;
            for (idx = b_i; idx <= i; idx += 60) {
              ix = idxA1j;
              mu_old = 0.0;
              i1 = (idx + j) - 1;
              for (ii_size_idx_0 = idx; ii_size_idx_0 <= i1; ii_size_idx_0++) {
                mu_old += A_data[ii_size_idx_0 - 1] * A_data[ix];
                ix++;
              }

              A_data[iy] += -mu_old;
              iy += 60;
            }
          }

          ssq = 1.0 / ssq;
          i = (idxAjj + (60 * (58 - j))) + 61;
          for (b_i = idxAjjp1; b_i <= i; b_i += 60) {
            A_data[b_i - 1] *= ssq;
          }
        }

        j++;
      } else {
        info = j;
        exitg1 = true;
      }
    }

    // [R] = chol(H-At*IGA);
    if ((info + 1) == 0) {
      for (i = 0; i < 60; i++) {
        d = 0.0;
        for (i1 = 0; i1 < 120; i1++) {
          d += (static_cast<real_T>(c_a[i + (60 * i1)])) * igr2[i1];
        }

        del_z[i] = r1[i] - d;
      }

      for (i = 0; i < 3600; i++) {
        y_tmp[i] = b_H[i] - y_tmp[i];
      }

      mldivide(y_tmp, del_z);

      // old method (LU?)
      //   del_z = linsolve (R, linsolve (R, (r1-At*igr2), opUT), opU); %exploit matrix properties for solving 
    } else {
      // Not Positive Definite (problem? eg infeasible)
      for (i = 0; i < 60; i++) {
        d = 0.0;
        for (i1 = 0; i1 < 120; i1++) {
          d += (static_cast<real_T>(c_a[i + (60 * i1)])) * igr2[i1];
        }

        del_z[i] = r1[i] - d;
      }

      for (i = 0; i < 3600; i++) {
        y_tmp[i] = b_H[i] - y_tmp[i];
      }

      mldivide(y_tmp, del_z);

      // old method (LU?)
    }

    // Decide on suitable alpha (from Wright's paper)
    // Try Max Increment (alpha = 1)
    // Check lam and ftol > 0
    for (b_i = 0; b_i < 120; b_i++) {
      d = 0.0;
      for (i = 0; i < 60; i++) {
        d += IGA[b_i + (120 * i)] * del_z[i];
      }

      d = igr2[b_i] - d;
      del_lam[b_i] = d;
      ssq = ftol[b_i];
      mu_old = ((-ssq) + mesil[b_i]) - ((ilam[b_i] * ssq) * d);
      mesil[b_i] = mu_old;
      d += lam[b_i];
      nlamt[b_i] = d;
      ssq += mu_old;
      ilam[b_i] = ssq;
      x[b_i] = (d < 2.2204460492503131E-16);
      x[b_i + 120] = (ssq < 2.2204460492503131E-16);
    }

    y = false;
    b_i = 0;
    exitg1 = false;
    while ((!exitg1) && (b_i < 240)) {
      if (!x[b_i]) {
        b_i++;
      } else {
        y = true;
        exitg1 = true;
      }
    }

    if (!y) {
      // KKT met
      std::memcpy(&lam[0], &nlamt[0], 120U * (sizeof(real_T)));
      std::memcpy(&ftol[0], &ilam[0], 120U * (sizeof(real_T)));
      for (i = 0; i < 60; i++) {
        X_QP[i] += del_z[i];
      }
    } else {
      // KKT failed - solve by finding minimum ratio
      for (i = 0; i < 120; i++) {
        result[i] = nlamt[i];
        result[i + 120] = ilam[i];
      }

      idx = 0;
      b_i = 0;
      exitg1 = false;
      while ((!exitg1) && (b_i < 240)) {
        if (result[b_i] < 2.2204460492503131E-16) {
          idx++;
          ii_data[idx - 1] = static_cast<uint8_T>(static_cast<int32_T>(b_i + 1));
          if (idx >= 240) {
            exitg1 = true;
          } else {
            b_i++;
          }
        } else {
          b_i++;
        }
      }

      if (1 > idx) {
        ii_size_idx_0 = 0;
      } else {
        ii_size_idx_0 = idx;
      }

      // detects elements breaking KKT condition
      for (i = 0; i < 120; i++) {
        b_del_lam[i] = del_lam[i];
        b_del_lam[i + 120] = mesil[i];
      }

      for (i = 0; i < ii_size_idx_0; i++) {
        b_i = (static_cast<int32_T>(ii_data[i])) - 1;
        varargin_1_data[i] = 1.0 - (result[b_i] / b_del_lam[b_i]);
      }

      if (ii_size_idx_0 <= 2) {
        if (ii_size_idx_0 == 1) {
          ssq = varargin_1_data[0];
        } else if (varargin_1_data[0] > varargin_1_data[1]) {
          ssq = varargin_1_data[1];
        } else if (rtIsNaN(varargin_1_data[0])) {
          if (!rtIsNaN(varargin_1_data[1])) {
            ssq = varargin_1_data[1];
          } else {
            ssq = varargin_1_data[0];
          }
        } else {
          ssq = varargin_1_data[0];
        }
      } else {
        if (!rtIsNaN(varargin_1_data[0])) {
          idx = 1;
        } else {
          idx = 0;
          b_i = 2;
          exitg1 = false;
          while ((!exitg1) && (b_i <= ii_size_idx_0)) {
            if (!rtIsNaN(varargin_1_data[b_i - 1])) {
              idx = b_i;
              exitg1 = true;
            } else {
              b_i++;
            }
          }
        }

        if (idx == 0) {
          ssq = varargin_1_data[0];
        } else {
          ssq = varargin_1_data[idx - 1];
          i = idx + 1;
          for (b_i = i; b_i <= ii_size_idx_0; b_i++) {
            d = varargin_1_data[b_i - 1];
            if (ssq > d) {
              ssq = d;
            }
          }
        }
      }

      ssq *= 0.995;

      // solves for min ratio (max value of alpha allowed)
      // Increment
      for (i = 0; i < 120; i++) {
        lam[i] += ssq * del_lam[i];
        ftol[i] += ssq * mesil[i];
      }

      for (i = 0; i < 60; i++) {
        X_QP[i] += ssq * del_z[i];
      }
    }

    // Complimentary Gap
    mu_old = mu;
    ssq = 0.0;
    for (i = 0; i < 120; i++) {
      ssq += ftol[i] * lam[i];
    }

    mu = ssq / 120.0;
    if (mu < 0.001) {
      exitflag = 1;
    } else {
      // Solve for new Sigma
      ssq = mu / mu_old;
      if (ssq > 0.1) {
        // to tune
        ssq = 0.1;
      }

      for (b_i = 0; b_i < 120; b_i++) {
        esig[b_i] = ssq;
      }
    }

    unusedU0++;
  }

  // Check for failure
  //  num_iter = iter;
  //  solution to warm-start next iteration
  //  Extract first control
  Fx = X_QP[0];
  Fy = X_QP[1];
  Fz = X_QP[2];
}

template<typename T>
void CoordinatorBase<T>::mldivide(double A[3600], double B[60])
  {
    double b_A[3600];
    int i;
    int ix;
    int iy;
    int jA;
    int k;
    signed char ipiv[60];
    std::memcpy(&b_A[0], &A[0], 3600U * sizeof(double));
    for (i = 0; i < 60; i++) {
      ipiv[i] = static_cast<signed char>(i + 1);
    }

    for (int j = 0; j < 59; j++) {
      double smax;
      int b_tmp;
      int jp1j;
      int mmj_tmp;
      signed char i1;
      mmj_tmp = 58 - j;
      b_tmp = j * 61;
      jp1j = b_tmp + 2;
      iy = 60 - j;
      jA = 0;
      ix = b_tmp;
      smax = std::abs(b_A[b_tmp]);
      for (k = 2; k <= iy; k++) {
        double s;
        ix++;
        s = std::abs(b_A[ix]);
        if (s > smax) {
          jA = k - 1;
          smax = s;
        }
      }

      if (b_A[b_tmp + jA] != 0.0) {
        if (jA != 0) {
          iy = j + jA;
          ipiv[j] = static_cast<signed char>(iy + 1);
          ix = j;
          for (k = 0; k < 60; k++) {
            smax = b_A[ix];
            b_A[ix] = b_A[iy];
            b_A[iy] = smax;
            ix += 60;
            iy += 60;
          }
        }

        i = (b_tmp - j) + 60;
        for (jA = jp1j; jA <= i; jA++) {
          b_A[jA - 1] /= b_A[b_tmp];
        }
      }

      iy = b_tmp + 60;
      jA = b_tmp;
      for (k = 0; k <= mmj_tmp; k++) {
        smax = b_A[iy];
        if (b_A[iy] != 0.0) {
          ix = b_tmp + 1;
          i = jA + 62;
          jp1j = (jA - j) + 120;
          for (int ijA = i; ijA <= jp1j; ijA++) {
            b_A[ijA - 1] += b_A[ix] * -smax;
            ix++;
          }
        }

        iy += 60;
        jA += 60;
      }

      i1 = ipiv[j];
      if (i1 != j + 1) {
        smax = B[j];
        B[j] = B[i1 - 1];
        B[i1 - 1] = smax;
      }
    }

    for (k = 0; k < 60; k++) {
      iy = 60 * k;
      if (B[k] != 0.0) {
        i = k + 2;
        for (jA = i; jA < 61; jA++) {
          B[jA - 1] -= B[k] * b_A[(jA + iy) - 1];
        }
      }
    }

    for (k = 59; k >= 0; k--) {
      iy = 60 * k;
      if (B[k] != 0.0) {
        B[k] /= b_A[k + iy];
        for (jA = 0; jA < k; jA++) {
          B[jA] -= B[k] * b_A[jA + iy];
        }
      }
    }
  }

