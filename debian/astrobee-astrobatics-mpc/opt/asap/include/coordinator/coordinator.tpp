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
double inti_e_x=0;
double inti_e_y=0;
double inti_e_z=0;
float q0_x = 0;
float q0_y = 0;
float q0_z = 0;

double z_nominal[6];
double zp_nextNominal[6];
double v_mpc[3];
bool initial_run = true;
bool initialzation = false;

int count=0;

double kn_tilda[3];
double kN[3];
bool rotation_done = false;
void step_PID();

// Function Declarations
//void main_MPC_Guidance_v3_sand();
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
   uint  speed_ =3;
  flight_mode_.speed=3;
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
    double prod_val;
    prod_val=qx*q0_x + qy*q0_y + qz*q0_z;
    double deno;
    deno=sqrt(prod_val*prod_val);
    if ((prod_val==0)){
    deno=1;
    }
    if(prod_val/deno <-0.9 ){
      qx=-qx;
      qy=-qy;
      qz=-qz;
      qw=-qw;
      //ROS_INFO(" Quaternion sign change detected >>>>>>>>>>>>>>>>>>>>> ");
    }
    q0_x=qx;
    q0_y=qy;
    q0_z=qz;
    attitude.x=qx;
    attitude.y=qy;
    attitude.z=qz;
    attitude.w=qw;
    omega.x=wx;
    omega.y=wy;
    omega.z=wz;
    geometry_msgs::Vector3 torque, axes_rot;
    double r=0, p=0, y=3.14159265/180*225;  // Rotate the previous pose by 45* about Z
    axes_rot.x = 0;
    axes_rot.y = 0;
    axes_rot.z = 1;
    double angle = 45/180*3.14570;
        q_ref.setRPY(r, p, y);
        //q_ref.setRotation(axes_rot, 45/180*3.14570);
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


    position_.x = px ;//+ (cm_x*R_11 + cm_y*R_21 + cm_z*R_31);
    position_.y = py ;//+ (cm_x*R_12 + cm_y*R_22 + cm_z*R_32);
    position_.z = pz ;//+ (cm_x*R_13 + cm_y*R_23 + cm_z*R_33);

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
    MPC_Guidance_v3_sand();
    v_mpc[0]=Fx;
    v_mpc[1]=Fy;
    v_mpc[2]=Fz;
    nominal_dynamics();
    //sqrt(q_e.getX()*q_e.getX()+q_e.getY()*q_e.getY()+q_e.getZ()*q_e.getZ())>0.005
    if (count==0){
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

    
    kn_tilda[0]=Fx;
    kn_tilda[1]=Fy;
    kn_tilda[2]=Fz;

      /* double sx=x0[0]-zp_nextNominal[0];
      double sy=x0[1]-zp_nextNominal[1];
      double sz=x0[2]-zp_nextNominal[2];
      double svx=x0[3]-zp_nextNominal[3];
      double svy=x0[4]-zp_nextNominal[4];
      double svz=x0[5]-zp_nextNominal[5]; */

    tubing_mpc();
    count+=1;
    if (count==11){

      count =0;
    }
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
  //sub-checkpoint --------------------------------------->>>>>
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
  // checkpoint nominal dynamics --------------------------->>>>>>>>
   double b_a[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0 };

   double a[18] = { 0.069074, 0.0, 0.0, 0.069074, 0.0, 0.0, 0.0,
    0.069074, 0.0, 0.0, 0.069074, 0.0, 0.0, 0.0, 0.069074, 0.0, 0.0,
    0.069074 };

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



 double qw = q_e.getW();
 double qx = q_e.getX();
 double qy = q_e.getY();
 double qz = q_e.getZ();
 double fx = Fx; 
 double fy = Fy;
 double fz = Fz;
 double Omega1 = arg_omegax;
 double Omega2 = arg_omegay;
 double Omega3 = arg_omegaz;
//check point attitude controller -------------------------------->>>
float a1[18] = { -0.1171782276523663, -0.0, -0.0, -0.0,
    -0.42978465701183144, -0.0, -0.0, -0.0, -0.35586060535946518,
    -1.171782276523663, -0.0, -0.0, -0.0, -4.2978465701183142, -0.0, -0.0, -0.0,
    -3.5586060535946515 };

  static const real_T b_a[9] = { 0.0, -0.048539486885480869, 0.0,
    0.048539486885480869, 0.0, 0.097078973770961738, -0.0, -0.097078973770961738,
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
    dv1[i] = ((dv[i] * Fx) + (dv[i + 3] * Fy)) + (dv[i + 6] * Fz);
    d = 0.0;
    for (int32_T i1 = 0; i1 < 6; i1++) {
      d += a[i + (3 * i1)] * b_qx[i1];
    }

    U[i] = d;
  }

  d = dv1[0];
  d1 = dv1[1];
  d2 = dv1[2];
  for (i = 0; i < 3; i++) {
    U[i] -= ((b_a[i] * d) + (b_a[i + 3] * d1)) + (b_a[i + 6] * d2);
  }
 inti_e_x-=q_e.getX()*0.16;
 inti_e_y-=q_e.getY()*0.16;
 inti_e_z-=q_e.getZ()*0.16;
 double Ki=0.0002;
 
  arg_tau_x = U[0] + Ki*inti_e_x;
  arg_tau_y = U[1] + Ki*inti_e_y;
  arg_tau_z = U[2] + Ki*inti_e_z;


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


//template<typename T>
/* void CoordinatorBase<T>::main_MPC_Guidance_v3_sand()
{

  // Initialize function 'MPC_Guidance_v3_sand' input arguments.
  // Initialize function input argument 'x0'.
  // Call the entry-point 'MPC_Guidance_v3_sand'.
  
  MPC_Guidance_v3_sand();
} */


template<typename T>
bool  CoordinatorBase<T>::rtIsNaN(double value){
  return ((value!=value) ? true : false);
}

template<typename T>
void CoordinatorBase<T>::MPC_Guidance_v3_sand()
{
   double b[14400] = { 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
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
    0.0, 0.0, 0.0, 0.0, 199577.444, 3.472E-9, -2.5369E-8, 59895.8732, -1.1471E-8,
    -1.8399E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.472E-9, 199577.444,
    6.4189E-9, 1.2957E-11, 59895.8732, -1.4055E-8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -2.5369E-8, 6.4189E-9, 199577.444, -1.3437E-8, 6.3899E-9,
    59895.8732, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 59895.8732,
    1.2957E-11, -1.3437E-8, 1.9621392775E+6, 2.1986E-11, 6.1217E-11, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.1471E-8, 59895.8732, 6.3899E-9,
    2.1986E-11, 1.9621392775E+6, 2.4262E-9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -1.8399E-8, -1.4055E-8, 59895.8732, 6.1217E-11, 2.4262E-9,
    1.9621392775E+6 };

   double b_b[7200] = { 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00046764, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00049436, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00052109, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0,
    1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00046764, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00049436, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00052109,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00046764, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00049436, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00052109, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00046764,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00049436, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00046764, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00049436, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00046764, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00049436, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00046764,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0004142,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00046764, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00046764, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00044092,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0004142, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0004142,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00038747, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00038747, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00036075, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0,
    0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00033403, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00030731, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00028058, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00025386, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00022714, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0,
    0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00020042, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00020042, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0001737, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00014697, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.00012025, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 9.3528E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701,
    0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 6.6806E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 6.6806E-5,
    0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0,
    4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0,
    0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 4.0084E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0,
    0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.3361E-5, 0.0, 0.0, 0.0016701, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.3361E-5, 0.0, 0.0, 0.0016701 };

   double b_H[3600] = {221.86203408940816, -1.7935051601617053E-14,
    -6.8847619827685469E-14, 21.293254106153611, -1.8032929589223377E-14,
    -6.6941200628136871E-14, 20.724478346088514, -1.8130770959550388E-14,
    -6.5035494641906465E-14, 20.155702871419727, -1.8228612329877396E-14,
    -6.3129788655676059E-14, 19.586927824949626, -1.83264537002044E-14,
    -6.1224082669445666E-14, 19.018149269924553, -1.8424331687810722E-14,
    -5.9317663469897069E-14, 18.449375509349135, -1.8522173058137733E-14,
    -5.7411957483666675E-14, 17.880602605443634, -1.8620014428464737E-14,
    -5.5506251497436269E-14, 17.311830701026452, -1.8717855798791748E-14,
    -5.3600545511205863E-14, 16.743055859333261, -1.8815733786398069E-14,
    -5.169412631165726E-14, 16.174286383363331, -1.891357515672508E-14,
    -4.9788420325426867E-14, 15.605518335310194, -1.9011416527052085E-14,
    -4.7882714339196467E-14, 15.03675185799225, -1.9109257897379093E-14,
    -4.5977008352966061E-14, 14.467987094227905, -1.92070992677061E-14,
    -4.4071302366735668E-14, 13.89922010725283, -1.9304977255312425E-14,
    -4.216488316718707E-14, 13.330459200066313, -1.9402818625639433E-14,
    -4.0259177180956664E-14, 12.761699618977392, -1.9500667319422303E-14,
    -3.8353328552062629E-14, 12.192942322934794, -1.9598516013205174E-14,
    -3.6447479923168587E-14, 11.624187454740882, -1.9696364706988044E-14,
    -3.4541631294274545E-14, 11.055434749242425, -1.9794217062498848E-14,
    -3.2635711344048686E-14, -1.7935051601617056E-14, 221.86203408940816,
    3.6789383283423793E-15, -1.7007601186151857E-14, 21.293254106153611,
    2.9296110745875248E-15, -1.6080497740539058E-14, 20.724478346088514,
    2.1805641527644851E-15, -1.5153394294926256E-14, 20.155702871419727,
    1.431517230941439E-15, -1.4226290849313458E-14, 19.586927824949626,
    6.82470309118396E-16, -1.3298840433848254E-14, 19.018149269924553,
    -6.685694463645525E-17, -1.2371736988235454E-14, 18.449375509349135,
    -8.1590386645949818E-16, -1.1444633542622655E-14, 17.880602605443634,
    -1.564950788282538E-15, -1.0517530097009855E-14, 17.311830701026452,
    -2.3139977101055777E-15, -9.5900796815446542E-15, 16.743055859333261,
    -3.0633249638604417E-15, -8.6629762359318554E-15, 16.174286383363331,
    -3.8123718856834814E-15, -7.7358727903190535E-15, 15.605518335310194,
    -4.5614188075065149E-15, -6.8087693447062547E-15, 15.03675185799225,
    -5.3104657293295578E-15, -5.8816658990934536E-15, 14.467987094227905,
    -6.0595126511525976E-15, -4.9542154836282548E-15, 13.89922010725283,
    -6.8088399049074552E-15, -4.0271120380154537E-15, 13.330459200066313,
    -7.5578868267305013E-15, -3.0999391984321746E-15, 12.761699618977392,
    -8.3069898149399028E-15, -2.172766358848894E-15, 12.192942322934794,
    -9.0560928031493074E-15, -1.2455935192656142E-15, 11.624187454740882,
    -9.8051957913587088E-15, -3.1838598269709405E-16, 11.055434749242425,
    -1.0554326812761297E-14, -6.8847619827685469E-14, 3.6789383283423824E-15,
    221.86203408940816, -6.6498175672884873E-14, 4.7550042442029233E-15,
    21.293254106153611, -6.4149610472126465E-14, 5.8306675914454805E-15,
    20.724478346088514, -6.1801045271368057E-14, 6.9063309386880424E-15,
    20.155702871419727, -5.9452480070609662E-14, 7.9819942859306012E-15,
    19.586927824949626, -5.7103035915809066E-14, 9.0580602017911437E-15,
    19.018149269924553, -5.4754470715050665E-14, 1.0133723549033701E-14,
    18.449375509349135, -5.2405905514292263E-14, 1.120938689627626E-14,
    17.880602605443634, -5.0057340313533867E-14, 1.2285050243518825E-14,
    17.311830701026452, -4.7707896158733259E-14, 1.3361116159379364E-14,
    16.743055859333261, -4.5359330957974864E-14, 1.4436779506621921E-14,
    16.174286383363331, -4.3010765757216462E-14, 1.5512442853864483E-14,
    15.605518335310194, -4.0662200556458061E-14, 1.6588106201107042E-14,
    15.03675185799225, -3.8313635355699665E-14, 1.7663769548349604E-14,
    14.467987094227905, -3.5964191200899063E-14, 1.8739835464210143E-14,
    13.89922010725283, -3.3615626000140662E-14, 1.9815498811452702E-14,
    13.330459200066313, -3.1266885008573821E-14, 2.0891242672418861E-14,
    12.761699618977392, -2.8918144017006981E-14, 2.1966986533385016E-14,
    12.192942322934794, -2.656940302544014E-14, 2.3042730394351172E-14,
    11.624187454740882, -2.422057413846908E-14, 2.4118514512179124E-14,
    11.055434749242425, 21.293254106153611, -1.7007601186151854E-14,
    -6.6498175672884873E-14, 221.28260872460294, -1.7100517735060578E-14,
    -6.4628008414096468E-14, 20.714120550445791, -1.7193399522821188E-14,
    -6.2758540806358862E-14, 20.145632518879907, -1.7286281310581795E-14,
    -6.0889073198621269E-14, 19.577144772710334, -1.73791630983424E-14,
    -5.9019605590883664E-14, 19.008653481824105, -1.7472079647251122E-14,
    -5.7149438332095271E-14, 18.440166735896412, -1.7564961435011732E-14,
    -5.5279970724357672E-14, 17.871680703820211, -1.7657843222772339E-14,
    -5.3410503116620072E-14, 17.303195528413923, -1.7750725010532949E-14,
    -5.1541035508882467E-14, 16.734707379580612, -1.784364155944167E-14,
    -4.9670868250094062E-14, 16.1662243469848, -1.793652334720228E-14,
    -4.7801400642356469E-14, 15.597742599514085, -1.8029405134962886E-14,
    -4.5931933034618863E-14, 15.029262279960165, -1.8122286922723493E-14,
    -4.4062465426881264E-14, 14.460783531141434, -1.82151687104841E-14,
    -4.2192997819143664E-14, 13.892302522960961, -1.8308085259392827E-14,
    -4.0322830560355272E-14, 13.323827345083265, -1.8400967047153434E-14,
    -3.8453362952617666E-14, 12.755353371812895, -1.8493855787143661E-14,
    -3.6583755414669909E-14, 12.186881540759753, -1.8586744527133895E-14,
    -3.4714147876722146E-14, 11.618411994752933, -1.8679633267124123E-14,
    -3.284454033877439E-14, 11.049944479303266, -1.8772525483229169E-14,
    -3.0974862835721548E-14, -1.8032929589223377E-14, 21.293254106153611,
    4.7550042442029186E-15, -1.7100517735060578E-14, 221.28260872460294,
    4.0148495073996836E-15, -1.6168454706881378E-14, 20.714120550445791,
    3.2749716709843226E-15, -1.5236391678702178E-14, 20.145632518879907,
    2.5350938345689586E-15, -1.4304328650522978E-14, 19.577144772710334,
    1.7952159981535976E-15, -1.3371916796360176E-14, 19.008653481824105,
    1.0550612613503626E-15, -1.2439853768180976E-14, 18.440166735896412,
    3.1518342493500171E-16, -1.1507790740001776E-14, 17.871680703820211,
    -4.2469441148035922E-16, -1.0575727711822576E-14, 17.303195528413923,
    -1.1645722478957201E-15, -9.6433158576597761E-15, 16.734707379580612,
    -1.9047269846989614E-15, -8.7112528294805777E-15, 16.1662243469848,
    -2.6446048211143224E-15, -7.7791898013013745E-15, 15.597742599514085,
    -3.384482657529677E-15, -6.8471267731221761E-15, 15.029262279960165,
    -4.1243604939450379E-15, -5.915063744942976E-15, 14.460783531141434,
    -4.8642383303603957E-15, -4.9826518907801774E-15, 13.892302522960961,
    -5.604393067163637E-15, -4.0505888626009758E-15, 13.323827345083265,
    -6.3442709035790011E-15, -3.1184560692250567E-15, 12.755353371812895,
    -7.0842041200719349E-15, -2.1863232758491364E-15, 12.186881540759753,
    -7.8241373365648719E-15, -1.2541904824732161E-15, 11.618411994752933,
    -8.5640705530578058E-15, -3.2202280649893605E-16, 11.049944479303266,
    -9.3040314595895308E-15, -6.6941200628136871E-14, 2.9296110745875248E-15,
    21.293254106153611, -6.4628008414096468E-14, 4.014849507399682E-15,
    221.28260872460294, -6.231568159183086E-14, 5.0996819400499212E-15,
    20.714120550445791, -6.0003354769565265E-14, 6.184514372700162E-15,
    20.145632518879907, -5.769102794729967E-14, 7.2693468053504028E-15,
    19.577144772710334, -5.5377835733259267E-14, 8.3545852381625647E-15,
    19.008653481824105, -5.3065508910993665E-14, 9.4394176708128E-15,
    18.440166735896412, -5.0753182088728064E-14, 1.0524250103463042E-14,
    17.871680703820211, -4.8440855266462469E-14, 1.1609082536113286E-14,
    17.303195528413923, -4.6127663052422066E-14, 1.2694320968925444E-14,
    16.734707379580612, -4.3815336230156458E-14, 1.3779153401575682E-14,
    16.1662243469848, -4.1503009407890863E-14, 1.4863985834225924E-14,
    15.597742599514085, -3.9190682585625261E-14, 1.5948818266876162E-14,
    15.029262279960165, -3.687835576335966E-14, 1.7033650699526406E-14,
    14.460783531141434, -3.4565163549319263E-14, 1.8118889132338561E-14,
    13.892302522960961, -3.2252836727053662E-14, 1.9203721564988802E-14,
    13.323827345083265, -2.99403368264331E-14, 2.0288635197671428E-14,
    12.755353371812895, -2.7627836925812544E-14, 2.1373548830354055E-14,
    12.186881540759753, -2.5315337025191979E-14, 2.2458462463036675E-14,
    11.618411994752933, -2.3002750585393941E-14, 2.3543416695735491E-14,
    11.049944479303266, 20.72447834608851, -1.6080497740539055E-14,
    -6.4149610472126465E-14, 20.714120550445791, -1.6168454706881378E-14,
    -6.2315681591830873E-14, 220.70376662185254, -1.6256378767512389E-14,
    -6.0482438805392465E-14, 20.135565926748725, -1.6344302828143397E-14,
    -5.8649196018954069E-14, 19.567365374236193, -1.64322268887744E-14,
    -5.6815953232515661E-14, 18.999161240805375, -1.6520183855116724E-14,
    -5.4982024352220069E-14, 18.430961402876555, -1.6608107915747732E-14,
    -5.3148781565781667E-14, 17.86276213600221, -1.6696031976378739E-14,
    -5.1315538779343265E-14, 17.294563582979354, -1.678395603700975E-14,
    -4.9482295992904869E-14, 16.726362020311825, -1.687191300335207E-14,
    -4.7648367112609264E-14, 16.158165324435924, -1.6959837063983081E-14,
    -4.5815124326170869E-14, 15.589969770893457, -1.7047761124614088E-14,
    -4.3981881539732461E-14, 15.021775502476093, -1.7135685185245093E-14,
    -4.2148638753294065E-14, 14.453582661975519, -1.72236092458761E-14,
    -4.0315395966855669E-14, 13.885387525895544, -1.7311566212218426E-14,
    -3.8481467086560071E-14, 13.317197970672479, -1.7399490272849434E-14,
    -3.6648224300121669E-14, 12.749009498585213, -1.74874209146227E-14,
    -3.4814844294911829E-14, 12.180823025886085, -1.7575351556395975E-14,
    -3.2981464289701989E-14, 11.612638695404186, -1.7663282198169243E-14,
    -3.1148084284492149E-14, 11.044456263337146, -1.775121613051365E-14,
    -2.9314635669896587E-14, -1.8130770959550385E-14, 20.72447834608851,
    5.8306675914454773E-15, -1.7193399522821188E-14, 20.714120550445791,
    5.0996819400499244E-15, -1.6256378767512386E-14, 220.70376662185254,
    4.3689697587820813E-15, -1.5319358012203587E-14, 20.135565926748725,
    3.6382575775142383E-15, -1.4382337256894788E-14, 19.567365374236193,
    2.9075453962463952E-15, -1.3444965820165585E-14, 18.999161240805375,
    2.1765597448508422E-15, -1.2507945064856784E-14, 18.430961402876555,
    1.4458475635829992E-15, -1.1570924309547985E-14, 17.86276213600221,
    7.1513538231516246E-16, -1.0633903554239184E-14, 17.294563582979354,
    -1.5576798952680586E-17, -9.6965321175099844E-15, 16.726362020311825,
    -7.46562450348243E-16, -8.7595113622011854E-15, 16.158165324435924,
    -1.4772746316160829E-15, -7.8224906068923832E-15, 15.589969770893457,
    -2.2079868128839165E-15, -6.8854698515835841E-15, 15.021775502476093,
    -2.9386989941517596E-15, -5.9484490962747835E-15, 14.453582661975519,
    -3.6694111754195963E-15, -5.0110776595455852E-15, 13.885387525895544,
    -4.4003968268151587E-15, -4.0740569042367837E-15, 13.317197970672479,
    -5.1311090080830018E-15, -3.1369660126439045E-15, 12.749009498585213,
    -5.8618758833763843E-15, -2.1998751210510241E-15, 12.180823025886085,
    -6.59264275866977E-15, -1.2627842294581441E-15, 11.612638695404186,
    -7.32340963396315E-15, -3.2565826972322408E-16, 11.044456263337146,
    -8.0542038562693063E-15, -6.5035494641906465E-14, 2.1805641527644851E-15,
    20.72447834608851, -6.2758540806358862E-14, 3.2749716709843226E-15,
    20.714120550445791, -6.0482438805392465E-14, 4.3689697587820813E-15,
    220.70376662185254, -5.8206336804426067E-14, 5.4629678465798432E-15,
    20.135565926748725, -5.5930234803459669E-14, 6.5569659343776026E-15,
    19.567365374236193, -5.3653280967912066E-14, 7.6513734525974426E-15,
    18.999161240805375, -5.1377178966945669E-14, 8.7453715403952012E-15,
    18.430961402876555, -4.9101076965979265E-14, 9.8393696281929615E-15,
    17.86276213600221, -4.6824974965012861E-14, 1.0933367715990726E-14,
    17.294563582979354, -4.4548021129465264E-14, 1.2027775234210564E-14,
    16.726362020311825, -4.227191912849886E-14, 1.3121773322008323E-14,
    16.158165324435924, -3.9995817127532462E-14, 1.4215771409806083E-14,
    15.589969770893457, -3.7719715126566065E-14, 1.5309769497603845E-14,
    15.021775502476093, -3.5443613125599667E-14, 1.6403767585401604E-14,
    14.453582661975519, -3.3166659290052064E-14, 1.7498175103621444E-14,
    13.885387525895544, -3.0890557289085667E-14, 1.8592173191419206E-14,
    13.317197970672479, -2.8614284921203021E-14, 1.9686253165301381E-14,
    12.749009498585213, -2.6338012553320383E-14, 2.0780333139183555E-14,
    12.180823025886085, -2.4061740185437741E-14, 2.1874413113065733E-14,
    11.612638695404186, -2.1785382634096982E-14, 2.2968534029990113E-14,
    11.044456263337146, 20.155702871419727, -1.5153394294926253E-14,
    -6.180104527136807E-14, 20.145632518879907, -1.5236391678702178E-14,
    -6.0003354769565265E-14, 20.135565926748729, -1.5319358012203587E-14,
    -5.8206336804426067E-14, 220.12549933441716, -1.5402324345704995E-14,
    -5.6409318839286856E-14, 19.557585975575, -1.54852906792064E-14,
    -5.4612300874147658E-14, 18.989668999610288, -1.5568288062982323E-14,
    -5.2814610372344866E-14, 18.421756069691014, -1.5651254396483731E-14,
    -5.1017592407205668E-14, 17.853843568023862, -1.5734220729985136E-14,
    -4.922057444206647E-14, 17.285931637411185, -1.5817187063486548E-14,
    -4.7423556476927265E-14, 16.718016660936151, -1.5900184447262467E-14,
    -4.5625865975124461E-14, 16.150106301780163, -1.5983150780763879E-14,
    -4.3828848009985263E-14, 15.582196942165929, -1.6066117114265284E-14,
    -4.2031830044846058E-14, 15.014288724885136, -1.6149083447766692E-14,
    -4.023481207970686E-14, 14.446381792729436, -1.6232049781268098E-14,
    -3.8437794114567662E-14, 13.878472528776683, -1.6315047165044026E-14,
    -3.6640103612764864E-14, 13.310568596208251, -1.6398013498545431E-14,
    -3.4843085647625672E-14, 12.742665625304086, -1.6480986042101742E-14,
    -3.3045933175153749E-14, 12.174764510985693, -1.6563958585658052E-14,
    -3.1248780702681825E-14, 11.606865396055438, -1.6646931129214363E-14,
    -2.9451628230209908E-14, 11.038968047371029, -1.6729906777798128E-14,
    -2.7654408504071629E-14, -1.8228612329877393E-14, 20.155702871419727,
    6.9063309386880393E-15, -1.7286281310581795E-14, 20.145632518879907,
    6.1845143727001652E-15, -1.6344302828143397E-14, 20.135565926748729,
    5.4629678465798463E-15, -1.5402324345704995E-14, 220.12549933441716,
    4.7414213204595211E-15, -1.4460345863266594E-14, 19.557585975575,
    4.0198747943391991E-15, -1.3518014843970993E-14, 18.989668999610288,
    3.298058228351325E-15, -1.2576036361532592E-14, 18.421756069691014,
    2.576511702231003E-15, -1.1634057879094194E-14, 17.853843568023862,
    1.8549651761106841E-15, -1.0692079396655792E-14, 17.285931637411185,
    1.1334186499903621E-15, -9.7497483773601928E-15, 16.718016660936151,
    4.1160208400247852E-16, -8.8077698949217931E-15, 16.150106301780163,
    -3.0994444211784033E-16, -7.8657914124833918E-15, 15.582196942165929,
    -1.031490968238156E-15, -6.9238129300449921E-15, 15.014288724885136,
    -1.753037494358478E-15, -5.9818344476065917E-15, 14.446381792729436,
    -2.4745840204787969E-15, -5.0395034283109929E-15, 13.878472528776683,
    -3.1964005864666742E-15, -4.0975249458725917E-15, 13.310568596208251,
    -3.9179471125869993E-15, -3.1554759560627527E-15, 12.742665625304086,
    -4.6395476466808274E-15, -2.2134269662529122E-15, 12.174764510985693,
    -5.3611481807746617E-15, -1.2713779764430721E-15, 11.606865396055438,
    -6.0827487148684929E-15, -3.2929373294751206E-16, 11.038968047371029,
    -6.8043762529490818E-15, -6.3129788655676059E-14, 1.4315172309414437E-15,
    20.155702871419727, -6.0889073198621257E-14, 2.5350938345689617E-15,
    20.145632518879907, -5.8649196018954069E-14, 3.63825757751424E-15,
    20.135565926748729, -5.6409318839286856E-14, 4.7414213204595211E-15,
    220.12549933441716, -5.4169441659619662E-14, 5.8445850634048E-15,
    19.557585975575, -5.1928726202564866E-14, 6.9481616670323228E-15,
    18.989668999610288, -4.9688849022897666E-14, 8.0513254099776E-15,
    18.421756069691014, -4.7448971843230466E-14, 9.15448915292288E-15,
    17.853843568023862, -4.5209094663563265E-14, 1.0257652895868164E-14,
    17.285931637411185, -4.2968379206508463E-14, 1.1361229499495682E-14,
    16.718016660936151, -4.0728502026841262E-14, 1.2464393242440962E-14,
    16.150106301780163, -3.8488624847174062E-14, 1.3567556985386242E-14,
    15.582196942165929, -3.6248747667506862E-14, 1.4670720728331521E-14,
    15.014288724885136, -3.4008870487839662E-14, 1.57738844712768E-14,
    14.446381792729436, -3.1768155030784859E-14, 1.6877461074904321E-14,
    13.878472528776683, -2.9528277851117659E-14, 1.7980624817849604E-14,
    13.310568596208251, -2.7288233015972943E-14, 1.9083871132931333E-14,
    12.742665625304086, -2.5048188180828221E-14, 2.0187117448013062E-14,
    12.174764510985693, -2.28081433456835E-14, 2.1290363763094787E-14,
    11.606865396055438, -2.0568014682800019E-14, 2.2393651364244739E-14,
    11.038968047371029, 19.586927824949626, -1.4226290849313454E-14,
    -5.9452480070609662E-14, 19.577144772710337, -1.4304328650522978E-14,
    -5.7691027947299658E-14, 19.567365374236193, -1.4382337256894788E-14,
    -5.5930234803459669E-14, 19.557585975575, -1.4460345863266594E-14,
    -5.4169441659619656E-14, 219.54780657671341, -1.45383544696384E-14,
    -5.2408648515779662E-14, 18.980176758228151, -1.4616392270847921E-14,
    -5.0647196392469664E-14, 18.412550736329113, -1.469440087721973E-14,
    -4.8886403248629663E-14, 17.844924999879826, -1.4772409483591537E-14,
    -4.7125610104789662E-14, 17.277299691682664, -1.4850418089963349E-14,
    -4.5364816960949662E-14, 16.709671301426873, -1.492845589117287E-14,
    -4.3603364837639664E-14, 16.142047279017515, -1.500646449754468E-14,
    -4.1842571693799663E-14, 15.574424113331517, -1.5084473103916486E-14,
    -4.0081778549959668E-14, 15.006801947187274, -1.5162481710288292E-14,
    -3.8320985406119661E-14, 14.439180923376467, -1.52404903166601E-14,
    -3.6560192262279667E-14, 13.871557531577656, -1.5318528117869626E-14,
    -3.4798740138969669E-14, 13.303939221690579, -1.5396536724241432E-14,
    -3.3037946995129662E-14, 12.736321751969514, -1.5474551169580782E-14,
    -3.1277022055395668E-14, 12.168705996031857, -1.5552565614920133E-14,
    -2.9516097115661662E-14, 11.601092096679968, -1.5630580060259483E-14,
    -2.7755172175927668E-14, 11.033479831404909, -1.5708597425082606E-14,
    -2.5994181338246668E-14, -1.83264537002044E-14, 19.586927824949626,
    7.9819942859306012E-15, -1.73791630983424E-14, 19.577144772710337,
    7.269346805350406E-15, -1.6432226888774404E-14, 19.567365374236193,
    6.556965934377605E-15, -1.54852906792064E-14, 19.557585975575,
    5.8445850634048009E-15, -1.4538354469638404E-14, 219.54780657671341,
    5.1322041924319967E-15, -1.3591063867776401E-14, 18.980176758228151,
    4.4195567118518046E-15, -1.2644127658208401E-14, 18.412550736329113,
    3.7071758408790036E-15, -1.16971914486404E-14, 17.844924999879826,
    2.9947949699062027E-15, -1.07502552390724E-14, 17.277299691682664,
    2.2824140989334017E-15, -9.8029646372104011E-15, 16.709671301426873,
    1.5697666183532E-15, -8.8560284276424E-15, 16.142047279017515,
    8.5738574738039911E-16, -7.9090922180743989E-15, 15.574424113331517,
    1.4500487640760445E-16, -6.9621560085064E-15, 15.006801947187274,
    -5.6737599456519653E-16, -6.0152197989384E-15, 14.439180923376467,
    -1.2797568655379943E-15, -5.0679291970764007E-15, 13.871557531577656,
    -1.9924043461181959E-15, -4.1209929875083996E-15, 13.303939221690579,
    -2.704785217091E-15, -3.1739858994816005E-15, 12.736321751969514,
    -3.4172194099852767E-15, -2.2269788114548003E-15, 12.168705996031857,
    -4.12965360287956E-15, -1.279971723428E-15, 11.601092096679968,
    -4.8420877957738364E-15, -3.3292919617180004E-16, 11.033479831404909,
    -5.5545486496288573E-15, -6.1224082669445666E-14, 6.8247030911840393E-16,
    19.586927824949626, -5.9019605590883664E-14, 1.7952159981536024E-15,
    19.577144772710337, -5.6815953232515667E-14, 2.9075453962464E-15,
    19.567365374236193, -5.4612300874147658E-14, 4.0198747943392023E-15,
    19.557585975575, -5.2408648515779668E-14, 5.1322041924320007E-15,
    219.54780657671341, -5.0204171437217665E-14, 6.2449498814672023E-15,
    18.980176758228151, -4.8000519078849663E-14, 7.35727927956E-15,
    18.412550736329113, -4.579686672048166E-14, 8.4696086776528E-15,
    17.844924999879826, -4.3593214362113657E-14, 9.5819380757456053E-15,
    17.277299691682664, -4.1388737283551661E-14, 1.0694683764780804E-14,
    16.709671301426873, -3.9185084925183665E-14, 1.1807013162873601E-14,
    16.142047279017515, -3.6981432566815662E-14, 1.2919342560966404E-14,
    15.574424113331517, -3.4777780208447665E-14, 1.4031671959059201E-14,
    15.006801947187274, -3.2574127850079656E-14, 1.5144001357152002E-14,
    14.439180923376467, -3.0369650771517666E-14, 1.6256747046187204E-14,
    13.871557531577656, -2.8165998413149661E-14, 1.736907644428E-14,
    13.303939221690579, -2.5962181110742865E-14, 1.8481489100561285E-14,
    12.736321751969514, -2.375836380833606E-14, 1.9593901756842565E-14,
    12.168705996031857, -2.1554546505929261E-14, 2.0706314413123845E-14,
    11.601092096679968, -1.935064673150306E-14, 2.1818768698499362E-14,
    11.033479831404909, 19.018149269924553, -1.3298840433848256E-14,
    -5.7103035915809066E-14, 19.008653481824108, -1.3371916796360176E-14,
    -5.5377835733259267E-14, 18.999161240805375, -1.3444965820165588E-14,
    -5.3653280967912066E-14, 18.989668999610288, -1.3518014843970995E-14,
    -5.1928726202564866E-14, 18.980176758228151, -1.3591063867776401E-14,
    -5.0204171437217659E-14, 218.97068097017313, -1.3664140230288322E-14,
    -4.8478971254667866E-14, 18.403341962908449, -1.3737189254093733E-14,
    -4.6754416489320666E-14, 17.836003098288465, -1.381023827789914E-14,
    -4.5029861723973465E-14, 17.268664519118236, -1.3883287301704547E-14,
    -4.3305306958626265E-14, 16.701322821727668, -1.3956363664216469E-14,
    -4.1580106776076465E-14, 16.133985242692432, -1.4029412688021879E-14,
    -3.9855552010729265E-14, 15.566648377562128, -1.4102461711827287E-14,
    -3.8130997245382065E-14, 14.999312369155188, -1.4175510735632692E-14,
    -3.6406442480034864E-14, 14.431977360289999, -1.42485597594381E-14,
    -3.4681887714687664E-14, 13.864639947285786, -1.4321636121950028E-14,
    -3.2956687532137864E-14, 13.29730736670753, -1.4394685145755433E-14,
    -3.1232132766790664E-14, 12.729975504805015, -1.4467739637302143E-14,
    -2.9507448918002949E-14, 12.162645213856816, -1.4540794128848854E-14,
    -2.7782765069215227E-14, 11.59531663669202, -1.4613848620395565E-14,
    -2.6058081220427506E-14, 11.027989561465748, -1.4686905845812927E-14,
    -2.433333282991953E-14, -1.8424331687810725E-14, 19.018149269924553,
    9.05806020179114E-15, -1.7472079647251126E-14, 19.008653481824108,
    8.3545852381625647E-15, -1.6520183855116724E-14, 18.999161240805375,
    7.6513734525974426E-15, -1.5568288062982323E-14, 18.989668999610288,
    6.9481616670323173E-15, -1.4616392270847924E-14, 18.980176758228151,
    6.2449498814671983E-15, -1.3664140230288322E-14, 218.97068097017313,
    5.5414749178386225E-15, -1.2712244438153922E-14, 18.403341962908449,
    4.8382631322735E-15, -1.1760348646019524E-14, 17.836003098288465,
    4.1350513467083814E-15, -1.0808452853885122E-14, 17.268664519118236,
    3.4318395611432624E-15, -9.856200813325523E-15, 16.701322821727668,
    2.7283645975146771E-15, -8.904305021191123E-15, 16.133985242692432,
    2.0251528119495582E-15, -7.9524092290567215E-15, 15.566648377562128,
    1.3219410263844424E-15, -7.0005134369223223E-15, 14.999312369155188,
    6.187292408193234E-16, -6.0486176447879215E-15, 14.431977360289999,
    -8.4482544745795568E-17, -5.0963656042283233E-15, 13.864639947285786,
    -7.8795750837437771E-16, -4.1444698120939217E-15, 13.29730736670753,
    -1.4911692939394998E-15, -3.1925027702744826E-15, 12.729975504805015,
    -2.1944337151173089E-15, -2.2405357284550422E-15, 12.162645213856816,
    -2.8976981362951243E-15, -1.2885686866356021E-15, 11.59531663669202,
    -3.6009625574729334E-15, -3.3656601997364208E-16, 11.027989561465748,
    -4.3042532964570938E-15, -5.9317663469897069E-14, -6.685694463645525E-17,
    19.018149269924553, -5.7149438332095271E-14, 1.0550612613503626E-15,
    19.008653481824108, -5.4982024352220069E-14, 2.1765597448508407E-15,
    18.999161240805375, -5.2814610372344866E-14, 3.2980582283513218E-15,
    18.989668999610288, -5.0647196392469664E-14, 4.4195567118518007E-15,
    18.980176758228151, -4.8478971254667866E-14, 5.5414749178386233E-15,
    218.97068097017313, -4.6311557274792664E-14, 6.6629734013391005E-15,
    18.403341962908449, -4.4144143294917461E-14, 7.78447188483958E-15,
    17.836003098288465, -4.1976729315042259E-14, 8.9059703683400645E-15,
    17.268664519118236, -3.9808504177240461E-14, 1.0027888574326884E-14,
    16.701322821727668, -3.7641090197365259E-14, 1.1149387057827362E-14,
    16.133985242692432, -3.5473676217490062E-14, 1.2270885541327842E-14,
    15.566648377562128, -3.3306262237614866E-14, 1.3392384024828323E-14,
    14.999312369155188, -3.1138848257739664E-14, 1.4513882508328804E-14,
    14.431977360289999, -2.8970623119937866E-14, 1.5635800714315622E-14,
    13.864639947285786, -2.680320914006266E-14, 1.67572991978161E-14,
    13.29730736670753, -2.4635632928602143E-14, 1.7878881625813852E-14,
    12.729975504805015, -2.2468056717141623E-14, 1.90004640538116E-14,
    12.162645213856816, -2.03004805056811E-14, 2.0122046481809348E-14,
    11.59531663669202, -1.8132823178427921E-14, 2.1243670882055729E-14,
    11.027989561465748, 18.449375509349135, -1.2371736988235457E-14,
    -5.4754470715050671E-14, 18.440166735896412, -1.2439853768180976E-14,
    -5.3065508910993672E-14, 18.430961402876555, -1.2507945064856786E-14,
    -5.1377178966945669E-14, 18.421756069691011, -1.2576036361532595E-14,
    -4.9688849022897666E-14, 18.412550736329113, -1.2644127658208402E-14,
    -4.8000519078849663E-14, 18.403341962908449, -1.2712244438153922E-14,
    -4.6311557274792664E-14, 218.39413662993405, -1.2780335734829733E-14,
    -4.4623227330744667E-14, 17.827084530502528, -1.284842703150554E-14,
    -4.293489738669667E-14, 17.260032573715733, -1.2916518328181348E-14,
    -4.1246567442648667E-14, 16.692977462506974, -1.2984635108126869E-14,
    -3.9557605638591662E-14, 16.125926220196998, -1.3052726404802678E-14,
    -3.7869275694543665E-14, 15.558875548994951, -1.3120817701478486E-14,
    -3.6180945750495662E-14, 14.99182559169784, -1.3188908998154294E-14,
    -3.4492615806447659E-14, 14.424776491124083, -1.32570002948301E-14,
    -3.2804285862399669E-14, 13.857724950220369, -1.3325117074775625E-14,
    -3.111532405834267E-14, 13.290677992296745, -1.3393208371451432E-14,
    -2.9426994114294667E-14, 12.723631631577335, -1.3461304764781182E-14,
    -2.7738537798244868E-14, 12.156586698983148, -1.3529401158110934E-14,
    -2.6050081482195067E-14, 11.589543337343272, -1.3597497551440683E-14,
    -2.4361625166145265E-14, 11.022501345499631, -1.3665596493097406E-14,
    -2.2673105664094569E-14, -1.8522173058137733E-14, 18.449375509349135,
    1.01337235490337E-14, -1.7564961435011732E-14, 18.440166735896412,
    9.4394176708128055E-15, -1.6608107915747735E-14, 18.430961402876555,
    8.7453715403952044E-15, -1.5651254396483731E-14, 18.421756069691011,
    8.051325409977597E-15, -1.4694400877219734E-14, 18.412550736329113,
    7.3572792795599959E-15, -1.373718925409373E-14, 18.403341962908449,
    6.6629734013391021E-15, -1.278033573482973E-14, 218.39413662993405,
    5.968927270921501E-15, -1.1823482215565731E-14, 17.827084530502528,
    5.2748811405039031E-15, -1.086662869630173E-14, 17.260032573715733,
    4.580835010086302E-15, -9.90941707317573E-15, 16.692977462506974,
    3.8865291318653987E-15, -8.95256355391173E-15, 16.125926220196998,
    3.1924830014477976E-15, -7.9957100346477285E-15, 15.558875548994951,
    2.4984368710302028E-15, -7.03885651538373E-15, 14.99182559169784,
    1.8043907406126018E-15, -6.08200299611973E-15, 14.424776491124083,
    1.1103446101950038E-15, -5.124791372993731E-15, 13.857724950220369,
    4.1603873197410369E-16, -4.16793785372973E-15, 13.290677992296745,
    -2.7800739844350055E-16, -3.2110127136933304E-15, 12.723631631577335,
    -9.7210547842175829E-16, -2.2540875736569303E-15, 12.156586698983148,
    -1.6662035584000192E-15, -1.29716243362053E-15, 11.589543337343272,
    -2.3603016383782769E-15, -3.4020148319793006E-16, 11.022501345499631,
    -3.0544256931368693E-15, -5.7411957483666675E-14, -8.15903866459495E-16,
    18.449375509349135, -5.5279970724357666E-14, 3.1518342493500329E-16,
    18.440166735896412, -5.3148781565781667E-14, 1.4458475635830008E-15,
    18.430961402876555, -5.1017592407205668E-14, 2.576511702231003E-15,
    18.421756069691011, -4.8886403248629669E-14, 3.7071758408790013E-15,
    18.412550736329113, -4.6754416489320666E-14, 4.8382631322735027E-15,
    18.403341962908449, -4.4623227330744667E-14, 5.968927270921501E-15,
    218.39413662993405, -4.2492038172168668E-14, 7.0995914095695016E-15,
    17.827084530502528, -4.0360849013592663E-14, 8.2302555482175054E-15,
    17.260032573715733, -3.8228862254283666E-14, 9.3613428396120038E-15,
    16.692977462506974, -3.6097673095707661E-14, 1.0492006978260003E-14,
    16.125926220196998, -3.3966483937131662E-14, 1.1622671116908003E-14,
    15.558875548994951, -3.1835294778555663E-14, 1.2753335255556003E-14,
    14.99182559169784, -2.9704105619979658E-14, 1.3883999394204003E-14,
    14.424776491124083, -2.7572118860670664E-14, 1.5015086685598505E-14,
    13.857724950220369, -2.5440929702094662E-14, 1.6145750824246505E-14,
    13.290677992296745, -2.3309581023372065E-14, 1.7276499593443804E-14,
    12.723631631577335, -2.1178232344649462E-14, 1.8407248362641104E-14,
    12.156586698983148, -1.9046883665926861E-14, 1.9537997131838403E-14,
    11.589543337343272, -1.6915455227130962E-14, 2.0668788216310355E-14,
    11.022501345499631, 17.880602605443634, -1.1444633542622657E-14,
    -5.2405905514292263E-14, 17.871680703820211, -1.1507790740001776E-14,
    -5.0753182088728064E-14, 17.86276213600221, -1.1570924309547987E-14,
    -4.9101076965979271E-14, 17.853843568023862, -1.1634057879094195E-14,
    -4.7448971843230466E-14, 17.844924999879826, -1.1697191448640402E-14,
    -4.579686672048166E-14, 17.836003098288465, -1.1760348646019522E-14,
    -4.4144143294917467E-14, 17.827084530502528, -1.1823482215565734E-14,
    -4.2492038172168668E-14, 217.81816596256962, -1.1886615785111941E-14,
    -4.0839933049419863E-14, 17.251400628179624, -1.1949749354658148E-14,
    -3.918782792667107E-14, 16.684632103163366, -1.2012906552037271E-14,
    -3.7535104501106865E-14, 16.117867197589334, -1.2076040121583479E-14,
    -3.5882999378358065E-14, 15.551102720320864, -1.2139173691129686E-14,
    -3.4230894255609266E-14, 14.984338814160328, -1.2202307260675893E-14,
    -3.2578789132860467E-14, 14.417575621904723, -1.22654408302221E-14,
    -3.0926684010111668E-14, 13.850809953101507, -1.2328598027601225E-14,
    -2.9273960584547469E-14, 13.284048617832516, -1.2391731597147432E-14,
    -2.7621855461798666E-14, 12.717287758296209, -1.2454869892260223E-14,
    -2.5969626678486788E-14, 12.150528184082756, -1.2518008187373015E-14,
    -2.4317397895174906E-14, 11.583770037994526, -1.2581146482485804E-14,
    -2.2665169111863024E-14, 11.017013129533511, -1.2644287140381887E-14,
    -2.1012878498269608E-14, -1.862001442846474E-14, 17.880602605443634,
    1.120938689627626E-14, -1.7657843222772339E-14, 17.871680703820211,
    1.0524250103463045E-14, -1.6696031976378742E-14, 17.86276213600221,
    9.8393696281929631E-15, -1.573422072998514E-14, 17.853843568023862,
    9.1544891529228783E-15, -1.477240948359154E-14, 17.844924999879826,
    8.4696086776527967E-15, -1.3810238277899138E-14, 17.836003098288465,
    7.7844718848395817E-15, -1.2848427031505539E-14, 17.827084530502528,
    7.0995914095695016E-15, -1.1886615785111939E-14, 217.81816596256962,
    6.4147109342994216E-15, -1.0924804538718338E-14, 17.251400628179624,
    5.7298304590293415E-15, -9.962633333025938E-15, 16.684632103163366,
    5.0446936662161171E-15, -9.00082208663234E-15, 16.117867197589334,
    4.35981319094604E-15, -8.0390108402387372E-15, 15.551102720320864,
    3.6749327156759633E-15, -7.0771995938451384E-15, 14.984338814160328,
    2.99005224040588E-15, -6.115388347451538E-15, 14.417575621904723,
    2.3051717651358032E-15, -5.1532171417591388E-15, 13.850809953101507,
    1.6200349723225819E-15, -4.1914058953655376E-15, 13.284048617832516,
    9.3515449705249873E-16, -3.2295226571121786E-15, 12.717287758296209,
    2.5022275827379233E-16, -2.267639418858818E-15, 12.150528184082756,
    -4.3470898050491723E-16, -1.3057561806054581E-15, 11.583770037994526,
    -1.1196407192836205E-15, -3.4383694642221804E-16, 11.017013129533511,
    -1.8045980898166448E-15, -5.5506251497436269E-14, -1.5649507882825348E-15,
    17.880602605443634, -5.3410503116620072E-14, -4.2469441148035764E-16,
    17.871680703820211, -5.1315538779343265E-14, 7.1513538231516088E-16,
    17.86276213600221, -4.922057444206647E-14, 1.8549651761106826E-15,
    17.853843568023862, -4.7125610104789669E-14, 2.9947949699062019E-15,
    17.844924999879826, -4.5029861723973465E-14, 4.1350513467083838E-15,
    17.836003098288465, -4.293489738669667E-14, 5.2748811405039015E-15,
    17.827084530502528, -4.0839933049419863E-14, 6.4147109342994216E-15,
    217.81816596256962, -3.8744968712143068E-14, 7.5545407280949448E-15,
    17.251400628179624, -3.6649220331326864E-14, 8.6947971048971236E-15,
    16.684632103163366, -3.4554255994050063E-14, 9.8346268986926421E-15,
    16.117867197589334, -3.2459291656773262E-14, 1.0974456692488162E-14,
    15.551102720320864, -3.0364327319496467E-14, 1.2114286486283682E-14,
    14.984338814160328, -2.8269362982219662E-14, 1.3254116280079204E-14,
    14.417575621904723, -2.6173614601403465E-14, 1.4394372656881384E-14,
    13.850809953101507, -2.4078650264126661E-14, 1.5534202450676903E-14,
    13.284048617832516, -2.198352911814198E-14, 1.6674117561073757E-14,
    12.717287758296209, -1.98884079721573E-14, 1.781403267147061E-14,
    12.150528184082756, -1.7793286826172623E-14, 1.8953947781867458E-14,
    11.583770037994526, -1.5698087275834003E-14, 2.0093905550564977E-14,
    11.017013129533511, 17.311830701026452, -1.0517530097009855E-14,
    -5.0057340313533867E-14, 17.303195528413926, -1.0575727711822575E-14,
    -4.8440855266462463E-14, 17.294563582979354, -1.0633903554239184E-14,
    -4.6824974965012861E-14, 17.285931637411185, -1.0692079396655794E-14,
    -4.5209094663563259E-14, 17.277299691682664, -1.07502552390724E-14,
    -4.3593214362113657E-14, 17.268664519118236, -1.080845285388512E-14,
    -4.1976729315042265E-14, 17.260032573715733, -1.0866628696301732E-14,
    -4.0360849013592663E-14, 17.251400628179624, -1.092480453871834E-14,
    -3.8744968712143068E-14, 217.24276868249655, -1.0982980381134946E-14,
    -3.7129088410693466E-14, 16.676286743686152, -1.1041177995947668E-14,
    -3.5512603363622061E-14, 16.109808174858756, -1.1099353838364277E-14,
    -3.3896723062172465E-14, 15.543329891534551, -1.1157529680780885E-14,
    -3.2280842760722857E-14, 14.976852036515911, -1.1215705523197492E-14,
    -3.0664962459273262E-14, 14.4103747526052, -1.12738813656141E-14,
    -2.9049082157823666E-14, 13.843894955929201, -1.1332078980426823E-14,
    -2.7432597110752262E-14, 13.277419243314844, -1.1390254822843431E-14,
    -2.5816716809302663E-14, 12.710943884961637, -1.1448435019739261E-14,
    -2.4200715558728707E-14, 12.144469669128918, -1.1506615216635093E-14,
    -2.2584714308154746E-14, 11.577996738619056, -1.1564795413530922E-14,
    -2.0968713057580784E-14, 11.011524913567394, -1.1622977787666365E-14,
    -1.9352651332444647E-14, -1.8717855798791748E-14, 17.311830701026452,
    1.2285050243518822E-14, -1.7750725010532949E-14, 17.303195528413926,
    1.1609082536113287E-14, -1.678395603700975E-14, 17.294563582979354,
    1.0933367715990725E-14, -1.5817187063486548E-14, 17.285931637411185,
    1.025765289586816E-14, -1.4850418089963349E-14, 17.277299691682664,
    9.581938075745599E-15, -1.3883287301704547E-14, 17.268664519118236,
    8.9059703683400645E-15, -1.2916518328181346E-14, 17.260032573715733,
    8.2302555482175023E-15, -1.1949749354658148E-14, 17.251400628179624,
    7.5545407280949433E-15, -1.0982980381134946E-14, 217.24276868249655,
    6.8788259079723827E-15, -1.0015849592876146E-14, 16.676286743686152,
    6.20285820056684E-15, -9.0490806193529477E-15, 16.109808174858756,
    5.5271433804442812E-15, -8.0823116458297459E-15, 15.543329891534551,
    4.8514285603217254E-15, -7.1155426723065456E-15, 14.976852036515911,
    4.1757137401991632E-15, -6.1487736987833462E-15, 14.4103747526052,
    3.4999989200766042E-15, -5.1816429105245473E-15, 13.843894955929201,
    2.8240312126710649E-15, -4.2148739370013455E-15, 13.277419243314844,
    2.1483163925485012E-15, -3.2480326005310264E-15, 12.710943884961637,
    1.4725509949693477E-15, -2.2811912640607061E-15, 12.144469669128918,
    7.9678559739018946E-16, -1.3143499275903861E-15, 11.577996738619056,
    1.2102019981103597E-16, -3.4747240964650607E-16, 11.011524913567394,
    -5.5477048649642028E-16, -5.3600545511205863E-14, -2.3139977101055762E-15,
    17.311830701026452, -5.1541035508882467E-14, -1.1645722478957186E-15,
    17.303195528413926, -4.9482295992904869E-14, -1.5576798952680586E-17,
    17.294563582979354, -4.7423556476927259E-14, 1.1334186499903621E-15,
    17.285931637411185, -4.5364816960949662E-14, 2.2824140989334009E-15,
    17.277299691682664, -4.3305306958626265E-14, 3.4318395611432616E-15,
    17.268664519118236, -4.1246567442648667E-14, 4.5808350100863004E-15,
    17.260032573715733, -3.9187827926671063E-14, 5.72983045902934E-15,
    17.251400628179624, -3.712908841069346E-14, 6.8788259079723843E-15,
    217.24276868249655, -3.5069578408370063E-14, 8.0282513701822434E-15,
    16.676286743686152, -3.3010838892392465E-14, 9.1772468191252814E-15,
    16.109808174858756, -3.0952099376414861E-14, 1.0326242268068321E-14,
    15.543329891534551, -2.8893359860437264E-14, 1.1475237717011362E-14,
    14.976852036515911, -2.683462034445966E-14, 1.2624233165954403E-14,
    14.4103747526052, -2.4775110342136263E-14, 1.3773658628164262E-14,
    13.843894955929201, -2.2716370826158659E-14, 1.4922654077107304E-14,
    13.277419243314844, -2.0657477212911902E-14, 1.6071735528703709E-14,
    12.710943884961637, -1.8598583599665138E-14, 1.722081698030011E-14,
    12.144469669128918, -1.6539689986418381E-14, 1.8369898431896516E-14,
    11.577996738619056, -1.4480719324537037E-14, 1.95190228848196E-14,
    11.011524913567394, 16.743055859333261, -9.5900796815446558E-15,
    -4.7707896158733266E-14, 16.734707379580612, -9.6433158576597761E-15,
    -4.6127663052422066E-14, 16.726362020311825, -9.6965321175099844E-15,
    -4.4548021129465264E-14, 16.718016660936151, -9.7497483773601943E-15,
    -4.2968379206508463E-14, 16.709671301426873, -9.8029646372104011E-15,
    -4.1388737283551661E-14, 16.701322821727668, -9.8562008133255214E-15,
    -3.9808504177240461E-14, 16.692977462506978, -9.9094170731757329E-15,
    -3.8228862254283666E-14, 16.684632103163366, -9.96263333302594E-15,
    -3.6649220331326864E-14, 16.676286743686152, -1.0015849592876146E-14,
    -3.5069578408370063E-14, 216.66793826403239, -1.006908576899127E-14,
    -3.3489345302058863E-14, 16.101746138565733, -1.0122302028841478E-14,
    -3.1909703379102061E-14, 15.535554155797229, -1.0175518288691685E-14,
    -3.0330061456145266E-14, 14.969362458531922, -1.0228734548541893E-14,
    -2.8750419533188458E-14, 14.403171189572175, -1.02819508083921E-14,
    -2.7170777610231663E-14, 13.836977371690779, -1.0335186984507223E-14,
    -2.5590544503920463E-14, 13.270787388358517, -1.0388403244357432E-14,
    -2.4010902580963665E-14, 12.704597637797137, -1.0441623487460622E-14,
    -2.2431142421335988E-14, 12.138408886953879, -1.0494843730563813E-14,
    -2.0851382261708305E-14, 11.572221278631108, -1.0548063973667002E-14,
    -1.9271622102080628E-14, 11.006034643628233, -1.0601286208396685E-14,
    -1.7691802824117506E-14, -1.8815733786398069E-14, 16.743055859333261,
    1.3361116159379361E-14, -1.784364155944167E-14, 16.734707379580612,
    1.2694320968925446E-14, -1.6871913003352073E-14, 16.726362020311825,
    1.2027775234210564E-14, -1.590018444726247E-14, 16.718016660936151,
    1.1361229499495679E-14, -1.492845589117287E-14, 16.709671301426873,
    1.0694683764780799E-14, -1.3956363664216469E-14, 16.701322821727668,
    1.0027888574326884E-14, -1.2984635108126868E-14, 16.692977462506978,
    9.3613428396120022E-15, -1.2012906552037269E-14, 16.684632103163366,
    8.6947971048971236E-15, -1.1041177995947669E-14, 16.676286743686152,
    8.0282513701822419E-15, -1.0069085768991268E-14, 216.66793826403239,
    7.3614561797283189E-15, -9.09735721290167E-15, 16.101746138565733,
    6.69491044501344E-15, -8.1256286568120669E-15, 15.535554155797229,
    6.0283647102985633E-15, -7.1539001007224685E-15, 14.969362458531922,
    5.3618189755836831E-15, -6.1821715446328678E-15, 14.403171189572175,
    4.6952732408688045E-15, -5.2100793176764691E-15, 13.836977371690779,
    4.0284780504148847E-15, -4.2383507615868677E-15, 13.270787388358517,
    3.3619323157E-15, -3.2665494713239085E-15, 12.704597637797137,
    2.6953366898373155E-15, -2.2947481810609481E-15, 12.138408886953879,
    2.0287410639746249E-15, -1.3229468907979881E-15, 11.572221278631108,
    1.362145438111939E-15, -3.5110923344834806E-16, 11.006034643628233,
    6.9552486667534633E-16, -5.1694126311657266E-14, -3.0633249638604369E-15,
    16.743055859333261, -4.9670868250094068E-14, -1.9047269846989583E-15,
    16.734707379580612, -4.7648367112609264E-14, -7.4656245034823988E-16,
    16.726362020311825, -4.5625865975124467E-14, 4.1160208400248326E-16,
    16.718016660936151, -4.360336483763967E-14, 1.5697666183532009E-15,
    16.709671301426873, -4.1580106776076465E-14, 2.7283645975146827E-15,
    16.701322821727668, -3.9557605638591668E-14, 3.8865291318654E-15,
    16.692977462506978, -3.7535104501106865E-14, 5.04469366621612E-15,
    16.684632103163366, -3.5512603363622061E-14, 6.202858200566845E-15,
    16.676286743686152, -3.3489345302058863E-14, 7.3614561797283236E-15,
    216.66793826403239, -3.1466844164574059E-14, 8.519620714079042E-15,
    16.101746138565733, -2.9444343027089262E-14, 9.677785248429762E-15,
    15.535554155797229, -2.7421841889604465E-14, 1.0835949782780482E-14,
    14.969362458531922, -2.5399340752119661E-14, 1.1994114317131204E-14,
    14.403171189572175, -2.3376082690556463E-14, 1.3152712296292682E-14,
    13.836977371690779, -2.1353581553071659E-14, 1.4310876830643404E-14,
    13.270787388358517, -1.9330929030771181E-14, 1.5469128053956276E-14,
    12.704597637797137, -1.7308276508470702E-14, 1.6627379277269149E-14,
    12.138408886953879, -1.528562398617022E-14, 1.7785630500582019E-14,
    11.572221278631108, -1.32628957714619E-14, 1.8943925068375967E-14,
    11.006034643628233, 16.174286383363331, -8.6629762359318554E-15,
    -4.5359330957974864E-14, 16.166224346984805, -8.7112528294805761E-15,
    -4.3815336230156464E-14, 16.158165324435924, -8.7595113622011854E-15,
    -4.2271919128498866E-14, 16.150106301780163, -8.8077698949217947E-15,
    -4.0728502026841262E-14, 16.142047279017515, -8.8560284276424E-15,
    -3.9185084925183665E-14, 16.133985242692432, -8.9043050211911214E-15,
    -3.7641090197365265E-14, 16.125926220196998, -8.9525635539117323E-15,
    -3.6097673095707661E-14, 16.117867197589334, -9.00082208663234E-15,
    -3.4554255994050063E-14, 16.109808174858756, -9.0490806193529477E-15,
    -3.3010838892392465E-14, 16.101746138565733, -9.09735721290167E-15,
    -3.1466844164574066E-14, 216.09368711611575, -9.1456157456222776E-15,
    -2.9923427062916461E-14, 15.527781327262112, -9.1938742783428853E-15,
    -2.8380009961258864E-14, 14.96187568110664, -9.242132811063493E-15,
    -2.6836592859601263E-14, 14.395970320454358, -9.2903913437841E-15,
    -2.5293175757943665E-14, 13.830062374678805, -9.3386679373328245E-15,
    -2.3749181030125262E-14, 13.264158014001179, -9.38692647005343E-15,
    -2.2205763928467664E-14, 12.698253764596179, -9.4351886149396628E-15,
    -2.0662231301577907E-14, 12.132350372080209, -9.4834507598258934E-15,
    -1.9118698674688144E-14, 11.56644797928236, -9.531712904712124E-15,
    -1.7575166047798387E-14, 11.000546427662115, -9.5799768556811661E-15,
    -1.6031575658292548E-14, -1.8913575156725077E-14, 16.174286383363331,
    1.4436779506621921E-14, -1.793652334720228E-14, 16.166224346984805,
    1.3779153401575685E-14, -1.6959837063983081E-14, 16.158165324435924,
    1.3121773322008324E-14, -1.5983150780763879E-14, 16.150106301780163,
    1.2464393242440959E-14, -1.500646449754468E-14, 16.142047279017515,
    1.1807013162873598E-14, -1.4029412688021877E-14, 16.133985242692432,
    1.1149387057827364E-14, -1.3052726404802677E-14, 16.125926220196998,
    1.0492006978260003E-14, -1.2076040121583478E-14, 16.117867197589334,
    9.8346268986926421E-15, -1.1099353838364277E-14, 16.109808174858756,
    9.1772468191252814E-15, -1.0122302028841477E-14, 16.101746138565733,
    8.5196207140790389E-15, -9.1456157456222776E-15, 216.09368711611575,
    7.86224063451168E-15, -8.1689294624030755E-15, 15.527781327262112,
    7.2048605549443238E-15, -7.1922431791838766E-15, 14.96187568110664,
    6.5474804753769631E-15, -6.215556895964676E-15, 14.395970320454358,
    5.8901003958096039E-15, -5.2385050864418769E-15, 13.830062374678805,
    5.232474290763363E-15, -4.2618188032226756E-15, 13.264158014001179,
    4.5750942111959991E-15, -3.2850594147427567E-15, 12.698253764596179,
    3.9176649265328661E-15, -2.3083000262628362E-15, 12.132350372080209,
    3.2602356418697284E-15, -1.3315406377829161E-15, 11.56644797928236,
    2.6028063572065955E-15, -3.5474469667263604E-16, 11.000546427662115,
    1.9453524699955693E-15, -4.9788420325426867E-14, -3.8123718856834751E-15,
    16.174286383363331, -4.7801400642356469E-14, -2.6446048211143176E-15,
    16.166224346984805, -4.5815124326170869E-14, -1.4772746316160798E-15,
    16.158165324435924, -4.3828848009985269E-14, -3.0994444211783717E-16,
    16.150106301780163, -4.1842571693799669E-14, 8.5738574738040148E-16,
    16.142047279017515, -3.9855552010729265E-14, 2.0251528119495629E-15,
    16.133985242692432, -3.7869275694543665E-14, 3.1924830014478004E-15,
    16.125926220196998, -3.5882999378358065E-14, 4.359813190946041E-15,
    16.117867197589334, -3.3896723062172465E-14, 5.5271433804442852E-15,
    16.109808174858756, -3.1909703379102061E-14, 6.6949104450134434E-15,
    16.101746138565733, -2.9923427062916461E-14, 7.8622406345116829E-15,
    216.09368711611575, -2.7937150746730862E-14, 9.0295708240099239E-15,
    15.527781327262112, -2.5950874430545262E-14, 1.0196901013508162E-14,
    14.96187568110664, -2.3964598114359662E-14, 1.1364231203006403E-14,
    14.395970320454358, -2.1977578431289264E-14, 1.2531998267575563E-14,
    13.830062374678805, -1.9991302115103661E-14, 1.3699328457073804E-14,
    13.264158014001179, -1.8004877125541102E-14, 1.4866746021586228E-14,
    12.698253764596179, -1.601845213597854E-14, 1.6034163586098652E-14,
    12.132350372080209, -1.403202714641598E-14, 1.7201581150611076E-14,
    11.56644797928236, -1.204552782016494E-14, 1.836904240263059E-14,
    11.000546427662115, 15.605518335310194, -7.7358727903190551E-15,
    -4.3010765757216462E-14, 15.597742599514088, -7.7791898013013761E-15,
    -4.1503009407890863E-14, 15.589969770893461, -7.8224906068923863E-15,
    -3.9995817127532462E-14, 15.582196942165931, -7.8657914124833934E-15,
    -3.8488624847174062E-14, 15.574424113331517, -7.9090922180744E-15,
    -3.6981432566815662E-14, 15.56664837756213, -7.9524092290567215E-15,
    -3.5473676217490069E-14, 15.558875548994951, -7.9957100346477317E-15,
    -3.3966483937131662E-14, 15.551102720320866, -8.0390108402387388E-15,
    -3.2459291656773262E-14, 15.543329891534551, -8.0823116458297459E-15,
    -3.0952099376414861E-14, 15.535554155797231, -8.1256286568120684E-15,
    -2.9444343027089262E-14, 15.527781327262112, -8.1689294624030771E-15,
    -2.7937150746730862E-14, 215.52000849863347, -8.2122302679940842E-15,
    -2.6429958466372461E-14, 14.95438890360119, -8.2555310735850928E-15,
    -2.4922766186014061E-14, 14.388769451267065, -8.2988318791761E-15,
    -2.3415573905655664E-14, 13.823147377608043, -8.3421488901584241E-15,
    -2.1907817556330061E-14, 13.257528639590394, -8.3854496957494311E-15,
    -2.0400625275971664E-14, 12.691909891368498, -8.4287537424187016E-15,
    -1.8893320181819827E-14, 12.126291857206541, -8.4720577890879737E-15,
    -1.7386015087667984E-14, 11.560674679933612, -8.5153618357572426E-15,
    -1.5878709993516147E-14, 10.995058211695996, -8.5586675029656456E-15,
    -1.4371348492467584E-14, -1.9011416527052085E-14, 15.605518335310194,
    1.551244285386448E-14, -1.8029405134962886E-14, 15.597742599514088,
    1.4863985834225924E-14, -1.7047761124614088E-14, 15.589969770893461,
    1.4215771409806083E-14, -1.6066117114265287E-14, 15.582196942165931,
    1.356755698538624E-14, -1.5084473103916486E-14, 15.574424113331517,
    1.2919342560966399E-14, -1.4102461711827284E-14, 15.56664837756213,
    1.2270885541327843E-14, -1.3120817701478484E-14, 15.558875548994951,
    1.1622671116908003E-14, -1.2139173691129685E-14, 15.551102720320866,
    1.0974456692488164E-14, -1.1157529680780885E-14, 15.543329891534551,
    1.0326242268068323E-14, -1.0175518288691685E-14, 15.535554155797231,
    9.67778524842976E-15, -9.1938742783428853E-15, 15.527781327262112,
    9.02957082400992E-15, -8.2122302679940826E-15, 215.52000849863347,
    8.3813563995900842E-15, -7.2305862576452846E-15, 14.95438890360119,
    7.733141975170243E-15, -6.2489422472964835E-15, 14.388769451267065,
    7.0849275507504049E-15, -5.2669308552072847E-15, 13.823147377608043,
    6.4364705311118444E-15, -4.2852868448584835E-15, 13.257528639590394,
    5.788256106692E-15, -3.3035693581616045E-15, 12.691909891368498,
    5.13999316322842E-15, -2.3218518714647243E-15, 12.126291857206541,
    4.4917302197648335E-15, -1.3401343847678441E-15, 11.560674679933612,
    3.8434672763012519E-15, -3.5838015989692407E-16, 10.995058211695996,
    3.1951800733157938E-15, -4.7882714339196467E-14, -4.5614188075065165E-15,
    15.605518335310194, -4.5931933034618863E-14, -3.3844826575296778E-15,
    15.597742599514088, -4.3981881539732467E-14, -2.2079868128839205E-15,
    15.589969770893461, -4.2031830044846065E-14, -1.0314909682381576E-15,
    15.582196942165931, -4.0081778549959668E-14, 1.450048764076005E-16,
    15.574424113331517, -3.8130997245382065E-14, 1.3219410263844424E-15,
    15.56664837756213, -3.6180945750495662E-14, 2.4984368710302E-15,
    15.558875548994951, -3.4230894255609266E-14, 3.67493271567596E-15,
    15.551102720320866, -3.2280842760722864E-14, 4.8514285603217246E-15,
    15.543329891534551, -3.033006145614526E-14, 6.0283647102985633E-15,
    15.535554155797231, -2.8380009961258864E-14, 7.2048605549443222E-15,
    15.527781327262112, -2.6429958466372461E-14, 8.3813563995900827E-15,
    215.52000849863347, -2.4479906971486062E-14, 9.5578522442358416E-15,
    14.95438890360119, -2.2529855476599663E-14, 1.0734348088881604E-14,
    14.388769451267065, -2.0579074172022065E-14, 1.1911284238858443E-14,
    13.823147377608043, -1.862902267713566E-14, 1.3087780083504202E-14,
    13.257528639590394, -1.6678825220311021E-14, 1.4264363989216181E-14,
    12.691909891368498, -1.4728627763486382E-14, 1.5440947894928156E-14,
    12.126291857206541, -1.277843030666174E-14, 1.6617531800640131E-14,
    11.560674679933612, -1.082815986886798E-14, 1.7794159736885216E-14,
    10.995058211695996, 15.03675185799225, -6.8087693447062547E-15,
    -4.0662200556458061E-14, 15.029262279960165, -6.8471267731221768E-15,
    -3.9190682585625268E-14, 15.021775502476093, -6.8854698515835857E-15,
    -3.7719715126566065E-14, 15.014288724885136, -6.9238129300449929E-15,
    -3.6248747667506862E-14, 15.006801947187274, -6.962156008506401E-15,
    -3.4777780208447659E-14, 14.999312369155188, -7.0005134369223223E-15,
    -3.3306262237614866E-14, 14.99182559169784, -7.0388565153837319E-15,
    -3.1835294778555663E-14, 14.984338814160328, -7.0771995938451392E-15,
    -3.036432731949646E-14, 14.976852036515911, -7.1155426723065472E-15,
    -2.8893359860437264E-14, 14.969362458531922, -7.1539001007224685E-15,
    -2.7421841889604465E-14, 14.961875681106639, -7.1922431791838781E-15,
    -2.5950874430545265E-14, 14.954388903601188, -7.2305862576452846E-15,
    -2.4479906971486062E-14, 214.94690212600221, -7.2689293361066927E-15,
    -2.3008939512426862E-14, 14.381568581999604, -7.3072724145681E-15,
    -2.1537972053367666E-14, 13.816232380467804, -7.3456298429840236E-15,
    -2.0066454082534863E-14, 13.25089926512082, -7.3839729214454316E-15,
    -1.8595486623475664E-14, 12.685566018087371, -7.422318869897742E-15,
    -1.7124409062061747E-14, 12.12023334230615, -7.460664818350054E-15,
    -1.5653331500647823E-14, 11.554901380584866, -7.4990107668023628E-15,
    -1.4182253939233905E-14, 10.989569995729877, -7.5373581502501251E-15,
    -1.2711121326642626E-14, -1.9109257897379093E-14, 15.03675185799225,
    1.6588106201107039E-14, -1.8122286922723496E-14, 15.029262279960165,
    1.5948818266876165E-14, -1.7135685185245096E-14, 15.021775502476093,
    1.5309769497603845E-14, -1.6149083447766692E-14, 15.014288724885136,
    1.4670720728331518E-14, -1.5162481710288295E-14, 15.006801947187274,
    1.4031671959059198E-14, -1.4175510735632692E-14, 14.999312369155188,
    1.3392384024828323E-14, -1.3188908998154292E-14, 14.99182559169784,
    1.2753335255556003E-14, -1.2202307260675893E-14, 14.984338814160328,
    1.2114286486283684E-14, -1.1215705523197493E-14, 14.976852036515911,
    1.1475237717011362E-14, -1.0228734548541892E-14, 14.969362458531922,
    1.0835949782780479E-14, -9.242132811063493E-15, 14.961875681106639,
    1.019690101350816E-14, -8.2555310735850913E-15, 14.954388903601188,
    9.5578522442358431E-15, -7.2689293361066927E-15, 214.94690212600221,
    8.9188034749635229E-15, -6.2823275986282917E-15, 14.381568581999604,
    8.2797547056912043E-15, -5.2953566239726932E-15, 13.816232380467804,
    7.6404667714603242E-15, -4.3087548864942914E-15, 13.25089926512082,
    7.0014180021880008E-15, -3.3220793015804527E-15, 12.685566018087371,
    6.3623213999239705E-15, -2.3354037166666124E-15, 12.12023334230615,
    5.7232247976599371E-15, -1.3487281317527721E-15, 11.554901380584866,
    5.0841281953959084E-15, -3.6201562312121205E-16, 10.989569995729877,
    4.4450076766360183E-15, -4.5977008352966067E-14, -5.3104657293295563E-15,
    15.03675185799225, -4.406246542688127E-14, -4.1243604939450371E-15,
    15.029262279960165, -4.2148638753294065E-14, -2.9386989941517596E-15,
    15.021775502476093, -4.0234812079706866E-14, -1.7530374943584773E-15,
    15.014288724885136, -3.8320985406119668E-14, -5.6737599456519889E-16,
    15.006801947187274, -3.6406442480034864E-14, 6.1872924081932261E-16,
    14.999312369155188, -3.4492615806447665E-14, 1.8043907406126006E-15,
    14.99182559169784, -3.2578789132860467E-14, 2.9900522404058809E-15,
    14.984338814160328, -3.0664962459273262E-14, 4.1757137401991648E-15,
    14.976852036515911, -2.8750419533188465E-14, 5.3618189755836831E-15,
    14.969362458531922, -2.6836592859601263E-14, 6.5474804753769615E-15,
    14.961875681106639, -2.4922766186014061E-14, 7.733141975170243E-15,
    14.954388903601188, -2.3008939512426862E-14, 8.9188034749635229E-15,
    214.94690212600221, -2.1095112838839664E-14, 1.0104464974756804E-14,
    14.381568581999604, -1.9180569912754863E-14, 1.1290570210141323E-14,
    13.816232380467804, -1.7266743239167661E-14, 1.2476231709934603E-14,
    13.25089926512082, -1.5352773315080942E-14, 1.3661981956846133E-14,
    12.685566018087371, -1.3438803390994222E-14, 1.4847732203757659E-14,
    12.12023334230615, -1.15248334669075E-14, 1.6033482450669186E-14,
    11.554901380584866, -9.61079191757102E-15, 1.7219277071139838E-14,
    10.989569995729877, 14.467987094227905, -5.8816658990934544E-15,
    -3.8313635355699665E-14, 14.460783531141436, -5.915063744942976E-15,
    -3.687835576335966E-14, 14.453582661975519, -5.948449096274785E-15,
    -3.5443613125599667E-14, 14.446381792729436, -5.9818344476065925E-15,
    -3.4008870487839662E-14, 14.439180923376467, -6.0152197989384007E-15,
    -3.2574127850079656E-14, 14.431977360289999, -6.0486176447879223E-15,
    -3.1138848257739664E-14, 14.424776491124085, -6.0820029961197305E-15,
    -2.9704105619979665E-14, 14.417575621904724, -6.1153883474515388E-15,
    -2.8269362982219662E-14, 14.4103747526052, -6.1487736987833462E-15,
    -2.6834620344459663E-14, 14.403171189572175, -6.1821715446328678E-15,
    -2.5399340752119661E-14, 14.395970320454358, -6.2155568959646768E-15,
    -2.3964598114359662E-14, 14.388769451267063, -6.2489422472964843E-15,
    -2.2529855476599663E-14, 14.381568581999606, -6.2823275986282925E-15,
    -2.1095112838839661E-14, 214.37436771263862, -6.3157129499601E-15,
    -1.9660370201079661E-14, 13.809317383247398, -6.3491107958096231E-15,
    -1.8225090608739663E-14, 13.244269890581769, -6.3824961471414306E-15,
    -1.6790347970979663E-14, 12.679222144747456, -6.4158839973767808E-15,
    -1.5355497942303663E-14, 12.114174827352313, -6.4492718476121327E-15,
    -1.3920647913627663E-14, 11.549128081209396, -6.4826596978474822E-15,
    -1.2485797884951662E-14, 10.984081779763759, -6.5160487975346054E-15,
    -1.1050894160817663E-14, -1.92070992677061E-14, 14.467987094227905,
    1.7663769548349604E-14, -1.8215168710484103E-14, 14.460783531141436,
    1.7033650699526406E-14, -1.7223609245876103E-14, 14.453582661975519,
    1.6403767585401607E-14, -1.62320497812681E-14, 14.446381792729436,
    1.57738844712768E-14, -1.5240490316660102E-14, 14.439180923376467,
    1.5144001357152E-14, -1.42485597594381E-14, 14.431977360289999,
    1.4513882508328804E-14, -1.32570002948301E-14, 14.424776491124085,
    1.3883999394204003E-14, -1.22654408302221E-14, 14.417575621904724,
    1.3254116280079204E-14, -1.1273881365614101E-14, 14.4103747526052,
    1.2624233165954403E-14, -1.02819508083921E-14, 14.403171189572175,
    1.19941143171312E-14, -9.2903913437841E-15, 14.395970320454358,
    1.1364231203006401E-14, -8.2988318791761E-15, 14.388769451267063,
    1.0734348088881605E-14, -7.3072724145681E-15, 14.381568581999606,
    1.0104464974756804E-14, -6.3157129499601E-15, 214.37436771263862,
    9.4745818606320053E-15, -5.323782392738101E-15, 13.809317383247398,
    8.844463011808804E-15, -4.3322229281300994E-15, 13.244269890581769,
    8.2145798976840017E-15, -3.3405892449993005E-15, 12.679222144747456,
    7.5846496366195243E-15, -2.3489555618685E-15, 12.114174827352313,
    6.9547193755550422E-15, -1.3573218787377E-15, 11.549128081209396,
    6.3247891144905648E-15, -3.6565108634550003E-16, 10.984081779763759,
    5.6948352799562428E-15, -4.4071302366735668E-14, -6.0595126511525968E-15,
    14.467987094227905, -4.2192997819143664E-14, -4.864238330360398E-15,
    14.460783531141436, -4.0315395966855669E-14, -3.6694111754196E-15,
    14.453582661975519, -3.8437794114567662E-14, -2.4745840204787977E-15,
    14.446381792729436, -3.6560192262279667E-14, -1.2797568655379991E-15,
    14.439180923376467, -3.4681887714687664E-14, -8.4482544745797146E-17,
    14.431977360289999, -3.2804285862399662E-14, 1.1103446101949995E-15,
    14.424776491124085, -3.0926684010111661E-14, 2.3051717651358E-15,
    14.417575621904724, -2.904908215782366E-14, 3.4999989200766042E-15,
    14.4103747526052, -2.7170777610231663E-14, 4.695273240868803E-15,
    14.403171189572175, -2.5293175757943662E-14, 5.8901003958096024E-15,
    14.395970320454358, -2.3415573905655661E-14, 7.0849275507504025E-15,
    14.388769451267063, -2.1537972053367663E-14, 8.2797547056912027E-15,
    14.381568581999606, -1.9660370201079661E-14, 9.4745818606320037E-15,
    214.37436771263862, -1.7782065653487661E-14, 1.0669856181424202E-14,
    13.809317383247398, -1.590446380119966E-14, 1.1864683336365003E-14,
    13.244269890581769, -1.4026721409850861E-14, 1.3059599924476083E-14,
    12.679222144747456, -1.214897901850206E-14, 1.4254516512587163E-14,
    12.114174827352313, -1.027123662715326E-14, 1.5449433100698244E-14,
    11.549128081209396, -8.39342396627406E-15, 1.6644394405394461E-14,
    10.984081779763759, 13.89922010725283, -4.9542154836282548E-15,
    -3.5964191200899063E-14, 13.892302522960962, -4.9826518907801766E-15,
    -3.4565163549319263E-14, 13.885387525895545, -5.0110776595455852E-15,
    -3.3166659290052064E-14, 13.878472528776683, -5.0395034283109929E-15,
    -3.1768155030784859E-14, 13.871557531577654, -5.0679291970764007E-15,
    -3.036965077151766E-14, 13.864639947285784, -5.0963656042283225E-15,
    -2.8970623119937866E-14, 13.857724950220369, -5.124791372993731E-15,
    -2.7572118860670661E-14, 13.850809953101507, -5.1532171417591396E-15,
    -2.6173614601403462E-14, 13.843894955929201, -5.1816429105245465E-15,
    -2.4775110342136263E-14, 13.836977371690779, -5.2100793176764684E-15,
    -2.3376082690556463E-14, 13.830062374678805, -5.2385050864418777E-15,
    -2.1977578431289264E-14, 13.823147377608043, -5.2669308552072847E-15,
    -2.0579074172022062E-14, 13.816232380467804, -5.2953566239726932E-15,
    -1.918056991275486E-14, 13.809317383247398, -5.323782392738101E-15,
    -1.7782065653487661E-14, 213.8023997989475, -5.3522187998900236E-15,
    -1.6383038001907861E-14, 13.237638035577342, -5.3806445686554313E-15,
    -1.4984533742640662E-14, 12.672875897561578, -5.4090724650981411E-15,
    -1.3585924804910944E-14, 12.108114045171927, -5.4375003615408532E-15,
    -1.2187315867181223E-14, 11.543352621221448, -5.465928257983563E-15,
    -1.0788706929451503E-14, 10.978591509824598, -5.4943572182649258E-15,
    -9.390045652490525E-15, -1.9304977255312425E-14, 13.89922010725283,
    1.8739835464210143E-14, -1.8308085259392824E-14, 13.892302522960962,
    1.8118889132338568E-14, -1.7311566212218426E-14, 13.885387525895545,
    1.7498175103621444E-14, -1.6315047165044023E-14, 13.878472528776683,
    1.6877461074904321E-14, -1.5318528117869626E-14, 13.871557531577654,
    1.6256747046187197E-14, -1.4321636121950021E-14, 13.864639947285784,
    1.5635800714315622E-14, -1.3325117074775622E-14, 13.857724950220369,
    1.5015086685598505E-14, -1.2328598027601223E-14, 13.850809953101507,
    1.4394372656881384E-14, -1.1332078980426823E-14, 13.843894955929201,
    1.3773658628164262E-14, -1.0335186984507222E-14, 13.836977371690779,
    1.3152712296292679E-14, -9.3386679373328229E-15, 13.830062374678805,
    1.253199826757556E-14, -8.3421488901584209E-15, 13.823147377608043,
    1.1911284238858445E-14, -7.345629842984022E-15, 13.816232380467804,
    1.1290570210141324E-14, -6.3491107958096215E-15, 13.809317383247398,
    1.0669856181424204E-14, -5.3522187998900228E-15, 213.8023997989475,
    1.0048909849552624E-14, -4.3556997527156215E-15, 13.237638035577342,
    9.4281958208355E-15, -3.3591061157921826E-15, 12.672875897561578,
    8.80743533148749E-15, -2.3625124788687421E-15, 12.108114045171927,
    8.1866748421394776E-15, -1.3659188419453021E-15, 11.543352621221448,
    7.5659143527914679E-15, -3.6928791014734207E-16, 10.978591509824598,
    6.9451306331280078E-15, -4.216488316718707E-14, -6.808839904907456E-15,
    13.89922010725283, -4.0322830560355265E-14, -5.604393067163637E-15,
    13.892302522960962, -3.8481467086560065E-14, -4.40039682681516E-15,
    13.885387525895545, -3.6640103612764864E-14, -3.1964005864666773E-15,
    13.878472528776683, -3.4798740138969669E-14, -1.9924043461181991E-15,
    13.871557531577654, -3.2956687532137864E-14, -7.8795750837437692E-16,
    13.864639947285784, -3.1115324058342663E-14, 4.1603873197410014E-16,
    13.857724950220369, -2.9273960584547462E-14, 1.6200349723225811E-15,
    13.850809953101507, -2.7432597110752262E-14, 2.8240312126710649E-15,
    13.843894955929201, -2.5590544503920463E-14, 4.0284780504148831E-15,
    13.836977371690779, -2.3749181030125262E-14, 5.2324742907633614E-15,
    13.830062374678805, -2.1907817556330061E-14, 6.4364705311118428E-15,
    13.823147377608043, -2.0066454082534863E-14, 7.6404667714603226E-15,
    13.816232380467804, -1.8225090608739663E-14, 8.844463011808804E-15,
    13.809317383247398, -1.6383038001907864E-14, 1.0048909849552622E-14,
    213.8023997989475, -1.4541674528112663E-14, 1.1252906089901104E-14,
    13.237638035577342, -1.2700173227710143E-14, 1.2456992449728652E-14,
    12.672875897561578, -1.0858671927307622E-14, 1.36610788095562E-14,
    12.108114045171927, -9.0171706269051E-15, 1.4865165169383747E-14,
    11.543352621221448, -7.1756004131989217E-15, 1.6069296588950828E-14,
    10.978591509824598, 13.330459200066313, -4.0271120380154537E-15,
    -3.3615626000140662E-14, 13.323827345083266, -4.0505888626009766E-15,
    -3.2252836727053662E-14, 13.317197970672481, -4.0740569042367845E-15,
    -3.0890557289085667E-14, 13.310568596208253, -4.0975249458725924E-15,
    -2.9528277851117659E-14, 13.303939221690579, -4.1209929875084E-15,
    -2.8165998413149657E-14, 13.29730736670753, -4.1444698120939217E-15,
    -2.6803209140062664E-14, 13.290677992296747, -4.1679378537297304E-15,
    -2.5440929702094662E-14, 13.284048617832518, -4.1914058953655384E-15,
    -2.4078650264126661E-14, 13.277419243314844, -4.2148739370013463E-15,
    -2.2716370826158659E-14, 13.270787388358517, -4.2383507615868677E-15,
    -2.1353581553071659E-14, 13.264158014001179, -4.2618188032226772E-15,
    -1.9991302115103661E-14, 13.257528639590394, -4.2852868448584843E-15,
    -1.8629022677135663E-14, 13.250899265120822, -4.3087548864942922E-15,
    -1.7266743239167661E-14, 13.244269890581769, -4.3322229281301E-15,
    -1.590446380119966E-14, 13.237638035577344, -4.3556997527156223E-15,
    -1.454167452811266E-14, 213.23100866115854, -4.37916779435143E-15,
    -1.3179395090144662E-14, 12.66653202431252, -4.4026375925771807E-15,
    -1.1817013685152862E-14, 12.102055530276882, -4.4261073908029327E-15,
    -1.0454632280161061E-14, 11.537579321867357, -4.4495771890286824E-15,
    -9.0922508751692612E-15, 10.973103293858481, -4.4730478655494053E-15,
    -7.7298184866655624E-15, -1.9402818625639433E-14, 13.330459200066313,
    1.9815498811452702E-14, -1.8400967047153434E-14, 13.323827345083266,
    1.9203721564988809E-14, -1.7399490272849434E-14, 13.317197970672481,
    1.8592173191419206E-14, -1.6398013498545431E-14, 13.310568596208253,
    1.79806248178496E-14, -1.5396536724241432E-14, 13.303939221690579,
    1.736907644428E-14, -1.4394685145755429E-14, 13.29730736670753,
    1.6757299197816105E-14, -1.339320837145143E-14, 13.290677992296747,
    1.6145750824246505E-14, -1.239173159714743E-14, 13.284048617832518,
    1.5534202450676903E-14, -1.1390254822843431E-14, 13.277419243314844,
    1.4922654077107304E-14, -1.038840324435743E-14, 13.270787388358517,
    1.43108768306434E-14, -9.38692647005343E-15, 13.264158014001179,
    1.3699328457073801E-14, -8.38544969574943E-15, 13.257528639590394,
    1.3087780083504205E-14, -7.38397292144543E-15, 13.250899265120822,
    1.2476231709934604E-14, -6.38249614714143E-15, 13.244269890581769,
    1.1864683336365003E-14, -5.3806445686554313E-15, 13.237638035577344,
    1.1252906089901105E-14, -4.3791677943514295E-15, 213.23100866115854,
    1.0641357716331501E-14, -3.3776160592110308E-15, 12.66653202431252,
    1.0029763568183044E-14, -2.37606432407063E-15, 12.102055530276882,
    9.4181694200345812E-15, -1.3745125889302301E-15, 11.537579321867357,
    8.8065752718861243E-15, -3.7292337337163005E-16, 10.973103293858481,
    8.1949582364482339E-15, -4.0259177180956664E-14, -7.5578868267304966E-15,
    13.330459200066313, -3.8453362952617666E-14, -6.3442709035789979E-15,
    13.323827345083266, -3.6648224300121669E-14, -5.131109008083E-15,
    13.317197970672481, -3.4843085647625665E-14, -3.9179471125869978E-15,
    13.310568596208253, -3.3037946995129668E-14, -2.7047852170909993E-15,
    13.303939221690579, -3.1232132766790664E-14, -1.4911692939394975E-15,
    13.29730736670753, -2.9426994114294667E-14, -2.7800739844350016E-16,
    13.290677992296747, -2.7621855461798663E-14, 9.3515449705250031E-16,
    13.284048617832518, -2.581671680930266E-14, 2.1483163925485035E-15,
    13.277419243314844, -2.4010902580963661E-14, 3.3619323157000026E-15,
    13.270787388358517, -2.2205763928467664E-14, 4.5750942111960023E-15,
    13.264158014001179, -2.0400625275971661E-14, 5.7882561066920023E-15,
    13.257528639590394, -1.8595486623475664E-14, 7.0014180021880016E-15,
    13.250899265120822, -1.679034797097966E-14, 8.2145798976840033E-15,
    13.244269890581769, -1.4984533742640662E-14, 9.4281958208355019E-15,
    13.237638035577344, -1.3179395090144662E-14, 1.0641357716331503E-14,
    213.23100866115854, -1.1374121322480061E-14, 1.1854610417358603E-14,
    12.66653202431252, -9.56884755481546E-15, 1.3067863118385705E-14,
    12.102055530276882, -7.76357378715086E-15, 1.42811158194128E-14,
    11.537579321867357, -5.95823246190196E-15, 1.549441392320545E-14,
    10.973103293858481, 12.761699618977392, -3.0999391984321746E-15,
    -3.1266885008573821E-14, 12.755353371812895, -3.1184560692250563E-15,
    -2.99403368264331E-14, 12.749009498585215, -3.1369660126439049E-15,
    -2.8614284921203021E-14, 12.742665625304086, -3.1554759560627527E-15,
    -2.728823301597294E-14, 12.736321751969516, -3.1739858994816005E-15,
    -2.5962181110742858E-14, 12.729975504805017, -3.1925027702744822E-15,
    -2.4635632928602143E-14, 12.723631631577337, -3.2110127136933308E-15,
    -2.3309581023372062E-14, 12.717287758296209, -3.2295226571121782E-15,
    -2.198352911814198E-14, 12.710943884961637, -3.2480326005310264E-15,
    -2.0657477212911902E-14, 12.704597637797139, -3.2665494713239085E-15,
    -1.9330929030771181E-14, 12.698253764596179, -3.2850594147427567E-15,
    -1.8004877125541102E-14, 12.691909891368498, -3.3035693581616045E-15,
    -1.6678825220311021E-14, 12.685566018087371, -3.3220793015804523E-15,
    -1.5352773315080939E-14, 12.679222144747456, -3.3405892449993E-15,
    -1.4026721409850861E-14, 12.67287589756158, -3.3591061157921826E-15,
    -1.2700173227710143E-14, 12.66653202431252, -3.3776160592110304E-15,
    -1.1374121322480063E-14, 212.66018767626809, -3.3961273881046851E-15,
    -1.0047970161867854E-14, 12.095996561921559, -3.41463871699834E-15,
    -8.7218190012556461E-15, 11.531805590385424, -3.4331500458919944E-15,
    -7.3956678406434385E-15, 10.967614667097752, -3.4516620675230535E-15,
    -6.0694670523401663E-15, -1.9500667319422303E-14, 12.761699618977392,
    2.0891242672418858E-14, -1.8493855787143664E-14, 12.755353371812895,
    2.0288635197671428E-14, -1.7487420914622705E-14, 12.749009498585215,
    1.9686253165301381E-14, -1.6480986042101742E-14, 12.742665625304086,
    1.9083871132931326E-14, -1.5474551169580782E-14, 12.736321751969516,
    1.8481489100561278E-14, -1.446773963730214E-14, 12.729975504805017,
    1.7878881625813852E-14, -1.3461304764781181E-14, 12.723631631577337,
    1.7276499593443804E-14, -1.2454869892260223E-14, 12.717287758296209,
    1.6674117561073757E-14, -1.1448435019739261E-14, 12.710943884961637,
    1.6071735528703709E-14, -1.0441623487460621E-14, 12.704597637797139,
    1.5469128053956273E-14, -9.4351886149396628E-15, 12.698253764596179,
    1.4866746021586225E-14, -8.4287537424187E-15, 12.691909891368498,
    1.4264363989216181E-14, -7.42231886989774E-15, 12.685566018087371,
    1.3661981956846133E-14, -6.4158839973767808E-15, 12.679222144747456,
    1.3059599924476085E-14, -5.4090724650981419E-15, 12.67287589756158,
    1.2456992449728652E-14, -4.4026375925771807E-15, 12.66653202431252,
    1.1854610417358601E-14, -3.3961273881046855E-15, 212.66018767626809,
    1.1252183296513078E-14, -2.389617183632189E-15, 12.095996561921559,
    1.0649756175667553E-14, -1.383106979159693E-15, 11.531805590385424,
    1.0047329054822031E-14, -3.7655910871142886E-16, 10.967614667097752,
    9.4448793897387662E-15, -3.8353328552062623E-14, -8.3069898149399E-15,
    12.761699618977392, -3.6583755414669909E-14, -7.0842041200719333E-15,
    12.755353371812895, -3.4814844294911829E-14, -5.8618758833763843E-15,
    12.749009498585215, -3.3045933175153742E-14, -4.6395476466808289E-15,
    12.742665625304086, -3.1277022055395668E-14, -3.4172194099852791E-15,
    12.736321751969516, -2.9507448918002942E-14, -2.1944337151173093E-15,
    12.729975504805017, -2.7738537798244865E-14, -9.7210547842175987E-16,
    12.723631631577337, -2.5969626678486785E-14, 2.5022275827379272E-16,
    12.717287758296209, -2.42007155587287E-14, 1.4725509949693479E-15,
    12.710943884961637, -2.2431142421335982E-14, 2.6953366898373147E-15,
    12.704597637797139, -2.06622313015779E-14, 3.9176649265328661E-15,
    12.698253764596179, -1.8893320181819821E-14, 5.1399931632284183E-15,
    12.691909891368498, -1.7124409062061743E-14, 6.36232139992397E-15,
    12.685566018087371, -1.535549794230366E-14, 7.5846496366195227E-15,
    12.679222144747456, -1.3585924804910944E-14, 8.80743533148749E-15,
    12.67287589756158, -1.181701368515286E-14, 1.0029763568183043E-14,
    12.66653202431252, -1.0047970161867854E-14, 1.1252183296513078E-14,
    212.66018767626809, -8.2789266385828441E-15, 1.2474603024843114E-14,
    12.095996561921559, -6.5098831152978362E-15, 1.3697022753173148E-14,
    11.531805590385424, -4.7407733902493646E-15, 1.4919488227320423E-14,
    10.967614667097752, 12.192942322934794, -2.172766358848894E-15,
    -2.8918144017006981E-14, 12.186881540759755, -2.186323275849136E-15,
    -2.7627836925812541E-14, 12.180823025886085, -2.1998751210510245E-15,
    -2.6338012553320383E-14, 12.174764510985694, -2.2134269662529122E-15,
    -2.5048188180828218E-14, 12.168705996031857, -2.2269788114548003E-15,
    -2.375836380833606E-14, 12.162645213856816, -2.2405357284550418E-15,
    -2.2468056717141623E-14, 12.156586698983148, -2.2540875736569303E-15,
    -2.1178232344649462E-14, 12.150528184082757, -2.267639418858818E-15,
    -1.98884079721573E-14, 12.14446966912892, -2.2811912640607061E-15,
    -1.8598583599665141E-14, 12.138408886953879, -2.2947481810609481E-15,
    -1.7308276508470702E-14, 12.132350372080211, -2.3083000262628362E-15,
    -1.601845213597854E-14, 12.126291857206541, -2.3218518714647239E-15,
    -1.4728627763486379E-14, 12.12023334230615, -2.335403716666612E-15,
    -1.343880339099422E-14, 12.114174827352313, -2.3489555618685E-15,
    -1.214897901850206E-14, 12.108114045171929, -2.3625124788687421E-15,
    -1.0858671927307621E-14, 12.102055530276882, -2.37606432407063E-15,
    -9.56884755481546E-15, 12.095996561921559, -2.3896171836321886E-15,
    -8.2789266385828457E-15, 212.08993759356358, -2.4031700431937479E-15,
    -6.9890057223502292E-15, 11.526031858903492, -2.4167229027553064E-15,
    -5.6990848061176134E-15, 10.962126040337026, -2.4302762694967008E-15,
    -4.4091156180147694E-15, -1.9598516013205174E-14, 12.192942322934794,
    2.1966986533385016E-14, -1.8586744527133895E-14, 12.186881540759755,
    2.1373548830354055E-14, -1.7575351556395975E-14, 12.180823025886085,
    2.0780333139183558E-14, -1.6563958585658052E-14, 12.174764510985694,
    2.0187117448013055E-14, -1.5552565614920133E-14, 12.168705996031857,
    1.9593901756842562E-14, -1.4540794128848851E-14, 12.162645213856816,
    1.90004640538116E-14, -1.3529401158110931E-14, 12.156586698983148,
    1.8407248362641104E-14, -1.2518008187373013E-14, 12.150528184082757,
    1.7814032671470607E-14, -1.1506615216635092E-14, 12.14446966912892,
    1.722081698030011E-14, -1.0494843730563812E-14, 12.138408886953879,
    1.6627379277269146E-14, -9.4834507598258934E-15, 12.132350372080211,
    1.6034163586098649E-14, -8.47205778908797E-15, 12.126291857206541,
    1.5440947894928156E-14, -7.4606648183500524E-15, 12.12023334230615,
    1.4847732203757659E-14, -6.4492718476121311E-15, 12.114174827352313,
    1.4254516512587166E-14, -5.4375003615408525E-15, 12.108114045171929,
    1.3661078809556201E-14, -4.4261073908029312E-15, 12.102055530276882,
    1.3067863118385701E-14, -3.41463871699834E-15, 12.095996561921559,
    1.2474603024843114E-14, -2.4031700431937479E-15, 212.08993759356358,
    1.1881342931300524E-14, -1.3917013693891557E-15, 11.526031858903492,
    1.1288082837757936E-14, -3.8019484405122766E-16, 10.962126040337026,
    1.0694800543029299E-14, -3.6447479923168587E-14, -9.0560928031493042E-15,
    12.192942322934794, -3.4714147876722146E-14, -7.82413733656487E-15,
    12.186881540759755, -3.2981464289701989E-14, -6.5926427586697684E-15,
    12.180823025886085, -3.1248780702681825E-14, -5.3611481807746617E-15,
    12.174764510985694, -2.9516097115661668E-14, -4.12965360287956E-15,
    12.168705996031857, -2.7782765069215224E-14, -2.8976981362951215E-15,
    12.162645213856816, -2.6050081482195063E-14, -1.6662035584000202E-15,
    12.156586698983148, -2.4317397895174903E-14, -4.3470898050491565E-16,
    12.150528184082757, -2.2584714308154742E-14, 7.9678559739019163E-16,
    12.14446966912892, -2.08513822617083E-14, 2.0287410639746265E-15,
    12.138408886953879, -1.9118698674688144E-14, 3.2602356418697296E-15,
    12.132350372080211, -1.7386015087667981E-14, 4.4917302197648343E-15,
    12.126291857206541, -1.5653331500647823E-14, 5.7232247976599371E-15,
    12.12023334230615, -1.3920647913627661E-14, 6.9547193755550438E-15,
    12.114174827352313, -1.2187315867181222E-14, 8.1866748421394776E-15,
    12.108114045171929, -1.0454632280161061E-14, 9.4181694200345827E-15,
    12.102055530276882, -8.7218190012556445E-15, 1.0649756175667554E-14,
    12.095996561921559, -6.9890057223502284E-15, 1.1881342931300524E-14,
    212.08993759356358, -5.2561924434448123E-15, 1.3112929686933494E-14,
    11.526031858903492, -3.523314318596768E-15, 1.4344562531435398E-14,
    10.962126040337026, 11.624187454740882, -1.245593519265614E-15,
    -2.656940302544014E-14, 11.618411994752934, -1.2541904824732159E-15,
    -2.5315337025191979E-14, 11.612638695404186, -1.2627842294581441E-15,
    -2.4061740185437744E-14, 11.60686539605544, -1.2713779764430723E-15,
    -2.28081433456835E-14, 11.60109209667997, -1.279971723428E-15,
    -2.1554546505929257E-14, 11.59531663669202, -1.2885686866356019E-15,
    -2.03004805056811E-14, 11.589543337343274, -1.29716243362053E-15,
    -1.9046883665926861E-14, 11.583770037994526, -1.3057561806054581E-15,
    -1.7793286826172619E-14, 11.577996738619058, -1.3143499275903861E-15,
    -1.6539689986418381E-14, 11.572221278631108, -1.3229468907979879E-15,
    -1.528562398617022E-14, 11.56644797928236, -1.3315406377829161E-15,
    -1.4032027146415982E-14, 11.560674679933612, -1.3401343847678441E-15,
    -1.277843030666174E-14, 11.554901380584868, -1.3487281317527721E-15,
    -1.15248334669075E-14, 11.549128081209398, -1.3573218787377E-15,
    -1.027123662715326E-14, 11.543352621221448, -1.3659188419453021E-15,
    -9.0171706269051E-15, 11.537579321867357, -1.3745125889302301E-15,
    -7.7635737871508618E-15, 11.531805590385424, -1.3831069791596928E-15,
    -6.509883115297837E-15, 11.526031858903492, -1.3917013693891557E-15,
    -5.2561924434448123E-15, 211.52025812741888, -1.4002957596186186E-15,
    -4.0025017715917883E-15, 10.9566374135763, -1.4088904714703488E-15,
    -2.7487641836893724E-15, -1.9696364706988044E-14, 11.624187454740882,
    2.3042730394351172E-14, -1.8679633267124126E-14, 11.618411994752934,
    2.2458462463036678E-14, -1.7663282198169246E-14, 11.612638695404186,
    2.1874413113065736E-14, -1.6646931129214363E-14, 11.60686539605544,
    2.1290363763094784E-14, -1.5630580060259486E-14, 11.60109209667997,
    2.0706314413123839E-14, -1.4613848620395562E-14, 11.59531663669202,
    2.0122046481809348E-14, -1.3597497551440682E-14, 11.589543337343274,
    1.9537997131838403E-14, -1.2581146482485804E-14, 11.583770037994526,
    1.8953947781867461E-14, -1.1564795413530924E-14, 11.577996738619058,
    1.8369898431896516E-14, -1.0548063973667002E-14, 11.572221278631108,
    1.7785630500582019E-14, -9.531712904712124E-15, 11.56644797928236,
    1.7201581150611073E-14, -8.515361835757241E-15, 11.560674679933612,
    1.6617531800640131E-14, -7.4990107668023628E-15, 11.554901380584868,
    1.6033482450669189E-14, -6.4826596978474822E-15, 11.549128081209398,
    1.5449433100698244E-14, -5.4659282579835638E-15, 11.543352621221448,
    1.486516516938375E-14, -4.4495771890286824E-15, 11.537579321867357,
    1.42811158194128E-14, -3.4331500458919952E-15, 11.531805590385424,
    1.3697022753173149E-14, -2.4167229027553068E-15, 11.526031858903492,
    1.3112929686933494E-14, -1.4002957596186186E-15, 211.52025812741888,
    1.2528836620693842E-14, -3.8383057939102646E-16, 10.9566374135763,
    1.1944721696319831E-14, -3.4541631294274545E-14, -9.8051957913587088E-15,
    11.624187454740882, -3.2844540338774383E-14, -8.5640705530578058E-15,
    11.618411994752934, -3.1148084284492149E-14, -7.3234096339631525E-15,
    11.612638695404186, -2.94516282302099E-14, -6.0827487148684937E-15,
    11.60686539605544, -2.7755172175927668E-14, -4.8420877957738396E-15,
    11.60109209667997, -2.6058081220427502E-14, -3.6009625574729334E-15,
    11.59531663669202, -2.4361625166145265E-14, -2.36030163837828E-15,
    11.589543337343274, -2.2665169111863024E-14, -1.1196407192836236E-15,
    11.583770037994526, -2.0968713057580781E-14, 1.2102019981103557E-16,
    11.577996738619058, -1.9271622102080622E-14, 1.3621454381119386E-15,
    11.572221278631108, -1.7575166047798381E-14, 2.6028063572065939E-15,
    11.56644797928236, -1.587870999351614E-14, 3.84346727630125E-15,
    11.560674679933612, -1.4182253939233903E-14, 5.0841281953959052E-15,
    11.554901380584868, -1.2485797884951661E-14, 6.3247891144905632E-15,
    11.549128081209398, -1.0788706929451503E-14, 7.5659143527914663E-15,
    11.543352621221448, -9.0922508751692612E-15, 8.8065752718861227E-15,
    11.537579321867357, -7.3956678406434369E-15, 1.0047329054822029E-14,
    11.531805590385424, -5.6990848061176126E-15, 1.1288082837757934E-14,
    11.526031858903492, -4.0025017715917883E-15, 1.2528836620693839E-14,
    211.52025812741888, -2.3058552469441722E-15, 1.3769636835550369E-14,
    10.9566374135763, 11.055434749242426, -3.18385982697094E-16,
    -2.422057413846908E-14, 11.049944479303267, -3.2202280649893605E-16,
    -2.3002750585393941E-14, 11.044456263337148, -3.2565826972322408E-16,
    -2.1785382634096982E-14, 11.038968047371029, -3.2929373294751206E-16,
    -2.0568014682800019E-14, 11.033479831404911, -3.3292919617180004E-16,
    -1.9350646731503057E-14, 11.02798956146575, -3.36566019973642E-16,
    -1.8132823178427921E-14, 11.022501345499633, -3.4020148319793011E-16,
    -1.6915455227130962E-14, 11.017013129533513, -3.4383694642221804E-16,
    -1.5698087275834E-14, 11.011524913567394, -3.47472409646506E-16,
    -1.448071932453704E-14, 11.006034643628235, -3.5110923344834796E-16,
    -1.32628957714619E-14, 11.000546427662115, -3.5474469667263604E-16,
    -1.2045527820164942E-14, 10.995058211695996, -3.5838015989692407E-16,
    -1.082815986886798E-14, 10.989569995729878, -3.62015623121212E-16,
    -9.6107919175710189E-15, 10.984081779763759, -3.656510863455E-16,
    -8.3934239662740584E-15, 10.9785915098246, -3.6928791014734207E-16,
    -7.17560041319892E-15, 10.973103293858481, -3.7292337337163E-16,
    -5.9582324619019611E-15, 10.967614667097754, -3.7655910871142881E-16,
    -4.7407733902493646E-15, 10.962126040337028, -3.8019484405122771E-16,
    -3.523314318596768E-15, 10.9566374135763, -3.8383057939102636E-16,
    -2.3058552469441722E-15, 210.95114858141827, -3.8746645078858057E-16,
    -1.0883506151137582E-15, -1.9794217062498844E-14, 11.055434749242426,
    2.4118514512179124E-14, -1.8772525483229166E-14, 11.049944479303267,
    2.3543416695735494E-14, -1.7751216130513647E-14, 11.044456263337148,
    2.296853402999012E-14, -1.6729906777798125E-14, 11.038968047371029,
    2.2393651364244736E-14, -1.5708597425082606E-14, 11.033479831404911,
    2.1818768698499359E-14, -1.4686905845812924E-14, 11.02798956146575,
    2.1243670882055729E-14, -1.3665596493097405E-14, 11.022501345499633,
    2.0668788216310355E-14, -1.2644287140381886E-14, 11.017013129533513,
    2.0093905550564977E-14, -1.1622977787666365E-14, 11.011524913567394,
    1.9519022884819603E-14, -1.0601286208396685E-14, 11.006034643628235,
    1.8943925068375967E-14, -9.5799768556811661E-15, 11.000546427662115,
    1.836904240263059E-14, -8.558667502965644E-15, 10.995058211695996,
    1.7794159736885216E-14, -7.5373581502501251E-15, 10.989569995729878,
    1.7219277071139841E-14, -6.5160487975346046E-15, 10.984081779763759,
    1.6644394405394464E-14, -5.4943572182649258E-15, 10.9785915098246,
    1.6069296588950831E-14, -4.4730478655494045E-15, 10.973103293858481,
    1.549441392320545E-14, -3.4516620675230531E-15, 10.967614667097754,
    1.4919488227320426E-14, -2.4302762694967008E-15, 10.962126040337028,
    1.4344562531435398E-14, -1.4088904714703488E-15, 10.9566374135763,
    1.3769636835550372E-14, -3.8746645078858067E-16, 210.95114858141827,
    1.3194689624595517E-14, -3.2635711344048686E-14, -1.0554326812761294E-14,
    11.055434749242426, -3.0974862835721548E-14, -9.30403145958953E-15,
    11.049944479303267, -2.9314635669896587E-14, -8.0542038562693079E-15,
    11.044456263337148, -2.7654408504071626E-14, -6.8043762529490818E-15,
    11.038968047371029, -2.5994181338246668E-14, -5.55454864962886E-15,
    11.033479831404911, -2.4333332829919524E-14, -4.3042532964570914E-15,
    11.02798956146575, -2.2673105664094563E-14, -3.05442569313687E-15,
    11.022501345499633, -2.1012878498269605E-14, -1.8045980898166456E-15,
    11.017013129533513, -1.935265133244464E-14, -5.547704864964184E-16,
    11.011524913567394, -1.7691802824117502E-14, 6.9552486667534652E-16,
    11.006034643628235, -1.6031575658292541E-14, 1.9453524699955697E-15,
    11.000546427662115, -1.437134849246758E-14, 3.1951800733157942E-15,
    10.995058211695996, -1.2711121326642622E-14, 4.4450076766360175E-15,
    10.989569995729878, -1.1050894160817661E-14, 5.6948352799562436E-15,
    10.984081779763759, -9.3900456524905234E-15, 6.9451306331280078E-15,
    10.9785915098246, -7.7298184866655608E-15, 8.1949582364482323E-15,
    10.973103293858481, -6.0694670523401647E-15, 9.4448793897387646E-15,
    10.967614667097754, -4.4091156180147686E-15, 1.0694800543029299E-14,
    10.962126040337028, -2.7487641836893721E-15, 1.1944721696319829E-14,
    10.9566374135763, -1.088350615113758E-15, 1.3194689624595515E-14,
    210.95114858141827} ;

   double b_a[3600] = { -221.86203408940816, 1.7935051601617053E-14,
    6.8847619827685469E-14, -21.293254106153611, 1.8032929589223377E-14,
    6.6941200628136871E-14, -20.724478346088514, 1.8130770959550388E-14,
    6.5035494641906465E-14, -20.155702871419727, 1.8228612329877396E-14,
    6.3129788655676059E-14, -19.586927824949626, 1.83264537002044E-14,
    6.1224082669445666E-14, -19.018149269924553, 1.8424331687810722E-14,
    5.9317663469897069E-14, -18.449375509349135, 1.8522173058137733E-14,
    5.7411957483666675E-14, -17.880602605443634, 1.8620014428464737E-14,
    5.5506251497436269E-14, -17.311830701026452, 1.8717855798791748E-14,
    5.3600545511205863E-14, -16.743055859333261, 1.8815733786398069E-14,
    5.169412631165726E-14, -16.174286383363331, 1.891357515672508E-14,
    4.9788420325426867E-14, -15.605518335310194, 1.9011416527052085E-14,
    4.7882714339196467E-14, -15.03675185799225, 1.9109257897379093E-14,
    4.5977008352966061E-14, -14.467987094227905, 1.92070992677061E-14,
    4.4071302366735668E-14, -13.89922010725283, 1.9304977255312425E-14,
    4.216488316718707E-14, -13.330459200066313, 1.9402818625639433E-14,
    4.0259177180956664E-14, -12.761699618977392, 1.9500667319422303E-14,
    3.8353328552062629E-14, -12.192942322934794, 1.9598516013205174E-14,
    3.6447479923168587E-14, -11.624187454740882, 1.9696364706988044E-14,
    3.4541631294274545E-14, -11.055434749242425, 1.9794217062498848E-14,
    3.2635711344048686E-14, 1.7935051601617056E-14, -221.86203408940816,
    -3.6789383283423793E-15, 1.7007601186151857E-14, -21.293254106153611,
    -2.9296110745875248E-15, 1.6080497740539058E-14, -20.724478346088514,
    -2.1805641527644851E-15, 1.5153394294926256E-14, -20.155702871419727,
    -1.431517230941439E-15, 1.4226290849313458E-14, -19.586927824949626,
    -6.82470309118396E-16, 1.3298840433848254E-14, -19.018149269924553,
    6.685694463645525E-17, 1.2371736988235454E-14, -18.449375509349135,
    8.1590386645949818E-16, 1.1444633542622655E-14, -17.880602605443634,
    1.564950788282538E-15, 1.0517530097009855E-14, -17.311830701026452,
    2.3139977101055777E-15, 9.5900796815446542E-15, -16.743055859333261,
    3.0633249638604417E-15, 8.6629762359318554E-15, -16.174286383363331,
    3.8123718856834814E-15, 7.7358727903190535E-15, -15.605518335310194,
    4.5614188075065149E-15, 6.8087693447062547E-15, -15.03675185799225,
    5.3104657293295578E-15, 5.8816658990934536E-15, -14.467987094227905,
    6.0595126511525976E-15, 4.9542154836282548E-15, -13.89922010725283,
    6.8088399049074552E-15, 4.0271120380154537E-15, -13.330459200066313,
    7.5578868267305013E-15, 3.0999391984321746E-15, -12.761699618977392,
    8.3069898149399028E-15, 2.172766358848894E-15, -12.192942322934794,
    9.0560928031493074E-15, 1.2455935192656142E-15, -11.624187454740882,
    9.8051957913587088E-15, 3.1838598269709405E-16, -11.055434749242425,
    1.0554326812761297E-14, 6.8847619827685469E-14, -3.6789383283423824E-15,
    -221.86203408940816, 6.6498175672884873E-14, -4.7550042442029233E-15,
    -21.293254106153611, 6.4149610472126465E-14, -5.8306675914454805E-15,
    -20.724478346088514, 6.1801045271368057E-14, -6.9063309386880424E-15,
    -20.155702871419727, 5.9452480070609662E-14, -7.9819942859306012E-15,
    -19.586927824949626, 5.7103035915809066E-14, -9.0580602017911437E-15,
    -19.018149269924553, 5.4754470715050665E-14, -1.0133723549033701E-14,
    -18.449375509349135, 5.2405905514292263E-14, -1.120938689627626E-14,
    -17.880602605443634, 5.0057340313533867E-14, -1.2285050243518825E-14,
    -17.311830701026452, 4.7707896158733259E-14, -1.3361116159379364E-14,
    -16.743055859333261, 4.5359330957974864E-14, -1.4436779506621921E-14,
    -16.174286383363331, 4.3010765757216462E-14, -1.5512442853864483E-14,
    -15.605518335310194, 4.0662200556458061E-14, -1.6588106201107042E-14,
    -15.03675185799225, 3.8313635355699665E-14, -1.7663769548349604E-14,
    -14.467987094227905, 3.5964191200899063E-14, -1.8739835464210143E-14,
    -13.89922010725283, 3.3615626000140662E-14, -1.9815498811452702E-14,
    -13.330459200066313, 3.1266885008573821E-14, -2.0891242672418861E-14,
    -12.761699618977392, 2.8918144017006981E-14, -2.1966986533385016E-14,
    -12.192942322934794, 2.656940302544014E-14, -2.3042730394351172E-14,
    -11.624187454740882, 2.422057413846908E-14, -2.4118514512179124E-14,
    -11.055434749242425, -21.293254106153611, 1.7007601186151854E-14,
    6.6498175672884873E-14, -221.28260872460294, 1.7100517735060578E-14,
    6.4628008414096468E-14, -20.714120550445791, 1.7193399522821188E-14,
    6.2758540806358862E-14, -20.145632518879907, 1.7286281310581795E-14,
    6.0889073198621269E-14, -19.577144772710334, 1.73791630983424E-14,
    5.9019605590883664E-14, -19.008653481824105, 1.7472079647251122E-14,
    5.7149438332095271E-14, -18.440166735896412, 1.7564961435011732E-14,
    5.5279970724357672E-14, -17.871680703820211, 1.7657843222772339E-14,
    5.3410503116620072E-14, -17.303195528413923, 1.7750725010532949E-14,
    5.1541035508882467E-14, -16.734707379580612, 1.784364155944167E-14,
    4.9670868250094062E-14, -16.1662243469848, 1.793652334720228E-14,
    4.7801400642356469E-14, -15.597742599514085, 1.8029405134962886E-14,
    4.5931933034618863E-14, -15.029262279960165, 1.8122286922723493E-14,
    4.4062465426881264E-14, -14.460783531141434, 1.82151687104841E-14,
    4.2192997819143664E-14, -13.892302522960961, 1.8308085259392827E-14,
    4.0322830560355272E-14, -13.323827345083265, 1.8400967047153434E-14,
    3.8453362952617666E-14, -12.755353371812895, 1.8493855787143661E-14,
    3.6583755414669909E-14, -12.186881540759753, 1.8586744527133895E-14,
    3.4714147876722146E-14, -11.618411994752933, 1.8679633267124123E-14,
    3.284454033877439E-14, -11.049944479303266, 1.8772525483229169E-14,
    3.0974862835721548E-14, 1.8032929589223377E-14, -21.293254106153611,
    -4.7550042442029186E-15, 1.7100517735060578E-14, -221.28260872460294,
    -4.0148495073996836E-15, 1.6168454706881378E-14, -20.714120550445791,
    -3.2749716709843226E-15, 1.5236391678702178E-14, -20.145632518879907,
    -2.5350938345689586E-15, 1.4304328650522978E-14, -19.577144772710334,
    -1.7952159981535976E-15, 1.3371916796360176E-14, -19.008653481824105,
    -1.0550612613503626E-15, 1.2439853768180976E-14, -18.440166735896412,
    -3.1518342493500171E-16, 1.1507790740001776E-14, -17.871680703820211,
    4.2469441148035922E-16, 1.0575727711822576E-14, -17.303195528413923,
    1.1645722478957201E-15, 9.6433158576597761E-15, -16.734707379580612,
    1.9047269846989614E-15, 8.7112528294805777E-15, -16.1662243469848,
    2.6446048211143224E-15, 7.7791898013013745E-15, -15.597742599514085,
    3.384482657529677E-15, 6.8471267731221761E-15, -15.029262279960165,
    4.1243604939450379E-15, 5.915063744942976E-15, -14.460783531141434,
    4.8642383303603957E-15, 4.9826518907801774E-15, -13.892302522960961,
    5.604393067163637E-15, 4.0505888626009758E-15, -13.323827345083265,
    6.3442709035790011E-15, 3.1184560692250567E-15, -12.755353371812895,
    7.0842041200719349E-15, 2.1863232758491364E-15, -12.186881540759753,
    7.8241373365648719E-15, 1.2541904824732161E-15, -11.618411994752933,
    8.5640705530578058E-15, 3.2202280649893605E-16, -11.049944479303266,
    9.3040314595895308E-15, 6.6941200628136871E-14, -2.9296110745875248E-15,
    -21.293254106153611, 6.4628008414096468E-14, -4.014849507399682E-15,
    -221.28260872460294, 6.231568159183086E-14, -5.0996819400499212E-15,
    -20.714120550445791, 6.0003354769565265E-14, -6.184514372700162E-15,
    -20.145632518879907, 5.769102794729967E-14, -7.2693468053504028E-15,
    -19.577144772710334, 5.5377835733259267E-14, -8.3545852381625647E-15,
    -19.008653481824105, 5.3065508910993665E-14, -9.4394176708128E-15,
    -18.440166735896412, 5.0753182088728064E-14, -1.0524250103463042E-14,
    -17.871680703820211, 4.8440855266462469E-14, -1.1609082536113286E-14,
    -17.303195528413923, 4.6127663052422066E-14, -1.2694320968925444E-14,
    -16.734707379580612, 4.3815336230156458E-14, -1.3779153401575682E-14,
    -16.1662243469848, 4.1503009407890863E-14, -1.4863985834225924E-14,
    -15.597742599514085, 3.9190682585625261E-14, -1.5948818266876162E-14,
    -15.029262279960165, 3.687835576335966E-14, -1.7033650699526406E-14,
    -14.460783531141434, 3.4565163549319263E-14, -1.8118889132338561E-14,
    -13.892302522960961, 3.2252836727053662E-14, -1.9203721564988802E-14,
    -13.323827345083265, 2.99403368264331E-14, -2.0288635197671428E-14,
    -12.755353371812895, 2.7627836925812544E-14, -2.1373548830354055E-14,
    -12.186881540759753, 2.5315337025191979E-14, -2.2458462463036675E-14,
    -11.618411994752933, 2.3002750585393941E-14, -2.3543416695735491E-14,
    -11.049944479303266, -20.72447834608851, 1.6080497740539055E-14,
    6.4149610472126465E-14, -20.714120550445791, 1.6168454706881378E-14,
    6.2315681591830873E-14, -220.70376662185254, 1.6256378767512389E-14,
    6.0482438805392465E-14, -20.135565926748725, 1.6344302828143397E-14,
    5.8649196018954069E-14, -19.567365374236193, 1.64322268887744E-14,
    5.6815953232515661E-14, -18.999161240805375, 1.6520183855116724E-14,
    5.4982024352220069E-14, -18.430961402876555, 1.6608107915747732E-14,
    5.3148781565781667E-14, -17.86276213600221, 1.6696031976378739E-14,
    5.1315538779343265E-14, -17.294563582979354, 1.678395603700975E-14,
    4.9482295992904869E-14, -16.726362020311825, 1.687191300335207E-14,
    4.7648367112609264E-14, -16.158165324435924, 1.6959837063983081E-14,
    4.5815124326170869E-14, -15.589969770893457, 1.7047761124614088E-14,
    4.3981881539732461E-14, -15.021775502476093, 1.7135685185245093E-14,
    4.2148638753294065E-14, -14.453582661975519, 1.72236092458761E-14,
    4.0315395966855669E-14, -13.885387525895544, 1.7311566212218426E-14,
    3.8481467086560071E-14, -13.317197970672479, 1.7399490272849434E-14,
    3.6648224300121669E-14, -12.749009498585213, 1.74874209146227E-14,
    3.4814844294911829E-14, -12.180823025886085, 1.7575351556395975E-14,
    3.2981464289701989E-14, -11.612638695404186, 1.7663282198169243E-14,
    3.1148084284492149E-14, -11.044456263337146, 1.775121613051365E-14,
    2.9314635669896587E-14, 1.8130770959550385E-14, -20.72447834608851,
    -5.8306675914454773E-15, 1.7193399522821188E-14, -20.714120550445791,
    -5.0996819400499244E-15, 1.6256378767512386E-14, -220.70376662185254,
    -4.3689697587820813E-15, 1.5319358012203587E-14, -20.135565926748725,
    -3.6382575775142383E-15, 1.4382337256894788E-14, -19.567365374236193,
    -2.9075453962463952E-15, 1.3444965820165585E-14, -18.999161240805375,
    -2.1765597448508422E-15, 1.2507945064856784E-14, -18.430961402876555,
    -1.4458475635829992E-15, 1.1570924309547985E-14, -17.86276213600221,
    -7.1513538231516246E-16, 1.0633903554239184E-14, -17.294563582979354,
    1.5576798952680586E-17, 9.6965321175099844E-15, -16.726362020311825,
    7.46562450348243E-16, 8.7595113622011854E-15, -16.158165324435924,
    1.4772746316160829E-15, 7.8224906068923832E-15, -15.589969770893457,
    2.2079868128839165E-15, 6.8854698515835841E-15, -15.021775502476093,
    2.9386989941517596E-15, 5.9484490962747835E-15, -14.453582661975519,
    3.6694111754195963E-15, 5.0110776595455852E-15, -13.885387525895544,
    4.4003968268151587E-15, 4.0740569042367837E-15, -13.317197970672479,
    5.1311090080830018E-15, 3.1369660126439045E-15, -12.749009498585213,
    5.8618758833763843E-15, 2.1998751210510241E-15, -12.180823025886085,
    6.59264275866977E-15, 1.2627842294581441E-15, -11.612638695404186,
    7.32340963396315E-15, 3.2565826972322408E-16, -11.044456263337146,
    8.0542038562693063E-15, 6.5035494641906465E-14, -2.1805641527644851E-15,
    -20.72447834608851, 6.2758540806358862E-14, -3.2749716709843226E-15,
    -20.714120550445791, 6.0482438805392465E-14, -4.3689697587820813E-15,
    -220.70376662185254, 5.8206336804426067E-14, -5.4629678465798432E-15,
    -20.135565926748725, 5.5930234803459669E-14, -6.5569659343776026E-15,
    -19.567365374236193, 5.3653280967912066E-14, -7.6513734525974426E-15,
    -18.999161240805375, 5.1377178966945669E-14, -8.7453715403952012E-15,
    -18.430961402876555, 4.9101076965979265E-14, -9.8393696281929615E-15,
    -17.86276213600221, 4.6824974965012861E-14, -1.0933367715990726E-14,
    -17.294563582979354, 4.4548021129465264E-14, -1.2027775234210564E-14,
    -16.726362020311825, 4.227191912849886E-14, -1.3121773322008323E-14,
    -16.158165324435924, 3.9995817127532462E-14, -1.4215771409806083E-14,
    -15.589969770893457, 3.7719715126566065E-14, -1.5309769497603845E-14,
    -15.021775502476093, 3.5443613125599667E-14, -1.6403767585401604E-14,
    -14.453582661975519, 3.3166659290052064E-14, -1.7498175103621444E-14,
    -13.885387525895544, 3.0890557289085667E-14, -1.8592173191419206E-14,
    -13.317197970672479, 2.8614284921203021E-14, -1.9686253165301381E-14,
    -12.749009498585213, 2.6338012553320383E-14, -2.0780333139183555E-14,
    -12.180823025886085, 2.4061740185437741E-14, -2.1874413113065733E-14,
    -11.612638695404186, 2.1785382634096982E-14, -2.2968534029990113E-14,
    -11.044456263337146, -20.155702871419727, 1.5153394294926253E-14,
    6.180104527136807E-14, -20.145632518879907, 1.5236391678702178E-14,
    6.0003354769565265E-14, -20.135565926748729, 1.5319358012203587E-14,
    5.8206336804426067E-14, -220.12549933441716, 1.5402324345704995E-14,
    5.6409318839286856E-14, -19.557585975575, 1.54852906792064E-14,
    5.4612300874147658E-14, -18.989668999610288, 1.5568288062982323E-14,
    5.2814610372344866E-14, -18.421756069691014, 1.5651254396483731E-14,
    5.1017592407205668E-14, -17.853843568023862, 1.5734220729985136E-14,
    4.922057444206647E-14, -17.285931637411185, 1.5817187063486548E-14,
    4.7423556476927265E-14, -16.718016660936151, 1.5900184447262467E-14,
    4.5625865975124461E-14, -16.150106301780163, 1.5983150780763879E-14,
    4.3828848009985263E-14, -15.582196942165929, 1.6066117114265284E-14,
    4.2031830044846058E-14, -15.014288724885136, 1.6149083447766692E-14,
    4.023481207970686E-14, -14.446381792729436, 1.6232049781268098E-14,
    3.8437794114567662E-14, -13.878472528776683, 1.6315047165044026E-14,
    3.6640103612764864E-14, -13.310568596208251, 1.6398013498545431E-14,
    3.4843085647625672E-14, -12.742665625304086, 1.6480986042101742E-14,
    3.3045933175153749E-14, -12.174764510985693, 1.6563958585658052E-14,
    3.1248780702681825E-14, -11.606865396055438, 1.6646931129214363E-14,
    2.9451628230209908E-14, -11.038968047371029, 1.6729906777798128E-14,
    2.7654408504071629E-14, 1.8228612329877393E-14, -20.155702871419727,
    -6.9063309386880393E-15, 1.7286281310581795E-14, -20.145632518879907,
    -6.1845143727001652E-15, 1.6344302828143397E-14, -20.135565926748729,
    -5.4629678465798463E-15, 1.5402324345704995E-14, -220.12549933441716,
    -4.7414213204595211E-15, 1.4460345863266594E-14, -19.557585975575,
    -4.0198747943391991E-15, 1.3518014843970993E-14, -18.989668999610288,
    -3.298058228351325E-15, 1.2576036361532592E-14, -18.421756069691014,
    -2.576511702231003E-15, 1.1634057879094194E-14, -17.853843568023862,
    -1.8549651761106841E-15, 1.0692079396655792E-14, -17.285931637411185,
    -1.1334186499903621E-15, 9.7497483773601928E-15, -16.718016660936151,
    -4.1160208400247852E-16, 8.8077698949217931E-15, -16.150106301780163,
    3.0994444211784033E-16, 7.8657914124833918E-15, -15.582196942165929,
    1.031490968238156E-15, 6.9238129300449921E-15, -15.014288724885136,
    1.753037494358478E-15, 5.9818344476065917E-15, -14.446381792729436,
    2.4745840204787969E-15, 5.0395034283109929E-15, -13.878472528776683,
    3.1964005864666742E-15, 4.0975249458725917E-15, -13.310568596208251,
    3.9179471125869993E-15, 3.1554759560627527E-15, -12.742665625304086,
    4.6395476466808274E-15, 2.2134269662529122E-15, -12.174764510985693,
    5.3611481807746617E-15, 1.2713779764430721E-15, -11.606865396055438,
    6.0827487148684929E-15, 3.2929373294751206E-16, -11.038968047371029,
    6.8043762529490818E-15, 6.3129788655676059E-14, -1.4315172309414437E-15,
    -20.155702871419727, 6.0889073198621257E-14, -2.5350938345689617E-15,
    -20.145632518879907, 5.8649196018954069E-14, -3.63825757751424E-15,
    -20.135565926748729, 5.6409318839286856E-14, -4.7414213204595211E-15,
    -220.12549933441716, 5.4169441659619662E-14, -5.8445850634048E-15,
    -19.557585975575, 5.1928726202564866E-14, -6.9481616670323228E-15,
    -18.989668999610288, 4.9688849022897666E-14, -8.0513254099776E-15,
    -18.421756069691014, 4.7448971843230466E-14, -9.15448915292288E-15,
    -17.853843568023862, 4.5209094663563265E-14, -1.0257652895868164E-14,
    -17.285931637411185, 4.2968379206508463E-14, -1.1361229499495682E-14,
    -16.718016660936151, 4.0728502026841262E-14, -1.2464393242440962E-14,
    -16.150106301780163, 3.8488624847174062E-14, -1.3567556985386242E-14,
    -15.582196942165929, 3.6248747667506862E-14, -1.4670720728331521E-14,
    -15.014288724885136, 3.4008870487839662E-14, -1.57738844712768E-14,
    -14.446381792729436, 3.1768155030784859E-14, -1.6877461074904321E-14,
    -13.878472528776683, 2.9528277851117659E-14, -1.7980624817849604E-14,
    -13.310568596208251, 2.7288233015972943E-14, -1.9083871132931333E-14,
    -12.742665625304086, 2.5048188180828221E-14, -2.0187117448013062E-14,
    -12.174764510985693, 2.28081433456835E-14, -2.1290363763094787E-14,
    -11.606865396055438, 2.0568014682800019E-14, -2.2393651364244739E-14,
    -11.038968047371029, -19.586927824949626, 1.4226290849313454E-14,
    5.9452480070609662E-14, -19.577144772710337, 1.4304328650522978E-14,
    5.7691027947299658E-14, -19.567365374236193, 1.4382337256894788E-14,
    5.5930234803459669E-14, -19.557585975575, 1.4460345863266594E-14,
    5.4169441659619656E-14, -219.54780657671341, 1.45383544696384E-14,
    5.2408648515779662E-14, -18.980176758228151, 1.4616392270847921E-14,
    5.0647196392469664E-14, -18.412550736329113, 1.469440087721973E-14,
    4.8886403248629663E-14, -17.844924999879826, 1.4772409483591537E-14,
    4.7125610104789662E-14, -17.277299691682664, 1.4850418089963349E-14,
    4.5364816960949662E-14, -16.709671301426873, 1.492845589117287E-14,
    4.3603364837639664E-14, -16.142047279017515, 1.500646449754468E-14,
    4.1842571693799663E-14, -15.574424113331517, 1.5084473103916486E-14,
    4.0081778549959668E-14, -15.006801947187274, 1.5162481710288292E-14,
    3.8320985406119661E-14, -14.439180923376467, 1.52404903166601E-14,
    3.6560192262279667E-14, -13.871557531577656, 1.5318528117869626E-14,
    3.4798740138969669E-14, -13.303939221690579, 1.5396536724241432E-14,
    3.3037946995129662E-14, -12.736321751969514, 1.5474551169580782E-14,
    3.1277022055395668E-14, -12.168705996031857, 1.5552565614920133E-14,
    2.9516097115661662E-14, -11.601092096679968, 1.5630580060259483E-14,
    2.7755172175927668E-14, -11.033479831404909, 1.5708597425082606E-14,
    2.5994181338246668E-14, 1.83264537002044E-14, -19.586927824949626,
    -7.9819942859306012E-15, 1.73791630983424E-14, -19.577144772710337,
    -7.269346805350406E-15, 1.6432226888774404E-14, -19.567365374236193,
    -6.556965934377605E-15, 1.54852906792064E-14, -19.557585975575,
    -5.8445850634048009E-15, 1.4538354469638404E-14, -219.54780657671341,
    -5.1322041924319967E-15, 1.3591063867776401E-14, -18.980176758228151,
    -4.4195567118518046E-15, 1.2644127658208401E-14, -18.412550736329113,
    -3.7071758408790036E-15, 1.16971914486404E-14, -17.844924999879826,
    -2.9947949699062027E-15, 1.07502552390724E-14, -17.277299691682664,
    -2.2824140989334017E-15, 9.8029646372104011E-15, -16.709671301426873,
    -1.5697666183532E-15, 8.8560284276424E-15, -16.142047279017515,
    -8.5738574738039911E-16, 7.9090922180743989E-15, -15.574424113331517,
    -1.4500487640760445E-16, 6.9621560085064E-15, -15.006801947187274,
    5.6737599456519653E-16, 6.0152197989384E-15, -14.439180923376467,
    1.2797568655379943E-15, 5.0679291970764007E-15, -13.871557531577656,
    1.9924043461181959E-15, 4.1209929875083996E-15, -13.303939221690579,
    2.704785217091E-15, 3.1739858994816005E-15, -12.736321751969514,
    3.4172194099852767E-15, 2.2269788114548003E-15, -12.168705996031857,
    4.12965360287956E-15, 1.279971723428E-15, -11.601092096679968,
    4.8420877957738364E-15, 3.3292919617180004E-16, -11.033479831404909,
    5.5545486496288573E-15, 6.1224082669445666E-14, -6.8247030911840393E-16,
    -19.586927824949626, 5.9019605590883664E-14, -1.7952159981536024E-15,
    -19.577144772710337, 5.6815953232515667E-14, -2.9075453962464E-15,
    -19.567365374236193, 5.4612300874147658E-14, -4.0198747943392023E-15,
    -19.557585975575, 5.2408648515779668E-14, -5.1322041924320007E-15,
    -219.54780657671341, 5.0204171437217665E-14, -6.2449498814672023E-15,
    -18.980176758228151, 4.8000519078849663E-14, -7.35727927956E-15,
    -18.412550736329113, 4.579686672048166E-14, -8.4696086776528E-15,
    -17.844924999879826, 4.3593214362113657E-14, -9.5819380757456053E-15,
    -17.277299691682664, 4.1388737283551661E-14, -1.0694683764780804E-14,
    -16.709671301426873, 3.9185084925183665E-14, -1.1807013162873601E-14,
    -16.142047279017515, 3.6981432566815662E-14, -1.2919342560966404E-14,
    -15.574424113331517, 3.4777780208447665E-14, -1.4031671959059201E-14,
    -15.006801947187274, 3.2574127850079656E-14, -1.5144001357152002E-14,
    -14.439180923376467, 3.0369650771517666E-14, -1.6256747046187204E-14,
    -13.871557531577656, 2.8165998413149661E-14, -1.736907644428E-14,
    -13.303939221690579, 2.5962181110742865E-14, -1.8481489100561285E-14,
    -12.736321751969514, 2.375836380833606E-14, -1.9593901756842565E-14,
    -12.168705996031857, 2.1554546505929261E-14, -2.0706314413123845E-14,
    -11.601092096679968, 1.935064673150306E-14, -2.1818768698499362E-14,
    -11.033479831404909, -19.018149269924553, 1.3298840433848256E-14,
    5.7103035915809066E-14, -19.008653481824108, 1.3371916796360176E-14,
    5.5377835733259267E-14, -18.999161240805375, 1.3444965820165588E-14,
    5.3653280967912066E-14, -18.989668999610288, 1.3518014843970995E-14,
    5.1928726202564866E-14, -18.980176758228151, 1.3591063867776401E-14,
    5.0204171437217659E-14, -218.97068097017313, 1.3664140230288322E-14,
    4.8478971254667866E-14, -18.403341962908449, 1.3737189254093733E-14,
    4.6754416489320666E-14, -17.836003098288465, 1.381023827789914E-14,
    4.5029861723973465E-14, -17.268664519118236, 1.3883287301704547E-14,
    4.3305306958626265E-14, -16.701322821727668, 1.3956363664216469E-14,
    4.1580106776076465E-14, -16.133985242692432, 1.4029412688021879E-14,
    3.9855552010729265E-14, -15.566648377562128, 1.4102461711827287E-14,
    3.8130997245382065E-14, -14.999312369155188, 1.4175510735632692E-14,
    3.6406442480034864E-14, -14.431977360289999, 1.42485597594381E-14,
    3.4681887714687664E-14, -13.864639947285786, 1.4321636121950028E-14,
    3.2956687532137864E-14, -13.29730736670753, 1.4394685145755433E-14,
    3.1232132766790664E-14, -12.729975504805015, 1.4467739637302143E-14,
    2.9507448918002949E-14, -12.162645213856816, 1.4540794128848854E-14,
    2.7782765069215227E-14, -11.59531663669202, 1.4613848620395565E-14,
    2.6058081220427506E-14, -11.027989561465748, 1.4686905845812927E-14,
    2.433333282991953E-14, 1.8424331687810725E-14, -19.018149269924553,
    -9.05806020179114E-15, 1.7472079647251126E-14, -19.008653481824108,
    -8.3545852381625647E-15, 1.6520183855116724E-14, -18.999161240805375,
    -7.6513734525974426E-15, 1.5568288062982323E-14, -18.989668999610288,
    -6.9481616670323173E-15, 1.4616392270847924E-14, -18.980176758228151,
    -6.2449498814671983E-15, 1.3664140230288322E-14, -218.97068097017313,
    -5.5414749178386225E-15, 1.2712244438153922E-14, -18.403341962908449,
    -4.8382631322735E-15, 1.1760348646019524E-14, -17.836003098288465,
    -4.1350513467083814E-15, 1.0808452853885122E-14, -17.268664519118236,
    -3.4318395611432624E-15, 9.856200813325523E-15, -16.701322821727668,
    -2.7283645975146771E-15, 8.904305021191123E-15, -16.133985242692432,
    -2.0251528119495582E-15, 7.9524092290567215E-15, -15.566648377562128,
    -1.3219410263844424E-15, 7.0005134369223223E-15, -14.999312369155188,
    -6.187292408193234E-16, 6.0486176447879215E-15, -14.431977360289999,
    8.4482544745795568E-17, 5.0963656042283233E-15, -13.864639947285786,
    7.8795750837437771E-16, 4.1444698120939217E-15, -13.29730736670753,
    1.4911692939394998E-15, 3.1925027702744826E-15, -12.729975504805015,
    2.1944337151173089E-15, 2.2405357284550422E-15, -12.162645213856816,
    2.8976981362951243E-15, 1.2885686866356021E-15, -11.59531663669202,
    3.6009625574729334E-15, 3.3656601997364208E-16, -11.027989561465748,
    4.3042532964570938E-15, 5.9317663469897069E-14, 6.685694463645525E-17,
    -19.018149269924553, 5.7149438332095271E-14, -1.0550612613503626E-15,
    -19.008653481824108, 5.4982024352220069E-14, -2.1765597448508407E-15,
    -18.999161240805375, 5.2814610372344866E-14, -3.2980582283513218E-15,
    -18.989668999610288, 5.0647196392469664E-14, -4.4195567118518007E-15,
    -18.980176758228151, 4.8478971254667866E-14, -5.5414749178386233E-15,
    -218.97068097017313, 4.6311557274792664E-14, -6.6629734013391005E-15,
    -18.403341962908449, 4.4144143294917461E-14, -7.78447188483958E-15,
    -17.836003098288465, 4.1976729315042259E-14, -8.9059703683400645E-15,
    -17.268664519118236, 3.9808504177240461E-14, -1.0027888574326884E-14,
    -16.701322821727668, 3.7641090197365259E-14, -1.1149387057827362E-14,
    -16.133985242692432, 3.5473676217490062E-14, -1.2270885541327842E-14,
    -15.566648377562128, 3.3306262237614866E-14, -1.3392384024828323E-14,
    -14.999312369155188, 3.1138848257739664E-14, -1.4513882508328804E-14,
    -14.431977360289999, 2.8970623119937866E-14, -1.5635800714315622E-14,
    -13.864639947285786, 2.680320914006266E-14, -1.67572991978161E-14,
    -13.29730736670753, 2.4635632928602143E-14, -1.7878881625813852E-14,
    -12.729975504805015, 2.2468056717141623E-14, -1.90004640538116E-14,
    -12.162645213856816, 2.03004805056811E-14, -2.0122046481809348E-14,
    -11.59531663669202, 1.8132823178427921E-14, -2.1243670882055729E-14,
    -11.027989561465748, -18.449375509349135, 1.2371736988235457E-14,
    5.4754470715050671E-14, -18.440166735896412, 1.2439853768180976E-14,
    5.3065508910993672E-14, -18.430961402876555, 1.2507945064856786E-14,
    5.1377178966945669E-14, -18.421756069691011, 1.2576036361532595E-14,
    4.9688849022897666E-14, -18.412550736329113, 1.2644127658208402E-14,
    4.8000519078849663E-14, -18.403341962908449, 1.2712244438153922E-14,
    4.6311557274792664E-14, -218.39413662993405, 1.2780335734829733E-14,
    4.4623227330744667E-14, -17.827084530502528, 1.284842703150554E-14,
    4.293489738669667E-14, -17.260032573715733, 1.2916518328181348E-14,
    4.1246567442648667E-14, -16.692977462506974, 1.2984635108126869E-14,
    3.9557605638591662E-14, -16.125926220196998, 1.3052726404802678E-14,
    3.7869275694543665E-14, -15.558875548994951, 1.3120817701478486E-14,
    3.6180945750495662E-14, -14.99182559169784, 1.3188908998154294E-14,
    3.4492615806447659E-14, -14.424776491124083, 1.32570002948301E-14,
    3.2804285862399669E-14, -13.857724950220369, 1.3325117074775625E-14,
    3.111532405834267E-14, -13.290677992296745, 1.3393208371451432E-14,
    2.9426994114294667E-14, -12.723631631577335, 1.3461304764781182E-14,
    2.7738537798244868E-14, -12.156586698983148, 1.3529401158110934E-14,
    2.6050081482195067E-14, -11.589543337343272, 1.3597497551440683E-14,
    2.4361625166145265E-14, -11.022501345499631, 1.3665596493097406E-14,
    2.2673105664094569E-14, 1.8522173058137733E-14, -18.449375509349135,
    -1.01337235490337E-14, 1.7564961435011732E-14, -18.440166735896412,
    -9.4394176708128055E-15, 1.6608107915747735E-14, -18.430961402876555,
    -8.7453715403952044E-15, 1.5651254396483731E-14, -18.421756069691011,
    -8.051325409977597E-15, 1.4694400877219734E-14, -18.412550736329113,
    -7.3572792795599959E-15, 1.373718925409373E-14, -18.403341962908449,
    -6.6629734013391021E-15, 1.278033573482973E-14, -218.39413662993405,
    -5.968927270921501E-15, 1.1823482215565731E-14, -17.827084530502528,
    -5.2748811405039031E-15, 1.086662869630173E-14, -17.260032573715733,
    -4.580835010086302E-15, 9.90941707317573E-15, -16.692977462506974,
    -3.8865291318653987E-15, 8.95256355391173E-15, -16.125926220196998,
    -3.1924830014477976E-15, 7.9957100346477285E-15, -15.558875548994951,
    -2.4984368710302028E-15, 7.03885651538373E-15, -14.99182559169784,
    -1.8043907406126018E-15, 6.08200299611973E-15, -14.424776491124083,
    -1.1103446101950038E-15, 5.124791372993731E-15, -13.857724950220369,
    -4.1603873197410369E-16, 4.16793785372973E-15, -13.290677992296745,
    2.7800739844350055E-16, 3.2110127136933304E-15, -12.723631631577335,
    9.7210547842175829E-16, 2.2540875736569303E-15, -12.156586698983148,
    1.6662035584000192E-15, 1.29716243362053E-15, -11.589543337343272,
    2.3603016383782769E-15, 3.4020148319793006E-16, -11.022501345499631,
    3.0544256931368693E-15, 5.7411957483666675E-14, 8.15903866459495E-16,
    -18.449375509349135, 5.5279970724357666E-14, -3.1518342493500329E-16,
    -18.440166735896412, 5.3148781565781667E-14, -1.4458475635830008E-15,
    -18.430961402876555, 5.1017592407205668E-14, -2.576511702231003E-15,
    -18.421756069691011, 4.8886403248629669E-14, -3.7071758408790013E-15,
    -18.412550736329113, 4.6754416489320666E-14, -4.8382631322735027E-15,
    -18.403341962908449, 4.4623227330744667E-14, -5.968927270921501E-15,
    -218.39413662993405, 4.2492038172168668E-14, -7.0995914095695016E-15,
    -17.827084530502528, 4.0360849013592663E-14, -8.2302555482175054E-15,
    -17.260032573715733, 3.8228862254283666E-14, -9.3613428396120038E-15,
    -16.692977462506974, 3.6097673095707661E-14, -1.0492006978260003E-14,
    -16.125926220196998, 3.3966483937131662E-14, -1.1622671116908003E-14,
    -15.558875548994951, 3.1835294778555663E-14, -1.2753335255556003E-14,
    -14.99182559169784, 2.9704105619979658E-14, -1.3883999394204003E-14,
    -14.424776491124083, 2.7572118860670664E-14, -1.5015086685598505E-14,
    -13.857724950220369, 2.5440929702094662E-14, -1.6145750824246505E-14,
    -13.290677992296745, 2.3309581023372065E-14, -1.7276499593443804E-14,
    -12.723631631577335, 2.1178232344649462E-14, -1.8407248362641104E-14,
    -12.156586698983148, 1.9046883665926861E-14, -1.9537997131838403E-14,
    -11.589543337343272, 1.6915455227130962E-14, -2.0668788216310355E-14,
    -11.022501345499631, -17.880602605443634, 1.1444633542622657E-14,
    5.2405905514292263E-14, -17.871680703820211, 1.1507790740001776E-14,
    5.0753182088728064E-14, -17.86276213600221, 1.1570924309547987E-14,
    4.9101076965979271E-14, -17.853843568023862, 1.1634057879094195E-14,
    4.7448971843230466E-14, -17.844924999879826, 1.1697191448640402E-14,
    4.579686672048166E-14, -17.836003098288465, 1.1760348646019522E-14,
    4.4144143294917467E-14, -17.827084530502528, 1.1823482215565734E-14,
    4.2492038172168668E-14, -217.81816596256962, 1.1886615785111941E-14,
    4.0839933049419863E-14, -17.251400628179624, 1.1949749354658148E-14,
    3.918782792667107E-14, -16.684632103163366, 1.2012906552037271E-14,
    3.7535104501106865E-14, -16.117867197589334, 1.2076040121583479E-14,
    3.5882999378358065E-14, -15.551102720320864, 1.2139173691129686E-14,
    3.4230894255609266E-14, -14.984338814160328, 1.2202307260675893E-14,
    3.2578789132860467E-14, -14.417575621904723, 1.22654408302221E-14,
    3.0926684010111668E-14, -13.850809953101507, 1.2328598027601225E-14,
    2.9273960584547469E-14, -13.284048617832516, 1.2391731597147432E-14,
    2.7621855461798666E-14, -12.717287758296209, 1.2454869892260223E-14,
    2.5969626678486788E-14, -12.150528184082756, 1.2518008187373015E-14,
    2.4317397895174906E-14, -11.583770037994526, 1.2581146482485804E-14,
    2.2665169111863024E-14, -11.017013129533511, 1.2644287140381887E-14,
    2.1012878498269608E-14, 1.862001442846474E-14, -17.880602605443634,
    -1.120938689627626E-14, 1.7657843222772339E-14, -17.871680703820211,
    -1.0524250103463045E-14, 1.6696031976378742E-14, -17.86276213600221,
    -9.8393696281929631E-15, 1.573422072998514E-14, -17.853843568023862,
    -9.1544891529228783E-15, 1.477240948359154E-14, -17.844924999879826,
    -8.4696086776527967E-15, 1.3810238277899138E-14, -17.836003098288465,
    -7.7844718848395817E-15, 1.2848427031505539E-14, -17.827084530502528,
    -7.0995914095695016E-15, 1.1886615785111939E-14, -217.81816596256962,
    -6.4147109342994216E-15, 1.0924804538718338E-14, -17.251400628179624,
    -5.7298304590293415E-15, 9.962633333025938E-15, -16.684632103163366,
    -5.0446936662161171E-15, 9.00082208663234E-15, -16.117867197589334,
    -4.35981319094604E-15, 8.0390108402387372E-15, -15.551102720320864,
    -3.6749327156759633E-15, 7.0771995938451384E-15, -14.984338814160328,
    -2.99005224040588E-15, 6.115388347451538E-15, -14.417575621904723,
    -2.3051717651358032E-15, 5.1532171417591388E-15, -13.850809953101507,
    -1.6200349723225819E-15, 4.1914058953655376E-15, -13.284048617832516,
    -9.3515449705249873E-16, 3.2295226571121786E-15, -12.717287758296209,
    -2.5022275827379233E-16, 2.267639418858818E-15, -12.150528184082756,
    4.3470898050491723E-16, 1.3057561806054581E-15, -11.583770037994526,
    1.1196407192836205E-15, 3.4383694642221804E-16, -11.017013129533511,
    1.8045980898166448E-15, 5.5506251497436269E-14, 1.5649507882825348E-15,
    -17.880602605443634, 5.3410503116620072E-14, 4.2469441148035764E-16,
    -17.871680703820211, 5.1315538779343265E-14, -7.1513538231516088E-16,
    -17.86276213600221, 4.922057444206647E-14, -1.8549651761106826E-15,
    -17.853843568023862, 4.7125610104789669E-14, -2.9947949699062019E-15,
    -17.844924999879826, 4.5029861723973465E-14, -4.1350513467083838E-15,
    -17.836003098288465, 4.293489738669667E-14, -5.2748811405039015E-15,
    -17.827084530502528, 4.0839933049419863E-14, -6.4147109342994216E-15,
    -217.81816596256962, 3.8744968712143068E-14, -7.5545407280949448E-15,
    -17.251400628179624, 3.6649220331326864E-14, -8.6947971048971236E-15,
    -16.684632103163366, 3.4554255994050063E-14, -9.8346268986926421E-15,
    -16.117867197589334, 3.2459291656773262E-14, -1.0974456692488162E-14,
    -15.551102720320864, 3.0364327319496467E-14, -1.2114286486283682E-14,
    -14.984338814160328, 2.8269362982219662E-14, -1.3254116280079204E-14,
    -14.417575621904723, 2.6173614601403465E-14, -1.4394372656881384E-14,
    -13.850809953101507, 2.4078650264126661E-14, -1.5534202450676903E-14,
    -13.284048617832516, 2.198352911814198E-14, -1.6674117561073757E-14,
    -12.717287758296209, 1.98884079721573E-14, -1.781403267147061E-14,
    -12.150528184082756, 1.7793286826172623E-14, -1.8953947781867458E-14,
    -11.583770037994526, 1.5698087275834003E-14, -2.0093905550564977E-14,
    -11.017013129533511, -17.311830701026452, 1.0517530097009855E-14,
    5.0057340313533867E-14, -17.303195528413926, 1.0575727711822575E-14,
    4.8440855266462463E-14, -17.294563582979354, 1.0633903554239184E-14,
    4.6824974965012861E-14, -17.285931637411185, 1.0692079396655794E-14,
    4.5209094663563259E-14, -17.277299691682664, 1.07502552390724E-14,
    4.3593214362113657E-14, -17.268664519118236, 1.080845285388512E-14,
    4.1976729315042265E-14, -17.260032573715733, 1.0866628696301732E-14,
    4.0360849013592663E-14, -17.251400628179624, 1.092480453871834E-14,
    3.8744968712143068E-14, -217.24276868249655, 1.0982980381134946E-14,
    3.7129088410693466E-14, -16.676286743686152, 1.1041177995947668E-14,
    3.5512603363622061E-14, -16.109808174858756, 1.1099353838364277E-14,
    3.3896723062172465E-14, -15.543329891534551, 1.1157529680780885E-14,
    3.2280842760722857E-14, -14.976852036515911, 1.1215705523197492E-14,
    3.0664962459273262E-14, -14.4103747526052, 1.12738813656141E-14,
    2.9049082157823666E-14, -13.843894955929201, 1.1332078980426823E-14,
    2.7432597110752262E-14, -13.277419243314844, 1.1390254822843431E-14,
    2.5816716809302663E-14, -12.710943884961637, 1.1448435019739261E-14,
    2.4200715558728707E-14, -12.144469669128918, 1.1506615216635093E-14,
    2.2584714308154746E-14, -11.577996738619056, 1.1564795413530922E-14,
    2.0968713057580784E-14, -11.011524913567394, 1.1622977787666365E-14,
    1.9352651332444647E-14, 1.8717855798791748E-14, -17.311830701026452,
    -1.2285050243518822E-14, 1.7750725010532949E-14, -17.303195528413926,
    -1.1609082536113287E-14, 1.678395603700975E-14, -17.294563582979354,
    -1.0933367715990725E-14, 1.5817187063486548E-14, -17.285931637411185,
    -1.025765289586816E-14, 1.4850418089963349E-14, -17.277299691682664,
    -9.581938075745599E-15, 1.3883287301704547E-14, -17.268664519118236,
    -8.9059703683400645E-15, 1.2916518328181346E-14, -17.260032573715733,
    -8.2302555482175023E-15, 1.1949749354658148E-14, -17.251400628179624,
    -7.5545407280949433E-15, 1.0982980381134946E-14, -217.24276868249655,
    -6.8788259079723827E-15, 1.0015849592876146E-14, -16.676286743686152,
    -6.20285820056684E-15, 9.0490806193529477E-15, -16.109808174858756,
    -5.5271433804442812E-15, 8.0823116458297459E-15, -15.543329891534551,
    -4.8514285603217254E-15, 7.1155426723065456E-15, -14.976852036515911,
    -4.1757137401991632E-15, 6.1487736987833462E-15, -14.4103747526052,
    -3.4999989200766042E-15, 5.1816429105245473E-15, -13.843894955929201,
    -2.8240312126710649E-15, 4.2148739370013455E-15, -13.277419243314844,
    -2.1483163925485012E-15, 3.2480326005310264E-15, -12.710943884961637,
    -1.4725509949693477E-15, 2.2811912640607061E-15, -12.144469669128918,
    -7.9678559739018946E-16, 1.3143499275903861E-15, -11.577996738619056,
    -1.2102019981103597E-16, 3.4747240964650607E-16, -11.011524913567394,
    5.5477048649642028E-16, 5.3600545511205863E-14, 2.3139977101055762E-15,
    -17.311830701026452, 5.1541035508882467E-14, 1.1645722478957186E-15,
    -17.303195528413926, 4.9482295992904869E-14, 1.5576798952680586E-17,
    -17.294563582979354, 4.7423556476927259E-14, -1.1334186499903621E-15,
    -17.285931637411185, 4.5364816960949662E-14, -2.2824140989334009E-15,
    -17.277299691682664, 4.3305306958626265E-14, -3.4318395611432616E-15,
    -17.268664519118236, 4.1246567442648667E-14, -4.5808350100863004E-15,
    -17.260032573715733, 3.9187827926671063E-14, -5.72983045902934E-15,
    -17.251400628179624, 3.712908841069346E-14, -6.8788259079723843E-15,
    -217.24276868249655, 3.5069578408370063E-14, -8.0282513701822434E-15,
    -16.676286743686152, 3.3010838892392465E-14, -9.1772468191252814E-15,
    -16.109808174858756, 3.0952099376414861E-14, -1.0326242268068321E-14,
    -15.543329891534551, 2.8893359860437264E-14, -1.1475237717011362E-14,
    -14.976852036515911, 2.683462034445966E-14, -1.2624233165954403E-14,
    -14.4103747526052, 2.4775110342136263E-14, -1.3773658628164262E-14,
    -13.843894955929201, 2.2716370826158659E-14, -1.4922654077107304E-14,
    -13.277419243314844, 2.0657477212911902E-14, -1.6071735528703709E-14,
    -12.710943884961637, 1.8598583599665138E-14, -1.722081698030011E-14,
    -12.144469669128918, 1.6539689986418381E-14, -1.8369898431896516E-14,
    -11.577996738619056, 1.4480719324537037E-14, -1.95190228848196E-14,
    -11.011524913567394, -16.743055859333261, 9.5900796815446558E-15,
    4.7707896158733266E-14, -16.734707379580612, 9.6433158576597761E-15,
    4.6127663052422066E-14, -16.726362020311825, 9.6965321175099844E-15,
    4.4548021129465264E-14, -16.718016660936151, 9.7497483773601943E-15,
    4.2968379206508463E-14, -16.709671301426873, 9.8029646372104011E-15,
    4.1388737283551661E-14, -16.701322821727668, 9.8562008133255214E-15,
    3.9808504177240461E-14, -16.692977462506978, 9.9094170731757329E-15,
    3.8228862254283666E-14, -16.684632103163366, 9.96263333302594E-15,
    3.6649220331326864E-14, -16.676286743686152, 1.0015849592876146E-14,
    3.5069578408370063E-14, -216.66793826403239, 1.006908576899127E-14,
    3.3489345302058863E-14, -16.101746138565733, 1.0122302028841478E-14,
    3.1909703379102061E-14, -15.535554155797229, 1.0175518288691685E-14,
    3.0330061456145266E-14, -14.969362458531922, 1.0228734548541893E-14,
    2.8750419533188458E-14, -14.403171189572175, 1.02819508083921E-14,
    2.7170777610231663E-14, -13.836977371690779, 1.0335186984507223E-14,
    2.5590544503920463E-14, -13.270787388358517, 1.0388403244357432E-14,
    2.4010902580963665E-14, -12.704597637797137, 1.0441623487460622E-14,
    2.2431142421335988E-14, -12.138408886953879, 1.0494843730563813E-14,
    2.0851382261708305E-14, -11.572221278631108, 1.0548063973667002E-14,
    1.9271622102080628E-14, -11.006034643628233, 1.0601286208396685E-14,
    1.7691802824117506E-14, 1.8815733786398069E-14, -16.743055859333261,
    -1.3361116159379361E-14, 1.784364155944167E-14, -16.734707379580612,
    -1.2694320968925446E-14, 1.6871913003352073E-14, -16.726362020311825,
    -1.2027775234210564E-14, 1.590018444726247E-14, -16.718016660936151,
    -1.1361229499495679E-14, 1.492845589117287E-14, -16.709671301426873,
    -1.0694683764780799E-14, 1.3956363664216469E-14, -16.701322821727668,
    -1.0027888574326884E-14, 1.2984635108126868E-14, -16.692977462506978,
    -9.3613428396120022E-15, 1.2012906552037269E-14, -16.684632103163366,
    -8.6947971048971236E-15, 1.1041177995947669E-14, -16.676286743686152,
    -8.0282513701822419E-15, 1.0069085768991268E-14, -216.66793826403239,
    -7.3614561797283189E-15, 9.09735721290167E-15, -16.101746138565733,
    -6.69491044501344E-15, 8.1256286568120669E-15, -15.535554155797229,
    -6.0283647102985633E-15, 7.1539001007224685E-15, -14.969362458531922,
    -5.3618189755836831E-15, 6.1821715446328678E-15, -14.403171189572175,
    -4.6952732408688045E-15, 5.2100793176764691E-15, -13.836977371690779,
    -4.0284780504148847E-15, 4.2383507615868677E-15, -13.270787388358517,
    -3.3619323157E-15, 3.2665494713239085E-15, -12.704597637797137,
    -2.6953366898373155E-15, 2.2947481810609481E-15, -12.138408886953879,
    -2.0287410639746249E-15, 1.3229468907979881E-15, -11.572221278631108,
    -1.362145438111939E-15, 3.5110923344834806E-16, -11.006034643628233,
    -6.9552486667534633E-16, 5.1694126311657266E-14, 3.0633249638604369E-15,
    -16.743055859333261, 4.9670868250094068E-14, 1.9047269846989583E-15,
    -16.734707379580612, 4.7648367112609264E-14, 7.4656245034823988E-16,
    -16.726362020311825, 4.5625865975124467E-14, -4.1160208400248326E-16,
    -16.718016660936151, 4.360336483763967E-14, -1.5697666183532009E-15,
    -16.709671301426873, 4.1580106776076465E-14, -2.7283645975146827E-15,
    -16.701322821727668, 3.9557605638591668E-14, -3.8865291318654E-15,
    -16.692977462506978, 3.7535104501106865E-14, -5.04469366621612E-15,
    -16.684632103163366, 3.5512603363622061E-14, -6.202858200566845E-15,
    -16.676286743686152, 3.3489345302058863E-14, -7.3614561797283236E-15,
    -216.66793826403239, 3.1466844164574059E-14, -8.519620714079042E-15,
    -16.101746138565733, 2.9444343027089262E-14, -9.677785248429762E-15,
    -15.535554155797229, 2.7421841889604465E-14, -1.0835949782780482E-14,
    -14.969362458531922, 2.5399340752119661E-14, -1.1994114317131204E-14,
    -14.403171189572175, 2.3376082690556463E-14, -1.3152712296292682E-14,
    -13.836977371690779, 2.1353581553071659E-14, -1.4310876830643404E-14,
    -13.270787388358517, 1.9330929030771181E-14, -1.5469128053956276E-14,
    -12.704597637797137, 1.7308276508470702E-14, -1.6627379277269149E-14,
    -12.138408886953879, 1.528562398617022E-14, -1.7785630500582019E-14,
    -11.572221278631108, 1.32628957714619E-14, -1.8943925068375967E-14,
    -11.006034643628233, -16.174286383363331, 8.6629762359318554E-15,
    4.5359330957974864E-14, -16.166224346984805, 8.7112528294805761E-15,
    4.3815336230156464E-14, -16.158165324435924, 8.7595113622011854E-15,
    4.2271919128498866E-14, -16.150106301780163, 8.8077698949217947E-15,
    4.0728502026841262E-14, -16.142047279017515, 8.8560284276424E-15,
    3.9185084925183665E-14, -16.133985242692432, 8.9043050211911214E-15,
    3.7641090197365265E-14, -16.125926220196998, 8.9525635539117323E-15,
    3.6097673095707661E-14, -16.117867197589334, 9.00082208663234E-15,
    3.4554255994050063E-14, -16.109808174858756, 9.0490806193529477E-15,
    3.3010838892392465E-14, -16.101746138565733, 9.09735721290167E-15,
    3.1466844164574066E-14, -216.09368711611575, 9.1456157456222776E-15,
    2.9923427062916461E-14, -15.527781327262112, 9.1938742783428853E-15,
    2.8380009961258864E-14, -14.96187568110664, 9.242132811063493E-15,
    2.6836592859601263E-14, -14.395970320454358, 9.2903913437841E-15,
    2.5293175757943665E-14, -13.830062374678805, 9.3386679373328245E-15,
    2.3749181030125262E-14, -13.264158014001179, 9.38692647005343E-15,
    2.2205763928467664E-14, -12.698253764596179, 9.4351886149396628E-15,
    2.0662231301577907E-14, -12.132350372080209, 9.4834507598258934E-15,
    1.9118698674688144E-14, -11.56644797928236, 9.531712904712124E-15,
    1.7575166047798387E-14, -11.000546427662115, 9.5799768556811661E-15,
    1.6031575658292548E-14, 1.8913575156725077E-14, -16.174286383363331,
    -1.4436779506621921E-14, 1.793652334720228E-14, -16.166224346984805,
    -1.3779153401575685E-14, 1.6959837063983081E-14, -16.158165324435924,
    -1.3121773322008324E-14, 1.5983150780763879E-14, -16.150106301780163,
    -1.2464393242440959E-14, 1.500646449754468E-14, -16.142047279017515,
    -1.1807013162873598E-14, 1.4029412688021877E-14, -16.133985242692432,
    -1.1149387057827364E-14, 1.3052726404802677E-14, -16.125926220196998,
    -1.0492006978260003E-14, 1.2076040121583478E-14, -16.117867197589334,
    -9.8346268986926421E-15, 1.1099353838364277E-14, -16.109808174858756,
    -9.1772468191252814E-15, 1.0122302028841477E-14, -16.101746138565733,
    -8.5196207140790389E-15, 9.1456157456222776E-15, -216.09368711611575,
    -7.86224063451168E-15, 8.1689294624030755E-15, -15.527781327262112,
    -7.2048605549443238E-15, 7.1922431791838766E-15, -14.96187568110664,
    -6.5474804753769631E-15, 6.215556895964676E-15, -14.395970320454358,
    -5.8901003958096039E-15, 5.2385050864418769E-15, -13.830062374678805,
    -5.232474290763363E-15, 4.2618188032226756E-15, -13.264158014001179,
    -4.5750942111959991E-15, 3.2850594147427567E-15, -12.698253764596179,
    -3.9176649265328661E-15, 2.3083000262628362E-15, -12.132350372080209,
    -3.2602356418697284E-15, 1.3315406377829161E-15, -11.56644797928236,
    -2.6028063572065955E-15, 3.5474469667263604E-16, -11.000546427662115,
    -1.9453524699955693E-15, 4.9788420325426867E-14, 3.8123718856834751E-15,
    -16.174286383363331, 4.7801400642356469E-14, 2.6446048211143176E-15,
    -16.166224346984805, 4.5815124326170869E-14, 1.4772746316160798E-15,
    -16.158165324435924, 4.3828848009985269E-14, 3.0994444211783717E-16,
    -16.150106301780163, 4.1842571693799669E-14, -8.5738574738040148E-16,
    -16.142047279017515, 3.9855552010729265E-14, -2.0251528119495629E-15,
    -16.133985242692432, 3.7869275694543665E-14, -3.1924830014478004E-15,
    -16.125926220196998, 3.5882999378358065E-14, -4.359813190946041E-15,
    -16.117867197589334, 3.3896723062172465E-14, -5.5271433804442852E-15,
    -16.109808174858756, 3.1909703379102061E-14, -6.6949104450134434E-15,
    -16.101746138565733, 2.9923427062916461E-14, -7.8622406345116829E-15,
    -216.09368711611575, 2.7937150746730862E-14, -9.0295708240099239E-15,
    -15.527781327262112, 2.5950874430545262E-14, -1.0196901013508162E-14,
    -14.96187568110664, 2.3964598114359662E-14, -1.1364231203006403E-14,
    -14.395970320454358, 2.1977578431289264E-14, -1.2531998267575563E-14,
    -13.830062374678805, 1.9991302115103661E-14, -1.3699328457073804E-14,
    -13.264158014001179, 1.8004877125541102E-14, -1.4866746021586228E-14,
    -12.698253764596179, 1.601845213597854E-14, -1.6034163586098652E-14,
    -12.132350372080209, 1.403202714641598E-14, -1.7201581150611076E-14,
    -11.56644797928236, 1.204552782016494E-14, -1.836904240263059E-14,
    -11.000546427662115, -15.605518335310194, 7.7358727903190551E-15,
    4.3010765757216462E-14, -15.597742599514088, 7.7791898013013761E-15,
    4.1503009407890863E-14, -15.589969770893461, 7.8224906068923863E-15,
    3.9995817127532462E-14, -15.582196942165931, 7.8657914124833934E-15,
    3.8488624847174062E-14, -15.574424113331517, 7.9090922180744E-15,
    3.6981432566815662E-14, -15.56664837756213, 7.9524092290567215E-15,
    3.5473676217490069E-14, -15.558875548994951, 7.9957100346477317E-15,
    3.3966483937131662E-14, -15.551102720320866, 8.0390108402387388E-15,
    3.2459291656773262E-14, -15.543329891534551, 8.0823116458297459E-15,
    3.0952099376414861E-14, -15.535554155797231, 8.1256286568120684E-15,
    2.9444343027089262E-14, -15.527781327262112, 8.1689294624030771E-15,
    2.7937150746730862E-14, -215.52000849863347, 8.2122302679940842E-15,
    2.6429958466372461E-14, -14.95438890360119, 8.2555310735850928E-15,
    2.4922766186014061E-14, -14.388769451267065, 8.2988318791761E-15,
    2.3415573905655664E-14, -13.823147377608043, 8.3421488901584241E-15,
    2.1907817556330061E-14, -13.257528639590394, 8.3854496957494311E-15,
    2.0400625275971664E-14, -12.691909891368498, 8.4287537424187016E-15,
    1.8893320181819827E-14, -12.126291857206541, 8.4720577890879737E-15,
    1.7386015087667984E-14, -11.560674679933612, 8.5153618357572426E-15,
    1.5878709993516147E-14, -10.995058211695996, 8.5586675029656456E-15,
    1.4371348492467584E-14, 1.9011416527052085E-14, -15.605518335310194,
    -1.551244285386448E-14, 1.8029405134962886E-14, -15.597742599514088,
    -1.4863985834225924E-14, 1.7047761124614088E-14, -15.589969770893461,
    -1.4215771409806083E-14, 1.6066117114265287E-14, -15.582196942165931,
    -1.356755698538624E-14, 1.5084473103916486E-14, -15.574424113331517,
    -1.2919342560966399E-14, 1.4102461711827284E-14, -15.56664837756213,
    -1.2270885541327843E-14, 1.3120817701478484E-14, -15.558875548994951,
    -1.1622671116908003E-14, 1.2139173691129685E-14, -15.551102720320866,
    -1.0974456692488164E-14, 1.1157529680780885E-14, -15.543329891534551,
    -1.0326242268068323E-14, 1.0175518288691685E-14, -15.535554155797231,
    -9.67778524842976E-15, 9.1938742783428853E-15, -15.527781327262112,
    -9.02957082400992E-15, 8.2122302679940826E-15, -215.52000849863347,
    -8.3813563995900842E-15, 7.2305862576452846E-15, -14.95438890360119,
    -7.733141975170243E-15, 6.2489422472964835E-15, -14.388769451267065,
    -7.0849275507504049E-15, 5.2669308552072847E-15, -13.823147377608043,
    -6.4364705311118444E-15, 4.2852868448584835E-15, -13.257528639590394,
    -5.788256106692E-15, 3.3035693581616045E-15, -12.691909891368498,
    -5.13999316322842E-15, 2.3218518714647243E-15, -12.126291857206541,
    -4.4917302197648335E-15, 1.3401343847678441E-15, -11.560674679933612,
    -3.8434672763012519E-15, 3.5838015989692407E-16, -10.995058211695996,
    -3.1951800733157938E-15, 4.7882714339196467E-14, 4.5614188075065165E-15,
    -15.605518335310194, 4.5931933034618863E-14, 3.3844826575296778E-15,
    -15.597742599514088, 4.3981881539732467E-14, 2.2079868128839205E-15,
    -15.589969770893461, 4.2031830044846065E-14, 1.0314909682381576E-15,
    -15.582196942165931, 4.0081778549959668E-14, -1.450048764076005E-16,
    -15.574424113331517, 3.8130997245382065E-14, -1.3219410263844424E-15,
    -15.56664837756213, 3.6180945750495662E-14, -2.4984368710302E-15,
    -15.558875548994951, 3.4230894255609266E-14, -3.67493271567596E-15,
    -15.551102720320866, 3.2280842760722864E-14, -4.8514285603217246E-15,
    -15.543329891534551, 3.033006145614526E-14, -6.0283647102985633E-15,
    -15.535554155797231, 2.8380009961258864E-14, -7.2048605549443222E-15,
    -15.527781327262112, 2.6429958466372461E-14, -8.3813563995900827E-15,
    -215.52000849863347, 2.4479906971486062E-14, -9.5578522442358416E-15,
    -14.95438890360119, 2.2529855476599663E-14, -1.0734348088881604E-14,
    -14.388769451267065, 2.0579074172022065E-14, -1.1911284238858443E-14,
    -13.823147377608043, 1.862902267713566E-14, -1.3087780083504202E-14,
    -13.257528639590394, 1.6678825220311021E-14, -1.4264363989216181E-14,
    -12.691909891368498, 1.4728627763486382E-14, -1.5440947894928156E-14,
    -12.126291857206541, 1.277843030666174E-14, -1.6617531800640131E-14,
    -11.560674679933612, 1.082815986886798E-14, -1.7794159736885216E-14,
    -10.995058211695996, -15.03675185799225, 6.8087693447062547E-15,
    4.0662200556458061E-14, -15.029262279960165, 6.8471267731221768E-15,
    3.9190682585625268E-14, -15.021775502476093, 6.8854698515835857E-15,
    3.7719715126566065E-14, -15.014288724885136, 6.9238129300449929E-15,
    3.6248747667506862E-14, -15.006801947187274, 6.962156008506401E-15,
    3.4777780208447659E-14, -14.999312369155188, 7.0005134369223223E-15,
    3.3306262237614866E-14, -14.99182559169784, 7.0388565153837319E-15,
    3.1835294778555663E-14, -14.984338814160328, 7.0771995938451392E-15,
    3.036432731949646E-14, -14.976852036515911, 7.1155426723065472E-15,
    2.8893359860437264E-14, -14.969362458531922, 7.1539001007224685E-15,
    2.7421841889604465E-14, -14.961875681106639, 7.1922431791838781E-15,
    2.5950874430545265E-14, -14.954388903601188, 7.2305862576452846E-15,
    2.4479906971486062E-14, -214.94690212600221, 7.2689293361066927E-15,
    2.3008939512426862E-14, -14.381568581999604, 7.3072724145681E-15,
    2.1537972053367666E-14, -13.816232380467804, 7.3456298429840236E-15,
    2.0066454082534863E-14, -13.25089926512082, 7.3839729214454316E-15,
    1.8595486623475664E-14, -12.685566018087371, 7.422318869897742E-15,
    1.7124409062061747E-14, -12.12023334230615, 7.460664818350054E-15,
    1.5653331500647823E-14, -11.554901380584866, 7.4990107668023628E-15,
    1.4182253939233905E-14, -10.989569995729877, 7.5373581502501251E-15,
    1.2711121326642626E-14, 1.9109257897379093E-14, -15.03675185799225,
    -1.6588106201107039E-14, 1.8122286922723496E-14, -15.029262279960165,
    -1.5948818266876165E-14, 1.7135685185245096E-14, -15.021775502476093,
    -1.5309769497603845E-14, 1.6149083447766692E-14, -15.014288724885136,
    -1.4670720728331518E-14, 1.5162481710288295E-14, -15.006801947187274,
    -1.4031671959059198E-14, 1.4175510735632692E-14, -14.999312369155188,
    -1.3392384024828323E-14, 1.3188908998154292E-14, -14.99182559169784,
    -1.2753335255556003E-14, 1.2202307260675893E-14, -14.984338814160328,
    -1.2114286486283684E-14, 1.1215705523197493E-14, -14.976852036515911,
    -1.1475237717011362E-14, 1.0228734548541892E-14, -14.969362458531922,
    -1.0835949782780479E-14, 9.242132811063493E-15, -14.961875681106639,
    -1.019690101350816E-14, 8.2555310735850913E-15, -14.954388903601188,
    -9.5578522442358431E-15, 7.2689293361066927E-15, -214.94690212600221,
    -8.9188034749635229E-15, 6.2823275986282917E-15, -14.381568581999604,
    -8.2797547056912043E-15, 5.2953566239726932E-15, -13.816232380467804,
    -7.6404667714603242E-15, 4.3087548864942914E-15, -13.25089926512082,
    -7.0014180021880008E-15, 3.3220793015804527E-15, -12.685566018087371,
    -6.3623213999239705E-15, 2.3354037166666124E-15, -12.12023334230615,
    -5.7232247976599371E-15, 1.3487281317527721E-15, -11.554901380584866,
    -5.0841281953959084E-15, 3.6201562312121205E-16, -10.989569995729877,
    -4.4450076766360183E-15, 4.5977008352966067E-14, 5.3104657293295563E-15,
    -15.03675185799225, 4.406246542688127E-14, 4.1243604939450371E-15,
    -15.029262279960165, 4.2148638753294065E-14, 2.9386989941517596E-15,
    -15.021775502476093, 4.0234812079706866E-14, 1.7530374943584773E-15,
    -15.014288724885136, 3.8320985406119668E-14, 5.6737599456519889E-16,
    -15.006801947187274, 3.6406442480034864E-14, -6.1872924081932261E-16,
    -14.999312369155188, 3.4492615806447665E-14, -1.8043907406126006E-15,
    -14.99182559169784, 3.2578789132860467E-14, -2.9900522404058809E-15,
    -14.984338814160328, 3.0664962459273262E-14, -4.1757137401991648E-15,
    -14.976852036515911, 2.8750419533188465E-14, -5.3618189755836831E-15,
    -14.969362458531922, 2.6836592859601263E-14, -6.5474804753769615E-15,
    -14.961875681106639, 2.4922766186014061E-14, -7.733141975170243E-15,
    -14.954388903601188, 2.3008939512426862E-14, -8.9188034749635229E-15,
    -214.94690212600221, 2.1095112838839664E-14, -1.0104464974756804E-14,
    -14.381568581999604, 1.9180569912754863E-14, -1.1290570210141323E-14,
    -13.816232380467804, 1.7266743239167661E-14, -1.2476231709934603E-14,
    -13.25089926512082, 1.5352773315080942E-14, -1.3661981956846133E-14,
    -12.685566018087371, 1.3438803390994222E-14, -1.4847732203757659E-14,
    -12.12023334230615, 1.15248334669075E-14, -1.6033482450669186E-14,
    -11.554901380584866, 9.61079191757102E-15, -1.7219277071139838E-14,
    -10.989569995729877, -14.467987094227905, 5.8816658990934544E-15,
    3.8313635355699665E-14, -14.460783531141436, 5.915063744942976E-15,
    3.687835576335966E-14, -14.453582661975519, 5.948449096274785E-15,
    3.5443613125599667E-14, -14.446381792729436, 5.9818344476065925E-15,
    3.4008870487839662E-14, -14.439180923376467, 6.0152197989384007E-15,
    3.2574127850079656E-14, -14.431977360289999, 6.0486176447879223E-15,
    3.1138848257739664E-14, -14.424776491124085, 6.0820029961197305E-15,
    2.9704105619979665E-14, -14.417575621904724, 6.1153883474515388E-15,
    2.8269362982219662E-14, -14.4103747526052, 6.1487736987833462E-15,
    2.6834620344459663E-14, -14.403171189572175, 6.1821715446328678E-15,
    2.5399340752119661E-14, -14.395970320454358, 6.2155568959646768E-15,
    2.3964598114359662E-14, -14.388769451267063, 6.2489422472964843E-15,
    2.2529855476599663E-14, -14.381568581999606, 6.2823275986282925E-15,
    2.1095112838839661E-14, -214.37436771263862, 6.3157129499601E-15,
    1.9660370201079661E-14, -13.809317383247398, 6.3491107958096231E-15,
    1.8225090608739663E-14, -13.244269890581769, 6.3824961471414306E-15,
    1.6790347970979663E-14, -12.679222144747456, 6.4158839973767808E-15,
    1.5355497942303663E-14, -12.114174827352313, 6.4492718476121327E-15,
    1.3920647913627663E-14, -11.549128081209396, 6.4826596978474822E-15,
    1.2485797884951662E-14, -10.984081779763759, 6.5160487975346054E-15,
    1.1050894160817663E-14, 1.92070992677061E-14, -14.467987094227905,
    -1.7663769548349604E-14, 1.8215168710484103E-14, -14.460783531141436,
    -1.7033650699526406E-14, 1.7223609245876103E-14, -14.453582661975519,
    -1.6403767585401607E-14, 1.62320497812681E-14, -14.446381792729436,
    -1.57738844712768E-14, 1.5240490316660102E-14, -14.439180923376467,
    -1.5144001357152E-14, 1.42485597594381E-14, -14.431977360289999,
    -1.4513882508328804E-14, 1.32570002948301E-14, -14.424776491124085,
    -1.3883999394204003E-14, 1.22654408302221E-14, -14.417575621904724,
    -1.3254116280079204E-14, 1.1273881365614101E-14, -14.4103747526052,
    -1.2624233165954403E-14, 1.02819508083921E-14, -14.403171189572175,
    -1.19941143171312E-14, 9.2903913437841E-15, -14.395970320454358,
    -1.1364231203006401E-14, 8.2988318791761E-15, -14.388769451267063,
    -1.0734348088881605E-14, 7.3072724145681E-15, -14.381568581999606,
    -1.0104464974756804E-14, 6.3157129499601E-15, -214.37436771263862,
    -9.4745818606320053E-15, 5.323782392738101E-15, -13.809317383247398,
    -8.844463011808804E-15, 4.3322229281300994E-15, -13.244269890581769,
    -8.2145798976840017E-15, 3.3405892449993005E-15, -12.679222144747456,
    -7.5846496366195243E-15, 2.3489555618685E-15, -12.114174827352313,
    -6.9547193755550422E-15, 1.3573218787377E-15, -11.549128081209396,
    -6.3247891144905648E-15, 3.6565108634550003E-16, -10.984081779763759,
    -5.6948352799562428E-15, 4.4071302366735668E-14, 6.0595126511525968E-15,
    -14.467987094227905, 4.2192997819143664E-14, 4.864238330360398E-15,
    -14.460783531141436, 4.0315395966855669E-14, 3.6694111754196E-15,
    -14.453582661975519, 3.8437794114567662E-14, 2.4745840204787977E-15,
    -14.446381792729436, 3.6560192262279667E-14, 1.2797568655379991E-15,
    -14.439180923376467, 3.4681887714687664E-14, 8.4482544745797146E-17,
    -14.431977360289999, 3.2804285862399662E-14, -1.1103446101949995E-15,
    -14.424776491124085, 3.0926684010111661E-14, -2.3051717651358E-15,
    -14.417575621904724, 2.904908215782366E-14, -3.4999989200766042E-15,
    -14.4103747526052, 2.7170777610231663E-14, -4.695273240868803E-15,
    -14.403171189572175, 2.5293175757943662E-14, -5.8901003958096024E-15,
    -14.395970320454358, 2.3415573905655661E-14, -7.0849275507504025E-15,
    -14.388769451267063, 2.1537972053367663E-14, -8.2797547056912027E-15,
    -14.381568581999606, 1.9660370201079661E-14, -9.4745818606320037E-15,
    -214.37436771263862, 1.7782065653487661E-14, -1.0669856181424202E-14,
    -13.809317383247398, 1.590446380119966E-14, -1.1864683336365003E-14,
    -13.244269890581769, 1.4026721409850861E-14, -1.3059599924476083E-14,
    -12.679222144747456, 1.214897901850206E-14, -1.4254516512587163E-14,
    -12.114174827352313, 1.027123662715326E-14, -1.5449433100698244E-14,
    -11.549128081209396, 8.39342396627406E-15, -1.6644394405394461E-14,
    -10.984081779763759, -13.89922010725283, 4.9542154836282548E-15,
    3.5964191200899063E-14, -13.892302522960962, 4.9826518907801766E-15,
    3.4565163549319263E-14, -13.885387525895545, 5.0110776595455852E-15,
    3.3166659290052064E-14, -13.878472528776683, 5.0395034283109929E-15,
    3.1768155030784859E-14, -13.871557531577654, 5.0679291970764007E-15,
    3.036965077151766E-14, -13.864639947285784, 5.0963656042283225E-15,
    2.8970623119937866E-14, -13.857724950220369, 5.124791372993731E-15,
    2.7572118860670661E-14, -13.850809953101507, 5.1532171417591396E-15,
    2.6173614601403462E-14, -13.843894955929201, 5.1816429105245465E-15,
    2.4775110342136263E-14, -13.836977371690779, 5.2100793176764684E-15,
    2.3376082690556463E-14, -13.830062374678805, 5.2385050864418777E-15,
    2.1977578431289264E-14, -13.823147377608043, 5.2669308552072847E-15,
    2.0579074172022062E-14, -13.816232380467804, 5.2953566239726932E-15,
    1.918056991275486E-14, -13.809317383247398, 5.323782392738101E-15,
    1.7782065653487661E-14, -213.8023997989475, 5.3522187998900236E-15,
    1.6383038001907861E-14, -13.237638035577342, 5.3806445686554313E-15,
    1.4984533742640662E-14, -12.672875897561578, 5.4090724650981411E-15,
    1.3585924804910944E-14, -12.108114045171927, 5.4375003615408532E-15,
    1.2187315867181223E-14, -11.543352621221448, 5.465928257983563E-15,
    1.0788706929451503E-14, -10.978591509824598, 5.4943572182649258E-15,
    9.390045652490525E-15, 1.9304977255312425E-14, -13.89922010725283,
    -1.8739835464210143E-14, 1.8308085259392824E-14, -13.892302522960962,
    -1.8118889132338568E-14, 1.7311566212218426E-14, -13.885387525895545,
    -1.7498175103621444E-14, 1.6315047165044023E-14, -13.878472528776683,
    -1.6877461074904321E-14, 1.5318528117869626E-14, -13.871557531577654,
    -1.6256747046187197E-14, 1.4321636121950021E-14, -13.864639947285784,
    -1.5635800714315622E-14, 1.3325117074775622E-14, -13.857724950220369,
    -1.5015086685598505E-14, 1.2328598027601223E-14, -13.850809953101507,
    -1.4394372656881384E-14, 1.1332078980426823E-14, -13.843894955929201,
    -1.3773658628164262E-14, 1.0335186984507222E-14, -13.836977371690779,
    -1.3152712296292679E-14, 9.3386679373328229E-15, -13.830062374678805,
    -1.253199826757556E-14, 8.3421488901584209E-15, -13.823147377608043,
    -1.1911284238858445E-14, 7.345629842984022E-15, -13.816232380467804,
    -1.1290570210141324E-14, 6.3491107958096215E-15, -13.809317383247398,
    -1.0669856181424204E-14, 5.3522187998900228E-15, -213.8023997989475,
    -1.0048909849552624E-14, 4.3556997527156215E-15, -13.237638035577342,
    -9.4281958208355E-15, 3.3591061157921826E-15, -12.672875897561578,
    -8.80743533148749E-15, 2.3625124788687421E-15, -12.108114045171927,
    -8.1866748421394776E-15, 1.3659188419453021E-15, -11.543352621221448,
    -7.5659143527914679E-15, 3.6928791014734207E-16, -10.978591509824598,
    -6.9451306331280078E-15, 4.216488316718707E-14, 6.808839904907456E-15,
    -13.89922010725283, 4.0322830560355265E-14, 5.604393067163637E-15,
    -13.892302522960962, 3.8481467086560065E-14, 4.40039682681516E-15,
    -13.885387525895545, 3.6640103612764864E-14, 3.1964005864666773E-15,
    -13.878472528776683, 3.4798740138969669E-14, 1.9924043461181991E-15,
    -13.871557531577654, 3.2956687532137864E-14, 7.8795750837437692E-16,
    -13.864639947285784, 3.1115324058342663E-14, -4.1603873197410014E-16,
    -13.857724950220369, 2.9273960584547462E-14, -1.6200349723225811E-15,
    -13.850809953101507, 2.7432597110752262E-14, -2.8240312126710649E-15,
    -13.843894955929201, 2.5590544503920463E-14, -4.0284780504148831E-15,
    -13.836977371690779, 2.3749181030125262E-14, -5.2324742907633614E-15,
    -13.830062374678805, 2.1907817556330061E-14, -6.4364705311118428E-15,
    -13.823147377608043, 2.0066454082534863E-14, -7.6404667714603226E-15,
    -13.816232380467804, 1.8225090608739663E-14, -8.844463011808804E-15,
    -13.809317383247398, 1.6383038001907864E-14, -1.0048909849552622E-14,
    -213.8023997989475, 1.4541674528112663E-14, -1.1252906089901104E-14,
    -13.237638035577342, 1.2700173227710143E-14, -1.2456992449728652E-14,
    -12.672875897561578, 1.0858671927307622E-14, -1.36610788095562E-14,
    -12.108114045171927, 9.0171706269051E-15, -1.4865165169383747E-14,
    -11.543352621221448, 7.1756004131989217E-15, -1.6069296588950828E-14,
    -10.978591509824598, -13.330459200066313, 4.0271120380154537E-15,
    3.3615626000140662E-14, -13.323827345083266, 4.0505888626009766E-15,
    3.2252836727053662E-14, -13.317197970672481, 4.0740569042367845E-15,
    3.0890557289085667E-14, -13.310568596208253, 4.0975249458725924E-15,
    2.9528277851117659E-14, -13.303939221690579, 4.1209929875084E-15,
    2.8165998413149657E-14, -13.29730736670753, 4.1444698120939217E-15,
    2.6803209140062664E-14, -13.290677992296747, 4.1679378537297304E-15,
    2.5440929702094662E-14, -13.284048617832518, 4.1914058953655384E-15,
    2.4078650264126661E-14, -13.277419243314844, 4.2148739370013463E-15,
    2.2716370826158659E-14, -13.270787388358517, 4.2383507615868677E-15,
    2.1353581553071659E-14, -13.264158014001179, 4.2618188032226772E-15,
    1.9991302115103661E-14, -13.257528639590394, 4.2852868448584843E-15,
    1.8629022677135663E-14, -13.250899265120822, 4.3087548864942922E-15,
    1.7266743239167661E-14, -13.244269890581769, 4.3322229281301E-15,
    1.590446380119966E-14, -13.237638035577344, 4.3556997527156223E-15,
    1.454167452811266E-14, -213.23100866115854, 4.37916779435143E-15,
    1.3179395090144662E-14, -12.66653202431252, 4.4026375925771807E-15,
    1.1817013685152862E-14, -12.102055530276882, 4.4261073908029327E-15,
    1.0454632280161061E-14, -11.537579321867357, 4.4495771890286824E-15,
    9.0922508751692612E-15, -10.973103293858481, 4.4730478655494053E-15,
    7.7298184866655624E-15, 1.9402818625639433E-14, -13.330459200066313,
    -1.9815498811452702E-14, 1.8400967047153434E-14, -13.323827345083266,
    -1.9203721564988809E-14, 1.7399490272849434E-14, -13.317197970672481,
    -1.8592173191419206E-14, 1.6398013498545431E-14, -13.310568596208253,
    -1.79806248178496E-14, 1.5396536724241432E-14, -13.303939221690579,
    -1.736907644428E-14, 1.4394685145755429E-14, -13.29730736670753,
    -1.6757299197816105E-14, 1.339320837145143E-14, -13.290677992296747,
    -1.6145750824246505E-14, 1.239173159714743E-14, -13.284048617832518,
    -1.5534202450676903E-14, 1.1390254822843431E-14, -13.277419243314844,
    -1.4922654077107304E-14, 1.038840324435743E-14, -13.270787388358517,
    -1.43108768306434E-14, 9.38692647005343E-15, -13.264158014001179,
    -1.3699328457073801E-14, 8.38544969574943E-15, -13.257528639590394,
    -1.3087780083504205E-14, 7.38397292144543E-15, -13.250899265120822,
    -1.2476231709934604E-14, 6.38249614714143E-15, -13.244269890581769,
    -1.1864683336365003E-14, 5.3806445686554313E-15, -13.237638035577344,
    -1.1252906089901105E-14, 4.3791677943514295E-15, -213.23100866115854,
    -1.0641357716331501E-14, 3.3776160592110308E-15, -12.66653202431252,
    -1.0029763568183044E-14, 2.37606432407063E-15, -12.102055530276882,
    -9.4181694200345812E-15, 1.3745125889302301E-15, -11.537579321867357,
    -8.8065752718861243E-15, 3.7292337337163005E-16, -10.973103293858481,
    -8.1949582364482339E-15, 4.0259177180956664E-14, 7.5578868267304966E-15,
    -13.330459200066313, 3.8453362952617666E-14, 6.3442709035789979E-15,
    -13.323827345083266, 3.6648224300121669E-14, 5.131109008083E-15,
    -13.317197970672481, 3.4843085647625665E-14, 3.9179471125869978E-15,
    -13.310568596208253, 3.3037946995129668E-14, 2.7047852170909993E-15,
    -13.303939221690579, 3.1232132766790664E-14, 1.4911692939394975E-15,
    -13.29730736670753, 2.9426994114294667E-14, 2.7800739844350016E-16,
    -13.290677992296747, 2.7621855461798663E-14, -9.3515449705250031E-16,
    -13.284048617832518, 2.581671680930266E-14, -2.1483163925485035E-15,
    -13.277419243314844, 2.4010902580963661E-14, -3.3619323157000026E-15,
    -13.270787388358517, 2.2205763928467664E-14, -4.5750942111960023E-15,
    -13.264158014001179, 2.0400625275971661E-14, -5.7882561066920023E-15,
    -13.257528639590394, 1.8595486623475664E-14, -7.0014180021880016E-15,
    -13.250899265120822, 1.679034797097966E-14, -8.2145798976840033E-15,
    -13.244269890581769, 1.4984533742640662E-14, -9.4281958208355019E-15,
    -13.237638035577344, 1.3179395090144662E-14, -1.0641357716331503E-14,
    -213.23100866115854, 1.1374121322480061E-14, -1.1854610417358603E-14,
    -12.66653202431252, 9.56884755481546E-15, -1.3067863118385705E-14,
    -12.102055530276882, 7.76357378715086E-15, -1.42811158194128E-14,
    -11.537579321867357, 5.95823246190196E-15, -1.549441392320545E-14,
    -10.973103293858481, -12.761699618977392, 3.0999391984321746E-15,
    3.1266885008573821E-14, -12.755353371812895, 3.1184560692250563E-15,
    2.99403368264331E-14, -12.749009498585215, 3.1369660126439049E-15,
    2.8614284921203021E-14, -12.742665625304086, 3.1554759560627527E-15,
    2.728823301597294E-14, -12.736321751969516, 3.1739858994816005E-15,
    2.5962181110742858E-14, -12.729975504805017, 3.1925027702744822E-15,
    2.4635632928602143E-14, -12.723631631577337, 3.2110127136933308E-15,
    2.3309581023372062E-14, -12.717287758296209, 3.2295226571121782E-15,
    2.198352911814198E-14, -12.710943884961637, 3.2480326005310264E-15,
    2.0657477212911902E-14, -12.704597637797139, 3.2665494713239085E-15,
    1.9330929030771181E-14, -12.698253764596179, 3.2850594147427567E-15,
    1.8004877125541102E-14, -12.691909891368498, 3.3035693581616045E-15,
    1.6678825220311021E-14, -12.685566018087371, 3.3220793015804523E-15,
    1.5352773315080939E-14, -12.679222144747456, 3.3405892449993E-15,
    1.4026721409850861E-14, -12.67287589756158, 3.3591061157921826E-15,
    1.2700173227710143E-14, -12.66653202431252, 3.3776160592110304E-15,
    1.1374121322480063E-14, -212.66018767626809, 3.3961273881046851E-15,
    1.0047970161867854E-14, -12.095996561921559, 3.41463871699834E-15,
    8.7218190012556461E-15, -11.531805590385424, 3.4331500458919944E-15,
    7.3956678406434385E-15, -10.967614667097752, 3.4516620675230535E-15,
    6.0694670523401663E-15, 1.9500667319422303E-14, -12.761699618977392,
    -2.0891242672418858E-14, 1.8493855787143664E-14, -12.755353371812895,
    -2.0288635197671428E-14, 1.7487420914622705E-14, -12.749009498585215,
    -1.9686253165301381E-14, 1.6480986042101742E-14, -12.742665625304086,
    -1.9083871132931326E-14, 1.5474551169580782E-14, -12.736321751969516,
    -1.8481489100561278E-14, 1.446773963730214E-14, -12.729975504805017,
    -1.7878881625813852E-14, 1.3461304764781181E-14, -12.723631631577337,
    -1.7276499593443804E-14, 1.2454869892260223E-14, -12.717287758296209,
    -1.6674117561073757E-14, 1.1448435019739261E-14, -12.710943884961637,
    -1.6071735528703709E-14, 1.0441623487460621E-14, -12.704597637797139,
    -1.5469128053956273E-14, 9.4351886149396628E-15, -12.698253764596179,
    -1.4866746021586225E-14, 8.4287537424187E-15, -12.691909891368498,
    -1.4264363989216181E-14, 7.42231886989774E-15, -12.685566018087371,
    -1.3661981956846133E-14, 6.4158839973767808E-15, -12.679222144747456,
    -1.3059599924476085E-14, 5.4090724650981419E-15, -12.67287589756158,
    -1.2456992449728652E-14, 4.4026375925771807E-15, -12.66653202431252,
    -1.1854610417358601E-14, 3.3961273881046855E-15, -212.66018767626809,
    -1.1252183296513078E-14, 2.389617183632189E-15, -12.095996561921559,
    -1.0649756175667553E-14, 1.383106979159693E-15, -11.531805590385424,
    -1.0047329054822031E-14, 3.7655910871142886E-16, -10.967614667097752,
    -9.4448793897387662E-15, 3.8353328552062623E-14, 8.3069898149399E-15,
    -12.761699618977392, 3.6583755414669909E-14, 7.0842041200719333E-15,
    -12.755353371812895, 3.4814844294911829E-14, 5.8618758833763843E-15,
    -12.749009498585215, 3.3045933175153742E-14, 4.6395476466808289E-15,
    -12.742665625304086, 3.1277022055395668E-14, 3.4172194099852791E-15,
    -12.736321751969516, 2.9507448918002942E-14, 2.1944337151173093E-15,
    -12.729975504805017, 2.7738537798244865E-14, 9.7210547842175987E-16,
    -12.723631631577337, 2.5969626678486785E-14, -2.5022275827379272E-16,
    -12.717287758296209, 2.42007155587287E-14, -1.4725509949693479E-15,
    -12.710943884961637, 2.2431142421335982E-14, -2.6953366898373147E-15,
    -12.704597637797139, 2.06622313015779E-14, -3.9176649265328661E-15,
    -12.698253764596179, 1.8893320181819821E-14, -5.1399931632284183E-15,
    -12.691909891368498, 1.7124409062061743E-14, -6.36232139992397E-15,
    -12.685566018087371, 1.535549794230366E-14, -7.5846496366195227E-15,
    -12.679222144747456, 1.3585924804910944E-14, -8.80743533148749E-15,
    -12.67287589756158, 1.181701368515286E-14, -1.0029763568183043E-14,
    -12.66653202431252, 1.0047970161867854E-14, -1.1252183296513078E-14,
    -212.66018767626809, 8.2789266385828441E-15, -1.2474603024843114E-14,
    -12.095996561921559, 6.5098831152978362E-15, -1.3697022753173148E-14,
    -11.531805590385424, 4.7407733902493646E-15, -1.4919488227320423E-14,
    -10.967614667097752, -12.192942322934794, 2.172766358848894E-15,
    2.8918144017006981E-14, -12.186881540759755, 2.186323275849136E-15,
    2.7627836925812541E-14, -12.180823025886085, 2.1998751210510245E-15,
    2.6338012553320383E-14, -12.174764510985694, 2.2134269662529122E-15,
    2.5048188180828218E-14, -12.168705996031857, 2.2269788114548003E-15,
    2.375836380833606E-14, -12.162645213856816, 2.2405357284550418E-15,
    2.2468056717141623E-14, -12.156586698983148, 2.2540875736569303E-15,
    2.1178232344649462E-14, -12.150528184082757, 2.267639418858818E-15,
    1.98884079721573E-14, -12.14446966912892, 2.2811912640607061E-15,
    1.8598583599665141E-14, -12.138408886953879, 2.2947481810609481E-15,
    1.7308276508470702E-14, -12.132350372080211, 2.3083000262628362E-15,
    1.601845213597854E-14, -12.126291857206541, 2.3218518714647239E-15,
    1.4728627763486379E-14, -12.12023334230615, 2.335403716666612E-15,
    1.343880339099422E-14, -12.114174827352313, 2.3489555618685E-15,
    1.214897901850206E-14, -12.108114045171929, 2.3625124788687421E-15,
    1.0858671927307621E-14, -12.102055530276882, 2.37606432407063E-15,
    9.56884755481546E-15, -12.095996561921559, 2.3896171836321886E-15,
    8.2789266385828457E-15, -212.08993759356358, 2.4031700431937479E-15,
    6.9890057223502292E-15, -11.526031858903492, 2.4167229027553064E-15,
    5.6990848061176134E-15, -10.962126040337026, 2.4302762694967008E-15,
    4.4091156180147694E-15, 1.9598516013205174E-14, -12.192942322934794,
    -2.1966986533385016E-14, 1.8586744527133895E-14, -12.186881540759755,
    -2.1373548830354055E-14, 1.7575351556395975E-14, -12.180823025886085,
    -2.0780333139183558E-14, 1.6563958585658052E-14, -12.174764510985694,
    -2.0187117448013055E-14, 1.5552565614920133E-14, -12.168705996031857,
    -1.9593901756842562E-14, 1.4540794128848851E-14, -12.162645213856816,
    -1.90004640538116E-14, 1.3529401158110931E-14, -12.156586698983148,
    -1.8407248362641104E-14, 1.2518008187373013E-14, -12.150528184082757,
    -1.7814032671470607E-14, 1.1506615216635092E-14, -12.14446966912892,
    -1.722081698030011E-14, 1.0494843730563812E-14, -12.138408886953879,
    -1.6627379277269146E-14, 9.4834507598258934E-15, -12.132350372080211,
    -1.6034163586098649E-14, 8.47205778908797E-15, -12.126291857206541,
    -1.5440947894928156E-14, 7.4606648183500524E-15, -12.12023334230615,
    -1.4847732203757659E-14, 6.4492718476121311E-15, -12.114174827352313,
    -1.4254516512587166E-14, 5.4375003615408525E-15, -12.108114045171929,
    -1.3661078809556201E-14, 4.4261073908029312E-15, -12.102055530276882,
    -1.3067863118385701E-14, 3.41463871699834E-15, -12.095996561921559,
    -1.2474603024843114E-14, 2.4031700431937479E-15, -212.08993759356358,
    -1.1881342931300524E-14, 1.3917013693891557E-15, -11.526031858903492,
    -1.1288082837757936E-14, 3.8019484405122766E-16, -10.962126040337026,
    -1.0694800543029299E-14, 3.6447479923168587E-14, 9.0560928031493042E-15,
    -12.192942322934794, 3.4714147876722146E-14, 7.82413733656487E-15,
    -12.186881540759755, 3.2981464289701989E-14, 6.5926427586697684E-15,
    -12.180823025886085, 3.1248780702681825E-14, 5.3611481807746617E-15,
    -12.174764510985694, 2.9516097115661668E-14, 4.12965360287956E-15,
    -12.168705996031857, 2.7782765069215224E-14, 2.8976981362951215E-15,
    -12.162645213856816, 2.6050081482195063E-14, 1.6662035584000202E-15,
    -12.156586698983148, 2.4317397895174903E-14, 4.3470898050491565E-16,
    -12.150528184082757, 2.2584714308154742E-14, -7.9678559739019163E-16,
    -12.14446966912892, 2.08513822617083E-14, -2.0287410639746265E-15,
    -12.138408886953879, 1.9118698674688144E-14, -3.2602356418697296E-15,
    -12.132350372080211, 1.7386015087667981E-14, -4.4917302197648343E-15,
    -12.126291857206541, 1.5653331500647823E-14, -5.7232247976599371E-15,
    -12.12023334230615, 1.3920647913627661E-14, -6.9547193755550438E-15,
    -12.114174827352313, 1.2187315867181222E-14, -8.1866748421394776E-15,
    -12.108114045171929, 1.0454632280161061E-14, -9.4181694200345827E-15,
    -12.102055530276882, 8.7218190012556445E-15, -1.0649756175667554E-14,
    -12.095996561921559, 6.9890057223502284E-15, -1.1881342931300524E-14,
    -212.08993759356358, 5.2561924434448123E-15, -1.3112929686933494E-14,
    -11.526031858903492, 3.523314318596768E-15, -1.4344562531435398E-14,
    -10.962126040337026, -11.624187454740882, 1.245593519265614E-15,
    2.656940302544014E-14, -11.618411994752934, 1.2541904824732159E-15,
    2.5315337025191979E-14, -11.612638695404186, 1.2627842294581441E-15,
    2.4061740185437744E-14, -11.60686539605544, 1.2713779764430723E-15,
    2.28081433456835E-14, -11.60109209667997, 1.279971723428E-15,
    2.1554546505929257E-14, -11.59531663669202, 1.2885686866356019E-15,
    2.03004805056811E-14, -11.589543337343274, 1.29716243362053E-15,
    1.9046883665926861E-14, -11.583770037994526, 1.3057561806054581E-15,
    1.7793286826172619E-14, -11.577996738619058, 1.3143499275903861E-15,
    1.6539689986418381E-14, -11.572221278631108, 1.3229468907979879E-15,
    1.528562398617022E-14, -11.56644797928236, 1.3315406377829161E-15,
    1.4032027146415982E-14, -11.560674679933612, 1.3401343847678441E-15,
    1.277843030666174E-14, -11.554901380584868, 1.3487281317527721E-15,
    1.15248334669075E-14, -11.549128081209398, 1.3573218787377E-15,
    1.027123662715326E-14, -11.543352621221448, 1.3659188419453021E-15,
    9.0171706269051E-15, -11.537579321867357, 1.3745125889302301E-15,
    7.7635737871508618E-15, -11.531805590385424, 1.3831069791596928E-15,
    6.509883115297837E-15, -11.526031858903492, 1.3917013693891557E-15,
    5.2561924434448123E-15, -211.52025812741888, 1.4002957596186186E-15,
    4.0025017715917883E-15, -10.9566374135763, 1.4088904714703488E-15,
    2.7487641836893724E-15, 1.9696364706988044E-14, -11.624187454740882,
    -2.3042730394351172E-14, 1.8679633267124126E-14, -11.618411994752934,
    -2.2458462463036678E-14, 1.7663282198169246E-14, -11.612638695404186,
    -2.1874413113065736E-14, 1.6646931129214363E-14, -11.60686539605544,
    -2.1290363763094784E-14, 1.5630580060259486E-14, -11.60109209667997,
    -2.0706314413123839E-14, 1.4613848620395562E-14, -11.59531663669202,
    -2.0122046481809348E-14, 1.3597497551440682E-14, -11.589543337343274,
    -1.9537997131838403E-14, 1.2581146482485804E-14, -11.583770037994526,
    -1.8953947781867461E-14, 1.1564795413530924E-14, -11.577996738619058,
    -1.8369898431896516E-14, 1.0548063973667002E-14, -11.572221278631108,
    -1.7785630500582019E-14, 9.531712904712124E-15, -11.56644797928236,
    -1.7201581150611073E-14, 8.515361835757241E-15, -11.560674679933612,
    -1.6617531800640131E-14, 7.4990107668023628E-15, -11.554901380584868,
    -1.6033482450669189E-14, 6.4826596978474822E-15, -11.549128081209398,
    -1.5449433100698244E-14, 5.4659282579835638E-15, -11.543352621221448,
    -1.486516516938375E-14, 4.4495771890286824E-15, -11.537579321867357,
    -1.42811158194128E-14, 3.4331500458919952E-15, -11.531805590385424,
    -1.3697022753173149E-14, 2.4167229027553068E-15, -11.526031858903492,
    -1.3112929686933494E-14, 1.4002957596186186E-15, -211.52025812741888,
    -1.2528836620693842E-14, 3.8383057939102646E-16, -10.9566374135763,
    -1.1944721696319831E-14, 3.4541631294274545E-14, 9.8051957913587088E-15,
    -11.624187454740882, 3.2844540338774383E-14, 8.5640705530578058E-15,
    -11.618411994752934, 3.1148084284492149E-14, 7.3234096339631525E-15,
    -11.612638695404186, 2.94516282302099E-14, 6.0827487148684937E-15,
    -11.60686539605544, 2.7755172175927668E-14, 4.8420877957738396E-15,
    -11.60109209667997, 2.6058081220427502E-14, 3.6009625574729334E-15,
    -11.59531663669202, 2.4361625166145265E-14, 2.36030163837828E-15,
    -11.589543337343274, 2.2665169111863024E-14, 1.1196407192836236E-15,
    -11.583770037994526, 2.0968713057580781E-14, -1.2102019981103557E-16,
    -11.577996738619058, 1.9271622102080622E-14, -1.3621454381119386E-15,
    -11.572221278631108, 1.7575166047798381E-14, -2.6028063572065939E-15,
    -11.56644797928236, 1.587870999351614E-14, -3.84346727630125E-15,
    -11.560674679933612, 1.4182253939233903E-14, -5.0841281953959052E-15,
    -11.554901380584868, 1.2485797884951661E-14, -6.3247891144905632E-15,
    -11.549128081209398, 1.0788706929451503E-14, -7.5659143527914663E-15,
    -11.543352621221448, 9.0922508751692612E-15, -8.8065752718861227E-15,
    -11.537579321867357, 7.3956678406434369E-15, -1.0047329054822029E-14,
    -11.531805590385424, 5.6990848061176126E-15, -1.1288082837757934E-14,
    -11.526031858903492, 4.0025017715917883E-15, -1.2528836620693839E-14,
    -211.52025812741888, 2.3058552469441722E-15, -1.3769636835550369E-14,
    -10.9566374135763, -11.055434749242426, 3.18385982697094E-16,
    2.422057413846908E-14, -11.049944479303267, 3.2202280649893605E-16,
    2.3002750585393941E-14, -11.044456263337148, 3.2565826972322408E-16,
    2.1785382634096982E-14, -11.038968047371029, 3.2929373294751206E-16,
    2.0568014682800019E-14, -11.033479831404911, 3.3292919617180004E-16,
    1.9350646731503057E-14, -11.02798956146575, 3.36566019973642E-16,
    1.8132823178427921E-14, -11.022501345499633, 3.4020148319793011E-16,
    1.6915455227130962E-14, -11.017013129533513, 3.4383694642221804E-16,
    1.5698087275834E-14, -11.011524913567394, 3.47472409646506E-16,
    1.448071932453704E-14, -11.006034643628235, 3.5110923344834796E-16,
    1.32628957714619E-14, -11.000546427662115, 3.5474469667263604E-16,
    1.2045527820164942E-14, -10.995058211695996, 3.5838015989692407E-16,
    1.082815986886798E-14, -10.989569995729878, 3.62015623121212E-16,
    9.6107919175710189E-15, -10.984081779763759, 3.656510863455E-16,
    8.3934239662740584E-15, -10.9785915098246, 3.6928791014734207E-16,
    7.17560041319892E-15, -10.973103293858481, 3.7292337337163E-16,
    5.9582324619019611E-15, -10.967614667097754, 3.7655910871142881E-16,
    4.7407733902493646E-15, -10.962126040337028, 3.8019484405122771E-16,
    3.523314318596768E-15, -10.9566374135763, 3.8383057939102636E-16,
    2.3058552469441722E-15, -210.95114858141827, 3.8746645078858057E-16,
    1.0883506151137582E-15, 1.9794217062498844E-14, -11.055434749242426,
    -2.4118514512179124E-14, 1.8772525483229166E-14, -11.049944479303267,
    -2.3543416695735494E-14, 1.7751216130513647E-14, -11.044456263337148,
    -2.296853402999012E-14, 1.6729906777798125E-14, -11.038968047371029,
    -2.2393651364244736E-14, 1.5708597425082606E-14, -11.033479831404911,
    -2.1818768698499359E-14, 1.4686905845812924E-14, -11.02798956146575,
    -2.1243670882055729E-14, 1.3665596493097405E-14, -11.022501345499633,
    -2.0668788216310355E-14, 1.2644287140381886E-14, -11.017013129533513,
    -2.0093905550564977E-14, 1.1622977787666365E-14, -11.011524913567394,
    -1.9519022884819603E-14, 1.0601286208396685E-14, -11.006034643628235,
    -1.8943925068375967E-14, 9.5799768556811661E-15, -11.000546427662115,
    -1.836904240263059E-14, 8.558667502965644E-15, -10.995058211695996,
    -1.7794159736885216E-14, 7.5373581502501251E-15, -10.989569995729878,
    -1.7219277071139841E-14, 6.5160487975346046E-15, -10.984081779763759,
    -1.6644394405394464E-14, 5.4943572182649258E-15, -10.9785915098246,
    -1.6069296588950831E-14, 4.4730478655494045E-15, -10.973103293858481,
    -1.549441392320545E-14, 3.4516620675230531E-15, -10.967614667097754,
    -1.4919488227320426E-14, 2.4302762694967008E-15, -10.962126040337028,
    -1.4344562531435398E-14, 1.4088904714703488E-15, -10.9566374135763,
    -1.3769636835550372E-14, 3.8746645078858067E-16, -210.95114858141827,
    -1.3194689624595517E-14, 3.2635711344048686E-14, 1.0554326812761294E-14,
    -11.055434749242426, 3.0974862835721548E-14, 9.30403145958953E-15,
    -11.049944479303267, 2.9314635669896587E-14, 8.0542038562693079E-15,
    -11.044456263337148, 2.7654408504071626E-14, 6.8043762529490818E-15,
    -11.038968047371029, 2.5994181338246668E-14, 5.55454864962886E-15,
    -11.033479831404911, 2.4333332829919524E-14, 4.3042532964570914E-15,
    -11.02798956146575, 2.2673105664094563E-14, 3.05442569313687E-15,
    -11.022501345499633, 2.1012878498269605E-14, 1.8045980898166456E-15,
    -11.017013129533513, 1.935265133244464E-14, 5.547704864964184E-16,
    -11.011524913567394, 1.7691802824117502E-14, -6.9552486667534652E-16,
    -11.006034643628235, 1.6031575658292541E-14, -1.9453524699955697E-15,
    -11.000546427662115, 1.437134849246758E-14, -3.1951800733157942E-15,
    -10.995058211695996, 1.2711121326642622E-14, -4.4450076766360175E-15,
    -10.989569995729878, 1.1050894160817661E-14, -5.6948352799562436E-15,
    -10.984081779763759, 9.3900456524905234E-15, -6.9451306331280078E-15,
    -10.9785915098246, 7.7298184866655608E-15, -8.1949582364482323E-15,
    -10.973103293858481, 6.0694670523401647E-15, -9.4448793897387646E-15,
    -10.967614667097754, 4.4091156180147686E-15, -1.0694800543029299E-14,
    -10.962126040337028, 2.7487641836893721E-15, -1.1944721696319829E-14,
    -10.9566374135763, 1.088350615113758E-15, -1.3194689624595515E-14,
    -210.95114858141827 };

   double dv[720] = { 1.0, 0.0, 0.0, 0.016, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.016, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.016, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
    0.0, 0.032, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.032, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.032, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.048, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.048, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.048, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0,
    0.064, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.064, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.064, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.08,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.08, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.096, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.096, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.096, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 0.0, 0.0, 0.112, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.112, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.112, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.128, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.128, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.128, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 1.0, 0.0, 0.0, 0.144, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.144, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.144, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.16, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.16, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.16, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0,
    0.0, 0.0, 0.176, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.176, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.176, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.192, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.192, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.192, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
    0.0, 0.208, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.208, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.208, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.224, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.224, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.224, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0,
    0.24, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.24, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.24,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.256, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.256,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.256, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.272, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.272, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.272, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 0.0, 0.0, 0.288, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.288, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.288, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.304, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.304, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.304, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 1.0, 0.0, 0.0, 0.32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.32, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.32, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

   double B[9] = { 0.6, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.6};
  /* double B[9] = { 0.68, 0.0, 0.0, 0.0, 0.332, 0.0, 0.0, 0.0, 0.394
  }; */

   signed char A[400] = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

  double IGA[7200];
  double A_data[3600];
  double a[3600];
  double b_del_lam[240];
  double result[240];
  double varargin_1_data[240];
  double Bineq[120];
  double GAMMA[120];
  double del_lam[120];
  double esig[120];
  double ftol[120];
  double igr2[120];
  double ilam[120];
  double lam[120];
  double mesil[120];
  double H[60];
  double del_z[60];
  double u_max[60];
  double absxk;
  double mu;
  double scale;
  double t;
  int a_tmp;
  int b_i;
  int b_i1;
  int exitflag;
  int i;
  int i1;
  int idx;
  int iter;
  int iy;
  int j2;
  short b_Aineq[7200];
  signed char Aineq[7200];
  signed char At[7200];
  unsigned char ii_data[240];
  boolean_T x[240];

  //  Set Constants - Extract variables from Structure MPCParams
  //  MPCParams=load('MPCParams.mat');
  // MPCParams.X;
  // MPCParams.A;
  // MPCParams.B;
  // MPCParams.P1;
  // MPCParams.PSI;
  // MPCParams.OMEGA;
  // MPCParams.PHI;
  // MPCParams.L1;
  // MPCParams.L2;
  // MPCParams.L3;
  // MPCParams.L4;
  // MPCParams.Fmaxx;
  // MPCParams.Fmaxy;
  // MPCParams.Fmaxz;
  //  MPCParams.ConvThresh;
  // MPCParams.rCollAvoid;
  // MPCParams.maxiter;
  // MPCParams.tol;
  // MPCParams.ObstAvoid;
  // MPCParams.aObst; % Obstacle semi-major axis
  // MPCParams.bObst; % Obstacle semi-minor axis
  // MPCParams.alphaObst; % Orientation of ellipse
  // P_debris = MPCParams.P_debris;
  // MPCParams.pos_of_debris;
  //  MPCParams.AppCone;
  //  MPCParams.phi;
  //  sampT = MPCParams.Ts;
  //  Set Decision Variables - Configure constraints
  //  Compute distance from target
  scale = 3.3121686421112381E-170;
  absxk = std::abs(x0[0]);
  if (absxk > 3.3121686421112381E-170) {
    dr = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    dr = t * t;
  }

  absxk = std::abs(x0[1]);
  if (absxk > scale) {
    t = scale / absxk;
    dr = dr * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    dr += t * t;
  }

  dr = scale * std::sqrt(dr);

  //  position tolerance to stop control action
  //  Define vectors for Obstacle Avoidance
  //  to current position
  //  to obstacle
  //  to final position
  //  from obstacle to target
  //  from obstacle to FSS
  //  Angle between unit vectors
  //  Approach Cone variables
  //  point where cone is placed
  //  orientation of cone (inertial)
  //  Unit vector of approach axis
  //  Angle between FSS and approach axis
  dock_flag = 0.0;
  CollAvoid_flag = 0.0;

  //  Set obstacle avoidance flag
  //  Define target point for current iteration
  //      xfinal = xfinal;
  //  to final position
  for (i = 0; i < 6; i++) {
    target_state[i] = 0.0;
  }

  //  Define and Solve QP
  //  dock_complete = 0;
  pt_sel = 0.0;
  dock_complete = 0.0;

  //  Compute QP Matrices
  // Dimension of GAMMA is n*horizon x n
  // for i = 1:horizon
  for (b_i = 0; b_i < 120; b_i++) {
    t = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      t += 2.0 * x0[i1] * dv[i1 + 6 * b_i];
    }

    GAMMA[b_i] = t;
  }

  for (b_i = 0; b_i < 120; b_i++) {
    t = 0.0;
    for (i1 = 0; i1 < 120; i1++) {
      t += GAMMA[i1] * b[i1 + 120 * b_i];
    }

    Bineq[b_i] = t;
  }

  for (b_i = 0; b_i < 60; b_i++) {
    t = 0.0;
    for (i1 = 0; i1 < 120; i1++) {
      t += Bineq[i1] * b_b[i1 + 120 * b_i];
    }

    H[b_i] = t;
  }

  //  Control Consraints
  i = -1;
  for (idx = 0; idx < 20; idx++) {
    for (j2 = 0; j2 < 3; j2++) {
      t = B[3 * j2];
      scale = B[3 * j2 + 1];
      absxk = B[3 * j2 + 2];
      for (b_i1 = 0; b_i1 < 20; b_i1++) {
        i++;
        a_tmp = A[b_i1 + 20 * idx];
        a[i] = static_cast<double>(a_tmp) * t;
        i++;
        a[i] = static_cast<double>(a_tmp) * scale;
        i++;
        a[i] = static_cast<double>(a_tmp) * absxk;
      }
    }
  }

  for (b_i = 0; b_i < 60; b_i++) {
    t = 0.0;
    for (i1 = 0; i1 < 60; i1++) {
      t += a[b_i + 60 * i1];
    }

    u_max[b_i] = t;
  }

  std::memset(&Aineq[0], 0, 7200U * sizeof(signed char));
  std::memset(&a[0], 0, 3600U * sizeof(double));
  for (i = 0; i < 60; i++) {
    a[i + 60 * i] = 1.0;
  }

  for (b_i = 0; b_i < 60; b_i++) {
    for (i1 = 0; i1 < 60; i1++) {
      idx = static_cast<int>(a[i1 + 60 * b_i]);
      i = i1 + 120 * b_i;
      Aineq[i] = static_cast<signed char>(-idx);
      Aineq[i + 60] = static_cast<signed char>(idx);
    }
  }

  std::memset(&Bineq[0], 0, 120U * sizeof(double));

  //  Control Constraints only
  //      Aineq = Aineq;
  //      Bineq = Bineq; % Joh and me comment this on spet 23
  //  Control Constraints only
  //      Aineq = Aineq;
  //      Bineq = Bineq;
  //  Call QP Solver
  for (i = 0; i < 60; i++) {
    t = u_max[i];
    Bineq[i] = t;
    Bineq[i + 60] = t;
  }
   /* if (X_QP[0].empty()){
   for (i = 0; i < 60; i++){
      X_QP[i] = 0.0;

    }
  } */
  //  Solve quadratic programming problem using Wright's (1997) Method
  //  Minimise J(x) = 1/2x'Hx + f'x
  //  Subject to: Ax <= b
  //  Supporting Functions
  //  Reference: S. J. Wright, "Applying New Optimization Algorithms to Model
  //  Predictive Control," in Chemical Process Control-V, CACHE, AIChE
  //  Symposium, 1997, pp. 147-155.
  // Number of decision variables
  //  p = 0;
  // Test for Cold Start
  // Warm Start
  // to tune
  // to tune
  // Default Values
  mu = 10000.0;
  for (i = 0; i < 120; i++) {
    lam[i] = 100.0;
    ftol[i] = 100.0;
    esig[i] = 0.001;
    for (b_i = 0; b_i < 60; b_i++) {
      At[b_i + 60 * i] = Aineq[i + 120 * b_i];
    }
  }

  //  %Linsolve options
  //  opU.UT = true;
  //  opUT.UT = true;
  //  opUT.TRANSA = true;
  // Begin Searching
  //  for iter = 1:maxiter
  iter = 0;
  exitflag = 0;
  while ((iter <= 100) && (exitflag != 1)) {
    boolean_T exitg1;
    boolean_T y;

    // Create common matrices
    for (i = 0; i < 120; i++) {
      t = lam[i];
      scale = 1.0 / t;
      ilam[i] = scale;
      GAMMA[i] = -t / ftol[i];
      mesil[i] = mu * esig[i] * scale;
    }

    // RHS
    for (i = 0; i < 60; i++) {
      for (b_i = 0; b_i < 120; b_i++) {
        idx = b_i + 120 * i;
        IGA[idx] = GAMMA[b_i] * static_cast<double>(Aineq[idx]);
      }

      t = 0.0;
      for (b_i = 0; b_i < 60; b_i++) {
        t += b_a[i + 60 * b_i] * X_QP[b_i];
      }

      scale = 0.0;
      for (b_i = 0; b_i < 120; b_i++) {
        scale += static_cast<double>(At[i + 60 * b_i]) * lam[b_i];
      }

      u_max[i] = (t - scale) - H[i];
    }

    for (b_i = 0; b_i < 7200; b_i++) {
      b_Aineq[b_i] = static_cast<short>(-Aineq[b_i]);
    }

    for (b_i = 0; b_i < 120; b_i++) {
      t = 0.0;
      for (i1 = 0; i1 < 60; i1++) {
        t += static_cast<double>(b_Aineq[b_i + 120 * i1]) * X_QP[i1];
      }

      igr2[b_i] = GAMMA[b_i] * ((t + Bineq[b_i]) - mesil[b_i]);
    }

    // Solve
    for (b_i = 0; b_i < 60; b_i++) {
      for (i1 = 0; i1 < 60; i1++) {
        t = 0.0;
        for (idx = 0; idx < 120; idx++) {
          t += static_cast<double>(At[b_i + 60 * idx]) * IGA[idx + 120 * i1];
        }

        a[b_i + 60 * i1] = t;
      }
    }

    for (b_i = 0; b_i < 3600; b_i++) {
      A_data[b_i] = b_H[b_i] - a[b_i];
    }

    a_tmp = -1;
    idx = 0;
    exitg1 = false;
    while ((!exitg1) && (idx < 60)) {
      int idxA1j;
      int idxAjj;
      int ix;
      idxA1j = idx * 60;
      idxAjj = idxA1j + idx;
      scale = 0.0;
      if (idx >= 1) {
        ix = idxA1j;
        iy = idxA1j;
        for (i = 0; i < idx; i++) {
          scale += A_data[ix] * A_data[iy];
          ix++;
          iy++;
        }
      }

      scale = A_data[idxAjj] - scale;
      if (scale > 0.0) {
        scale = std::sqrt(scale);
        A_data[idxAjj] = scale;
        if (idx + 1 < 60) {
          int idxAjjp1;
          i = idxA1j + 61;
          idxAjjp1 = idxAjj + 61;
          if (idx != 0) {
            iy = idxAjj + 60;
            b_i = (idxA1j + 60 * (58 - idx)) + 61;
            for (j2 = i; j2 <= b_i; j2 += 60) {
              ix = idxA1j;
              absxk = 0.0;
              i1 = (j2 + idx) - 1;
              for (b_i1 = j2; b_i1 <= i1; b_i1++) {
                absxk += A_data[b_i1 - 1] * A_data[ix];
                ix++;
              }

              A_data[iy] += -absxk;
              iy += 60;
            }
          }

          scale = 1.0 / scale;
          b_i = (idxAjj + 60 * (58 - idx)) + 61;
          for (i = idxAjjp1; i <= b_i; i += 60) {
            A_data[i - 1] *= scale;
          }
        }

        idx++;
      } else {
        a_tmp = idx;
        exitg1 = true;
      }
    }

    // [R] = chol(H-At*IGA);
    if (a_tmp + 1 == 0) {
      for (b_i = 0; b_i < 60; b_i++) {
        t = 0.0;
        for (i1 = 0; i1 < 120; i1++) {
          t += static_cast<double>(At[b_i + 60 * i1]) * igr2[i1];
        }

        del_z[b_i] = u_max[b_i] - t;
      }

      for (b_i = 0; b_i < 3600; b_i++) {
        a[b_i] = b_H[b_i] - a[b_i];
      }

      mldivide(a, del_z);

      // old method (LU?)
      //   del_z = linsolve (R, linsolve (R, (r1-At*igr2), opUT), opU); %exploit matrix properties for solving 
    } else {
      // Not Positive Definite (problem? eg infeasible)
      for (b_i = 0; b_i < 60; b_i++) {
        t = 0.0;
        for (i1 = 0; i1 < 120; i1++) {
          t += static_cast<double>(At[b_i + 60 * i1]) * igr2[i1];
        }

        del_z[b_i] = u_max[b_i] - t;
      }

      for (b_i = 0; b_i < 3600; b_i++) {
        a[b_i] = b_H[b_i] - a[b_i];
      }

      mldivide(a, del_z);

      // old method (LU?)
    }

    // Decide on suitable alpha (from Wright's paper)
    // Try Max Increment (alpha = 1)
    // Check lam and ftol > 0
    for (i = 0; i < 120; i++) {
      t = 0.0;
      for (b_i = 0; b_i < 60; b_i++) {
        t += IGA[i + 120 * b_i] * del_z[b_i];
      }

      t = igr2[i] - t;
      del_lam[i] = t;
      scale = ftol[i];
      absxk = (-scale + mesil[i]) - ilam[i] * scale * t;
      mesil[i] = absxk;
      t += lam[i];
      GAMMA[i] = t;
      scale += absxk;
      ilam[i] = scale;
      x[i] = (t < 2.2204460492503131E-16);
      x[i + 120] = (scale < 2.2204460492503131E-16);
    }

    y = false;
    i = 0;
    exitg1 = false;
    while ((!exitg1) && (i < 240)) {
      if (!x[i]) {
        i++;
      } else {
        y = true;
        exitg1 = true;
      }
    }

    if (!y) {
      // KKT met
      std::memcpy(&lam[0], &GAMMA[0], 120U * sizeof(double));
      std::memcpy(&ftol[0], &ilam[0], 120U * sizeof(double));
      for (b_i = 0; b_i < 60; b_i++) {
        X_QP[b_i] += del_z[b_i];
      }
    } else {
      // KKT failed - solve by finding minimum ratio
      for (b_i = 0; b_i < 120; b_i++) {
        result[b_i] = GAMMA[b_i];
        result[b_i + 120] = ilam[b_i];
      }

      idx = 0;
      i = 0;
      exitg1 = false;
      while ((!exitg1) && (i < 240)) {
        if (result[i] < 2.2204460492503131E-16) {
          idx++;
          ii_data[idx - 1] = static_cast<unsigned char>(i + 1);
          if (idx >= 240) {
            exitg1 = true;
          } else {
            i++;
          }
        } else {
          i++;
        }
      }

      if (1 > idx) {
        j2 = 0;
      } else {
        j2 = idx;
      }

      // detects elements breaking KKT condition
      for (b_i = 0; b_i < 120; b_i++) {
        b_del_lam[b_i] = del_lam[b_i];
        b_del_lam[b_i + 120] = mesil[b_i];
      }

      for (b_i = 0; b_i < j2; b_i++) {
        i = ii_data[b_i] - 1;
        varargin_1_data[b_i] = 1.0 - result[i] / b_del_lam[i];
      }

      if (j2 <= 2) {
        if (j2 == 1) {
          scale = varargin_1_data[0];
        } else if ((varargin_1_data[0] > varargin_1_data[1]) || (
                    (varargin_1_data[0]) && (!rtIsNaN(varargin_1_data[1])))) {
          scale = varargin_1_data[1];
        } else {
          scale = varargin_1_data[0];
        }
      } else {
        if (!rtIsNaN(varargin_1_data[0])) {
          idx = 1;
        } else {
          idx = 0;
          i = 2;
          exitg1 = false;
          while ((!exitg1) && (i <= j2)) {
            if (!rtIsNaN(varargin_1_data[i - 1])) {
              idx = i;
              exitg1 = true;
            } else {
              i++;
            }
          }
        }

        if (idx == 0) {
          scale = varargin_1_data[0];
        } else {
          scale = varargin_1_data[idx - 1];
          b_i = idx + 1;
          for (i = b_i; i <= j2; i++) {
            t = varargin_1_data[i - 1];
            if (scale > t) {
              scale = t;
            }
          }
        }
      }

      scale *= 0.995;

      // solves for min ratio (max value of alpha allowed)
      // Increment
      for (b_i = 0; b_i < 120; b_i++) {
        lam[b_i] += scale * del_lam[b_i];
        ftol[b_i] += scale * mesil[b_i];
      }

      for (b_i = 0; b_i < 60; b_i++) {
        X_QP[b_i] += scale * del_z[b_i];
      }
    }

    // Complimentary Gap
    absxk = mu;
    scale = 0.0;
    for (b_i = 0; b_i < 120; b_i++) {
      scale += ftol[b_i] * lam[b_i];
    }

    mu = scale / 120.0;

    //      if(mu < tol)
    //          exitflag = 1;
    //          return
    //      end
    //      %Solve for new Sigma
    //      sigma = mu/mu_old;
    //      if(sigma > 0.1) %to tune
    //          sigma = 0.1;
    //      end
    //      esig = sigma*ones(mc,1);
    if (mu < 0.001) {
      exitflag = 1;
    } else {
      // Solve for new Sigma
      scale = mu / absxk;
      if (scale > 0.1) {
        // to tune
        scale = 0.1;
      }

      for (i = 0; i < 120; i++) {
        esig[i] = scale;
      }
    }

    iter++;
  }

  // Check for failure
  num_iter = iter;

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

