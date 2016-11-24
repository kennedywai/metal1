/*
 * =====================================================================================
 *
 *       Filename:  mega_base_MIMO_andbot1dot2.ino
 *
 *    Description:  The program is for wheel hub motor control (both left & right).
 *                  Vq max/min is 1000/-1000
 *                  Id max/min is 20/-20 A
 *
 *                  [HW Arduino Mega 2560]
 *                  Serial port (Default serial for Connect ROSSerial )
 *                  Serial1 port (connect to Motor control board Right wheel)
 *                  Serial2 port (connect to Motor control board Left wheel)
 *                  Serial3 port (connect to BT (Test only))
 *
 *        Version:  20161019
 *        Created:
 *       Revision:  none
 *       Compiler:
 *
 *         Author:
 *        Company:  AR
 *
 * =====================================================================================
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <WheelCmd.h>
#include <WheelFb.h>
#include <DriverState.h>
#include <Metro.h>
#include "BLDCMotor.h"
#include "mech_param.h"

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

//#define LOOPTIME 10//40//100
int LOOPTIME = 10;
#define PUBLISHTIME 40
//#define rate 25
float rate;
unsigned long lastMilli = 0;
unsigned long PrePubMilli = 0;
long testMilli = 0;
long dT = 0;

BLDCMotor::SetdqCmdLimit DDWheelInputLimit = {Volt_MAX,Volt_MIN,Vq_MAX,Vq_MIN};
BLDCMotor::SetMotorParam DDWheelParam = {CPR,gear_ratio,MAXAngularSpeed};//MotorDefault;//CPR,gear_ratio,MAXAngularSpeed};
BLDCMotor::SetENCPin DDWheelENCAxisleft = {enc_pinA_left,enc_pinB_left,enc_pinC_left};
BLDCMotor::SetENCPin DDWheelENCAxisright = {enc_pinA_right,enc_pinB_right,enc_pinC_right};
BLDCMotor::SetCtrlParam DDWheelCtrlAxisleft = {Axis_left, dqCmdMode};
BLDCMotor::SetCtrlParam DDWheelCtrlAxisright = {Axis_right, dqCmdMode};

// robot
BLDCMotor Wheel_left(&DDWheelENCAxisleft,&DDWheelParam,&DDWheelCtrlAxisleft,&DDWheelInputLimit);
BLDCMotor Wheel_right(&DDWheelENCAxisright,&DDWheelParam,&DDWheelCtrlAxisright,&DDWheelInputLimit);


// variables for MIMO closed loop control
double vel_ref = 0.0;
double vel_fb = 0.0;
double omega_ref = 0.0;
double omega_fb = 0.0;
double u_sum = 0.0;
double u_diff = 0.0;
double omega_fb_right = 0.0;
double omega_fb_left = 0.0;

// variables for controllers
double sum_error_vel = 0.0;
double sum_error_omega = 0.0;

//declarations
void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
void DriverState_service_callback(const andbot1dot2::DriverStateRequest& req, andbot1dot2::DriverStateResponse& res);
void WheelCmdModeCallback(const andbot1dot2::WheelCmd &WheelCmd);
void LoopRateCallback(const std_msgs::Float32 &rate);

/* ************  declarations for ROS usages *****************************************/

/*  define  ROS node and topics */
ros::NodeHandle nh;
andbot1dot2::WheelFb WheelFb_msgs;
andbot1dot2::WheelCmd WheelCmd_msgs;
geometry_msgs::Twist VelFb_msgs;
std_msgs::Int16 ENC_msgs;
std_msgs::Int16 ENCPre_msgs;
std_msgs::Float64 elecVel_msgs;
ros::Publisher feedback_wheel_angularVel_pub("andbot1dot2/feedback_wheel_angularVel",&WheelFb_msgs);
ros::Publisher feedbackVel_pub("andbot1dot2/feedbackVel",&VelFb_msgs);
ros::Publisher cmd_wheel_volt_pub("andbot1dot2/cmd_wheel_volt", &WheelCmd_msgs);
ros::Publisher ENC_raw_pub("ENC_raw",&ENC_msgs);
ros::Publisher ENCPre_pub("ENCPre_raw",&ENCPre_msgs);
ros::Publisher elecVel_pub("elecVel",&elecVel_msgs);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/andbot1dot2/cmd_vel", cmd_velCallback);
ros::Subscriber<andbot1dot2::WheelCmd> WheelCmd_sub("andbot1dot2/WheelInput", WheelCmdModeCallback);
ros::Subscriber<std_msgs::Float32> LoopRate_sub("andbot1dot2/base_mega_LoopRate", LoopRateCallback);
ros::ServiceServer<andbot1dot2::DriverStateRequest, andbot1dot2::DriverStateResponse> service("DriverState_service", &DriverState_service_callback);

/* ************  End of declarations for ROS usages ****************/
void LoopRateCallback(const std_msgs::Float32 &rate)
{
	LOOPTIME = (double)1/(rate.data);
}

void WheelCmdModeCallback(const andbot1dot2::WheelCmd &WheelCmd)
{
	Wheel_left.CmdRef.VoltCmd = WheelCmd.speed1;
	Wheel_right.CmdRef.VoltCmd = WheelCmd.speed2;

	// for publish
	WheelCmd_msgs.speed1 = WheelCmd.speed1 ;
	WheelCmd_msgs.speed2 = WheelCmd.speed2;

	Wheel_left.SendCmd();
	Wheel_right.SendCmd();
}

void DriverState_service_callback(const andbot1dot2::DriverStateRequest& req, andbot1dot2::DriverStateResponse& res)
{
    driverEn = req.driverstate;
    if (driverEn == true)
    {
      res.driverstate = true;
      Wheel_left.Enable();
      Wheel_right.Enable();
    }
    else
    {
      res.driverstate = false;
      Wheel_left.Disable();
      Wheel_right.Disable();
    }
    Serial.print("From Client");
    Serial.println(req.driverstate,DEC);
    Serial.print("Server says");
    Serial.print(res.driverstate,DEC);
}
double vel_controller(double targetValue, double currentValue)
{
  //static double last_error = 0;
  //long dT = 1/rate;
  double error;
  double iTerm;
  double iTerm_Umax = 6;
  double iTerm_Umin = -6;
  double pidTerm ;
  //double sum_error ;
  // PI control
  double Kp = 14.9624;
  double Ki = 21.3962;

  error = targetValue - currentValue;

  sum_error_vel = sum_error_vel + error * dT;//(1/rate);
  iTerm = Ki * sum_error_vel;

  if (iTerm >= iTerm_Umax)        	iTerm = iTerm_Umax;
  else if (iTerm<= iTerm_Umin)		iTerm = iTerm_Umin;

  pidTerm = Kp * error + iTerm;

  //saturation protection
//  if (pidTerm >= 10)        	constrained_pidTerm = 10;
//  else if (pidTerm <= -10)  	constrained_pidTerm = -10;
//  else 	                        constrained_pidTerm = pidTerm ;

  return pidTerm;
}
double omega_controller(double targetValue, double currentValue)
{
  //static double last_error = 0;
  //long dT = 1/rate;
  double error;
  double iTerm;
  double iTerm_Umax = 6;
  double iTerm_Umin = -6;
  double pidTerm;
  //double sum_error;
  double Kp = 4.4157;
  double Ki = 5.8287;

  error = targetValue - currentValue;

  sum_error_omega = sum_error_omega + error * dT;//(1/rate);
  iTerm = Ki * sum_error_omega;

  if (iTerm >= iTerm_Umax)        	iTerm = iTerm_Umax;
  else if (iTerm<= iTerm_Umin)		iTerm = iTerm_Umin;

  pidTerm = Kp * error + iTerm;

//  if (pidTerm >= 10)  		constrained_pidTerm = 10;
//  else if (pidTerm <= -10) 	constrained_pidTerm = -10;
//  else  	                constrained_pidTerm = pidTerm ;

  return pidTerm;
}
// MIMO control loop
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  double u_left = 0.0;
  double u_right = 0.0;
  double volt_friction_compensation = 0.7;
  double FeedForward_vel = 6.313/1.054; // from model
  double FeedForward_omega = 4.9887/3.2393; //from model

  vel_ref = twist_aux.linear.x;
  omega_ref = twist_aux.angular.z;

  u_sum = vel_controller(vel_ref,vel_fb) + vel_ref * FeedForward_vel;
  u_diff = omega_controller(omega_ref, omega_fb) + omega_ref * FeedForward_omega ;

//  if(u_sum >= Umax_volt)              u_sum = Umax_volt;
//  else if(u_sum <= Umin_volt) u_sum = Umin_volt;
//  else;
//
//  if(u_diff >= Umax_volt)             u_diff = Umax_volt;
//  else if(u_diff <= Umin_volt)        u_diff = Umin_volt;
//  else;

  u_right = (u_sum + u_diff) / 2 ;
  u_left = (u_sum - u_diff) / 2 ;

  // friction compensation
  if (vel_ref != 0.0 || omega_ref != 0.0)
  {
        if (u_right < 0.0)      u_right = u_right - volt_friction_compensation;
        else                            u_right = u_right + volt_friction_compensation;

        if (u_left < 0.0)       u_left = u_left - volt_friction_compensation;
        else                            u_left = u_left + volt_friction_compensation;
  }
  else;

  WheelCmd_msgs.speed1 = u_left ;
  WheelCmd_msgs.speed2 = u_right;

  Wheel_left.CmdRef.VoltCmd = u_left;
  Wheel_right.CmdRef.VoltCmd = u_right;
}
void FbVelCal(const andbot1dot2::WheelFb &wheel)
{
  omega_fb_left = wheel.speed1;
  omega_fb_right = wheel.speed2;

  // for publish
  WheelFb_msgs.speed1 = Wheel_left.FbMotorInfo.AngularVel;
  WheelFb_msgs.speed2 = Wheel_right.FbMotorInfo.AngularVel;

  //mobile robot kinematics transformation: differential drive
  vel_fb = double(wheelRadius) / 2 * (omega_fb_right + omega_fb_left);
  omega_fb = double(wheelRadius) / double(wheelSeparation) * (omega_fb_right - omega_fb_left);

  VelFb_msgs.linear.x = vel_fb;
  VelFb_msgs.angular.z = omega_fb;
}
void setup(){

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(cmd_wheel_volt_pub);
    nh.advertise(feedbackVel_pub);
    nh.advertise(feedback_wheel_angularVel_pub);
    nh.advertise(ENC_raw_pub);
    nh.advertise(ENCPre_pub);
    nh.advertise(elecVel_pub);
    nh.subscribe(cmd_vel_sub);
    nh.subscribe(WheelCmd_sub);
    nh.subscribe(LoopRate_sub);
    nh.advertiseService(service);
//
//
    Serial2.begin(115200);
    Serial3.begin(115200);
//
//    pinMode(51, OUTPUT);                                                           //for VD_ENABLE_VALUE check
//
    Wheel_left.Init(); //left wheel
    Wheel_right.Init();//right wheel
//
	attachInterrupt(0, ISRENCleft, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
	attachInterrupt(1, ISRENCleft, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
	attachInterrupt(2, ISRENCleft, CHANGE);
	attachInterrupt(3, ISRENCright, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
	attachInterrupt(4, ISRENCright, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
	attachInterrupt(5, ISRENCright, CHANGE);
}

void loop()
{
    if ((dT = millis() - lastMilli) >= LOOPTIME)
    {
        lastMilli = millis();

        Wheel_left.GetMotorData(dT,ENCOutputMode);
        Wheel_right.GetMotorData(dT,ENCOutputMode);

        FbVelCal(WheelFb_msgs);
        ENC_msgs.data = dT;//Wheel_right.curEncoderpos;
        ENCPre_msgs.data = Wheel_right.EncoderposPre;
        elecVel_msgs.data = double(Wheel_left.SerialGetDriverData(Axis_left)) * 1.0472 * double(10) / double(30); //rad/s
        Wheel_left.SendCmd();
        Wheel_right.SendCmd();
    }
    else;

    if ((millis() - PrePubMilli)>= PUBLISHTIME)
    {
    	PrePubMilli = millis();
    	cmd_wheel_volt_pub.publish(&WheelCmd_msgs);
    	feedbackVel_pub.publish(&VelFb_msgs);
    	feedback_wheel_angularVel_pub.publish(&WheelFb_msgs);
    	ENC_raw_pub.publish(&ENC_msgs);
    	ENCPre_pub.publish(&ENCPre_msgs);
    	//elecVel_pub.publish(&elecVel_msgs);
    }
    else;

//    Serial.println(String("head=") + " " + \
//	String(testhead) + " " + \
//	String("bottom=") + " " + \
//	String(test) + " " + \
//	String("diff=") + " " + \
//	String(testMilli));
    nh.spinOnce();
}

void ISRENCleft()
{
	Wheel_left.doEncoder();
}
void ISRENCright()
{
	Wheel_right.doEncoder();
}
