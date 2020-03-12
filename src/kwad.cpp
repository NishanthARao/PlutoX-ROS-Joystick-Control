//##########################################################################################################################################
#include <iostream>
#include <cmath>
#include <cstdlib>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/SetModelState.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "chrono"
//##########################################################################################################################################
using namespace std;

float x_ddot, y_ddot, z_ddot, x_dot, y_dot, z_dot, x, y, z;
float phi_dot, theta_dot, psi_dot, phi, theta, psi, p, q, r, p_dot, q_dot, r_dot;

float u2, u3, u4;
float o1, o2, o3, o4, o;

float X_, Y_, Z_;
float x_bf, y_bf, z_bf, x_bf_dot, y_bf_dot, z_bf_dot;

float x_error, x_error_sum, y_error, y_error_sum;
float cp, ci, cd;

float z_error, z_error_sum;
float phi_error,phi_error_sum, p_des;
float theta_error, theta_error_sum, q_des;
float psi_error, psi_error_sum, r_des;

float p_error, p_error_sum;
float q_error, q_error_sum;
float r_error, r_error_sum;

float w1, w2, w3, w4;
//##########################################################################################################################################
bool flag = 0;
bool pos_hold_flag = 0;

#define Ts 0.01
#define g 9.81
#define m 1.4
#define l 0.56
#define t 0.02

#define kd 0.0000013858
#define kdx 0.16481
#define kdy 0.31892
#define kdz 0.0000011

#define jx 0.05
#define jy 0.05
#define jz 0.24
#define kt 0.000013328
#define jp 0.044

#define max_motor_speed 925
#define min_motor_speed 0
#define max_motor_speed_2 855625
#define min_motor_speed_2 0

#define u1_max 43.5
#define u1_min 0
#define u2_max 6.25
#define u2_min -6.25
#define u3_max 6.25
#define u3_min -6.25
#define u4_max 2.25
#define u4_min -2.25

#define x_kp 0.35
#define x_ki 0.25
#define x_kd -0.35
#define x_ki_lim 0.25

#define y_kp 0.35
#define y_ki 0.25
#define y_kd -0.35
#define y_ki_lim 0.25

#define z_kp 5.88
#define z_ki 0
#define z_kd -5.05
#define z_ki_lim 0.25

#define phi_kp 4.5
#define phi_ki 0
#define phi_kd 0
#define phi_max 0.7853
#define phi_ki_lim 0.0349

#define theta_kp 4.5
#define theta_ki 0
#define theta_kd 0
#define theta_max 0.7853
#define theta_ki_lim 0.0349

#define psi_kp 4.5
#define psi_ki 0
#define psi_kd 0
#define psi_max 10
#define psi_ki_lim 0.1396

#define p_kp 2.7
#define p_ki 1
#define p_kd -0.01
#define p_max 0.87266
#define p_ki_lim 0.174532

#define q_kp 2.7
#define q_ki 1
#define q_kd -0.01
#define q_max 0.87266
#define q_ki_lim 0.174532

#define r_kp 2.7
#define r_ki 1
#define r_kd -0.01
#define r_max 0.87266
#define r_ki_lim 0.174532

#define R2D 57.295779513
#define D2R 0.017453293

float x_des = 1.0;
float y_des = 1.0;
float z_des = 1.0;
float phi_des = 0.0;
float theta_des = 0.0;
float psi_des = 0.0;
float u1 = 13.734;

auto t2 = chrono::high_resolution_clock::now();

ros::Publisher velPub;
ros::ServiceClient stateClient;
gazebo_msgs::ModelStates S;
tf2::Quaternion _quat;
geometry_msgs::Quaternion quat;
geometry_msgs::Point positionObj;
geometry_msgs::Pose kwadPose;
geometry_msgs::Twist twistObj;
gazebo_msgs::ModelState kwadModelState;
gazebo_msgs::SetModelState srv;
std_msgs::Float64MultiArray f;
//##########################################################################################################################################
void quad_dynamics()
{
	x_ddot = (-(cos(phi)*sin(theta)*cos(psi) + sin(psi)*sin(phi)) * u1 - kdx*x_dot) / m;
	y_ddot = (-(cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)) * u1 - kdy*y_dot) / m;
	z_ddot = ((-(cos(phi)*cos(theta))*u1 - kdz*z_dot)/m) + g;
	
	p_dot = (q*r*(jy - jz) - jp*p*o + l*u2)/jx;
	q_dot = (p*r*(jz - jx) + jp*q*o + l*u3)/jy;
	r_dot = (p*q*(jx - jy) + u4)/jz;
	
	phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
	theta_dot = cos(phi)*q - sin(phi)*r;
	psi_dot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
	
	z_dot = z_ddot * Ts + z_dot;
	z = z_dot * Ts + z;
	if(z < 0)z = 0;
	
	x_dot = x_ddot * Ts + x_dot;
	x = x_dot * Ts + x;
	
	y_dot = y_ddot * Ts + y_dot;
	y = y_dot * Ts + y;
	
	p = p_dot * Ts + p;
	if(p > p_max)p = p_max;
	if(p < -p_max)p = -p_max;
	q = q_dot * Ts + q;
	if(q > q_max)q = q_max;
	if(q < -q_max)q = -q_max;
	r = r_dot * Ts + r;
	if(r > r_max)r = r_max;
	if(r < -r_max)r = -r_max;
	
	phi = phi_dot * Ts + phi;
	if(phi > phi_max)phi = phi_max;
	if(phi < -phi_max)phi = -phi_max;
	theta = theta_dot * Ts + theta;
	if(theta > theta_max)theta = theta_max;
	if(theta < -theta_max)theta = -theta_max;
	
	
	psi = psi_dot * Ts + psi;
	if(psi > psi_max)psi = psi_max;
	if(psi < -psi_max)psi = -psi_max;
	
}
//##########################################################################################################################################
void rotateGFtoBF(float X, float Y, float Z, float PHI,float THETA,float PSI)
{
	X_ = cos(PSI)*cos(THETA)*X + sin(PSI)*cos(THETA)*Y - sin(THETA)*Z;
	Y_ = (cos(PSI)*sin(PHI)*sin(THETA) - cos(PHI)*sin(PSI))*X + (sin(PHI)*sin(PSI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(THETA)*sin(PHI))*Z;
	Z_ = (cos(PHI)*cos(PSI)*sin(THETA) + sin(PHI)*sin(PSI))*X + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Y + (cos(PHI)*cos(THETA))*Z;
}
//###########################################################################################################################################
void PID_position()
{
	if(flag == 0)
	{
		x_error_sum = 0;
		y_error_sum = 0;
		z_error_sum = 0;
	}
	rotateGFtoBF(x_des, y_des, z_des, 0, 0, psi);
	x_des = X_;y_des = Y_;z_des = Z_;
	rotateGFtoBF(x, y, z, phi, theta, psi);
	x_bf = X_;y_bf = Y_;z_bf = Z_;
	rotateGFtoBF(x_dot, y_dot, z_dot, phi, theta, psi);
	x_bf_dot = X_;y_bf_dot = Y_;z_bf_dot = Z_;
	
	x_error = x_des - x_bf;
	if(abs(x_error) < x_ki_lim)x_error_sum += x_error;
	cp = x_kp*x_error;
	ci = x_ki*Ts*x_error_sum;
	if(ci > theta_max)ci = theta_max;
	if(ci < -theta_max)ci = -theta_max;
	cd = x_kd*x_bf_dot;
	theta_des = -(cp + ci + cd);
	if(theta_des > theta_max)theta_des = theta_max;
	if(theta_des < -theta_max)theta_des = -theta_max;
	
	y_error = y_des - y_bf;
	if(abs(y_error) < y_ki_lim)y_error_sum += y_error;
	cp = y_kp*y_error;
	ci = y_ki*Ts*y_error_sum;
	if(ci > phi_max)ci = phi_max;
	if(ci < -phi_max)ci = -phi_max;
	cd = y_kd*y_bf_dot;
	phi_des = cp + ci + cd;
	if(phi_des > phi_max)phi_des = phi_max;
	if(phi_des < -phi_max)phi_des = -phi_max;

	z_error = z_des - z_bf;
	if(abs(z_error) < z_ki_lim)z_error_sum += z_error;
	cp = z_kp*z_error;
	ci = z_ki*Ts*z_error_sum;
	if(ci > u1_max)ci = u1_max;
	if(ci < u1_min)ci = u1_min;
	cd = z_kd*z_dot;
	u1 = -(cp + ci + cd)/(cos(theta)*cos(phi)) + (m*g)/(cos(theta)*cos(phi));
	if(u1 > u1_max)u1 = u1_max;
	if(u1 < u1_min)u1 = u1_min;
	
}
//###########################################################################################################################################
void PID_attitude()
{
	if(flag == 0)
	{
		phi_error_sum = 0;
		theta_error_sum = 0;
		psi_error_sum = 0;
	}

	phi_error = phi_des - phi;
	if(abs(phi_error) < phi_ki_lim)phi_error_sum += phi_error;
	cp = phi_kp*phi_error;
	ci = phi_ki*Ts*phi_error_sum;
	if(ci > p_max)ci = p_max;
	if(ci < -p_max)ci = -p_max;
	cd = phi_kd*p;
	p_des = cp + ci + cd;
	if(p_des > p_max)p_des = p_max;
	if(p_des < -p_max)p_des = -p_max;
	
	theta_error = theta_des - theta;
	if(abs(theta_error) < theta_ki_lim)theta_error_sum += theta_error;
	cp = theta_kp*theta_error;
	ci = theta_ki*Ts*theta_error_sum;
	if(ci > q_max)ci = q_max;
	if(ci < -q_max)ci = -q_max;
	cd = theta_kd*q;
	q_des = cp + ci + cd;
	if(q_des > q_max)q_des = q_max;
	if(q_des < -q_max)q_des = -q_max;
	
	
	psi_error = psi_des - psi;
	if(abs(psi_error) < psi_ki_lim)psi_error_sum += psi_error;
	cp = psi_kp*psi_error;
	ci = psi_ki*Ts*psi_error_sum;
	if(ci > r_max)ci = r_max;
	if(ci < -r_max)ci = -r_max;
	cd = psi_kd*r;
	r_des = cp + ci + cd;
	if(r_des > r_max)r_des = r_max;
	if(r_des < -r_max)r_des = -r_max;

}
//###########################################################################################################################################
void PID_rate()
{
	if(flag == 0)
	{
		p_error_sum = 0;
		q_error_sum = 0;
		r_error_sum = 0;
		flag = 1;
	}
	
	p_error = p_des - p;
	if(abs(p_error) < p_ki_lim)p_error_sum += p_error;
	cp = p_kp*p_error;
	ci = p_ki*Ts*p_error_sum;
	if(ci > u2_max)ci = u2_max;
	if(ci < u2_min)ci = u2_min;
	cd = p_kd*p_dot;
	u2 = cp + ci + cd;
	if(u2 > u2_max)u2 = u2_max;
	if(u2 < u2_min)u2 = u2_min;
	
	q_error = q_des - q;
	if(abs(q_error) < q_ki_lim)q_error_sum += q_error;
	cp = q_kp*q_error;
	ci = q_ki*Ts*q_error_sum;
	if(ci > u3_max)ci = u3_max;
	if(ci < u3_min)ci = u3_min;
	cd = q_kd*q_dot;
	u3 = cp + ci + cd;
	if(u3 > u3_max)u3 = u3_max;
	if(u3 < u3_min)u3 = u3_min;
	
	r_error = r_des - r;
	if(abs(r_error) < r_ki_lim)r_error_sum += r_error;
	cp = r_kp*r_error;
	ci = r_ki*Ts*r_error_sum;
	if(ci > u4_max)ci = u4_max;
	if(ci < u4_min)ci = u4_min;
	cd = r_kd*r_dot;
	u4 = cp + ci + cd;
	if(u4 > u4_max)u4 = u4_max;
	if(u4 < u4_min)u4 = u4_min;
	
}
//###########################################################################################################################################
void quad_motor_speed()
{
	w1 = u1/(4*kt) + u3/(2*kt*l) + u4/(4*kd);
	w2 = u1/(4*kt) - u2/(2*kt*l) - u4/(4*kd);
	w3 = u1/(4*kt) - u3/(2*kt*l) + u4/(4*kd);
	w4 = u1/(4*kt) + u2/(2*kt*l) - u4/(4*kd);
	
	if(w1 > max_motor_speed_2)w1 = max_motor_speed_2;
	if(w1 < min_motor_speed_2)w1 = min_motor_speed_2;
	
	if(w2 > max_motor_speed_2)w2 = max_motor_speed_2;
	if(w2 < min_motor_speed_2)w2 = min_motor_speed_2;
	
	if(w3 > max_motor_speed_2)w3 = max_motor_speed_2;
	if(w3 < min_motor_speed_2)w3 = min_motor_speed_2;
	
	if(w4 > max_motor_speed_2)w4 = max_motor_speed_2;
	if(w4 < min_motor_speed_2)w4 = min_motor_speed_2;
	
	o1 = sqrt(w1);
	o2 = sqrt(w2);
	o3 = sqrt(w3);
	o4 = sqrt(w4);
	
	u1 = kt*(w1 + w2 + w3 + w4);
	u2 = kt*l*(w4 - w2);
	u3 = kt*l*(w1 - w3);
	u4 = kd*(w1 + w3 - w2 - w4);
	
	o = o1 - o2 + o3 - o4;
}
//###########################################################################################################################################
void control_kwad(const gazebo_msgs::ModelStates::ConstPtr& msg2)
{	
	
	auto t1 = chrono::high_resolution_clock::now();
	auto dt = chrono::duration_cast<chrono::milliseconds>(t1-t2).count();
	if(dt > 10)
	{
		if(pos_hold_flag == 1)PID_position();
		PID_attitude();
		PID_rate();
		quad_motor_speed();
		quad_dynamics();
		
		t2 = t1;
	}
	
	_quat.setRPY(theta, phi, psi);
	_quat.normalize();
	quat = tf2::toMsg(_quat);

	positionObj.x = y;
	positionObj.y = x;
	positionObj.z = z;
	twistObj.linear.x = y_dot;
	twistObj.linear.y = x_dot;
	twistObj.linear.z = z_dot;
	twistObj.angular.x = q;
	twistObj.angular.y = p;
	twistObj.angular.z = r;
	
	kwadPose.position = positionObj;
	kwadPose.orientation = quat;
	
	kwadModelState.model_name = (string)"Kwad";
	kwadModelState.pose = kwadPose;
	kwadModelState.twist = twistObj;
	
	/*
	cout << x << '\t';
	cout << y << '\t';
	cout << z << '\t';
	cout << phi << '\t';
	cout << theta << '\t';
	cout << psi << '\t';
	cout << dt << '\n';
	*/

	f.data = {20, -20, 20, -20};
	velPub.publish(f);
	
	srv.request.model_state = kwadModelState;
		
	if(!stateClient.call(srv))
    {
        ROS_ERROR("The setModelState Service isn't available!:%s",srv.response.status_message.c_str());
    }
	
}
//###########################################################################################################################################
void sensor_meas(const sensor_msgs::Imu::ConstPtr& msg1)
{
		
	
	x_ddot = msg1->linear_acceleration.x;
	y_ddot = msg1->linear_acceleration.y;
	z_ddot = msg1->linear_acceleration.z;

	p = msg1->angular_velocity.x;
	q = msg1->angular_velocity.y;
	r = msg1->angular_velocity.z;
	
	//std::cout << x_ddot << '\n';
}
//###########################################################################################################################################
void JoyData(const sensor_msgs::Joy::ConstPtr& msg3)
{
	if(pos_hold_flag == 0){
		u1 = (-msg3->axes[1]*2) + 13.734;
		if(u1 < 0)u1 = 0;
		phi_des = msg3->axes[3]*-10*D2R;
		theta_des = msg3->axes[2]*-10*D2R;
	}
	pos_hold_flag -= msg3->buttons[5];
	//cout << pos_hold_flag << '\n';
}
//###########################################################################################################################################
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Control");
	ros::NodeHandle nodeObj;
	
	velPub = nodeObj.advertise<std_msgs::Float64MultiArray>("/Kwad/joint_motor_controller/command", 40);
	stateClient = nodeObj.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", 1000);
	//ros::Subscriber PoseSub = nodeObj.subscribe("/gazebo/model_states", 10, control_kwad);
	ros::Subscriber ImuSub = nodeObj.subscribe("/Kwad/imu", 10, sensor_meas);
	ros::Subscriber posSub = nodeObj.subscribe("/gazebo/model_states", 10, control_kwad);
	ros::Subscriber joyData = nodeObj.subscribe("/Kwad/joy", 10, JoyData);
	
	ros::spin();
	
	return 0;
}
