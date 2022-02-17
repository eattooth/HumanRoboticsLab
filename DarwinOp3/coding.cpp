#include "Walk.hpp"
#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp> 
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp> 
#include <webots/Touchsensor.hpp>
#include <webots/Robot.hpp>
#include "time.h"
#include <cmath>   
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <chrono>

#include "stdio.h"
#include "math.h"

using namespace webots;
using namespace managers;
using namespace std;
using namespace chrono;
using namespace Robot;

// static const char *motorNames[NMOTORS] = {
// 	"ShoulderR" /*0 ID1 */, "ShoulderL" /*1 ID2 */, "ArmUpperR" /*2 ID3 */, "ArmUpperL" /*3 ID4 */, "ArmLowerR" /*4 ID5 */,
// 	"ArmLowerL" /*5 ID6 */, "PelvYR" /*6 ID7 */,    "PelvYL" /*7 ID8 */,    "PelvR" /*8 ID9 */,     "PelvL" /*9 ID10*/,
// 	"LegUpperR" /*10 ID11*/, "LegUpperL" /*11 ID12*/, "LegLowerR" /*12 ID13*/, "LegLowerL" /*13 ID14*/, "AnkleR" /*14 ID15*/,
// 	"AnkleL" /*15 ID16*/,    "FootR" /*16 ID17*/,     "FootL" /*17 ID18*/,     "Neck" /*18 ID19*/,      "Head" /*19 ID20*/
// };

static const char *motorNames[NMOTORS] = {
	"PelvYR" /*6 ID7 */,    "PelvYL" /*7 ID8 */,    "PelvR" /*8 ID9 */,     "PelvL" /*9 ID10*/,
	"LegUpperR" /*10 ID11*/, "LegUpperL" /*11 ID12*/, "LegLowerR" /*12 ID13*/, "LegLowerL" /*13 ID14*/, "AnkleR" /*14 ID15*/,
	"AnkleL" /*15 ID16*/,    "FootR" /*16 ID17*/,     "FootL" /*17 ID18*/
};

static const char *ftmotorNames[FTMOTORS] = {
	"romotorxr", "romotoryr", "romotorzr", "romotorxl", "romotoryl", "romotorzl"
};
//origin 28
#define MOTOR_P_GAIN 28
#define MOTOR_I_GAIN 0
#define MOTOR_D_GAIN 0

// #define ankleRollR  16
// #define anklePitchR 14
// #define kneePitchR  12
// #define hipPitchR   10
// #define hipRollR    8
// #define hipYawR     6
// #define hipYawL     7
// #define hipRollL    9
// #define hipPitchL   11
// #define kneePitchL  13
// #define anklePitchL 15
// #define ankleRollL  17

#define ankleRollR  10
#define anklePitchR 8
#define kneePitchR  6
#define hipPitchR   4
#define hipRollR    2
#define hipYawR     0
#define hipYawL     1
#define hipRollL    3
#define hipPitchL   5
#define kneePitchL  7
#define anklePitchL 9
#define ankleRollL  11
//////////////////////////////////////control pannel
////////////////////////////////////////////////////
#define debugPrintOn 0
#define debugFprintOn 0
int samplingTime = 1; // integer sampling time
double samplingtime = 0.001;//0.008;
int samplingPeriod = samplingTime;
int inverse_type = 1; // 0 : direct inverse from C , 1 : inverse solved from Matlab
int control_on = 0; // 1 : ZMP control , 0 : nominal value
int ps_stand = 250;//a250;//400/samplingTime;
int tq_stand = 10000;//0;//3000;//4000/samplingTime;
//////////////////////////////////////control pannel torque
//////////////////////////////////////////////////////////
#define torqueModeOn 1
#define torqueModeStand 0
#define GravityCompensation 1
#define jointCtrlOn 0
#define iCtrlOn 0
#define onlyOneMotorTorque 1
double Tr=0.015;//0;//0;//0.02;//
double Tr_walk_ssp=0.015;
double Tr_walk_dsp=0.015;
double _kv=2*(1.8/Tr);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
double _kp=(1.8/Tr)*(1.8/Tr);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
// double pointmass= 1.5;//1.0;
// double _m = 1+1.18888;//3.01;//1.1655 + pointmass; Jinu kang kang
// double _m = 1.34928+1.0502;
double _m = 1.34928+1.18888;
// double _m = 1.34928+0.23061*2+0.14807+1.18888;
int sensorOn=7;
//(=0 fz: constant)(=1 fz: constant tx,ty: measured)//(=2 fz tx ty sensorLPF live)(=3 all sensorLPF live)//(=4 fz: measured)(5=fz: measured(LPF))//(=6 fz: constant tx,ty: measured(LPF))//(=7 fz tx ty : constant)
double jointDamperGain=0.0;
double iCtrlGain=0;
double jointCtrlGain=0.4;
double jointCtrlPGain[12]={
	jointCtrlGain,jointCtrlGain,jointCtrlGain,jointCtrlGain,
	jointCtrlGain,0.0,0.0,jointCtrlGain,
	jointCtrlGain,jointCtrlGain,jointCtrlGain,jointCtrlGain};//0.7;
//////////////////////////////////////control pannel cpt
////////////////////////////////////////////////////////
#define localSSPTimeLimitOn 0
#define cpVelFilterOn 0
#define cp_err_limit_on 0
#define cpZmpCurrentOn 0
#define protectZmpEscapeOn 0
double localSSPTimeLimit=0.3;
int CpSwitch=2, cpDspSwitch=2, cptOn=0;
//Cpswitch=1->cpt on //Cpswitch=2->dcm cpt on(only ssp)
//cpDspSwitch=2->dsp cp interpolation by com cubic&euler
//cpZmpCurrentOn =0 -> zmp.cur=0 / =1 -> zmp.cur=EEfxtxty상수기반zmp
double cp_cur_limit=0.007;
double cpgain_all=0.0;
double cpgain_ssp=0.0;
double Kf_D[2] = {0.0+cpgain_all,0.0+cpgain_all}; ///CP gain 
double Kf_S[2] = {0.0+cpgain_all+cpgain_ssp,0.0+cpgain_all+cpgain_ssp}; ///CP gain 
double Kf_D_origin[2]={Kf_D[0],Kf_D[1]};
double Kf_S_origin[2]= {Kf_S[0],Kf_S[1]};
int DspInitSwitch=1; 
//0=>dsp 시작이 수식으로 결정됨, 
//1=>ssp의 끝점이 dsp시작으로 cpref는 xcom origin 그대로


double th_error=0;
double th_errorsum=0;
double th_errorsum_p=0;
double result_th[12]={0,};
int phaseflag = 0, realphaseflag = 0, sspflag = 0, stepcheck=0;
double footup = 0;
int datacountft = 0;
int walkingdatatime = 0;
double local_stride_F, local_stride_L;
double pre_local_stride_F, pre_local_stride_L;
double force[8], pos_x[8], pos_y[8], zmp[2], zmp_global[2];
double force_sum;
double sumofzmp_r, sumofzmp_l;
double foot_boundary_x = 0.127; // [cm] sagittal FSR boundary
double foot_boundary_y = 0.08;//0.027; // [cm] lateral FSR boundary (with offset -> +3.8 -1.6)
double foot_offset = 0.0125;//1.1; // [cm] lateral FSR offset (length between the foot base coordinate to middle of the foot)
int none=0;

//add 민하
double ps_RforceZ_avg=0;
double ps_RtorqueX_avg=0;
double ps_RtorqueY_avg=0;
double ps_LforceZ_avg=0;
double ps_LtorqueX_avg=0;
double ps_LtorqueY_avg=0;
int ps_sensing_count=0;
double ps_RforceZ_last=0;
double ps_RtorqueX_last=0;
double ps_RtorqueY_last=0;
double ps_LforceZ_last=0;
double ps_LtorqueX_last=0;
double ps_LtorqueY_last=0;
/////////////////////////////////////////////////////////add -민하
//add 민하
double f_Xl, f_Yl, f_Zl;
double tq_Xl, tq_Yl, tq_Zl;
double f_Xr, f_Yr, f_Zr;
double tq_Xr, tq_Yr, tq_Zr;

double d_Fx_SSP=0.0;
double d_Fy_SSP=0.0;
double d_Fz_SSP = _m*9.8;//22.0;//22.0;//28.0;
double d_tqX_SSP=0.0;//-0.15;//0.0;//-0.15;//
double d_tqY_SSP=0.0;//-0.2;//0.0;//-0.2;//
double d_tqZ_SSP=0.0;       

double d_Fx_DSP=0.0;
double d_Fy_DSP=0.0;
double d_Fz_DSP = _m*9.8*0.5;//0;// _m*9.8*0.5;//0.0;//_m*9.8*0.5;//11.0;//14.0;
double d_tqX_DSP=0.0;
double d_tqY_DSP=0.0;//-0.2;
double d_tqZ_DSP=0.0;    

//phase0 SSP_L 왼발 지지
double f_zr_p0_pos=0.0;
double tq_xr_p0_pos=0.0;
double tq_yr_p0_pos=0.0;
double f_zl_p0_pos=d_Fz_SSP;
double tq_xl_p0_pos=-0.24;
double tq_yl_p0_pos=-0.15;//-0.05;//0;//0.1;//0.1;//0;//
//phase1 SSP_R 오른발 지지
double f_zr_p1_pos=d_Fz_SSP;
double tq_xr_p1_pos=0.24;
double tq_yr_p1_pos=-0.15;//-0.05;//0;//0.1;//0;//
double f_zl_p1_pos=0.0;
double tq_xl_p1_pos=0.0;
double tq_yl_p1_pos=0.0;
//phase2 DSP_L 오른발 지지 이후 두발 지지
double f_zr_p2_pos=d_Fz_DSP;
double tq_xr_p2_pos=-0.12;
double tq_yr_p2_pos=0.08;//0.18;//
double f_zl_p2_pos=d_Fz_DSP;
double tq_xl_p2_pos=0.2;
double tq_yl_p2_pos=-0.1;//0.0;//
//phase3 DSP_R 왼발 지지 이후 두발 지지
double f_zr_p3_pos=d_Fz_DSP;
double tq_xr_p3_pos=-0.2;
double tq_yr_p3_pos=-0.1;//0;//
double f_zl_p3_pos=d_Fz_DSP;
double tq_xl_p3_pos=0.12;
double tq_yl_p3_pos=0.08;//0.18;//

double LF = 0.0305;//0.0265+0.004=0.0305//0.02715;//0.035;
double BX = -0.01501;//0;//+0.01;//-0.01;//-0.01;//0.01;
double BZ = 0.06582;//0.01;//0.01;
double L1 = 0.11;//0.093;
double L2 = 0.11;//0.093;
double L3 = 0.035;//0.037;
double pi = 3.141592;

double _zc = 0.27582;//0.22;//0.2;//0.09 * 2.0;//0.2;//0.098 * 2.0 ;
//0.22+0.05582=0.27582 because CoM move 0.01 to 0.06582
double _g = 9.81;
double _Tc = sqrt(_zc / _g);

enum _phase { SSP_L, SSP_R, DSP_L, DSP_R };
int new_phase_num = 0;
double FS = 0; // Frontal stride
double LS = 0; // Lateral stride

_phase phase;

double _px = 0;
double _py = 0;
double _ppx = 0;
double _ppy = 0;
double _rsx_des = 0;
double _rsy_des = 0;
double _lsx_des = 0;
double _lsy_des = 2 * L3;

double t = 0;
double C = 0;
double S = 0;
double x_ter = 0;
double y_ter = 0;
double x_terf = 0;
double y_terf = 0;
double vx_ter = 0;
double vy_ter = 0;
double x_com = 0;
double y_com = 0;
double x_des = 0;
double y_des = 0;

double a = 0.03 / (2.0 * pi);
double d = 0.074 / (2.0 * pi) / 4.0;
double b = 0.015 / 2;
double c = 0;

double Fz = 2.8 * 9.81;

double _A[12][3] = { { -1,0,0 } ,
{ 0,-1,0 },
{ 0,-1,0 },
{ 0,-1,0 },
{ -1,0,0 },
{ 0,0,-1 },
{ 0,0,1 },
{ 1,0,0 },
{ 0,1,0 },
{ 0,1,0 },
{ 0,1,0 },
{ 1,0,0 }
};

double CP_err[2]={0,};
double CP_cur[2]={0,};            //current local CP
double CP_ref[2]={0,};            //reference local CP
double CP_init[2]={0,};
double ZMP_c[2]={0,};           //current local ZMP
double ZMP_d[2]={0,};           //desired local ZMP
double a_desired[2]={0,};
double dsp_x_a0;
double dsp_x_a1;
double dsp_x_a2;
double dsp_x_a3;
double dsp_y_a0;
double dsp_y_a1;
double dsp_y_a2;
double dsp_y_a3;
double dsp_y_a0_nominal;
double dsp_y_a1_nominal;
double dsp_y_a2_nominal;
double dsp_y_a3_nominal;
double dsp_x_a0_nominal;
double dsp_x_a1_nominal;
double dsp_x_a2_nominal;
double dsp_x_a3_nominal;
double ndsp_x_a0;
double ndsp_x_a1;
double ndsp_x_a2;
double ndsp_x_a3;
double ndsp_x_a4;
double ndsp_x_a5;

double ndsp_y_a0;
double ndsp_y_a1;
double ndsp_y_a2;
double ndsp_y_a3;
double ndsp_y_a4;
double ndsp_y_a5;

//module 화를 하기 위한 전역변수

//add 0630 281 - 890 : matrix 정의하고 계산하려고 쓴 구조

typedef struct Matrix_1X3 
{
	double x[1][3];
}Matrix_1X3;


typedef struct Matrix_3X1 
{
    double x[3][1];
}Matrix_3X1;

typedef struct Matrix_3X3
{
	double x[3][3];
}Matrix_3X3;


typedef struct Matrix_6X1 
{
    double x[6][1];
}Matrix_6X1;


typedef struct Matrix_6X6 
{
	double x[6][6];
}Matrix_6X6;


typedef struct Matrix_12X1 
{
	double x[12][1];
}Matrix_12X1;

typedef struct Matrix_12X851
{
	double x[12][851];
}Matrix_12X851;

typedef struct Matrix_M31X7 
{
	Matrix_3X1 x[1][7];
}Matrix_M31X7;

typedef struct Matrix_M3X12 
{
	Matrix_3X1 x[1][12];
}Matrix_M3X12;

typedef struct Matrix_M12X1000
{
	Matrix_12X1 x[1][1000];
}Matrix_M12X1000;

typedef struct Matrix_MM33X12 
{
	Matrix_3X3 x[1][12];
}Matrix_MM33X12;

typedef struct Matrix_M33X7
{
		Matrix_3X3 x[7][1];
}Matrix_M33X7;

typedef struct Matrix_M31X1000 
{
		Matrix_3X1 x[1][1000];
}Matrix_M31X1000;

Matrix_1X3 transposeMatrix_31(Matrix_3X1 m)
{

	Matrix_1X3 r;

	r.x[0][0] = m.x[0][0];
	r.x[0][1] = m.x[1][0];
	r.x[0][2] = m.x[2][0];
	return r;
}

Matrix_3X3 transposeMatrix_33(Matrix_3X3 m)
{

	Matrix_3X3 r;

	r.x[0][0] = m.x[0][0];
	r.x[1][1] = m.x[1][1];
	r.x[2][2] = m.x[2][2];

	r.x[0][1] = m.x[1][0];
	r.x[0][2] = m.x[2][0];

	r.x[1][0] = m.x[0][1];
	r.x[1][2] = m.x[2][1];

	r.x[2][0] = m.x[0][2];
	r.x[2][1] = m.x[1][2];

	return r;

}


double productmatrix13_31 (Matrix_1X3 m1, Matrix_3X1 m2){

	double r;

	r = m1.x[0][0]*m2.x[0][0] + m1.x[0][1]*m2.x[1][0] + m1.x[0][2]*m2.x[2][0];

	return r;

}

Matrix_3X1 productmatrix11_31 (double m1, Matrix_3X1 m2){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = m1*m2.x[0][0] ;
	rMatrix.x[1][0] = m1*m2.x[1][0];
	rMatrix.x[2][0] = m1*m2.x[2][0];
  
	return rMatrix;

}

Matrix_3X1 productmatrix33_31 (Matrix_3X3 m1, Matrix_3X1 m2){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0]*m2.x[0][0] + m1.x[0][1]*m2.x[1][0] + m1.x[0][2]*m2.x[2][0];
	rMatrix.x[1][0] = m1.x[1][0]*m2.x[0][0] + m1.x[1][1]*m2.x[1][0] + m1.x[1][2]*m2.x[2][0];
	rMatrix.x[2][0] = m1.x[2][0]*m2.x[0][0] + m1.x[2][1]*m2.x[1][0] + m1.x[2][2]*m2.x[2][0];

	return rMatrix;

}

Matrix_6X1 productmatrix66_61 (Matrix_6X6 m1, Matrix_6X1 m2){

	Matrix_6X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0]*m2.x[0][0] + m1.x[0][1]*m2.x[1][0] + m1.x[0][2]*m2.x[2][0] +m1.x[0][3]*m2.x[3][0] +m1.x[0][4]*m2.x[4][0] +m1.x[0][5]*m2.x[5][0];
	rMatrix.x[1][0] = m1.x[1][0]*m2.x[0][0] + m1.x[1][1]*m2.x[1][0] + m1.x[1][2]*m2.x[2][0] +m1.x[1][3]*m2.x[3][0] +m1.x[1][4]*m2.x[4][0] +m1.x[1][5]*m2.x[5][0];
	rMatrix.x[2][0] = m1.x[2][0]*m2.x[0][0] + m1.x[2][1]*m2.x[1][0] + m1.x[2][2]*m2.x[2][0] +m1.x[2][3]*m2.x[3][0] +m1.x[2][4]*m2.x[4][0] +m1.x[2][5]*m2.x[5][0];
	rMatrix.x[3][0] = m1.x[3][0]*m2.x[0][0] + m1.x[3][1]*m2.x[1][0] + m1.x[3][2]*m2.x[2][0] +m1.x[3][3]*m2.x[3][0] +m1.x[3][4]*m2.x[4][0] +m1.x[3][5]*m2.x[5][0];
	rMatrix.x[4][0] = m1.x[4][0]*m2.x[0][0] + m1.x[4][1]*m2.x[1][0] + m1.x[4][2]*m2.x[2][0] +m1.x[4][3]*m2.x[3][0] +m1.x[4][4]*m2.x[4][0] +m1.x[4][5]*m2.x[5][0];
	rMatrix.x[5][0] = m1.x[5][0]*m2.x[0][0] + m1.x[5][1]*m2.x[1][0] + m1.x[5][2]*m2.x[2][0] +m1.x[5][3]*m2.x[3][0] +m1.x[5][4]*m2.x[4][0] +m1.x[5][5]*m2.x[5][0];
	return rMatrix;

}

Matrix_3X1 crossproductmatrix31_31 (Matrix_3X1 m1, Matrix_3X1 m2){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = m1.x[1][0]*m2.x[2][0] - m1.x[2][0]*m2.x[1][0] ;
	rMatrix.x[1][0] = -m1.x[0][0]*m2.x[2][0] + m1.x[2][0]*m2.x[0][0] ;
	rMatrix.x[2][0] = m1.x[0][0]*m2.x[1][0] - m1.x[1][0]*m2.x[0][0] ;
  
	return rMatrix;
  
}

Matrix_3X1 vectorplus31_31 (Matrix_3X1 m1, Matrix_3X1 m2){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] + m2.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] + m2.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] + m2.x[2][0] ;
  
	return rMatrix;

}

Matrix_3X1 vectorplus31_31_31 (Matrix_3X1 m1, Matrix_3X1 m2, Matrix_3X1 m3){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] + m2.x[0][0] + m3.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] + m2.x[1][0] + m3.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] + m2.x[2][0] + m3.x[2][0] ;
  
	return rMatrix;

}

Matrix_3X1 vectorplus31_31_31_31 (Matrix_3X1 m1, Matrix_3X1 m2, Matrix_3X1 m3, Matrix_3X1 m4){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] + m2.x[0][0] + m3.x[0][0] + m4.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] + m2.x[1][0] + m3.x[1][0] + m4.x[1][0];
	rMatrix.x[2][0] = m1.x[2][0] + m2.x[2][0] + m3.x[2][0] + m4.x[2][0];

	return rMatrix;

}
Matrix_12X1 vectorplus121_121 (Matrix_12X1 m1, Matrix_12X1 m2){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] + m2.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] + m2.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] + m2.x[2][0] ;
	rMatrix.x[3][0] = m1.x[3][0] + m2.x[3][0] ;
	rMatrix.x[4][0] = m1.x[4][0] + m2.x[4][0] ;
	rMatrix.x[5][0] = m1.x[5][0] + m2.x[5][0] ;
	rMatrix.x[6][0] = m1.x[6][0] + m2.x[6][0] ;
	rMatrix.x[7][0] = m1.x[7][0] + m2.x[7][0] ;
	rMatrix.x[8][0] = m1.x[8][0] + m2.x[8][0] ;
	rMatrix.x[9][0] = m1.x[9][0] + m2.x[9][0] ;
	rMatrix.x[10][0] = m1.x[10][0] + m2.x[10][0] ;
	rMatrix.x[11][0] = m1.x[11][0] + m2.x[11][0] ;
  
	return rMatrix;

}
Matrix_12X1 productmatrix121_d_d (double m1, double m2, Matrix_12X1 m3){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = m1 * m2 * m3.x[0][0];
	rMatrix.x[1][0] = m1 * m2 * m3.x[1][0] ;
	rMatrix.x[2][0] = m1 * m2 * m3.x[2][0] ;
	rMatrix.x[3][0] = m1 * m2 * m3.x[3][0] ;
	rMatrix.x[4][0] = m1 * m2 * m3.x[4][0] ;
	rMatrix.x[5][0] = m1 * m2 * m3.x[5][0] ;
	rMatrix.x[6][0] = m1 * m2 * m3.x[6][0] ;
	rMatrix.x[7][0] = m1 * m2 * m3.x[7][0] ;
	rMatrix.x[8][0] = m1 * m2 * m3.x[8][0] ;
	rMatrix.x[9][0] = m1 * m2 * m3.x[9][0] ;
	rMatrix.x[10][0] = m1 * m2 * m3.x[10][0];
	rMatrix.x[11][0] = m1 * m2 * m3.x[11][0] ;
  
	return rMatrix;

}
Matrix_12X1 vectorplus121_121_121_121 (Matrix_12X1 m1, Matrix_12X1 m2, Matrix_12X1 m3, Matrix_12X1 m4){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] + m2.x[0][0] + m3.x[0][0] + m4.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] + m2.x[1][0] + m3.x[1][0] + m4.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] + m2.x[2][0] + m3.x[2][0] + m4.x[2][0] ;
	rMatrix.x[3][0] = m1.x[3][0] + m2.x[3][0] + m3.x[3][0] + m4.x[3][0] ;
	rMatrix.x[4][0] = m1.x[4][0] + m2.x[4][0] + m3.x[4][0] + m4.x[4][0] ;
	rMatrix.x[5][0] = m1.x[5][0] + m2.x[5][0] + m3.x[5][0] + m4.x[5][0] ;
	rMatrix.x[6][0] = m1.x[6][0] + m2.x[6][0] + m3.x[6][0] + m4.x[6][0] ;
	rMatrix.x[7][0] = m1.x[7][0] + m2.x[7][0] + m3.x[7][0] + m4.x[7][0] ;
	rMatrix.x[8][0] = m1.x[8][0] + m2.x[8][0] + m3.x[8][0] + m4.x[8][0] ;
	rMatrix.x[9][0] = m1.x[9][0] + m2.x[9][0] + m3.x[9][0] + m4.x[9][0] ;
	rMatrix.x[10][0] = m1.x[10][0] + m2.x[10][0] + m3.x[10][0] + m4.x[10][0] ;
	rMatrix.x[11][0] = m1.x[11][0] + m2.x[11][0] + m3.x[11][0] + m4.x[11][0] ;
  
	return rMatrix;

}
Matrix_12X1 vectorplus121_121_121 (Matrix_12X1 m1, Matrix_12X1 m2, Matrix_12X1 m3){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] + m2.x[0][0] + m3.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] + m2.x[1][0] + m3.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] + m2.x[2][0] + m3.x[2][0] ;
	rMatrix.x[3][0] = m1.x[3][0] + m2.x[3][0] + m3.x[3][0] ;
	rMatrix.x[4][0] = m1.x[4][0] + m2.x[4][0] + m3.x[4][0] ;
	rMatrix.x[5][0] = m1.x[5][0] + m2.x[5][0] + m3.x[5][0] ;
	rMatrix.x[6][0] = m1.x[6][0] + m2.x[6][0] + m3.x[6][0] ;
	rMatrix.x[7][0] = m1.x[7][0] + m2.x[7][0] + m3.x[7][0] ;
	rMatrix.x[8][0] = m1.x[8][0] + m2.x[8][0] + m3.x[8][0] ;
	rMatrix.x[9][0] = m1.x[9][0] + m2.x[9][0] + m3.x[9][0] ;
	rMatrix.x[10][0] = m1.x[10][0] + m2.x[10][0] + m3.x[10][0] ;
	rMatrix.x[11][0] = m1.x[11][0] + m2.x[11][0] + m3.x[11][0] ;
  
	return rMatrix;

}

Matrix_3X1 s_vectorminus31_31 (Matrix_3X1 m1, Matrix_3X1 m2){

	Matrix_3X1 rMatrix;

	rMatrix.x[0][0] = (m1.x[0][0] - m2.x[0][0])/samplingtime ;
	rMatrix.x[1][0] = (m1.x[1][0] - m2.x[1][0])/samplingtime ;
	rMatrix.x[2][0] = (m1.x[2][0] - m2.x[2][0])/samplingtime ;
 
	return rMatrix;

}

Matrix_6X1 vectorminus61_61 (Matrix_6X1 m1, Matrix_6X1 m2){

	Matrix_6X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] - m2.x[0][0] ;
	rMatrix.x[1][0] = m1.x[1][0] - m2.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] - m2.x[2][0] ;
	rMatrix.x[3][0] = m1.x[3][0] - m2.x[3][0] ;
	rMatrix.x[4][0] = m1.x[4][0] - m2.x[4][0] ;
	rMatrix.x[5][0] = m1.x[5][0] - m2.x[5][0] ;
  
	return rMatrix;

}

Matrix_6X1 s_vectorminus61_61 (Matrix_6X1 m1, Matrix_6X1 m2){

	Matrix_6X1 rMatrix;

	rMatrix.x[0][0] = (m1.x[0][0] - m2.x[0][0])/samplingtime ;
	rMatrix.x[1][0] = (m1.x[1][0] - m2.x[1][0])/samplingtime ;
	rMatrix.x[2][0] = (m1.x[2][0] - m2.x[2][0])/samplingtime ;
	rMatrix.x[3][0] = (m1.x[3][0] - m2.x[3][0])/samplingtime ;
	rMatrix.x[4][0] = (m1.x[4][0] - m2.x[4][0])/samplingtime ;
	rMatrix.x[5][0] = (m1.x[5][0] - m2.x[5][0])/samplingtime;
  
	return rMatrix;

}

Matrix_12X1 vectorminus121_121 (Matrix_12X1 m1, Matrix_12X1 m2){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = m1.x[0][0] - m2.x[0][0];
	rMatrix.x[1][0] = m1.x[1][0] - m2.x[1][0] ;
	rMatrix.x[2][0] = m1.x[2][0] - m2.x[2][0] ;
	rMatrix.x[3][0] = m1.x[3][0] - m2.x[3][0] ;
	rMatrix.x[4][0] = m1.x[4][0] - m2.x[4][0] ;
	rMatrix.x[5][0] = m1.x[5][0] - m2.x[5][0] ;
	rMatrix.x[6][0] = m1.x[6][0] - m2.x[6][0] ;
	rMatrix.x[7][0] = m1.x[7][0] - m2.x[7][0] ;
	rMatrix.x[8][0] = m1.x[8][0] - m2.x[8][0] ;
	rMatrix.x[9][0] = m1.x[9][0] - m2.x[9][0] ;
	rMatrix.x[10][0] = m1.x[10][0] - m2.x[10][0];
	rMatrix.x[11][0] = m1.x[11][0] - m2.x[11][0] ;
  
	return rMatrix;

}

Matrix_12X1 productmatrix121_d (double m1, Matrix_12X1 m2){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = m1 * m2.x[0][0];
	rMatrix.x[1][0] = m1 * m2.x[1][0] ;
	rMatrix.x[2][0] = m1 * m2.x[2][0] ;
	rMatrix.x[3][0] = m1 * m2.x[3][0] ;
	rMatrix.x[4][0] = m1 * m2.x[4][0] ;
	rMatrix.x[5][0] = m1 * m2.x[5][0] ;
	rMatrix.x[6][0] = m1 * m2.x[6][0] ;
	rMatrix.x[7][0] = m1 * m2.x[7][0] ;
	rMatrix.x[8][0] = m1 * m2.x[8][0] ;
	rMatrix.x[9][0] = m1 * m2.x[9][0] ;
	rMatrix.x[10][0] = m1 * m2.x[10][0];
	rMatrix.x[11][0] = m1 * m2.x[11][0] ;
  
	return rMatrix;

}

Matrix_12X1 s_vectorminus121_121 (Matrix_12X1 m1, Matrix_12X1 m2){

	Matrix_12X1 rMatrix;

	rMatrix.x[0][0] = (m1.x[0][0] - m2.x[0][0])/samplingtime ;
	rMatrix.x[1][0] = (m1.x[1][0] - m2.x[1][0])/samplingtime ;
	rMatrix.x[2][0] = (m1.x[2][0] - m2.x[2][0])/samplingtime ;
	rMatrix.x[3][0] = (m1.x[3][0] - m2.x[3][0])/samplingtime ;
	rMatrix.x[4][0] = (m1.x[4][0] - m2.x[4][0])/samplingtime ;
	rMatrix.x[5][0] = (m1.x[5][0] - m2.x[5][0])/samplingtime ;
	rMatrix.x[6][0] = (m1.x[6][0] - m2.x[6][0])/samplingtime ;
	rMatrix.x[7][0] = (m1.x[7][0] - m2.x[7][0])/samplingtime ;
	rMatrix.x[8][0] = (m1.x[8][0] - m2.x[8][0])/samplingtime ;
	rMatrix.x[9][0] = (m1.x[9][0] - m2.x[9][0])/samplingtime ;
	rMatrix.x[10][0] = (m1.x[10][0] - m2.x[10][0])/samplingtime ;
	rMatrix.x[11][0] = (m1.x[11][0] - m2.x[11][0])/samplingtime ;
  
	return rMatrix;

}

Matrix_6X6 s_vectorminus66_66 (Matrix_6X6 m1, Matrix_6X6 m2){

	Matrix_6X6 rMatrix;

	rMatrix.x[0][0] = (m1.x[0][0]-m2.x[0][0])/samplingtime;
	rMatrix.x[1][0] = (m1.x[1][0]-m2.x[1][0])/samplingtime;
	rMatrix.x[2][0] = (m1.x[2][0]-m2.x[2][0])/samplingtime;
	rMatrix.x[3][0] = (m1.x[3][0]-m2.x[3][0])/samplingtime;
	rMatrix.x[4][0] = (m1.x[4][0]-m2.x[4][0])/samplingtime;
	rMatrix.x[5][0] = (m1.x[5][0]-m2.x[5][0])/samplingtime;

	rMatrix.x[0][1] = (m1.x[0][1]-m2.x[0][1])/samplingtime;
	rMatrix.x[1][1] = (m1.x[1][1]-m2.x[1][1])/samplingtime;
	rMatrix.x[2][1] = (m1.x[2][1]-m2.x[2][1])/samplingtime;
	rMatrix.x[3][1] = (m1.x[3][1]-m2.x[3][1])/samplingtime;
	rMatrix.x[4][1] = (m1.x[4][1]-m2.x[4][1])/samplingtime;
	rMatrix.x[5][1] = (m1.x[5][1]-m2.x[5][1])/samplingtime;

	rMatrix.x[0][2] = (m1.x[0][2]-m2.x[0][2])/samplingtime;
	rMatrix.x[1][2] = (m1.x[1][2]-m2.x[1][2])/samplingtime;
	rMatrix.x[2][2] = (m1.x[2][2]-m2.x[2][2])/samplingtime;
	rMatrix.x[3][2] = (m1.x[3][2]-m2.x[3][2])/samplingtime;
	rMatrix.x[4][2] = (m1.x[4][2]-m2.x[4][2])/samplingtime;
	rMatrix.x[5][2] = (m1.x[5][2]-m2.x[5][2])/samplingtime;

	rMatrix.x[0][3] = (m1.x[0][3]-m2.x[0][3])/samplingtime;
	rMatrix.x[1][3] = (m1.x[1][3]-m2.x[1][3])/samplingtime;
	rMatrix.x[2][3] = (m1.x[2][3]-m2.x[2][3])/samplingtime;
	rMatrix.x[3][3] = (m1.x[3][3]-m2.x[3][3])/samplingtime;
	rMatrix.x[4][3] = (m1.x[4][3]-m2.x[4][3])/samplingtime;
	rMatrix.x[5][3] = (m1.x[5][3]-m2.x[5][3])/samplingtime;


	rMatrix.x[0][4] = (m1.x[0][4]-m2.x[0][4])/samplingtime;
	rMatrix.x[1][4] = (m1.x[1][4]-m2.x[1][4])/samplingtime;
	rMatrix.x[2][4] = (m1.x[2][4]-m2.x[2][4])/samplingtime;
	rMatrix.x[3][4] = (m1.x[3][4]-m2.x[3][4])/samplingtime;
	rMatrix.x[4][4] = (m1.x[4][4]-m2.x[4][4])/samplingtime;
	rMatrix.x[5][4] = (m1.x[5][4]-m2.x[5][4])/samplingtime;


	rMatrix.x[0][5] = (m1.x[0][5]-m2.x[0][5])/samplingtime;
	rMatrix.x[1][5] = (m1.x[1][5]-m2.x[1][5])/samplingtime;
	rMatrix.x[2][5] = (m1.x[2][5]-m2.x[2][5])/samplingtime;
	rMatrix.x[3][5] = (m1.x[3][5]-m2.x[3][5])/samplingtime;
	rMatrix.x[4][5] = (m1.x[4][5]-m2.x[4][5])/samplingtime;
	rMatrix.x[5][5] = (m1.x[5][5]-m2.x[5][5])/samplingtime;

	return rMatrix;

}

Matrix_6X6 saveMatrix_66(double m[6][6])
{
	Matrix_6X6 r;

	
	r.x[0][0] = m[0][0];
	r.x[1][1] = m[1][1];
	r.x[2][2] = m[2][2];
	r.x[3][3] = m[3][3];
	r.x[4][4] = m[4][4];
	r.x[5][5] = m[5][5];
  
  
	r.x[0][1] = m[0][1];
	r.x[0][2] = m[0][2];
	r.x[0][3] = m[0][3];
	r.x[0][4] = m[0][4];
	r.x[0][5] = m[0][5];
 
  
	r.x[1][0] = m[1][0];
	r.x[1][2] = m[1][2];
	r.x[1][3] = m[1][3];
	r.x[1][4] = m[1][4];
	r.x[1][5] = m[1][5];

	r.x[2][0] = m[2][0];
	r.x[2][1] = m[2][1];
	r.x[2][3] = m[2][3];
	r.x[2][4] = m[2][4];
	r.x[2][5] = m[2][5];
  
	r.x[3][0] = m[3][0];
	r.x[3][1] = m[3][1];
	r.x[3][2] = m[3][2];
	r.x[3][4] = m[3][4];
	r.x[3][5] = m[3][5];
  
  
	r.x[4][0] = m[4][0];
	r.x[4][1] = m[4][1];
	r.x[4][2] = m[4][2];
	r.x[4][3] = m[4][3];
	r.x[4][5] = m[4][5];

  
	r.x[5][0] = m[5][0];
	r.x[5][1] = m[5][1];
	r.x[5][2] = m[5][2];
	r.x[5][3] = m[5][3];
	r.x[5][4] = m[5][4];
  
  
	return r;
}

void savedouble_66(double m[6][6], double m2[6][6])
{
	//double _jacobi_n;
	
	for(int i=0; i<6; i++)

	{

		for(int j=0; j<6; j++){
    
			m2[i][j] = m[i][5-j];

		}

	}
	return;
}

Matrix_6X6 M_saveMatrix_66(Matrix_6X6 m)
{
	Matrix_6X6 r;
	
	r.x[0][0] = m.x[0][0];
	r.x[1][1] = m.x[1][1];
	r.x[2][2] = m.x[2][2];
	r.x[3][3] = m.x[3][3];
	r.x[4][4] = m.x[4][4];
	r.x[5][5] = m.x[5][5];
  
  
	r.x[0][1] = m.x[0][1];
	r.x[0][2] = m.x[0][2];
	r.x[0][3] = m.x[0][3];
	r.x[0][4] = m.x[0][4];
	r.x[0][5] = m.x[0][5];
 
  
	r.x[1][0] = m.x[1][0];
	r.x[1][2] = m.x[1][2];
	r.x[1][3] = m.x[1][3];
	r.x[1][4] = m.x[1][4];
	r.x[1][5] = m.x[1][5];

	r.x[2][0] = m.x[2][0];
	r.x[2][1] = m.x[2][1];
	r.x[2][3] = m.x[2][3];
	r.x[2][4] = m.x[2][4];
	r.x[2][5] = m.x[2][5];
  
	r.x[3][0] = m.x[3][0];
	r.x[3][1] = m.x[3][1];
	r.x[3][2] = m.x[3][2];
	r.x[3][4] = m.x[3][4];
	r.x[3][5] = m.x[3][5];
  
  
	r.x[4][0] = m.x[4][0];
	r.x[4][1] = m.x[4][1];
	r.x[4][2] = m.x[4][2];
	r.x[4][3] = m.x[4][3];
	r.x[4][5] = m.x[4][5];
  
  
	r.x[5][0] = m.x[5][0];
	r.x[5][1] = m.x[5][1];
	r.x[5][2] = m.x[5][2];
	r.x[5][3] = m.x[5][3];
	r.x[5][4] = m.x[5][4];
  
  
	return r;
}

Matrix_6X6 changeMatrix_66(Matrix_6X6 m1)
{
	Matrix_6X6 rMatrix;

	
	rMatrix.x[0][0] = m1.x[0][5];
	rMatrix.x[1][0] = m1.x[1][5];
	rMatrix.x[2][0] = m1.x[2][5];
	rMatrix.x[3][0] = m1.x[3][5];
	rMatrix.x[4][0] = m1.x[4][5];
	rMatrix.x[5][0] = m1.x[5][5];
  
  
	rMatrix.x[0][1] = m1.x[0][4];
	rMatrix.x[1][1] = m1.x[1][4];
	rMatrix.x[2][1] = m1.x[2][4];
	rMatrix.x[3][1] = m1.x[3][4];
	rMatrix.x[4][1] = m1.x[4][4];
	rMatrix.x[5][1] = m1.x[5][4];
 
  
	rMatrix.x[0][2] = m1.x[0][3];
	rMatrix.x[1][2] = m1.x[1][3];
	rMatrix.x[2][2] = m1.x[2][3];
	rMatrix.x[3][2] = m1.x[3][3];
	rMatrix.x[4][2] = m1.x[4][3];
	rMatrix.x[5][2] = m1.x[5][3];
  
  
	rMatrix.x[0][3] = m1.x[0][2];
	rMatrix.x[1][3] = m1.x[1][2];
	rMatrix.x[2][3] = m1.x[2][2];
	rMatrix.x[3][3] = m1.x[3][2];
	rMatrix.x[4][3] = m1.x[4][2];
	rMatrix.x[5][3] = m1.x[5][2];
  
  
	rMatrix.x[0][4] = m1.x[0][1];
	rMatrix.x[1][4] = m1.x[1][1];
	rMatrix.x[2][4] = m1.x[2][1];
	rMatrix.x[3][4] = m1.x[3][1];
	rMatrix.x[4][4] = m1.x[4][1];
	rMatrix.x[5][4] = m1.x[5][1];
  
  
	rMatrix.x[0][5] = m1.x[0][0];
	rMatrix.x[1][5] = m1.x[1][0];
	rMatrix.x[2][5] = m1.x[2][0];
	rMatrix.x[3][5] = m1.x[3][0];
	rMatrix.x[4][5] = m1.x[4][0];
	rMatrix.x[5][5] = m1.x[5][0];
  
	return rMatrix;
  
}
///////////////////////////////////////////////////////////////////////add -민하

double targetp[3] = { 0.02, 0.023, 0.1568 };
double targetr[3][3] = { { 1,0,0 }
,{ 0,1,0 }
,{ 0,0,1 } };
double btargetp[3] = { 0.02, 0.023, 0.1568 };
double btargetr[3][3] = { { 1,0,0 }
,{ 0,1,0 }
,{ 0,0,1 } };
double rtargetp[3] = { 0.02, 0.023, 0.1568 };
double rtargetr[3][3] = { { 1,0,0 }
,{ 0,1,0 }
,{ 0,0,1 } };
double ltargetp[3] = { 0.02, 0.023, 0.1568 };
double ltargetr[3][3] = { { 1,0,0 }
,{ 0,1,0 }
,{ 0,0,1 } };

//B to R 좌표계

double TWB[4][4] = { { 1,0,0,0 }
,{ 0,1,0,L3 }
,{ 0,0,1,_zc }
,{ 0, 0, 0, 1 } };
double TRWE[4][4];
double TRW6[4][4];
double TRW5[4][4];
double TRW4[4][4];
double TRW3[4][4];
double TRW2[4][4];
double TRW1[4][4];
//add 민하 917 - 982 : forward 때 쓰려고 선언돼 있는건데 혹시 없으면 추가하면 돼
double TRB1[4][4];
double TR12[4][4];
double TR23[4][4];
double TR34[4][4];
double TR45[4][4];
double TR56[4][4];
double TR6E[4][4];

double TRB2[4][4];
double TRB3[4][4];
double TRB4[4][4];
double TRB5[4][4];
double TRB6[4][4];
double TRBE[4][4];

double _jacobi[6][6];
double _jacobi_n[6][6];
///////////////////////////////////////////////////////////////////add -민하

//B to l 좌표계

double TLWE[4][4] = { 0, };
double TLW6[4][4] = { 0, };
double TLW5[4][4] = { 0, };
double TLW4[4][4] = { 0, };
double TLW3[4][4] = { 0, };
double TLW2[4][4] = { 0, };
double TLW1[4][4] = { 0, };

//add 0630 917 - 982 : forward 때 쓰려고 선언돼 있는건데 혹시 없으면 추가하면 돼
double TLB1[4][4];
double TL12[4][4];
double TL23[4][4];
double TL34[4][4];
double TL45[4][4];
double TL56[4][4];
double TL6E[4][4];


double TLB2[4][4];
double TLB3[4][4];
double TLB4[4][4];
double TLB5[4][4];
double TLB6[4][4];
double TLBE[4][4];
//////////////////////////////////////////////////////////////////////////////////////////


double err[6] = { 0, };

int cin_function(double a[6][6], double jacobi[6][6]) {
	int i, j, n = 6;
	for (i = 0; i < n; i++)
		for (j = 0; j < n; j++) {
			a[i][j] = jacobi[i][j];
		}
	return n;
}

void cout_function(double a[6][6], int n, int show) {
	int i, j;
	if (show == 1)
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++)
				printf(" %lf \t", a[i][j]);
			printf("\n");
		}
	else if (show == 2) {
		printf("\n\n The Inverse Of Matrix Is : \n\n");
		for (i = 0; i < n; i++) {
			for (j = 0; j < n; j++)
				printf(" %lf \t", a[i][j]);
			printf("\n");
		}
	}
}

void minor(double b[6][6], double a[6][6], int i, int n) {
	int j, l, h = 0, k = 0;
	for (l = 1; l < n; l++)
		for (j = 0; j < n; j++) {
			if (j == i)
				continue;
			b[h][k] = a[l][j];
			k++;
			if (k == (n - 1)) {
				h++;
				k = 0;
			}
		}
}

double det(double a[6][6], int n) {
	int i;
	double b[6][6], sum = 0;
	if (n == 1)
		return a[0][0];
	else if (n == 2)
		return (a[0][0] * a[1][1] - a[0][1] * a[1][0]);
	else
		for (i = 0; i < n; i++) {
			minor(b, a, i, n);   // read function
			sum = (double)(sum + a[0][i] * pow(-1, i) * det(b, (n - 1)));   // read function   // sum = determinte matrix
		}
	return sum;
}

void transpose(double c[6][6], double d[6][6], double n, double det) {
	int i, j;
	double b[6][6];
	for (i = 0; i < n; i++)
		for (j = 0; j < n; j++)
			b[i][j] = c[j][i];
	for (i = 0; i < n; i++)
		for (j = 0; j < n; j++)
			d[i][j] = b[i][j] / det;   // array d[][] = inverse matrix
}

void cofactor(double a[6][6], double d[6][6], double n, double determinte) {
	double b[6][6], c[6][6];
	int l, h, m, k, i, j;
	for (h = 0; h < n; h++)
		for (l = 0; l < n; l++) {
			m = 0;
			k = 0;
			for (i = 0; i < n; i++)
				for (j = 0; j < n; j++)
					if (i != h && j != l) {
						b[m][k] = a[i][j];
						if (k < (n - 2))
							k++;
						else {
							k = 0;
							m++;
						}
					}
			c[h][l] = pow(-1, (h + l)) * det(b, (n - 1));   // c = cofactor Matrix
		}
	transpose(c, d, n, determinte);   // read function
}



void inverse(double a[6][6], double d[6][6], int n, double det) {
	if (det == 0)
		printf("\nInverse of Entered Matrix is not possible\n");
	else if (n == 1)
		d[0][0] = 1;
	else
		cofactor(a, d, n, det);
}

void matrixprint3(double A[][3]) {
	int i = 0;
	int j = 0;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			printf("%lf ", A[i][j]);
		}
		printf("\n");
	}
}

//앞에꺼는 크기가 4*4짜리만 가능
void matrixmultiply3(double A[][4], double B[][3], double S[][3]) {
	int i = 0;
	int j = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			for (k = 0; k < 3; k++) {
				sum += A[i][k] * B[k][j];
			}
			S[i][j] = sum;
			sum = 0;
		}
	}


}

void matrixmultiply331(double A[][4], double B[], double S[]) {
	int i = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 3; i++) {
		for (k = 0; k < 3; k++) {
			sum += A[i][k] * B[k];
		}
		S[i] = sum;
		sum = 0;
	}


}

void reversematrixmultiply331(double A[][4], double B[], double S[]) {
	int i = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 3; i++) {
		for (k = 0; k < 3; k++) {
			sum += A[k][i] * B[k];
		}
		S[i] = sum;
		sum = 0;
	}


}

void freversematrixmultiply3(double A[][4], double B[][3], double S[][3]) {
	int i = 0;
	int j = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			for (k = 0; k < 3; k++) {
				sum += A[k][i] * B[k][j];
			}
			S[i][j] = sum;
			sum = 0;
		}
	}


}

void rreversematrixmultiply3(double A[][3], double B[][4], double S[][3]) {
	int i = 0;
	int j = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			for (k = 0; k < 3; k++) {
				sum += A[i][k] * B[j][k];
			}
			S[i][j] = sum;
			sum = 0;
		}
	}
}


void matrixassign(double A[][4], double B[][4]) {
	int i = 0;
	int j = 0;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			B[i][j] = A[i][j];
		}

	}
}
void matrixprint4(double A[][4]) {
	int i = 0;
	int j = 0;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			printf("%lf ", A[i][j]);
		}
		printf("\n");
	}
}

void matrixmultiply4(double A[][4], double B[][4], double S[][4]) {
	int i = 0;
	int j = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			for (k = 0; k < 4; k++) {
				sum += A[i][k] * B[k][j];
			}
			S[i][j] = sum;
			sum = 0;
		}
	}


}

void matrixmultiply6(double A[][6], double B[][6], double S[][6]) {
	int i = 0;
	int j = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			for (k = 0; k < 6; k++) {
				sum += A[i][k] * B[k][j];
			}
			S[i][j] = sum;
			sum = 0;
		}
	}


}

void matrixmultiply661(double A[][6], double B[], double S[]) {
	int i = 0;
	// int j = 0;
	int k = 0;
	double sum = 0;

	for (i = 0; i < 6; i++) {
		for (k = 0; k < 6; k++) {
			sum += A[i][k] * B[k];
		}
		S[i] = sum;
		sum = 0;
	}
}

void matrixprint6(double A[][6]) {
	int i = 0;
	int j = 0;

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			printf("%lf ", A[i][j]);
		}
		printf("\n");
	}
}

void forwardkinematics(double th[]) {
	//original 0630
	// double TR6E[4][4] = { { 1,0,0,0 }
	// 	,{ 0,1,0, 0 }
	// 	,{ 0,0,1,-LF }
	// ,{ 0, 0, 0, 1 } };
	// double TR56[4][4] = { { 1,0,0,0 }
	// 	,{ 0,cos(-th[0]),-sin(-th[0]),0 }
	// 	,{ 0,sin(-th[0]),cos(-th[0]),0 }
	// ,{ 0, 0, 0, 1 } };
	// double TR45[4][4] = { { cos(-th[1]),0,sin(-th[1]),0 }
	// 	,{ 0,1,0,0 }
	// 	,{ -sin(-th[1]),0,cos(-th[1]),-L2 }
	// ,{ 0, 0, 0, 1 } };
	// double TR34[4][4] = { { cos(-th[2]),0,sin(-th[2]),0 }
	// 	,{ 0,1,0,0 }
	// 	,{ -sin(-th[2]),0,cos(-th[2]),-L1 }
	// ,{ 0, 0, 0, 1 } };
	// double TR23[4][4] = { { cos(-th[3]),0,sin(-th[3]),0 }
	// 	,{ 0,1,0,0 }
	// 	,{ -sin(-th[3]),0,cos(-th[3]),0 }
	// ,{ 0, 0, 0, 1 } };
	// double TR12[4][4] = { { 1,0,0,0 }
	// 	,{ 0,cos(-th[4]),-sin(-th[4]),0 }
	// 	,{ 0,sin(-th[4]),cos(-th[4]),0 }
	// ,{ 0, 0, 0, 1 } };
	// double TRB1[4][4] = { { cos(-th[5]),-sin(-th[5]),0,-BX }
	// 	,{ sin(-th[5]),cos(-th[5]),0,-L3 }
	// 	,{ 0,0,1,-BZ }
	// ,{ 0, 0, 0, 1 } };

	// double TLB1[4][4] = { { cos(th[6]),-sin(th[6]),0,-BX }
	// 	,{ sin(th[6]),cos(th[6]),0,L3 }
	// 	,{ 0,0,1,-BZ }
	// ,{ 0, 0, 0, 1 } };
	// double TL12[4][4] = { { 1,0,0,0 }
	// 	,{ 0,cos(th[7]),-sin(th[7]),0 }
	// 	,{ 0,sin(th[7]),cos(th[7]),0 }
	// ,{ 0, 0, 0, 1 } };
	// double TL23[4][4] = { { cos(th[8]),0,sin(th[8]),0 }
	// 	,{ 0,1,0,0 }
	// 	,{ -sin(th[8]),0,cos(th[8]),0 }
	// ,{ 0, 0, 0, 1 } };
	// double TL34[4][4] = { { cos(th[9]),0,sin(th[9]),0 }
	// 	,{ 0,1,0,0 }
	// 	,{ -sin(th[9]),0,cos(th[9]),-L2 }
	// ,{ 0, 0, 0, 1 } };
	// double TL45[4][4] = { { cos(th[10]),0,sin(th[10]),0 }
	// 	,{ 0,1,0,0 }
	// 	,{ -sin(th[10]),0,cos(th[10]),-L1 }
	// ,{ 0, 0, 0, 1 } };
	// double TL56[4][4] = { { 1,0,0,0 }
	// 	,{ 0,cos(th[11]),-sin(th[11]),0 }
	// 	,{ 0,sin(th[11]),cos(th[11]),0 }
	// ,{ 0, 0, 0, 1 } };
	// double TL6E[4][4] = { { 1,0,0,0 }
	// 	,{ 0,1,0,0 }
	// 	,{ 0,0,1,-LF }
	// ,{ 0, 0, 0, 1 } };

	// matrixmultiply4(TWB, TRB1, TRW1);
	// matrixmultiply4(TRW1, TR12, TRW2);
	// matrixmultiply4(TRW2, TR23, TRW3);
	// matrixmultiply4(TRW3, TR34, TRW4);
	// matrixmultiply4(TRW4, TR45, TRW5);
	// matrixmultiply4(TRW5, TR56, TRW6);
	// matrixmultiply4(TRW6, TR6E, TRWE);
	// matrixmultiply4(TWB, TLB1, TLW1);
	// matrixmultiply4(TLW1, TL12, TLW2);
	// matrixmultiply4(TLW2, TL23, TLW3);
	// matrixmultiply4(TLW3, TL34, TLW4);
	// matrixmultiply4(TLW4, TL45, TLW5);
	// matrixmultiply4(TLW5, TL56, TLW6);
	// matrixmultiply4(TLW6, TL6E, TLWE);
	/*printf("1 \n");
	matrixprint4(TRWB);
	printf("2 \n");
	matrixprint4(TRB12);*/
	////////////////////////////////////////////////////////////////////////////////////////////
	//add 0630 1325 - 1647 : forward 도 아마 진우가 가지고 있는 기존 구현 방법이랑 달라졌을 것 같아
	TR6E[0][0]= 1;
	TR6E[0][1]= 0;
	TR6E[0][2]= 0;
	TR6E[0][3]= 0.024;

	TR6E[1][0]= 0;
	TR6E[1][1]= 1;
	TR6E[1][2]= 0;
	TR6E[1][3]= 0;//-0.0125;//0;//

	TR6E[2][0]= 0;
	TR6E[2][1]= 0;
	TR6E[2][2]= 1;
	TR6E[2][3]= -LF;

	TR6E[3][0]= 0;
	TR6E[3][1]= 0;
	TR6E[3][2]= 0;
	TR6E[3][3]= 1;
	
	TR56[0][0]= 1;
	TR56[0][1]= 0;
	TR56[0][2]= 0;
	TR56[0][3]= -0.0241;

	TR56[1][0]= 0;
	TR56[1][1]= cos(-th[0]);
	TR56[1][2]= -sin(-th[0]);
	TR56[1][3]= 0.019;

	TR56[2][0]= 0;
	TR56[2][1]= sin(-th[0]);
	TR56[2][2]= cos(-th[0]);
	TR56[2][3]= 0;

	TR56[3][0]= 0;
	TR56[3][1]= 0;
	TR56[3][2]= 0;
	TR56[3][3]= 1;
	
	TR45[0][0]= cos(-th[1]);
	TR45[0][1]= 0;
	TR45[0][2]= sin(-th[1]);
	TR45[0][3]= 0;

	TR45[1][0]= 0;
	TR45[1][1]= 1;
	TR45[1][2]= 0;
	TR45[1][3]= 0;

	TR45[2][0]= -sin(-th[1]);
	TR45[2][1]= 0;
	TR45[2][2]= cos(-th[1]);
	TR45[2][3]= -L2 ;

	TR45[3][0]= 0;
	TR45[3][1]= 0;
	TR45[3][2]= 0;
	TR45[3][3]= 1;
	

		
	TR34[0][0]= cos(-th[2]);
	TR34[0][1]= 0;
	TR34[0][2]= sin(-th[2]);
	TR34[0][3]= 0;

	TR34[1][0]= 0;
	TR34[1][1]= 1;
	TR34[1][2]= 0;
	TR34[1][3]= 0;

	TR34[2][0]= -sin(-th[2]);
	TR34[2][1]= 0;
	TR34[2][2]= cos(-th[2]);
	TR34[2][3]= -L1;

	TR34[3][0]= 0;
	TR34[3][1]= 0;
	TR34[3][2]= 0;
	TR34[3][3]= 1;
	
		
		
	TR23[0][0]= cos(-th[3]);
	TR23[0][1]= 0;
	TR23[0][2]= sin(-th[3]);
	TR23[0][3]= 0.0241;

	TR23[1][0]= 0;
	TR23[1][1]= 1;
	TR23[1][2]= 0;
	TR23[1][3]= -0.019;

	TR23[2][0]= -sin(-th[3]);
	TR23[2][1]= 0;
	TR23[2][2]= cos(-th[3]);
	TR23[2][3]= 0;

	TR23[3][0]= 0;
	TR23[3][1]= 0;
	TR23[3][2]= 0;
	TR23[3][3]= 1;
	
			
	TR12[0][0]= 1;
	TR12[0][1]= 0;
	TR12[0][2]= 0;
	TR12[0][3]= -0.024;

	TR12[1][0]= 0;
	TR12[1][1]= cos(-th[4]);
	TR12[1][2]= -sin(-th[4]);
	TR12[1][3]= 0;

	TR12[2][0]= 0;
	TR12[2][1]= sin(-th[4]);
	TR12[2][2]= cos(-th[4]);
	TR12[2][3]= -0.0285;

	TR12[3][0]= 0;
	TR12[3][1]= 0;
	TR12[3][2]= 0;
	TR12[3][3]= 1;


	
	TRB1[0][0]= cos(-th[5]);
	TRB1[0][1]= -sin(-th[5]);
	TRB1[0][2]= 0;
	TRB1[0][3]= -BX;

	TRB1[1][0]= sin(-th[5]);
	TRB1[1][1]= cos(-th[5]);
	TRB1[1][2]= 0;
	TRB1[1][3]= -L3;

	TRB1[2][0]= 0;
	TRB1[2][1]= 0;
	TRB1[2][2]= 1;
	TRB1[2][3]= -BZ;

	TRB1[3][0]= 0;
	TRB1[3][1]= 0;
	TRB1[3][2]= 0;
	TRB1[3][3]= 1;



	TLB1[0][0]= cos(th[6]);
	TLB1[0][1]= -sin(th[6]);
	TLB1[0][2]= 0;
	TLB1[0][3]= -BX;

	TLB1[1][0]= sin(th[6]);
	TLB1[1][1]= cos(th[6]);
	TLB1[1][2]= 0;
	TLB1[1][3]= L3;

	TLB1[2][0]= 0;
	TLB1[2][1]= 0;
	TLB1[2][2]= 1;
	TLB1[2][3]= -BZ;

	TLB1[3][0]= 0;
	TLB1[3][1]= 0;
	TLB1[3][2]= 0;
	TLB1[3][3]= 1;


	TL12[0][0]= 1;
	TL12[0][1]= 0;
	TL12[0][2]= 0;
	TL12[0][3]= -0.024;

	TL12[1][0]= 0;
	TL12[1][1]= cos(th[7]);
	TL12[1][2]= -sin(th[7]);
	TL12[1][3]= 0;

	TL12[2][0]= 0;
	TL12[2][1]= sin(th[7]);
	TL12[2][2]= cos(th[7]);
	TL12[2][3]= -0.0285;

	TL12[3][0]= 0;
	TL12[3][1]= 0;
	TL12[3][2]= 0;
	TL12[3][3]= 1;


	TL23[0][0]= cos(th[8]);
	TL23[0][1]= 0;
	TL23[0][2]= sin(th[8]);
	TL23[0][3]= 0.0241;

	TL23[1][0]= 0;
	TL23[1][1]= 1;
	TL23[1][2]= 0;
	TL23[1][3]= 0.019;

	TL23[2][0]= -sin(th[8]);
	TL23[2][1]= 0;
	TL23[2][2]= cos(th[8]);
	TL23[2][3]= 0;

	TL23[3][0]= 0;
	TL23[3][1]= 0;
	TL23[3][2]= 0;
	TL23[3][3]= 1;
	
	
	TL34[0][0]= cos(th[9]);
	TL34[0][1]= 0;
	TL34[0][2]= sin(th[9]);
	TL34[0][3]= 0;

	TL34[1][0]= 0;
	TL34[1][1]= 1;
	TL34[1][2]= 0;
	TL34[1][3]= 0;

	TL34[2][0]= -sin(th[9]);
	TL34[2][1]= 0;
	TL34[2][2]= cos(th[9]);
	TL34[2][3]= -L1;

	TL34[3][0]= 0;
	TL34[3][1]= 0;
	TL34[3][2]= 0;
	TL34[3][3]= 1;


	
	TL45[0][0]= cos(th[10]);
	TL45[0][1]= 0;
	TL45[0][2]= sin(th[10]);
	TL45[0][3]= 0;

	TL45[1][0]= 0;
	TL45[1][1]= 1;
	TL45[1][2]= 0;
	TL45[1][3]= 0;

	TL45[2][0]= -sin(th[10]);
	TL45[2][1]= 0;
	TL45[2][2]= cos(th[10]);
	TL45[2][3]= -L2 ;

	TL45[3][0]= 0;
	TL45[3][1]= 0;
	TL45[3][2]= 0;
	TL45[3][3]= 1;
	

	
	TL56[0][0]= 1;
	TL56[0][1]= 0;
	TL56[0][2]= 0;
	TL56[0][3]= -0.0241;

	TL56[1][0]= 0;
	TL56[1][1]= cos(th[11]);
	TL56[1][2]= -sin(th[11]);
	TL56[1][3]= -0.019;

	TL56[2][0]= 0;
	TL56[2][1]= sin(th[11]);
	TL56[2][2]= cos(th[11]);
	TL56[2][3]= 0;

	TL56[3][0]= 0;
	TL56[3][1]= 0;
	TL56[3][2]= 0;
	TL56[3][3]= 1;
	
	
	TL6E[0][0]= 1;
	TL6E[0][1]= 0;
	TL6E[0][2]= 0;
	TL6E[0][3]= 0.024;

	TL6E[1][0]= 0;
	TL6E[1][1]= 1;
	TL6E[1][2]= 0;
	TL6E[1][3]= 0;//0.0125;//0;//

	TL6E[2][0]= 0;
	TL6E[2][1]= 0;
	TL6E[2][2]= 1;
	TL6E[2][3]= -LF;

	TL6E[3][0]= 0;
	TL6E[3][1]= 0;
	TL6E[3][2]= 0;
	TL6E[3][3]= 1;
	
		
	matrixmultiply4(TWB, TRB1, TRW1);
	matrixmultiply4(TRW1, TR12, TRW2);
	matrixmultiply4(TRW2, TR23, TRW3);
	matrixmultiply4(TRW3, TR34, TRW4);
	matrixmultiply4(TRW4, TR45, TRW5);
	matrixmultiply4(TRW5, TR56, TRW6);
	matrixmultiply4(TRW6, TR6E, TRWE);
	matrixmultiply4(TWB, TLB1, TLW1);
	matrixmultiply4(TLW1, TL12, TLW2);
	matrixmultiply4(TLW2, TL23, TLW3);
	matrixmultiply4(TLW3, TL34, TLW4);
	matrixmultiply4(TLW4, TL45, TLW5);
	matrixmultiply4(TLW5, TL56, TLW6);
	matrixmultiply4(TLW6, TL6E, TLWE); 
}

void cross(double A[], double B[], double result[]) {
	result[0] = A[1] * B[2] - A[2] * B[1];
	result[1] = -A[0] * B[2] + A[2] * B[0];
	result[2] = A[0] * B[1] - A[1] * B[0];
}
void rot2omega(double r[][3], double result[]) {
	double el[3] = { 0, };
	double norm_el = 0;
	double trace = 0;
	double atanresult = 0;
	el[0] = r[2][1] - r[1][2];
	el[1] = r[0][2] - r[2][0];
	el[2] = r[1][0] - r[0][1];

	norm_el = sqrt(el[0] * el[0] + el[1] * el[1] + el[2] * el[2]);
	trace = r[0][0] + r[1][1] + r[2][2];
	atanresult = atan2(norm_el, trace - 1);
	if (norm_el > 0.0000000000000000001) {
		result[0] = atanresult / norm_el * el[0];
		result[1] = atanresult / norm_el * el[1];
		result[2] = atanresult / norm_el * el[2];
	}
	else if (r[0][0] > 0 && r[1][1] > 0 && r[2][2] > 0) {
		result[0] = 0;
		result[1] = 0;
		result[2] = 0;
	}
	else {
		result[0] = pi / 2 * (r[0][0] + 1);
		result[1] = pi / 2 * (r[1][1] + 1);
		result[2] = pi / 2 * (r[2][2] + 1);
	}
}

void errcalc(double targetp[], double targetr[][3], double curstate[][4], double err[]) {
	double perr[3] = { 0, };
	double rerr[3][3] = { 0, };
	double r2oresult[3] = { 0, };
	double werr[3] = { 0, };


	int i;

	for (i = 0; i < 3; i++) {
		perr[i] = targetp[i] - curstate[i][3];
	}

	freversematrixmultiply3(curstate, targetr, rerr);
	rot2omega(rerr, r2oresult);
	matrixmultiply331(curstate, r2oresult, werr);


	for (i = 0; i < 3; i++) {
		err[i] = perr[i];

	}
	for (i = 3; i < 6; i++) {
		err[i] = werr[i - 3];


	}
}

void inverse_solved(double a[6][6], double d[6][6]) {


    double A00 = a[0][0], A01 = a[0][1], A02 = a[0][2], A03 = a[0][3], A04 = a[0][4], A05 = a[0][5];
    double A10 = a[1][0], A11 = a[1][1], A12 = a[1][2], A13 = a[1][3], A14 = a[1][4], A15 = a[1][5];
    double A20 = a[2][0], A21 = a[2][1], A22 = a[2][2], A23 = a[2][3], A24 = a[2][4], A25 = a[2][5];
    double A30 = a[3][0], A31 = a[3][1], A32 = a[3][2], A33 = a[3][3], A34 = a[3][4], A35 = a[3][5];
    double A40 = a[4][0], A41 = a[4][1], A42 = a[4][2], A43 = a[4][3], A44 = a[4][4], A45 = a[4][5];
    double A50 = a[5][0], A51 = a[5][1], A52 = a[5][2], A53 = a[5][3], A54 = a[5][4], A55 = a[5][5];

    double detA01 = (+A00 * A11 * A22 * A33 * A44 * A55 - A00 * A11 * A22 * A33 * A45 * A54 - A00 * A11 * A22 * A34 * A43 * A55 + A00 * A11 * A22 * A34 * A45 * A53 + A00 * A11 * A22 * A35 * A43 * A54 - A00 * A11 * A22 * A35 * A44 * A53 - A00 * A11 * A23 * A32 * A44 * A55 + A00 * A11 * A23 * A32 * A45 * A54 + A00 * A11 * A23 * A34 * A42 * A55 - A00 * A11 * A23 * A34 * A45 * A52 - A00 * A11 * A23 * A35 * A42 * A54 + A00 * A11 * A23 * A35 * A44 * A52 + A00 * A11 * A24 * A32 * A43 * A55 - A00 * A11 * A24 * A32 * A45 * A53 - A00 * A11 * A24 * A33 * A42 * A55 + A00 * A11 * A24 * A33 * A45 * A52 + A00 * A11 * A24 * A35 * A42 * A53 - A00 * A11 * A24 * A35 * A43 * A52 - A00 * A11 * A25 * A32 * A43 * A54 + A00 * A11 * A25 * A32 * A44 * A53 + A00 * A11 * A25 * A33 * A42 * A54 - A00 * A11 * A25 * A33 * A44 * A52 - A00 * A11 * A25 * A34 * A42 * A53 + A00 * A11 * A25 * A34 * A43 * A52 - A00 * A12 * A21 * A33 * A44 * A55 + A00 * A12 * A21 * A33 * A45 * A54 + A00 * A12 * A21 * A34 * A43 * A55 - A00 * A12 * A21 * A34 * A45 * A53 - A00 * A12 * A21 * A35 * A43 * A54 + A00 * A12 * A21 * A35 * A44 * A53 + A00 * A12 * A23 * A31 * A44 * A55 - A00 * A12 * A23 * A31 * A45 * A54 - A00 * A12 * A23 * A34 * A41 * A55 + A00 * A12 * A23 * A34 * A45 * A51 + A00 * A12 * A23 * A35 * A41 * A54 - A00 * A12 * A23 * A35 * A44 * A51 - A00 * A12 * A24 * A31 * A43 * A55 + A00 * A12 * A24 * A31 * A45 * A53 + A00 * A12 * A24 * A33 * A41 * A55 - A00 * A12 * A24 * A33 * A45 * A51 - A00 * A12 * A24 * A35 * A41 * A53 + A00 * A12 * A24 * A35 * A43 * A51 + A00 * A12 * A25 * A31 * A43 * A54 - A00 * A12 * A25 * A31 * A44 * A53 - A00 * A12 * A25 * A33 * A41 * A54 + A00 * A12 * A25 * A33 * A44 * A51 + A00 * A12 * A25 * A34 * A41 * A53 - A00 * A12 * A25 * A34 * A43 * A51 + A00 * A13 * A21 * A32 * A44 * A55 - A00 * A13 * A21 * A32 * A45 * A54 - A00 * A13 * A21 * A34 * A42 * A55 + A00 * A13 * A21 * A34 * A45 * A52 + A00 * A13 * A21 * A35 * A42 * A54 - A00 * A13 * A21 * A35 * A44 * A52 - A00 * A13 * A22 * A31 * A44 * A55 + A00 * A13 * A22 * A31 * A45 * A54 + A00 * A13 * A22 * A34 * A41 * A55 - A00 * A13 * A22 * A34 * A45 * A51 - A00 * A13 * A22 * A35 * A41 * A54 + A00 * A13 * A22 * A35 * A44 * A51 + A00 * A13 * A24 * A31 * A42 * A55 - A00 * A13 * A24 * A31 * A45 * A52 - A00 * A13 * A24 * A32 * A41 * A55 + A00 * A13 * A24 * A32 * A45 * A51 + A00 * A13 * A24 * A35 * A41 * A52 - A00 * A13 * A24 * A35 * A42 * A51 - A00 * A13 * A25 * A31 * A42 * A54 + A00 * A13 * A25 * A31 * A44 * A52 + A00 * A13 * A25 * A32 * A41 * A54 - A00 * A13 * A25 * A32 * A44 * A51 - A00 * A13 * A25 * A34 * A41 * A52 + A00 * A13 * A25 * A34 * A42 * A51 - A00 * A14 * A21 * A32 * A43 * A55 + A00 * A14 * A21 * A32 * A45 * A53 + A00 * A14 * A21 * A33 * A42 * A55 - A00 * A14 * A21 * A33 * A45 * A52 - A00 * A14 * A21 * A35 * A42 * A53 + A00 * A14 * A21 * A35 * A43 * A52 + A00 * A14 * A22 * A31 * A43 * A55 - A00 * A14 * A22 * A31 * A45 * A53 - A00 * A14 * A22 * A33 * A41 * A55 + A00 * A14 * A22 * A33 * A45 * A51 + A00 * A14 * A22 * A35 * A41 * A53 - A00 * A14 * A22 * A35 * A43 * A51 - A00 * A14 * A23 * A31 * A42 * A55 + A00 * A14 * A23 * A31 * A45 * A52 + A00 * A14 * A23 * A32 * A41 * A55 - A00 * A14 * A23 * A32 * A45 * A51 - A00 * A14 * A23 * A35 * A41 * A52 + A00 * A14 * A23 * A35 * A42 * A51 + A00 * A14 * A25 * A31 * A42 * A53 - A00 * A14 * A25 * A31 * A43 * A52 - A00 * A14 * A25 * A32 * A41 * A53 + A00 * A14 * A25 * A32 * A43 * A51 + A00 * A14 * A25 * A33 * A41 * A52 - A00 * A14 * A25 * A33 * A42 * A51 + A00 * A15 * A21 * A32 * A43 * A54 - A00 * A15 * A21 * A32 * A44 * A53 - A00 * A15 * A21 * A33 * A42 * A54 + A00 * A15 * A21 * A33 * A44 * A52);
    double detA02 = (+A00 * A15 * A21 * A34 * A42 * A53 - A00 * A15 * A21 * A34 * A43 * A52 - A00 * A15 * A22 * A31 * A43 * A54 + A00 * A15 * A22 * A31 * A44 * A53 + A00 * A15 * A22 * A33 * A41 * A54 - A00 * A15 * A22 * A33 * A44 * A51 - A00 * A15 * A22 * A34 * A41 * A53 + A00 * A15 * A22 * A34 * A43 * A51 + A00 * A15 * A23 * A31 * A42 * A54 - A00 * A15 * A23 * A31 * A44 * A52 - A00 * A15 * A23 * A32 * A41 * A54 + A00 * A15 * A23 * A32 * A44 * A51 + A00 * A15 * A23 * A34 * A41 * A52 - A00 * A15 * A23 * A34 * A42 * A51 - A00 * A15 * A24 * A31 * A42 * A53 + A00 * A15 * A24 * A31 * A43 * A52 + A00 * A15 * A24 * A32 * A41 * A53 - A00 * A15 * A24 * A32 * A43 * A51 - A00 * A15 * A24 * A33 * A41 * A52 + A00 * A15 * A24 * A33 * A42 * A51 - A01 * A10 * A22 * A33 * A44 * A55 + A01 * A10 * A22 * A33 * A45 * A54 + A01 * A10 * A22 * A34 * A43 * A55 - A01 * A10 * A22 * A34 * A45 * A53 - A01 * A10 * A22 * A35 * A43 * A54 + A01 * A10 * A22 * A35 * A44 * A53 + A01 * A10 * A23 * A32 * A44 * A55 - A01 * A10 * A23 * A32 * A45 * A54 - A01 * A10 * A23 * A34 * A42 * A55 + A01 * A10 * A23 * A34 * A45 * A52 + A01 * A10 * A23 * A35 * A42 * A54 - A01 * A10 * A23 * A35 * A44 * A52 - A01 * A10 * A24 * A32 * A43 * A55 + A01 * A10 * A24 * A32 * A45 * A53 + A01 * A10 * A24 * A33 * A42 * A55 - A01 * A10 * A24 * A33 * A45 * A52 - A01 * A10 * A24 * A35 * A42 * A53 + A01 * A10 * A24 * A35 * A43 * A52 + A01 * A10 * A25 * A32 * A43 * A54 - A01 * A10 * A25 * A32 * A44 * A53 - A01 * A10 * A25 * A33 * A42 * A54 + A01 * A10 * A25 * A33 * A44 * A52 + A01 * A10 * A25 * A34 * A42 * A53 - A01 * A10 * A25 * A34 * A43 * A52 + A01 * A12 * A20 * A33 * A44 * A55 - A01 * A12 * A20 * A33 * A45 * A54 - A01 * A12 * A20 * A34 * A43 * A55 + A01 * A12 * A20 * A34 * A45 * A53 + A01 * A12 * A20 * A35 * A43 * A54 - A01 * A12 * A20 * A35 * A44 * A53 - A01 * A12 * A23 * A30 * A44 * A55 + A01 * A12 * A23 * A30 * A45 * A54 + A01 * A12 * A23 * A34 * A40 * A55 - A01 * A12 * A23 * A34 * A45 * A50 - A01 * A12 * A23 * A35 * A40 * A54 + A01 * A12 * A23 * A35 * A44 * A50 + A01 * A12 * A24 * A30 * A43 * A55 - A01 * A12 * A24 * A30 * A45 * A53 - A01 * A12 * A24 * A33 * A40 * A55 + A01 * A12 * A24 * A33 * A45 * A50 + A01 * A12 * A24 * A35 * A40 * A53 - A01 * A12 * A24 * A35 * A43 * A50 - A01 * A12 * A25 * A30 * A43 * A54 + A01 * A12 * A25 * A30 * A44 * A53 + A01 * A12 * A25 * A33 * A40 * A54 - A01 * A12 * A25 * A33 * A44 * A50 - A01 * A12 * A25 * A34 * A40 * A53 + A01 * A12 * A25 * A34 * A43 * A50 - A01 * A13 * A20 * A32 * A44 * A55 + A01 * A13 * A20 * A32 * A45 * A54 + A01 * A13 * A20 * A34 * A42 * A55 - A01 * A13 * A20 * A34 * A45 * A52 - A01 * A13 * A20 * A35 * A42 * A54 + A01 * A13 * A20 * A35 * A44 * A52 + A01 * A13 * A22 * A30 * A44 * A55 - A01 * A13 * A22 * A30 * A45 * A54 - A01 * A13 * A22 * A34 * A40 * A55 + A01 * A13 * A22 * A34 * A45 * A50 + A01 * A13 * A22 * A35 * A40 * A54 - A01 * A13 * A22 * A35 * A44 * A50 - A01 * A13 * A24 * A30 * A42 * A55 + A01 * A13 * A24 * A30 * A45 * A52 + A01 * A13 * A24 * A32 * A40 * A55 - A01 * A13 * A24 * A32 * A45 * A50 - A01 * A13 * A24 * A35 * A40 * A52 + A01 * A13 * A24 * A35 * A42 * A50 + A01 * A13 * A25 * A30 * A42 * A54 - A01 * A13 * A25 * A30 * A44 * A52 - A01 * A13 * A25 * A32 * A40 * A54 + A01 * A13 * A25 * A32 * A44 * A50 + A01 * A13 * A25 * A34 * A40 * A52 - A01 * A13 * A25 * A34 * A42 * A50 + A01 * A14 * A20 * A32 * A43 * A55 - A01 * A14 * A20 * A32 * A45 * A53 - A01 * A14 * A20 * A33 * A42 * A55 + A01 * A14 * A20 * A33 * A45 * A52 + A01 * A14 * A20 * A35 * A42 * A53 - A01 * A14 * A20 * A35 * A43 * A52 - A01 * A14 * A22 * A30 * A43 * A55 + A01 * A14 * A22 * A30 * A45 * A53);
    double detA03 = (+A01 * A14 * A22 * A33 * A40 * A55 - A01 * A14 * A22 * A33 * A45 * A50 - A01 * A14 * A22 * A35 * A40 * A53 + A01 * A14 * A22 * A35 * A43 * A50 + A01 * A14 * A23 * A30 * A42 * A55 - A01 * A14 * A23 * A30 * A45 * A52 - A01 * A14 * A23 * A32 * A40 * A55 + A01 * A14 * A23 * A32 * A45 * A50 + A01 * A14 * A23 * A35 * A40 * A52 - A01 * A14 * A23 * A35 * A42 * A50 - A01 * A14 * A25 * A30 * A42 * A53 + A01 * A14 * A25 * A30 * A43 * A52 + A01 * A14 * A25 * A32 * A40 * A53 - A01 * A14 * A25 * A32 * A43 * A50 - A01 * A14 * A25 * A33 * A40 * A52 + A01 * A14 * A25 * A33 * A42 * A50 - A01 * A15 * A20 * A32 * A43 * A54 + A01 * A15 * A20 * A32 * A44 * A53 + A01 * A15 * A20 * A33 * A42 * A54 - A01 * A15 * A20 * A33 * A44 * A52 - A01 * A15 * A20 * A34 * A42 * A53 + A01 * A15 * A20 * A34 * A43 * A52 + A01 * A15 * A22 * A30 * A43 * A54 - A01 * A15 * A22 * A30 * A44 * A53 - A01 * A15 * A22 * A33 * A40 * A54 + A01 * A15 * A22 * A33 * A44 * A50 + A01 * A15 * A22 * A34 * A40 * A53 - A01 * A15 * A22 * A34 * A43 * A50 - A01 * A15 * A23 * A30 * A42 * A54 + A01 * A15 * A23 * A30 * A44 * A52 + A01 * A15 * A23 * A32 * A40 * A54 - A01 * A15 * A23 * A32 * A44 * A50 - A01 * A15 * A23 * A34 * A40 * A52 + A01 * A15 * A23 * A34 * A42 * A50 + A01 * A15 * A24 * A30 * A42 * A53 - A01 * A15 * A24 * A30 * A43 * A52 - A01 * A15 * A24 * A32 * A40 * A53 + A01 * A15 * A24 * A32 * A43 * A50 + A01 * A15 * A24 * A33 * A40 * A52 - A01 * A15 * A24 * A33 * A42 * A50 + A02 * A10 * A21 * A33 * A44 * A55 - A02 * A10 * A21 * A33 * A45 * A54 - A02 * A10 * A21 * A34 * A43 * A55 + A02 * A10 * A21 * A34 * A45 * A53 + A02 * A10 * A21 * A35 * A43 * A54 - A02 * A10 * A21 * A35 * A44 * A53 - A02 * A10 * A23 * A31 * A44 * A55 + A02 * A10 * A23 * A31 * A45 * A54 + A02 * A10 * A23 * A34 * A41 * A55 - A02 * A10 * A23 * A34 * A45 * A51 - A02 * A10 * A23 * A35 * A41 * A54 + A02 * A10 * A23 * A35 * A44 * A51 + A02 * A10 * A24 * A31 * A43 * A55 - A02 * A10 * A24 * A31 * A45 * A53 - A02 * A10 * A24 * A33 * A41 * A55 + A02 * A10 * A24 * A33 * A45 * A51 + A02 * A10 * A24 * A35 * A41 * A53 - A02 * A10 * A24 * A35 * A43 * A51 - A02 * A10 * A25 * A31 * A43 * A54 + A02 * A10 * A25 * A31 * A44 * A53 + A02 * A10 * A25 * A33 * A41 * A54 - A02 * A10 * A25 * A33 * A44 * A51 - A02 * A10 * A25 * A34 * A41 * A53 + A02 * A10 * A25 * A34 * A43 * A51 - A02 * A11 * A20 * A33 * A44 * A55 + A02 * A11 * A20 * A33 * A45 * A54 + A02 * A11 * A20 * A34 * A43 * A55 - A02 * A11 * A20 * A34 * A45 * A53 - A02 * A11 * A20 * A35 * A43 * A54 + A02 * A11 * A20 * A35 * A44 * A53 + A02 * A11 * A23 * A30 * A44 * A55 - A02 * A11 * A23 * A30 * A45 * A54 - A02 * A11 * A23 * A34 * A40 * A55 + A02 * A11 * A23 * A34 * A45 * A50 + A02 * A11 * A23 * A35 * A40 * A54 - A02 * A11 * A23 * A35 * A44 * A50 - A02 * A11 * A24 * A30 * A43 * A55 + A02 * A11 * A24 * A30 * A45 * A53 + A02 * A11 * A24 * A33 * A40 * A55 - A02 * A11 * A24 * A33 * A45 * A50 - A02 * A11 * A24 * A35 * A40 * A53 + A02 * A11 * A24 * A35 * A43 * A50 + A02 * A11 * A25 * A30 * A43 * A54 - A02 * A11 * A25 * A30 * A44 * A53 - A02 * A11 * A25 * A33 * A40 * A54 + A02 * A11 * A25 * A33 * A44 * A50 + A02 * A11 * A25 * A34 * A40 * A53 - A02 * A11 * A25 * A34 * A43 * A50 + A02 * A13 * A20 * A31 * A44 * A55 - A02 * A13 * A20 * A31 * A45 * A54 - A02 * A13 * A20 * A34 * A41 * A55 + A02 * A13 * A20 * A34 * A45 * A51 + A02 * A13 * A20 * A35 * A41 * A54 - A02 * A13 * A20 * A35 * A44 * A51 - A02 * A13 * A21 * A30 * A44 * A55 + A02 * A13 * A21 * A30 * A45 * A54 + A02 * A13 * A21 * A34 * A40 * A55 - A02 * A13 * A21 * A34 * A45 * A50 - A02 * A13 * A21 * A35 * A40 * A54 + A02 * A13 * A21 * A35 * A44 * A50);
    double detA04 = (+A02 * A13 * A24 * A30 * A41 * A55 - A02 * A13 * A24 * A30 * A45 * A51 - A02 * A13 * A24 * A31 * A40 * A55 + A02 * A13 * A24 * A31 * A45 * A50 + A02 * A13 * A24 * A35 * A40 * A51 - A02 * A13 * A24 * A35 * A41 * A50 - A02 * A13 * A25 * A30 * A41 * A54 + A02 * A13 * A25 * A30 * A44 * A51 + A02 * A13 * A25 * A31 * A40 * A54 - A02 * A13 * A25 * A31 * A44 * A50 - A02 * A13 * A25 * A34 * A40 * A51 + A02 * A13 * A25 * A34 * A41 * A50 - A02 * A14 * A20 * A31 * A43 * A55 + A02 * A14 * A20 * A31 * A45 * A53 + A02 * A14 * A20 * A33 * A41 * A55 - A02 * A14 * A20 * A33 * A45 * A51 - A02 * A14 * A20 * A35 * A41 * A53 + A02 * A14 * A20 * A35 * A43 * A51 + A02 * A14 * A21 * A30 * A43 * A55 - A02 * A14 * A21 * A30 * A45 * A53 - A02 * A14 * A21 * A33 * A40 * A55 + A02 * A14 * A21 * A33 * A45 * A50 + A02 * A14 * A21 * A35 * A40 * A53 - A02 * A14 * A21 * A35 * A43 * A50 - A02 * A14 * A23 * A30 * A41 * A55 + A02 * A14 * A23 * A30 * A45 * A51 + A02 * A14 * A23 * A31 * A40 * A55 - A02 * A14 * A23 * A31 * A45 * A50 - A02 * A14 * A23 * A35 * A40 * A51 + A02 * A14 * A23 * A35 * A41 * A50 + A02 * A14 * A25 * A30 * A41 * A53 - A02 * A14 * A25 * A30 * A43 * A51 - A02 * A14 * A25 * A31 * A40 * A53 + A02 * A14 * A25 * A31 * A43 * A50 + A02 * A14 * A25 * A33 * A40 * A51 - A02 * A14 * A25 * A33 * A41 * A50 + A02 * A15 * A20 * A31 * A43 * A54 - A02 * A15 * A20 * A31 * A44 * A53 - A02 * A15 * A20 * A33 * A41 * A54 + A02 * A15 * A20 * A33 * A44 * A51 + A02 * A15 * A20 * A34 * A41 * A53 - A02 * A15 * A20 * A34 * A43 * A51 - A02 * A15 * A21 * A30 * A43 * A54 + A02 * A15 * A21 * A30 * A44 * A53 + A02 * A15 * A21 * A33 * A40 * A54 - A02 * A15 * A21 * A33 * A44 * A50 - A02 * A15 * A21 * A34 * A40 * A53 + A02 * A15 * A21 * A34 * A43 * A50 + A02 * A15 * A23 * A30 * A41 * A54 - A02 * A15 * A23 * A30 * A44 * A51 - A02 * A15 * A23 * A31 * A40 * A54 + A02 * A15 * A23 * A31 * A44 * A50 + A02 * A15 * A23 * A34 * A40 * A51 - A02 * A15 * A23 * A34 * A41 * A50 - A02 * A15 * A24 * A30 * A41 * A53 + A02 * A15 * A24 * A30 * A43 * A51 + A02 * A15 * A24 * A31 * A40 * A53 - A02 * A15 * A24 * A31 * A43 * A50 - A02 * A15 * A24 * A33 * A40 * A51 + A02 * A15 * A24 * A33 * A41 * A50 - A03 * A10 * A21 * A32 * A44 * A55 + A03 * A10 * A21 * A32 * A45 * A54 + A03 * A10 * A21 * A34 * A42 * A55 - A03 * A10 * A21 * A34 * A45 * A52 - A03 * A10 * A21 * A35 * A42 * A54 + A03 * A10 * A21 * A35 * A44 * A52 + A03 * A10 * A22 * A31 * A44 * A55 - A03 * A10 * A22 * A31 * A45 * A54 - A03 * A10 * A22 * A34 * A41 * A55 + A03 * A10 * A22 * A34 * A45 * A51 + A03 * A10 * A22 * A35 * A41 * A54 - A03 * A10 * A22 * A35 * A44 * A51 - A03 * A10 * A24 * A31 * A42 * A55 + A03 * A10 * A24 * A31 * A45 * A52 + A03 * A10 * A24 * A32 * A41 * A55 - A03 * A10 * A24 * A32 * A45 * A51 - A03 * A10 * A24 * A35 * A41 * A52 + A03 * A10 * A24 * A35 * A42 * A51 + A03 * A10 * A25 * A31 * A42 * A54 - A03 * A10 * A25 * A31 * A44 * A52 - A03 * A10 * A25 * A32 * A41 * A54 + A03 * A10 * A25 * A32 * A44 * A51 + A03 * A10 * A25 * A34 * A41 * A52 - A03 * A10 * A25 * A34 * A42 * A51 + A03 * A11 * A20 * A32 * A44 * A55 - A03 * A11 * A20 * A32 * A45 * A54 - A03 * A11 * A20 * A34 * A42 * A55 + A03 * A11 * A20 * A34 * A45 * A52 + A03 * A11 * A20 * A35 * A42 * A54 - A03 * A11 * A20 * A35 * A44 * A52 - A03 * A11 * A22 * A30 * A44 * A55 + A03 * A11 * A22 * A30 * A45 * A54 + A03 * A11 * A22 * A34 * A40 * A55 - A03 * A11 * A22 * A34 * A45 * A50 - A03 * A11 * A22 * A35 * A40 * A54 + A03 * A11 * A22 * A35 * A44 * A50 + A03 * A11 * A24 * A30 * A42 * A55 - A03 * A11 * A24 * A30 * A45 * A52 - A03 * A11 * A24 * A32 * A40 * A55 + A03 * A11 * A24 * A32 * A45 * A50);
    double detA05 = (+A03 * A11 * A24 * A35 * A40 * A52 - A03 * A11 * A24 * A35 * A42 * A50 - A03 * A11 * A25 * A30 * A42 * A54 + A03 * A11 * A25 * A30 * A44 * A52 + A03 * A11 * A25 * A32 * A40 * A54 - A03 * A11 * A25 * A32 * A44 * A50 - A03 * A11 * A25 * A34 * A40 * A52 + A03 * A11 * A25 * A34 * A42 * A50 - A03 * A12 * A20 * A31 * A44 * A55 + A03 * A12 * A20 * A31 * A45 * A54 + A03 * A12 * A20 * A34 * A41 * A55 - A03 * A12 * A20 * A34 * A45 * A51 - A03 * A12 * A20 * A35 * A41 * A54 + A03 * A12 * A20 * A35 * A44 * A51 + A03 * A12 * A21 * A30 * A44 * A55 - A03 * A12 * A21 * A30 * A45 * A54 - A03 * A12 * A21 * A34 * A40 * A55 + A03 * A12 * A21 * A34 * A45 * A50 + A03 * A12 * A21 * A35 * A40 * A54 - A03 * A12 * A21 * A35 * A44 * A50 - A03 * A12 * A24 * A30 * A41 * A55 + A03 * A12 * A24 * A30 * A45 * A51 + A03 * A12 * A24 * A31 * A40 * A55 - A03 * A12 * A24 * A31 * A45 * A50 - A03 * A12 * A24 * A35 * A40 * A51 + A03 * A12 * A24 * A35 * A41 * A50 + A03 * A12 * A25 * A30 * A41 * A54 - A03 * A12 * A25 * A30 * A44 * A51 - A03 * A12 * A25 * A31 * A40 * A54 + A03 * A12 * A25 * A31 * A44 * A50 + A03 * A12 * A25 * A34 * A40 * A51 - A03 * A12 * A25 * A34 * A41 * A50 + A03 * A14 * A20 * A31 * A42 * A55 - A03 * A14 * A20 * A31 * A45 * A52 - A03 * A14 * A20 * A32 * A41 * A55 + A03 * A14 * A20 * A32 * A45 * A51 + A03 * A14 * A20 * A35 * A41 * A52 - A03 * A14 * A20 * A35 * A42 * A51 - A03 * A14 * A21 * A30 * A42 * A55 + A03 * A14 * A21 * A30 * A45 * A52 + A03 * A14 * A21 * A32 * A40 * A55 - A03 * A14 * A21 * A32 * A45 * A50 - A03 * A14 * A21 * A35 * A40 * A52 + A03 * A14 * A21 * A35 * A42 * A50 + A03 * A14 * A22 * A30 * A41 * A55 - A03 * A14 * A22 * A30 * A45 * A51 - A03 * A14 * A22 * A31 * A40 * A55 + A03 * A14 * A22 * A31 * A45 * A50 + A03 * A14 * A22 * A35 * A40 * A51 - A03 * A14 * A22 * A35 * A41 * A50 - A03 * A14 * A25 * A30 * A41 * A52 + A03 * A14 * A25 * A30 * A42 * A51 + A03 * A14 * A25 * A31 * A40 * A52 - A03 * A14 * A25 * A31 * A42 * A50 - A03 * A14 * A25 * A32 * A40 * A51 + A03 * A14 * A25 * A32 * A41 * A50 - A03 * A15 * A20 * A31 * A42 * A54 + A03 * A15 * A20 * A31 * A44 * A52 + A03 * A15 * A20 * A32 * A41 * A54 - A03 * A15 * A20 * A32 * A44 * A51 - A03 * A15 * A20 * A34 * A41 * A52 + A03 * A15 * A20 * A34 * A42 * A51 + A03 * A15 * A21 * A30 * A42 * A54 - A03 * A15 * A21 * A30 * A44 * A52 - A03 * A15 * A21 * A32 * A40 * A54 + A03 * A15 * A21 * A32 * A44 * A50 + A03 * A15 * A21 * A34 * A40 * A52 - A03 * A15 * A21 * A34 * A42 * A50 - A03 * A15 * A22 * A30 * A41 * A54 + A03 * A15 * A22 * A30 * A44 * A51 + A03 * A15 * A22 * A31 * A40 * A54 - A03 * A15 * A22 * A31 * A44 * A50 - A03 * A15 * A22 * A34 * A40 * A51 + A03 * A15 * A22 * A34 * A41 * A50 + A03 * A15 * A24 * A30 * A41 * A52 - A03 * A15 * A24 * A30 * A42 * A51 - A03 * A15 * A24 * A31 * A40 * A52 + A03 * A15 * A24 * A31 * A42 * A50 + A03 * A15 * A24 * A32 * A40 * A51 - A03 * A15 * A24 * A32 * A41 * A50 + A04 * A10 * A21 * A32 * A43 * A55 - A04 * A10 * A21 * A32 * A45 * A53 - A04 * A10 * A21 * A33 * A42 * A55 + A04 * A10 * A21 * A33 * A45 * A52 + A04 * A10 * A21 * A35 * A42 * A53 - A04 * A10 * A21 * A35 * A43 * A52 - A04 * A10 * A22 * A31 * A43 * A55 + A04 * A10 * A22 * A31 * A45 * A53 + A04 * A10 * A22 * A33 * A41 * A55 - A04 * A10 * A22 * A33 * A45 * A51 - A04 * A10 * A22 * A35 * A41 * A53 + A04 * A10 * A22 * A35 * A43 * A51 + A04 * A10 * A23 * A31 * A42 * A55 - A04 * A10 * A23 * A31 * A45 * A52 - A04 * A10 * A23 * A32 * A41 * A55 + A04 * A10 * A23 * A32 * A45 * A51 + A04 * A10 * A23 * A35 * A41 * A52 - A04 * A10 * A23 * A35 * A42 * A51 - A04 * A10 * A25 * A31 * A42 * A53 + A04 * A10 * A25 * A31 * A43 * A52);
    double detA06 = (+A04 * A10 * A25 * A32 * A41 * A53 - A04 * A10 * A25 * A32 * A43 * A51 - A04 * A10 * A25 * A33 * A41 * A52 + A04 * A10 * A25 * A33 * A42 * A51 - A04 * A11 * A20 * A32 * A43 * A55 + A04 * A11 * A20 * A32 * A45 * A53 + A04 * A11 * A20 * A33 * A42 * A55 - A04 * A11 * A20 * A33 * A45 * A52 - A04 * A11 * A20 * A35 * A42 * A53 + A04 * A11 * A20 * A35 * A43 * A52 + A04 * A11 * A22 * A30 * A43 * A55 - A04 * A11 * A22 * A30 * A45 * A53 - A04 * A11 * A22 * A33 * A40 * A55 + A04 * A11 * A22 * A33 * A45 * A50 + A04 * A11 * A22 * A35 * A40 * A53 - A04 * A11 * A22 * A35 * A43 * A50 - A04 * A11 * A23 * A30 * A42 * A55 + A04 * A11 * A23 * A30 * A45 * A52 + A04 * A11 * A23 * A32 * A40 * A55 - A04 * A11 * A23 * A32 * A45 * A50 - A04 * A11 * A23 * A35 * A40 * A52 + A04 * A11 * A23 * A35 * A42 * A50 + A04 * A11 * A25 * A30 * A42 * A53 - A04 * A11 * A25 * A30 * A43 * A52 - A04 * A11 * A25 * A32 * A40 * A53 + A04 * A11 * A25 * A32 * A43 * A50 + A04 * A11 * A25 * A33 * A40 * A52 - A04 * A11 * A25 * A33 * A42 * A50 + A04 * A12 * A20 * A31 * A43 * A55 - A04 * A12 * A20 * A31 * A45 * A53 - A04 * A12 * A20 * A33 * A41 * A55 + A04 * A12 * A20 * A33 * A45 * A51 + A04 * A12 * A20 * A35 * A41 * A53 - A04 * A12 * A20 * A35 * A43 * A51 - A04 * A12 * A21 * A30 * A43 * A55 + A04 * A12 * A21 * A30 * A45 * A53 + A04 * A12 * A21 * A33 * A40 * A55 - A04 * A12 * A21 * A33 * A45 * A50 - A04 * A12 * A21 * A35 * A40 * A53 + A04 * A12 * A21 * A35 * A43 * A50 + A04 * A12 * A23 * A30 * A41 * A55 - A04 * A12 * A23 * A30 * A45 * A51 - A04 * A12 * A23 * A31 * A40 * A55 + A04 * A12 * A23 * A31 * A45 * A50 + A04 * A12 * A23 * A35 * A40 * A51 - A04 * A12 * A23 * A35 * A41 * A50 - A04 * A12 * A25 * A30 * A41 * A53 + A04 * A12 * A25 * A30 * A43 * A51 + A04 * A12 * A25 * A31 * A40 * A53 - A04 * A12 * A25 * A31 * A43 * A50 - A04 * A12 * A25 * A33 * A40 * A51 + A04 * A12 * A25 * A33 * A41 * A50 - A04 * A13 * A20 * A31 * A42 * A55 + A04 * A13 * A20 * A31 * A45 * A52 + A04 * A13 * A20 * A32 * A41 * A55 - A04 * A13 * A20 * A32 * A45 * A51 - A04 * A13 * A20 * A35 * A41 * A52 + A04 * A13 * A20 * A35 * A42 * A51 + A04 * A13 * A21 * A30 * A42 * A55 - A04 * A13 * A21 * A30 * A45 * A52 - A04 * A13 * A21 * A32 * A40 * A55 + A04 * A13 * A21 * A32 * A45 * A50 + A04 * A13 * A21 * A35 * A40 * A52 - A04 * A13 * A21 * A35 * A42 * A50 - A04 * A13 * A22 * A30 * A41 * A55 + A04 * A13 * A22 * A30 * A45 * A51 + A04 * A13 * A22 * A31 * A40 * A55 - A04 * A13 * A22 * A31 * A45 * A50 - A04 * A13 * A22 * A35 * A40 * A51 + A04 * A13 * A22 * A35 * A41 * A50 + A04 * A13 * A25 * A30 * A41 * A52 - A04 * A13 * A25 * A30 * A42 * A51 - A04 * A13 * A25 * A31 * A40 * A52 + A04 * A13 * A25 * A31 * A42 * A50 + A04 * A13 * A25 * A32 * A40 * A51 - A04 * A13 * A25 * A32 * A41 * A50 + A04 * A15 * A20 * A31 * A42 * A53 - A04 * A15 * A20 * A31 * A43 * A52 - A04 * A15 * A20 * A32 * A41 * A53 + A04 * A15 * A20 * A32 * A43 * A51 + A04 * A15 * A20 * A33 * A41 * A52 - A04 * A15 * A20 * A33 * A42 * A51 - A04 * A15 * A21 * A30 * A42 * A53 + A04 * A15 * A21 * A30 * A43 * A52 + A04 * A15 * A21 * A32 * A40 * A53 - A04 * A15 * A21 * A32 * A43 * A50 - A04 * A15 * A21 * A33 * A40 * A52 + A04 * A15 * A21 * A33 * A42 * A50 + A04 * A15 * A22 * A30 * A41 * A53 - A04 * A15 * A22 * A30 * A43 * A51 - A04 * A15 * A22 * A31 * A40 * A53 + A04 * A15 * A22 * A31 * A43 * A50 + A04 * A15 * A22 * A33 * A40 * A51 - A04 * A15 * A22 * A33 * A41 * A50 - A04 * A15 * A23 * A30 * A41 * A52 + A04 * A15 * A23 * A30 * A42 * A51 + A04 * A15 * A23 * A31 * A40 * A52 - A04 * A15 * A23 * A31 * A42 * A50 - A04 * A15 * A23 * A32 * A40 * A51 + A04 * A15 * A23 * A32 * A41 * A50);
    double detA07 = (-A05 * A10 * A21 * A32 * A43 * A54 + A05 * A10 * A21 * A32 * A44 * A53 + A05 * A10 * A21 * A33 * A42 * A54 - A05 * A10 * A21 * A33 * A44 * A52 - A05 * A10 * A21 * A34 * A42 * A53 + A05 * A10 * A21 * A34 * A43 * A52 + A05 * A10 * A22 * A31 * A43 * A54 - A05 * A10 * A22 * A31 * A44 * A53 - A05 * A10 * A22 * A33 * A41 * A54 + A05 * A10 * A22 * A33 * A44 * A51 + A05 * A10 * A22 * A34 * A41 * A53 - A05 * A10 * A22 * A34 * A43 * A51 - A05 * A10 * A23 * A31 * A42 * A54 + A05 * A10 * A23 * A31 * A44 * A52 + A05 * A10 * A23 * A32 * A41 * A54 - A05 * A10 * A23 * A32 * A44 * A51 - A05 * A10 * A23 * A34 * A41 * A52 + A05 * A10 * A23 * A34 * A42 * A51 + A05 * A10 * A24 * A31 * A42 * A53 - A05 * A10 * A24 * A31 * A43 * A52 - A05 * A10 * A24 * A32 * A41 * A53 + A05 * A10 * A24 * A32 * A43 * A51 + A05 * A10 * A24 * A33 * A41 * A52 - A05 * A10 * A24 * A33 * A42 * A51 + A05 * A11 * A20 * A32 * A43 * A54 - A05 * A11 * A20 * A32 * A44 * A53 - A05 * A11 * A20 * A33 * A42 * A54 + A05 * A11 * A20 * A33 * A44 * A52 + A05 * A11 * A20 * A34 * A42 * A53 - A05 * A11 * A20 * A34 * A43 * A52 - A05 * A11 * A22 * A30 * A43 * A54 + A05 * A11 * A22 * A30 * A44 * A53 + A05 * A11 * A22 * A33 * A40 * A54 - A05 * A11 * A22 * A33 * A44 * A50 - A05 * A11 * A22 * A34 * A40 * A53 + A05 * A11 * A22 * A34 * A43 * A50 + A05 * A11 * A23 * A30 * A42 * A54 - A05 * A11 * A23 * A30 * A44 * A52 - A05 * A11 * A23 * A32 * A40 * A54 + A05 * A11 * A23 * A32 * A44 * A50 + A05 * A11 * A23 * A34 * A40 * A52 - A05 * A11 * A23 * A34 * A42 * A50 - A05 * A11 * A24 * A30 * A42 * A53 + A05 * A11 * A24 * A30 * A43 * A52 + A05 * A11 * A24 * A32 * A40 * A53 - A05 * A11 * A24 * A32 * A43 * A50 - A05 * A11 * A24 * A33 * A40 * A52 + A05 * A11 * A24 * A33 * A42 * A50 - A05 * A12 * A20 * A31 * A43 * A54 + A05 * A12 * A20 * A31 * A44 * A53 + A05 * A12 * A20 * A33 * A41 * A54 - A05 * A12 * A20 * A33 * A44 * A51 - A05 * A12 * A20 * A34 * A41 * A53 + A05 * A12 * A20 * A34 * A43 * A51 + A05 * A12 * A21 * A30 * A43 * A54 - A05 * A12 * A21 * A30 * A44 * A53 - A05 * A12 * A21 * A33 * A40 * A54 + A05 * A12 * A21 * A33 * A44 * A50 + A05 * A12 * A21 * A34 * A40 * A53 - A05 * A12 * A21 * A34 * A43 * A50 - A05 * A12 * A23 * A30 * A41 * A54 + A05 * A12 * A23 * A30 * A44 * A51 + A05 * A12 * A23 * A31 * A40 * A54 - A05 * A12 * A23 * A31 * A44 * A50 - A05 * A12 * A23 * A34 * A40 * A51 + A05 * A12 * A23 * A34 * A41 * A50 + A05 * A12 * A24 * A30 * A41 * A53 - A05 * A12 * A24 * A30 * A43 * A51 - A05 * A12 * A24 * A31 * A40 * A53 + A05 * A12 * A24 * A31 * A43 * A50 + A05 * A12 * A24 * A33 * A40 * A51 - A05 * A12 * A24 * A33 * A41 * A50 + A05 * A13 * A20 * A31 * A42 * A54 - A05 * A13 * A20 * A31 * A44 * A52 - A05 * A13 * A20 * A32 * A41 * A54 + A05 * A13 * A20 * A32 * A44 * A51 + A05 * A13 * A20 * A34 * A41 * A52 - A05 * A13 * A20 * A34 * A42 * A51 - A05 * A13 * A21 * A30 * A42 * A54 + A05 * A13 * A21 * A30 * A44 * A52 + A05 * A13 * A21 * A32 * A40 * A54 - A05 * A13 * A21 * A32 * A44 * A50 - A05 * A13 * A21 * A34 * A40 * A52 + A05 * A13 * A21 * A34 * A42 * A50 + A05 * A13 * A22 * A30 * A41 * A54 - A05 * A13 * A22 * A30 * A44 * A51 - A05 * A13 * A22 * A31 * A40 * A54 + A05 * A13 * A22 * A31 * A44 * A50 + A05 * A13 * A22 * A34 * A40 * A51 - A05 * A13 * A22 * A34 * A41 * A50 - A05 * A13 * A24 * A30 * A41 * A52 + A05 * A13 * A24 * A30 * A42 * A51 + A05 * A13 * A24 * A31 * A40 * A52 - A05 * A13 * A24 * A31 * A42 * A50 - A05 * A13 * A24 * A32 * A40 * A51 + A05 * A13 * A24 * A32 * A41 * A50 - A05 * A14 * A20 * A31 * A42 * A53 + A05 * A14 * A20 * A31 * A43 * A52 + A05 * A14 * A20 * A32 * A41 * A53 - A05 * A14 * A20 * A32 * A43 * A51);
    double detA08 = (-A05 * A14 * A20 * A33 * A41 * A52 + A05 * A14 * A20 * A33 * A42 * A51 + A05 * A14 * A21 * A30 * A42 * A53 - A05 * A14 * A21 * A30 * A43 * A52 - A05 * A14 * A21 * A32 * A40 * A53 + A05 * A14 * A21 * A32 * A43 * A50 + A05 * A14 * A21 * A33 * A40 * A52 - A05 * A14 * A21 * A33 * A42 * A50 - A05 * A14 * A22 * A30 * A41 * A53 + A05 * A14 * A22 * A30 * A43 * A51 + A05 * A14 * A22 * A31 * A40 * A53 - A05 * A14 * A22 * A31 * A43 * A50 - A05 * A14 * A22 * A33 * A40 * A51 + A05 * A14 * A22 * A33 * A41 * A50 + A05 * A14 * A23 * A30 * A41 * A52 - A05 * A14 * A23 * A30 * A42 * A51 - A05 * A14 * A23 * A31 * A40 * A52 + A05 * A14 * A23 * A31 * A42 * A50 + A05 * A14 * A23 * A32 * A40 * A51 - A05 * A14 * A23 * A32 * A41 * A50);

    double detA = detA01 + detA02 + detA03 + detA04 + detA05 + detA06 + detA07 + detA08;

    d[0][0] = (A11 * A22 * A33 * A44 * A55 - A11 * A22 * A33 * A45 * A54 - A11 * A22 * A34 * A43 * A55 + A11 * A22 * A34 * A45 * A53 + A11 * A22 * A35 * A43 * A54 - A11 * A22 * A35 * A44 * A53 - A11 * A23 * A32 * A44 * A55 + A11 * A23 * A32 * A45 * A54 + A11 * A23 * A34 * A42 * A55 - A11 * A23 * A34 * A45 * A52 - A11 * A23 * A35 * A42 * A54 + A11 * A23 * A35 * A44 * A52 + A11 * A24 * A32 * A43 * A55 - A11 * A24 * A32 * A45 * A53 - A11 * A24 * A33 * A42 * A55 + A11 * A24 * A33 * A45 * A52 + A11 * A24 * A35 * A42 * A53 - A11 * A24 * A35 * A43 * A52 - A11 * A25 * A32 * A43 * A54 + A11 * A25 * A32 * A44 * A53 + A11 * A25 * A33 * A42 * A54 - A11 * A25 * A33 * A44 * A52 - A11 * A25 * A34 * A42 * A53 + A11 * A25 * A34 * A43 * A52 - A12 * A21 * A33 * A44 * A55 + A12 * A21 * A33 * A45 * A54 + A12 * A21 * A34 * A43 * A55 - A12 * A21 * A34 * A45 * A53 - A12 * A21 * A35 * A43 * A54 + A12 * A21 * A35 * A44 * A53 + A12 * A23 * A31 * A44 * A55 - A12 * A23 * A31 * A45 * A54 - A12 * A23 * A34 * A41 * A55 + A12 * A23 * A34 * A45 * A51 + A12 * A23 * A35 * A41 * A54 - A12 * A23 * A35 * A44 * A51 - A12 * A24 * A31 * A43 * A55 + A12 * A24 * A31 * A45 * A53 + A12 * A24 * A33 * A41 * A55 - A12 * A24 * A33 * A45 * A51 - A12 * A24 * A35 * A41 * A53 + A12 * A24 * A35 * A43 * A51 + A12 * A25 * A31 * A43 * A54 - A12 * A25 * A31 * A44 * A53 - A12 * A25 * A33 * A41 * A54 + A12 * A25 * A33 * A44 * A51 + A12 * A25 * A34 * A41 * A53 - A12 * A25 * A34 * A43 * A51 + A13 * A21 * A32 * A44 * A55 - A13 * A21 * A32 * A45 * A54 - A13 * A21 * A34 * A42 * A55 + A13 * A21 * A34 * A45 * A52 + A13 * A21 * A35 * A42 * A54 - A13 * A21 * A35 * A44 * A52 - A13 * A22 * A31 * A44 * A55 + A13 * A22 * A31 * A45 * A54 + A13 * A22 * A34 * A41 * A55 - A13 * A22 * A34 * A45 * A51 - A13 * A22 * A35 * A41 * A54 + A13 * A22 * A35 * A44 * A51 + A13 * A24 * A31 * A42 * A55 - A13 * A24 * A31 * A45 * A52 - A13 * A24 * A32 * A41 * A55 + A13 * A24 * A32 * A45 * A51 + A13 * A24 * A35 * A41 * A52 - A13 * A24 * A35 * A42 * A51 - A13 * A25 * A31 * A42 * A54 + A13 * A25 * A31 * A44 * A52 + A13 * A25 * A32 * A41 * A54 - A13 * A25 * A32 * A44 * A51 - A13 * A25 * A34 * A41 * A52 + A13 * A25 * A34 * A42 * A51 - A14 * A21 * A32 * A43 * A55 + A14 * A21 * A32 * A45 * A53 + A14 * A21 * A33 * A42 * A55 - A14 * A21 * A33 * A45 * A52 - A14 * A21 * A35 * A42 * A53 + A14 * A21 * A35 * A43 * A52 + A14 * A22 * A31 * A43 * A55 - A14 * A22 * A31 * A45 * A53 - A14 * A22 * A33 * A41 * A55 + A14 * A22 * A33 * A45 * A51 + A14 * A22 * A35 * A41 * A53 - A14 * A22 * A35 * A43 * A51 - A14 * A23 * A31 * A42 * A55 + A14 * A23 * A31 * A45 * A52 + A14 * A23 * A32 * A41 * A55 - A14 * A23 * A32 * A45 * A51 - A14 * A23 * A35 * A41 * A52 + A14 * A23 * A35 * A42 * A51 + A14 * A25 * A31 * A42 * A53 - A14 * A25 * A31 * A43 * A52 - A14 * A25 * A32 * A41 * A53 + A14 * A25 * A32 * A43 * A51 + A14 * A25 * A33 * A41 * A52 - A14 * A25 * A33 * A42 * A51 + A15 * A21 * A32 * A43 * A54 - A15 * A21 * A32 * A44 * A53 - A15 * A21 * A33 * A42 * A54 + A15 * A21 * A33 * A44 * A52 + A15 * A21 * A34 * A42 * A53 - A15 * A21 * A34 * A43 * A52 - A15 * A22 * A31 * A43 * A54 + A15 * A22 * A31 * A44 * A53 + A15 * A22 * A33 * A41 * A54 - A15 * A22 * A33 * A44 * A51 - A15 * A22 * A34 * A41 * A53 + A15 * A22 * A34 * A43 * A51 + A15 * A23 * A31 * A42 * A54 - A15 * A23 * A31 * A44 * A52 - A15 * A23 * A32 * A41 * A54 + A15 * A23 * A32 * A44 * A51 + A15 * A23 * A34 * A41 * A52 - A15 * A23 * A34 * A42 * A51 - A15 * A24 * A31 * A42 * A53 + A15 * A24 * A31 * A43 * A52 + A15 * A24 * A32 * A41 * A53 - A15 * A24 * A32 * A43 * A51 - A15 * A24 * A33 * A41 * A52 + A15 * A24 * A33 * A42 * A51) / detA;
    d[0][1] = (A01 * A22 * A33 * A45 * A54 - A01 * A22 * A33 * A44 * A55 + A01 * A22 * A34 * A43 * A55 - A01 * A22 * A34 * A45 * A53 - A01 * A22 * A35 * A43 * A54 + A01 * A22 * A35 * A44 * A53 + A01 * A23 * A32 * A44 * A55 - A01 * A23 * A32 * A45 * A54 - A01 * A23 * A34 * A42 * A55 + A01 * A23 * A34 * A45 * A52 + A01 * A23 * A35 * A42 * A54 - A01 * A23 * A35 * A44 * A52 - A01 * A24 * A32 * A43 * A55 + A01 * A24 * A32 * A45 * A53 + A01 * A24 * A33 * A42 * A55 - A01 * A24 * A33 * A45 * A52 - A01 * A24 * A35 * A42 * A53 + A01 * A24 * A35 * A43 * A52 + A01 * A25 * A32 * A43 * A54 - A01 * A25 * A32 * A44 * A53 - A01 * A25 * A33 * A42 * A54 + A01 * A25 * A33 * A44 * A52 + A01 * A25 * A34 * A42 * A53 - A01 * A25 * A34 * A43 * A52 + A02 * A21 * A33 * A44 * A55 - A02 * A21 * A33 * A45 * A54 - A02 * A21 * A34 * A43 * A55 + A02 * A21 * A34 * A45 * A53 + A02 * A21 * A35 * A43 * A54 - A02 * A21 * A35 * A44 * A53 - A02 * A23 * A31 * A44 * A55 + A02 * A23 * A31 * A45 * A54 + A02 * A23 * A34 * A41 * A55 - A02 * A23 * A34 * A45 * A51 - A02 * A23 * A35 * A41 * A54 + A02 * A23 * A35 * A44 * A51 + A02 * A24 * A31 * A43 * A55 - A02 * A24 * A31 * A45 * A53 - A02 * A24 * A33 * A41 * A55 + A02 * A24 * A33 * A45 * A51 + A02 * A24 * A35 * A41 * A53 - A02 * A24 * A35 * A43 * A51 - A02 * A25 * A31 * A43 * A54 + A02 * A25 * A31 * A44 * A53 + A02 * A25 * A33 * A41 * A54 - A02 * A25 * A33 * A44 * A51 - A02 * A25 * A34 * A41 * A53 + A02 * A25 * A34 * A43 * A51 - A03 * A21 * A32 * A44 * A55 + A03 * A21 * A32 * A45 * A54 + A03 * A21 * A34 * A42 * A55 - A03 * A21 * A34 * A45 * A52 - A03 * A21 * A35 * A42 * A54 + A03 * A21 * A35 * A44 * A52 + A03 * A22 * A31 * A44 * A55 - A03 * A22 * A31 * A45 * A54 - A03 * A22 * A34 * A41 * A55 + A03 * A22 * A34 * A45 * A51 + A03 * A22 * A35 * A41 * A54 - A03 * A22 * A35 * A44 * A51 - A03 * A24 * A31 * A42 * A55 + A03 * A24 * A31 * A45 * A52 + A03 * A24 * A32 * A41 * A55 - A03 * A24 * A32 * A45 * A51 - A03 * A24 * A35 * A41 * A52 + A03 * A24 * A35 * A42 * A51 + A03 * A25 * A31 * A42 * A54 - A03 * A25 * A31 * A44 * A52 - A03 * A25 * A32 * A41 * A54 + A03 * A25 * A32 * A44 * A51 + A03 * A25 * A34 * A41 * A52 - A03 * A25 * A34 * A42 * A51 + A04 * A21 * A32 * A43 * A55 - A04 * A21 * A32 * A45 * A53 - A04 * A21 * A33 * A42 * A55 + A04 * A21 * A33 * A45 * A52 + A04 * A21 * A35 * A42 * A53 - A04 * A21 * A35 * A43 * A52 - A04 * A22 * A31 * A43 * A55 + A04 * A22 * A31 * A45 * A53 + A04 * A22 * A33 * A41 * A55 - A04 * A22 * A33 * A45 * A51 - A04 * A22 * A35 * A41 * A53 + A04 * A22 * A35 * A43 * A51 + A04 * A23 * A31 * A42 * A55 - A04 * A23 * A31 * A45 * A52 - A04 * A23 * A32 * A41 * A55 + A04 * A23 * A32 * A45 * A51 + A04 * A23 * A35 * A41 * A52 - A04 * A23 * A35 * A42 * A51 - A04 * A25 * A31 * A42 * A53 + A04 * A25 * A31 * A43 * A52 + A04 * A25 * A32 * A41 * A53 - A04 * A25 * A32 * A43 * A51 - A04 * A25 * A33 * A41 * A52 + A04 * A25 * A33 * A42 * A51 - A05 * A21 * A32 * A43 * A54 + A05 * A21 * A32 * A44 * A53 + A05 * A21 * A33 * A42 * A54 - A05 * A21 * A33 * A44 * A52 - A05 * A21 * A34 * A42 * A53 + A05 * A21 * A34 * A43 * A52 + A05 * A22 * A31 * A43 * A54 - A05 * A22 * A31 * A44 * A53 - A05 * A22 * A33 * A41 * A54 + A05 * A22 * A33 * A44 * A51 + A05 * A22 * A34 * A41 * A53 - A05 * A22 * A34 * A43 * A51 - A05 * A23 * A31 * A42 * A54 + A05 * A23 * A31 * A44 * A52 + A05 * A23 * A32 * A41 * A54 - A05 * A23 * A32 * A44 * A51 - A05 * A23 * A34 * A41 * A52 + A05 * A23 * A34 * A42 * A51 + A05 * A24 * A31 * A42 * A53 - A05 * A24 * A31 * A43 * A52 - A05 * A24 * A32 * A41 * A53 + A05 * A24 * A32 * A43 * A51 + A05 * A24 * A33 * A41 * A52 - A05 * A24 * A33 * A42 * A51) / detA;
    d[0][2] = (A01 * A12 * A33 * A44 * A55 - A01 * A12 * A33 * A45 * A54 - A01 * A12 * A34 * A43 * A55 + A01 * A12 * A34 * A45 * A53 + A01 * A12 * A35 * A43 * A54 - A01 * A12 * A35 * A44 * A53 - A01 * A13 * A32 * A44 * A55 + A01 * A13 * A32 * A45 * A54 + A01 * A13 * A34 * A42 * A55 - A01 * A13 * A34 * A45 * A52 - A01 * A13 * A35 * A42 * A54 + A01 * A13 * A35 * A44 * A52 + A01 * A14 * A32 * A43 * A55 - A01 * A14 * A32 * A45 * A53 - A01 * A14 * A33 * A42 * A55 + A01 * A14 * A33 * A45 * A52 + A01 * A14 * A35 * A42 * A53 - A01 * A14 * A35 * A43 * A52 - A01 * A15 * A32 * A43 * A54 + A01 * A15 * A32 * A44 * A53 + A01 * A15 * A33 * A42 * A54 - A01 * A15 * A33 * A44 * A52 - A01 * A15 * A34 * A42 * A53 + A01 * A15 * A34 * A43 * A52 - A02 * A11 * A33 * A44 * A55 + A02 * A11 * A33 * A45 * A54 + A02 * A11 * A34 * A43 * A55 - A02 * A11 * A34 * A45 * A53 - A02 * A11 * A35 * A43 * A54 + A02 * A11 * A35 * A44 * A53 + A02 * A13 * A31 * A44 * A55 - A02 * A13 * A31 * A45 * A54 - A02 * A13 * A34 * A41 * A55 + A02 * A13 * A34 * A45 * A51 + A02 * A13 * A35 * A41 * A54 - A02 * A13 * A35 * A44 * A51 - A02 * A14 * A31 * A43 * A55 + A02 * A14 * A31 * A45 * A53 + A02 * A14 * A33 * A41 * A55 - A02 * A14 * A33 * A45 * A51 - A02 * A14 * A35 * A41 * A53 + A02 * A14 * A35 * A43 * A51 + A02 * A15 * A31 * A43 * A54 - A02 * A15 * A31 * A44 * A53 - A02 * A15 * A33 * A41 * A54 + A02 * A15 * A33 * A44 * A51 + A02 * A15 * A34 * A41 * A53 - A02 * A15 * A34 * A43 * A51 + A03 * A11 * A32 * A44 * A55 - A03 * A11 * A32 * A45 * A54 - A03 * A11 * A34 * A42 * A55 + A03 * A11 * A34 * A45 * A52 + A03 * A11 * A35 * A42 * A54 - A03 * A11 * A35 * A44 * A52 - A03 * A12 * A31 * A44 * A55 + A03 * A12 * A31 * A45 * A54 + A03 * A12 * A34 * A41 * A55 - A03 * A12 * A34 * A45 * A51 - A03 * A12 * A35 * A41 * A54 + A03 * A12 * A35 * A44 * A51 + A03 * A14 * A31 * A42 * A55 - A03 * A14 * A31 * A45 * A52 - A03 * A14 * A32 * A41 * A55 + A03 * A14 * A32 * A45 * A51 + A03 * A14 * A35 * A41 * A52 - A03 * A14 * A35 * A42 * A51 - A03 * A15 * A31 * A42 * A54 + A03 * A15 * A31 * A44 * A52 + A03 * A15 * A32 * A41 * A54 - A03 * A15 * A32 * A44 * A51 - A03 * A15 * A34 * A41 * A52 + A03 * A15 * A34 * A42 * A51 - A04 * A11 * A32 * A43 * A55 + A04 * A11 * A32 * A45 * A53 + A04 * A11 * A33 * A42 * A55 - A04 * A11 * A33 * A45 * A52 - A04 * A11 * A35 * A42 * A53 + A04 * A11 * A35 * A43 * A52 + A04 * A12 * A31 * A43 * A55 - A04 * A12 * A31 * A45 * A53 - A04 * A12 * A33 * A41 * A55 + A04 * A12 * A33 * A45 * A51 + A04 * A12 * A35 * A41 * A53 - A04 * A12 * A35 * A43 * A51 - A04 * A13 * A31 * A42 * A55 + A04 * A13 * A31 * A45 * A52 + A04 * A13 * A32 * A41 * A55 - A04 * A13 * A32 * A45 * A51 - A04 * A13 * A35 * A41 * A52 + A04 * A13 * A35 * A42 * A51 + A04 * A15 * A31 * A42 * A53 - A04 * A15 * A31 * A43 * A52 - A04 * A15 * A32 * A41 * A53 + A04 * A15 * A32 * A43 * A51 + A04 * A15 * A33 * A41 * A52 - A04 * A15 * A33 * A42 * A51 + A05 * A11 * A32 * A43 * A54 - A05 * A11 * A32 * A44 * A53 - A05 * A11 * A33 * A42 * A54 + A05 * A11 * A33 * A44 * A52 + A05 * A11 * A34 * A42 * A53 - A05 * A11 * A34 * A43 * A52 - A05 * A12 * A31 * A43 * A54 + A05 * A12 * A31 * A44 * A53 + A05 * A12 * A33 * A41 * A54 - A05 * A12 * A33 * A44 * A51 - A05 * A12 * A34 * A41 * A53 + A05 * A12 * A34 * A43 * A51 + A05 * A13 * A31 * A42 * A54 - A05 * A13 * A31 * A44 * A52 - A05 * A13 * A32 * A41 * A54 + A05 * A13 * A32 * A44 * A51 + A05 * A13 * A34 * A41 * A52 - A05 * A13 * A34 * A42 * A51 - A05 * A14 * A31 * A42 * A53 + A05 * A14 * A31 * A43 * A52 + A05 * A14 * A32 * A41 * A53 - A05 * A14 * A32 * A43 * A51 - A05 * A14 * A33 * A41 * A52 + A05 * A14 * A33 * A42 * A51) / detA;
    d[0][3] = (A01 * A12 * A23 * A45 * A54 - A01 * A12 * A23 * A44 * A55 + A01 * A12 * A24 * A43 * A55 - A01 * A12 * A24 * A45 * A53 - A01 * A12 * A25 * A43 * A54 + A01 * A12 * A25 * A44 * A53 + A01 * A13 * A22 * A44 * A55 - A01 * A13 * A22 * A45 * A54 - A01 * A13 * A24 * A42 * A55 + A01 * A13 * A24 * A45 * A52 + A01 * A13 * A25 * A42 * A54 - A01 * A13 * A25 * A44 * A52 - A01 * A14 * A22 * A43 * A55 + A01 * A14 * A22 * A45 * A53 + A01 * A14 * A23 * A42 * A55 - A01 * A14 * A23 * A45 * A52 - A01 * A14 * A25 * A42 * A53 + A01 * A14 * A25 * A43 * A52 + A01 * A15 * A22 * A43 * A54 - A01 * A15 * A22 * A44 * A53 - A01 * A15 * A23 * A42 * A54 + A01 * A15 * A23 * A44 * A52 + A01 * A15 * A24 * A42 * A53 - A01 * A15 * A24 * A43 * A52 + A02 * A11 * A23 * A44 * A55 - A02 * A11 * A23 * A45 * A54 - A02 * A11 * A24 * A43 * A55 + A02 * A11 * A24 * A45 * A53 + A02 * A11 * A25 * A43 * A54 - A02 * A11 * A25 * A44 * A53 - A02 * A13 * A21 * A44 * A55 + A02 * A13 * A21 * A45 * A54 + A02 * A13 * A24 * A41 * A55 - A02 * A13 * A24 * A45 * A51 - A02 * A13 * A25 * A41 * A54 + A02 * A13 * A25 * A44 * A51 + A02 * A14 * A21 * A43 * A55 - A02 * A14 * A21 * A45 * A53 - A02 * A14 * A23 * A41 * A55 + A02 * A14 * A23 * A45 * A51 + A02 * A14 * A25 * A41 * A53 - A02 * A14 * A25 * A43 * A51 - A02 * A15 * A21 * A43 * A54 + A02 * A15 * A21 * A44 * A53 + A02 * A15 * A23 * A41 * A54 - A02 * A15 * A23 * A44 * A51 - A02 * A15 * A24 * A41 * A53 + A02 * A15 * A24 * A43 * A51 - A03 * A11 * A22 * A44 * A55 + A03 * A11 * A22 * A45 * A54 + A03 * A11 * A24 * A42 * A55 - A03 * A11 * A24 * A45 * A52 - A03 * A11 * A25 * A42 * A54 + A03 * A11 * A25 * A44 * A52 + A03 * A12 * A21 * A44 * A55 - A03 * A12 * A21 * A45 * A54 - A03 * A12 * A24 * A41 * A55 + A03 * A12 * A24 * A45 * A51 + A03 * A12 * A25 * A41 * A54 - A03 * A12 * A25 * A44 * A51 - A03 * A14 * A21 * A42 * A55 + A03 * A14 * A21 * A45 * A52 + A03 * A14 * A22 * A41 * A55 - A03 * A14 * A22 * A45 * A51 - A03 * A14 * A25 * A41 * A52 + A03 * A14 * A25 * A42 * A51 + A03 * A15 * A21 * A42 * A54 - A03 * A15 * A21 * A44 * A52 - A03 * A15 * A22 * A41 * A54 + A03 * A15 * A22 * A44 * A51 + A03 * A15 * A24 * A41 * A52 - A03 * A15 * A24 * A42 * A51 + A04 * A11 * A22 * A43 * A55 - A04 * A11 * A22 * A45 * A53 - A04 * A11 * A23 * A42 * A55 + A04 * A11 * A23 * A45 * A52 + A04 * A11 * A25 * A42 * A53 - A04 * A11 * A25 * A43 * A52 - A04 * A12 * A21 * A43 * A55 + A04 * A12 * A21 * A45 * A53 + A04 * A12 * A23 * A41 * A55 - A04 * A12 * A23 * A45 * A51 - A04 * A12 * A25 * A41 * A53 + A04 * A12 * A25 * A43 * A51 + A04 * A13 * A21 * A42 * A55 - A04 * A13 * A21 * A45 * A52 - A04 * A13 * A22 * A41 * A55 + A04 * A13 * A22 * A45 * A51 + A04 * A13 * A25 * A41 * A52 - A04 * A13 * A25 * A42 * A51 - A04 * A15 * A21 * A42 * A53 + A04 * A15 * A21 * A43 * A52 + A04 * A15 * A22 * A41 * A53 - A04 * A15 * A22 * A43 * A51 - A04 * A15 * A23 * A41 * A52 + A04 * A15 * A23 * A42 * A51 - A05 * A11 * A22 * A43 * A54 + A05 * A11 * A22 * A44 * A53 + A05 * A11 * A23 * A42 * A54 - A05 * A11 * A23 * A44 * A52 - A05 * A11 * A24 * A42 * A53 + A05 * A11 * A24 * A43 * A52 + A05 * A12 * A21 * A43 * A54 - A05 * A12 * A21 * A44 * A53 - A05 * A12 * A23 * A41 * A54 + A05 * A12 * A23 * A44 * A51 + A05 * A12 * A24 * A41 * A53 - A05 * A12 * A24 * A43 * A51 - A05 * A13 * A21 * A42 * A54 + A05 * A13 * A21 * A44 * A52 + A05 * A13 * A22 * A41 * A54 - A05 * A13 * A22 * A44 * A51 - A05 * A13 * A24 * A41 * A52 + A05 * A13 * A24 * A42 * A51 + A05 * A14 * A21 * A42 * A53 - A05 * A14 * A21 * A43 * A52 - A05 * A14 * A22 * A41 * A53 + A05 * A14 * A22 * A43 * A51 + A05 * A14 * A23 * A41 * A52 - A05 * A14 * A23 * A42 * A51) / detA;
    d[0][4] = (A01 * A12 * A23 * A34 * A55 - A01 * A12 * A23 * A35 * A54 - A01 * A12 * A24 * A33 * A55 + A01 * A12 * A24 * A35 * A53 + A01 * A12 * A25 * A33 * A54 - A01 * A12 * A25 * A34 * A53 - A01 * A13 * A22 * A34 * A55 + A01 * A13 * A22 * A35 * A54 + A01 * A13 * A24 * A32 * A55 - A01 * A13 * A24 * A35 * A52 - A01 * A13 * A25 * A32 * A54 + A01 * A13 * A25 * A34 * A52 + A01 * A14 * A22 * A33 * A55 - A01 * A14 * A22 * A35 * A53 - A01 * A14 * A23 * A32 * A55 + A01 * A14 * A23 * A35 * A52 + A01 * A14 * A25 * A32 * A53 - A01 * A14 * A25 * A33 * A52 - A01 * A15 * A22 * A33 * A54 + A01 * A15 * A22 * A34 * A53 + A01 * A15 * A23 * A32 * A54 - A01 * A15 * A23 * A34 * A52 - A01 * A15 * A24 * A32 * A53 + A01 * A15 * A24 * A33 * A52 - A02 * A11 * A23 * A34 * A55 + A02 * A11 * A23 * A35 * A54 + A02 * A11 * A24 * A33 * A55 - A02 * A11 * A24 * A35 * A53 - A02 * A11 * A25 * A33 * A54 + A02 * A11 * A25 * A34 * A53 + A02 * A13 * A21 * A34 * A55 - A02 * A13 * A21 * A35 * A54 - A02 * A13 * A24 * A31 * A55 + A02 * A13 * A24 * A35 * A51 + A02 * A13 * A25 * A31 * A54 - A02 * A13 * A25 * A34 * A51 - A02 * A14 * A21 * A33 * A55 + A02 * A14 * A21 * A35 * A53 + A02 * A14 * A23 * A31 * A55 - A02 * A14 * A23 * A35 * A51 - A02 * A14 * A25 * A31 * A53 + A02 * A14 * A25 * A33 * A51 + A02 * A15 * A21 * A33 * A54 - A02 * A15 * A21 * A34 * A53 - A02 * A15 * A23 * A31 * A54 + A02 * A15 * A23 * A34 * A51 + A02 * A15 * A24 * A31 * A53 - A02 * A15 * A24 * A33 * A51 + A03 * A11 * A22 * A34 * A55 - A03 * A11 * A22 * A35 * A54 - A03 * A11 * A24 * A32 * A55 + A03 * A11 * A24 * A35 * A52 + A03 * A11 * A25 * A32 * A54 - A03 * A11 * A25 * A34 * A52 - A03 * A12 * A21 * A34 * A55 + A03 * A12 * A21 * A35 * A54 + A03 * A12 * A24 * A31 * A55 - A03 * A12 * A24 * A35 * A51 - A03 * A12 * A25 * A31 * A54 + A03 * A12 * A25 * A34 * A51 + A03 * A14 * A21 * A32 * A55 - A03 * A14 * A21 * A35 * A52 - A03 * A14 * A22 * A31 * A55 + A03 * A14 * A22 * A35 * A51 + A03 * A14 * A25 * A31 * A52 - A03 * A14 * A25 * A32 * A51 - A03 * A15 * A21 * A32 * A54 + A03 * A15 * A21 * A34 * A52 + A03 * A15 * A22 * A31 * A54 - A03 * A15 * A22 * A34 * A51 - A03 * A15 * A24 * A31 * A52 + A03 * A15 * A24 * A32 * A51 - A04 * A11 * A22 * A33 * A55 + A04 * A11 * A22 * A35 * A53 + A04 * A11 * A23 * A32 * A55 - A04 * A11 * A23 * A35 * A52 - A04 * A11 * A25 * A32 * A53 + A04 * A11 * A25 * A33 * A52 + A04 * A12 * A21 * A33 * A55 - A04 * A12 * A21 * A35 * A53 - A04 * A12 * A23 * A31 * A55 + A04 * A12 * A23 * A35 * A51 + A04 * A12 * A25 * A31 * A53 - A04 * A12 * A25 * A33 * A51 - A04 * A13 * A21 * A32 * A55 + A04 * A13 * A21 * A35 * A52 + A04 * A13 * A22 * A31 * A55 - A04 * A13 * A22 * A35 * A51 - A04 * A13 * A25 * A31 * A52 + A04 * A13 * A25 * A32 * A51 + A04 * A15 * A21 * A32 * A53 - A04 * A15 * A21 * A33 * A52 - A04 * A15 * A22 * A31 * A53 + A04 * A15 * A22 * A33 * A51 + A04 * A15 * A23 * A31 * A52 - A04 * A15 * A23 * A32 * A51 + A05 * A11 * A22 * A33 * A54 - A05 * A11 * A22 * A34 * A53 - A05 * A11 * A23 * A32 * A54 + A05 * A11 * A23 * A34 * A52 + A05 * A11 * A24 * A32 * A53 - A05 * A11 * A24 * A33 * A52 - A05 * A12 * A21 * A33 * A54 + A05 * A12 * A21 * A34 * A53 + A05 * A12 * A23 * A31 * A54 - A05 * A12 * A23 * A34 * A51 - A05 * A12 * A24 * A31 * A53 + A05 * A12 * A24 * A33 * A51 + A05 * A13 * A21 * A32 * A54 - A05 * A13 * A21 * A34 * A52 - A05 * A13 * A22 * A31 * A54 + A05 * A13 * A22 * A34 * A51 + A05 * A13 * A24 * A31 * A52 - A05 * A13 * A24 * A32 * A51 - A05 * A14 * A21 * A32 * A53 + A05 * A14 * A21 * A33 * A52 + A05 * A14 * A22 * A31 * A53 - A05 * A14 * A22 * A33 * A51 - A05 * A14 * A23 * A31 * A52 + A05 * A14 * A23 * A32 * A51) / detA;
    d[0][5] = (A01 * A12 * A23 * A35 * A44 - A01 * A12 * A23 * A34 * A45 + A01 * A12 * A24 * A33 * A45 - A01 * A12 * A24 * A35 * A43 - A01 * A12 * A25 * A33 * A44 + A01 * A12 * A25 * A34 * A43 + A01 * A13 * A22 * A34 * A45 - A01 * A13 * A22 * A35 * A44 - A01 * A13 * A24 * A32 * A45 + A01 * A13 * A24 * A35 * A42 + A01 * A13 * A25 * A32 * A44 - A01 * A13 * A25 * A34 * A42 - A01 * A14 * A22 * A33 * A45 + A01 * A14 * A22 * A35 * A43 + A01 * A14 * A23 * A32 * A45 - A01 * A14 * A23 * A35 * A42 - A01 * A14 * A25 * A32 * A43 + A01 * A14 * A25 * A33 * A42 + A01 * A15 * A22 * A33 * A44 - A01 * A15 * A22 * A34 * A43 - A01 * A15 * A23 * A32 * A44 + A01 * A15 * A23 * A34 * A42 + A01 * A15 * A24 * A32 * A43 - A01 * A15 * A24 * A33 * A42 + A02 * A11 * A23 * A34 * A45 - A02 * A11 * A23 * A35 * A44 - A02 * A11 * A24 * A33 * A45 + A02 * A11 * A24 * A35 * A43 + A02 * A11 * A25 * A33 * A44 - A02 * A11 * A25 * A34 * A43 - A02 * A13 * A21 * A34 * A45 + A02 * A13 * A21 * A35 * A44 + A02 * A13 * A24 * A31 * A45 - A02 * A13 * A24 * A35 * A41 - A02 * A13 * A25 * A31 * A44 + A02 * A13 * A25 * A34 * A41 + A02 * A14 * A21 * A33 * A45 - A02 * A14 * A21 * A35 * A43 - A02 * A14 * A23 * A31 * A45 + A02 * A14 * A23 * A35 * A41 + A02 * A14 * A25 * A31 * A43 - A02 * A14 * A25 * A33 * A41 - A02 * A15 * A21 * A33 * A44 + A02 * A15 * A21 * A34 * A43 + A02 * A15 * A23 * A31 * A44 - A02 * A15 * A23 * A34 * A41 - A02 * A15 * A24 * A31 * A43 + A02 * A15 * A24 * A33 * A41 - A03 * A11 * A22 * A34 * A45 + A03 * A11 * A22 * A35 * A44 + A03 * A11 * A24 * A32 * A45 - A03 * A11 * A24 * A35 * A42 - A03 * A11 * A25 * A32 * A44 + A03 * A11 * A25 * A34 * A42 + A03 * A12 * A21 * A34 * A45 - A03 * A12 * A21 * A35 * A44 - A03 * A12 * A24 * A31 * A45 + A03 * A12 * A24 * A35 * A41 + A03 * A12 * A25 * A31 * A44 - A03 * A12 * A25 * A34 * A41 - A03 * A14 * A21 * A32 * A45 + A03 * A14 * A21 * A35 * A42 + A03 * A14 * A22 * A31 * A45 - A03 * A14 * A22 * A35 * A41 - A03 * A14 * A25 * A31 * A42 + A03 * A14 * A25 * A32 * A41 + A03 * A15 * A21 * A32 * A44 - A03 * A15 * A21 * A34 * A42 - A03 * A15 * A22 * A31 * A44 + A03 * A15 * A22 * A34 * A41 + A03 * A15 * A24 * A31 * A42 - A03 * A15 * A24 * A32 * A41 + A04 * A11 * A22 * A33 * A45 - A04 * A11 * A22 * A35 * A43 - A04 * A11 * A23 * A32 * A45 + A04 * A11 * A23 * A35 * A42 + A04 * A11 * A25 * A32 * A43 - A04 * A11 * A25 * A33 * A42 - A04 * A12 * A21 * A33 * A45 + A04 * A12 * A21 * A35 * A43 + A04 * A12 * A23 * A31 * A45 - A04 * A12 * A23 * A35 * A41 - A04 * A12 * A25 * A31 * A43 + A04 * A12 * A25 * A33 * A41 + A04 * A13 * A21 * A32 * A45 - A04 * A13 * A21 * A35 * A42 - A04 * A13 * A22 * A31 * A45 + A04 * A13 * A22 * A35 * A41 + A04 * A13 * A25 * A31 * A42 - A04 * A13 * A25 * A32 * A41 - A04 * A15 * A21 * A32 * A43 + A04 * A15 * A21 * A33 * A42 + A04 * A15 * A22 * A31 * A43 - A04 * A15 * A22 * A33 * A41 - A04 * A15 * A23 * A31 * A42 + A04 * A15 * A23 * A32 * A41 - A05 * A11 * A22 * A33 * A44 + A05 * A11 * A22 * A34 * A43 + A05 * A11 * A23 * A32 * A44 - A05 * A11 * A23 * A34 * A42 - A05 * A11 * A24 * A32 * A43 + A05 * A11 * A24 * A33 * A42 + A05 * A12 * A21 * A33 * A44 - A05 * A12 * A21 * A34 * A43 - A05 * A12 * A23 * A31 * A44 + A05 * A12 * A23 * A34 * A41 + A05 * A12 * A24 * A31 * A43 - A05 * A12 * A24 * A33 * A41 - A05 * A13 * A21 * A32 * A44 + A05 * A13 * A21 * A34 * A42 + A05 * A13 * A22 * A31 * A44 - A05 * A13 * A22 * A34 * A41 - A05 * A13 * A24 * A31 * A42 + A05 * A13 * A24 * A32 * A41 + A05 * A14 * A21 * A32 * A43 - A05 * A14 * A21 * A33 * A42 - A05 * A14 * A22 * A31 * A43 + A05 * A14 * A22 * A33 * A41 + A05 * A14 * A23 * A31 * A42 - A05 * A14 * A23 * A32 * A41) / detA;


    d[1][0] = (A10 * A22 * A33 * A45 * A54 - A10 * A22 * A33 * A44 * A55 + A10 * A22 * A34 * A43 * A55 - A10 * A22 * A34 * A45 * A53 - A10 * A22 * A35 * A43 * A54 + A10 * A22 * A35 * A44 * A53 + A10 * A23 * A32 * A44 * A55 - A10 * A23 * A32 * A45 * A54 - A10 * A23 * A34 * A42 * A55 + A10 * A23 * A34 * A45 * A52 + A10 * A23 * A35 * A42 * A54 - A10 * A23 * A35 * A44 * A52 - A10 * A24 * A32 * A43 * A55 + A10 * A24 * A32 * A45 * A53 + A10 * A24 * A33 * A42 * A55 - A10 * A24 * A33 * A45 * A52 - A10 * A24 * A35 * A42 * A53 + A10 * A24 * A35 * A43 * A52 + A10 * A25 * A32 * A43 * A54 - A10 * A25 * A32 * A44 * A53 - A10 * A25 * A33 * A42 * A54 + A10 * A25 * A33 * A44 * A52 + A10 * A25 * A34 * A42 * A53 - A10 * A25 * A34 * A43 * A52 + A12 * A20 * A33 * A44 * A55 - A12 * A20 * A33 * A45 * A54 - A12 * A20 * A34 * A43 * A55 + A12 * A20 * A34 * A45 * A53 + A12 * A20 * A35 * A43 * A54 - A12 * A20 * A35 * A44 * A53 - A12 * A23 * A30 * A44 * A55 + A12 * A23 * A30 * A45 * A54 + A12 * A23 * A34 * A40 * A55 - A12 * A23 * A34 * A45 * A50 - A12 * A23 * A35 * A40 * A54 + A12 * A23 * A35 * A44 * A50 + A12 * A24 * A30 * A43 * A55 - A12 * A24 * A30 * A45 * A53 - A12 * A24 * A33 * A40 * A55 + A12 * A24 * A33 * A45 * A50 + A12 * A24 * A35 * A40 * A53 - A12 * A24 * A35 * A43 * A50 - A12 * A25 * A30 * A43 * A54 + A12 * A25 * A30 * A44 * A53 + A12 * A25 * A33 * A40 * A54 - A12 * A25 * A33 * A44 * A50 - A12 * A25 * A34 * A40 * A53 + A12 * A25 * A34 * A43 * A50 - A13 * A20 * A32 * A44 * A55 + A13 * A20 * A32 * A45 * A54 + A13 * A20 * A34 * A42 * A55 - A13 * A20 * A34 * A45 * A52 - A13 * A20 * A35 * A42 * A54 + A13 * A20 * A35 * A44 * A52 + A13 * A22 * A30 * A44 * A55 - A13 * A22 * A30 * A45 * A54 - A13 * A22 * A34 * A40 * A55 + A13 * A22 * A34 * A45 * A50 + A13 * A22 * A35 * A40 * A54 - A13 * A22 * A35 * A44 * A50 - A13 * A24 * A30 * A42 * A55 + A13 * A24 * A30 * A45 * A52 + A13 * A24 * A32 * A40 * A55 - A13 * A24 * A32 * A45 * A50 - A13 * A24 * A35 * A40 * A52 + A13 * A24 * A35 * A42 * A50 + A13 * A25 * A30 * A42 * A54 - A13 * A25 * A30 * A44 * A52 - A13 * A25 * A32 * A40 * A54 + A13 * A25 * A32 * A44 * A50 + A13 * A25 * A34 * A40 * A52 - A13 * A25 * A34 * A42 * A50 + A14 * A20 * A32 * A43 * A55 - A14 * A20 * A32 * A45 * A53 - A14 * A20 * A33 * A42 * A55 + A14 * A20 * A33 * A45 * A52 + A14 * A20 * A35 * A42 * A53 - A14 * A20 * A35 * A43 * A52 - A14 * A22 * A30 * A43 * A55 + A14 * A22 * A30 * A45 * A53 + A14 * A22 * A33 * A40 * A55 - A14 * A22 * A33 * A45 * A50 - A14 * A22 * A35 * A40 * A53 + A14 * A22 * A35 * A43 * A50 + A14 * A23 * A30 * A42 * A55 - A14 * A23 * A30 * A45 * A52 - A14 * A23 * A32 * A40 * A55 + A14 * A23 * A32 * A45 * A50 + A14 * A23 * A35 * A40 * A52 - A14 * A23 * A35 * A42 * A50 - A14 * A25 * A30 * A42 * A53 + A14 * A25 * A30 * A43 * A52 + A14 * A25 * A32 * A40 * A53 - A14 * A25 * A32 * A43 * A50 - A14 * A25 * A33 * A40 * A52 + A14 * A25 * A33 * A42 * A50 - A15 * A20 * A32 * A43 * A54 + A15 * A20 * A32 * A44 * A53 + A15 * A20 * A33 * A42 * A54 - A15 * A20 * A33 * A44 * A52 - A15 * A20 * A34 * A42 * A53 + A15 * A20 * A34 * A43 * A52 + A15 * A22 * A30 * A43 * A54 - A15 * A22 * A30 * A44 * A53 - A15 * A22 * A33 * A40 * A54 + A15 * A22 * A33 * A44 * A50 + A15 * A22 * A34 * A40 * A53 - A15 * A22 * A34 * A43 * A50 - A15 * A23 * A30 * A42 * A54 + A15 * A23 * A30 * A44 * A52 + A15 * A23 * A32 * A40 * A54 - A15 * A23 * A32 * A44 * A50 - A15 * A23 * A34 * A40 * A52 + A15 * A23 * A34 * A42 * A50 + A15 * A24 * A30 * A42 * A53 - A15 * A24 * A30 * A43 * A52 - A15 * A24 * A32 * A40 * A53 + A15 * A24 * A32 * A43 * A50 + A15 * A24 * A33 * A40 * A52 - A15 * A24 * A33 * A42 * A50) / detA;
    d[1][1] = (A00 * A22 * A33 * A44 * A55 - A00 * A22 * A33 * A45 * A54 - A00 * A22 * A34 * A43 * A55 + A00 * A22 * A34 * A45 * A53 + A00 * A22 * A35 * A43 * A54 - A00 * A22 * A35 * A44 * A53 - A00 * A23 * A32 * A44 * A55 + A00 * A23 * A32 * A45 * A54 + A00 * A23 * A34 * A42 * A55 - A00 * A23 * A34 * A45 * A52 - A00 * A23 * A35 * A42 * A54 + A00 * A23 * A35 * A44 * A52 + A00 * A24 * A32 * A43 * A55 - A00 * A24 * A32 * A45 * A53 - A00 * A24 * A33 * A42 * A55 + A00 * A24 * A33 * A45 * A52 + A00 * A24 * A35 * A42 * A53 - A00 * A24 * A35 * A43 * A52 - A00 * A25 * A32 * A43 * A54 + A00 * A25 * A32 * A44 * A53 + A00 * A25 * A33 * A42 * A54 - A00 * A25 * A33 * A44 * A52 - A00 * A25 * A34 * A42 * A53 + A00 * A25 * A34 * A43 * A52 - A02 * A20 * A33 * A44 * A55 + A02 * A20 * A33 * A45 * A54 + A02 * A20 * A34 * A43 * A55 - A02 * A20 * A34 * A45 * A53 - A02 * A20 * A35 * A43 * A54 + A02 * A20 * A35 * A44 * A53 + A02 * A23 * A30 * A44 * A55 - A02 * A23 * A30 * A45 * A54 - A02 * A23 * A34 * A40 * A55 + A02 * A23 * A34 * A45 * A50 + A02 * A23 * A35 * A40 * A54 - A02 * A23 * A35 * A44 * A50 - A02 * A24 * A30 * A43 * A55 + A02 * A24 * A30 * A45 * A53 + A02 * A24 * A33 * A40 * A55 - A02 * A24 * A33 * A45 * A50 - A02 * A24 * A35 * A40 * A53 + A02 * A24 * A35 * A43 * A50 + A02 * A25 * A30 * A43 * A54 - A02 * A25 * A30 * A44 * A53 - A02 * A25 * A33 * A40 * A54 + A02 * A25 * A33 * A44 * A50 + A02 * A25 * A34 * A40 * A53 - A02 * A25 * A34 * A43 * A50 + A03 * A20 * A32 * A44 * A55 - A03 * A20 * A32 * A45 * A54 - A03 * A20 * A34 * A42 * A55 + A03 * A20 * A34 * A45 * A52 + A03 * A20 * A35 * A42 * A54 - A03 * A20 * A35 * A44 * A52 - A03 * A22 * A30 * A44 * A55 + A03 * A22 * A30 * A45 * A54 + A03 * A22 * A34 * A40 * A55 - A03 * A22 * A34 * A45 * A50 - A03 * A22 * A35 * A40 * A54 + A03 * A22 * A35 * A44 * A50 + A03 * A24 * A30 * A42 * A55 - A03 * A24 * A30 * A45 * A52 - A03 * A24 * A32 * A40 * A55 + A03 * A24 * A32 * A45 * A50 + A03 * A24 * A35 * A40 * A52 - A03 * A24 * A35 * A42 * A50 - A03 * A25 * A30 * A42 * A54 + A03 * A25 * A30 * A44 * A52 + A03 * A25 * A32 * A40 * A54 - A03 * A25 * A32 * A44 * A50 - A03 * A25 * A34 * A40 * A52 + A03 * A25 * A34 * A42 * A50 - A04 * A20 * A32 * A43 * A55 + A04 * A20 * A32 * A45 * A53 + A04 * A20 * A33 * A42 * A55 - A04 * A20 * A33 * A45 * A52 - A04 * A20 * A35 * A42 * A53 + A04 * A20 * A35 * A43 * A52 + A04 * A22 * A30 * A43 * A55 - A04 * A22 * A30 * A45 * A53 - A04 * A22 * A33 * A40 * A55 + A04 * A22 * A33 * A45 * A50 + A04 * A22 * A35 * A40 * A53 - A04 * A22 * A35 * A43 * A50 - A04 * A23 * A30 * A42 * A55 + A04 * A23 * A30 * A45 * A52 + A04 * A23 * A32 * A40 * A55 - A04 * A23 * A32 * A45 * A50 - A04 * A23 * A35 * A40 * A52 + A04 * A23 * A35 * A42 * A50 + A04 * A25 * A30 * A42 * A53 - A04 * A25 * A30 * A43 * A52 - A04 * A25 * A32 * A40 * A53 + A04 * A25 * A32 * A43 * A50 + A04 * A25 * A33 * A40 * A52 - A04 * A25 * A33 * A42 * A50 + A05 * A20 * A32 * A43 * A54 - A05 * A20 * A32 * A44 * A53 - A05 * A20 * A33 * A42 * A54 + A05 * A20 * A33 * A44 * A52 + A05 * A20 * A34 * A42 * A53 - A05 * A20 * A34 * A43 * A52 - A05 * A22 * A30 * A43 * A54 + A05 * A22 * A30 * A44 * A53 + A05 * A22 * A33 * A40 * A54 - A05 * A22 * A33 * A44 * A50 - A05 * A22 * A34 * A40 * A53 + A05 * A22 * A34 * A43 * A50 + A05 * A23 * A30 * A42 * A54 - A05 * A23 * A30 * A44 * A52 - A05 * A23 * A32 * A40 * A54 + A05 * A23 * A32 * A44 * A50 + A05 * A23 * A34 * A40 * A52 - A05 * A23 * A34 * A42 * A50 - A05 * A24 * A30 * A42 * A53 + A05 * A24 * A30 * A43 * A52 + A05 * A24 * A32 * A40 * A53 - A05 * A24 * A32 * A43 * A50 - A05 * A24 * A33 * A40 * A52 + A05 * A24 * A33 * A42 * A50) / detA;
    d[1][2] = (A00 * A12 * A33 * A45 * A54 - A00 * A12 * A33 * A44 * A55 + A00 * A12 * A34 * A43 * A55 - A00 * A12 * A34 * A45 * A53 - A00 * A12 * A35 * A43 * A54 + A00 * A12 * A35 * A44 * A53 + A00 * A13 * A32 * A44 * A55 - A00 * A13 * A32 * A45 * A54 - A00 * A13 * A34 * A42 * A55 + A00 * A13 * A34 * A45 * A52 + A00 * A13 * A35 * A42 * A54 - A00 * A13 * A35 * A44 * A52 - A00 * A14 * A32 * A43 * A55 + A00 * A14 * A32 * A45 * A53 + A00 * A14 * A33 * A42 * A55 - A00 * A14 * A33 * A45 * A52 - A00 * A14 * A35 * A42 * A53 + A00 * A14 * A35 * A43 * A52 + A00 * A15 * A32 * A43 * A54 - A00 * A15 * A32 * A44 * A53 - A00 * A15 * A33 * A42 * A54 + A00 * A15 * A33 * A44 * A52 + A00 * A15 * A34 * A42 * A53 - A00 * A15 * A34 * A43 * A52 + A02 * A10 * A33 * A44 * A55 - A02 * A10 * A33 * A45 * A54 - A02 * A10 * A34 * A43 * A55 + A02 * A10 * A34 * A45 * A53 + A02 * A10 * A35 * A43 * A54 - A02 * A10 * A35 * A44 * A53 - A02 * A13 * A30 * A44 * A55 + A02 * A13 * A30 * A45 * A54 + A02 * A13 * A34 * A40 * A55 - A02 * A13 * A34 * A45 * A50 - A02 * A13 * A35 * A40 * A54 + A02 * A13 * A35 * A44 * A50 + A02 * A14 * A30 * A43 * A55 - A02 * A14 * A30 * A45 * A53 - A02 * A14 * A33 * A40 * A55 + A02 * A14 * A33 * A45 * A50 + A02 * A14 * A35 * A40 * A53 - A02 * A14 * A35 * A43 * A50 - A02 * A15 * A30 * A43 * A54 + A02 * A15 * A30 * A44 * A53 + A02 * A15 * A33 * A40 * A54 - A02 * A15 * A33 * A44 * A50 - A02 * A15 * A34 * A40 * A53 + A02 * A15 * A34 * A43 * A50 - A03 * A10 * A32 * A44 * A55 + A03 * A10 * A32 * A45 * A54 + A03 * A10 * A34 * A42 * A55 - A03 * A10 * A34 * A45 * A52 - A03 * A10 * A35 * A42 * A54 + A03 * A10 * A35 * A44 * A52 + A03 * A12 * A30 * A44 * A55 - A03 * A12 * A30 * A45 * A54 - A03 * A12 * A34 * A40 * A55 + A03 * A12 * A34 * A45 * A50 + A03 * A12 * A35 * A40 * A54 - A03 * A12 * A35 * A44 * A50 - A03 * A14 * A30 * A42 * A55 + A03 * A14 * A30 * A45 * A52 + A03 * A14 * A32 * A40 * A55 - A03 * A14 * A32 * A45 * A50 - A03 * A14 * A35 * A40 * A52 + A03 * A14 * A35 * A42 * A50 + A03 * A15 * A30 * A42 * A54 - A03 * A15 * A30 * A44 * A52 - A03 * A15 * A32 * A40 * A54 + A03 * A15 * A32 * A44 * A50 + A03 * A15 * A34 * A40 * A52 - A03 * A15 * A34 * A42 * A50 + A04 * A10 * A32 * A43 * A55 - A04 * A10 * A32 * A45 * A53 - A04 * A10 * A33 * A42 * A55 + A04 * A10 * A33 * A45 * A52 + A04 * A10 * A35 * A42 * A53 - A04 * A10 * A35 * A43 * A52 - A04 * A12 * A30 * A43 * A55 + A04 * A12 * A30 * A45 * A53 + A04 * A12 * A33 * A40 * A55 - A04 * A12 * A33 * A45 * A50 - A04 * A12 * A35 * A40 * A53 + A04 * A12 * A35 * A43 * A50 + A04 * A13 * A30 * A42 * A55 - A04 * A13 * A30 * A45 * A52 - A04 * A13 * A32 * A40 * A55 + A04 * A13 * A32 * A45 * A50 + A04 * A13 * A35 * A40 * A52 - A04 * A13 * A35 * A42 * A50 - A04 * A15 * A30 * A42 * A53 + A04 * A15 * A30 * A43 * A52 + A04 * A15 * A32 * A40 * A53 - A04 * A15 * A32 * A43 * A50 - A04 * A15 * A33 * A40 * A52 + A04 * A15 * A33 * A42 * A50 - A05 * A10 * A32 * A43 * A54 + A05 * A10 * A32 * A44 * A53 + A05 * A10 * A33 * A42 * A54 - A05 * A10 * A33 * A44 * A52 - A05 * A10 * A34 * A42 * A53 + A05 * A10 * A34 * A43 * A52 + A05 * A12 * A30 * A43 * A54 - A05 * A12 * A30 * A44 * A53 - A05 * A12 * A33 * A40 * A54 + A05 * A12 * A33 * A44 * A50 + A05 * A12 * A34 * A40 * A53 - A05 * A12 * A34 * A43 * A50 - A05 * A13 * A30 * A42 * A54 + A05 * A13 * A30 * A44 * A52 + A05 * A13 * A32 * A40 * A54 - A05 * A13 * A32 * A44 * A50 - A05 * A13 * A34 * A40 * A52 + A05 * A13 * A34 * A42 * A50 + A05 * A14 * A30 * A42 * A53 - A05 * A14 * A30 * A43 * A52 - A05 * A14 * A32 * A40 * A53 + A05 * A14 * A32 * A43 * A50 + A05 * A14 * A33 * A40 * A52 - A05 * A14 * A33 * A42 * A50) / detA;
    d[1][3] = (A00 * A12 * A23 * A44 * A55 - A00 * A12 * A23 * A45 * A54 - A00 * A12 * A24 * A43 * A55 + A00 * A12 * A24 * A45 * A53 + A00 * A12 * A25 * A43 * A54 - A00 * A12 * A25 * A44 * A53 - A00 * A13 * A22 * A44 * A55 + A00 * A13 * A22 * A45 * A54 + A00 * A13 * A24 * A42 * A55 - A00 * A13 * A24 * A45 * A52 - A00 * A13 * A25 * A42 * A54 + A00 * A13 * A25 * A44 * A52 + A00 * A14 * A22 * A43 * A55 - A00 * A14 * A22 * A45 * A53 - A00 * A14 * A23 * A42 * A55 + A00 * A14 * A23 * A45 * A52 + A00 * A14 * A25 * A42 * A53 - A00 * A14 * A25 * A43 * A52 - A00 * A15 * A22 * A43 * A54 + A00 * A15 * A22 * A44 * A53 + A00 * A15 * A23 * A42 * A54 - A00 * A15 * A23 * A44 * A52 - A00 * A15 * A24 * A42 * A53 + A00 * A15 * A24 * A43 * A52 - A02 * A10 * A23 * A44 * A55 + A02 * A10 * A23 * A45 * A54 + A02 * A10 * A24 * A43 * A55 - A02 * A10 * A24 * A45 * A53 - A02 * A10 * A25 * A43 * A54 + A02 * A10 * A25 * A44 * A53 + A02 * A13 * A20 * A44 * A55 - A02 * A13 * A20 * A45 * A54 - A02 * A13 * A24 * A40 * A55 + A02 * A13 * A24 * A45 * A50 + A02 * A13 * A25 * A40 * A54 - A02 * A13 * A25 * A44 * A50 - A02 * A14 * A20 * A43 * A55 + A02 * A14 * A20 * A45 * A53 + A02 * A14 * A23 * A40 * A55 - A02 * A14 * A23 * A45 * A50 - A02 * A14 * A25 * A40 * A53 + A02 * A14 * A25 * A43 * A50 + A02 * A15 * A20 * A43 * A54 - A02 * A15 * A20 * A44 * A53 - A02 * A15 * A23 * A40 * A54 + A02 * A15 * A23 * A44 * A50 + A02 * A15 * A24 * A40 * A53 - A02 * A15 * A24 * A43 * A50 + A03 * A10 * A22 * A44 * A55 - A03 * A10 * A22 * A45 * A54 - A03 * A10 * A24 * A42 * A55 + A03 * A10 * A24 * A45 * A52 + A03 * A10 * A25 * A42 * A54 - A03 * A10 * A25 * A44 * A52 - A03 * A12 * A20 * A44 * A55 + A03 * A12 * A20 * A45 * A54 + A03 * A12 * A24 * A40 * A55 - A03 * A12 * A24 * A45 * A50 - A03 * A12 * A25 * A40 * A54 + A03 * A12 * A25 * A44 * A50 + A03 * A14 * A20 * A42 * A55 - A03 * A14 * A20 * A45 * A52 - A03 * A14 * A22 * A40 * A55 + A03 * A14 * A22 * A45 * A50 + A03 * A14 * A25 * A40 * A52 - A03 * A14 * A25 * A42 * A50 - A03 * A15 * A20 * A42 * A54 + A03 * A15 * A20 * A44 * A52 + A03 * A15 * A22 * A40 * A54 - A03 * A15 * A22 * A44 * A50 - A03 * A15 * A24 * A40 * A52 + A03 * A15 * A24 * A42 * A50 - A04 * A10 * A22 * A43 * A55 + A04 * A10 * A22 * A45 * A53 + A04 * A10 * A23 * A42 * A55 - A04 * A10 * A23 * A45 * A52 - A04 * A10 * A25 * A42 * A53 + A04 * A10 * A25 * A43 * A52 + A04 * A12 * A20 * A43 * A55 - A04 * A12 * A20 * A45 * A53 - A04 * A12 * A23 * A40 * A55 + A04 * A12 * A23 * A45 * A50 + A04 * A12 * A25 * A40 * A53 - A04 * A12 * A25 * A43 * A50 - A04 * A13 * A20 * A42 * A55 + A04 * A13 * A20 * A45 * A52 + A04 * A13 * A22 * A40 * A55 - A04 * A13 * A22 * A45 * A50 - A04 * A13 * A25 * A40 * A52 + A04 * A13 * A25 * A42 * A50 + A04 * A15 * A20 * A42 * A53 - A04 * A15 * A20 * A43 * A52 - A04 * A15 * A22 * A40 * A53 + A04 * A15 * A22 * A43 * A50 + A04 * A15 * A23 * A40 * A52 - A04 * A15 * A23 * A42 * A50 + A05 * A10 * A22 * A43 * A54 - A05 * A10 * A22 * A44 * A53 - A05 * A10 * A23 * A42 * A54 + A05 * A10 * A23 * A44 * A52 + A05 * A10 * A24 * A42 * A53 - A05 * A10 * A24 * A43 * A52 - A05 * A12 * A20 * A43 * A54 + A05 * A12 * A20 * A44 * A53 + A05 * A12 * A23 * A40 * A54 - A05 * A12 * A23 * A44 * A50 - A05 * A12 * A24 * A40 * A53 + A05 * A12 * A24 * A43 * A50 + A05 * A13 * A20 * A42 * A54 - A05 * A13 * A20 * A44 * A52 - A05 * A13 * A22 * A40 * A54 + A05 * A13 * A22 * A44 * A50 + A05 * A13 * A24 * A40 * A52 - A05 * A13 * A24 * A42 * A50 - A05 * A14 * A20 * A42 * A53 + A05 * A14 * A20 * A43 * A52 + A05 * A14 * A22 * A40 * A53 - A05 * A14 * A22 * A43 * A50 - A05 * A14 * A23 * A40 * A52 + A05 * A14 * A23 * A42 * A50) / detA;
    d[1][4] = (A00 * A12 * A23 * A35 * A54 - A00 * A12 * A23 * A34 * A55 + A00 * A12 * A24 * A33 * A55 - A00 * A12 * A24 * A35 * A53 - A00 * A12 * A25 * A33 * A54 + A00 * A12 * A25 * A34 * A53 + A00 * A13 * A22 * A34 * A55 - A00 * A13 * A22 * A35 * A54 - A00 * A13 * A24 * A32 * A55 + A00 * A13 * A24 * A35 * A52 + A00 * A13 * A25 * A32 * A54 - A00 * A13 * A25 * A34 * A52 - A00 * A14 * A22 * A33 * A55 + A00 * A14 * A22 * A35 * A53 + A00 * A14 * A23 * A32 * A55 - A00 * A14 * A23 * A35 * A52 - A00 * A14 * A25 * A32 * A53 + A00 * A14 * A25 * A33 * A52 + A00 * A15 * A22 * A33 * A54 - A00 * A15 * A22 * A34 * A53 - A00 * A15 * A23 * A32 * A54 + A00 * A15 * A23 * A34 * A52 + A00 * A15 * A24 * A32 * A53 - A00 * A15 * A24 * A33 * A52 + A02 * A10 * A23 * A34 * A55 - A02 * A10 * A23 * A35 * A54 - A02 * A10 * A24 * A33 * A55 + A02 * A10 * A24 * A35 * A53 + A02 * A10 * A25 * A33 * A54 - A02 * A10 * A25 * A34 * A53 - A02 * A13 * A20 * A34 * A55 + A02 * A13 * A20 * A35 * A54 + A02 * A13 * A24 * A30 * A55 - A02 * A13 * A24 * A35 * A50 - A02 * A13 * A25 * A30 * A54 + A02 * A13 * A25 * A34 * A50 + A02 * A14 * A20 * A33 * A55 - A02 * A14 * A20 * A35 * A53 - A02 * A14 * A23 * A30 * A55 + A02 * A14 * A23 * A35 * A50 + A02 * A14 * A25 * A30 * A53 - A02 * A14 * A25 * A33 * A50 - A02 * A15 * A20 * A33 * A54 + A02 * A15 * A20 * A34 * A53 + A02 * A15 * A23 * A30 * A54 - A02 * A15 * A23 * A34 * A50 - A02 * A15 * A24 * A30 * A53 + A02 * A15 * A24 * A33 * A50 - A03 * A10 * A22 * A34 * A55 + A03 * A10 * A22 * A35 * A54 + A03 * A10 * A24 * A32 * A55 - A03 * A10 * A24 * A35 * A52 - A03 * A10 * A25 * A32 * A54 + A03 * A10 * A25 * A34 * A52 + A03 * A12 * A20 * A34 * A55 - A03 * A12 * A20 * A35 * A54 - A03 * A12 * A24 * A30 * A55 + A03 * A12 * A24 * A35 * A50 + A03 * A12 * A25 * A30 * A54 - A03 * A12 * A25 * A34 * A50 - A03 * A14 * A20 * A32 * A55 + A03 * A14 * A20 * A35 * A52 + A03 * A14 * A22 * A30 * A55 - A03 * A14 * A22 * A35 * A50 - A03 * A14 * A25 * A30 * A52 + A03 * A14 * A25 * A32 * A50 + A03 * A15 * A20 * A32 * A54 - A03 * A15 * A20 * A34 * A52 - A03 * A15 * A22 * A30 * A54 + A03 * A15 * A22 * A34 * A50 + A03 * A15 * A24 * A30 * A52 - A03 * A15 * A24 * A32 * A50 + A04 * A10 * A22 * A33 * A55 - A04 * A10 * A22 * A35 * A53 - A04 * A10 * A23 * A32 * A55 + A04 * A10 * A23 * A35 * A52 + A04 * A10 * A25 * A32 * A53 - A04 * A10 * A25 * A33 * A52 - A04 * A12 * A20 * A33 * A55 + A04 * A12 * A20 * A35 * A53 + A04 * A12 * A23 * A30 * A55 - A04 * A12 * A23 * A35 * A50 - A04 * A12 * A25 * A30 * A53 + A04 * A12 * A25 * A33 * A50 + A04 * A13 * A20 * A32 * A55 - A04 * A13 * A20 * A35 * A52 - A04 * A13 * A22 * A30 * A55 + A04 * A13 * A22 * A35 * A50 + A04 * A13 * A25 * A30 * A52 - A04 * A13 * A25 * A32 * A50 - A04 * A15 * A20 * A32 * A53 + A04 * A15 * A20 * A33 * A52 + A04 * A15 * A22 * A30 * A53 - A04 * A15 * A22 * A33 * A50 - A04 * A15 * A23 * A30 * A52 + A04 * A15 * A23 * A32 * A50 - A05 * A10 * A22 * A33 * A54 + A05 * A10 * A22 * A34 * A53 + A05 * A10 * A23 * A32 * A54 - A05 * A10 * A23 * A34 * A52 - A05 * A10 * A24 * A32 * A53 + A05 * A10 * A24 * A33 * A52 + A05 * A12 * A20 * A33 * A54 - A05 * A12 * A20 * A34 * A53 - A05 * A12 * A23 * A30 * A54 + A05 * A12 * A23 * A34 * A50 + A05 * A12 * A24 * A30 * A53 - A05 * A12 * A24 * A33 * A50 - A05 * A13 * A20 * A32 * A54 + A05 * A13 * A20 * A34 * A52 + A05 * A13 * A22 * A30 * A54 - A05 * A13 * A22 * A34 * A50 - A05 * A13 * A24 * A30 * A52 + A05 * A13 * A24 * A32 * A50 + A05 * A14 * A20 * A32 * A53 - A05 * A14 * A20 * A33 * A52 - A05 * A14 * A22 * A30 * A53 + A05 * A14 * A22 * A33 * A50 + A05 * A14 * A23 * A30 * A52 - A05 * A14 * A23 * A32 * A50) / detA;
    d[1][5] = (A00 * A12 * A23 * A34 * A45 - A00 * A12 * A23 * A35 * A44 - A00 * A12 * A24 * A33 * A45 + A00 * A12 * A24 * A35 * A43 + A00 * A12 * A25 * A33 * A44 - A00 * A12 * A25 * A34 * A43 - A00 * A13 * A22 * A34 * A45 + A00 * A13 * A22 * A35 * A44 + A00 * A13 * A24 * A32 * A45 - A00 * A13 * A24 * A35 * A42 - A00 * A13 * A25 * A32 * A44 + A00 * A13 * A25 * A34 * A42 + A00 * A14 * A22 * A33 * A45 - A00 * A14 * A22 * A35 * A43 - A00 * A14 * A23 * A32 * A45 + A00 * A14 * A23 * A35 * A42 + A00 * A14 * A25 * A32 * A43 - A00 * A14 * A25 * A33 * A42 - A00 * A15 * A22 * A33 * A44 + A00 * A15 * A22 * A34 * A43 + A00 * A15 * A23 * A32 * A44 - A00 * A15 * A23 * A34 * A42 - A00 * A15 * A24 * A32 * A43 + A00 * A15 * A24 * A33 * A42 - A02 * A10 * A23 * A34 * A45 + A02 * A10 * A23 * A35 * A44 + A02 * A10 * A24 * A33 * A45 - A02 * A10 * A24 * A35 * A43 - A02 * A10 * A25 * A33 * A44 + A02 * A10 * A25 * A34 * A43 + A02 * A13 * A20 * A34 * A45 - A02 * A13 * A20 * A35 * A44 - A02 * A13 * A24 * A30 * A45 + A02 * A13 * A24 * A35 * A40 + A02 * A13 * A25 * A30 * A44 - A02 * A13 * A25 * A34 * A40 - A02 * A14 * A20 * A33 * A45 + A02 * A14 * A20 * A35 * A43 + A02 * A14 * A23 * A30 * A45 - A02 * A14 * A23 * A35 * A40 - A02 * A14 * A25 * A30 * A43 + A02 * A14 * A25 * A33 * A40 + A02 * A15 * A20 * A33 * A44 - A02 * A15 * A20 * A34 * A43 - A02 * A15 * A23 * A30 * A44 + A02 * A15 * A23 * A34 * A40 + A02 * A15 * A24 * A30 * A43 - A02 * A15 * A24 * A33 * A40 + A03 * A10 * A22 * A34 * A45 - A03 * A10 * A22 * A35 * A44 - A03 * A10 * A24 * A32 * A45 + A03 * A10 * A24 * A35 * A42 + A03 * A10 * A25 * A32 * A44 - A03 * A10 * A25 * A34 * A42 - A03 * A12 * A20 * A34 * A45 + A03 * A12 * A20 * A35 * A44 + A03 * A12 * A24 * A30 * A45 - A03 * A12 * A24 * A35 * A40 - A03 * A12 * A25 * A30 * A44 + A03 * A12 * A25 * A34 * A40 + A03 * A14 * A20 * A32 * A45 - A03 * A14 * A20 * A35 * A42 - A03 * A14 * A22 * A30 * A45 + A03 * A14 * A22 * A35 * A40 + A03 * A14 * A25 * A30 * A42 - A03 * A14 * A25 * A32 * A40 - A03 * A15 * A20 * A32 * A44 + A03 * A15 * A20 * A34 * A42 + A03 * A15 * A22 * A30 * A44 - A03 * A15 * A22 * A34 * A40 - A03 * A15 * A24 * A30 * A42 + A03 * A15 * A24 * A32 * A40 - A04 * A10 * A22 * A33 * A45 + A04 * A10 * A22 * A35 * A43 + A04 * A10 * A23 * A32 * A45 - A04 * A10 * A23 * A35 * A42 - A04 * A10 * A25 * A32 * A43 + A04 * A10 * A25 * A33 * A42 + A04 * A12 * A20 * A33 * A45 - A04 * A12 * A20 * A35 * A43 - A04 * A12 * A23 * A30 * A45 + A04 * A12 * A23 * A35 * A40 + A04 * A12 * A25 * A30 * A43 - A04 * A12 * A25 * A33 * A40 - A04 * A13 * A20 * A32 * A45 + A04 * A13 * A20 * A35 * A42 + A04 * A13 * A22 * A30 * A45 - A04 * A13 * A22 * A35 * A40 - A04 * A13 * A25 * A30 * A42 + A04 * A13 * A25 * A32 * A40 + A04 * A15 * A20 * A32 * A43 - A04 * A15 * A20 * A33 * A42 - A04 * A15 * A22 * A30 * A43 + A04 * A15 * A22 * A33 * A40 + A04 * A15 * A23 * A30 * A42 - A04 * A15 * A23 * A32 * A40 + A05 * A10 * A22 * A33 * A44 - A05 * A10 * A22 * A34 * A43 - A05 * A10 * A23 * A32 * A44 + A05 * A10 * A23 * A34 * A42 + A05 * A10 * A24 * A32 * A43 - A05 * A10 * A24 * A33 * A42 - A05 * A12 * A20 * A33 * A44 + A05 * A12 * A20 * A34 * A43 + A05 * A12 * A23 * A30 * A44 - A05 * A12 * A23 * A34 * A40 - A05 * A12 * A24 * A30 * A43 + A05 * A12 * A24 * A33 * A40 + A05 * A13 * A20 * A32 * A44 - A05 * A13 * A20 * A34 * A42 - A05 * A13 * A22 * A30 * A44 + A05 * A13 * A22 * A34 * A40 + A05 * A13 * A24 * A30 * A42 - A05 * A13 * A24 * A32 * A40 - A05 * A14 * A20 * A32 * A43 + A05 * A14 * A20 * A33 * A42 + A05 * A14 * A22 * A30 * A43 - A05 * A14 * A22 * A33 * A40 - A05 * A14 * A23 * A30 * A42 + A05 * A14 * A23 * A32 * A40) / detA;


    d[2][0] = (A10 * A21 * A33 * A44 * A55 - A10 * A21 * A33 * A45 * A54 - A10 * A21 * A34 * A43 * A55 + A10 * A21 * A34 * A45 * A53 + A10 * A21 * A35 * A43 * A54 - A10 * A21 * A35 * A44 * A53 - A10 * A23 * A31 * A44 * A55 + A10 * A23 * A31 * A45 * A54 + A10 * A23 * A34 * A41 * A55 - A10 * A23 * A34 * A45 * A51 - A10 * A23 * A35 * A41 * A54 + A10 * A23 * A35 * A44 * A51 + A10 * A24 * A31 * A43 * A55 - A10 * A24 * A31 * A45 * A53 - A10 * A24 * A33 * A41 * A55 + A10 * A24 * A33 * A45 * A51 + A10 * A24 * A35 * A41 * A53 - A10 * A24 * A35 * A43 * A51 - A10 * A25 * A31 * A43 * A54 + A10 * A25 * A31 * A44 * A53 + A10 * A25 * A33 * A41 * A54 - A10 * A25 * A33 * A44 * A51 - A10 * A25 * A34 * A41 * A53 + A10 * A25 * A34 * A43 * A51 - A11 * A20 * A33 * A44 * A55 + A11 * A20 * A33 * A45 * A54 + A11 * A20 * A34 * A43 * A55 - A11 * A20 * A34 * A45 * A53 - A11 * A20 * A35 * A43 * A54 + A11 * A20 * A35 * A44 * A53 + A11 * A23 * A30 * A44 * A55 - A11 * A23 * A30 * A45 * A54 - A11 * A23 * A34 * A40 * A55 + A11 * A23 * A34 * A45 * A50 + A11 * A23 * A35 * A40 * A54 - A11 * A23 * A35 * A44 * A50 - A11 * A24 * A30 * A43 * A55 + A11 * A24 * A30 * A45 * A53 + A11 * A24 * A33 * A40 * A55 - A11 * A24 * A33 * A45 * A50 - A11 * A24 * A35 * A40 * A53 + A11 * A24 * A35 * A43 * A50 + A11 * A25 * A30 * A43 * A54 - A11 * A25 * A30 * A44 * A53 - A11 * A25 * A33 * A40 * A54 + A11 * A25 * A33 * A44 * A50 + A11 * A25 * A34 * A40 * A53 - A11 * A25 * A34 * A43 * A50 + A13 * A20 * A31 * A44 * A55 - A13 * A20 * A31 * A45 * A54 - A13 * A20 * A34 * A41 * A55 + A13 * A20 * A34 * A45 * A51 + A13 * A20 * A35 * A41 * A54 - A13 * A20 * A35 * A44 * A51 - A13 * A21 * A30 * A44 * A55 + A13 * A21 * A30 * A45 * A54 + A13 * A21 * A34 * A40 * A55 - A13 * A21 * A34 * A45 * A50 - A13 * A21 * A35 * A40 * A54 + A13 * A21 * A35 * A44 * A50 + A13 * A24 * A30 * A41 * A55 - A13 * A24 * A30 * A45 * A51 - A13 * A24 * A31 * A40 * A55 + A13 * A24 * A31 * A45 * A50 + A13 * A24 * A35 * A40 * A51 - A13 * A24 * A35 * A41 * A50 - A13 * A25 * A30 * A41 * A54 + A13 * A25 * A30 * A44 * A51 + A13 * A25 * A31 * A40 * A54 - A13 * A25 * A31 * A44 * A50 - A13 * A25 * A34 * A40 * A51 + A13 * A25 * A34 * A41 * A50 - A14 * A20 * A31 * A43 * A55 + A14 * A20 * A31 * A45 * A53 + A14 * A20 * A33 * A41 * A55 - A14 * A20 * A33 * A45 * A51 - A14 * A20 * A35 * A41 * A53 + A14 * A20 * A35 * A43 * A51 + A14 * A21 * A30 * A43 * A55 - A14 * A21 * A30 * A45 * A53 - A14 * A21 * A33 * A40 * A55 + A14 * A21 * A33 * A45 * A50 + A14 * A21 * A35 * A40 * A53 - A14 * A21 * A35 * A43 * A50 - A14 * A23 * A30 * A41 * A55 + A14 * A23 * A30 * A45 * A51 + A14 * A23 * A31 * A40 * A55 - A14 * A23 * A31 * A45 * A50 - A14 * A23 * A35 * A40 * A51 + A14 * A23 * A35 * A41 * A50 + A14 * A25 * A30 * A41 * A53 - A14 * A25 * A30 * A43 * A51 - A14 * A25 * A31 * A40 * A53 + A14 * A25 * A31 * A43 * A50 + A14 * A25 * A33 * A40 * A51 - A14 * A25 * A33 * A41 * A50 + A15 * A20 * A31 * A43 * A54 - A15 * A20 * A31 * A44 * A53 - A15 * A20 * A33 * A41 * A54 + A15 * A20 * A33 * A44 * A51 + A15 * A20 * A34 * A41 * A53 - A15 * A20 * A34 * A43 * A51 - A15 * A21 * A30 * A43 * A54 + A15 * A21 * A30 * A44 * A53 + A15 * A21 * A33 * A40 * A54 - A15 * A21 * A33 * A44 * A50 - A15 * A21 * A34 * A40 * A53 + A15 * A21 * A34 * A43 * A50 + A15 * A23 * A30 * A41 * A54 - A15 * A23 * A30 * A44 * A51 - A15 * A23 * A31 * A40 * A54 + A15 * A23 * A31 * A44 * A50 + A15 * A23 * A34 * A40 * A51 - A15 * A23 * A34 * A41 * A50 - A15 * A24 * A30 * A41 * A53 + A15 * A24 * A30 * A43 * A51 + A15 * A24 * A31 * A40 * A53 - A15 * A24 * A31 * A43 * A50 - A15 * A24 * A33 * A40 * A51 + A15 * A24 * A33 * A41 * A50) / detA;
    d[2][1] = (A00 * A21 * A33 * A45 * A54 - A00 * A21 * A33 * A44 * A55 + A00 * A21 * A34 * A43 * A55 - A00 * A21 * A34 * A45 * A53 - A00 * A21 * A35 * A43 * A54 + A00 * A21 * A35 * A44 * A53 + A00 * A23 * A31 * A44 * A55 - A00 * A23 * A31 * A45 * A54 - A00 * A23 * A34 * A41 * A55 + A00 * A23 * A34 * A45 * A51 + A00 * A23 * A35 * A41 * A54 - A00 * A23 * A35 * A44 * A51 - A00 * A24 * A31 * A43 * A55 + A00 * A24 * A31 * A45 * A53 + A00 * A24 * A33 * A41 * A55 - A00 * A24 * A33 * A45 * A51 - A00 * A24 * A35 * A41 * A53 + A00 * A24 * A35 * A43 * A51 + A00 * A25 * A31 * A43 * A54 - A00 * A25 * A31 * A44 * A53 - A00 * A25 * A33 * A41 * A54 + A00 * A25 * A33 * A44 * A51 + A00 * A25 * A34 * A41 * A53 - A00 * A25 * A34 * A43 * A51 + A01 * A20 * A33 * A44 * A55 - A01 * A20 * A33 * A45 * A54 - A01 * A20 * A34 * A43 * A55 + A01 * A20 * A34 * A45 * A53 + A01 * A20 * A35 * A43 * A54 - A01 * A20 * A35 * A44 * A53 - A01 * A23 * A30 * A44 * A55 + A01 * A23 * A30 * A45 * A54 + A01 * A23 * A34 * A40 * A55 - A01 * A23 * A34 * A45 * A50 - A01 * A23 * A35 * A40 * A54 + A01 * A23 * A35 * A44 * A50 + A01 * A24 * A30 * A43 * A55 - A01 * A24 * A30 * A45 * A53 - A01 * A24 * A33 * A40 * A55 + A01 * A24 * A33 * A45 * A50 + A01 * A24 * A35 * A40 * A53 - A01 * A24 * A35 * A43 * A50 - A01 * A25 * A30 * A43 * A54 + A01 * A25 * A30 * A44 * A53 + A01 * A25 * A33 * A40 * A54 - A01 * A25 * A33 * A44 * A50 - A01 * A25 * A34 * A40 * A53 + A01 * A25 * A34 * A43 * A50 - A03 * A20 * A31 * A44 * A55 + A03 * A20 * A31 * A45 * A54 + A03 * A20 * A34 * A41 * A55 - A03 * A20 * A34 * A45 * A51 - A03 * A20 * A35 * A41 * A54 + A03 * A20 * A35 * A44 * A51 + A03 * A21 * A30 * A44 * A55 - A03 * A21 * A30 * A45 * A54 - A03 * A21 * A34 * A40 * A55 + A03 * A21 * A34 * A45 * A50 + A03 * A21 * A35 * A40 * A54 - A03 * A21 * A35 * A44 * A50 - A03 * A24 * A30 * A41 * A55 + A03 * A24 * A30 * A45 * A51 + A03 * A24 * A31 * A40 * A55 - A03 * A24 * A31 * A45 * A50 - A03 * A24 * A35 * A40 * A51 + A03 * A24 * A35 * A41 * A50 + A03 * A25 * A30 * A41 * A54 - A03 * A25 * A30 * A44 * A51 - A03 * A25 * A31 * A40 * A54 + A03 * A25 * A31 * A44 * A50 + A03 * A25 * A34 * A40 * A51 - A03 * A25 * A34 * A41 * A50 + A04 * A20 * A31 * A43 * A55 - A04 * A20 * A31 * A45 * A53 - A04 * A20 * A33 * A41 * A55 + A04 * A20 * A33 * A45 * A51 + A04 * A20 * A35 * A41 * A53 - A04 * A20 * A35 * A43 * A51 - A04 * A21 * A30 * A43 * A55 + A04 * A21 * A30 * A45 * A53 + A04 * A21 * A33 * A40 * A55 - A04 * A21 * A33 * A45 * A50 - A04 * A21 * A35 * A40 * A53 + A04 * A21 * A35 * A43 * A50 + A04 * A23 * A30 * A41 * A55 - A04 * A23 * A30 * A45 * A51 - A04 * A23 * A31 * A40 * A55 + A04 * A23 * A31 * A45 * A50 + A04 * A23 * A35 * A40 * A51 - A04 * A23 * A35 * A41 * A50 - A04 * A25 * A30 * A41 * A53 + A04 * A25 * A30 * A43 * A51 + A04 * A25 * A31 * A40 * A53 - A04 * A25 * A31 * A43 * A50 - A04 * A25 * A33 * A40 * A51 + A04 * A25 * A33 * A41 * A50 - A05 * A20 * A31 * A43 * A54 + A05 * A20 * A31 * A44 * A53 + A05 * A20 * A33 * A41 * A54 - A05 * A20 * A33 * A44 * A51 - A05 * A20 * A34 * A41 * A53 + A05 * A20 * A34 * A43 * A51 + A05 * A21 * A30 * A43 * A54 - A05 * A21 * A30 * A44 * A53 - A05 * A21 * A33 * A40 * A54 + A05 * A21 * A33 * A44 * A50 + A05 * A21 * A34 * A40 * A53 - A05 * A21 * A34 * A43 * A50 - A05 * A23 * A30 * A41 * A54 + A05 * A23 * A30 * A44 * A51 + A05 * A23 * A31 * A40 * A54 - A05 * A23 * A31 * A44 * A50 - A05 * A23 * A34 * A40 * A51 + A05 * A23 * A34 * A41 * A50 + A05 * A24 * A30 * A41 * A53 - A05 * A24 * A30 * A43 * A51 - A05 * A24 * A31 * A40 * A53 + A05 * A24 * A31 * A43 * A50 + A05 * A24 * A33 * A40 * A51 - A05 * A24 * A33 * A41 * A50) / detA;
    d[2][2] = (A00 * A11 * A33 * A44 * A55 - A00 * A11 * A33 * A45 * A54 - A00 * A11 * A34 * A43 * A55 + A00 * A11 * A34 * A45 * A53 + A00 * A11 * A35 * A43 * A54 - A00 * A11 * A35 * A44 * A53 - A00 * A13 * A31 * A44 * A55 + A00 * A13 * A31 * A45 * A54 + A00 * A13 * A34 * A41 * A55 - A00 * A13 * A34 * A45 * A51 - A00 * A13 * A35 * A41 * A54 + A00 * A13 * A35 * A44 * A51 + A00 * A14 * A31 * A43 * A55 - A00 * A14 * A31 * A45 * A53 - A00 * A14 * A33 * A41 * A55 + A00 * A14 * A33 * A45 * A51 + A00 * A14 * A35 * A41 * A53 - A00 * A14 * A35 * A43 * A51 - A00 * A15 * A31 * A43 * A54 + A00 * A15 * A31 * A44 * A53 + A00 * A15 * A33 * A41 * A54 - A00 * A15 * A33 * A44 * A51 - A00 * A15 * A34 * A41 * A53 + A00 * A15 * A34 * A43 * A51 - A01 * A10 * A33 * A44 * A55 + A01 * A10 * A33 * A45 * A54 + A01 * A10 * A34 * A43 * A55 - A01 * A10 * A34 * A45 * A53 - A01 * A10 * A35 * A43 * A54 + A01 * A10 * A35 * A44 * A53 + A01 * A13 * A30 * A44 * A55 - A01 * A13 * A30 * A45 * A54 - A01 * A13 * A34 * A40 * A55 + A01 * A13 * A34 * A45 * A50 + A01 * A13 * A35 * A40 * A54 - A01 * A13 * A35 * A44 * A50 - A01 * A14 * A30 * A43 * A55 + A01 * A14 * A30 * A45 * A53 + A01 * A14 * A33 * A40 * A55 - A01 * A14 * A33 * A45 * A50 - A01 * A14 * A35 * A40 * A53 + A01 * A14 * A35 * A43 * A50 + A01 * A15 * A30 * A43 * A54 - A01 * A15 * A30 * A44 * A53 - A01 * A15 * A33 * A40 * A54 + A01 * A15 * A33 * A44 * A50 + A01 * A15 * A34 * A40 * A53 - A01 * A15 * A34 * A43 * A50 + A03 * A10 * A31 * A44 * A55 - A03 * A10 * A31 * A45 * A54 - A03 * A10 * A34 * A41 * A55 + A03 * A10 * A34 * A45 * A51 + A03 * A10 * A35 * A41 * A54 - A03 * A10 * A35 * A44 * A51 - A03 * A11 * A30 * A44 * A55 + A03 * A11 * A30 * A45 * A54 + A03 * A11 * A34 * A40 * A55 - A03 * A11 * A34 * A45 * A50 - A03 * A11 * A35 * A40 * A54 + A03 * A11 * A35 * A44 * A50 + A03 * A14 * A30 * A41 * A55 - A03 * A14 * A30 * A45 * A51 - A03 * A14 * A31 * A40 * A55 + A03 * A14 * A31 * A45 * A50 + A03 * A14 * A35 * A40 * A51 - A03 * A14 * A35 * A41 * A50 - A03 * A15 * A30 * A41 * A54 + A03 * A15 * A30 * A44 * A51 + A03 * A15 * A31 * A40 * A54 - A03 * A15 * A31 * A44 * A50 - A03 * A15 * A34 * A40 * A51 + A03 * A15 * A34 * A41 * A50 - A04 * A10 * A31 * A43 * A55 + A04 * A10 * A31 * A45 * A53 + A04 * A10 * A33 * A41 * A55 - A04 * A10 * A33 * A45 * A51 - A04 * A10 * A35 * A41 * A53 + A04 * A10 * A35 * A43 * A51 + A04 * A11 * A30 * A43 * A55 - A04 * A11 * A30 * A45 * A53 - A04 * A11 * A33 * A40 * A55 + A04 * A11 * A33 * A45 * A50 + A04 * A11 * A35 * A40 * A53 - A04 * A11 * A35 * A43 * A50 - A04 * A13 * A30 * A41 * A55 + A04 * A13 * A30 * A45 * A51 + A04 * A13 * A31 * A40 * A55 - A04 * A13 * A31 * A45 * A50 - A04 * A13 * A35 * A40 * A51 + A04 * A13 * A35 * A41 * A50 + A04 * A15 * A30 * A41 * A53 - A04 * A15 * A30 * A43 * A51 - A04 * A15 * A31 * A40 * A53 + A04 * A15 * A31 * A43 * A50 + A04 * A15 * A33 * A40 * A51 - A04 * A15 * A33 * A41 * A50 + A05 * A10 * A31 * A43 * A54 - A05 * A10 * A31 * A44 * A53 - A05 * A10 * A33 * A41 * A54 + A05 * A10 * A33 * A44 * A51 + A05 * A10 * A34 * A41 * A53 - A05 * A10 * A34 * A43 * A51 - A05 * A11 * A30 * A43 * A54 + A05 * A11 * A30 * A44 * A53 + A05 * A11 * A33 * A40 * A54 - A05 * A11 * A33 * A44 * A50 - A05 * A11 * A34 * A40 * A53 + A05 * A11 * A34 * A43 * A50 + A05 * A13 * A30 * A41 * A54 - A05 * A13 * A30 * A44 * A51 - A05 * A13 * A31 * A40 * A54 + A05 * A13 * A31 * A44 * A50 + A05 * A13 * A34 * A40 * A51 - A05 * A13 * A34 * A41 * A50 - A05 * A14 * A30 * A41 * A53 + A05 * A14 * A30 * A43 * A51 + A05 * A14 * A31 * A40 * A53 - A05 * A14 * A31 * A43 * A50 - A05 * A14 * A33 * A40 * A51 + A05 * A14 * A33 * A41 * A50) / detA;
    d[2][3] = (A00 * A11 * A23 * A45 * A54 - A00 * A11 * A23 * A44 * A55 + A00 * A11 * A24 * A43 * A55 - A00 * A11 * A24 * A45 * A53 - A00 * A11 * A25 * A43 * A54 + A00 * A11 * A25 * A44 * A53 + A00 * A13 * A21 * A44 * A55 - A00 * A13 * A21 * A45 * A54 - A00 * A13 * A24 * A41 * A55 + A00 * A13 * A24 * A45 * A51 + A00 * A13 * A25 * A41 * A54 - A00 * A13 * A25 * A44 * A51 - A00 * A14 * A21 * A43 * A55 + A00 * A14 * A21 * A45 * A53 + A00 * A14 * A23 * A41 * A55 - A00 * A14 * A23 * A45 * A51 - A00 * A14 * A25 * A41 * A53 + A00 * A14 * A25 * A43 * A51 + A00 * A15 * A21 * A43 * A54 - A00 * A15 * A21 * A44 * A53 - A00 * A15 * A23 * A41 * A54 + A00 * A15 * A23 * A44 * A51 + A00 * A15 * A24 * A41 * A53 - A00 * A15 * A24 * A43 * A51 + A01 * A10 * A23 * A44 * A55 - A01 * A10 * A23 * A45 * A54 - A01 * A10 * A24 * A43 * A55 + A01 * A10 * A24 * A45 * A53 + A01 * A10 * A25 * A43 * A54 - A01 * A10 * A25 * A44 * A53 - A01 * A13 * A20 * A44 * A55 + A01 * A13 * A20 * A45 * A54 + A01 * A13 * A24 * A40 * A55 - A01 * A13 * A24 * A45 * A50 - A01 * A13 * A25 * A40 * A54 + A01 * A13 * A25 * A44 * A50 + A01 * A14 * A20 * A43 * A55 - A01 * A14 * A20 * A45 * A53 - A01 * A14 * A23 * A40 * A55 + A01 * A14 * A23 * A45 * A50 + A01 * A14 * A25 * A40 * A53 - A01 * A14 * A25 * A43 * A50 - A01 * A15 * A20 * A43 * A54 + A01 * A15 * A20 * A44 * A53 + A01 * A15 * A23 * A40 * A54 - A01 * A15 * A23 * A44 * A50 - A01 * A15 * A24 * A40 * A53 + A01 * A15 * A24 * A43 * A50 - A03 * A10 * A21 * A44 * A55 + A03 * A10 * A21 * A45 * A54 + A03 * A10 * A24 * A41 * A55 - A03 * A10 * A24 * A45 * A51 - A03 * A10 * A25 * A41 * A54 + A03 * A10 * A25 * A44 * A51 + A03 * A11 * A20 * A44 * A55 - A03 * A11 * A20 * A45 * A54 - A03 * A11 * A24 * A40 * A55 + A03 * A11 * A24 * A45 * A50 + A03 * A11 * A25 * A40 * A54 - A03 * A11 * A25 * A44 * A50 - A03 * A14 * A20 * A41 * A55 + A03 * A14 * A20 * A45 * A51 + A03 * A14 * A21 * A40 * A55 - A03 * A14 * A21 * A45 * A50 - A03 * A14 * A25 * A40 * A51 + A03 * A14 * A25 * A41 * A50 + A03 * A15 * A20 * A41 * A54 - A03 * A15 * A20 * A44 * A51 - A03 * A15 * A21 * A40 * A54 + A03 * A15 * A21 * A44 * A50 + A03 * A15 * A24 * A40 * A51 - A03 * A15 * A24 * A41 * A50 + A04 * A10 * A21 * A43 * A55 - A04 * A10 * A21 * A45 * A53 - A04 * A10 * A23 * A41 * A55 + A04 * A10 * A23 * A45 * A51 + A04 * A10 * A25 * A41 * A53 - A04 * A10 * A25 * A43 * A51 - A04 * A11 * A20 * A43 * A55 + A04 * A11 * A20 * A45 * A53 + A04 * A11 * A23 * A40 * A55 - A04 * A11 * A23 * A45 * A50 - A04 * A11 * A25 * A40 * A53 + A04 * A11 * A25 * A43 * A50 + A04 * A13 * A20 * A41 * A55 - A04 * A13 * A20 * A45 * A51 - A04 * A13 * A21 * A40 * A55 + A04 * A13 * A21 * A45 * A50 + A04 * A13 * A25 * A40 * A51 - A04 * A13 * A25 * A41 * A50 - A04 * A15 * A20 * A41 * A53 + A04 * A15 * A20 * A43 * A51 + A04 * A15 * A21 * A40 * A53 - A04 * A15 * A21 * A43 * A50 - A04 * A15 * A23 * A40 * A51 + A04 * A15 * A23 * A41 * A50 - A05 * A10 * A21 * A43 * A54 + A05 * A10 * A21 * A44 * A53 + A05 * A10 * A23 * A41 * A54 - A05 * A10 * A23 * A44 * A51 - A05 * A10 * A24 * A41 * A53 + A05 * A10 * A24 * A43 * A51 + A05 * A11 * A20 * A43 * A54 - A05 * A11 * A20 * A44 * A53 - A05 * A11 * A23 * A40 * A54 + A05 * A11 * A23 * A44 * A50 + A05 * A11 * A24 * A40 * A53 - A05 * A11 * A24 * A43 * A50 - A05 * A13 * A20 * A41 * A54 + A05 * A13 * A20 * A44 * A51 + A05 * A13 * A21 * A40 * A54 - A05 * A13 * A21 * A44 * A50 - A05 * A13 * A24 * A40 * A51 + A05 * A13 * A24 * A41 * A50 + A05 * A14 * A20 * A41 * A53 - A05 * A14 * A20 * A43 * A51 - A05 * A14 * A21 * A40 * A53 + A05 * A14 * A21 * A43 * A50 + A05 * A14 * A23 * A40 * A51 - A05 * A14 * A23 * A41 * A50) / detA;
    d[2][4] = (A00 * A11 * A23 * A34 * A55 - A00 * A11 * A23 * A35 * A54 - A00 * A11 * A24 * A33 * A55 + A00 * A11 * A24 * A35 * A53 + A00 * A11 * A25 * A33 * A54 - A00 * A11 * A25 * A34 * A53 - A00 * A13 * A21 * A34 * A55 + A00 * A13 * A21 * A35 * A54 + A00 * A13 * A24 * A31 * A55 - A00 * A13 * A24 * A35 * A51 - A00 * A13 * A25 * A31 * A54 + A00 * A13 * A25 * A34 * A51 + A00 * A14 * A21 * A33 * A55 - A00 * A14 * A21 * A35 * A53 - A00 * A14 * A23 * A31 * A55 + A00 * A14 * A23 * A35 * A51 + A00 * A14 * A25 * A31 * A53 - A00 * A14 * A25 * A33 * A51 - A00 * A15 * A21 * A33 * A54 + A00 * A15 * A21 * A34 * A53 + A00 * A15 * A23 * A31 * A54 - A00 * A15 * A23 * A34 * A51 - A00 * A15 * A24 * A31 * A53 + A00 * A15 * A24 * A33 * A51 - A01 * A10 * A23 * A34 * A55 + A01 * A10 * A23 * A35 * A54 + A01 * A10 * A24 * A33 * A55 - A01 * A10 * A24 * A35 * A53 - A01 * A10 * A25 * A33 * A54 + A01 * A10 * A25 * A34 * A53 + A01 * A13 * A20 * A34 * A55 - A01 * A13 * A20 * A35 * A54 - A01 * A13 * A24 * A30 * A55 + A01 * A13 * A24 * A35 * A50 + A01 * A13 * A25 * A30 * A54 - A01 * A13 * A25 * A34 * A50 - A01 * A14 * A20 * A33 * A55 + A01 * A14 * A20 * A35 * A53 + A01 * A14 * A23 * A30 * A55 - A01 * A14 * A23 * A35 * A50 - A01 * A14 * A25 * A30 * A53 + A01 * A14 * A25 * A33 * A50 + A01 * A15 * A20 * A33 * A54 - A01 * A15 * A20 * A34 * A53 - A01 * A15 * A23 * A30 * A54 + A01 * A15 * A23 * A34 * A50 + A01 * A15 * A24 * A30 * A53 - A01 * A15 * A24 * A33 * A50 + A03 * A10 * A21 * A34 * A55 - A03 * A10 * A21 * A35 * A54 - A03 * A10 * A24 * A31 * A55 + A03 * A10 * A24 * A35 * A51 + A03 * A10 * A25 * A31 * A54 - A03 * A10 * A25 * A34 * A51 - A03 * A11 * A20 * A34 * A55 + A03 * A11 * A20 * A35 * A54 + A03 * A11 * A24 * A30 * A55 - A03 * A11 * A24 * A35 * A50 - A03 * A11 * A25 * A30 * A54 + A03 * A11 * A25 * A34 * A50 + A03 * A14 * A20 * A31 * A55 - A03 * A14 * A20 * A35 * A51 - A03 * A14 * A21 * A30 * A55 + A03 * A14 * A21 * A35 * A50 + A03 * A14 * A25 * A30 * A51 - A03 * A14 * A25 * A31 * A50 - A03 * A15 * A20 * A31 * A54 + A03 * A15 * A20 * A34 * A51 + A03 * A15 * A21 * A30 * A54 - A03 * A15 * A21 * A34 * A50 - A03 * A15 * A24 * A30 * A51 + A03 * A15 * A24 * A31 * A50 - A04 * A10 * A21 * A33 * A55 + A04 * A10 * A21 * A35 * A53 + A04 * A10 * A23 * A31 * A55 - A04 * A10 * A23 * A35 * A51 - A04 * A10 * A25 * A31 * A53 + A04 * A10 * A25 * A33 * A51 + A04 * A11 * A20 * A33 * A55 - A04 * A11 * A20 * A35 * A53 - A04 * A11 * A23 * A30 * A55 + A04 * A11 * A23 * A35 * A50 + A04 * A11 * A25 * A30 * A53 - A04 * A11 * A25 * A33 * A50 - A04 * A13 * A20 * A31 * A55 + A04 * A13 * A20 * A35 * A51 + A04 * A13 * A21 * A30 * A55 - A04 * A13 * A21 * A35 * A50 - A04 * A13 * A25 * A30 * A51 + A04 * A13 * A25 * A31 * A50 + A04 * A15 * A20 * A31 * A53 - A04 * A15 * A20 * A33 * A51 - A04 * A15 * A21 * A30 * A53 + A04 * A15 * A21 * A33 * A50 + A04 * A15 * A23 * A30 * A51 - A04 * A15 * A23 * A31 * A50 + A05 * A10 * A21 * A33 * A54 - A05 * A10 * A21 * A34 * A53 - A05 * A10 * A23 * A31 * A54 + A05 * A10 * A23 * A34 * A51 + A05 * A10 * A24 * A31 * A53 - A05 * A10 * A24 * A33 * A51 - A05 * A11 * A20 * A33 * A54 + A05 * A11 * A20 * A34 * A53 + A05 * A11 * A23 * A30 * A54 - A05 * A11 * A23 * A34 * A50 - A05 * A11 * A24 * A30 * A53 + A05 * A11 * A24 * A33 * A50 + A05 * A13 * A20 * A31 * A54 - A05 * A13 * A20 * A34 * A51 - A05 * A13 * A21 * A30 * A54 + A05 * A13 * A21 * A34 * A50 + A05 * A13 * A24 * A30 * A51 - A05 * A13 * A24 * A31 * A50 - A05 * A14 * A20 * A31 * A53 + A05 * A14 * A20 * A33 * A51 + A05 * A14 * A21 * A30 * A53 - A05 * A14 * A21 * A33 * A50 - A05 * A14 * A23 * A30 * A51 + A05 * A14 * A23 * A31 * A50) / detA;
    d[2][5] = (A00 * A11 * A23 * A35 * A44 - A00 * A11 * A23 * A34 * A45 + A00 * A11 * A24 * A33 * A45 - A00 * A11 * A24 * A35 * A43 - A00 * A11 * A25 * A33 * A44 + A00 * A11 * A25 * A34 * A43 + A00 * A13 * A21 * A34 * A45 - A00 * A13 * A21 * A35 * A44 - A00 * A13 * A24 * A31 * A45 + A00 * A13 * A24 * A35 * A41 + A00 * A13 * A25 * A31 * A44 - A00 * A13 * A25 * A34 * A41 - A00 * A14 * A21 * A33 * A45 + A00 * A14 * A21 * A35 * A43 + A00 * A14 * A23 * A31 * A45 - A00 * A14 * A23 * A35 * A41 - A00 * A14 * A25 * A31 * A43 + A00 * A14 * A25 * A33 * A41 + A00 * A15 * A21 * A33 * A44 - A00 * A15 * A21 * A34 * A43 - A00 * A15 * A23 * A31 * A44 + A00 * A15 * A23 * A34 * A41 + A00 * A15 * A24 * A31 * A43 - A00 * A15 * A24 * A33 * A41 + A01 * A10 * A23 * A34 * A45 - A01 * A10 * A23 * A35 * A44 - A01 * A10 * A24 * A33 * A45 + A01 * A10 * A24 * A35 * A43 + A01 * A10 * A25 * A33 * A44 - A01 * A10 * A25 * A34 * A43 - A01 * A13 * A20 * A34 * A45 + A01 * A13 * A20 * A35 * A44 + A01 * A13 * A24 * A30 * A45 - A01 * A13 * A24 * A35 * A40 - A01 * A13 * A25 * A30 * A44 + A01 * A13 * A25 * A34 * A40 + A01 * A14 * A20 * A33 * A45 - A01 * A14 * A20 * A35 * A43 - A01 * A14 * A23 * A30 * A45 + A01 * A14 * A23 * A35 * A40 + A01 * A14 * A25 * A30 * A43 - A01 * A14 * A25 * A33 * A40 - A01 * A15 * A20 * A33 * A44 + A01 * A15 * A20 * A34 * A43 + A01 * A15 * A23 * A30 * A44 - A01 * A15 * A23 * A34 * A40 - A01 * A15 * A24 * A30 * A43 + A01 * A15 * A24 * A33 * A40 - A03 * A10 * A21 * A34 * A45 + A03 * A10 * A21 * A35 * A44 + A03 * A10 * A24 * A31 * A45 - A03 * A10 * A24 * A35 * A41 - A03 * A10 * A25 * A31 * A44 + A03 * A10 * A25 * A34 * A41 + A03 * A11 * A20 * A34 * A45 - A03 * A11 * A20 * A35 * A44 - A03 * A11 * A24 * A30 * A45 + A03 * A11 * A24 * A35 * A40 + A03 * A11 * A25 * A30 * A44 - A03 * A11 * A25 * A34 * A40 - A03 * A14 * A20 * A31 * A45 + A03 * A14 * A20 * A35 * A41 + A03 * A14 * A21 * A30 * A45 - A03 * A14 * A21 * A35 * A40 - A03 * A14 * A25 * A30 * A41 + A03 * A14 * A25 * A31 * A40 + A03 * A15 * A20 * A31 * A44 - A03 * A15 * A20 * A34 * A41 - A03 * A15 * A21 * A30 * A44 + A03 * A15 * A21 * A34 * A40 + A03 * A15 * A24 * A30 * A41 - A03 * A15 * A24 * A31 * A40 + A04 * A10 * A21 * A33 * A45 - A04 * A10 * A21 * A35 * A43 - A04 * A10 * A23 * A31 * A45 + A04 * A10 * A23 * A35 * A41 + A04 * A10 * A25 * A31 * A43 - A04 * A10 * A25 * A33 * A41 - A04 * A11 * A20 * A33 * A45 + A04 * A11 * A20 * A35 * A43 + A04 * A11 * A23 * A30 * A45 - A04 * A11 * A23 * A35 * A40 - A04 * A11 * A25 * A30 * A43 + A04 * A11 * A25 * A33 * A40 + A04 * A13 * A20 * A31 * A45 - A04 * A13 * A20 * A35 * A41 - A04 * A13 * A21 * A30 * A45 + A04 * A13 * A21 * A35 * A40 + A04 * A13 * A25 * A30 * A41 - A04 * A13 * A25 * A31 * A40 - A04 * A15 * A20 * A31 * A43 + A04 * A15 * A20 * A33 * A41 + A04 * A15 * A21 * A30 * A43 - A04 * A15 * A21 * A33 * A40 - A04 * A15 * A23 * A30 * A41 + A04 * A15 * A23 * A31 * A40 - A05 * A10 * A21 * A33 * A44 + A05 * A10 * A21 * A34 * A43 + A05 * A10 * A23 * A31 * A44 - A05 * A10 * A23 * A34 * A41 - A05 * A10 * A24 * A31 * A43 + A05 * A10 * A24 * A33 * A41 + A05 * A11 * A20 * A33 * A44 - A05 * A11 * A20 * A34 * A43 - A05 * A11 * A23 * A30 * A44 + A05 * A11 * A23 * A34 * A40 + A05 * A11 * A24 * A30 * A43 - A05 * A11 * A24 * A33 * A40 - A05 * A13 * A20 * A31 * A44 + A05 * A13 * A20 * A34 * A41 + A05 * A13 * A21 * A30 * A44 - A05 * A13 * A21 * A34 * A40 - A05 * A13 * A24 * A30 * A41 + A05 * A13 * A24 * A31 * A40 + A05 * A14 * A20 * A31 * A43 - A05 * A14 * A20 * A33 * A41 - A05 * A14 * A21 * A30 * A43 + A05 * A14 * A21 * A33 * A40 + A05 * A14 * A23 * A30 * A41 - A05 * A14 * A23 * A31 * A40) / detA;


    d[3][0] = (A10 * A21 * A32 * A45 * A54 - A10 * A21 * A32 * A44 * A55 + A10 * A21 * A34 * A42 * A55 - A10 * A21 * A34 * A45 * A52 - A10 * A21 * A35 * A42 * A54 + A10 * A21 * A35 * A44 * A52 + A10 * A22 * A31 * A44 * A55 - A10 * A22 * A31 * A45 * A54 - A10 * A22 * A34 * A41 * A55 + A10 * A22 * A34 * A45 * A51 + A10 * A22 * A35 * A41 * A54 - A10 * A22 * A35 * A44 * A51 - A10 * A24 * A31 * A42 * A55 + A10 * A24 * A31 * A45 * A52 + A10 * A24 * A32 * A41 * A55 - A10 * A24 * A32 * A45 * A51 - A10 * A24 * A35 * A41 * A52 + A10 * A24 * A35 * A42 * A51 + A10 * A25 * A31 * A42 * A54 - A10 * A25 * A31 * A44 * A52 - A10 * A25 * A32 * A41 * A54 + A10 * A25 * A32 * A44 * A51 + A10 * A25 * A34 * A41 * A52 - A10 * A25 * A34 * A42 * A51 + A11 * A20 * A32 * A44 * A55 - A11 * A20 * A32 * A45 * A54 - A11 * A20 * A34 * A42 * A55 + A11 * A20 * A34 * A45 * A52 + A11 * A20 * A35 * A42 * A54 - A11 * A20 * A35 * A44 * A52 - A11 * A22 * A30 * A44 * A55 + A11 * A22 * A30 * A45 * A54 + A11 * A22 * A34 * A40 * A55 - A11 * A22 * A34 * A45 * A50 - A11 * A22 * A35 * A40 * A54 + A11 * A22 * A35 * A44 * A50 + A11 * A24 * A30 * A42 * A55 - A11 * A24 * A30 * A45 * A52 - A11 * A24 * A32 * A40 * A55 + A11 * A24 * A32 * A45 * A50 + A11 * A24 * A35 * A40 * A52 - A11 * A24 * A35 * A42 * A50 - A11 * A25 * A30 * A42 * A54 + A11 * A25 * A30 * A44 * A52 + A11 * A25 * A32 * A40 * A54 - A11 * A25 * A32 * A44 * A50 - A11 * A25 * A34 * A40 * A52 + A11 * A25 * A34 * A42 * A50 - A12 * A20 * A31 * A44 * A55 + A12 * A20 * A31 * A45 * A54 + A12 * A20 * A34 * A41 * A55 - A12 * A20 * A34 * A45 * A51 - A12 * A20 * A35 * A41 * A54 + A12 * A20 * A35 * A44 * A51 + A12 * A21 * A30 * A44 * A55 - A12 * A21 * A30 * A45 * A54 - A12 * A21 * A34 * A40 * A55 + A12 * A21 * A34 * A45 * A50 + A12 * A21 * A35 * A40 * A54 - A12 * A21 * A35 * A44 * A50 - A12 * A24 * A30 * A41 * A55 + A12 * A24 * A30 * A45 * A51 + A12 * A24 * A31 * A40 * A55 - A12 * A24 * A31 * A45 * A50 - A12 * A24 * A35 * A40 * A51 + A12 * A24 * A35 * A41 * A50 + A12 * A25 * A30 * A41 * A54 - A12 * A25 * A30 * A44 * A51 - A12 * A25 * A31 * A40 * A54 + A12 * A25 * A31 * A44 * A50 + A12 * A25 * A34 * A40 * A51 - A12 * A25 * A34 * A41 * A50 + A14 * A20 * A31 * A42 * A55 - A14 * A20 * A31 * A45 * A52 - A14 * A20 * A32 * A41 * A55 + A14 * A20 * A32 * A45 * A51 + A14 * A20 * A35 * A41 * A52 - A14 * A20 * A35 * A42 * A51 - A14 * A21 * A30 * A42 * A55 + A14 * A21 * A30 * A45 * A52 + A14 * A21 * A32 * A40 * A55 - A14 * A21 * A32 * A45 * A50 - A14 * A21 * A35 * A40 * A52 + A14 * A21 * A35 * A42 * A50 + A14 * A22 * A30 * A41 * A55 - A14 * A22 * A30 * A45 * A51 - A14 * A22 * A31 * A40 * A55 + A14 * A22 * A31 * A45 * A50 + A14 * A22 * A35 * A40 * A51 - A14 * A22 * A35 * A41 * A50 - A14 * A25 * A30 * A41 * A52 + A14 * A25 * A30 * A42 * A51 + A14 * A25 * A31 * A40 * A52 - A14 * A25 * A31 * A42 * A50 - A14 * A25 * A32 * A40 * A51 + A14 * A25 * A32 * A41 * A50 - A15 * A20 * A31 * A42 * A54 + A15 * A20 * A31 * A44 * A52 + A15 * A20 * A32 * A41 * A54 - A15 * A20 * A32 * A44 * A51 - A15 * A20 * A34 * A41 * A52 + A15 * A20 * A34 * A42 * A51 + A15 * A21 * A30 * A42 * A54 - A15 * A21 * A30 * A44 * A52 - A15 * A21 * A32 * A40 * A54 + A15 * A21 * A32 * A44 * A50 + A15 * A21 * A34 * A40 * A52 - A15 * A21 * A34 * A42 * A50 - A15 * A22 * A30 * A41 * A54 + A15 * A22 * A30 * A44 * A51 + A15 * A22 * A31 * A40 * A54 - A15 * A22 * A31 * A44 * A50 - A15 * A22 * A34 * A40 * A51 + A15 * A22 * A34 * A41 * A50 + A15 * A24 * A30 * A41 * A52 - A15 * A24 * A30 * A42 * A51 - A15 * A24 * A31 * A40 * A52 + A15 * A24 * A31 * A42 * A50 + A15 * A24 * A32 * A40 * A51 - A15 * A24 * A32 * A41 * A50) / detA;
    d[3][1] = (A00 * A21 * A32 * A44 * A55 - A00 * A21 * A32 * A45 * A54 - A00 * A21 * A34 * A42 * A55 + A00 * A21 * A34 * A45 * A52 + A00 * A21 * A35 * A42 * A54 - A00 * A21 * A35 * A44 * A52 - A00 * A22 * A31 * A44 * A55 + A00 * A22 * A31 * A45 * A54 + A00 * A22 * A34 * A41 * A55 - A00 * A22 * A34 * A45 * A51 - A00 * A22 * A35 * A41 * A54 + A00 * A22 * A35 * A44 * A51 + A00 * A24 * A31 * A42 * A55 - A00 * A24 * A31 * A45 * A52 - A00 * A24 * A32 * A41 * A55 + A00 * A24 * A32 * A45 * A51 + A00 * A24 * A35 * A41 * A52 - A00 * A24 * A35 * A42 * A51 - A00 * A25 * A31 * A42 * A54 + A00 * A25 * A31 * A44 * A52 + A00 * A25 * A32 * A41 * A54 - A00 * A25 * A32 * A44 * A51 - A00 * A25 * A34 * A41 * A52 + A00 * A25 * A34 * A42 * A51 - A01 * A20 * A32 * A44 * A55 + A01 * A20 * A32 * A45 * A54 + A01 * A20 * A34 * A42 * A55 - A01 * A20 * A34 * A45 * A52 - A01 * A20 * A35 * A42 * A54 + A01 * A20 * A35 * A44 * A52 + A01 * A22 * A30 * A44 * A55 - A01 * A22 * A30 * A45 * A54 - A01 * A22 * A34 * A40 * A55 + A01 * A22 * A34 * A45 * A50 + A01 * A22 * A35 * A40 * A54 - A01 * A22 * A35 * A44 * A50 - A01 * A24 * A30 * A42 * A55 + A01 * A24 * A30 * A45 * A52 + A01 * A24 * A32 * A40 * A55 - A01 * A24 * A32 * A45 * A50 - A01 * A24 * A35 * A40 * A52 + A01 * A24 * A35 * A42 * A50 + A01 * A25 * A30 * A42 * A54 - A01 * A25 * A30 * A44 * A52 - A01 * A25 * A32 * A40 * A54 + A01 * A25 * A32 * A44 * A50 + A01 * A25 * A34 * A40 * A52 - A01 * A25 * A34 * A42 * A50 + A02 * A20 * A31 * A44 * A55 - A02 * A20 * A31 * A45 * A54 - A02 * A20 * A34 * A41 * A55 + A02 * A20 * A34 * A45 * A51 + A02 * A20 * A35 * A41 * A54 - A02 * A20 * A35 * A44 * A51 - A02 * A21 * A30 * A44 * A55 + A02 * A21 * A30 * A45 * A54 + A02 * A21 * A34 * A40 * A55 - A02 * A21 * A34 * A45 * A50 - A02 * A21 * A35 * A40 * A54 + A02 * A21 * A35 * A44 * A50 + A02 * A24 * A30 * A41 * A55 - A02 * A24 * A30 * A45 * A51 - A02 * A24 * A31 * A40 * A55 + A02 * A24 * A31 * A45 * A50 + A02 * A24 * A35 * A40 * A51 - A02 * A24 * A35 * A41 * A50 - A02 * A25 * A30 * A41 * A54 + A02 * A25 * A30 * A44 * A51 + A02 * A25 * A31 * A40 * A54 - A02 * A25 * A31 * A44 * A50 - A02 * A25 * A34 * A40 * A51 + A02 * A25 * A34 * A41 * A50 - A04 * A20 * A31 * A42 * A55 + A04 * A20 * A31 * A45 * A52 + A04 * A20 * A32 * A41 * A55 - A04 * A20 * A32 * A45 * A51 - A04 * A20 * A35 * A41 * A52 + A04 * A20 * A35 * A42 * A51 + A04 * A21 * A30 * A42 * A55 - A04 * A21 * A30 * A45 * A52 - A04 * A21 * A32 * A40 * A55 + A04 * A21 * A32 * A45 * A50 + A04 * A21 * A35 * A40 * A52 - A04 * A21 * A35 * A42 * A50 - A04 * A22 * A30 * A41 * A55 + A04 * A22 * A30 * A45 * A51 + A04 * A22 * A31 * A40 * A55 - A04 * A22 * A31 * A45 * A50 - A04 * A22 * A35 * A40 * A51 + A04 * A22 * A35 * A41 * A50 + A04 * A25 * A30 * A41 * A52 - A04 * A25 * A30 * A42 * A51 - A04 * A25 * A31 * A40 * A52 + A04 * A25 * A31 * A42 * A50 + A04 * A25 * A32 * A40 * A51 - A04 * A25 * A32 * A41 * A50 + A05 * A20 * A31 * A42 * A54 - A05 * A20 * A31 * A44 * A52 - A05 * A20 * A32 * A41 * A54 + A05 * A20 * A32 * A44 * A51 + A05 * A20 * A34 * A41 * A52 - A05 * A20 * A34 * A42 * A51 - A05 * A21 * A30 * A42 * A54 + A05 * A21 * A30 * A44 * A52 + A05 * A21 * A32 * A40 * A54 - A05 * A21 * A32 * A44 * A50 - A05 * A21 * A34 * A40 * A52 + A05 * A21 * A34 * A42 * A50 + A05 * A22 * A30 * A41 * A54 - A05 * A22 * A30 * A44 * A51 - A05 * A22 * A31 * A40 * A54 + A05 * A22 * A31 * A44 * A50 + A05 * A22 * A34 * A40 * A51 - A05 * A22 * A34 * A41 * A50 - A05 * A24 * A30 * A41 * A52 + A05 * A24 * A30 * A42 * A51 + A05 * A24 * A31 * A40 * A52 - A05 * A24 * A31 * A42 * A50 - A05 * A24 * A32 * A40 * A51 + A05 * A24 * A32 * A41 * A50) / detA;
    d[3][2] = (A00 * A11 * A32 * A45 * A54 - A00 * A11 * A32 * A44 * A55 + A00 * A11 * A34 * A42 * A55 - A00 * A11 * A34 * A45 * A52 - A00 * A11 * A35 * A42 * A54 + A00 * A11 * A35 * A44 * A52 + A00 * A12 * A31 * A44 * A55 - A00 * A12 * A31 * A45 * A54 - A00 * A12 * A34 * A41 * A55 + A00 * A12 * A34 * A45 * A51 + A00 * A12 * A35 * A41 * A54 - A00 * A12 * A35 * A44 * A51 - A00 * A14 * A31 * A42 * A55 + A00 * A14 * A31 * A45 * A52 + A00 * A14 * A32 * A41 * A55 - A00 * A14 * A32 * A45 * A51 - A00 * A14 * A35 * A41 * A52 + A00 * A14 * A35 * A42 * A51 + A00 * A15 * A31 * A42 * A54 - A00 * A15 * A31 * A44 * A52 - A00 * A15 * A32 * A41 * A54 + A00 * A15 * A32 * A44 * A51 + A00 * A15 * A34 * A41 * A52 - A00 * A15 * A34 * A42 * A51 + A01 * A10 * A32 * A44 * A55 - A01 * A10 * A32 * A45 * A54 - A01 * A10 * A34 * A42 * A55 + A01 * A10 * A34 * A45 * A52 + A01 * A10 * A35 * A42 * A54 - A01 * A10 * A35 * A44 * A52 - A01 * A12 * A30 * A44 * A55 + A01 * A12 * A30 * A45 * A54 + A01 * A12 * A34 * A40 * A55 - A01 * A12 * A34 * A45 * A50 - A01 * A12 * A35 * A40 * A54 + A01 * A12 * A35 * A44 * A50 + A01 * A14 * A30 * A42 * A55 - A01 * A14 * A30 * A45 * A52 - A01 * A14 * A32 * A40 * A55 + A01 * A14 * A32 * A45 * A50 + A01 * A14 * A35 * A40 * A52 - A01 * A14 * A35 * A42 * A50 - A01 * A15 * A30 * A42 * A54 + A01 * A15 * A30 * A44 * A52 + A01 * A15 * A32 * A40 * A54 - A01 * A15 * A32 * A44 * A50 - A01 * A15 * A34 * A40 * A52 + A01 * A15 * A34 * A42 * A50 - A02 * A10 * A31 * A44 * A55 + A02 * A10 * A31 * A45 * A54 + A02 * A10 * A34 * A41 * A55 - A02 * A10 * A34 * A45 * A51 - A02 * A10 * A35 * A41 * A54 + A02 * A10 * A35 * A44 * A51 + A02 * A11 * A30 * A44 * A55 - A02 * A11 * A30 * A45 * A54 - A02 * A11 * A34 * A40 * A55 + A02 * A11 * A34 * A45 * A50 + A02 * A11 * A35 * A40 * A54 - A02 * A11 * A35 * A44 * A50 - A02 * A14 * A30 * A41 * A55 + A02 * A14 * A30 * A45 * A51 + A02 * A14 * A31 * A40 * A55 - A02 * A14 * A31 * A45 * A50 - A02 * A14 * A35 * A40 * A51 + A02 * A14 * A35 * A41 * A50 + A02 * A15 * A30 * A41 * A54 - A02 * A15 * A30 * A44 * A51 - A02 * A15 * A31 * A40 * A54 + A02 * A15 * A31 * A44 * A50 + A02 * A15 * A34 * A40 * A51 - A02 * A15 * A34 * A41 * A50 + A04 * A10 * A31 * A42 * A55 - A04 * A10 * A31 * A45 * A52 - A04 * A10 * A32 * A41 * A55 + A04 * A10 * A32 * A45 * A51 + A04 * A10 * A35 * A41 * A52 - A04 * A10 * A35 * A42 * A51 - A04 * A11 * A30 * A42 * A55 + A04 * A11 * A30 * A45 * A52 + A04 * A11 * A32 * A40 * A55 - A04 * A11 * A32 * A45 * A50 - A04 * A11 * A35 * A40 * A52 + A04 * A11 * A35 * A42 * A50 + A04 * A12 * A30 * A41 * A55 - A04 * A12 * A30 * A45 * A51 - A04 * A12 * A31 * A40 * A55 + A04 * A12 * A31 * A45 * A50 + A04 * A12 * A35 * A40 * A51 - A04 * A12 * A35 * A41 * A50 - A04 * A15 * A30 * A41 * A52 + A04 * A15 * A30 * A42 * A51 + A04 * A15 * A31 * A40 * A52 - A04 * A15 * A31 * A42 * A50 - A04 * A15 * A32 * A40 * A51 + A04 * A15 * A32 * A41 * A50 - A05 * A10 * A31 * A42 * A54 + A05 * A10 * A31 * A44 * A52 + A05 * A10 * A32 * A41 * A54 - A05 * A10 * A32 * A44 * A51 - A05 * A10 * A34 * A41 * A52 + A05 * A10 * A34 * A42 * A51 + A05 * A11 * A30 * A42 * A54 - A05 * A11 * A30 * A44 * A52 - A05 * A11 * A32 * A40 * A54 + A05 * A11 * A32 * A44 * A50 + A05 * A11 * A34 * A40 * A52 - A05 * A11 * A34 * A42 * A50 - A05 * A12 * A30 * A41 * A54 + A05 * A12 * A30 * A44 * A51 + A05 * A12 * A31 * A40 * A54 - A05 * A12 * A31 * A44 * A50 - A05 * A12 * A34 * A40 * A51 + A05 * A12 * A34 * A41 * A50 + A05 * A14 * A30 * A41 * A52 - A05 * A14 * A30 * A42 * A51 - A05 * A14 * A31 * A40 * A52 + A05 * A14 * A31 * A42 * A50 + A05 * A14 * A32 * A40 * A51 - A05 * A14 * A32 * A41 * A50) / detA;
    d[3][3] = (A00 * A11 * A22 * A44 * A55 - A00 * A11 * A22 * A45 * A54 - A00 * A11 * A24 * A42 * A55 + A00 * A11 * A24 * A45 * A52 + A00 * A11 * A25 * A42 * A54 - A00 * A11 * A25 * A44 * A52 - A00 * A12 * A21 * A44 * A55 + A00 * A12 * A21 * A45 * A54 + A00 * A12 * A24 * A41 * A55 - A00 * A12 * A24 * A45 * A51 - A00 * A12 * A25 * A41 * A54 + A00 * A12 * A25 * A44 * A51 + A00 * A14 * A21 * A42 * A55 - A00 * A14 * A21 * A45 * A52 - A00 * A14 * A22 * A41 * A55 + A00 * A14 * A22 * A45 * A51 + A00 * A14 * A25 * A41 * A52 - A00 * A14 * A25 * A42 * A51 - A00 * A15 * A21 * A42 * A54 + A00 * A15 * A21 * A44 * A52 + A00 * A15 * A22 * A41 * A54 - A00 * A15 * A22 * A44 * A51 - A00 * A15 * A24 * A41 * A52 + A00 * A15 * A24 * A42 * A51 - A01 * A10 * A22 * A44 * A55 + A01 * A10 * A22 * A45 * A54 + A01 * A10 * A24 * A42 * A55 - A01 * A10 * A24 * A45 * A52 - A01 * A10 * A25 * A42 * A54 + A01 * A10 * A25 * A44 * A52 + A01 * A12 * A20 * A44 * A55 - A01 * A12 * A20 * A45 * A54 - A01 * A12 * A24 * A40 * A55 + A01 * A12 * A24 * A45 * A50 + A01 * A12 * A25 * A40 * A54 - A01 * A12 * A25 * A44 * A50 - A01 * A14 * A20 * A42 * A55 + A01 * A14 * A20 * A45 * A52 + A01 * A14 * A22 * A40 * A55 - A01 * A14 * A22 * A45 * A50 - A01 * A14 * A25 * A40 * A52 + A01 * A14 * A25 * A42 * A50 + A01 * A15 * A20 * A42 * A54 - A01 * A15 * A20 * A44 * A52 - A01 * A15 * A22 * A40 * A54 + A01 * A15 * A22 * A44 * A50 + A01 * A15 * A24 * A40 * A52 - A01 * A15 * A24 * A42 * A50 + A02 * A10 * A21 * A44 * A55 - A02 * A10 * A21 * A45 * A54 - A02 * A10 * A24 * A41 * A55 + A02 * A10 * A24 * A45 * A51 + A02 * A10 * A25 * A41 * A54 - A02 * A10 * A25 * A44 * A51 - A02 * A11 * A20 * A44 * A55 + A02 * A11 * A20 * A45 * A54 + A02 * A11 * A24 * A40 * A55 - A02 * A11 * A24 * A45 * A50 - A02 * A11 * A25 * A40 * A54 + A02 * A11 * A25 * A44 * A50 + A02 * A14 * A20 * A41 * A55 - A02 * A14 * A20 * A45 * A51 - A02 * A14 * A21 * A40 * A55 + A02 * A14 * A21 * A45 * A50 + A02 * A14 * A25 * A40 * A51 - A02 * A14 * A25 * A41 * A50 - A02 * A15 * A20 * A41 * A54 + A02 * A15 * A20 * A44 * A51 + A02 * A15 * A21 * A40 * A54 - A02 * A15 * A21 * A44 * A50 - A02 * A15 * A24 * A40 * A51 + A02 * A15 * A24 * A41 * A50 - A04 * A10 * A21 * A42 * A55 + A04 * A10 * A21 * A45 * A52 + A04 * A10 * A22 * A41 * A55 - A04 * A10 * A22 * A45 * A51 - A04 * A10 * A25 * A41 * A52 + A04 * A10 * A25 * A42 * A51 + A04 * A11 * A20 * A42 * A55 - A04 * A11 * A20 * A45 * A52 - A04 * A11 * A22 * A40 * A55 + A04 * A11 * A22 * A45 * A50 + A04 * A11 * A25 * A40 * A52 - A04 * A11 * A25 * A42 * A50 - A04 * A12 * A20 * A41 * A55 + A04 * A12 * A20 * A45 * A51 + A04 * A12 * A21 * A40 * A55 - A04 * A12 * A21 * A45 * A50 - A04 * A12 * A25 * A40 * A51 + A04 * A12 * A25 * A41 * A50 + A04 * A15 * A20 * A41 * A52 - A04 * A15 * A20 * A42 * A51 - A04 * A15 * A21 * A40 * A52 + A04 * A15 * A21 * A42 * A50 + A04 * A15 * A22 * A40 * A51 - A04 * A15 * A22 * A41 * A50 + A05 * A10 * A21 * A42 * A54 - A05 * A10 * A21 * A44 * A52 - A05 * A10 * A22 * A41 * A54 + A05 * A10 * A22 * A44 * A51 + A05 * A10 * A24 * A41 * A52 - A05 * A10 * A24 * A42 * A51 - A05 * A11 * A20 * A42 * A54 + A05 * A11 * A20 * A44 * A52 + A05 * A11 * A22 * A40 * A54 - A05 * A11 * A22 * A44 * A50 - A05 * A11 * A24 * A40 * A52 + A05 * A11 * A24 * A42 * A50 + A05 * A12 * A20 * A41 * A54 - A05 * A12 * A20 * A44 * A51 - A05 * A12 * A21 * A40 * A54 + A05 * A12 * A21 * A44 * A50 + A05 * A12 * A24 * A40 * A51 - A05 * A12 * A24 * A41 * A50 - A05 * A14 * A20 * A41 * A52 + A05 * A14 * A20 * A42 * A51 + A05 * A14 * A21 * A40 * A52 - A05 * A14 * A21 * A42 * A50 - A05 * A14 * A22 * A40 * A51 + A05 * A14 * A22 * A41 * A50) / detA;
    d[3][4] = (A00 * A11 * A22 * A35 * A54 - A00 * A11 * A22 * A34 * A55 + A00 * A11 * A24 * A32 * A55 - A00 * A11 * A24 * A35 * A52 - A00 * A11 * A25 * A32 * A54 + A00 * A11 * A25 * A34 * A52 + A00 * A12 * A21 * A34 * A55 - A00 * A12 * A21 * A35 * A54 - A00 * A12 * A24 * A31 * A55 + A00 * A12 * A24 * A35 * A51 + A00 * A12 * A25 * A31 * A54 - A00 * A12 * A25 * A34 * A51 - A00 * A14 * A21 * A32 * A55 + A00 * A14 * A21 * A35 * A52 + A00 * A14 * A22 * A31 * A55 - A00 * A14 * A22 * A35 * A51 - A00 * A14 * A25 * A31 * A52 + A00 * A14 * A25 * A32 * A51 + A00 * A15 * A21 * A32 * A54 - A00 * A15 * A21 * A34 * A52 - A00 * A15 * A22 * A31 * A54 + A00 * A15 * A22 * A34 * A51 + A00 * A15 * A24 * A31 * A52 - A00 * A15 * A24 * A32 * A51 + A01 * A10 * A22 * A34 * A55 - A01 * A10 * A22 * A35 * A54 - A01 * A10 * A24 * A32 * A55 + A01 * A10 * A24 * A35 * A52 + A01 * A10 * A25 * A32 * A54 - A01 * A10 * A25 * A34 * A52 - A01 * A12 * A20 * A34 * A55 + A01 * A12 * A20 * A35 * A54 + A01 * A12 * A24 * A30 * A55 - A01 * A12 * A24 * A35 * A50 - A01 * A12 * A25 * A30 * A54 + A01 * A12 * A25 * A34 * A50 + A01 * A14 * A20 * A32 * A55 - A01 * A14 * A20 * A35 * A52 - A01 * A14 * A22 * A30 * A55 + A01 * A14 * A22 * A35 * A50 + A01 * A14 * A25 * A30 * A52 - A01 * A14 * A25 * A32 * A50 - A01 * A15 * A20 * A32 * A54 + A01 * A15 * A20 * A34 * A52 + A01 * A15 * A22 * A30 * A54 - A01 * A15 * A22 * A34 * A50 - A01 * A15 * A24 * A30 * A52 + A01 * A15 * A24 * A32 * A50 - A02 * A10 * A21 * A34 * A55 + A02 * A10 * A21 * A35 * A54 + A02 * A10 * A24 * A31 * A55 - A02 * A10 * A24 * A35 * A51 - A02 * A10 * A25 * A31 * A54 + A02 * A10 * A25 * A34 * A51 + A02 * A11 * A20 * A34 * A55 - A02 * A11 * A20 * A35 * A54 - A02 * A11 * A24 * A30 * A55 + A02 * A11 * A24 * A35 * A50 + A02 * A11 * A25 * A30 * A54 - A02 * A11 * A25 * A34 * A50 - A02 * A14 * A20 * A31 * A55 + A02 * A14 * A20 * A35 * A51 + A02 * A14 * A21 * A30 * A55 - A02 * A14 * A21 * A35 * A50 - A02 * A14 * A25 * A30 * A51 + A02 * A14 * A25 * A31 * A50 + A02 * A15 * A20 * A31 * A54 - A02 * A15 * A20 * A34 * A51 - A02 * A15 * A21 * A30 * A54 + A02 * A15 * A21 * A34 * A50 + A02 * A15 * A24 * A30 * A51 - A02 * A15 * A24 * A31 * A50 + A04 * A10 * A21 * A32 * A55 - A04 * A10 * A21 * A35 * A52 - A04 * A10 * A22 * A31 * A55 + A04 * A10 * A22 * A35 * A51 + A04 * A10 * A25 * A31 * A52 - A04 * A10 * A25 * A32 * A51 - A04 * A11 * A20 * A32 * A55 + A04 * A11 * A20 * A35 * A52 + A04 * A11 * A22 * A30 * A55 - A04 * A11 * A22 * A35 * A50 - A04 * A11 * A25 * A30 * A52 + A04 * A11 * A25 * A32 * A50 + A04 * A12 * A20 * A31 * A55 - A04 * A12 * A20 * A35 * A51 - A04 * A12 * A21 * A30 * A55 + A04 * A12 * A21 * A35 * A50 + A04 * A12 * A25 * A30 * A51 - A04 * A12 * A25 * A31 * A50 - A04 * A15 * A20 * A31 * A52 + A04 * A15 * A20 * A32 * A51 + A04 * A15 * A21 * A30 * A52 - A04 * A15 * A21 * A32 * A50 - A04 * A15 * A22 * A30 * A51 + A04 * A15 * A22 * A31 * A50 - A05 * A10 * A21 * A32 * A54 + A05 * A10 * A21 * A34 * A52 + A05 * A10 * A22 * A31 * A54 - A05 * A10 * A22 * A34 * A51 - A05 * A10 * A24 * A31 * A52 + A05 * A10 * A24 * A32 * A51 + A05 * A11 * A20 * A32 * A54 - A05 * A11 * A20 * A34 * A52 - A05 * A11 * A22 * A30 * A54 + A05 * A11 * A22 * A34 * A50 + A05 * A11 * A24 * A30 * A52 - A05 * A11 * A24 * A32 * A50 - A05 * A12 * A20 * A31 * A54 + A05 * A12 * A20 * A34 * A51 + A05 * A12 * A21 * A30 * A54 - A05 * A12 * A21 * A34 * A50 - A05 * A12 * A24 * A30 * A51 + A05 * A12 * A24 * A31 * A50 + A05 * A14 * A20 * A31 * A52 - A05 * A14 * A20 * A32 * A51 - A05 * A14 * A21 * A30 * A52 + A05 * A14 * A21 * A32 * A50 + A05 * A14 * A22 * A30 * A51 - A05 * A14 * A22 * A31 * A50) / detA;
    d[3][5] = (A00 * A11 * A22 * A34 * A45 - A00 * A11 * A22 * A35 * A44 - A00 * A11 * A24 * A32 * A45 + A00 * A11 * A24 * A35 * A42 + A00 * A11 * A25 * A32 * A44 - A00 * A11 * A25 * A34 * A42 - A00 * A12 * A21 * A34 * A45 + A00 * A12 * A21 * A35 * A44 + A00 * A12 * A24 * A31 * A45 - A00 * A12 * A24 * A35 * A41 - A00 * A12 * A25 * A31 * A44 + A00 * A12 * A25 * A34 * A41 + A00 * A14 * A21 * A32 * A45 - A00 * A14 * A21 * A35 * A42 - A00 * A14 * A22 * A31 * A45 + A00 * A14 * A22 * A35 * A41 + A00 * A14 * A25 * A31 * A42 - A00 * A14 * A25 * A32 * A41 - A00 * A15 * A21 * A32 * A44 + A00 * A15 * A21 * A34 * A42 + A00 * A15 * A22 * A31 * A44 - A00 * A15 * A22 * A34 * A41 - A00 * A15 * A24 * A31 * A42 + A00 * A15 * A24 * A32 * A41 - A01 * A10 * A22 * A34 * A45 + A01 * A10 * A22 * A35 * A44 + A01 * A10 * A24 * A32 * A45 - A01 * A10 * A24 * A35 * A42 - A01 * A10 * A25 * A32 * A44 + A01 * A10 * A25 * A34 * A42 + A01 * A12 * A20 * A34 * A45 - A01 * A12 * A20 * A35 * A44 - A01 * A12 * A24 * A30 * A45 + A01 * A12 * A24 * A35 * A40 + A01 * A12 * A25 * A30 * A44 - A01 * A12 * A25 * A34 * A40 - A01 * A14 * A20 * A32 * A45 + A01 * A14 * A20 * A35 * A42 + A01 * A14 * A22 * A30 * A45 - A01 * A14 * A22 * A35 * A40 - A01 * A14 * A25 * A30 * A42 + A01 * A14 * A25 * A32 * A40 + A01 * A15 * A20 * A32 * A44 - A01 * A15 * A20 * A34 * A42 - A01 * A15 * A22 * A30 * A44 + A01 * A15 * A22 * A34 * A40 + A01 * A15 * A24 * A30 * A42 - A01 * A15 * A24 * A32 * A40 + A02 * A10 * A21 * A34 * A45 - A02 * A10 * A21 * A35 * A44 - A02 * A10 * A24 * A31 * A45 + A02 * A10 * A24 * A35 * A41 + A02 * A10 * A25 * A31 * A44 - A02 * A10 * A25 * A34 * A41 - A02 * A11 * A20 * A34 * A45 + A02 * A11 * A20 * A35 * A44 + A02 * A11 * A24 * A30 * A45 - A02 * A11 * A24 * A35 * A40 - A02 * A11 * A25 * A30 * A44 + A02 * A11 * A25 * A34 * A40 + A02 * A14 * A20 * A31 * A45 - A02 * A14 * A20 * A35 * A41 - A02 * A14 * A21 * A30 * A45 + A02 * A14 * A21 * A35 * A40 + A02 * A14 * A25 * A30 * A41 - A02 * A14 * A25 * A31 * A40 - A02 * A15 * A20 * A31 * A44 + A02 * A15 * A20 * A34 * A41 + A02 * A15 * A21 * A30 * A44 - A02 * A15 * A21 * A34 * A40 - A02 * A15 * A24 * A30 * A41 + A02 * A15 * A24 * A31 * A40 - A04 * A10 * A21 * A32 * A45 + A04 * A10 * A21 * A35 * A42 + A04 * A10 * A22 * A31 * A45 - A04 * A10 * A22 * A35 * A41 - A04 * A10 * A25 * A31 * A42 + A04 * A10 * A25 * A32 * A41 + A04 * A11 * A20 * A32 * A45 - A04 * A11 * A20 * A35 * A42 - A04 * A11 * A22 * A30 * A45 + A04 * A11 * A22 * A35 * A40 + A04 * A11 * A25 * A30 * A42 - A04 * A11 * A25 * A32 * A40 - A04 * A12 * A20 * A31 * A45 + A04 * A12 * A20 * A35 * A41 + A04 * A12 * A21 * A30 * A45 - A04 * A12 * A21 * A35 * A40 - A04 * A12 * A25 * A30 * A41 + A04 * A12 * A25 * A31 * A40 + A04 * A15 * A20 * A31 * A42 - A04 * A15 * A20 * A32 * A41 - A04 * A15 * A21 * A30 * A42 + A04 * A15 * A21 * A32 * A40 + A04 * A15 * A22 * A30 * A41 - A04 * A15 * A22 * A31 * A40 + A05 * A10 * A21 * A32 * A44 - A05 * A10 * A21 * A34 * A42 - A05 * A10 * A22 * A31 * A44 + A05 * A10 * A22 * A34 * A41 + A05 * A10 * A24 * A31 * A42 - A05 * A10 * A24 * A32 * A41 - A05 * A11 * A20 * A32 * A44 + A05 * A11 * A20 * A34 * A42 + A05 * A11 * A22 * A30 * A44 - A05 * A11 * A22 * A34 * A40 - A05 * A11 * A24 * A30 * A42 + A05 * A11 * A24 * A32 * A40 + A05 * A12 * A20 * A31 * A44 - A05 * A12 * A20 * A34 * A41 - A05 * A12 * A21 * A30 * A44 + A05 * A12 * A21 * A34 * A40 + A05 * A12 * A24 * A30 * A41 - A05 * A12 * A24 * A31 * A40 - A05 * A14 * A20 * A31 * A42 + A05 * A14 * A20 * A32 * A41 + A05 * A14 * A21 * A30 * A42 - A05 * A14 * A21 * A32 * A40 - A05 * A14 * A22 * A30 * A41 + A05 * A14 * A22 * A31 * A40) / detA;


    d[4][0] = (A10 * A21 * A32 * A43 * A55 - A10 * A21 * A32 * A45 * A53 - A10 * A21 * A33 * A42 * A55 + A10 * A21 * A33 * A45 * A52 + A10 * A21 * A35 * A42 * A53 - A10 * A21 * A35 * A43 * A52 - A10 * A22 * A31 * A43 * A55 + A10 * A22 * A31 * A45 * A53 + A10 * A22 * A33 * A41 * A55 - A10 * A22 * A33 * A45 * A51 - A10 * A22 * A35 * A41 * A53 + A10 * A22 * A35 * A43 * A51 + A10 * A23 * A31 * A42 * A55 - A10 * A23 * A31 * A45 * A52 - A10 * A23 * A32 * A41 * A55 + A10 * A23 * A32 * A45 * A51 + A10 * A23 * A35 * A41 * A52 - A10 * A23 * A35 * A42 * A51 - A10 * A25 * A31 * A42 * A53 + A10 * A25 * A31 * A43 * A52 + A10 * A25 * A32 * A41 * A53 - A10 * A25 * A32 * A43 * A51 - A10 * A25 * A33 * A41 * A52 + A10 * A25 * A33 * A42 * A51 - A11 * A20 * A32 * A43 * A55 + A11 * A20 * A32 * A45 * A53 + A11 * A20 * A33 * A42 * A55 - A11 * A20 * A33 * A45 * A52 - A11 * A20 * A35 * A42 * A53 + A11 * A20 * A35 * A43 * A52 + A11 * A22 * A30 * A43 * A55 - A11 * A22 * A30 * A45 * A53 - A11 * A22 * A33 * A40 * A55 + A11 * A22 * A33 * A45 * A50 + A11 * A22 * A35 * A40 * A53 - A11 * A22 * A35 * A43 * A50 - A11 * A23 * A30 * A42 * A55 + A11 * A23 * A30 * A45 * A52 + A11 * A23 * A32 * A40 * A55 - A11 * A23 * A32 * A45 * A50 - A11 * A23 * A35 * A40 * A52 + A11 * A23 * A35 * A42 * A50 + A11 * A25 * A30 * A42 * A53 - A11 * A25 * A30 * A43 * A52 - A11 * A25 * A32 * A40 * A53 + A11 * A25 * A32 * A43 * A50 + A11 * A25 * A33 * A40 * A52 - A11 * A25 * A33 * A42 * A50 + A12 * A20 * A31 * A43 * A55 - A12 * A20 * A31 * A45 * A53 - A12 * A20 * A33 * A41 * A55 + A12 * A20 * A33 * A45 * A51 + A12 * A20 * A35 * A41 * A53 - A12 * A20 * A35 * A43 * A51 - A12 * A21 * A30 * A43 * A55 + A12 * A21 * A30 * A45 * A53 + A12 * A21 * A33 * A40 * A55 - A12 * A21 * A33 * A45 * A50 - A12 * A21 * A35 * A40 * A53 + A12 * A21 * A35 * A43 * A50 + A12 * A23 * A30 * A41 * A55 - A12 * A23 * A30 * A45 * A51 - A12 * A23 * A31 * A40 * A55 + A12 * A23 * A31 * A45 * A50 + A12 * A23 * A35 * A40 * A51 - A12 * A23 * A35 * A41 * A50 - A12 * A25 * A30 * A41 * A53 + A12 * A25 * A30 * A43 * A51 + A12 * A25 * A31 * A40 * A53 - A12 * A25 * A31 * A43 * A50 - A12 * A25 * A33 * A40 * A51 + A12 * A25 * A33 * A41 * A50 - A13 * A20 * A31 * A42 * A55 + A13 * A20 * A31 * A45 * A52 + A13 * A20 * A32 * A41 * A55 - A13 * A20 * A32 * A45 * A51 - A13 * A20 * A35 * A41 * A52 + A13 * A20 * A35 * A42 * A51 + A13 * A21 * A30 * A42 * A55 - A13 * A21 * A30 * A45 * A52 - A13 * A21 * A32 * A40 * A55 + A13 * A21 * A32 * A45 * A50 + A13 * A21 * A35 * A40 * A52 - A13 * A21 * A35 * A42 * A50 - A13 * A22 * A30 * A41 * A55 + A13 * A22 * A30 * A45 * A51 + A13 * A22 * A31 * A40 * A55 - A13 * A22 * A31 * A45 * A50 - A13 * A22 * A35 * A40 * A51 + A13 * A22 * A35 * A41 * A50 + A13 * A25 * A30 * A41 * A52 - A13 * A25 * A30 * A42 * A51 - A13 * A25 * A31 * A40 * A52 + A13 * A25 * A31 * A42 * A50 + A13 * A25 * A32 * A40 * A51 - A13 * A25 * A32 * A41 * A50 + A15 * A20 * A31 * A42 * A53 - A15 * A20 * A31 * A43 * A52 - A15 * A20 * A32 * A41 * A53 + A15 * A20 * A32 * A43 * A51 + A15 * A20 * A33 * A41 * A52 - A15 * A20 * A33 * A42 * A51 - A15 * A21 * A30 * A42 * A53 + A15 * A21 * A30 * A43 * A52 + A15 * A21 * A32 * A40 * A53 - A15 * A21 * A32 * A43 * A50 - A15 * A21 * A33 * A40 * A52 + A15 * A21 * A33 * A42 * A50 + A15 * A22 * A30 * A41 * A53 - A15 * A22 * A30 * A43 * A51 - A15 * A22 * A31 * A40 * A53 + A15 * A22 * A31 * A43 * A50 + A15 * A22 * A33 * A40 * A51 - A15 * A22 * A33 * A41 * A50 - A15 * A23 * A30 * A41 * A52 + A15 * A23 * A30 * A42 * A51 + A15 * A23 * A31 * A40 * A52 - A15 * A23 * A31 * A42 * A50 - A15 * A23 * A32 * A40 * A51 + A15 * A23 * A32 * A41 * A50) / detA;
    d[4][1] = (A00 * A21 * A32 * A45 * A53 - A00 * A21 * A32 * A43 * A55 + A00 * A21 * A33 * A42 * A55 - A00 * A21 * A33 * A45 * A52 - A00 * A21 * A35 * A42 * A53 + A00 * A21 * A35 * A43 * A52 + A00 * A22 * A31 * A43 * A55 - A00 * A22 * A31 * A45 * A53 - A00 * A22 * A33 * A41 * A55 + A00 * A22 * A33 * A45 * A51 + A00 * A22 * A35 * A41 * A53 - A00 * A22 * A35 * A43 * A51 - A00 * A23 * A31 * A42 * A55 + A00 * A23 * A31 * A45 * A52 + A00 * A23 * A32 * A41 * A55 - A00 * A23 * A32 * A45 * A51 - A00 * A23 * A35 * A41 * A52 + A00 * A23 * A35 * A42 * A51 + A00 * A25 * A31 * A42 * A53 - A00 * A25 * A31 * A43 * A52 - A00 * A25 * A32 * A41 * A53 + A00 * A25 * A32 * A43 * A51 + A00 * A25 * A33 * A41 * A52 - A00 * A25 * A33 * A42 * A51 + A01 * A20 * A32 * A43 * A55 - A01 * A20 * A32 * A45 * A53 - A01 * A20 * A33 * A42 * A55 + A01 * A20 * A33 * A45 * A52 + A01 * A20 * A35 * A42 * A53 - A01 * A20 * A35 * A43 * A52 - A01 * A22 * A30 * A43 * A55 + A01 * A22 * A30 * A45 * A53 + A01 * A22 * A33 * A40 * A55 - A01 * A22 * A33 * A45 * A50 - A01 * A22 * A35 * A40 * A53 + A01 * A22 * A35 * A43 * A50 + A01 * A23 * A30 * A42 * A55 - A01 * A23 * A30 * A45 * A52 - A01 * A23 * A32 * A40 * A55 + A01 * A23 * A32 * A45 * A50 + A01 * A23 * A35 * A40 * A52 - A01 * A23 * A35 * A42 * A50 - A01 * A25 * A30 * A42 * A53 + A01 * A25 * A30 * A43 * A52 + A01 * A25 * A32 * A40 * A53 - A01 * A25 * A32 * A43 * A50 - A01 * A25 * A33 * A40 * A52 + A01 * A25 * A33 * A42 * A50 - A02 * A20 * A31 * A43 * A55 + A02 * A20 * A31 * A45 * A53 + A02 * A20 * A33 * A41 * A55 - A02 * A20 * A33 * A45 * A51 - A02 * A20 * A35 * A41 * A53 + A02 * A20 * A35 * A43 * A51 + A02 * A21 * A30 * A43 * A55 - A02 * A21 * A30 * A45 * A53 - A02 * A21 * A33 * A40 * A55 + A02 * A21 * A33 * A45 * A50 + A02 * A21 * A35 * A40 * A53 - A02 * A21 * A35 * A43 * A50 - A02 * A23 * A30 * A41 * A55 + A02 * A23 * A30 * A45 * A51 + A02 * A23 * A31 * A40 * A55 - A02 * A23 * A31 * A45 * A50 - A02 * A23 * A35 * A40 * A51 + A02 * A23 * A35 * A41 * A50 + A02 * A25 * A30 * A41 * A53 - A02 * A25 * A30 * A43 * A51 - A02 * A25 * A31 * A40 * A53 + A02 * A25 * A31 * A43 * A50 + A02 * A25 * A33 * A40 * A51 - A02 * A25 * A33 * A41 * A50 + A03 * A20 * A31 * A42 * A55 - A03 * A20 * A31 * A45 * A52 - A03 * A20 * A32 * A41 * A55 + A03 * A20 * A32 * A45 * A51 + A03 * A20 * A35 * A41 * A52 - A03 * A20 * A35 * A42 * A51 - A03 * A21 * A30 * A42 * A55 + A03 * A21 * A30 * A45 * A52 + A03 * A21 * A32 * A40 * A55 - A03 * A21 * A32 * A45 * A50 - A03 * A21 * A35 * A40 * A52 + A03 * A21 * A35 * A42 * A50 + A03 * A22 * A30 * A41 * A55 - A03 * A22 * A30 * A45 * A51 - A03 * A22 * A31 * A40 * A55 + A03 * A22 * A31 * A45 * A50 + A03 * A22 * A35 * A40 * A51 - A03 * A22 * A35 * A41 * A50 - A03 * A25 * A30 * A41 * A52 + A03 * A25 * A30 * A42 * A51 + A03 * A25 * A31 * A40 * A52 - A03 * A25 * A31 * A42 * A50 - A03 * A25 * A32 * A40 * A51 + A03 * A25 * A32 * A41 * A50 - A05 * A20 * A31 * A42 * A53 + A05 * A20 * A31 * A43 * A52 + A05 * A20 * A32 * A41 * A53 - A05 * A20 * A32 * A43 * A51 - A05 * A20 * A33 * A41 * A52 + A05 * A20 * A33 * A42 * A51 + A05 * A21 * A30 * A42 * A53 - A05 * A21 * A30 * A43 * A52 - A05 * A21 * A32 * A40 * A53 + A05 * A21 * A32 * A43 * A50 + A05 * A21 * A33 * A40 * A52 - A05 * A21 * A33 * A42 * A50 - A05 * A22 * A30 * A41 * A53 + A05 * A22 * A30 * A43 * A51 + A05 * A22 * A31 * A40 * A53 - A05 * A22 * A31 * A43 * A50 - A05 * A22 * A33 * A40 * A51 + A05 * A22 * A33 * A41 * A50 + A05 * A23 * A30 * A41 * A52 - A05 * A23 * A30 * A42 * A51 - A05 * A23 * A31 * A40 * A52 + A05 * A23 * A31 * A42 * A50 + A05 * A23 * A32 * A40 * A51 - A05 * A23 * A32 * A41 * A50) / detA;
    d[4][2] = (A00 * A11 * A32 * A43 * A55 - A00 * A11 * A32 * A45 * A53 - A00 * A11 * A33 * A42 * A55 + A00 * A11 * A33 * A45 * A52 + A00 * A11 * A35 * A42 * A53 - A00 * A11 * A35 * A43 * A52 - A00 * A12 * A31 * A43 * A55 + A00 * A12 * A31 * A45 * A53 + A00 * A12 * A33 * A41 * A55 - A00 * A12 * A33 * A45 * A51 - A00 * A12 * A35 * A41 * A53 + A00 * A12 * A35 * A43 * A51 + A00 * A13 * A31 * A42 * A55 - A00 * A13 * A31 * A45 * A52 - A00 * A13 * A32 * A41 * A55 + A00 * A13 * A32 * A45 * A51 + A00 * A13 * A35 * A41 * A52 - A00 * A13 * A35 * A42 * A51 - A00 * A15 * A31 * A42 * A53 + A00 * A15 * A31 * A43 * A52 + A00 * A15 * A32 * A41 * A53 - A00 * A15 * A32 * A43 * A51 - A00 * A15 * A33 * A41 * A52 + A00 * A15 * A33 * A42 * A51 - A01 * A10 * A32 * A43 * A55 + A01 * A10 * A32 * A45 * A53 + A01 * A10 * A33 * A42 * A55 - A01 * A10 * A33 * A45 * A52 - A01 * A10 * A35 * A42 * A53 + A01 * A10 * A35 * A43 * A52 + A01 * A12 * A30 * A43 * A55 - A01 * A12 * A30 * A45 * A53 - A01 * A12 * A33 * A40 * A55 + A01 * A12 * A33 * A45 * A50 + A01 * A12 * A35 * A40 * A53 - A01 * A12 * A35 * A43 * A50 - A01 * A13 * A30 * A42 * A55 + A01 * A13 * A30 * A45 * A52 + A01 * A13 * A32 * A40 * A55 - A01 * A13 * A32 * A45 * A50 - A01 * A13 * A35 * A40 * A52 + A01 * A13 * A35 * A42 * A50 + A01 * A15 * A30 * A42 * A53 - A01 * A15 * A30 * A43 * A52 - A01 * A15 * A32 * A40 * A53 + A01 * A15 * A32 * A43 * A50 + A01 * A15 * A33 * A40 * A52 - A01 * A15 * A33 * A42 * A50 + A02 * A10 * A31 * A43 * A55 - A02 * A10 * A31 * A45 * A53 - A02 * A10 * A33 * A41 * A55 + A02 * A10 * A33 * A45 * A51 + A02 * A10 * A35 * A41 * A53 - A02 * A10 * A35 * A43 * A51 - A02 * A11 * A30 * A43 * A55 + A02 * A11 * A30 * A45 * A53 + A02 * A11 * A33 * A40 * A55 - A02 * A11 * A33 * A45 * A50 - A02 * A11 * A35 * A40 * A53 + A02 * A11 * A35 * A43 * A50 + A02 * A13 * A30 * A41 * A55 - A02 * A13 * A30 * A45 * A51 - A02 * A13 * A31 * A40 * A55 + A02 * A13 * A31 * A45 * A50 + A02 * A13 * A35 * A40 * A51 - A02 * A13 * A35 * A41 * A50 - A02 * A15 * A30 * A41 * A53 + A02 * A15 * A30 * A43 * A51 + A02 * A15 * A31 * A40 * A53 - A02 * A15 * A31 * A43 * A50 - A02 * A15 * A33 * A40 * A51 + A02 * A15 * A33 * A41 * A50 - A03 * A10 * A31 * A42 * A55 + A03 * A10 * A31 * A45 * A52 + A03 * A10 * A32 * A41 * A55 - A03 * A10 * A32 * A45 * A51 - A03 * A10 * A35 * A41 * A52 + A03 * A10 * A35 * A42 * A51 + A03 * A11 * A30 * A42 * A55 - A03 * A11 * A30 * A45 * A52 - A03 * A11 * A32 * A40 * A55 + A03 * A11 * A32 * A45 * A50 + A03 * A11 * A35 * A40 * A52 - A03 * A11 * A35 * A42 * A50 - A03 * A12 * A30 * A41 * A55 + A03 * A12 * A30 * A45 * A51 + A03 * A12 * A31 * A40 * A55 - A03 * A12 * A31 * A45 * A50 - A03 * A12 * A35 * A40 * A51 + A03 * A12 * A35 * A41 * A50 + A03 * A15 * A30 * A41 * A52 - A03 * A15 * A30 * A42 * A51 - A03 * A15 * A31 * A40 * A52 + A03 * A15 * A31 * A42 * A50 + A03 * A15 * A32 * A40 * A51 - A03 * A15 * A32 * A41 * A50 + A05 * A10 * A31 * A42 * A53 - A05 * A10 * A31 * A43 * A52 - A05 * A10 * A32 * A41 * A53 + A05 * A10 * A32 * A43 * A51 + A05 * A10 * A33 * A41 * A52 - A05 * A10 * A33 * A42 * A51 - A05 * A11 * A30 * A42 * A53 + A05 * A11 * A30 * A43 * A52 + A05 * A11 * A32 * A40 * A53 - A05 * A11 * A32 * A43 * A50 - A05 * A11 * A33 * A40 * A52 + A05 * A11 * A33 * A42 * A50 + A05 * A12 * A30 * A41 * A53 - A05 * A12 * A30 * A43 * A51 - A05 * A12 * A31 * A40 * A53 + A05 * A12 * A31 * A43 * A50 + A05 * A12 * A33 * A40 * A51 - A05 * A12 * A33 * A41 * A50 - A05 * A13 * A30 * A41 * A52 + A05 * A13 * A30 * A42 * A51 + A05 * A13 * A31 * A40 * A52 - A05 * A13 * A31 * A42 * A50 - A05 * A13 * A32 * A40 * A51 + A05 * A13 * A32 * A41 * A50) / detA;
    d[4][3] = (A00 * A11 * A22 * A45 * A53 - A00 * A11 * A22 * A43 * A55 + A00 * A11 * A23 * A42 * A55 - A00 * A11 * A23 * A45 * A52 - A00 * A11 * A25 * A42 * A53 + A00 * A11 * A25 * A43 * A52 + A00 * A12 * A21 * A43 * A55 - A00 * A12 * A21 * A45 * A53 - A00 * A12 * A23 * A41 * A55 + A00 * A12 * A23 * A45 * A51 + A00 * A12 * A25 * A41 * A53 - A00 * A12 * A25 * A43 * A51 - A00 * A13 * A21 * A42 * A55 + A00 * A13 * A21 * A45 * A52 + A00 * A13 * A22 * A41 * A55 - A00 * A13 * A22 * A45 * A51 - A00 * A13 * A25 * A41 * A52 + A00 * A13 * A25 * A42 * A51 + A00 * A15 * A21 * A42 * A53 - A00 * A15 * A21 * A43 * A52 - A00 * A15 * A22 * A41 * A53 + A00 * A15 * A22 * A43 * A51 + A00 * A15 * A23 * A41 * A52 - A00 * A15 * A23 * A42 * A51 + A01 * A10 * A22 * A43 * A55 - A01 * A10 * A22 * A45 * A53 - A01 * A10 * A23 * A42 * A55 + A01 * A10 * A23 * A45 * A52 + A01 * A10 * A25 * A42 * A53 - A01 * A10 * A25 * A43 * A52 - A01 * A12 * A20 * A43 * A55 + A01 * A12 * A20 * A45 * A53 + A01 * A12 * A23 * A40 * A55 - A01 * A12 * A23 * A45 * A50 - A01 * A12 * A25 * A40 * A53 + A01 * A12 * A25 * A43 * A50 + A01 * A13 * A20 * A42 * A55 - A01 * A13 * A20 * A45 * A52 - A01 * A13 * A22 * A40 * A55 + A01 * A13 * A22 * A45 * A50 + A01 * A13 * A25 * A40 * A52 - A01 * A13 * A25 * A42 * A50 - A01 * A15 * A20 * A42 * A53 + A01 * A15 * A20 * A43 * A52 + A01 * A15 * A22 * A40 * A53 - A01 * A15 * A22 * A43 * A50 - A01 * A15 * A23 * A40 * A52 + A01 * A15 * A23 * A42 * A50 - A02 * A10 * A21 * A43 * A55 + A02 * A10 * A21 * A45 * A53 + A02 * A10 * A23 * A41 * A55 - A02 * A10 * A23 * A45 * A51 - A02 * A10 * A25 * A41 * A53 + A02 * A10 * A25 * A43 * A51 + A02 * A11 * A20 * A43 * A55 - A02 * A11 * A20 * A45 * A53 - A02 * A11 * A23 * A40 * A55 + A02 * A11 * A23 * A45 * A50 + A02 * A11 * A25 * A40 * A53 - A02 * A11 * A25 * A43 * A50 - A02 * A13 * A20 * A41 * A55 + A02 * A13 * A20 * A45 * A51 + A02 * A13 * A21 * A40 * A55 - A02 * A13 * A21 * A45 * A50 - A02 * A13 * A25 * A40 * A51 + A02 * A13 * A25 * A41 * A50 + A02 * A15 * A20 * A41 * A53 - A02 * A15 * A20 * A43 * A51 - A02 * A15 * A21 * A40 * A53 + A02 * A15 * A21 * A43 * A50 + A02 * A15 * A23 * A40 * A51 - A02 * A15 * A23 * A41 * A50 + A03 * A10 * A21 * A42 * A55 - A03 * A10 * A21 * A45 * A52 - A03 * A10 * A22 * A41 * A55 + A03 * A10 * A22 * A45 * A51 + A03 * A10 * A25 * A41 * A52 - A03 * A10 * A25 * A42 * A51 - A03 * A11 * A20 * A42 * A55 + A03 * A11 * A20 * A45 * A52 + A03 * A11 * A22 * A40 * A55 - A03 * A11 * A22 * A45 * A50 - A03 * A11 * A25 * A40 * A52 + A03 * A11 * A25 * A42 * A50 + A03 * A12 * A20 * A41 * A55 - A03 * A12 * A20 * A45 * A51 - A03 * A12 * A21 * A40 * A55 + A03 * A12 * A21 * A45 * A50 + A03 * A12 * A25 * A40 * A51 - A03 * A12 * A25 * A41 * A50 - A03 * A15 * A20 * A41 * A52 + A03 * A15 * A20 * A42 * A51 + A03 * A15 * A21 * A40 * A52 - A03 * A15 * A21 * A42 * A50 - A03 * A15 * A22 * A40 * A51 + A03 * A15 * A22 * A41 * A50 - A05 * A10 * A21 * A42 * A53 + A05 * A10 * A21 * A43 * A52 + A05 * A10 * A22 * A41 * A53 - A05 * A10 * A22 * A43 * A51 - A05 * A10 * A23 * A41 * A52 + A05 * A10 * A23 * A42 * A51 + A05 * A11 * A20 * A42 * A53 - A05 * A11 * A20 * A43 * A52 - A05 * A11 * A22 * A40 * A53 + A05 * A11 * A22 * A43 * A50 + A05 * A11 * A23 * A40 * A52 - A05 * A11 * A23 * A42 * A50 - A05 * A12 * A20 * A41 * A53 + A05 * A12 * A20 * A43 * A51 + A05 * A12 * A21 * A40 * A53 - A05 * A12 * A21 * A43 * A50 - A05 * A12 * A23 * A40 * A51 + A05 * A12 * A23 * A41 * A50 + A05 * A13 * A20 * A41 * A52 - A05 * A13 * A20 * A42 * A51 - A05 * A13 * A21 * A40 * A52 + A05 * A13 * A21 * A42 * A50 + A05 * A13 * A22 * A40 * A51 - A05 * A13 * A22 * A41 * A50) / detA;
    d[4][4] = (A00 * A11 * A22 * A33 * A55 - A00 * A11 * A22 * A35 * A53 - A00 * A11 * A23 * A32 * A55 + A00 * A11 * A23 * A35 * A52 + A00 * A11 * A25 * A32 * A53 - A00 * A11 * A25 * A33 * A52 - A00 * A12 * A21 * A33 * A55 + A00 * A12 * A21 * A35 * A53 + A00 * A12 * A23 * A31 * A55 - A00 * A12 * A23 * A35 * A51 - A00 * A12 * A25 * A31 * A53 + A00 * A12 * A25 * A33 * A51 + A00 * A13 * A21 * A32 * A55 - A00 * A13 * A21 * A35 * A52 - A00 * A13 * A22 * A31 * A55 + A00 * A13 * A22 * A35 * A51 + A00 * A13 * A25 * A31 * A52 - A00 * A13 * A25 * A32 * A51 - A00 * A15 * A21 * A32 * A53 + A00 * A15 * A21 * A33 * A52 + A00 * A15 * A22 * A31 * A53 - A00 * A15 * A22 * A33 * A51 - A00 * A15 * A23 * A31 * A52 + A00 * A15 * A23 * A32 * A51 - A01 * A10 * A22 * A33 * A55 + A01 * A10 * A22 * A35 * A53 + A01 * A10 * A23 * A32 * A55 - A01 * A10 * A23 * A35 * A52 - A01 * A10 * A25 * A32 * A53 + A01 * A10 * A25 * A33 * A52 + A01 * A12 * A20 * A33 * A55 - A01 * A12 * A20 * A35 * A53 - A01 * A12 * A23 * A30 * A55 + A01 * A12 * A23 * A35 * A50 + A01 * A12 * A25 * A30 * A53 - A01 * A12 * A25 * A33 * A50 - A01 * A13 * A20 * A32 * A55 + A01 * A13 * A20 * A35 * A52 + A01 * A13 * A22 * A30 * A55 - A01 * A13 * A22 * A35 * A50 - A01 * A13 * A25 * A30 * A52 + A01 * A13 * A25 * A32 * A50 + A01 * A15 * A20 * A32 * A53 - A01 * A15 * A20 * A33 * A52 - A01 * A15 * A22 * A30 * A53 + A01 * A15 * A22 * A33 * A50 + A01 * A15 * A23 * A30 * A52 - A01 * A15 * A23 * A32 * A50 + A02 * A10 * A21 * A33 * A55 - A02 * A10 * A21 * A35 * A53 - A02 * A10 * A23 * A31 * A55 + A02 * A10 * A23 * A35 * A51 + A02 * A10 * A25 * A31 * A53 - A02 * A10 * A25 * A33 * A51 - A02 * A11 * A20 * A33 * A55 + A02 * A11 * A20 * A35 * A53 + A02 * A11 * A23 * A30 * A55 - A02 * A11 * A23 * A35 * A50 - A02 * A11 * A25 * A30 * A53 + A02 * A11 * A25 * A33 * A50 + A02 * A13 * A20 * A31 * A55 - A02 * A13 * A20 * A35 * A51 - A02 * A13 * A21 * A30 * A55 + A02 * A13 * A21 * A35 * A50 + A02 * A13 * A25 * A30 * A51 - A02 * A13 * A25 * A31 * A50 - A02 * A15 * A20 * A31 * A53 + A02 * A15 * A20 * A33 * A51 + A02 * A15 * A21 * A30 * A53 - A02 * A15 * A21 * A33 * A50 - A02 * A15 * A23 * A30 * A51 + A02 * A15 * A23 * A31 * A50 - A03 * A10 * A21 * A32 * A55 + A03 * A10 * A21 * A35 * A52 + A03 * A10 * A22 * A31 * A55 - A03 * A10 * A22 * A35 * A51 - A03 * A10 * A25 * A31 * A52 + A03 * A10 * A25 * A32 * A51 + A03 * A11 * A20 * A32 * A55 - A03 * A11 * A20 * A35 * A52 - A03 * A11 * A22 * A30 * A55 + A03 * A11 * A22 * A35 * A50 + A03 * A11 * A25 * A30 * A52 - A03 * A11 * A25 * A32 * A50 - A03 * A12 * A20 * A31 * A55 + A03 * A12 * A20 * A35 * A51 + A03 * A12 * A21 * A30 * A55 - A03 * A12 * A21 * A35 * A50 - A03 * A12 * A25 * A30 * A51 + A03 * A12 * A25 * A31 * A50 + A03 * A15 * A20 * A31 * A52 - A03 * A15 * A20 * A32 * A51 - A03 * A15 * A21 * A30 * A52 + A03 * A15 * A21 * A32 * A50 + A03 * A15 * A22 * A30 * A51 - A03 * A15 * A22 * A31 * A50 + A05 * A10 * A21 * A32 * A53 - A05 * A10 * A21 * A33 * A52 - A05 * A10 * A22 * A31 * A53 + A05 * A10 * A22 * A33 * A51 + A05 * A10 * A23 * A31 * A52 - A05 * A10 * A23 * A32 * A51 - A05 * A11 * A20 * A32 * A53 + A05 * A11 * A20 * A33 * A52 + A05 * A11 * A22 * A30 * A53 - A05 * A11 * A22 * A33 * A50 - A05 * A11 * A23 * A30 * A52 + A05 * A11 * A23 * A32 * A50 + A05 * A12 * A20 * A31 * A53 - A05 * A12 * A20 * A33 * A51 - A05 * A12 * A21 * A30 * A53 + A05 * A12 * A21 * A33 * A50 + A05 * A12 * A23 * A30 * A51 - A05 * A12 * A23 * A31 * A50 - A05 * A13 * A20 * A31 * A52 + A05 * A13 * A20 * A32 * A51 + A05 * A13 * A21 * A30 * A52 - A05 * A13 * A21 * A32 * A50 - A05 * A13 * A22 * A30 * A51 + A05 * A13 * A22 * A31 * A50) / detA;
    d[4][5] = (A00 * A11 * A22 * A35 * A43 - A00 * A11 * A22 * A33 * A45 + A00 * A11 * A23 * A32 * A45 - A00 * A11 * A23 * A35 * A42 - A00 * A11 * A25 * A32 * A43 + A00 * A11 * A25 * A33 * A42 + A00 * A12 * A21 * A33 * A45 - A00 * A12 * A21 * A35 * A43 - A00 * A12 * A23 * A31 * A45 + A00 * A12 * A23 * A35 * A41 + A00 * A12 * A25 * A31 * A43 - A00 * A12 * A25 * A33 * A41 - A00 * A13 * A21 * A32 * A45 + A00 * A13 * A21 * A35 * A42 + A00 * A13 * A22 * A31 * A45 - A00 * A13 * A22 * A35 * A41 - A00 * A13 * A25 * A31 * A42 + A00 * A13 * A25 * A32 * A41 + A00 * A15 * A21 * A32 * A43 - A00 * A15 * A21 * A33 * A42 - A00 * A15 * A22 * A31 * A43 + A00 * A15 * A22 * A33 * A41 + A00 * A15 * A23 * A31 * A42 - A00 * A15 * A23 * A32 * A41 + A01 * A10 * A22 * A33 * A45 - A01 * A10 * A22 * A35 * A43 - A01 * A10 * A23 * A32 * A45 + A01 * A10 * A23 * A35 * A42 + A01 * A10 * A25 * A32 * A43 - A01 * A10 * A25 * A33 * A42 - A01 * A12 * A20 * A33 * A45 + A01 * A12 * A20 * A35 * A43 + A01 * A12 * A23 * A30 * A45 - A01 * A12 * A23 * A35 * A40 - A01 * A12 * A25 * A30 * A43 + A01 * A12 * A25 * A33 * A40 + A01 * A13 * A20 * A32 * A45 - A01 * A13 * A20 * A35 * A42 - A01 * A13 * A22 * A30 * A45 + A01 * A13 * A22 * A35 * A40 + A01 * A13 * A25 * A30 * A42 - A01 * A13 * A25 * A32 * A40 - A01 * A15 * A20 * A32 * A43 + A01 * A15 * A20 * A33 * A42 + A01 * A15 * A22 * A30 * A43 - A01 * A15 * A22 * A33 * A40 - A01 * A15 * A23 * A30 * A42 + A01 * A15 * A23 * A32 * A40 - A02 * A10 * A21 * A33 * A45 + A02 * A10 * A21 * A35 * A43 + A02 * A10 * A23 * A31 * A45 - A02 * A10 * A23 * A35 * A41 - A02 * A10 * A25 * A31 * A43 + A02 * A10 * A25 * A33 * A41 + A02 * A11 * A20 * A33 * A45 - A02 * A11 * A20 * A35 * A43 - A02 * A11 * A23 * A30 * A45 + A02 * A11 * A23 * A35 * A40 + A02 * A11 * A25 * A30 * A43 - A02 * A11 * A25 * A33 * A40 - A02 * A13 * A20 * A31 * A45 + A02 * A13 * A20 * A35 * A41 + A02 * A13 * A21 * A30 * A45 - A02 * A13 * A21 * A35 * A40 - A02 * A13 * A25 * A30 * A41 + A02 * A13 * A25 * A31 * A40 + A02 * A15 * A20 * A31 * A43 - A02 * A15 * A20 * A33 * A41 - A02 * A15 * A21 * A30 * A43 + A02 * A15 * A21 * A33 * A40 + A02 * A15 * A23 * A30 * A41 - A02 * A15 * A23 * A31 * A40 + A03 * A10 * A21 * A32 * A45 - A03 * A10 * A21 * A35 * A42 - A03 * A10 * A22 * A31 * A45 + A03 * A10 * A22 * A35 * A41 + A03 * A10 * A25 * A31 * A42 - A03 * A10 * A25 * A32 * A41 - A03 * A11 * A20 * A32 * A45 + A03 * A11 * A20 * A35 * A42 + A03 * A11 * A22 * A30 * A45 - A03 * A11 * A22 * A35 * A40 - A03 * A11 * A25 * A30 * A42 + A03 * A11 * A25 * A32 * A40 + A03 * A12 * A20 * A31 * A45 - A03 * A12 * A20 * A35 * A41 - A03 * A12 * A21 * A30 * A45 + A03 * A12 * A21 * A35 * A40 + A03 * A12 * A25 * A30 * A41 - A03 * A12 * A25 * A31 * A40 - A03 * A15 * A20 * A31 * A42 + A03 * A15 * A20 * A32 * A41 + A03 * A15 * A21 * A30 * A42 - A03 * A15 * A21 * A32 * A40 - A03 * A15 * A22 * A30 * A41 + A03 * A15 * A22 * A31 * A40 - A05 * A10 * A21 * A32 * A43 + A05 * A10 * A21 * A33 * A42 + A05 * A10 * A22 * A31 * A43 - A05 * A10 * A22 * A33 * A41 - A05 * A10 * A23 * A31 * A42 + A05 * A10 * A23 * A32 * A41 + A05 * A11 * A20 * A32 * A43 - A05 * A11 * A20 * A33 * A42 - A05 * A11 * A22 * A30 * A43 + A05 * A11 * A22 * A33 * A40 + A05 * A11 * A23 * A30 * A42 - A05 * A11 * A23 * A32 * A40 - A05 * A12 * A20 * A31 * A43 + A05 * A12 * A20 * A33 * A41 + A05 * A12 * A21 * A30 * A43 - A05 * A12 * A21 * A33 * A40 - A05 * A12 * A23 * A30 * A41 + A05 * A12 * A23 * A31 * A40 + A05 * A13 * A20 * A31 * A42 - A05 * A13 * A20 * A32 * A41 - A05 * A13 * A21 * A30 * A42 + A05 * A13 * A21 * A32 * A40 + A05 * A13 * A22 * A30 * A41 - A05 * A13 * A22 * A31 * A40) / detA;


    d[5][0] = (A10 * A21 * A32 * A44 * A53 - A10 * A21 * A32 * A43 * A54 + A10 * A21 * A33 * A42 * A54 - A10 * A21 * A33 * A44 * A52 - A10 * A21 * A34 * A42 * A53 + A10 * A21 * A34 * A43 * A52 + A10 * A22 * A31 * A43 * A54 - A10 * A22 * A31 * A44 * A53 - A10 * A22 * A33 * A41 * A54 + A10 * A22 * A33 * A44 * A51 + A10 * A22 * A34 * A41 * A53 - A10 * A22 * A34 * A43 * A51 - A10 * A23 * A31 * A42 * A54 + A10 * A23 * A31 * A44 * A52 + A10 * A23 * A32 * A41 * A54 - A10 * A23 * A32 * A44 * A51 - A10 * A23 * A34 * A41 * A52 + A10 * A23 * A34 * A42 * A51 + A10 * A24 * A31 * A42 * A53 - A10 * A24 * A31 * A43 * A52 - A10 * A24 * A32 * A41 * A53 + A10 * A24 * A32 * A43 * A51 + A10 * A24 * A33 * A41 * A52 - A10 * A24 * A33 * A42 * A51 + A11 * A20 * A32 * A43 * A54 - A11 * A20 * A32 * A44 * A53 - A11 * A20 * A33 * A42 * A54 + A11 * A20 * A33 * A44 * A52 + A11 * A20 * A34 * A42 * A53 - A11 * A20 * A34 * A43 * A52 - A11 * A22 * A30 * A43 * A54 + A11 * A22 * A30 * A44 * A53 + A11 * A22 * A33 * A40 * A54 - A11 * A22 * A33 * A44 * A50 - A11 * A22 * A34 * A40 * A53 + A11 * A22 * A34 * A43 * A50 + A11 * A23 * A30 * A42 * A54 - A11 * A23 * A30 * A44 * A52 - A11 * A23 * A32 * A40 * A54 + A11 * A23 * A32 * A44 * A50 + A11 * A23 * A34 * A40 * A52 - A11 * A23 * A34 * A42 * A50 - A11 * A24 * A30 * A42 * A53 + A11 * A24 * A30 * A43 * A52 + A11 * A24 * A32 * A40 * A53 - A11 * A24 * A32 * A43 * A50 - A11 * A24 * A33 * A40 * A52 + A11 * A24 * A33 * A42 * A50 - A12 * A20 * A31 * A43 * A54 + A12 * A20 * A31 * A44 * A53 + A12 * A20 * A33 * A41 * A54 - A12 * A20 * A33 * A44 * A51 - A12 * A20 * A34 * A41 * A53 + A12 * A20 * A34 * A43 * A51 + A12 * A21 * A30 * A43 * A54 - A12 * A21 * A30 * A44 * A53 - A12 * A21 * A33 * A40 * A54 + A12 * A21 * A33 * A44 * A50 + A12 * A21 * A34 * A40 * A53 - A12 * A21 * A34 * A43 * A50 - A12 * A23 * A30 * A41 * A54 + A12 * A23 * A30 * A44 * A51 + A12 * A23 * A31 * A40 * A54 - A12 * A23 * A31 * A44 * A50 - A12 * A23 * A34 * A40 * A51 + A12 * A23 * A34 * A41 * A50 + A12 * A24 * A30 * A41 * A53 - A12 * A24 * A30 * A43 * A51 - A12 * A24 * A31 * A40 * A53 + A12 * A24 * A31 * A43 * A50 + A12 * A24 * A33 * A40 * A51 - A12 * A24 * A33 * A41 * A50 + A13 * A20 * A31 * A42 * A54 - A13 * A20 * A31 * A44 * A52 - A13 * A20 * A32 * A41 * A54 + A13 * A20 * A32 * A44 * A51 + A13 * A20 * A34 * A41 * A52 - A13 * A20 * A34 * A42 * A51 - A13 * A21 * A30 * A42 * A54 + A13 * A21 * A30 * A44 * A52 + A13 * A21 * A32 * A40 * A54 - A13 * A21 * A32 * A44 * A50 - A13 * A21 * A34 * A40 * A52 + A13 * A21 * A34 * A42 * A50 + A13 * A22 * A30 * A41 * A54 - A13 * A22 * A30 * A44 * A51 - A13 * A22 * A31 * A40 * A54 + A13 * A22 * A31 * A44 * A50 + A13 * A22 * A34 * A40 * A51 - A13 * A22 * A34 * A41 * A50 - A13 * A24 * A30 * A41 * A52 + A13 * A24 * A30 * A42 * A51 + A13 * A24 * A31 * A40 * A52 - A13 * A24 * A31 * A42 * A50 - A13 * A24 * A32 * A40 * A51 + A13 * A24 * A32 * A41 * A50 - A14 * A20 * A31 * A42 * A53 + A14 * A20 * A31 * A43 * A52 + A14 * A20 * A32 * A41 * A53 - A14 * A20 * A32 * A43 * A51 - A14 * A20 * A33 * A41 * A52 + A14 * A20 * A33 * A42 * A51 + A14 * A21 * A30 * A42 * A53 - A14 * A21 * A30 * A43 * A52 - A14 * A21 * A32 * A40 * A53 + A14 * A21 * A32 * A43 * A50 + A14 * A21 * A33 * A40 * A52 - A14 * A21 * A33 * A42 * A50 - A14 * A22 * A30 * A41 * A53 + A14 * A22 * A30 * A43 * A51 + A14 * A22 * A31 * A40 * A53 - A14 * A22 * A31 * A43 * A50 - A14 * A22 * A33 * A40 * A51 + A14 * A22 * A33 * A41 * A50 + A14 * A23 * A30 * A41 * A52 - A14 * A23 * A30 * A42 * A51 - A14 * A23 * A31 * A40 * A52 + A14 * A23 * A31 * A42 * A50 + A14 * A23 * A32 * A40 * A51 - A14 * A23 * A32 * A41 * A50) / detA;
    d[5][1] = (A00 * A21 * A32 * A43 * A54 - A00 * A21 * A32 * A44 * A53 - A00 * A21 * A33 * A42 * A54 + A00 * A21 * A33 * A44 * A52 + A00 * A21 * A34 * A42 * A53 - A00 * A21 * A34 * A43 * A52 - A00 * A22 * A31 * A43 * A54 + A00 * A22 * A31 * A44 * A53 + A00 * A22 * A33 * A41 * A54 - A00 * A22 * A33 * A44 * A51 - A00 * A22 * A34 * A41 * A53 + A00 * A22 * A34 * A43 * A51 + A00 * A23 * A31 * A42 * A54 - A00 * A23 * A31 * A44 * A52 - A00 * A23 * A32 * A41 * A54 + A00 * A23 * A32 * A44 * A51 + A00 * A23 * A34 * A41 * A52 - A00 * A23 * A34 * A42 * A51 - A00 * A24 * A31 * A42 * A53 + A00 * A24 * A31 * A43 * A52 + A00 * A24 * A32 * A41 * A53 - A00 * A24 * A32 * A43 * A51 - A00 * A24 * A33 * A41 * A52 + A00 * A24 * A33 * A42 * A51 - A01 * A20 * A32 * A43 * A54 + A01 * A20 * A32 * A44 * A53 + A01 * A20 * A33 * A42 * A54 - A01 * A20 * A33 * A44 * A52 - A01 * A20 * A34 * A42 * A53 + A01 * A20 * A34 * A43 * A52 + A01 * A22 * A30 * A43 * A54 - A01 * A22 * A30 * A44 * A53 - A01 * A22 * A33 * A40 * A54 + A01 * A22 * A33 * A44 * A50 + A01 * A22 * A34 * A40 * A53 - A01 * A22 * A34 * A43 * A50 - A01 * A23 * A30 * A42 * A54 + A01 * A23 * A30 * A44 * A52 + A01 * A23 * A32 * A40 * A54 - A01 * A23 * A32 * A44 * A50 - A01 * A23 * A34 * A40 * A52 + A01 * A23 * A34 * A42 * A50 + A01 * A24 * A30 * A42 * A53 - A01 * A24 * A30 * A43 * A52 - A01 * A24 * A32 * A40 * A53 + A01 * A24 * A32 * A43 * A50 + A01 * A24 * A33 * A40 * A52 - A01 * A24 * A33 * A42 * A50 + A02 * A20 * A31 * A43 * A54 - A02 * A20 * A31 * A44 * A53 - A02 * A20 * A33 * A41 * A54 + A02 * A20 * A33 * A44 * A51 + A02 * A20 * A34 * A41 * A53 - A02 * A20 * A34 * A43 * A51 - A02 * A21 * A30 * A43 * A54 + A02 * A21 * A30 * A44 * A53 + A02 * A21 * A33 * A40 * A54 - A02 * A21 * A33 * A44 * A50 - A02 * A21 * A34 * A40 * A53 + A02 * A21 * A34 * A43 * A50 + A02 * A23 * A30 * A41 * A54 - A02 * A23 * A30 * A44 * A51 - A02 * A23 * A31 * A40 * A54 + A02 * A23 * A31 * A44 * A50 + A02 * A23 * A34 * A40 * A51 - A02 * A23 * A34 * A41 * A50 - A02 * A24 * A30 * A41 * A53 + A02 * A24 * A30 * A43 * A51 + A02 * A24 * A31 * A40 * A53 - A02 * A24 * A31 * A43 * A50 - A02 * A24 * A33 * A40 * A51 + A02 * A24 * A33 * A41 * A50 - A03 * A20 * A31 * A42 * A54 + A03 * A20 * A31 * A44 * A52 + A03 * A20 * A32 * A41 * A54 - A03 * A20 * A32 * A44 * A51 - A03 * A20 * A34 * A41 * A52 + A03 * A20 * A34 * A42 * A51 + A03 * A21 * A30 * A42 * A54 - A03 * A21 * A30 * A44 * A52 - A03 * A21 * A32 * A40 * A54 + A03 * A21 * A32 * A44 * A50 + A03 * A21 * A34 * A40 * A52 - A03 * A21 * A34 * A42 * A50 - A03 * A22 * A30 * A41 * A54 + A03 * A22 * A30 * A44 * A51 + A03 * A22 * A31 * A40 * A54 - A03 * A22 * A31 * A44 * A50 - A03 * A22 * A34 * A40 * A51 + A03 * A22 * A34 * A41 * A50 + A03 * A24 * A30 * A41 * A52 - A03 * A24 * A30 * A42 * A51 - A03 * A24 * A31 * A40 * A52 + A03 * A24 * A31 * A42 * A50 + A03 * A24 * A32 * A40 * A51 - A03 * A24 * A32 * A41 * A50 + A04 * A20 * A31 * A42 * A53 - A04 * A20 * A31 * A43 * A52 - A04 * A20 * A32 * A41 * A53 + A04 * A20 * A32 * A43 * A51 + A04 * A20 * A33 * A41 * A52 - A04 * A20 * A33 * A42 * A51 - A04 * A21 * A30 * A42 * A53 + A04 * A21 * A30 * A43 * A52 + A04 * A21 * A32 * A40 * A53 - A04 * A21 * A32 * A43 * A50 - A04 * A21 * A33 * A40 * A52 + A04 * A21 * A33 * A42 * A50 + A04 * A22 * A30 * A41 * A53 - A04 * A22 * A30 * A43 * A51 - A04 * A22 * A31 * A40 * A53 + A04 * A22 * A31 * A43 * A50 + A04 * A22 * A33 * A40 * A51 - A04 * A22 * A33 * A41 * A50 - A04 * A23 * A30 * A41 * A52 + A04 * A23 * A30 * A42 * A51 + A04 * A23 * A31 * A40 * A52 - A04 * A23 * A31 * A42 * A50 - A04 * A23 * A32 * A40 * A51 + A04 * A23 * A32 * A41 * A50) / detA;
    d[5][2] = (A00 * A11 * A32 * A44 * A53 - A00 * A11 * A32 * A43 * A54 + A00 * A11 * A33 * A42 * A54 - A00 * A11 * A33 * A44 * A52 - A00 * A11 * A34 * A42 * A53 + A00 * A11 * A34 * A43 * A52 + A00 * A12 * A31 * A43 * A54 - A00 * A12 * A31 * A44 * A53 - A00 * A12 * A33 * A41 * A54 + A00 * A12 * A33 * A44 * A51 + A00 * A12 * A34 * A41 * A53 - A00 * A12 * A34 * A43 * A51 - A00 * A13 * A31 * A42 * A54 + A00 * A13 * A31 * A44 * A52 + A00 * A13 * A32 * A41 * A54 - A00 * A13 * A32 * A44 * A51 - A00 * A13 * A34 * A41 * A52 + A00 * A13 * A34 * A42 * A51 + A00 * A14 * A31 * A42 * A53 - A00 * A14 * A31 * A43 * A52 - A00 * A14 * A32 * A41 * A53 + A00 * A14 * A32 * A43 * A51 + A00 * A14 * A33 * A41 * A52 - A00 * A14 * A33 * A42 * A51 + A01 * A10 * A32 * A43 * A54 - A01 * A10 * A32 * A44 * A53 - A01 * A10 * A33 * A42 * A54 + A01 * A10 * A33 * A44 * A52 + A01 * A10 * A34 * A42 * A53 - A01 * A10 * A34 * A43 * A52 - A01 * A12 * A30 * A43 * A54 + A01 * A12 * A30 * A44 * A53 + A01 * A12 * A33 * A40 * A54 - A01 * A12 * A33 * A44 * A50 - A01 * A12 * A34 * A40 * A53 + A01 * A12 * A34 * A43 * A50 + A01 * A13 * A30 * A42 * A54 - A01 * A13 * A30 * A44 * A52 - A01 * A13 * A32 * A40 * A54 + A01 * A13 * A32 * A44 * A50 + A01 * A13 * A34 * A40 * A52 - A01 * A13 * A34 * A42 * A50 - A01 * A14 * A30 * A42 * A53 + A01 * A14 * A30 * A43 * A52 + A01 * A14 * A32 * A40 * A53 - A01 * A14 * A32 * A43 * A50 - A01 * A14 * A33 * A40 * A52 + A01 * A14 * A33 * A42 * A50 - A02 * A10 * A31 * A43 * A54 + A02 * A10 * A31 * A44 * A53 + A02 * A10 * A33 * A41 * A54 - A02 * A10 * A33 * A44 * A51 - A02 * A10 * A34 * A41 * A53 + A02 * A10 * A34 * A43 * A51 + A02 * A11 * A30 * A43 * A54 - A02 * A11 * A30 * A44 * A53 - A02 * A11 * A33 * A40 * A54 + A02 * A11 * A33 * A44 * A50 + A02 * A11 * A34 * A40 * A53 - A02 * A11 * A34 * A43 * A50 - A02 * A13 * A30 * A41 * A54 + A02 * A13 * A30 * A44 * A51 + A02 * A13 * A31 * A40 * A54 - A02 * A13 * A31 * A44 * A50 - A02 * A13 * A34 * A40 * A51 + A02 * A13 * A34 * A41 * A50 + A02 * A14 * A30 * A41 * A53 - A02 * A14 * A30 * A43 * A51 - A02 * A14 * A31 * A40 * A53 + A02 * A14 * A31 * A43 * A50 + A02 * A14 * A33 * A40 * A51 - A02 * A14 * A33 * A41 * A50 + A03 * A10 * A31 * A42 * A54 - A03 * A10 * A31 * A44 * A52 - A03 * A10 * A32 * A41 * A54 + A03 * A10 * A32 * A44 * A51 + A03 * A10 * A34 * A41 * A52 - A03 * A10 * A34 * A42 * A51 - A03 * A11 * A30 * A42 * A54 + A03 * A11 * A30 * A44 * A52 + A03 * A11 * A32 * A40 * A54 - A03 * A11 * A32 * A44 * A50 - A03 * A11 * A34 * A40 * A52 + A03 * A11 * A34 * A42 * A50 + A03 * A12 * A30 * A41 * A54 - A03 * A12 * A30 * A44 * A51 - A03 * A12 * A31 * A40 * A54 + A03 * A12 * A31 * A44 * A50 + A03 * A12 * A34 * A40 * A51 - A03 * A12 * A34 * A41 * A50 - A03 * A14 * A30 * A41 * A52 + A03 * A14 * A30 * A42 * A51 + A03 * A14 * A31 * A40 * A52 - A03 * A14 * A31 * A42 * A50 - A03 * A14 * A32 * A40 * A51 + A03 * A14 * A32 * A41 * A50 - A04 * A10 * A31 * A42 * A53 + A04 * A10 * A31 * A43 * A52 + A04 * A10 * A32 * A41 * A53 - A04 * A10 * A32 * A43 * A51 - A04 * A10 * A33 * A41 * A52 + A04 * A10 * A33 * A42 * A51 + A04 * A11 * A30 * A42 * A53 - A04 * A11 * A30 * A43 * A52 - A04 * A11 * A32 * A40 * A53 + A04 * A11 * A32 * A43 * A50 + A04 * A11 * A33 * A40 * A52 - A04 * A11 * A33 * A42 * A50 - A04 * A12 * A30 * A41 * A53 + A04 * A12 * A30 * A43 * A51 + A04 * A12 * A31 * A40 * A53 - A04 * A12 * A31 * A43 * A50 - A04 * A12 * A33 * A40 * A51 + A04 * A12 * A33 * A41 * A50 + A04 * A13 * A30 * A41 * A52 - A04 * A13 * A30 * A42 * A51 - A04 * A13 * A31 * A40 * A52 + A04 * A13 * A31 * A42 * A50 + A04 * A13 * A32 * A40 * A51 - A04 * A13 * A32 * A41 * A50) / detA;
    d[5][3] = (A00 * A11 * A22 * A43 * A54 - A00 * A11 * A22 * A44 * A53 - A00 * A11 * A23 * A42 * A54 + A00 * A11 * A23 * A44 * A52 + A00 * A11 * A24 * A42 * A53 - A00 * A11 * A24 * A43 * A52 - A00 * A12 * A21 * A43 * A54 + A00 * A12 * A21 * A44 * A53 + A00 * A12 * A23 * A41 * A54 - A00 * A12 * A23 * A44 * A51 - A00 * A12 * A24 * A41 * A53 + A00 * A12 * A24 * A43 * A51 + A00 * A13 * A21 * A42 * A54 - A00 * A13 * A21 * A44 * A52 - A00 * A13 * A22 * A41 * A54 + A00 * A13 * A22 * A44 * A51 + A00 * A13 * A24 * A41 * A52 - A00 * A13 * A24 * A42 * A51 - A00 * A14 * A21 * A42 * A53 + A00 * A14 * A21 * A43 * A52 + A00 * A14 * A22 * A41 * A53 - A00 * A14 * A22 * A43 * A51 - A00 * A14 * A23 * A41 * A52 + A00 * A14 * A23 * A42 * A51 - A01 * A10 * A22 * A43 * A54 + A01 * A10 * A22 * A44 * A53 + A01 * A10 * A23 * A42 * A54 - A01 * A10 * A23 * A44 * A52 - A01 * A10 * A24 * A42 * A53 + A01 * A10 * A24 * A43 * A52 + A01 * A12 * A20 * A43 * A54 - A01 * A12 * A20 * A44 * A53 - A01 * A12 * A23 * A40 * A54 + A01 * A12 * A23 * A44 * A50 + A01 * A12 * A24 * A40 * A53 - A01 * A12 * A24 * A43 * A50 - A01 * A13 * A20 * A42 * A54 + A01 * A13 * A20 * A44 * A52 + A01 * A13 * A22 * A40 * A54 - A01 * A13 * A22 * A44 * A50 - A01 * A13 * A24 * A40 * A52 + A01 * A13 * A24 * A42 * A50 + A01 * A14 * A20 * A42 * A53 - A01 * A14 * A20 * A43 * A52 - A01 * A14 * A22 * A40 * A53 + A01 * A14 * A22 * A43 * A50 + A01 * A14 * A23 * A40 * A52 - A01 * A14 * A23 * A42 * A50 + A02 * A10 * A21 * A43 * A54 - A02 * A10 * A21 * A44 * A53 - A02 * A10 * A23 * A41 * A54 + A02 * A10 * A23 * A44 * A51 + A02 * A10 * A24 * A41 * A53 - A02 * A10 * A24 * A43 * A51 - A02 * A11 * A20 * A43 * A54 + A02 * A11 * A20 * A44 * A53 + A02 * A11 * A23 * A40 * A54 - A02 * A11 * A23 * A44 * A50 - A02 * A11 * A24 * A40 * A53 + A02 * A11 * A24 * A43 * A50 + A02 * A13 * A20 * A41 * A54 - A02 * A13 * A20 * A44 * A51 - A02 * A13 * A21 * A40 * A54 + A02 * A13 * A21 * A44 * A50 + A02 * A13 * A24 * A40 * A51 - A02 * A13 * A24 * A41 * A50 - A02 * A14 * A20 * A41 * A53 + A02 * A14 * A20 * A43 * A51 + A02 * A14 * A21 * A40 * A53 - A02 * A14 * A21 * A43 * A50 - A02 * A14 * A23 * A40 * A51 + A02 * A14 * A23 * A41 * A50 - A03 * A10 * A21 * A42 * A54 + A03 * A10 * A21 * A44 * A52 + A03 * A10 * A22 * A41 * A54 - A03 * A10 * A22 * A44 * A51 - A03 * A10 * A24 * A41 * A52 + A03 * A10 * A24 * A42 * A51 + A03 * A11 * A20 * A42 * A54 - A03 * A11 * A20 * A44 * A52 - A03 * A11 * A22 * A40 * A54 + A03 * A11 * A22 * A44 * A50 + A03 * A11 * A24 * A40 * A52 - A03 * A11 * A24 * A42 * A50 - A03 * A12 * A20 * A41 * A54 + A03 * A12 * A20 * A44 * A51 + A03 * A12 * A21 * A40 * A54 - A03 * A12 * A21 * A44 * A50 - A03 * A12 * A24 * A40 * A51 + A03 * A12 * A24 * A41 * A50 + A03 * A14 * A20 * A41 * A52 - A03 * A14 * A20 * A42 * A51 - A03 * A14 * A21 * A40 * A52 + A03 * A14 * A21 * A42 * A50 + A03 * A14 * A22 * A40 * A51 - A03 * A14 * A22 * A41 * A50 + A04 * A10 * A21 * A42 * A53 - A04 * A10 * A21 * A43 * A52 - A04 * A10 * A22 * A41 * A53 + A04 * A10 * A22 * A43 * A51 + A04 * A10 * A23 * A41 * A52 - A04 * A10 * A23 * A42 * A51 - A04 * A11 * A20 * A42 * A53 + A04 * A11 * A20 * A43 * A52 + A04 * A11 * A22 * A40 * A53 - A04 * A11 * A22 * A43 * A50 - A04 * A11 * A23 * A40 * A52 + A04 * A11 * A23 * A42 * A50 + A04 * A12 * A20 * A41 * A53 - A04 * A12 * A20 * A43 * A51 - A04 * A12 * A21 * A40 * A53 + A04 * A12 * A21 * A43 * A50 + A04 * A12 * A23 * A40 * A51 - A04 * A12 * A23 * A41 * A50 - A04 * A13 * A20 * A41 * A52 + A04 * A13 * A20 * A42 * A51 + A04 * A13 * A21 * A40 * A52 - A04 * A13 * A21 * A42 * A50 - A04 * A13 * A22 * A40 * A51 + A04 * A13 * A22 * A41 * A50) / detA;
    d[5][4] = (A00 * A11 * A22 * A34 * A53 - A00 * A11 * A22 * A33 * A54 + A00 * A11 * A23 * A32 * A54 - A00 * A11 * A23 * A34 * A52 - A00 * A11 * A24 * A32 * A53 + A00 * A11 * A24 * A33 * A52 + A00 * A12 * A21 * A33 * A54 - A00 * A12 * A21 * A34 * A53 - A00 * A12 * A23 * A31 * A54 + A00 * A12 * A23 * A34 * A51 + A00 * A12 * A24 * A31 * A53 - A00 * A12 * A24 * A33 * A51 - A00 * A13 * A21 * A32 * A54 + A00 * A13 * A21 * A34 * A52 + A00 * A13 * A22 * A31 * A54 - A00 * A13 * A22 * A34 * A51 - A00 * A13 * A24 * A31 * A52 + A00 * A13 * A24 * A32 * A51 + A00 * A14 * A21 * A32 * A53 - A00 * A14 * A21 * A33 * A52 - A00 * A14 * A22 * A31 * A53 + A00 * A14 * A22 * A33 * A51 + A00 * A14 * A23 * A31 * A52 - A00 * A14 * A23 * A32 * A51 + A01 * A10 * A22 * A33 * A54 - A01 * A10 * A22 * A34 * A53 - A01 * A10 * A23 * A32 * A54 + A01 * A10 * A23 * A34 * A52 + A01 * A10 * A24 * A32 * A53 - A01 * A10 * A24 * A33 * A52 - A01 * A12 * A20 * A33 * A54 + A01 * A12 * A20 * A34 * A53 + A01 * A12 * A23 * A30 * A54 - A01 * A12 * A23 * A34 * A50 - A01 * A12 * A24 * A30 * A53 + A01 * A12 * A24 * A33 * A50 + A01 * A13 * A20 * A32 * A54 - A01 * A13 * A20 * A34 * A52 - A01 * A13 * A22 * A30 * A54 + A01 * A13 * A22 * A34 * A50 + A01 * A13 * A24 * A30 * A52 - A01 * A13 * A24 * A32 * A50 - A01 * A14 * A20 * A32 * A53 + A01 * A14 * A20 * A33 * A52 + A01 * A14 * A22 * A30 * A53 - A01 * A14 * A22 * A33 * A50 - A01 * A14 * A23 * A30 * A52 + A01 * A14 * A23 * A32 * A50 - A02 * A10 * A21 * A33 * A54 + A02 * A10 * A21 * A34 * A53 + A02 * A10 * A23 * A31 * A54 - A02 * A10 * A23 * A34 * A51 - A02 * A10 * A24 * A31 * A53 + A02 * A10 * A24 * A33 * A51 + A02 * A11 * A20 * A33 * A54 - A02 * A11 * A20 * A34 * A53 - A02 * A11 * A23 * A30 * A54 + A02 * A11 * A23 * A34 * A50 + A02 * A11 * A24 * A30 * A53 - A02 * A11 * A24 * A33 * A50 - A02 * A13 * A20 * A31 * A54 + A02 * A13 * A20 * A34 * A51 + A02 * A13 * A21 * A30 * A54 - A02 * A13 * A21 * A34 * A50 - A02 * A13 * A24 * A30 * A51 + A02 * A13 * A24 * A31 * A50 + A02 * A14 * A20 * A31 * A53 - A02 * A14 * A20 * A33 * A51 - A02 * A14 * A21 * A30 * A53 + A02 * A14 * A21 * A33 * A50 + A02 * A14 * A23 * A30 * A51 - A02 * A14 * A23 * A31 * A50 + A03 * A10 * A21 * A32 * A54 - A03 * A10 * A21 * A34 * A52 - A03 * A10 * A22 * A31 * A54 + A03 * A10 * A22 * A34 * A51 + A03 * A10 * A24 * A31 * A52 - A03 * A10 * A24 * A32 * A51 - A03 * A11 * A20 * A32 * A54 + A03 * A11 * A20 * A34 * A52 + A03 * A11 * A22 * A30 * A54 - A03 * A11 * A22 * A34 * A50 - A03 * A11 * A24 * A30 * A52 + A03 * A11 * A24 * A32 * A50 + A03 * A12 * A20 * A31 * A54 - A03 * A12 * A20 * A34 * A51 - A03 * A12 * A21 * A30 * A54 + A03 * A12 * A21 * A34 * A50 + A03 * A12 * A24 * A30 * A51 - A03 * A12 * A24 * A31 * A50 - A03 * A14 * A20 * A31 * A52 + A03 * A14 * A20 * A32 * A51 + A03 * A14 * A21 * A30 * A52 - A03 * A14 * A21 * A32 * A50 - A03 * A14 * A22 * A30 * A51 + A03 * A14 * A22 * A31 * A50 - A04 * A10 * A21 * A32 * A53 + A04 * A10 * A21 * A33 * A52 + A04 * A10 * A22 * A31 * A53 - A04 * A10 * A22 * A33 * A51 - A04 * A10 * A23 * A31 * A52 + A04 * A10 * A23 * A32 * A51 + A04 * A11 * A20 * A32 * A53 - A04 * A11 * A20 * A33 * A52 - A04 * A11 * A22 * A30 * A53 + A04 * A11 * A22 * A33 * A50 + A04 * A11 * A23 * A30 * A52 - A04 * A11 * A23 * A32 * A50 - A04 * A12 * A20 * A31 * A53 + A04 * A12 * A20 * A33 * A51 + A04 * A12 * A21 * A30 * A53 - A04 * A12 * A21 * A33 * A50 - A04 * A12 * A23 * A30 * A51 + A04 * A12 * A23 * A31 * A50 + A04 * A13 * A20 * A31 * A52 - A04 * A13 * A20 * A32 * A51 - A04 * A13 * A21 * A30 * A52 + A04 * A13 * A21 * A32 * A50 + A04 * A13 * A22 * A30 * A51 - A04 * A13 * A22 * A31 * A50) / detA;
    d[5][5] = (A00 * A11 * A22 * A33 * A44 - A00 * A11 * A22 * A34 * A43 - A00 * A11 * A23 * A32 * A44 + A00 * A11 * A23 * A34 * A42 + A00 * A11 * A24 * A32 * A43 - A00 * A11 * A24 * A33 * A42 - A00 * A12 * A21 * A33 * A44 + A00 * A12 * A21 * A34 * A43 + A00 * A12 * A23 * A31 * A44 - A00 * A12 * A23 * A34 * A41 - A00 * A12 * A24 * A31 * A43 + A00 * A12 * A24 * A33 * A41 + A00 * A13 * A21 * A32 * A44 - A00 * A13 * A21 * A34 * A42 - A00 * A13 * A22 * A31 * A44 + A00 * A13 * A22 * A34 * A41 + A00 * A13 * A24 * A31 * A42 - A00 * A13 * A24 * A32 * A41 - A00 * A14 * A21 * A32 * A43 + A00 * A14 * A21 * A33 * A42 + A00 * A14 * A22 * A31 * A43 - A00 * A14 * A22 * A33 * A41 - A00 * A14 * A23 * A31 * A42 + A00 * A14 * A23 * A32 * A41 - A01 * A10 * A22 * A33 * A44 + A01 * A10 * A22 * A34 * A43 + A01 * A10 * A23 * A32 * A44 - A01 * A10 * A23 * A34 * A42 - A01 * A10 * A24 * A32 * A43 + A01 * A10 * A24 * A33 * A42 + A01 * A12 * A20 * A33 * A44 - A01 * A12 * A20 * A34 * A43 - A01 * A12 * A23 * A30 * A44 + A01 * A12 * A23 * A34 * A40 + A01 * A12 * A24 * A30 * A43 - A01 * A12 * A24 * A33 * A40 - A01 * A13 * A20 * A32 * A44 + A01 * A13 * A20 * A34 * A42 + A01 * A13 * A22 * A30 * A44 - A01 * A13 * A22 * A34 * A40 - A01 * A13 * A24 * A30 * A42 + A01 * A13 * A24 * A32 * A40 + A01 * A14 * A20 * A32 * A43 - A01 * A14 * A20 * A33 * A42 - A01 * A14 * A22 * A30 * A43 + A01 * A14 * A22 * A33 * A40 + A01 * A14 * A23 * A30 * A42 - A01 * A14 * A23 * A32 * A40 + A02 * A10 * A21 * A33 * A44 - A02 * A10 * A21 * A34 * A43 - A02 * A10 * A23 * A31 * A44 + A02 * A10 * A23 * A34 * A41 + A02 * A10 * A24 * A31 * A43 - A02 * A10 * A24 * A33 * A41 - A02 * A11 * A20 * A33 * A44 + A02 * A11 * A20 * A34 * A43 + A02 * A11 * A23 * A30 * A44 - A02 * A11 * A23 * A34 * A40 - A02 * A11 * A24 * A30 * A43 + A02 * A11 * A24 * A33 * A40 + A02 * A13 * A20 * A31 * A44 - A02 * A13 * A20 * A34 * A41 - A02 * A13 * A21 * A30 * A44 + A02 * A13 * A21 * A34 * A40 + A02 * A13 * A24 * A30 * A41 - A02 * A13 * A24 * A31 * A40 - A02 * A14 * A20 * A31 * A43 + A02 * A14 * A20 * A33 * A41 + A02 * A14 * A21 * A30 * A43 - A02 * A14 * A21 * A33 * A40 - A02 * A14 * A23 * A30 * A41 + A02 * A14 * A23 * A31 * A40 - A03 * A10 * A21 * A32 * A44 + A03 * A10 * A21 * A34 * A42 + A03 * A10 * A22 * A31 * A44 - A03 * A10 * A22 * A34 * A41 - A03 * A10 * A24 * A31 * A42 + A03 * A10 * A24 * A32 * A41 + A03 * A11 * A20 * A32 * A44 - A03 * A11 * A20 * A34 * A42 - A03 * A11 * A22 * A30 * A44 + A03 * A11 * A22 * A34 * A40 + A03 * A11 * A24 * A30 * A42 - A03 * A11 * A24 * A32 * A40 - A03 * A12 * A20 * A31 * A44 + A03 * A12 * A20 * A34 * A41 + A03 * A12 * A21 * A30 * A44 - A03 * A12 * A21 * A34 * A40 - A03 * A12 * A24 * A30 * A41 + A03 * A12 * A24 * A31 * A40 + A03 * A14 * A20 * A31 * A42 - A03 * A14 * A20 * A32 * A41 - A03 * A14 * A21 * A30 * A42 + A03 * A14 * A21 * A32 * A40 + A03 * A14 * A22 * A30 * A41 - A03 * A14 * A22 * A31 * A40 + A04 * A10 * A21 * A32 * A43 - A04 * A10 * A21 * A33 * A42 - A04 * A10 * A22 * A31 * A43 + A04 * A10 * A22 * A33 * A41 + A04 * A10 * A23 * A31 * A42 - A04 * A10 * A23 * A32 * A41 - A04 * A11 * A20 * A32 * A43 + A04 * A11 * A20 * A33 * A42 + A04 * A11 * A22 * A30 * A43 - A04 * A11 * A22 * A33 * A40 - A04 * A11 * A23 * A30 * A42 + A04 * A11 * A23 * A32 * A40 + A04 * A12 * A20 * A31 * A43 - A04 * A12 * A20 * A33 * A41 - A04 * A12 * A21 * A30 * A43 + A04 * A12 * A21 * A33 * A40 + A04 * A12 * A23 * A30 * A41 - A04 * A12 * A23 * A31 * A40 - A04 * A13 * A20 * A31 * A42 + A04 * A13 * A20 * A32 * A41 + A04 * A13 * A21 * A30 * A42 - A04 * A13 * A21 * A32 * A40 - A04 * A13 * A22 * A30 * A41 + A04 * A13 * A22 * A31 * A40) / detA;

}

void inversekinematicsBtoL(double th[]) {

	double tempA[3] = { 0, };
	double tempB[3] = { 0, };
	double tempC[3] = { 0, };
	double diffp[3] = { 0, };
	//double jacobi[6][6] = { 0, };
	double dq[6] = { 0, };
	double a[6][6], d[6][6], deter;

	int i, n;
	// int print_matrix = 1;
	// int print_inverse = 2;


	//1
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[6][i];
	}

	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW1[i][3];
	}
	
	cross(tempA, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][0] = tempB[i];
	}


	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][0] = _A[6][i];
	}

	//2

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[7][i];
	}
	matrixmultiply331(TLW1, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW2[i][3];
	}
	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][1] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][1] = tempC[i];
	}


	//3

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[8][i];
	}
	matrixmultiply331(TLW2, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW3[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][2] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][2] = tempC[i];
	}

	//4
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[9][i];
	}
	matrixmultiply331(TLW3, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW4[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][3] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][3] = tempC[i];
	}


	//5

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[10][i];
	}
	matrixmultiply331(TLW4, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW5[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][4] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][4] = tempC[i];
	}

	//6
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[11][i];
	}
	matrixmultiply331(TLW5, tempA, tempC);
	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW6[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {	
		_jacobi[i][5] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][5] = tempC[i];
	}


	//matrixprint6(jacobi);
	

	if(inverse_type == 0)
	{
		n = cin_function(a, _jacobi);   // read function
 								//cout_function(a, n, print_matrix);   // read function
		deter = (double)det(a, n);   // read function
		inverse(a, d, n, deter);   // read function
	}
	else
	{
		inverse_solved(_jacobi, d);
	}
							//cout_function(d, n, print_inverse);   // read function

	errcalc(targetp, targetr, TLWE, err);
	matrixmultiply661(d, err, dq);

	for (i = 0; i < 6; i++) {	
		th[6 + i] = 0.5 * dq[i] + th[6 + i];
	}
}

void inversekinematicsBtoR(double th[]) {

	double tempA[3] = { 0, };
	double tempB[3] = { 0, };
	double tempC[3] = { 0, };
	double diffp[3] = { 0, };
	//double jacobi[6][6] = { 0, };
	double dq[6] = { 0, };

	double a[6][6], d[6][6], deter;
	int i, n;
	// int print_matrix = 1;
	// int print_inverse = 2;



	//1

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[5][i];
	}


	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW1[i][3];
	}


	cross(tempA, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][0] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][0] = _A[5][i];
	}

	//2
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[4][i];
	}

	matrixmultiply331(TRW1, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW2[i][3];
	}

	cross(tempC, diffp, tempB);


	for (i = 0; i < 3; i++) {
		_jacobi[i][1] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][1] = tempC[i];
	}


	//3

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[3][i];
	}
	matrixmultiply331(TRW2, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW3[i][3];
	}

	cross(tempC, diffp, tempB);


	for (i = 0; i < 3; i++) {
		_jacobi[i][2] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][2] = tempC[i];
	}


	//4

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[2][i];
	}
	matrixmultiply331(TRW3, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW4[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][3] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][3] = tempC[i];
	}

	//5
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[1][i];
	}
	matrixmultiply331(TRW4, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW5[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][4] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][4] = tempC[i];
	}
	

	//6

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[0][i];
	}
	matrixmultiply331(TRW5, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW6[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi[i][5] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi[3 + i][5] = tempC[i];
	}



//matrixprint6(jacobi);

	if(inverse_type == 0)
	{
		n = cin_function(a, _jacobi);   // read function
  								//cout_function(a, n, print_matrix);   // read function
		deter = (double)det(a, n);   // read function
		inverse(a, d, n, deter);   // read function
	}

	else
	{
		inverse_solved(_jacobi, d);
	}
							//cout_function(d, n, print_inverse);   // read function

	errcalc(targetp, targetr, TRWE, err);
	matrixmultiply661(d, err, dq);

	for (i = 0; i < 6; i++) {
		th[5 - i] = 0.5 * dq[i] + th[5 - i];
	}
}
//add 0630

double _IA[12][3] = {
	{ 1,0,0 } ,
	{ 0,1,0 },
	{ 0,1,0 },
	{ 0,1,0 },
	{ 1,0,0 },
	{ 0,0,1 },
	{ 0,0,-1 },
	{ -1,0,0 },
	{ 0,-1,0 },
	{ 0,-1,0 },
	{ 0,-1,0 },
	{ -1,0,0 }
};


// //double Itargetp[3];


double TRWB[4][4] = { { 1,0,0,0 }
,{ 0,1,0,0 }
,{ 0,0,1,LF }
,{ 0, 0, 0, 1 } };

double TLWB[4][4] = { { 1,0,0,0 }
,{ 0,1,0,L3*2 }
,{ 0,0,1,LF }
,{ 0, 0, 0, 1 } };



double i_jacobi[6][6];
///////////////////INVERSE  좌표계///////////////


// double ITRWE[4][4];
// double ITRW6[4][4];
// double ITRW5[4][4];
// double ITRW4[4][4];
// double ITRW3[4][4];
// double ITRW2[4][4];
// double ITRW1[4][4];

double ITRB1[4][4];
double ITR12[4][4];
double ITR23[4][4];
double ITR34[4][4];
double ITR45[4][4];
double ITR56[4][4];
double ITR6E[4][4];


double ITRB2[4][4];
double ITRB3[4][4];
double ITRB4[4][4];
double ITRB5[4][4];
double ITRB6[4][4];
double ITRBE[4][4];

//B to l 좌표계
/*
double ITLWE[4][4] = { 0, };
double ITLW6[4][4] = { 0, };
double ITLW5[4][4] = { 0, };
double ITLW4[4][4] = { 0, };
double ITLW3[4][4] = { 0, };
double ITLW2[4][4] = { 0, };
double ITLW1[4][4] = { 0, };
*/


double ITLB1[4][4];
double ITL12[4][4];
double ITL23[4][4];
double ITL34[4][4];
double ITL45[4][4];
double ITL56[4][4];
double ITL6E[4][4];


double ITLB2[4][4];
double ITLB3[4][4];
double ITLB4[4][4];
double ITLB5[4][4];
double ITLB6[4][4];
double ITLBE[4][4];




void forwardkinematics_FTB(double th[]) {


	ITRB1[0][0] = 1;
	ITRB1[0][1] = 0;
	ITRB1[0][2] = 0;
	ITRB1[0][3] = -0.024;

	ITRB1[1][0] = 0;
	ITRB1[1][1] = cos(th[0]);
	ITRB1[1][2] = -sin(th[0]);
	ITRB1[1][3] = 0;//0.0125;

	ITRB1[2][0] = 0;
	ITRB1[2][1] = sin(th[0]);
	ITRB1[2][2] = cos(th[0]);
	ITRB1[2][3] = LF;//0;//LF;//LF;//

	ITRB1[3][0] = 0;
	ITRB1[3][1] = 0;
	ITRB1[3][2] = 0;
	ITRB1[3][3] = 1;



	ITR12[0][0] = cos(th[1]);
	ITR12[0][1] = 0;
	ITR12[0][2] = sin(th[1]);
	ITR12[0][3] = 0.0241;

	ITR12[1][0] = 0;
	ITR12[1][1] = 1;
	ITR12[1][2] = 0;
	ITR12[1][3] = -0.019;

	ITR12[2][0] = -sin(th[1]);
	ITR12[2][1] = 0;
	ITR12[2][2] = cos(th[1]);
	ITR12[2][3] = 0;

	ITR12[3][0] = 0;
	ITR12[3][1] = 0;
	ITR12[3][2] = 0;
	ITR12[3][3] = 1;



	ITR23[0][0] = cos(th[2]);
	ITR23[0][1] = 0;
	ITR23[0][2] = sin(th[2]);
	ITR23[0][3] = 0;

	ITR23[1][0] = 0;
	ITR23[1][1] = 1;
	ITR23[1][2] = 0;
	ITR23[1][3] = 0;

	ITR23[2][0] = -sin(th[2]);
	ITR23[2][1] = 0;
	ITR23[2][2] = cos(th[2]);
	ITR23[2][3] = L2;

	ITR23[3][0] = 0;
	ITR23[3][1] = 0;
	ITR23[3][2] = 0;
	ITR23[3][3] = 1;



	ITR34[0][0] = cos(th[3]);
	ITR34[0][1] = 0;
	ITR34[0][2] = sin(th[3]);
	ITR34[0][3] = 0;

	ITR34[1][0] = 0;
	ITR34[1][1] = 1;
	ITR34[1][2] = 0;
	ITR34[1][3] = 0;

	ITR34[2][0] = -sin(th[3]);
	ITR34[2][1] = 0;
	ITR34[2][2] = cos(th[3]);
	ITR34[2][3] = L1;

	ITR34[3][0] = 0;
	ITR34[3][1] = 0;
	ITR34[3][2] = 0;
	ITR34[3][3] = 1;



	ITR45[0][0] = 1;
	ITR45[0][1] = 0;
	ITR45[0][2] = 0;
	ITR45[0][3] = -0.0241;

	ITR45[1][0] = 0;
	ITR45[1][1] = cos(th[4]);
	ITR45[1][2] = -sin(th[4]);
	ITR45[1][3] = 0.019;

	ITR45[2][0] = 0;
	ITR45[2][1] = sin(th[4]);
	ITR45[2][2] = cos(th[4]);
	ITR45[2][3] = 0;

	ITR45[3][0] = 0;
	ITR45[3][1] = 0;
	ITR45[3][2] = 0;
	ITR45[3][3] = 1;


	ITR56[0][0] = cos(th[5]);
	ITR56[0][1] = -sin(th[5]);
	ITR56[0][2] = 0;
	ITR56[0][3] = 0.024;

	ITR56[1][0] = sin(th[5]);
	ITR56[1][1] = cos(th[5]);
	ITR56[1][2] = 0;
	ITR56[1][3] = 0;

	ITR56[2][0] = 0;
	ITR56[2][1] = 0;
	ITR56[2][2] = 1;
	ITR56[2][3] = 0.0285;

	ITR56[3][0] = 0;
	ITR56[3][1] = 0;
	ITR56[3][2] = 0;
	ITR56[3][3] = 1;



	ITR6E[0][0] = 1;
	ITR6E[0][1] = 0;
	ITR6E[0][2] = 0;
	ITR6E[0][3] = BX;     //재확인

	ITR6E[1][0] = 0;
	ITR6E[1][1] = 1;
	ITR6E[1][2] = 0;
	ITR6E[1][3] = L3;

	ITR6E[2][0] = 0;
	ITR6E[2][1] = 0;
	ITR6E[2][2] = 1;
	ITR6E[2][3] = BZ;

	ITR6E[3][0] = 0;
	ITR6E[3][1] = 0;
	ITR6E[3][2] = 0;
	ITR6E[3][3] = 1;



	ITL6E[0][0] = 1;
	ITL6E[0][1] = 0;
	ITL6E[0][2] = 0;
	ITL6E[0][3] = BX;

	ITL6E[1][0] = 0;
	ITL6E[1][1] = 1;
	ITL6E[1][2] = 0;
	ITL6E[1][3] = -L3;

	ITL6E[2][0] = 0;
	ITL6E[2][1] = 0;
	ITL6E[2][2] = 1;
	ITL6E[2][3] = BZ;

	ITL6E[3][0] = 0;
	ITL6E[3][1] = 0;
	ITL6E[3][2] = 0;
	ITL6E[3][3] = 1;


	ITL56[0][0] = cos(-th[6]);
	ITL56[0][1] = -sin(-th[6]);
	ITL56[0][2] = 0;
	ITL56[0][3] = 0.024;

	ITL56[1][0] = sin(-th[6]);
	ITL56[1][1] = cos(-th[6]);
	ITL56[1][2] = 0;
	ITL56[1][3] = 0;

	ITL56[2][0] = 0;
	ITL56[2][1] = 0;
	ITL56[2][2] = 1;
	ITL56[2][3] = 0.0285;

	ITL56[3][0] = 0;
	ITL56[3][1] = 0;
	ITL56[3][2] = 0;
	ITL56[3][3] = 1;


	ITL45[0][0] = 1;
	ITL45[0][1] = 0;
	ITL45[0][2] = 0;
	ITL45[0][3] = -0.0241;

	ITL45[1][0] = 0;
	ITL45[1][1] = cos(-th[7]);
	ITL45[1][2] = -sin(-th[7]);
	ITL45[1][3] = -0.019;

	ITL45[2][0] = 0;
	ITL45[2][1] = sin(-th[7]);
	ITL45[2][2] = cos(-th[7]);
	ITL45[2][3] = 0;

	ITL45[3][0] = 0;
	ITL45[3][1] = 0;
	ITL45[3][2] = 0;
	ITL45[3][3] = 1;


	ITL34[0][0] = cos(-th[8]);
	ITL34[0][1] = 0;
	ITL34[0][2] = sin(-th[8]);
	ITL34[0][3] = 0;

	ITL34[1][0] = 0;
	ITL34[1][1] = 1;
	ITL34[1][2] = 0;
	ITL34[1][3] = 0;

	ITL34[2][0] = -sin(-th[8]);
	ITL34[2][1] = 0;
	ITL34[2][2] = cos(-th[8]);
	ITL34[2][3] = L1;

	ITL34[3][0] = 0;
	ITL34[3][1] = 0;
	ITL34[3][2] = 0;
	ITL34[3][3] = 1;



	ITL23[0][0] = cos(-th[9]);
	ITL23[0][1] = 0;
	ITL23[0][2] = sin(-th[9]);
	ITL23[0][3] = 0;

	ITL23[1][0] = 0;
	ITL23[1][1] = 1;
	ITL23[1][2] = 0;
	ITL23[1][3] = 0;

	ITL23[2][0] = -sin(-th[9]);
	ITL23[2][1] = 0;
	ITL23[2][2] = cos(-th[9]);
	ITL23[2][3] = L2;

	ITL23[3][0] = 0;
	ITL23[3][1] = 0;
	ITL23[3][2] = 0;
	ITL23[3][3] = 1;



	ITL12[0][0] = cos(-th[10]);
	ITL12[0][1] = 0;
	ITL12[0][2] = sin(-th[10]);
	ITL12[0][3] = 0.0241;

	ITL12[1][0] = 0;
	ITL12[1][1] = 1;
	ITL12[1][2] = 0;
	ITL12[1][3] = 0.019;

	ITL12[2][0] = -sin(-th[10]);
	ITL12[2][1] = 0;
	ITL12[2][2] = cos(-th[10]);
	ITL12[2][3] = 0;

	ITL12[3][0] = 0;
	ITL12[3][1] = 0;
	ITL12[3][2] = 0;
	ITL12[3][3] = 1;


	ITLB1[0][0] = 1;
	ITLB1[0][1] = 0;
	ITLB1[0][2] = 0;
	ITLB1[0][3] = -0.024;

	ITLB1[1][0] = 0;
	ITLB1[1][1] = cos(-th[11]);
	ITLB1[1][2] = -sin(-th[11]);
	ITLB1[1][3] = 0;//-0.0125;

	ITLB1[2][0] = 0;
	ITLB1[2][1] = sin(-th[11]);
	ITLB1[2][2] = cos(-th[11]);
	ITLB1[2][3] = LF;//0;//LF;//

	ITLB1[3][0] = 0;
	ITLB1[3][1] = 0;
	ITLB1[3][2] = 0;
	ITLB1[3][3] = 1;

	/*
	//matrixmultiply4(TRWF, TRF1, TRW1);
	matrixmultiply4(TRWB, ITRB1, ITRW1);
	matrixmultiply4(ITRW1, ITR12, ITRW2);
	matrixmultiply4(ITRW2, ITR23, ITRW3);
	matrixmultiply4(ITRW3, ITR34, ITRW4);
	matrixmultiply4(ITRW4, ITR45, ITRW5);
	matrixmultiply4(ITRW5, ITR56, ITRW6);
	matrixmultiply4(ITRW6, ITR6E, ITRWE);

	//matrixmultiply4(TLWF, TLF12, TLW12);
	matrixmultiply4(TLWB, ITLB1, ITLW1);
	matrixmultiply4(ITLW1, ITL12, ITLW2);
	matrixmultiply4(ITLW2, ITL23, ITLW3);
	matrixmultiply4(ITLW3, ITL34, ITLW4);
	matrixmultiply4(ITLW4, ITL45, ITLW5);
	matrixmultiply4(ITLW5, ITL56, ITLW6);
	matrixmultiply4(ITLW6, ITL6E, ITLWE);
	*/


	matrixmultiply4(ITRB1, ITR12, ITRB2);
	matrixmultiply4(ITRB2, ITR23, ITRB3);
	matrixmultiply4(ITRB3, ITR34, ITRB4);
	matrixmultiply4(ITRB4, ITR45, ITRB5);
	matrixmultiply4(ITRB5, ITR56, ITRB6);
	matrixmultiply4(ITRB6, ITR6E, ITRBE);

	matrixmultiply4(ITLB1, ITL12, ITLB2);
	matrixmultiply4(ITLB2, ITL23, ITLB3);
	matrixmultiply4(ITLB3, ITL34, ITLB4);
	matrixmultiply4(ITLB4, ITL45, ITLB5);
	matrixmultiply4(ITLB5, ITL56, ITLB6);
	matrixmultiply4(ITLB6, ITL6E, ITLBE);
}

void inversekinematicsLtoB(double th[]) {
	// clock_t inverse_timer_start, inverse_timer_end;
	// inverse_timer_start = clock();
	double tempA[3] = { 0, };
	double tempB[3] = { 0, };
	double tempC[3] = { 0, };
	double diffp[3] = { 0, };
	//double jacobi[6][6] = { 0, };
	// double dq[6] = { 0, };
	// double a[6][6], d[6][6], deter;

	int i;
	// int n;
	// int print_matrix = 1;
	// int print_inverse = 2;


	//1
	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[11][i];
	}

	for (i = 0; i < 3; i++) {
		diffp[i] = ITLBE[i][3] - ITLB1[i][3];
	}

	cross(tempA, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][0] = tempB[i];
	}


	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][0] = _IA[11][i];
	}

	//2

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[10][i];
	}
	matrixmultiply331(ITLB1, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = ITLBE[i][3] - ITLB2[i][3];
	}
	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][1] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][1] = tempC[i];
	}


	//3

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[9][i];
	}
	matrixmultiply331(ITLB2, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = ITLBE[i][3] - ITLB3[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][2] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][2] = tempC[i];
	}

	//4
	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[8][i];
	}
	matrixmultiply331(ITLB3, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = ITLBE[i][3] - ITLB4[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][3] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][3] = tempC[i];
	}


	//5

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[7][i];
	}
	matrixmultiply331(ITLB4, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = ITLBE[i][3] - ITLB5[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][4] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][4] = tempC[i];
	}

	//6
	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[6][i];
	}
	matrixmultiply331(ITLB5, tempA, tempC);
	for (i = 0; i < 3; i++) {
		diffp[i] = ITLBE[i][3] - ITLB6[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][5] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][5] = tempC[i];
	}


	//matrixprint6(jacobi);
	/*
	if (inverse_type == 0)
	{
		n = cin_function(a, i_jacobi);   // read function
								//cout_function(a, n, print_matrix);   // read function
		deter = (double)det(a, n);   // read function
		inverse(a, d, n, deter);   // read function
	}
	else
	{
		//inverse_solved(jacobi, d);
	}
	//cout_function(d, n, print_inverse);   // read function

	errcalc(targetp, targetr, ITLBE, err);
	matrixmultiply661(d, err, dq);


	for (i = 0; i < 6; i++) {
		th[11 - i] = 0.5 * dq[i] + th[11 - i];
	}
	*/
	// inverse_timer_end = clock();
	// float inverse_timer_result = (float)(inverse_timer_end - inverse_timer_start) / CLOCKS_PER_SEC;
	// printf("LtoB calc time : %f\n",inverse_timer_result);

}


void inversekinematicsRtoB(double th[]) {
	// clock_t inverse_timer_start, inverse_timer_end;
	// inverse_timer_start = clock();
	double tempA[3] = { 0, };
	double tempB[3] = { 0, };
	double tempC[3] = { 0, };
	double diffp[3] = { 0, };
	//double jacobi[6][6] = { 0, };
	// double dq[6] = { 0, };

	// double a[6][6], d[6][6], deter;
	int i;
	// int n;
	// int print_matrix = 1;
	// int print_inverse = 2;


	//1

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[0][i];
	}


	for (i = 0; i < 3; i++) {
		diffp[i] = ITRBE[i][3] - ITRB1[i][3];
	}


	cross(tempA, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][0] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][0] = _IA[0][i];
	}

	//2
	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[1][i];
	}

	matrixmultiply331(ITRB1, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = ITRBE[i][3] - ITRB2[i][3];
	}

	cross(tempC, diffp, tempB);


	for (i = 0; i < 3; i++) {
		i_jacobi[i][1] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][1] = tempC[i];
	}


	//3

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[2][i];
	}
	matrixmultiply331(ITRB2, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = ITRBE[i][3] - ITRB3[i][3];
	}

	cross(tempC, diffp, tempB);


	for (i = 0; i < 3; i++) {
		i_jacobi[i][2] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][2] = tempC[i];
	}


	//4

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[3][i];
	}
	matrixmultiply331(ITRB3, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = ITRBE[i][3] - ITRB4[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][3] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][3] = tempC[i];
	}

	//5
	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[4][i];
	}
	matrixmultiply331(ITRB4, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = ITRBE[i][3] - ITRB5[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][4] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][4] = tempC[i];
	}


	//6

	for (i = 0; i < 3; i++) {
		tempA[i] = _IA[5][i];
	}
	matrixmultiply331(ITRB5, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = ITRBE[i][3] - ITRB6[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		i_jacobi[i][5] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		i_jacobi[3 + i][5] = tempC[i];
	}


	/*
	//matrixprint6(jacobi);

	if (inverse_type == 0)
	{
		n = cin_function(a, i_jacobi);   // read function
								//cout_function(a, n, print_matrix);   // read function
		deter = (double)det(a, n);   // read function
		inverse(a, d, n, deter);   // read function
	}

	else
	{
		//inverse_solved(jacobi, d);
	}
	//cout_function(d, n, print_inverse);   // read function

	errcalc(targetp, targetr, ITRBE, err);
	matrixmultiply661(d, err, dq);

	for (i = 0; i < 6; i++) {
		th[i] = 0.5 * dq[i] + th[i];
	}
	*/
	// inverse_timer_end = clock();
	// float inverse_timer_result = (float)(inverse_timer_end - inverse_timer_start) / CLOCKS_PER_SEC;
	// printf("RtoB calc time : %f\n",inverse_timer_result);

}




//////////////////////2860 - 3124 : 각가속도 구하는 것 때매 자코비안만 구하는 함수 새로 구현


void newjacobiBtoL(double th[]) {

	double tempA[3] = { 0, };
	double tempB[3] = { 0, };
	double tempC[3] = { 0, };
	double diffp[3] = { 0, };

	//double a[6][6], d[6][6], deter;
	int i;


	//1
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[6][i];
	}
 
	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW1[i][3];
	}

	cross(tempA, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][0] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][0] = _A[6][i];
	}

	//2

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[7][i];
	}
	matrixmultiply331(TLW1, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW2[i][3];
	}

	cross(tempC, diffp, tempB);


	for (i = 0; i < 3; i++) {
		_jacobi_n[i][1] = tempB[i];
	}

	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][1] = tempC[i];
	}

	//3

	for (i = 0; i < 3; i++) {
		tempA[i] = _A[8][i];
	}
	matrixmultiply331(TLW2, tempA, tempC);


	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW3[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][2] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][2] = tempC[i];
	}


	//4
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[9][i];
	}
	matrixmultiply331(TLW3, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW4[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][3] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][3] = tempC[i];
	}

	//5
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[10][i];
	}
	matrixmultiply331(TLW4, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW5[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][4] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][4] = tempC[i];
	}
	//6
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[11][i];
	}
	matrixmultiply331(TLW5, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TLWE[i][3] - TLW6[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][5] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][5] = tempC[i];
	}


	
	
}

void newjacobiBtoR(double th[]) {
	double tempA[3] = { 0, };
	double tempB[3] = { 0, };
	double tempC[3] = { 0, };
	double diffp[3] = { 0, };
	//double jacobi[6][6] = { 0, };
	//double dq[6] = { 0, };

	//double a[6][6], d[6][6], deter;
	int i;
	//int n;
	//int print_matrix = 1;
	//int print_inverse = 2;

	//1
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[5][i];
	}

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW1[i][3];
	}

	cross(tempA, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][0] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][0] = _A[5][i];
	}

	//2
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[4][i];
	}
	matrixmultiply331(TRW1, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW2[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][1] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][1] = tempC[i];
	}

	//3
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[3][i];
	}
	matrixmultiply331(TRW2, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW3[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][2] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][2] = tempC[i];
	}

	//4
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[2][i];
	}
	matrixmultiply331(TRW3, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW4[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][3] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][3] = tempC[i];
	}

	//5
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[1][i];
	}
	matrixmultiply331(TRW4, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW5[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][4] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][4] = tempC[i];
	}
	//6
	for (i = 0; i < 3; i++) {
		tempA[i] = _A[0][i];
	}
	matrixmultiply331(TRW5, tempA, tempC);

	for (i = 0; i < 3; i++) {
		diffp[i] = TRWE[i][3] - TRW6[i][3];
	}

	cross(tempC, diffp, tempB);

	for (i = 0; i < 3; i++) {
		_jacobi_n[i][5] = tempB[i];
	}
	for (i = 0; i < 3; i++) {
		_jacobi_n[3 + i][5] = tempC[i];
	}

}

////////////////////////////////////////////////////////////////////////////////////////////////
/*
void Walk::FsrSensing(int num, int phase, double _px, double _py, double _ppx, double _ppy)
{
// if(time > 3*samplingtime)
// {

const double *FSR1_R = mFSR1r->getValues();
const double *FSR2_R = mFSR2r->getValues();
const double *FSR3_R = mFSR3r->getValues();
const double *FSR4_R = mFSR4r->getValues();

double FSR1r = FSR1_R[1] / 10;
double FSR2r = FSR2_R[1] / 10;
double FSR3r = FSR3_R[1] / 10;
double FSR4r = FSR4_R[1] / 10;
//double FSRr = FSR1r + FSR2r + FSR3r + FSR4r;

const double *FSR1_L = mFSR1l->getValues();
const double *FSR2_L = mFSR2l->getValues();
const double *FSR3_L = mFSR3l->getValues();
const double *FSR4_L = mFSR4l->getValues();

double FSR1l = FSR1_L[1] / 10;
double FSR2l = FSR2_L[1] / 10;
double FSR3l = FSR3_L[1] / 10;
double FSR4l = FSR4_L[1] / 10;
//double FSRl = FSR1l + FSR2l + FSR3l + FSR4l;

//if (sWPG.nCommandState % 2 == 0)
sumofzmp_r = FSR1r + FSR2r + FSR3r + FSR4r;
//else
sumofzmp_l = FSR1l + FSR2l + FSR3l + FSR4l;
// printf("%lf, %lf#######################\n",sumofzmp_r, sumofzmp_l);

force[0] = FSR1l;
force[1] = FSR2l;
force[2] = FSR3l;
force[3] = FSR4l;

force[4] = FSR1r;
force[5] = FSR2r;
force[6] = FSR3r;
force[7] = FSR4r;





zmp[0] = 0;
zmp[1] = 0;
zmp_global[0]=0;
zmp_global[1]=0;
force_sum = 0;


// printf("pre_local_stride_L= %lf     pre_local_stride_F= %lf _px=%lf   _py=%lf \n", pre_local_stride_L, pre_local_stride_F, _px, _py);
// printf("_lsx_des = %lf    _lsy_des = %lf _rsy_des = %lf _rsy_des = %lf \n", _lsx_des,_lsy_des, _rsy_des, _rsy_des);
if (phase == 2) // DSP_R
{

// printf("phase=2222222222222222222222\n");
pos_x[0] = + 0.04 - pre_local_stride_F;
pos_x[1] = - 0.04 - pre_local_stride_F;
pos_x[2] = - 0.04 - pre_local_stride_F;
pos_x[3] = + 0.04 - pre_local_stride_F;
// 0 ~ 3 : left
pos_x[4] = + 0.04;
pos_x[5] = - 0.04;
pos_x[6] = - 0.04;
pos_x[7] = + 0.04;
// 4 ~ 7 : right
pos_y[0] = + 0.027 + 0.0125 + pre_local_stride_L;// +2.1; // 1.0
pos_y[1] = + 0.027 + 0.0125 + pre_local_stride_L;// +2.1; // 1.0
pos_y[2] = - 0.027 + 0.0125 + pre_local_stride_L;// +7.45;//6.35;
pos_y[3] = - 0.027 + 0.0125 + pre_local_stride_L;// +7.45;//6.35;

pos_y[4] = + 0.027 - 0.0125;
pos_y[5] = + 0.027 - 0.0125;
pos_y[6] = - 0.027 - 0.0125;//6.35;
pos_y[7] = - 0.027 - 0.0125;//6.35;

//printf("foot_boundary_x : %lf , FS : %lf\n", foot_boundary_x, FS);

for (int i = 0; i < 8; i++) {
force_sum = force_sum + force[i];
zmp[0] = zmp[0] + force[i] * pos_x[i];
zmp[1] = zmp[1] + force[i] * pos_y[i];
}



}

else if (phase == 4) // DSP_L
{
// printf("phase=444444444444444444444444444444\n");
pos_x[0] = +0.04;
pos_x[1] = -0.04;
pos_x[2] = -0.04;
pos_x[3] = +0.04;
// 0 ~ 3 : left
pos_x[4] = + 0.04 - pre_local_stride_F;
pos_x[5] = - 0.04 - pre_local_stride_F;
pos_x[6] = - 0.04 - pre_local_stride_F;
pos_x[7] = + 0.04 - pre_local_stride_F;
// 4 ~ 7 : right
pos_y[0] = + 0.027 + 0.0125;
pos_y[1] = + 0.027 + 0.0125;
pos_y[2] = - 0.027 + 0.0125;
pos_y[3] = - 0.027 + 0.0125;

pos_y[4] = +0.027 - 0.0125 - pre_local_stride_L;
pos_y[5] = +0.027 - 0.0125 - pre_local_stride_L;
pos_y[6] = -0.027 - 0.0125 - pre_local_stride_L;
pos_y[7] = -0.027 - 0.0125 - pre_local_stride_L;


for (int i = 0; i < 8; i++) {
force_sum = force_sum + force[i];
zmp[0] = zmp[0] + force[i] * pos_x[i];
zmp[1] = zmp[1] + force[i] * pos_y[i];
}



}

else if (phase == 1) { // SSP_R
// printf("phase=111111111111111111111111111111111111\n");
pos_x[0] = +0.04 + (_lsx_des - _px); // +local_stride_F
pos_x[1] = -0.04 + (_lsx_des - _px);
pos_x[2] = -0.04 + (_lsx_des - _px);
pos_x[3] = +0.04 + (_lsx_des - _px);
// 0 ~ 3 : left
pos_x[4] = +0.04;
pos_x[5] = -0.04;
pos_x[6] = -0.04;
pos_x[7] = +0.04;
// 4 ~ 7 : right
pos_y[0] = +0.027 + 0.0125 + (_lsy_des - _py); // +local_stride_L
pos_y[1] = +0.027 + 0.0125 + (_lsy_des - _py);
pos_y[2] = -0.027 + 0.0125 + (_lsy_des - _py);
pos_y[3] = -0.027 + 0.0125 + (_lsy_des - _py);

pos_y[4] = +0.027 - 0.0125;
pos_y[5] = +0.027 - 0.0125;
pos_y[6] = -0.027 - 0.0125;
pos_y[7] = -0.027 - 0.0125;


for (int i = 0; i < 8; i++) {
force_sum = force_sum + force[i];
zmp[0] = zmp[0] + force[i] * pos_x[i];
zmp[1] = zmp[1] + force[i] * pos_y[i];
}

}

else if (phase == 3) { // SSP_L
// printf("phase=33333333333333333333333333333333333333333333333\n");
// printf(" _rsx_des = %lf _rsy_des = %lf \n", _rsx_des, _rsy_des);
pos_x[0] = +0.04;
pos_x[1] = -0.04;
pos_x[2] = -0.04;
pos_x[3] = +0.04;
// 0 ~ 3 : left
pos_x[4] = +0.04 + (_rsx_des - _px); // +local_stride_F
pos_x[5] = -0.04 + (_rsx_des - _px);
pos_x[6] = -0.04 + (_rsx_des - _px);
pos_x[7] = +0.04 + (_rsx_des - _px);
// 4 ~ 7 : right
pos_y[0] = +0.027 + 0.0125;
pos_y[1] = +0.027 + 0.0125;
pos_y[2] = -0.027 + 0.0125;
pos_y[3] = -0.027 + 0.0125;

pos_y[4] = +0.027 - 0.0125 + (_rsy_des - _py); // -local_stride_L
pos_y[5] = +0.027 - 0.0125 + (_rsy_des - _py);
pos_y[6] = -0.027 - 0.0125 + (_rsy_des - _py);
pos_y[7] = -0.027 - 0.0125 + (_rsy_des - _py);
// pos_y[4] = +0.027 - 0.0125 -_rsy_des ; // -local_stride_L
// pos_y[5] = +0.027 - 0.0125 -_rsy_des ;
// pos_y[6] = -0.027 - 0.0125 -_rsy_des ;
// pos_y[7] = -0.027 - 0.0125 -_rsy_des ;


for (int i = 0; i < 8; i++) {
force_sum = force_sum + force[i];
zmp[0] = zmp[0] + force[i] * pos_x[i];
zmp[1] = zmp[1] + force[i] * pos_y[i];
}

//printf("\n---------aaa%lfaaaa---------------\n", local_stride_L);
//zmp[1] = zmp[1] + local_stride_L / 2;


}

zmp[0] = zmp[0] / force_sum;
zmp[1] = zmp[1] / force_sum;
zmp_global[0]=zmp[0]+_px;
zmp_global[1]=zmp[1]+_py;
if (force_sum < 0.0000001) {
zmp[0] = 0.0; zmp[1] = 0.0;
}


//printf("zmp[0] : %lf , zmp[1] : %lf\n", zmp[0], zmp[1]);
// }

}
*/
void LPF(double fc, double pre[], double pre_f[], double result[]) {

	double lamda;
	double alpha;

	lamda = 2 * (3.14) * fc;

	alpha = (lamda * samplingtime) / (1 + lamda * samplingtime);

	result[0] = (alpha) * (pre[0]) + (1 - alpha) * (pre_f[0]);
	result[1] = (alpha) * (pre[1]) + (1 - alpha) * (pre_f[1]);
	result[2] = (alpha) * (pre[2]) + (1 - alpha) * (pre_f[2]);

}

void highpoly(double th_i[],double th_f[],double dth_i[],double dth_f[],double ddth_i[],double ddth_f[], double tf, double poly_t, double result[]) {
	double a0[12],a1[12],a2[12],a3[12],a4[12],a5[12];
	for(int i=0;i<12;)
    {
		   
		a0[i] = th_i[i];
	    a1[i] = dth_i[i];
	    a2[i] = ddth_i[i]/2;
	    a3[i] = (20*th_f[i]-20*th_i[i]-(8*dth_f[i]+12*dth_i[i])*tf-(3*ddth_i[i]-ddth_f[i])*pow(tf,2))/(2*pow(tf,3));
	    a4[i] = (30*th_i[i]-30*th_f[i]+(14*dth_f[i]+16*dth_i[i])*tf+(3*ddth_i[i]-2*ddth_f[i])*pow(tf,2))/(2*pow(tf,4));
	    a5[i] = (12*th_f[i]-12*th_i[i]-(6*dth_f[i]+6*dth_i[i])*tf-(ddth_i[i]-ddth_f[i])*pow(tf,2))/(2*pow(tf,5));

	    result[i] = a0[i] + a1[i]*poly_t + a2[i]* pow(poly_t,2) + a3[i]*pow(poly_t,3) + a4[i]*pow(poly_t,4) + a5[i]*pow(poly_t,5);

	    i++;

    }
}

void protectZmpEscape(int phase)
{
	double sparex=0.03;
	double sparey=0.02;
	if(phase==1)//오른발 지지
	{
		if(ZMP_d[0] >= (foot_boundary_x/2 - sparex))ZMP_d[0]=(foot_boundary_x/2 - sparex);
		else if(ZMP_d[0] <= (-foot_boundary_x/2 + sparex))ZMP_d[0]=(-foot_boundary_x/2 + sparex);
		if(ZMP_d[1]>=((foot_boundary_y/2)+foot_offset - sparey))ZMP_d[1]=((foot_boundary_y/2)+foot_offset - sparey);
		else if(ZMP_d[1]<=(-(foot_boundary_y/2)+foot_offset))ZMP_d[1]=(-(foot_boundary_y/2)+foot_offset);
	}
	else if(phase==3)//왼발 지지
	{
		if(ZMP_d[0] >= (foot_boundary_x/2 - sparex))ZMP_d[0]=(foot_boundary_x/2 - sparex);
		else if(ZMP_d[0] <= (-foot_boundary_x/2 + sparex))ZMP_d[0]=(-foot_boundary_x/2 + sparex);
		if(ZMP_d[1]>=((foot_boundary_y/2)-foot_offset))ZMP_d[1]=((foot_boundary_y/2)+foot_offset);
		else if(ZMP_d[1]<=(-(foot_boundary_y/2)-foot_offset + sparey))ZMP_d[1]=(-(foot_boundary_y/2)-foot_offset + sparey);
	}
}

void DCM_ver3(int axis,double gain, double com_nominal, double vcom_nominal, double com_cur, double vcom_cur, double coordinate,int _i,double localtime )
{
	CP_ref[axis]=com_nominal + _Tc * vcom_nominal;
	CP_cur[axis]=(com_cur - coordinate) + _Tc * vcom_cur;
	ZMP_d[axis] = ZMP_c[axis] + (1 + gain*sqrt(_zc / _g)) * (CP_cur[axis] -CP_ref[axis]);
	CP_err[axis] = CP_cur[axis] - CP_ref[axis];
	if(_i<cptOn)
	{
		ZMP_d[axis] = ZMP_c[axis] ;
	}
#if localSSPTimeLimitOn
	if(localtime>localSSPTimeLimit)
	{	
#if debugPrintOn
		printf("not control");
#endif
		ZMP_d[axis] = ZMP_c[axis] ;
	}
#endif
	if(gain==0)
	{
		ZMP_d[axis] = 0;
	}
#if (cp_err_limit_on==1)
	if( CP_err[axis] < cp_cur_limit)
	{
		ZMP_d[0] = ZMP_c[0] ;
		ZMP_d[1] = ZMP_c[1] ;						
	}
#endif
#if debugPrintOn
	printf("\n localtime=%lf",localtime);
	printf("\n CP_err[%d]=%lf, Pd[%d]=%lf",axis,CP_err[axis],axis,ZMP_d[axis]);
#endif
}

void CPT_ver2_ssp(int phase,double localtime,int _i,double xgain,double ygain,double rot_th, double _x_ter, double _vx_ter, double _y_ter, double _vy_ter, double xcom_cur,double vxcom_cur, double ycom_cur,double vycom_cur,double _px,double _py)
{
	if(phase==1 || phase==3)//ssp
	{
		double xi = cos(rot_th)*(-_x_ter * cosh(0.0 / _Tc) + _Tc * _vx_ter * sinh(0.0 / _Tc)) - sin(rot_th) * (_y_ter * cosh(0.0 / _Tc) - _Tc * _vy_ter * sinh(0.0 / _Tc));
		double vxi = (-_x_ter / _Tc) * sinh(0.0 / _Tc) + _vx_ter * cosh(0.0 / _Tc);
		double xcom_init = xi * cosh(0.0 / _Tc) + _Tc * vxi * sinh(0.0 / _Tc);
		double vxcom_init = xi / _Tc * sinh(0.0 / _Tc) + vxi * cosh(0.0 / _Tc);
		double yi = (_y_ter * cosh(0.0 / _Tc) - _Tc * _vy_ter * sinh(0.0 / _Tc));
		double vyi = (_y_ter / _Tc) * sinh(0.0 / _Tc) - _vy_ter * cosh(0.0 / _Tc);
		double ycom_init = yi * cosh(0.0 / _Tc) + _Tc * vyi * sinh(0.0 / _Tc);
		double vycom_init = (yi / _Tc * sinh(0.0 / _Tc) + vyi * cosh(0.0 / _Tc));
		CP_init[0] = xi + vxi * _Tc;
		CP_init[1] = yi + vyi * _Tc;
		CP_cur[0] = (xcom_cur - _px) + _Tc * (vxcom_cur);
		CP_cur[1] = (ycom_cur - _py) + _Tc * (vycom_cur);
		CP_ref[0] = exp(localtime / _Tc)*CP_init[0];
		CP_ref[1] = exp(localtime / _Tc)*CP_init[1];
		CP_err[0] = (CP_cur[0] - CP_ref[0]);
		CP_err[1] = (CP_cur[1] - CP_ref[1]);
		if(_i<cptOn)
		{
			xgain=0;
			ygain=0;
		}
		else{
			xgain=Kf_S_origin[0];
			ygain=Kf_S_origin[1];	
		}
		ZMP_c[0] = 0.0;
		ZMP_c[1] = 0.0;
		if(cpZmpCurrentOn==1)
		{
			if(phase==1)
			{
				ZMP_c[0] = tq_yr_p1_pos/f_zr_p1_pos;
				ZMP_c[1] = tq_xr_p1_pos/f_zr_p1_pos;
				// printf("\ncpZmpCurrentOn : ZMP_c[0]=%lf,ZMP_c[1]=%lf_lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",ZMP_c[0],ZMP_c[1],_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);
			}
			else if(phase==3)//왼발지지
			{
				ZMP_c[0] = tq_yl_p0_pos/f_zl_p0_pos;
				ZMP_c[1] = tq_xl_p0_pos/f_zl_p0_pos;
				// printf("\ncpZmpCurrentOn : ZMP_c[0]=%lf,ZMP_c[1]=%lf_lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",ZMP_c[0],ZMP_c[1],_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);
			}
			
		}
		// ZMP_d[0] = (CP_ref[0] - (exp(samplingtime / _Tc)*CP_cur[0])) / (1 - exp(samplingtime / _Tc));
		// ZMP_d[1] = (CP_ref[1] - (exp(samplingtime / _Tc)*CP_cur[1])) / (1 - exp(samplingtime / _Tc));
		// a_desired[0] = Kf_S[0] * ((_m*_g) / _zc) * (ZMP_c[0] - ZMP_d[0]);
		// a_desired[1] = Kf_S[1] * ((_m*_g) / _zc) * (ZMP_c[1] - ZMP_d[1]);
		a_desired[0] = (_m*_g / _zc) * ( - (1 + xgain*sqrt(_zc / _g))*(CP_cur[0] - CP_ref[0]));
		a_desired[1] = (_m*_g / _zc) * ( - (1 + ygain*sqrt(_zc / _g))*(CP_cur[1] - CP_ref[1]));
		if(xgain || ygain)
		{
			a_desired[0]=0;
			a_desired[1]=0;
		}
	}
}

void cubicPolynomialInterpolationInitForDsp(double dsp_x_i,double dsp_x_f,double dsp_dx_i,double dsp_dx_f,double dsp_y_i,double dsp_y_f,double dsp_dy_i,double dsp_dy_f,double time_f,int order, int nominalOn)
{
	if(order==3)
	{
		if(nominalOn==0)
		{
			dsp_x_a0 = dsp_x_i;
			dsp_x_a1 = dsp_dx_i;
			dsp_x_a2 = 3 / pow(time_f, 2) * (dsp_x_f - dsp_x_i) - 2 / time_f * dsp_dx_i - dsp_dx_f / time_f;
			dsp_x_a3 = -2 / pow(time_f, 3) * (dsp_x_f - dsp_x_i) + 1 / pow(time_f, 2) * (dsp_dx_f + dsp_dx_i);
			dsp_y_a0 = dsp_y_i;
			dsp_y_a1 = dsp_dy_i;
			dsp_y_a2 = 3 / pow(time_f, 2) * (dsp_y_f - dsp_y_i) - 2 / time_f * dsp_dy_i - dsp_dy_f / time_f;
			dsp_y_a3 = -2 / pow(time_f, 3) * (dsp_y_f - dsp_y_i) + 1 / pow(time_f, 2) * (dsp_dy_f + dsp_dy_i);
		}
		else if(nominalOn==1)
		{
			dsp_x_a0_nominal = dsp_x_i;
			dsp_x_a1_nominal = dsp_dx_i;
			dsp_x_a2_nominal = 3 / pow(time_f, 2) * (dsp_x_f - dsp_x_i) - 2 / time_f * dsp_dx_i - dsp_dx_f / time_f;
			dsp_x_a3_nominal = -2 / pow(time_f, 3) * (dsp_x_f - dsp_x_i) + 1 / pow(time_f, 2) * (dsp_dx_f + dsp_dx_i);
			dsp_y_a0_nominal = dsp_y_i;
			dsp_y_a1_nominal = dsp_dy_i;
			dsp_y_a2_nominal = 3 / pow(time_f, 2) * (dsp_y_f - dsp_y_i) - 2 / time_f * dsp_dy_i - dsp_dy_f / time_f;
			dsp_y_a3_nominal = -2 / pow(time_f, 3) * (dsp_y_f - dsp_y_i) + 1 / pow(time_f, 2) * (dsp_dy_f + dsp_dy_i);
		}
		
	}
	else if(order==5)
	{
		ndsp_x_a0=dsp_x_i;
		ndsp_x_a1=dsp_dx_i;
		ndsp_x_a2=0;
		ndsp_x_a3=(20*(dsp_x_f - dsp_x_i)-(8*dsp_dx_f+12*dsp_dx_i)* time_f)/(2*pow(time_f, 3));
		ndsp_x_a4=(30*(dsp_x_i - dsp_x_f)+(14*dsp_dx_f+16*dsp_dx_i)*time_f)/(2*pow(time_f, 4));
		ndsp_x_a5=(12*(dsp_x_f - dsp_x_i)-(6*dsp_dx_f+6*dsp_dx_i)*time_f)/(2*pow(time_f, 5));
		ndsp_y_a0=dsp_y_i;
		ndsp_y_a1=dsp_dy_i;
		ndsp_y_a2=0;
		ndsp_y_a3=(20*(dsp_y_f - dsp_y_i)-(8*dsp_dy_f+12*dsp_dy_i)* time_f)/(2*pow(time_f, 3));
		ndsp_y_a4=(30*(dsp_y_i - dsp_y_f)+(14*dsp_dy_f+16*dsp_dy_i)*time_f)/(2*pow(time_f, 4));
		ndsp_y_a5=(12*(dsp_y_f - dsp_y_i)-(6*dsp_dy_f+6*dsp_dy_i)*time_f)/(2*pow(time_f, 5));
	}
}


Walk::Walk() : Robot() {
	mTimeStep = getBasicTimeStep();
	

	mAccelerometer = getAccelerometer("Accelerometer");
	mAccelerometer->enable(mTimeStep);
	AccR = getAccelerometer("accR");
	AccR -> enable(mTimeStep);
	AccL = getAccelerometer("accL");
	AccL -> enable(mTimeStep);
	
	mAccelerometer->enable(mTimeStep);
	

	mCompassR = getCompass("compassR");
	mCompassR->enable(samplingPeriod);
	mGPSR = getGPS("gpsR");
	mGPSR->enable(samplingPeriod);

	mCompassL = getCompass("compassL");
	mCompassL->enable(samplingPeriod);
	mGPSL = getGPS("gpsL");
	mGPSL->enable(samplingPeriod);
	mCompassC=getCompass("compassC");
	mCompassC->enable(samplingPeriod);



	/// ////////////////////////////////////////////////////

	// mFSR1r = getTouchSensor("FSR1r");
	// mFSR2r = getTouchSensor("FSR2r");
	// mFSR3r = getTouchSensor("FSR3r");
	// mFSR4r = getTouchSensor("FSR4r");
	// mFSR1r->enable(mTimeStep);
	// mFSR2r->enable(mTimeStep);
	// mFSR3r->enable(mTimeStep);
	// mFSR4r->enable(mTimeStep);

	// mFSR1l = getTouchSensor("FSR1l");
	// mFSR2l = getTouchSensor("FSR2l");
	// mFSR3l = getTouchSensor("FSR3l");
	// mFSR4l = getTouchSensor("FSR4l");
	// mFSR1l->enable(mTimeStep);
	// mFSR2l->enable(mTimeStep);
	// mFSR3l->enable(mTimeStep);
	// mFSR4l->enable(mTimeStep);

	/// ////////////////////////////////////////////////////////
	for (int i = 0; i < NMOTORS; i++) {
		mMotors[i] = getMotor(motorNames[i]);
		string sensorName = motorNames[i];
		sensorName.push_back('S');
		mPositionSensors[i] = getPositionSensor(sensorName);
		mPositionSensors[i]->enable(mTimeStep);
	}

	for (int i = 0; i < FTMOTORS; i++) {
		ftMotors[i] = getMotor(ftmotorNames[i]);
	}

	ftMotors[0]->enableTorqueFeedback(samplingPeriod);
	ftMotors[1]->enableTorqueFeedback(samplingPeriod);
	ftMotors[2]->enableTorqueFeedback(samplingPeriod);
	ftMotors[3]->enableTorqueFeedback(samplingPeriod);
	ftMotors[4]->enableTorqueFeedback(samplingPeriod);
	ftMotors[5]->enableTorqueFeedback(samplingPeriod);

	// mMotionManager = new RobotisOp2MotionManager(this);
	// mGaitManager = new RobotisOp2GaitManager(this, "config.ini");
	mFlu = getTouchSensor("FSRlu");
	mFld = getTouchSensor("FSRld");
	mFru = getTouchSensor("FSRru");
	mFrd = getTouchSensor("FSRrd");

	mFlu->enable(mTimeStep);
	mFld->enable(mTimeStep);
	mFru->enable(mTimeStep);
	mFrd->enable(mTimeStep);
}

Walk::~Walk() {
}

void Walk::myStep() {
	int ret = step(mTimeStep);
	if (ret == -1)
		exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
	double startTime = getTime();
	double ss = (double)ms / 1000.0;
	while (ss + startTime >= getTime())
		myStep();
}

// function containing the main feedback loop
void Walk::run() {
	const double *acc = mAccelerometer->getValues();
	const double *accR = AccR->getValues();
	const double *accL = AccL->getValues();
	const double *gpsr = mGPSR->getValues();
	const double *gpsl = mGPSL->getValues();
	// const char gpscoordinateR = mGPSR->getCoordinateSystem();
	// const char gpscoordinateL = mGPSL->getCoordinateSystem();
	const double *compassr = mCompassR->getValues();
	const double *compassl = mCompassL->getValues();
	const double *compassc = mCompassC->getValues();

	// printf("\n%c#############%c @@@@@@@@@@@@@@@@", gpscoordinateR, gpscoordinateL);

	double gpsrx = gpsr[0];
	double gpsry = gpsr[1];
	double gpsrz = gpsr[2];
	double gpslx = gpsl[0];
	double gpsly = gpsl[1];
	double gpslz = gpsl[2];
	double local_gpsrx=0;
	double local_gpsrz=0;
	double local_gpsrx_ssp_begin=0;
	double local_gpsrz_ssp_begin=0;
	
	double local_gpslx_ssp_begin=0;
	double local_gpslz_ssp_begin=0;
	
	double compassrx = compassr[0];
	double compassry = compassr[1];
	double compassrz = compassr[2];
	double compasslx = compassl[0];
	double compassly = compassl[1];
	double compasslz = compassl[2];
	double compasscx = compassc[0];
	double compasscy = compassc[1];
	double compasscz = compassc[2];
	double footradR = 0, footradL = 0;
    double footdegreeR = 0, footdegreeL=0;
	double comRad=0, comDegree=0;
	double localFootDegree=0;
	double localFootDegreeRSspI=0, localFootDegreeLSspI=0;
	

	double gpsrxoffset = 0, gpsryoffset = 0, gpsrzoffset = 0;
	double gpslxoffset = 0, gpslyoffset = 0, gpslzoffset = 0;
	double sspgpsrz=0, _sspgpsrz=0;
    double sspgpsrx=0, _sspgpsrx=0;
	double footdegreeRoffset = 0, footdegreeLoffset = 0,comdegreeOffset=0;
    double sspdegreeR=0, _sspdegreeR=0;

	double _xcom_acc_measure = acc[0];
	double _ycom_acc_measure = acc[1];
	double _zcom_acc_measure = acc[2];
	
	double footRX_acc = accR[0];
	double footRY_acc = accR[1];
	double footRZ_acc = accR[2];
	double footLX_acc = accL[0];
	double footLY_acc = accL[1];
	double footLZ_acc = accL[2];
	
	double torquexr = 0;
	double torqueyr = 0;
	double torquezr = 0;
	double torquexl = 0;
	double torqueyl = 0;
	double torquezl = 0;
////////////////////////////control pannel//////////////FSLIP
    int fslipControlOn=0;
    int controlswitchYR=0, controlswitchXR=0;
	int controlswitchYL=0, controlswitchXL=0;
	int dspDecelerator=0, DSPDelFslipNoiseOn=1;
	int dspFSlipControl=0, LPForder=1;
	int sspYFslipControlOn=4, dspFslipControlOn=4;
	int sspXFslipControlOn=4,dspDeceleratorOn=4;//need stepconmand
	double dspSlowTime=0.25, dspConstant=3, dspOrigin=0;
    double fslipKy=0.1, fslipKx=0.1;
	double fslipKyDsp=0.1, fslipKxDsp=0.1;
	int fslipAddFoot=1, sspStartInit=0;//1이면 초기화
	int onlySSPon=0;
	double onlySSPTime=0.4;
////////////////////////////control pannel//////////////END
    int fslipInitCheck=0;
	double fslipy_print=0, fslipx_print=0;
    double frefY_print=0,frefX_print=0;
	int DSPDelCheck=0; 
	double fslipy = 0, fslipx = 0;
	double pre_fslipy=0, pre_fslipx=0;
	double footLYAccLpf=0, footRYAccLpf=0;
	double lamda;
	double alpha;
	double fc=3, fcFoot=10, Qfactor=0.707;
	double lpf2_gainH=1;
	lamda=2*3.14*fc;
	alpha = (lamda * samplingtime) / (1 + lamda * samplingtime);
	double _xcom_c = 0,  _ycom_c = 0;
	double _ycom_cv = 0, _xcom_cv=0;
	double pre_ycom_c = 0, pre_xcom_c=0;
	double pre_ycom_cv = 0, pre_xcom_cv=0;
	double  _ycom_fslip = 0,_xcom_fslip = 0;
    double dsp_dy_i_c=0;
    double dsp_dx_i_c=0;
    double dsp_y_i_c = 0;
    double dsp_x_i_c = 0;
    double dspYICGlobal=0;
    double dspXICGlobal=0;
	double M = 3.17;
	double pre_xcom;
	double pre_ycom;
	double yComVFinalPrint=0;
	/////////////////////for cpt/////////////////////
	double pre_ssp_cp[2] ={0,};       //global CP    

	double CP_cur_global_ssp[2] ={0,};       //global CP    
	double CP_cur_global_dsp[2]={0,};
	double CP_ref_global_ssp[2] ={0,};       //global CP   
	double CP_ref_global_dsp[2]={0,}; 
         // initial local CP
	double CP_sspinit[2]={0,};            //ssp initial local CP

	double CP_E[2] = { 0, };           //Encoder local CP

	double ZMP_global[2]={0,};      //current global ZMP
	double ZMP_global_d[2]={0,};      //current global ZMP

	double localtime;

	double xi, vxi , xcom_init , vxcom_init;
	double yi, vyi , ycom_init , vycom_init;
    double CPx_ddot=0, CPy_ddot=0;
    double CPx_c=0, CPy_c=0;
    double CPx_cv=0, CPy_cv=0;
    double pre_CPx_ddot=0, pre_CPy_ddot=0;
    double pre_CPx_c=0, pre_CPy_c=0;
    double pre_CPx_cv=0, pre_CPy_cv=0;
    double _xcom_CPT=0, _ycom_CPT=0;
    double _xcom_CPT_global=0, _ycom_CPT_global=0;
    double _xcom_CPT_v=0, _ycom_CPT_v=0;

	double _targetxcom;
	double _targetycom;
	double _xcom;
	double _ycom;
	double _vxcom;
	double _vycom;
	double _xcom_nominal;
	double _ycom_nominal;
	double _vxcom_nominal;
	double _vycom_nominal;
	double _xcom_origin;
	double _ycom_origin;
	double _xcom_origin_global;
	double _ycom_origin_global;
	double _xcom_v_origin;
	double _ycom_v_origin;
	double _xcom_nominal_global;
	double _ycom_nominal_global;
	double _xcom_v = 0;
	double _ycom_v = 0;
	double _xcom_v_nominal = 0;
	double _ycom_v_nominal = 0;
	double _xcom_acc = 0;
	double _ycom_acc = 0;
	const double *F_Lu = mFlu->getValues();
	const double *F_Ld = mFld->getValues();
	const double *F_Ru = mFru->getValues();
	const double *F_Rd = mFrd->getValues();

	double Frxu = F_Ru[0];
	double Fryu = F_Ru[1];
	double Frzu = F_Ru[2];
	double Frxd = F_Rd[2];
	double Fryd = F_Rd[1];
	double Frzd = F_Rd[0];
	double Flxu = F_Lu[0];
	double Flyu = F_Lu[1];
	double Flzu = F_Lu[2];
	double Flxd = F_Ld[2];
	double Flyd = F_Ld[1];
	double Flzd = F_Ld[0];
	double FXl = 0, FYl = 0, FZl = 0, FXr = 0, FYr = 0, FZr = 0;

	double forceZr;
	double forceZl;
	double forceXr;
	double forceXl;
	double forceYr;
	double forceYl;
	double ftzmpxr = 0, ftzmpyr = 0, ftzmpxl = 0, ftzmpyl = 0, ftzmpx = 0, ftzmpy = 0;
	double fgroundX = 0, fgroundY = 0, fswitchZ = 0;

	double force_R[3], force_L[3];
	double torque_R[3], torque_L[3];

	double pre_force_R[3], pre_torque_R[3];
	double pre_force_L[3], pre_torque_L[3];

	double pre_force_F_R[3], pre_torque_F_R[3];
	double pre_force_F_L[3], pre_torque_F_L[3];
	
	double force_F_R[3], torque_F_R[3];
	double force_F_L[3], torque_F_L[3];

	//int i = 0;
	//int j = 0;
	double angle[12] = {};
    double err_th1 = 0;
    double err_th2 = 0;
    double err_th3 = 0;
    double err_th4 = 0;
    double err_th5 = 0;
    double err_th6 = 0;
    double err_th7 = 0;
    double err_th8 = 0;
    double err_th9 = 0;
    double err_th10 = 0;
    double err_th11 = 0;
    double err_th12 = 0;
    double err_dth1 = 0;
    double err_dth2 = 0;
    double err_dth3 = 0;
    double err_dth4 = 0;
    double err_dth5 = 0;
    double err_dth6 = 0;
    double err_dth7 = 0;
    double err_dth8 = 0;
    double err_dth9 = 0;
    double err_dth10 = 0;
    double err_dth11 = 0;
    double err_dth12 = 0;

	myStep();
	//제어주기 1step 진행해주는 함수 if 안넣으면 for문의 경우 제어주기 상관없이 최대한 빠른속도로 돎
	// First step to update sensors values
	for (int i = 0; i < NMOTORS; i++) {
		mMotors[i]->setControlPID(MOTOR_P_GAIN, MOTOR_I_GAIN, MOTOR_D_GAIN);  ////////// 
	}
	// for (int i = 0; i < 6; i++) {
	// ftMotors[i]->setControlPID(MOTOR_P_GAIN, MOTOR_I_GAIN, MOTOR_D_GAIN);  ////////// 
	// }

	double _th[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_i[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_interpolation[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_position[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	// double _th_torque[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_printf[12]={0,};

	double _th_torque[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.000000, -0.000000, -0.0, -0.0, -1.57, -0.0, 0.0};
	
	// for (int i = 0; i<100; i++)
	// {
	// 	// mMotors[16]->setPosition(-_th[0]/100*i);
	// 	// mMotors[14]->setPosition(_th[1]/100*i);
	// 	// mMotors[12]->setPosition(-_th[2]/100*i);
	// 	// mMotors[10]->setPosition(-_th[3]/100*i);
	// 	// mMotors[8]->setPosition(_th[4]/100*i);
	// 	// mMotors[6]->setPosition(_th[5]/100*i);
	// 	// mMotors[7]->setPosition(-_th[6]/100*i);
	// 	// mMotors[9]->setPosition(-_th[7]/100*i);
	// 	// mMotors[11]->setPosition(-_th[8]/100*i);
	// 	// mMotors[13]->setPosition(-_th[9]/100*i);
	// 	// mMotors[15]->setPosition(_th[10]/100*i);
	// 	// mMotors[17]->setPosition(_th[11]/100*i);
	// 	////////////////////////////////////////////////////////////////
	// 	mMotors[ankleRollR]->setPosition(-_th_interpolation[0] / 100 * i);
	// 	mMotors[anklePitchR]->setPosition(-_th_interpolation[1] / 100 * i);
	// 	mMotors[kneePitchR]->setPosition(_th_interpolation[2] / 100 * i);
	// 	mMotors[hipPitchR]->setPosition(_th_interpolation[3] / 100 * i);
	// 	mMotors[hipRollR]->setPosition(_th_interpolation[4] / 100 * i);
	// 	mMotors[hipYawR]->setPosition(_th_interpolation[5] / 100 * i);
	// 	mMotors[hipYawL]->setPosition(-_th_interpolation[6] / 100 * i);
	// 	mMotors[hipRollL]->setPosition(-_th_interpolation[7] / 100 * i);
	// 	mMotors[hipPitchL]->setPosition(_th_interpolation[8] / 100 * i);
	// 	mMotors[kneePitchL]->setPosition(_th_interpolation[9] / 100 * i);
	// 	mMotors[anklePitchL]->setPosition(-_th_interpolation[10] / 100 * i);
	// 	mMotors[ankleRollL]->setPosition(_th_interpolation[11] / 100 * i);
	// 	//////////////////////////////////////////////////////////////////
	// 	// mMotors[16]->setPosition(0);
	// 	// mMotors[14]->setPosition(0);
	// 	// mMotors[12]->setPosition(0);
	// 	// mMotors[10]->setPosition(0);
	// 	// mMotors[8]->setPosition(0);
	// 	// mMotors[6]->setPosition(0);
	// 	// mMotors[7]->setPosition(0);
	// 	// mMotors[9]->setPosition(0);
	// 	// mMotors[11]->setPosition(0);
	// 	// mMotors[13]->setPosition(0);
	// 	// mMotors[15]->setPosition(0);
	// 	// mMotors[17]->setPosition(0);
	// 	wait(1);
	// }
	// for(int i = 0;i<20;i++)
	// {
	// printf("[%d] %lf , ",i,mPositionSensors[i]->getValue());
	// }
	wait(200);//0726 modi.
	//wait(100);

	int _i, _j;
	int _sn=0;
	int _pi;
	double _tempp[3] = { 0, };
	double _tempr[3][3] = { 0, };

	
	double dsptime = 0.4;
	double ssptime = 0.4;
	dspOrigin=dsptime;
	if(dspDecelerator==1 )
	{
		dsptime = dspSlowTime;
	}
	if(onlySSPon==1)
	{
		dsptime=0;
		dspOrigin=0;
		ssptime=onlySSPTime;
	}
	double stride_beg = 0.03;
	double stride_mid = 0.03;
	double stride_end = 0.03;
	double stridey_beg = 0.00;
	double stridey_mid = 0.00;
	double stridey_end = 0.00;
	
	// 보행계획
	// double _s[100][7] = {
	// 	{ 0,    0,  0,0,0,0,0 },
	// { 0,    0,  0,0,0,0,0 },
	// { 0   , L3 * 2, 0,  0,0,0,0 },
	// { stride_beg, L3 * 2+stridey_beg, ssptime,0.3,0.0*pi / 180,0,0 },
	// { stride_beg, L3 * 2+stridey_beg, ssptime,0.3,0 * pi / 180,0,0 },
	// { stride_beg, L3 * 2+stridey_beg, ssptime,0.3,0 * pi / 180,0,0 },
	// { stride_mid, L3 * 2+stridey_mid, ssptime,0.3,0 * pi / 180,0,0 },
	// { stride_mid, L3 * 2+stridey_mid, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_mid, L3 * 2+stridey_mid, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { 0   , L3 * 2, ssptime,dsptime,0 * pi / 180,0,0 },
	// { 0   , 0     ,   0,dsptime,0,0,0 } };

	double _s[100][7] = {
		{ 0,    0,  0,0,0,0,0 },
	{ 0,    0,  0,0,0,0,0 },
	{ 0   , L3 * 2, 0,  0,0,0,0 },
	{ stride_beg, L3 * 2+stridey_beg, ssptime,0.2,0*pi / 180,0,0 },//I=3
	{ stride_beg, L3 * 2+stridey_beg, ssptime,dsptime,0 * pi / 180,0,0 },//I=4
	{ stride_beg, L3 * 2+stridey_beg, ssptime,dsptime,0 * pi / 180,0,0 },//I=5
	{ stride_mid, L3 * 2+stridey_mid, ssptime,dsptime, 0* pi / 180,0,0 },//I=6
	{ stride_mid, L3 * 2+stridey_mid, ssptime,dsptime,0 * pi / 180,0,0 },//여기서부터dsptime
	{ stride_mid, L3 * 2+stridey_mid, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },//여기부터
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ 0   , L3 * 2, ssptime,dsptime,0 * pi / 180,0,0 },
	{ 0   , 0     ,   0,dsptime,0,0,0 } };

	//시현 졸업논문 동적 보행
	// double _s[100][7] = {
	// 	{ 0,    0,  0,0,0,0,0 },
	// { 0,    0,  0,0,0,0,0 },
	// { 0   , L3 * 2, 0,  0,0,0,0 },
	// { 0.05, L3 * 2+stridey_beg, 0.4,0.4,0*pi / 180,0,0 },//I=3
	// { 0.05, L3 * 2+0.05, 0.4,0.4,0 * pi / 180,0,0 },//I=4
	// { 0.05, L3 * 2, 0.4,0.4,0 * pi / 180,0,0 },//I=5
	// { 0.05, L3 * 2, 0.4,0.4, 0* pi / 180,0,0 },//I=6
	// { 0.05, L3 * 2+0.05, 0.4,0.4,0 * pi / 180,0,0 },//여기서부터dsptime
	// { 0.05, L3 * 2, 0.4,0.4,0 * pi / 180,0,0 },
	// { 0.05, L3 * 2, 0.4,0.4,0 * pi / 180,0,0 },
	// { 0.05, L3 * 2+0.05, 0.4,0.4,0 * pi / 180,0,0 },
	// { 0.05, L3 * 2+stridey_end, 0.4,0.4,0 * pi / 180,0,0 },
	// { 0.05, L3 * 2+stridey_end, 0.4,0.4,0 * pi / 180,0,0 },//여기부터
	// { stride_end, L3 * 2+stridey_end, 0.4,0.4,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, 0.4,0.4,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	// { 0   , L3 * 2, ssptime,dsptime,0 * pi / 180,0,0 },
	// { 0   , 0     ,   0,dsptime,0,0,0 } };

	int _step_length = 20;//11
	double _stepheight = 0.015;
	double _swingtime = 0;
	double sspf_coeff = 0.5;//34,66//3 7 0.375
	double dspf_coeff = 0.5;
	if(onlySSPon==1)
	{
		sspf_coeff=1;
		dspf_coeff=0.0;
	}
	double _t = 0;
	double _C = 0;
	double _S = 0;

	double _x_ter = 0;
	double _y_ter = 0;
	double _x_terf = 0;
	double _y_terf = 0;
	double _vx_ter = 0;
	double _vy_ter = 0;
	double _pvx_ter = 0;
	double _pvy_ter = 0;
	double _x_com = 0;
	double _y_com = 0;
	double _x_des = 0;
	double _y_des = 0;
	double _x_rot = 0;
	double _y_rot = 0;
	double _x_rotf = 0;
	double _y_rotf = 0;

	double _theta = 0;

	double dsp_x_i, dsp_x_f; 
	double dsp_y_i, dsp_y_f;
	double dsp_dy_i, dsp_dy_f;
	double dsp_dx_i, dsp_dx_f;

	double dsp_x_i_nominal;
	double dsp_x_f_nominal; 
	double dsp_dx_i_nominal;
	double dsp_dx_f_nominal;
	
	double dsp_y_i_nominal;
	double dsp_y_f_nominal;
	double dsp_dy_i_nominal;
	double dsp_dy_f_nominal;
	

//add 0630
	double _rsx_des = 0;
	double _rsy_des = 0;
	double _rsz_des = 0;
	double _lsx_des = 0;
	double _lsy_des = 2 * L3;
	double _lsz_des = 0;
		
	double _dd_rsx_des = 0;
	double _dd_rsy_des = 0;
	double _dd_rsz_des = 0;
	double _dd_lsx_des = 0;
	double _dd_lsy_des = 0;
	double _dd_lsz_des = 0;
	
////////////////add -0630
	double _lsr = 0;
	double _rsr = 0;

	double _dsp_x_des;
	double _dsp_y_des;
	double _baserot = 0;
	double _swingrot_time = 0;
	double _baserot_time = 0;

	_s[0][5] = 0;

	for (_i = 2; _i <= _step_length + 2; _i++)
	{
		_s[_i][5] = _s[_i - 1][5] + _s[_i][2] + _s[_i][3];
		_s[_i][6] = 0;

	}
	for (_i = 3; _i <= _step_length + 2; _i++)
	{
		local_stride_F = _s[_i][0];
		local_stride_L = _s[_i][1];
		pre_local_stride_F = _s[_i - 1][0];
		pre_local_stride_L = _s[_i - 1][1];
	}
	
	FILE* footprint 	= fopen("footprint.txt", "w");
	FILE* slipcontrol 	= fopen("slipcontrol.txt","w");
	FILE* cpt			= fopen("cpt.txt","w");
	FILE* rne			= fopen("rne.txt","w");
	FILE* ft			= fopen("ft.txt","w");

	forwardkinematics(_th);
	_i = 2;
	_t = samplingtime;

	//clock_t start1, start2, start3, start4, end1, end2, end3, end4;

	double rot_th = 0;
//add 0630  4685 - 5143 : RNE에 필요한 물성치 정의랑 matrix 변수 선언 부분인데 확인하면서 추가하면 될 것 같아
	Matrix_3X1 ta1, ta2, ta3, ta4, ta5, ta6, ta7, ta8, ta9, ta10, ta11, ta12;

	ta1.x[0][0] = -1;
	ta1.x[1][0] = 0;
	ta1.x[2][0] = 0;

	ta2.x[0][0] = 0;
	ta2.x[1][0] = -1;
	ta2.x[2][0] = 0;

	ta3.x[0][0] = 0;
	ta3.x[1][0] = -1;
	ta3.x[2][0] = 0;

	ta4.x[0][0] = 0;
	ta4.x[1][0] = -1;
	ta4.x[2][0] = 0;

	ta5.x[0][0] = -1;
	ta5.x[1][0] = 0;
	ta5.x[2][0] = 0;

	ta6.x[0][0] = 0;
	ta6.x[1][0] = 0;
	ta6.x[2][0] = -1;

	ta7.x[0][0] = 0;
	ta7.x[1][0] = 0;
	ta7.x[2][0] = 1;

	ta8.x[0][0] = 1;
	ta8.x[1][0] = 0;
	ta8.x[2][0] = 0;

	ta9.x[0][0] = 0;
	ta9.x[1][0] = 1;
	ta9.x[2][0] = 0;

	ta10.x[0][0] = 0;
	ta10.x[1][0] = 1;
	ta10.x[2][0] = 0;

	ta11.x[0][0] = 0;
	ta11.x[1][0] = 1;
	ta11.x[2][0] = 0;

	ta12.x[0][0] = 1;
	ta12.x[1][0] = 0;
	ta12.x[2][0] = 0;

	//printf("%lf\n\n",ta12.x[0][0]);

	Matrix_M3X12 ta;

	ta.x[0][0] = ta1;
	ta.x[0][1] = ta2;
	ta.x[0][2] = ta3;
	ta.x[0][3] = ta4;
	ta.x[0][4] = ta5;
	ta.x[0][5] = ta6;
	ta.x[0][6] = ta7;
	ta.x[0][7] = ta8;
	ta.x[0][8] = ta9;
	ta.x[0][9] = ta10;
	ta.x[0][10] = ta11;
	ta.x[0][11] = ta12;

		Matrix_3X1 Pc11, Pc22, Pc33, Pc44, Pc55, Pc66, Pc77, Pc88, Pc99, Pc1010, Pc1111, Pc1212;
		//무게중심 입력
	//    Pc11.x[0][0] = -0.00296;
	//    Pc11.x[1][0] = -0.00882;
	//    Pc11.x[2][0] = -0.02692;

	//    Pc22.x[0][0] = -0.01991;
	//    Pc22.x[1][0] = -0.00006;
	//    Pc22.x[2][0] = 0.01126;

	//    Pc33.x[0][0] = 0.00729;
	//    Pc33.x[1][0] = 0.00031;
	//    Pc33.x[2][0] = -0.03762;

		//    Pc44.x[0][0] = 0.00095;
		//    Pc44.x[1][0] = 0.00007;
		//    Pc44.x[2][0] = -0.06731;

	//    Pc55.x[0][0] = -0.01991;
	//    Pc55.x[1][0] = -0.00006;
	//    Pc55.x[2][0] = -0.01108;

	//    Pc66.x[0][0] = 0.00157;
	//    Pc66.x[1][0] = 0;
	//    Pc66.x[2][0] = 0.0196;

	//    Pc77.x[0][0] = 0.00157;
	//    Pc77.x[1][0] = 0;
	//    Pc77.x[2][0] = 0.0196;

	//    Pc88.x[0][0] = -0.01991;
	//    Pc88.x[1][0] = -0.00003;
	//    Pc88.x[2][0] = -0.01108;

	//    Pc99.x[0][0] = 0.00095;
	//    Pc99.x[1][0] = -0.00009;
	//    Pc99.x[2][0] = -0.06731;

	//    Pc1010.x[0][0] = 0.00729;
	//    Pc1010.x[1][0] = -0.00038;
	//    Pc1010.x[2][0] = -0.03762;

	//    Pc1111.x[0][0] = -0.01991;
	//    Pc1111.x[1][0] = -0.00003;
	//    Pc1111.x[2][0] = 0.01126;

	//    Pc1212.x[0][0] = -0.00318;
	//    Pc1212.x[1][0] = 0.00845;
	//    Pc1212.x[2][0] = -0.02687;
	//민하 original

	Pc11.x[0][0] = 0.02373;
	Pc11.x[1][0] = -0.01037;
	Pc11.x[2][0] = -0.0276;

	// Pc11.x[0][0] = -0.000502877;
	// Pc11.x[1][0] = -0.00950588;
	// Pc11.x[2][0] = -0.0259953;//op2

	Pc22.x[0][0] = -0.02022;
	Pc22.x[1][0] = 0.01872;
	Pc22.x[2][0] = 0.01214;

	Pc33.x[0][0] = 0;
	Pc33.x[1][0] = 0.02151;
	Pc33.x[2][0] = -0.055;

	Pc44.x[0][0] = 0.00059;
	Pc44.x[1][0] = 0.01901;
	Pc44.x[2][0] = -0.08408;

	Pc55.x[0][0] = 0.00388;
	Pc55.x[1][0] = -0.00028;
	Pc55.x[2][0] = -0.01214;

	Pc66.x[0][0] = -0.00157;
	Pc66.x[1][0] = 0;
	Pc66.x[2][0] = -0.00774;

	Pc77.x[0][0] = -0.00157;
	Pc77.x[1][0] = 0;
	Pc77.x[2][0] = -0.00774;

	Pc88.x[0][0] = 0.00388;
	Pc88.x[1][0] = 0.00028;
	Pc88.x[2][0] = -0.01214;

	Pc99.x[0][0] = 0.00059;
	Pc99.x[1][0] = -0.01901;
	Pc99.x[2][0] = -0.08408;

	Pc1010.x[0][0] = 0;
	Pc1010.x[1][0] = -0.02151;
	Pc1010.x[2][0] = -0.055;

	Pc1111.x[0][0] = -0.02022;
	Pc1111.x[1][0] = -0.01872;
	Pc1111.x[2][0] = 0.01214;

	Pc1212.x[0][0] = 0.02373;
	Pc1212.x[1][0] = 0.01037;
	Pc1212.x[2][0] = -0.0276;//op3

	// Pc1212.x[0][0] = -0.000502877;
	// Pc1212.x[1][0] = 0.00950588;
	// Pc1212.x[2][0] = -0.0259953;//op2
	//webots original

	// Pc11.x[0][0] = -0.070;
	// Pc11.x[1][0] = 0.000;
	// Pc11.x[2][0] = -0.048;

	// Pc22.x[0][0] = -0.011;
	// Pc22.x[1][0] = 0.033;
	// Pc22.x[2][0] = 0.000;

	// Pc33.x[0][0] = -0.002;
	// Pc33.x[1][0] = 0.066;
	// Pc33.x[2][0] = -0.183;

	// Pc44.x[0][0] = 0.022;
	// Pc44.x[1][0] = 0.007;
	// Pc44.x[2][0] = -0.168;

	// Pc55.x[0][0] = -0.068;
	// Pc55.x[1][0] = 0.000;
	// Pc55.x[2][0] = 0.000;

	// Pc66.x[0][0] = -0.012;
	// Pc66.x[1][0] = 0;
	// Pc66.x[2][0] = -0.025;

	// Pc77.x[0][0] = 0.012;
	// Pc77.x[1][0] = 0.000;
	// Pc77.x[2][0] = -0.025;

	// Pc88.x[0][0] = -0.068;
	// Pc88.x[1][0] = 0.000;
	// Pc88.x[2][0] = 0.000;

	// Pc99.x[0][0] = 0.022;
	// Pc99.x[1][0] = -0.007;
	// Pc99.x[2][0] = -0.168;

	// Pc1010.x[0][0] = -0.002;
	// Pc1010.x[1][0] = -0.066;
	// Pc1010.x[2][0] = -0.183;

	// Pc1111.x[0][0] = -0.011;
	// Pc1111.x[1][0] = -0.033;
	// Pc1111.x[2][0] = 0.000;

	// Pc1212.x[0][0] = -0.070;
	// Pc1212.x[1][0] = 0.000;
	// Pc1212.x[2][0] = -0.048;
	//from op3_kinematics_dynamics.cpp

		Matrix_M3X12 PositionC;

		PositionC.x[0][0] = Pc11;
		PositionC.x[0][1] = Pc22;
		PositionC.x[0][2] = Pc33;
		PositionC.x[0][3] = Pc44;
		PositionC.x[0][4] = Pc55;
		PositionC.x[0][5] = Pc66;
		PositionC.x[0][6] = Pc77;
		PositionC.x[0][7] = Pc88;
		PositionC.x[0][8] = Pc99;
		PositionC.x[0][9] = Pc1010;
		PositionC.x[0][10] = Pc1111;
		PositionC.x[0][11] = Pc1212;

		//링크질량
		double m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12;
	//    m1 = 0.06925; // 단위 : kg
	//    m2 = 0.15899;
	//    m3 = 0.06438;
	//    m4 = 0.11083;
	//    m5 = 0.15899;
	//    m6 = 0.02029;
	//    m7 = 0.02029;
	//    m8 = 0.15899;
	//    m9 = 0.11083;
	//    m10 = 0.06438;
	//    m11 = 0.15899;
	//    m12 = 0.06925;
	//민하 original춻C!

	//    m1 = 0.0794462;//0.0223; // 단위 : kg
	//    m2 = 0.1045;
	//    m3 = 0.2401;
	//    m4 = 0.3095;
	//    m5 = 0.1045;
	//    m6 = 0.0243;
	//    m7 = 0.0243;
	//    m8 = 0.1045;
	//    m9 = 0.3095;
	//    m10 = 0.2401;
	//    m11 = 0.1045;
	//    m12 = 0.0794462;//0.2223;
		//from op3_kinematics_dynamics.cpp

		m1 = 0.06934;//0.0794462; //0.06934// 단위 : kg
		m2 = 0.17886;
		m3 = 0.04015;
		m4 = 0.11543;
		m5 = 0.17886;
		m6 = 0.0118;
		m7 = 0.0118;
		m8 = 0.17886;
		m9 = 0.11543;
		m10 = 0.04015;
		m11 = 0.17886;
		m12 = 0.06934;//0.0794462;
		//webots original

		Matrix_3X3 I11, I22, I33, I44, I55, I66, I77, I88, I99, I1010, I1111, I1212;


	//    I11.x[0][0] = 3.193154e-5;
	//    I11.x[0][1] = 3.0959e-7;
	//    I11.x[0][2] = -4.9998e-7;
	//    I11.x[1][0] = 3.0959e-7;
	//    I11.x[1][1] = 6.171732e-5;
	//    I11.x[1][2] = 1.36194e-6;
	//    I11.x[2][0] = -4.9998e-7;
	//    I11.x[2][1] = 1.36194e-6;
	//    I11.x[2][2] = 8.205425e-5;

	//    I22.x[0][0] = 4.189985e-5;
	//    I22.x[0][1] = 1.2602e-7;
	//    I22.x[0][2] = -3.04186e-6;
	//    I22.x[1][0] = 1.2602e-7;
	//    I22.x[1][1] = 1.2011812e-4;
	//    I22.x[1][2] = -3.2218e-7;
	//    I22.x[2][0] = -3.04186e-6;
	//    I22.x[2][1] = -3.2218e-7;
	//    I22.x[2][2] = 1.0692839e-4;

	//    I33.x[0][0] = 9.724951e-5;
	//    I33.x[0][1] = -1.686e-7;
	//    I33.x[0][2] = 1.25639e-5;
	//    I33.x[1][0] = -1.686e-7;
	//    I33.x[1][1] = 7.99727e-5;
	//    I33.x[1][2] = -1.9937e-7;
	//    I33.x[2][0] = 1.25639e-5;
	//    I33.x[2][1] = -1.9937e-7;
	//    I33.x[2][2] = 4.3535e-5;

	//    I44.x[0][0] = 1.0483387e-4;
	//    I44.x[0][1] = -1.551e-8;
	//    I44.x[0][2] = 1.78907e-6;
	//    I44.x[1][0] = -1.551e-8;
	//    I44.x[1][1] = 9.09557e-5;
	//    I44.x[1][2] = 4.4481e-7;
	//    I44.x[2][0] = 1.78907e-6;
	//    I44.x[2][1] = 4.4481e-7;
	//    I44.x[2][2] = 3.111228e-5;

	//    I55.x[0][0] = 4.138086e-5;
	//    I55.x[0][1] = 1.2598e-7;
	//    I55.x[0][2] = 2.19671e-6;
	//    I55.x[1][0] = 1.2598e-7;
	//    I55.x[1][1] = 1.1960832e-4;
	//    I55.x[1][2] = 3.2115e-7;
	//    I55.x[2][0] = 2.19671e-6;
	//    I55.x[2][1] = 3.2115e-7;
	//    I55.x[2][2] = 1.0691919e-4;

	//    I66.x[0][0] = 4.46122e-6;
	//    I66.x[0][1] = 0;
	//    I66.x[0][2] = 3.3983e-7;
	//    I66.x[1][0] = 0;
	//    I66.x[1][1] = 1.125258e-5;
	//    I66.x[1][2] = 0;
	//    I66.x[2][0] = 3.3983e-7;
	//    I66.x[2][1] = 0;
	//    I66.x[2][2] = 8.90209e-6;

	//    I77.x[0][0] = 4.46122e-6;
	//    I77.x[0][1] = 0;
	//    I77.x[0][2] = 3.3983e-7;
	//    I77.x[1][0] = 0;
	//    I77.x[1][1] = 1.125258e-5;
	//    I77.x[1][2] = 0;
	//    I77.x[2][0] = 3.3983e-7;
	//    I77.x[2][1] = 0;
	//    I77.x[2][2] = 8.90209e-6;


	//    I88.x[0][0] = 4.138131e-5;
	//    I88.x[0][1] = 2.2091e-7;
	//    I88.x[0][2] = 2.19672e-6;
	//    I88.x[1][0] = 2.2091e-7;
	//    I88.x[1][1] = 1.1960832e-4;
	//    I88.x[1][2] = -3.1460e-7;
	//    I88.x[2][0] = 2.19672e-6;
	//    I88.x[2][1] = -3.1460e-7;
	//    I88.x[2][2] = 1.0691964e-4;

	//    I99.x[0][0] = 1.0483350e-4;
	//    I99.x[0][1] = 1.16e-9;
	//    I99.x[0][2] = 1.78908e-6;
	//    I99.x[1][0] = 1.16e-9;
	//    I99.x[1][1] = 9.09557e-5;
	//    I99.x[1][2] = -4.8729e-7;
	//    I99.x[2][0] = 1.78908e-6;
	//    I99.x[2][1] = -4.8729e-7;
	//    I99.x[2][2] = 3.111191e-5;

	//    I1010.x[0][0] = 9.724641e-5;
	//    I1010.x[0][1] = 1.5590e-7;
	//    I1010.x[0][2] = 1.25639e-5;
	//    I1010.x[1][0] = 1.5590e-7;
	//    I1010.x[1][1] = 7.99727e-5;
	//    I1010.x[1][2] = 1.9613e-7;
	//    I1010.x[2][0] = 1.25639e-5;
	//    I1010.x[2][1] = 1.9613e-7;
	//    I1010.x[2][2] = 4.353189e-5;

	//    I1111.x[0][0] = 4.190032e-5;
	//    I1111.x[0][1] = 2.2096e-7;
	//    I1111.x[0][2] = -3.04186e-6;
	//    I1111.x[1][0] = 2.2096e-7;
	//    I1111.x[1][1] = 1.2011811e-4;
	//    I1111.x[1][2] = 3.1269e-7;
	//    I1111.x[2][0] = -3.04186e-6;
	//    I1111.x[2][1] = 3.1269e-7;
	//    I1111.x[2][2] = 1.0692885e-4;

	//    I1212.x[0][0] = 3.179931e-5;
	//    I1212.x[0][1] = -3.3779e-7;
	//    I1212.x[0][2] = -4.6555e-7;
	//    I1212.x[1][0] = -3.3779e-7;
	//    I1212.x[1][1] = 6.176469e-5;
	//    I1212.x[1][2] = -1.25437e-6;
	//    I1212.x[2][0] = -4.6555e-7;
	//    I1212.x[2][1] = -1.25437e-6;
	//    I1212.x[2][2] = 8.192080e-5;
	//민하 original

	I11.x[0][0] = 4.034e-05;//op3
	I11.x[0][1] = 1.9e-07;
	I11.x[0][2] = 1.2e-07;
	I11.x[1][0] = 1.9e-07;
	I11.x[1][1] = 7.874e-05;
	I11.x[1][2] = -1.01e-06;
	I11.x[2][0] = 1.2e-07;
	I11.x[2][1] = -1.01e-06;
	I11.x[2][2] = 0.00011579;

	// I11.x[0][0] = 3.58408e-05;//op2
	// I11.x[0][1] = 9.76205e-07;
	// I11.x[0][2] = -2.53305e-06;
	// I11.x[1][0] = 9.76205e-07;
	// I11.x[1][1] = 6.83256e-05;
	// I11.x[1][2] = -2.18696e-07;
	// I11.x[2][0] = -2.53305e-06;
	// I11.x[2][1] = -2.18696e-07;
	// I11.x[2][2] = 8.78905e-05;

	I22.x[0][0] = 4.661e-05;
	I22.x[0][1] = 1.01e-06;
	I22.x[0][2] = 1.31e-06;
	I22.x[1][0] = 1.01e-06;
	I22.x[1][1] = 0.00012523;
	I22.x[1][2] = -6e-08;
	I22.x[2][0] = 1.31e-06;
	I22.x[2][1] = -6e-08;
	I22.x[2][2] = 0.00010857;

	I33.x[0][0] = 3.715e-05;
	I33.x[0][1] = 0;
	I33.x[0][2] = 0;
	I33.x[1][0] = 0;
	I33.x[1][1] = 2.751e-05;
	I33.x[1][2] = 0;
	I33.x[2][0] = 0;
	I33.x[2][1] = 0;
	I33.x[2][2] = 1.511e-05;

	I44.x[0][0] = 0.00010499;
	I44.x[0][1] = 1e-08;
	I44.x[0][2] = -7.1e-07;
	I44.x[1][0] = 1e-08;
	I44.x[1][1] = 9.613e-05;
	I44.x[1][2] = -3.53e-06;
	I44.x[2][0] = -7.1e-07;
	I44.x[2][1] = -3.53e-06;
	I44.x[2][2] = 2.493e-05;

	I55.x[0][0] = 4.661e-05;
	I55.x[0][1] = 1.01e-06;
	I55.x[0][2] = -1.31e-06;
	I55.x[1][0] = 1.01e-06;
	I55.x[1][1] = 0.00012523;
	I55.x[1][2] = 6e-08;
	I55.x[2][0] = -1.31e-06;
	I55.x[2][1] = 6e-08;
	I55.x[2][2] = 0.00010857;

	I66.x[0][0] = 1.51e-06;
	I66.x[0][1] = 0;
	I66.x[0][2] = 1e-08;
	I66.x[1][0] = 0;
	I66.x[1][1] = 4.3e-06;
	I66.x[1][2] = 0;
	I66.x[2][0] = 1e-08;
	I66.x[2][1] = 0;
	I66.x[2][2] = 4.12e-06;

	I77.x[0][0] = 1.51e-06;
	I77.x[0][1] = 0;
	I77.x[0][2] = 1e-08;
	I77.x[1][0] = 0;
	I77.x[1][1] = 4.3e-06;
	I77.x[1][2] = 0;
	I77.x[2][0] = 1e-08;
	I77.x[2][1] = 0;
	I77.x[2][2] = 4.12e-06;


	I88.x[0][0] = 4.661e-05;
	I88.x[0][1] = -1.01e-06;
	I88.x[0][2] = -1.31e-06;
	I88.x[1][0] = -1.01e-06;
	I88.x[1][1] = 0.00012523;
	I88.x[1][2] = -6e-08;
	I88.x[2][0] = -1.31e-06;
	I88.x[2][1] = -6e-08;
	I88.x[2][2] = 0.00010857;

	I99.x[0][0] = 0.00010499;
	I99.x[0][1] = -1e-08;
	I99.x[0][2] = -7.1e-07;
	I99.x[1][0] = -1e-08;
	I99.x[1][1] = 9.613e-05;
	I99.x[1][2] = 3.53e-06;
	I99.x[2][0] = -7.1e-07;
	I99.x[2][1] = 3.53e-06;
	I99.x[2][2] = 2.493e-05;

	I1010.x[0][0] = 3.715e-05;
	I1010.x[0][1] = 0;
	I1010.x[0][2] = 0;
	I1010.x[1][0] = 0;
	I1010.x[1][1] = 2.751e-05;
	I1010.x[1][2] = 0;
	I1010.x[2][0] = 0;
	I1010.x[2][1] = 0;
	I1010.x[2][2] = 1.511e-05;

	I1111.x[0][0] = 4.661e-05;
	I1111.x[0][1] = -1.01e-06;
	I1111.x[0][2] = 1.31e-06;
	I1111.x[1][0] = -1.01e-06;
	I1111.x[1][1] = 0.00012523;
	I1111.x[1][2] = 6e-08;
	I1111.x[2][0] = 1.31e-06;
	I1111.x[2][1] = 6e-08;
	I1111.x[2][2] = 0.00010857;

	// I1212.x[0][0] = 3.58408e-05;//op2
	// I1212.x[0][1] = 9.76205e-07;
	// I1212.x[0][2] = 2.53305e-06;
	// I1212.x[1][0] = 9.76205e-07;
	// I1212.x[1][1] = 6.83256e-05;
	// I1212.x[1][2] = 2.18696e-07;
	// I1212.x[2][0] = 2.53305e-06;
	// I1212.x[2][1] = 2.18696e-07;
	// I1212.x[2][2] = 8.78905e-05;

	I1212.x[0][0] = 4.034e-05;//op3
	I1212.x[0][1] = -1.9e-07;
	I1212.x[0][2] = 1.2e-07;
	I1212.x[1][0] = -1.9e-07;
	I1212.x[1][1] = 7.874e-05;
	I1212.x[1][2] = 1.01e-06;
	I1212.x[2][0] = 1.2e-07;
	I1212.x[2][1] = 1.01e-06;
	I1212.x[2][2] = 0.00011579;
	//webots original

	// I11.x[0][0] = 0.00022;
	// I11.x[0][1] = 0.00099;
	// I11.x[0][2] = 0.00000;
	// I11.x[1][0] = 0.00099;
	// I11.x[1][1] = 0.00000;
	// I11.x[1][2] = 0.00091;
	// I11.x[2][0] = 0.00000;
	// I11.x[2][1] = 0.00091;
	// I11.x[2][2] = -0.00001;

	// I22.x[0][0] = 0.00056;
	// I22.x[0][1] = 0.00168;
	// I22.x[0][2] = 0.00000;
	// I22.x[1][0] = 0.00168;
	// I22.x[1][1] = 0.00000;
	// I22.x[1][2] = 0.00171;
	// I22.x[2][0] = 0.00000;
	// I22.x[2][1] = 0.00171;
	// I22.x[2][2] = 0.00000;

	// I33.x[0][0] = 0.01971;
	// I33.x[0][1] = 0.01687;
	// I33.x[0][2] = -0.00140;
	// I33.x[1][0] = 0.01687;
	// I33.x[1][1] = -0.00031;
	// I33.x[1][2] = 0.00574;
	// I33.x[2][0] = -0.00140;
	// I33.x[2][1] = 0.00574;
	// I33.x[2][2] = -0.00294;

	// I44.x[0][0] = 0.04329;
	// I44.x[0][1] = 0.04042;
	// I44.x[0][2] = 0.00203;
	// I44.x[1][0] = 0.04042;
	// I44.x[1][1] = -0.00027;
	// I44.x[1][2] = 0.00560;
	// I44.x[2][0] = 0.00203;
	// I44.x[2][1] = 0.00560;
	// I44.x[2][2] = 0.00286;

	// I55.x[0][0] = 0.00056;
	// I55.x[0][1] = 0.00168;
	// I55.x[0][2] = 0.00000;
	// I55.x[1][0] = 0.00168;
	// I55.x[1][1] = 0.00000;
	// I55.x[1][2] = 0.00171;
	// I55.x[2][0] = 0.00000;
	// I55.x[2][1] = 0.00171;
	// I55.x[2][2] = 0.00000;

	// I66.x[0][0] = 0.00024;
	// I66.x[0][1] = 0.00101;
	// I66.x[0][2] = 0.00000;
	// I66.x[1][0] = 0.00101;
	// I66.x[1][1] = 0.00000;
	// I66.x[1][2] = 0.00092;
	// I66.x[2][0] = 0.00000;
	// I66.x[2][1] = 0.00092;
	// I66.x[2][2] = 0.00000;

	// I77.x[0][0] = 0.00024;
	// I77.x[0][1] = 0.00101;
	// I77.x[0][2] = 0.00000;
	// I77.x[1][0] = 0.00101;
	// I77.x[1][1] = 0.00000;
	// I77.x[1][2] = 0.00092;
	// I77.x[2][0] = 0.00000;
	// I77.x[2][1] = 0.00092;
	// I77.x[2][2] = 0.00000;


	// I88.x[0][0] = 0.00056;
	// I88.x[0][1] = 0.00168;
	// I88.x[0][2] = 0.00000;
	// I88.x[1][0] = 0.00168;
	// I88.x[1][1] = 0.00000;
	// I88.x[1][2] = 0.00171;
	// I88.x[2][0] = 0.00000;
	// I88.x[2][1] = 0.00171;
	// I88.x[2][2] = 0.00000;

	// I99.x[0][0] = 0.04328;
	// I99.x[0][1] = 0.04042;
	// I99.x[0][2] = -0.00202;
	// I99.x[1][0] = 0.04042;
	// I99.x[1][1] = 0.00028;
	// I99.x[1][2] = 0.00560;
	// I99.x[2][0] = -0.00202;
	// I99.x[2][1] = 0.00560;
	// I99.x[2][2] = 0.00288;

	// I1010.x[0][0] = 0.01971;
	// I1010.x[0][1] = 0.01687;
	// I1010.x[0][2] = 0.00140;
	// I1010.x[1][0] = 0.01687;
	// I1010.x[1][1] = 0.00031;
	// I1010.x[1][2] = 0.00574;
	// I1010.x[2][0] = 0.00140;
	// I1010.x[2][1] = 0.00574;
	// I1010.x[2][2] = -0.00294;

	// I1111.x[0][0] = 0.00056;
	// I1111.x[0][1] = 0.00168;
	// I1111.x[0][2] = 0.00000;
	// I1111.x[1][0] = 0.00168;
	// I1111.x[1][1] = 0.00000;
	// I1111.x[1][2] = 0.00171;
	// I1111.x[2][0] = 0.00000;
	// I1111.x[2][1] = 0.00171;
	// I1111.x[2][2] = 0.00000;

	// I1212.x[0][0] = 0.00022;
	// I1212.x[0][1] = 0.00099;
	// I1212.x[0][2] = 0.00000;
	// I1212.x[1][0] = 0.00099;
	// I1212.x[1][1] = 0.00000;
	// I1212.x[1][2] = 0.00091;
	// I1212.x[2][0] = 0.00000;
	// I1212.x[2][1] = 0.00091;
	// I1212.x[2][2] = -0.00001;
	//from op3_kinematics_dynamics.cpp


	Matrix_MM33X12 Inertia;//33X12 를 표현

	Inertia.x[0][0] = I11;
	Inertia.x[0][1] = I22;
	Inertia.x[0][2] = I33;
	Inertia.x[0][3] = I44;
	Inertia.x[0][4] = I55;
	Inertia.x[0][5] = I66;
	Inertia.x[0][6] = I77;
	Inertia.x[0][7] = I88;
	Inertia.x[0][8] = I99;
	Inertia.x[0][9] = I1010;
	Inertia.x[0][10] = I1111;
	Inertia.x[0][11] = I1212;

	double rtorque1 = 0;
	double rtorque2 = 0;
	double rtorque3 = 0;
	double rtorque4 = 0;
	double rtorque5 = 0;
	double rtorque6 = 0;
	double rtorque7 = 0;
	double rtorque8 = 0;
	double rtorque9 = 0;
	double rtorque10 = 0;
	double rtorque11 = 0;
	double rtorque12 = 0;


	//int samplingPeriod = 8;

	mMotors[ankleRollR]-> enableTorqueFeedback(samplingTime);
	mMotors[anklePitchR]-> enableTorqueFeedback(samplingTime);
	mMotors[kneePitchR]-> enableTorqueFeedback(samplingTime);
	mMotors[hipPitchR]-> enableTorqueFeedback(samplingTime);
	mMotors[hipRollR]-> enableTorqueFeedback(samplingTime);
	mMotors[hipYawR]-> enableTorqueFeedback(samplingTime);
	mMotors[hipYawL]-> enableTorqueFeedback(samplingTime);
	mMotors[hipRollL]-> enableTorqueFeedback(samplingTime);
	mMotors[hipPitchL]-> enableTorqueFeedback(samplingTime);
	mMotors[kneePitchL]-> enableTorqueFeedback(samplingTime);
	mMotors[anklePitchL]-> enableTorqueFeedback(samplingTime);
	mMotors[ankleRollL]-> enableTorqueFeedback(samplingTime);
  		
  		
	Matrix_3X1 _q_LE_p, _q_RE_p, _w_LE_p, _w_RE_p, _q_LE_n, _q_RE_n, _w_LE_n, _w_RE_n,_dw_LE_n,_dw_RE_n;
	Matrix_3X3 RR01, RR12, RR23, RR34, RR45, RR56, RR67;
	Matrix_3X3 RL01, RL12, RL23, RL34, RL45, RL56, RL67;
	Matrix_3X1 _ddp_LE;
	Matrix_12X1 _theta_p, _theta_n;
	Matrix_M33X7 RR, RL;
	Matrix_3X3 RR10, RR21, RR32, RR43, RR54, RR65, RR76;
	Matrix_3X3 RL10, RL21, RL32, RL43, RL54, RL65, RL76;
	Matrix_M33X7 invRR, invRL;
	Matrix_3X1 wr00, wr11, wr22, wr33, wr44, wr55, wr66, wl00, wl11, wl22, wl33, wl44, wl55, wl66;
	Matrix_3X1 dwr00, dwr11, dwr22, dwr33, dwr44, dwr55, dwr66, dwl00, dwl11, dwl22, dwl33, dwl44, dwl55, dwl66;
	Matrix_3X1 dvr00, dvr11, dvr22, dvr33, dvr44, dvr55, dvr66,  dvl00, dvl11, dvl22, dvl33, dvl44, dvl55, dvl66;
	Matrix_3X1 dvcr00, dvcr11, dvcr22, dvcr33, dvcr44, dvcr55, dvcr66, dvcl00, dvcl11, dvcl22, dvcl33, dvcl44, dvcl55, dvcl66;
	/*
	Matrix_3X1 dth11, dth22, dth33, dth44 ,dth55, dth66, dth77, dth88, dth99, dth1010, dth1111, dth1212;
	Matrix_3X1 ddth11, ddth22, ddth33, ddth44 ,ddth55, ddth66, ddth77, ddth88, ddth99, ddth1010, ddth1111, ddth1212;
	*/
	Matrix_M31X7 Mwr, Mdwr, Mdvr, Mdvcr, Mwl, Mdwl, Mdvl, Mdvcl;
	Matrix_3X1 PR01, PR12, PR23, PR34, PR45 ,PR56, PR67, PL01, PL12, PL23, PL34, PL45 ,PL56, PL67 ;
	Matrix_M31X7 PositionppR;
	Matrix_M31X7 PositionppL;
	double _a_L[6][6], _d_L[6][6], _deter_L;
	int  _n_L;


	Matrix_6X6 _jacobi_L_p, _invjacobi_L_p;
	Matrix_6X6 _jacobi_L_n;

	double _a_R[6][6], _d_R[6][6], _deter_R;
	int  _n_R;

	
	Matrix_6X6 _jacobi_R_p_1, _invjacobi_R_p_1, _jacobi_R_p, _invjacobi_R_p;
	Matrix_6X6 _jacobi_R_n_1,_jacobi_R_n;
		
		
	Matrix_12X1 _dth_p;
	Matrix_12X1 _dth_n;
	Matrix_6X6 ddq_Jdot_L_p , ddq_Jdot_R_p, ddq_Jdot_L_n , ddq_Jdot_R_n;
	Matrix_12X1  Mdth_d, Mddth_d;
	Matrix_6X1 ddq_ddx_L, ddq_ddx_R;
	Matrix_6X1 ddq_dq_L, ddq_dq_R;
	Matrix_6X1 ddq_help_1_L, ddq_help_1_R, ddq_help_2_L, ddq_help_2_R, ddq_L,ddq_R ;

	//CTM	
	double _th_encoder[12];
	double _th_encoder_p[12];
	Matrix_12X1 _th_current_n, _th_current_p, Mdth;
	Matrix_12X1 _error_dth_1, _error_dth, _error_th_1, _error_th, Mddth;
	
	double i_th_encoder[12];
	double j_th_encoder[12];
	Matrix_12X1 i_th_current_n, i_th_current_p,  iMdth;
	Matrix_12X1 i_error, pre_i_control, i_control, pre_error_th_1;
	
	//double _kv, _kp;

	/////CPT

	Matrix_6X6 inv_jac_R, inv_jac_L;
	Matrix_6X1 inv_th_R, inv_th_L, pre_inv_vel_1,inv_vel;
	Matrix_3X1 inv_pos ;
	double pre_inv_vel[2];
	double inv_F_vel[2];
	double pre_inv_F_vel[2];
	double wr_m2_1, dwr_m2_1;

	Matrix_M31X7 RNEwr, RNEdwr, RNEdvr, RNEdvcr, RNEFr, RNENr, RNEfr, RNEnr;
	Matrix_3X1 Fr00, Nr00, fr77, nr77;

	Matrix_3X3 rotationr, RNENr_1_1;
	Matrix_3X1 omegar, productwr, productdwr1, cproductdwr1,ta_zr;
	Matrix_3X1 vpr1_m1, vpr1, vpr2_1, vpr2, vpr3, domegar, wr_m2, wr_m3, dwr_m2, vectorplusdvr1;
	Matrix_3X1 Rdvcr1, positionCr, Rdvcr2_1, Rdvcr2, RNENr_1, RNENr_2, RNENr_3;
	Matrix_3X1 helpRNEdvcr, helpRNEFr;

	double RNEmass[12] = { m1, m2, m3, m4 , m5, m6, m7, m8 , m9, m10, m11, m12 };//질량
		//printf("%lf\t %lf\t %lf\t %lf\n", (float)m1, (float)m2, (float)m3, (float)m4);

	Matrix_3X1 testmatrixr1, testmatrixr2, testmatrixr3;

	double torque1 = 0;
	double torque2 = 0;
	double torque3 = 0;
	double torque4 = 0;
	double torque5 = 0;
	double torque6 = 0;
	double torque7 = 0;
	double torque8 = 0;
	double torque9 = 0;
	double torque10 = 0;
	double torque11 = 0;
	double torque12 = 0;

	int k;

	Matrix_3X1 RNEfr_1, RNEfr_2, ta_zr_1;
	Matrix_3X1 RNEnr_1, RNEnr_2, RNEnr_3;
	Matrix_3X1 inwardpositionCr;
	Matrix_3X1 helpFinaltorquer;
	Matrix_1X3 helpFinaltorquer_1;
	Matrix_12X1 Finaltorque;

	Matrix_3X1 Fl00, Nl00, fl77, nl77;

	Matrix_M31X7 RNEwl, RNEdwl, RNEdvl, RNEdvcl, RNEFl, RNENl, RNEfl, RNEnl;

	double wl_m2_1, dwl_m2_1;
	Matrix_3X3 rotationl, RNENl_1_1;

	Matrix_3X1 omegal, productwl, productdwl1, cproductdwl1, ta_zl;
	Matrix_3X1 vpl1_m1, vpl1, vpl2_1, vpl2, vpl3, domegal, wl_m2, wl_m3, dwl_m2, vectorplusdvl1;
	Matrix_3X1 Rdvcl1, positionCl, Rdvcl2_1, Rdvcl2, RNENl_1, RNENl_2, RNENl_3;
	Matrix_3X1 helpRNEdvcl, helpRNEFl;

	Matrix_3X1 testmatrixl1, testmatrixl2, testmatrixl3;

	Matrix_3X1 RNEfl_1, RNEfl_2, ta_zl_1;
	Matrix_3X1 RNEnl_1, RNEnl_2, RNEnl_3;
	Matrix_3X1 inwardpositionCl;
	Matrix_3X1 helpFinaltorquel;
	Matrix_1X3 helpFinaltorquel_1;
////////////////////////////////////////////////////////////////////////////////////////////

	for (int i = 1; i<ps_stand; i++)
	{
		// printf("\n------------------------------------------------------------------------");
		// printf("\n-------------------------POSITION MODE STAND ON-------------------------");
		// printf("\n------------------------------------------------------------------------");
		// printf("\n%c#############%c @@@@@@@@@@@@@@@@", gpscoordinateR, gpscoordinateL);
		gpsr = mGPSR->getValues();
		gpsl = mGPSL->getValues();
		// gpscoordinateR= = mGPSR -> getCoordinateSystem();
		// gpscoordinateL= = mGPSL -> getCoordinateSystem();
		compassr = mCompassR->getValues();
		compassl = mCompassL->getValues();
		compassc=mCompassC->getValues();

		gpsrx = gpsr[0];
		gpsry = gpsr[1];
		gpsrz = gpsr[2];
		gpslx = gpsl[0];
		gpsly = gpsl[1];
		gpslz = gpsl[2];

		compassrx = compassr[0];
		compassry = compassr[1];
		compassrz = compassr[2];
		compasslx = compassl[0];
		compassly = compassl[1];
		compasslz = compassl[2];
		compasscx = compassc[0];
		compasscy = compassc[1];
		compasscz = compassc[2];

		gpsrxoffset = gpsrx;
		gpsryoffset = gpsry;
		gpsrzoffset = gpsrz;
		gpslxoffset = gpslx;
		gpslyoffset = gpsly;
		gpslzoffset = gpslz;

		footradR = atan2(compassrx, compassry);
		footradL = atan2(compasslx, compassly);
		comRad=atan2(compasscx,compasscy);
		footdegreeR = (footradR - 1.5708) / M_PI * 180.0;if (footdegreeR<-180){footdegreeR+=360; }   
		footdegreeL = (footradL - 1.5708) / M_PI * 180.0;if (footdegreeL<-180){footdegreeL+=360; }   
		comDegree=(comRad-1.5708)/M_PI*180.0;if(comDegree<-180){comDegree+=360;}
                            
		footdegreeRoffset = footdegreeR;
		footdegreeLoffset = footdegreeL;
		comdegreeOffset=comDegree;

		// printf("\n gpsrx : %lf ", gpsrx);
		// printf("\t gpsry : %lf ", gpsry);
		// printf("\t gpsrz : %lf ", gpsrz);
		// printf("\t\t gpslx : %lf ", gpslx);
		// printf("\t gpsly : %lf ", gpsly);
		// printf("\t gpslz : %lf ", gpslz);
		// printf("\n compassrx : %lf ", compassrx);
		// printf("\t compassry : %lf ", compassry);
		// printf("\t compassrz : %lf ", compassrz);
		// printf("\t\t compasslx : %lf ", compasslx);
		// printf("\t compassly : %lf ", compassly);
		// printf("\t compasslz : %lf ", compasslz);
        // printf("\t footradR : %lf ", footradR);
        // printf("\t footdegreeR : %lf ", footdegreeR);

		// printf("\n================footradR : %lf\t footradL : %lf=====================", footradR, footradL);
		// printf("\t gpscoordinateR : %c ", gpscoordinateR);
		// printf("\t gpscoordinateL : %c ", gpscoordinateL);

		acc = mAccelerometer->getValues();
		_xcom_acc_measure = acc[0];
		_ycom_acc_measure = acc[1];
		_zcom_acc_measure = acc[2];
		footRX_acc = accR[0];
		footRY_acc = accR[1];
		footRZ_acc = accR[2];
		footLX_acc = accL[0];
		footLY_acc = accL[1];
		footLZ_acc = accL[2];

		ftMotors[0]->setPosition(0);
		ftMotors[1]->setPosition(0);
		ftMotors[2]->setPosition(0);
		ftMotors[3]->setPosition(0);
		ftMotors[4]->setPosition(0);
		ftMotors[5]->setPosition(0);

		torquexr = ftMotors[0]->getTorqueFeedback();
		torqueyr = ftMotors[1]->getTorqueFeedback();
		torquezr = ftMotors[2]->getTorqueFeedback();
		torquexl = ftMotors[3]->getTorqueFeedback();
		torqueyl = ftMotors[4]->getTorqueFeedback();
		torquezl = ftMotors[5]->getTorqueFeedback();
		printf("\n ptorquerollR : %lf ",torquexr);
		printf("\t ptorquepitchR : %lf ",torqueyr);
		printf("\t ptorqueyawR : %lf ",torquezr);
		printf("\t\t ptorquerollL : %lf ",torquexl);
		printf("\t ptorquepitchL : %lf ",torqueyl);
		printf("\t ptorqueyawL : %lf ",torquezl);

		F_Lu = mFlu->getValues();
		F_Ld = mFld->getValues();
		F_Ru = mFru->getValues();
		F_Rd = mFrd->getValues();

		Frxu = F_Ru[0];
		Fryu = F_Ru[1];
		Frzu = F_Ru[2];
		Frxd = F_Rd[2];
		Fryd = F_Rd[1];
		Frzd = F_Rd[0];

		Flxu = F_Lu[0];
		Flyu = F_Lu[1];
		Flzu = F_Lu[2];
		Flxd = F_Ld[2];
		Flyd = F_Ld[1];
		Flzd = F_Ld[0];

		Flxu >= Flxd ? FXl = Flxu : FXl = -Flxd;
		Flyu >= Flyd ? FYl = Flyu : FYl = -Flyd;
		Flzu >= Flzd ? FZl = Flzu : FZl = -Flzd;
		
		Frxu >= Frxd ? FXr = Frxu : FXr = -Frxd;
		Fryu >= Fryd ? FYr = Fryu : FYr = -Fryd;
		Frzu >= Frzd ? FZr = Frzu : FZr = -Frzd;
		
		// printf("\n Flxu : %lf ",Flxu);
		// printf("\t Flyu : %lf ",Flyu);
		// printf("\t Flzu : %lf ",Flzu);
		// printf("\t Flxd : %lf ",Flxd);
		// printf("\t Flyd : %lf ",Flyd);
		// printf("\t Flzd : %lf ",Flzd); 
		// printf("\t Frxu : %lf ",Flxu);
		// printf("\t Fryu : %lf ",Flyu);
		// printf("\t Frzu : %lf ",Frzu);
		// printf("\t Frxd : %lf ",Flxd);
		// printf("\t Fryd : %lf ",Flyd);
		// printf("\t Frzd : %lf \n",Frzd);
		ftzmpxr = (-torqueyr) / Frzu * 10;
		ftzmpxl = (-torqueyl) / Flzu * 10;
		ftzmpyr = (torquexr) / Frzu * 10;
		ftzmpyl = (torquexl) / Flzu * 10;

		forceZr = (FZr / 10);
		forceZl = (FZl / 10);
		forceXr = (FXr / 10);
		forceXl = (FXl / 10);
		forceYr = (FYr / 10);
		forceYl = (FYl / 10);

		// printf("\n PForceXr : %lf ",FXr/10);
		// printf("\t PForceYr : %lf ",FYr/10);
		// printf("\t PForceZr : %lf ",FZr/10);
		// printf("\t\t\t\t PForceXl : %lf ",FXl/10);
		// printf("\t PForceYl : %lf ",FYl/10);
		// printf("\t PForceZl : %lf ",FZl/10);

		phaseflag = 0;
		realphaseflag = 0;

		if (Frzu == 0) {
			ftzmpxr = 0;
			ftzmpyr = 0;
		}
		if (Flzu == 0) {
			ftzmpxl = 0;
			ftzmpyl = 0;
		}
		footup = 0;
		
		if (new_phase_num == 2)//DSP_R
		{
			local_gpsrx_ssp_begin=gpsrx;
			local_gpsrz_ssp_begin=gpsrz;

			local_gpsrx=gpslx-local_gpslx_ssp_begin;
			local_gpsrz=gpslz-local_gpslz_ssp_begin;

			localFootDegreeRSspI=footdegreeR;
			localFootDegree=footdegreeL-localFootDegreeLSspI;
			// printf("localFootDegree=%lf footdegreeL=%lf localFootDegreeLSspI=%lf\n",localFootDegree, footdegreeL,localFootDegreeLSspI);
			ftzmpx = ((Flzu/10)*(ftzmpxl - pre_local_stride_F) + (Frzu/10) * ftzmpxr) / ((Frzu/10) + (Flzu/10));
			ftzmpy = ((Flzu/10)*(ftzmpyl + pre_local_stride_L) + (Frzu/10) * ftzmpyr) / ((Frzu/10) + (Flzu/10));
			// printf("222222222222222222222222222222");
		}

		else if (new_phase_num == 4)//DSP_L
		{
            local_gpslx_ssp_begin=gpslx;
			local_gpslz_ssp_begin=gpslz;
			local_gpsrx=gpsrx-local_gpsrx_ssp_begin;
			local_gpsrz=gpsrz-local_gpsrz_ssp_begin;

			localFootDegreeLSspI=footdegreeL;
			localFootDegree=footdegreeR-localFootDegreeRSspI;
			// printf("localFootDegree=%lf footdegreeR=%lf localFootDegreeRSspI=%lf\n",localFootDegree, footdegreeR,localFootDegreeRSspI);
			ftzmpx = ((Flzu/10)*(ftzmpxl)+(Frzu/10) * (ftzmpxr - pre_local_stride_F)) / ((Frzu/10) + (Flzu/10));
			// printf("ftzmpxl=%lf ftzmpxr=%lf",ftzmpxl,ftzmpxr);
			ftzmpy = ((Flzu/10)*(ftzmpyl)+(Frzu/10) * (ftzmpyr - pre_local_stride_L)) / ((Frzu/10) + (Flzu/10));

			// printf("4444@@%lf %lf\n",(Flzd/10)*ftzmpxl, (Frzd/10)*(ftzmpxr-pre_local_stride_F));
			// printf("4444444444444444444444444444444444444444");
		}
		
		else if (new_phase_num == 1)//SSP_R
		{
			double FT_denominator_lx = (Flzu/10) * (ftzmpxl + (_lsx_des - _px));
			double FT_denominator_ly = (Flzu/10) * (ftzmpyl + (_lsy_des - _py));

			local_gpsrx=gpsrx-local_gpsrx_ssp_begin;
			local_gpsrz=gpsrz-local_gpsrz_ssp_begin;
 
			localFootDegree=footdegreeR-localFootDegreeRSspI;
			
			ftzmpx = (FT_denominator_lx + (Frzu/10) * ftzmpxr) / ((Frzu/10) + (Flzu/10));
			ftzmpy = (FT_denominator_ly + (Frzu/10) * ftzmpyr) / ((Frzu/10) + (Flzu/10));
			// printf("localFootDegree=%lf footdegreeR=%lf localFootDegreeRSspI=%lf\n",localFootDegree, footdegreeR,localFootDegreeRSspI);
			// printf("111111111111111111111111111111");
		}
		else if (new_phase_num == 3)//SSP_L
		{
			
			double FT_denominator_rx = (Frzu/10) * (ftzmpxr + (_rsx_des - _px));
			double FT_denominator_ry = (Frzu/10) * (ftzmpyl + (_rsy_des - _py));
			//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!rsx des=%lf   rsy des = %lf   _py=%lf\n", _rsx_des, _rsy_des,_py);
			
			local_gpsrx=gpslx-local_gpslx_ssp_begin;
			local_gpsrz=gpslz-local_gpslz_ssp_begin;
			
			localFootDegree=footdegreeL-localFootDegreeLSspI;
			
			ftzmpx = ((Flzu/10)*(ftzmpxl)+FT_denominator_rx) / ((Frzu/10) + (Flzu/10));
			ftzmpy = ((Flzu/10)*(ftzmpyl)+FT_denominator_ry) / ((Frzu/10) + (Flzu/10));
			// printf("localFootDegree=%lf footdegreeL=%lf localFootDegreeLSspI=%lf\n",localFootDegree, footdegreeL,localFootDegreeLSspI);
			// printf("3333333333333333333333333333333");
		}
		
		phase = DSP_L;
		if (phase == DSP_L)
		{		
			new_phase_num = 4;
			phaseflag = 1;
		}

		//TorqueControl
	
		if(_sn==0)
			forwardkinematics(result_th);
		
		_theta_p.x[0][0] = result_th[0];
		_theta_p.x[1][0] = result_th[1];
		_theta_p.x[2][0] = result_th[2];
		_theta_p.x[3][0] = result_th[3];
		_theta_p.x[4][0] = result_th[4];
		_theta_p.x[5][0] = result_th[5];
		_theta_p.x[6][0] = result_th[6];
		_theta_p.x[7][0] = result_th[7];
		_theta_p.x[8][0] = result_th[8];
		_theta_p.x[9][0] = result_th[9];
		_theta_p.x[10][0] = result_th[10];
		_theta_p.x[11][0] = result_th[11];

		RR01.x[0][0] = TRB1[0][0];
		RR01.x[0][1] = TRB1[0][1];
		RR01.x[0][2] = TRB1[0][2];

		RR01.x[1][0] = TRB1[1][0];
		RR01.x[1][1] = TRB1[1][1];
		RR01.x[1][2] = TRB1[1][2];

		RR01.x[2][0] = TRB1[2][0];
		RR01.x[2][1] = TRB1[2][1];
		RR01.x[2][2] = TRB1[2][2];

		RR12.x[0][0] = TR12[0][0];
		RR12.x[0][1] = TR12[0][1];
		RR12.x[0][2] = TR12[0][2];

		RR12.x[1][0] = TR12[1][0];
		RR12.x[1][1] = TR12[1][1];
		RR12.x[1][2] = TR12[1][2];

		RR12.x[2][0] = TR12[2][0];
		RR12.x[2][1] = TR12[2][1];
		RR12.x[2][2] = TR12[2][2];

		RR23.x[0][0] = TR23[0][0];
		RR23.x[0][1] = TR23[0][1];
		RR23.x[0][2] = TR23[0][2];

		RR23.x[1][0] = TR23[1][0];
		RR23.x[1][1] = TR23[1][1];
		RR23.x[1][2] = TR23[1][2];

		RR23.x[2][0] = TR23[2][0];
		RR23.x[2][1] = TR23[2][1];
		RR23.x[2][2] = TR23[2][2];

		RR34.x[0][0] = TR34[0][0];
		RR34.x[0][1] = TR34[0][1];
		RR34.x[0][2] = TR34[0][2];

		RR34.x[1][0] = TR34[1][0];
		RR34.x[1][1] = TR34[1][1];
		RR34.x[1][2] = TR34[1][2];

		RR34.x[2][0] = TR34[2][0];
		RR34.x[2][1] = TR34[2][1];
		RR34.x[2][2] = TR34[2][2];

		RR45.x[0][0] = TR45[0][0];
		RR45.x[0][1] = TR45[0][1];
		RR45.x[0][2] = TR45[0][2];

		RR45.x[1][0] = TR45[1][0];
		RR45.x[1][1] = TR45[1][1];
		RR45.x[1][2] = TR45[1][2];

		RR45.x[2][0] = TR45[2][0];
		RR45.x[2][1] = TR45[2][1];
		RR45.x[2][2] = TR45[2][2];


		RR56.x[0][0] = TR56[0][0];
		RR56.x[0][1] = TR56[0][1];
		RR56.x[0][2] = TR56[0][2];

		RR56.x[1][0] = TR56[1][0];
		RR56.x[1][1] = TR56[1][1];
		RR56.x[1][2] = TR56[1][2];

		RR56.x[2][0] = TR56[2][0];
		RR56.x[2][1] = TR56[2][1];
		RR56.x[2][2] = TR56[2][2];


		RR67.x[0][0] = TR6E[0][0];
		RR67.x[0][1] = TR6E[0][1];
		RR67.x[0][2] = TR6E[0][2];

		RR67.x[1][0] = TR6E[1][0];
		RR67.x[1][1] = TR6E[1][1];
		RR67.x[1][2] = TR6E[1][2];

		RR67.x[2][0] = TR6E[2][0];
		RR67.x[2][1] = TR6E[2][1];
		RR67.x[2][2] = TR6E[2][2];


		RL01.x[0][0] = TLB1[0][0];
		RL01.x[0][1] = TLB1[0][1];
		RL01.x[0][2] = TLB1[0][2];

		RL01.x[1][0] = TLB1[1][0];
		RL01.x[1][1] = TLB1[1][1];
		RL01.x[1][2] = TLB1[1][2];

		RL01.x[2][0] = TLB1[2][0];
		RL01.x[2][1] = TLB1[2][1];
		RL01.x[2][2] = TLB1[2][2];

		RL12.x[0][0] = TL12[0][0];
		RL12.x[0][1] = TL12[0][1];
		RL12.x[0][2] = TL12[0][2];

		RL12.x[1][0] = TL12[1][0];
		RL12.x[1][1] = TL12[1][1];
		RL12.x[1][2] = TL12[1][2];

		RL12.x[2][0] = TL12[2][0];
		RL12.x[2][1] = TL12[2][1];
		RL12.x[2][2] = TL12[2][2];

		RL23.x[0][0] = TL23[0][0];
		RL23.x[0][1] = TL23[0][1];
		RL23.x[0][2] = TL23[0][2];

		RL23.x[1][0] = TL23[1][0];
		RL23.x[1][1] = TL23[1][1];
		RL23.x[1][2] = TL23[1][2];

		RL23.x[2][0] = TL23[2][0];
		RL23.x[2][1] = TL23[2][1];
		RL23.x[2][2] = TL23[2][2];

		RL34.x[0][0] = TL34[0][0];
		RL34.x[0][1] = TL34[0][1];
		RL34.x[0][2] = TL34[0][2];

		RL34.x[1][0] = TL34[1][0];
		RL34.x[1][1] = TL34[1][1];
		RL34.x[1][2] = TL34[1][2];

		RL34.x[2][0] = TL34[2][0];
		RL34.x[2][1] = TL34[2][1];
		RL34.x[2][2] = TL34[2][2];

		RL45.x[0][0] = TL45[0][0];
		RL45.x[0][1] = TL45[0][1];
		RL45.x[0][2] = TL45[0][2];

		RL45.x[1][0] = TL45[1][0];
		RL45.x[1][1] = TL45[1][1];
		RL45.x[1][2] = TL45[1][2];

		RL45.x[2][0] = TL45[2][0];
		RL45.x[2][1] = TL45[2][1];
		RL45.x[2][2] = TL45[2][2];


		RL56.x[0][0] = TL56[0][0];
		RL56.x[0][1] = TL56[0][1];
		RL56.x[0][2] = TL56[0][2];

		RL56.x[1][0] = TL56[1][0];
		RL56.x[1][1] = TL56[1][1];
		RL56.x[1][2] = TL56[1][2];

		RL56.x[2][0] = TL56[2][0];
		RL56.x[2][1] = TL56[2][1];
		RL56.x[2][2] = TL56[2][2];


		RL67.x[0][0] = TL6E[0][0];
		RL67.x[0][1] = TL6E[0][1];
		RL67.x[0][2] = TL6E[0][2];

		RL67.x[1][0] = TL6E[1][0];
		RL67.x[1][1] = TL6E[1][1];
		RL67.x[1][2] = TL6E[1][2];

		RL67.x[2][0] = TL6E[2][0];
		RL67.x[2][1] = TL6E[2][1];
		RL67.x[2][2] = TL6E[2][2];

		//Matrix_M33X7 RR, RL;


		RR.x[0][0] = RR01;
		RR.x[1][0] = RR12;
		RR.x[2][0] = RR23;
		RR.x[3][0] = RR34;
		RR.x[4][0] = RR45;
		RR.x[5][0] = RR56;
		RR.x[6][0] = RR67;

		RL.x[0][0] = RL01;
		RL.x[1][0] = RL12;
		RL.x[2][0] = RL23;
		RL.x[3][0] = RL34;
		RL.x[4][0] = RL45;
		RL.x[5][0] = RL56;
		RL.x[6][0] = RL67;

		//Matrix_3X3 RR10, RR21, RR32, RR43, RR54, RR65, RR76;
		//Matrix_3X3 RL10, RL21, RL32, RL43, RL54, RL65, RL76;


		RR10 = transposeMatrix_33(RR01);
		RR21 = transposeMatrix_33(RR12);
		RR32 = transposeMatrix_33(RR23);
		RR43 = transposeMatrix_33(RR34);
		RR54 = transposeMatrix_33(RR45);
		RR65 = transposeMatrix_33(RR56);
		RR76 = transposeMatrix_33(RR67);

		RL10 = transposeMatrix_33(RL01);
		RL21 = transposeMatrix_33(RL12);
		RL32 = transposeMatrix_33(RL23);
		RL43 = transposeMatrix_33(RL34);
		RL54 = transposeMatrix_33(RL45);
		RL65 = transposeMatrix_33(RL56);
		RL76 = transposeMatrix_33(RL67);
		//Matrix_M33X7 invRR, invRL;

		invRR.x[0][0] = RR10;
		invRR.x[1][0] = RR21;
		invRR.x[2][0] = RR32;
		invRR.x[3][0] = RR43;
		invRR.x[4][0] = RR54;
		invRR.x[5][0] = RR65;
		invRR.x[6][0] = RR76;

		invRL.x[0][0] = RL10;
		invRL.x[1][0] = RL21;
		invRL.x[2][0] = RL32;
		invRL.x[3][0] = RL43;
		invRL.x[4][0] = RL54;
		invRL.x[5][0] = RL65;
		invRL.x[6][0] = RL76;

		PR01.x[0][0] = TRB1[0][3];
		PR01.x[1][0] = TRB1[1][3];
		PR01.x[2][0] = TRB1[2][3];
	
		PR12.x[0][0] = TR12[0][3];
		PR12.x[1][0] = TR12[1][3];
		PR12.x[2][0] = TR12[2][3];

		PR23.x[0][0] = TR23[0][3];
		PR23.x[1][0] = TR23[1][3];
		PR23.x[2][0] = TR23[2][3];

		PR34.x[0][0] = TR34[0][3];
		PR34.x[1][0] = TR34[1][3];
		PR34.x[2][0] = TR34[2][3];

		PR45.x[0][0] = TR45[0][3];
		PR45.x[1][0] = TR45[1][3];
		PR45.x[2][0] = TR45[2][3];
	
		PR56.x[0][0] = TR56[0][3];
		PR56.x[1][0] = TR56[1][3];
		PR56.x[2][0] = TR56[2][3];

		PR67.x[0][0] = TR6E[0][3];
		PR67.x[1][0] = TR6E[1][3];
		PR67.x[2][0] = TR6E[2][3];
	

		PL01.x[0][0] = TLB1[0][3];
		PL01.x[1][0] = TLB1[1][3];
		PL01.x[2][0] = TLB1[2][3];
	
		PL12.x[0][0] = TL12[0][3];
		PL12.x[1][0] = TL12[1][3];
		PL12.x[2][0] = TL12[2][3];

		PL23.x[0][0] = TL23[0][3];
		PL23.x[1][0] = TL23[1][3];
		PL23.x[2][0] = TL23[2][3];

		PL34.x[0][0] = TL34[0][3];
		PL34.x[1][0] = TL34[1][3];
		PL34.x[2][0] = TL34[2][3];

		PL45.x[0][0] = TL45[0][3];
		PL45.x[1][0] = TL45[1][3];
		PL45.x[2][0] = TL45[2][3];
	
		PL56.x[0][0] = TL56[0][3];
		PL56.x[1][0] = TL56[1][3];
		PL56.x[2][0] = TL56[2][3];

		PL67.x[0][0] = TL6E[0][3];
		PL67.x[1][0] = TL6E[1][3];
		PL67.x[2][0] = TL6E[2][3];
	
		//Matrix_M31X7 PositionppR;

		PositionppR.x[0][0] = PR01;
		PositionppR.x[0][1] = PR12;
		PositionppR.x[0][2] = PR23;
		PositionppR.x[0][3] = PR34;
		PositionppR.x[0][4] = PR45;
		PositionppR.x[0][5] = PR56;
		PositionppR.x[0][6] = PR67;
	
		//Matrix_M31X7 PositionppL;

		PositionppL.x[0][0] = PL01;
		PositionppL.x[0][1] = PL12;
		PositionppL.x[0][2] = PL23;
		PositionppL.x[0][3] = PL34;
		PositionppL.x[0][4] = PL45;
		PositionppL.x[0][5] = PL56;
		PositionppL.x[0][6] = PL67;

		double poly_th_i[12]={0,0,0,0,0,0,0,0,0,0,0,0};
		// double poly_th_f[12]= {-0.005136 ,0.825219, -1.825708, 1.000489 ,0.005136, 0.000000 ,0.000000, -0.005136, -1.000489, 1.825708, -0.825219 ,0.005136 };


		double poly_th_f[12]= {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
		//{ -0.000940,  0.917193,  -1.831663,  0.914470,  0.000940,  0.000000,  0.000000,  -0.000940,  -0.914470,  1.831663,  -0.917193,  0.000940 };

		double poly_dth_i[12]={0,0,0,0,0,0,0,0,0,0,0,0};
		double poly_dth_f[12]={0,0,0,0,0,0,0,0,0,0,0,0};

		double poly_ddth_i[12]={0,0,0,0,0,0,0,0,0,0,0,0};
		double poly_ddth_f[12]={0,0,0,0,0,0,0,0,0,0,0,0};
		double _tf=(ps_stand/2)*samplingtime;

		double poly_t = (i-1)*samplingtime;

		if(i<(ps_stand/2)+1){
			highpoly(poly_th_i,poly_th_f,poly_dth_i,poly_dth_f,poly_ddth_i,poly_ddth_f, _tf, poly_t, result_th);

#if (torqueModeStand==0)
			mMotors[ankleRollR]->setPosition(-result_th[0]);
			mMotors[anklePitchR]->setPosition(-result_th[1]);
			mMotors[kneePitchR]->setPosition(result_th[2]);
			mMotors[hipPitchR]->setPosition(result_th[3]);
			mMotors[hipRollR]->setPosition(result_th[4]);
			mMotors[hipYawR]->setPosition(result_th[5]);
			mMotors[hipYawL]->setPosition(-result_th[6]);
			mMotors[hipRollL]->setPosition(-result_th[7]);
			mMotors[hipPitchL]->setPosition(result_th[8]);
			mMotors[kneePitchL]->setPosition(result_th[9]);
			mMotors[anklePitchL]->setPosition(-result_th[10]);
			mMotors[ankleRollL]->setPosition(result_th[11]);

			myStep();
#endif
		}
		else{
			result_th[0]=_th_position[0];
			result_th[1]=_th_position[1];
			result_th[2]=_th_position[2];
			result_th[3]=_th_position[3];
			result_th[4]=_th_position[4];
			result_th[5]=_th_position[5];
			result_th[6]=_th_position[6];
			result_th[7]=_th_position[7];
			result_th[8]=_th_position[8];
			result_th[9]=_th_position[9];
			result_th[10]=_th_position[10];
			result_th[11]=_th_position[11];
#if (torqueModeStand==0)
			mMotors[ankleRollR]->setPosition(-_th_position[0]);
			mMotors[anklePitchR]->setPosition(-_th_position[1]);
			mMotors[kneePitchR]->setPosition(_th_position[2]);
			mMotors[hipPitchR]->setPosition(_th_position[3]);
			mMotors[hipRollR]->setPosition(_th_position[4]);
			mMotors[hipYawR]->setPosition(_th_position[5]);
			mMotors[hipYawL]->setPosition(-_th_position[6]);
			mMotors[hipRollL]->setPosition(-_th_position[7]);
			mMotors[hipPitchL]->setPosition(_th_position[8]);
			mMotors[kneePitchL]->setPosition(_th_position[9]);
			mMotors[anklePitchL]->setPosition(-_th_position[10]);
			mMotors[ankleRollL]->setPosition(_th_position[11]);

			myStep();
#endif
		}
		forwardkinematics(result_th);



		ddq_Jdot_R_p.x[0][0] = 0.0;
		ddq_Jdot_R_p.x[1][0] = 0.0;
		ddq_Jdot_R_p.x[2][0] = 0.0;
		ddq_Jdot_R_p.x[3][0] = 0.0;
		ddq_Jdot_R_p.x[4][0] = 0.0;
		ddq_Jdot_R_p.x[5][0] = 0.0;
  
  
		ddq_Jdot_R_p.x[0][1] = 0.0;
		ddq_Jdot_R_p.x[1][1] = 0.0;
		ddq_Jdot_R_p.x[2][1] = 0.0;
		ddq_Jdot_R_p.x[3][1] = 0.0;
		ddq_Jdot_R_p.x[4][1] = 0.0;
		ddq_Jdot_R_p.x[5][1] = 0.0;


		ddq_Jdot_R_p.x[0][2] = 0.0;
		ddq_Jdot_R_p.x[1][2] = 0.0;
		ddq_Jdot_R_p.x[2][2] = 0.0;
		ddq_Jdot_R_p.x[3][2] = 0.0;
		ddq_Jdot_R_p.x[4][2] = 0.0;
		ddq_Jdot_R_p.x[5][2] = 0.0;


		ddq_Jdot_R_p.x[0][3] = 0.0;
		ddq_Jdot_R_p.x[1][3] = 0.0;
		ddq_Jdot_R_p.x[2][3] = 0.0;
		ddq_Jdot_R_p.x[3][3] = 0.0;
		ddq_Jdot_R_p.x[4][3] = 0.0;
		ddq_Jdot_R_p.x[5][3] = 0.0;

  
		ddq_Jdot_R_p.x[0][4] = 0.0;
		ddq_Jdot_R_p.x[1][4] = 0.0;
		ddq_Jdot_R_p.x[2][4] = 0.0;
		ddq_Jdot_R_p.x[3][4] = 0.0;
		ddq_Jdot_R_p.x[4][4] = 0.0;
		ddq_Jdot_R_p.x[5][4] = 0.0;


		ddq_Jdot_R_p.x[0][5] = 0.0;
		ddq_Jdot_R_p.x[1][5] = 0.0;
		ddq_Jdot_R_p.x[2][5] = 0.0;
		ddq_Jdot_R_p.x[3][5] = 0.0;
		ddq_Jdot_R_p.x[4][5] = 0.0;
		ddq_Jdot_R_p.x[5][5] = 0.0;

		  
		_w_LE_n.x[0][0]=0;
		_w_LE_n.x[1][0]=0;
		_w_LE_n.x[2][0]=0;


		_w_RE_n.x[0][0]=0;
		_w_RE_n.x[1][0]=0;
		_w_RE_n.x[2][0]=0;


		_dw_LE_n.x[0][0]=0;
		_dw_LE_n.x[1][0]=0;
		_dw_LE_n.x[2][0]=0;


		_dw_RE_n.x[0][0]=0;
		_dw_RE_n.x[1][0]=0;
		_dw_RE_n.x[2][0]=0;


		Mdth_d.x[0][0]=0.0;
		Mdth_d.x[1][0]=0.0;
		Mdth_d.x[2][0]=0.0;
		Mdth_d.x[3][0]=0.0;
		Mdth_d.x[4][0]=0.0;
		Mdth_d.x[5][0]=0.0;
		Mdth_d.x[6][0]=0.0;
		Mdth_d.x[7][0]=0.0;
		Mdth_d.x[8][0]=0.0;
		Mdth_d.x[9][0]=0.0;
		Mdth_d.x[10][0]=0.0;
		Mdth_d.x[11][0]=0.0;

		_th_encoder[0] = -mPositionSensors[ankleRollR]->getValue();
		_th_encoder[1] = -mPositionSensors[anklePitchR]->getValue();
		_th_encoder[2] = mPositionSensors[kneePitchR]->getValue();
		_th_encoder[3] = mPositionSensors[hipPitchR]->getValue();
		_th_encoder[4] = mPositionSensors[hipRollR]->getValue();
		_th_encoder[5] = mPositionSensors[hipYawR]->getValue();
		_th_encoder[6] = -mPositionSensors[hipYawL]->getValue();
		_th_encoder[7] = -mPositionSensors[hipRollL]->getValue();
		_th_encoder[8] = mPositionSensors[hipPitchL]->getValue();
		_th_encoder[9] = mPositionSensors[kneePitchL]->getValue();
		_th_encoder[10] = -mPositionSensors[anklePitchL]->getValue();
		_th_encoder[11] = mPositionSensors[ankleRollL]->getValue();

		if (_sn==0)
		{
			_th_current_p =_theta_p;
		}

		else{
			_th_current_p =_th_current_n;
		}

		_th_current_n.x[0][0]=_th_encoder[0];
		_th_current_n.x[1][0]=_th_encoder[1];
		_th_current_n.x[2][0]=_th_encoder[2];
		_th_current_n.x[3][0]=_th_encoder[3];
		_th_current_n.x[4][0]=_th_encoder[4];
		_th_current_n.x[5][0]=_th_encoder[5];
		_th_current_n.x[6][0]=_th_encoder[6];
		_th_current_n.x[7][0]=_th_encoder[7];
		_th_current_n.x[8][0]=_th_encoder[8];
		_th_current_n.x[9][0]=_th_encoder[9];
		_th_current_n.x[10][0]=_th_encoder[10];
		_th_current_n.x[11][0]=_th_encoder[11];

		if (_sn==0)
		{
			Mdth.x[0][0]=0.0;
			Mdth.x[1][0]=0.0;
			Mdth.x[2][0]=0.0;
			Mdth.x[3][0]=0.0;
			Mdth.x[4][0]=0.0;
			Mdth.x[5][0]=0.0;
			Mdth.x[6][0]=0.0;
			Mdth.x[7][0]=0.0;
			Mdth.x[8][0]=0.0;
			Mdth.x[9][0]=0.0;
			Mdth.x[10][0]=0.0;
			Mdth.x[11][0]=0.0;
		}

		else
		{
			Mdth = s_vectorminus121_121(_th_current_n, _th_current_p);
		}

		// double Tr_anklePitch=0.015;//0;//0.02;//
		// double _kv_anklePitch=2*(1.8/Tr_anklePitch);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
		// double _kp_anklePitch=(1.8/Tr_anklePitch)*(1.8/Tr_anklePitch);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
		// double Tr_ankleRoll=0.01;//0;//0.02;//
		// double _kv_ankleRoll=2*(1.8/Tr_ankleRoll);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
		// double _kp_ankleRoll=(1.8/Tr_ankleRoll)*(1.8/Tr_ankleRoll);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
		_error_dth_1=vectorminus121_121(Mdth_d,Mdth);
		_error_dth=productmatrix121_d(_kv,_error_dth_1);
		// _error_dth.x[1][0]=_error_dth.x[1][0]/_kv*_kv_anklePitch;
		// _error_dth.x[10][0]=_error_dth.x[10][0]/_kv*_kv_anklePitch;
		// _error_dth.x[0][0]=_error_dth.x[0][0]/_kv*_kv_ankleRoll;
		// _error_dth.x[11][0]=_error_dth.x[11][0]/_kv*_kv_ankleRoll;

		_error_th_1=vectorminus121_121(_theta_p,_th_current_n);
		_error_th=productmatrix121_d(_kp,_error_th_1);
		// _error_th.x[1][0]=_error_th.x[1][0]/_kp*_kp_anklePitch;
		// _error_th.x[10][0]=_error_th.x[10][0]/_kp*_kp_anklePitch;
		// _error_th.x[0][0]=_error_th.x[0][0]/_kp*_kp_ankleRoll;
		// _error_th.x[11][0]=_error_th.x[11][0]/_kp*_kp_ankleRoll;

		Mddth =vectorplus121_121_121(Mddth_d,_error_dth,_error_th);
		
		//RNE 초기 조건
		wr00.x[0][0] = 0;
		wr00.x[1][0] = 0;
		wr00.x[2][0] = 0;

		dwr00.x[0][0] = 0;
		dwr00.x[1][0] = 0;
		dwr00.x[2][0] = 0;

		dvr00.x[0][0] = 0;
		dvr00.x[1][0] = 0;
		dvr00.x[2][0] = 9.81; //0

		dvcr00.x[0][0] = 0;
		dvcr00.x[1][0] = 0;
		dvcr00.x[2][0] = 0;

		//Matrix_3X1 Fr00, Nr00, fr77, nr77;

		Fr00.x[0][0] = 0;
		Fr00.x[1][0] = 0;
		Fr00.x[2][0] = 0;

		Nr00.x[0][0] = 0;
		Nr00.x[1][0] = 0;
		Nr00.x[2][0] = 0;

		//f,n 


		fr77.x[0][0] =  0;
		fr77.x[1][0] =  0;
		fr77.x[2][0] =  0;//-11.672488;//-ps_RforceZ_last;//-force_F_R[2];//-ps_RforceZ_last;//-forceZr;//-force_F_R[2];//-d_Fz_DSP;//
		// printf("\nforce_F_R[2]=%d",force_F_R[2]);


		nr77.x[0][0] =  0;//0.142319;//-ps_RtorqueX_last;//-torque_F_R[0];//-ps_RtorqueX_last;//-torquexr;//-torque_F_R[0];//0;//
		nr77.x[1][0] =  0;//0.053947;//-ps_RtorqueY_last;//-torque_F_R[1];//-ps_RtorqueY_last;//-torqueyr;//-torque_F_R[1];//0;//
		nr77.x[2][0] =0;
		// printf("\nR : fz=%lf tx=%lf ty=%lf",ps_RforceZ_last,ps_RtorqueX_last,ps_RtorqueY_last);


		//RNE
		//outward iteration
		//Matrix_M31X7 RNEwr, RNEdwr, RNEdvr, RNEdvcr, RNEFr, RNENr, RNEfr, RNEnr;




		RNEwr.x[0][0] = wr00;
		RNEdwr.x[0][0] = dwr00;
		RNEdvr.x[0][0] = dvr00;
		RNEdvcr.x[0][0] = dvcr00;
		RNEFr.x[0][0] = Fr00;
		RNENr.x[0][0] = Nr00;

		RNEfr.x[0][6] = fr77;
		RNEnr.x[0][6] = nr77;


		for (k = 0; k < 6; k++) { ///outward iteration

				rotationr = invRR.x[k][0]; //3x3
				omegar = RNEwr.x[0][k]; //3x1
				productwr = productmatrix33_31(rotationr, omegar);//3x1

				ta_zr=ta.x[0][5-k];//3x1
				wr_m2_1=Mdth.x[5-k][0];//1x1(double)
				wr_m2 = productmatrix11_31(wr_m2_1, ta_zr);//3x1


				RNEwr.x[0][k + 1] = vectorplus31_31(productwr, wr_m2); // 3x1


				domegar = RNEdwr.x[0][k];//3x1
				productdwr1 = productmatrix33_31(rotationr, domegar);//3x1

				cproductdwr1 = crossproductmatrix31_31(productwr, wr_m2);//3x1


				ta_zr=ta.x[0][5-k];
				dwr_m2_1=Mddth.x[5-k][0];
				dwr_m2 = productmatrix11_31(dwr_m2_1, ta_zr);//3x1

				RNEdwr.x[0][k + 1] = vectorplus31_31_31(productdwr1, cproductdwr1, dwr_m2);//3x1

				vpr1 = crossproductmatrix31_31(domegar, PositionppR.x[0][k]);//3x1
				vpr2_1 = crossproductmatrix31_31(omegar, PositionppR.x[0][k]);//3x1
				vpr2 = crossproductmatrix31_31(omegar, vpr2_1);//3x1
				vpr3 = RNEdvr.x[0][k];//3x1
				vectorplusdvr1 = vectorplus31_31_31(vpr1, vpr2, vpr3);//3x1

				RNEdvr.x[0][k + 1] = productmatrix33_31(rotationr, vectorplusdvr1);//3x1

				positionCr = PositionC.x[0][5-k];//3x1
				Rdvcr1 = crossproductmatrix31_31(RNEdwr.x[0][k + 1], positionCr);//3x1
				Rdvcr2_1 = crossproductmatrix31_31(RNEwr.x[0][k + 1], positionCr);//3x1
				Rdvcr2 = crossproductmatrix31_31(RNEwr.x[0][k + 1], Rdvcr2_1);//3x1

				RNEdvcr.x[0][k + 1] = vectorplus31_31_31(Rdvcr1, Rdvcr2, RNEdvr.x[0][k + 1]);//3x1

				helpRNEdvcr = RNEdvcr.x[0][k + 1];//3x1
				helpRNEFr.x[0][0] = RNEmass[5-k] * helpRNEdvcr.x[0][0];
				helpRNEFr.x[1][0] = RNEmass[5-k] * helpRNEdvcr.x[1][0];
				helpRNEFr.x[2][0] = RNEmass[5-k] * helpRNEdvcr.x[2][0];

				RNEFr.x[0][k + 1] = helpRNEFr;//3x1

				RNENr_1_1 = Inertia.x[0][5-k];//3x3
				RNENr_1 = productmatrix33_31(RNENr_1_1, RNEdwr.x[0][k + 1]);//3x1
				RNENr_2 = productmatrix33_31(RNENr_1_1, RNEwr.x[0][k + 1]);//3x1
				RNENr_3 = crossproductmatrix31_31(RNEwr.x[0][k + 1], RNENr_2);//3x1

				RNENr.x[0][k + 1] = vectorplus31_31(RNENr_1, RNENr_3);//3x1


			}


			for (k = 6; k > 0; k = k - 1) {  ///inward iteration

				RNEfr_1 = productmatrix33_31(RR.x[k][0], RNEfr.x[0][k]);//3x1
				RNEfr.x[0][k - 1] = vectorplus31_31(RNEfr_1, RNEFr.x[0][k]);//3x1

				inwardpositionCr = PositionC.x[0][6-k];//3x1
				RNEnr_1 = productmatrix33_31(RR.x[k][0], RNEnr.x[0][k]);//3x1
				RNEnr_2 = crossproductmatrix31_31(inwardpositionCr, RNEFr.x[0][k]);//3x1
				RNEnr_3 = crossproductmatrix31_31(PositionppR.x[0][k], RNEfr_1);//3x1

				RNEnr.x[0][k - 1] = vectorplus31_31_31_31(RNENr.x[0][k], RNEnr_1, RNEnr_2, RNEnr_3);//3x1


				helpFinaltorquer = RNEnr.x[0][k - 1];//3x1
				helpFinaltorquer_1= transposeMatrix_31(helpFinaltorquer);//1x3
				ta_zr_1 = ta.x[0][6-k];//3x1
				Finaltorque.x[6-k][0] = productmatrix13_31(helpFinaltorquer_1,ta_zr_1);//double
			}



		//RNE 초기 조건 
		wl00.x[0][0] = 0;
		wl00.x[1][0] = 0;
		wl00.x[2][0] = 0;

		dwl00.x[0][0] = 0;
		dwl00.x[1][0] = 0;
		dwl00.x[2][0] = 0;

		dvl00.x[0][0] = 0;
		dvl00.x[1][0] = 0;
		dvl00.x[2][0] = 9.81; //0

		dvcl00.x[0][0] = 0;
		dvcl00.x[1][0] = 0;
		dvcl00.x[2][0] = 0;
		//Matrix_3X1 Fl00, Nl00, fl77, nl77;

		Fl00.x[0][0] = 0;
		Fl00.x[1][0] = 0;
		Fl00.x[2][0] = 0;

		Nl00.x[0][0] = 0;
		Nl00.x[1][0] = 0;
		Nl00.x[2][0] = 0;

		//f,n
		fl77.x[0][0] = 0;
		fl77.x[1][0] = 0;
		fl77.x[2][0] = 0;//-11.884312;//-ps_LforceZ_last;//-force_F_L[2];//-ps_LforceZ_last;//-forceZl;//-force_F_L[2];//-d_Fz_DSP;//

		nl77.x[0][0] = 0;//-0.155095;//-ps_LtorqueX_last;//-torque_F_L[0];//-ps_LtorqueX_last;//-torquexl;//-torque_F_L[0];//0;//
		nl77.x[1][0] = 0;//0.054012;//-ps_LtorqueY_last;//-torque_F_L[1];//-ps_LtorqueY_last;//-torqueyl;//-torque_F_L[1];//0;//
		nl77.x[2][0] = 0;
		// printf("\nL : fz=%lf tx=%lf ty=%lf",ps_LforceZ_last,ps_LtorqueX_last,ps_LtorqueY_last);

		RNEwl.x[0][0] = wl00;
		RNEdwl.x[0][0] = dwl00;
		RNEdvl.x[0][0] = dvl00;
		RNEdvcl.x[0][0] = dvcl00;
		RNEFl.x[0][0] = Fl00;
		RNENl.x[0][0] = Nl00;

		RNEfl.x[0][6] = fl77;
		RNEnl.x[0][6] = nl77;

		// B->R
			for (k = 0; k < 6; k++) { ///outward iteration

				rotationl = invRL.x[k][0]; //3x3
				omegal = RNEwl.x[0][k]; //3x1
				productwl = productmatrix33_31(rotationl, omegal);//3x1

				ta_zl=ta.x[0][k+6];//3x1
				wl_m2_1=Mdth.x[k+6][0];//double
				wl_m2 = productmatrix11_31(wl_m2_1, ta_zl);//3x1

				RNEwl.x[0][k + 1] = vectorplus31_31(productwl, wl_m2); // w(i+1)

				domegal = RNEdwl.x[0][k];//3x1
				productdwl1 = productmatrix33_31(rotationl, domegal);//3x1

				cproductdwl1 = crossproductmatrix31_31(productwl, wl_m2);//3x1

				ta_zl=ta.x[0][k+6];//3x1
				dwl_m2_1=Mddth.x[k+6][0];//double
				dwl_m2 = productmatrix11_31(dwl_m2_1, ta_zl);

				RNEdwl.x[0][k + 1] = vectorplus31_31_31(productdwl1, cproductdwl1, dwl_m2);//3x1

				vpl1 = crossproductmatrix31_31(domegal, PositionppL.x[0][k]);//3x1
				vpl2_1 = crossproductmatrix31_31(omegal, PositionppL.x[0][k]);//3x1
				vpl2 = crossproductmatrix31_31(omegal, vpl2_1);//3x1
				vpl3 = RNEdvl.x[0][k];//3x1
				vectorplusdvl1 = vectorplus31_31_31(vpl1, vpl2, vpl3);//3x1

				RNEdvl.x[0][k + 1] = productmatrix33_31(rotationl, vectorplusdvl1);//3x1

				positionCl = PositionC.x[0][k+6];//3x1
				Rdvcl1 = crossproductmatrix31_31(RNEdwl.x[0][k + 1], positionCl);//3x1
				Rdvcl2_1 = crossproductmatrix31_31(RNEwl.x[0][k + 1], positionCl);//3x1
				Rdvcl2 = crossproductmatrix31_31(RNEwl.x[0][k + 1], Rdvcl2_1);//3x1

				RNEdvcl.x[0][k + 1] = vectorplus31_31_31(Rdvcl1, Rdvcl2, RNEdvl.x[0][k + 1]);//3x1

				helpRNEdvcl = RNEdvcl.x[0][k + 1];//3x1
				helpRNEFl.x[0][0] = RNEmass[k+6] * helpRNEdvcl.x[0][0];//m*dvc
				helpRNEFl.x[1][0] = RNEmass[k+6] * helpRNEdvcl.x[1][0];
				helpRNEFl.x[2][0] = RNEmass[k+6] * helpRNEdvcl.x[2][0];

				RNEFl.x[0][k + 1] = helpRNEFl;//3x1

				RNENl_1_1 = Inertia.x[0][k+6];//3x3
				RNENl_1 = productmatrix33_31(RNENl_1_1, RNEdwl.x[0][k + 1]);//3x1
				RNENl_2 = productmatrix33_31(RNENl_1_1, RNEwl.x[0][k + 1]);//3x1
				RNENl_3 = crossproductmatrix31_31(RNEwl.x[0][k + 1], RNENl_2);//3x1

				RNENl.x[0][k + 1] = vectorplus31_31(RNENl_1, RNENl_3);//3x1
			}
			for (k = 6; k > 0; k = k - 1) {  ///inward iteration

				RNEfl_1 = productmatrix33_31(RL.x[k][0], RNEfl.x[0][k]);//3x1
				RNEfl.x[0][k - 1] = vectorplus31_31(RNEfl_1, RNEFl.x[0][k]);//3x1

				inwardpositionCl = PositionC.x[0][k+5];//3x1
				RNEnl_1 = productmatrix33_31(RL.x[k][0], RNEnl.x[0][k]);//3x1
				RNEnl_2 = crossproductmatrix31_31(inwardpositionCl, RNEFl.x[0][k]);//3x1
				RNEnl_3 = crossproductmatrix31_31(PositionppL.x[0][k], RNEfl_1);//3x1

				RNEnl.x[0][k - 1] = vectorplus31_31_31_31(RNENl.x[0][k], RNEnl_1, RNEnl_2, RNEnl_3);//3x1

			/*
			helpFinaltorque = RNEn.x[0][i - 1];//n(i)-1
			Finaltorque.x[6-i][0] = productmatrix11_31(helpFinaltorque_1,ta_z);
			*/

				helpFinaltorquel = RNEnl.x[0][k - 1];//3x1
				helpFinaltorquel_1= transposeMatrix_31(helpFinaltorquel);//1x3
				ta_zl_1 = ta.x[0][k+5];//3x1
				Finaltorque.x[k+5][0] = productmatrix13_31(helpFinaltorquel_1,ta_zl_1);//double
				// printf("Finaltorque.x[%d][0]=%lf",k+5,Finaltorque.x[k+5][0]);
			}
		torque1 = Finaltorque.x[0][0];
		torque2 = Finaltorque.x[1][0];
		torque3 = Finaltorque.x[2][0];
		torque4 = Finaltorque.x[3][0];
		torque5 = Finaltorque.x[4][0];
		torque6 = Finaltorque.x[5][0];
		torque7 = Finaltorque.x[6][0];
		torque8 = Finaltorque.x[7][0];
		torque9 = Finaltorque.x[8][0];
		torque10 = Finaltorque.x[9][0];
		torque11 = Finaltorque.x[10][0];
		torque12 = Finaltorque.x[11][0];
	
		
		 double torque[12]={torque1,torque2,torque3,torque4,torque5,torque6,torque7,torque8,torque9,torque10,torque11,torque12};


		// printf("\nankleroll  /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:-R=%6lf ,L=%6lf",_th_torque[0]*180/pi, _th_torque[11]*180/pi,_th_encoder[0]*180/pi,_th_encoder[11]*180/pi,_error_th_1.x[0][0],_error_th_1.x[11][0],_error_dth_1.x[0][0],_error_dth_1.x[11][0],-torque[0],torque[11]);
		// printf("\nanklepitch /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:-R=%6lf ,-L=%6lf",_th_torque[1]*180/pi, _th_torque[10]*180/pi,_th_encoder[1]*180/pi,_th_encoder[10]*180/pi,_error_th_1.x[1][0],_error_th_1.x[10][0],_error_dth_1.x[1][0],_error_dth_1.x[10][0],-torque[1],-torque[10]);
		// printf("\nkneepitch  /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,L=%6lf",_th_torque[2]*180/pi, _th_torque[9]*180/pi,_th_encoder[2]*180/pi,_th_encoder[9]*180/pi,_error_th_1.x[2][0],_error_th_1.x[9][0],_error_dth_1.x[2][0],_error_dth_1.x[9][0],torque[2],torque[9]);
		// printf("\nhippitch   /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,L=%6lf",_th_torque[3]*180/pi, _th_torque[8]*180/pi,_th_encoder[3]*180/pi,_th_encoder[8]*180/pi,_error_th_1.x[3][0],_error_th_1.x[8][0],_error_dth_1.x[3][0],_error_dth_1.x[8][0],torque[3],torque[8]);
		// printf("\nhiproll    /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,-L=%6lf",_th_torque[4]*180/pi, _th_torque[7]*180/pi,_th_encoder[4]*180/pi,_th_encoder[7]*180/pi,_error_th_1.x[4][0],_error_th_1.x[7][0],_error_dth_1.x[4][0],_error_dth_1.x[7][0],torque[4],-torque[7]);
		// printf("\nhipyaw     /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,-L=%6lf",_th_torque[5]*180/pi, _th_torque[6]*180/pi,_th_encoder[5]*180/pi,_th_encoder[6]*180/pi,_error_th_1.x[5][0],_error_th_1.x[6][0],_error_dth_1.x[5][0],_error_dth_1.x[6][0],torque[5],-torque[6]);
		
#if torqueModeStand
		mMotors[ankleRollR]->setTorque(-torque[0]);
		mMotors[anklePitchR]->setTorque(-torque[1]);
		mMotors[kneePitchR]->setTorque(torque[2]);
		mMotors[hipPitchR]->setTorque(torque[3]);
		mMotors[hipRollR]->setTorque(torque[4]);
		mMotors[hipYawR]->setTorque(torque[5]);
		mMotors[hipYawL]->setTorque(-torque[6]);
		mMotors[hipRollL]->setTorque(-torque[7]);
		mMotors[hipPitchL]->setTorque(torque[8]);
		mMotors[kneePitchL]->setTorque(torque[9]);
		mMotors[anklePitchL]->setTorque(-torque[10]);
		mMotors[ankleRollL]->setTorque(torque[11]);
#endif
		pre_force_R[0]=forceXr;
		pre_force_R[1]=forceYr;
		pre_force_R[2]=forceZr;
	
		pre_force_L[0]=forceXl;
		pre_force_L[1]=forceYl;
		pre_force_L[2]=forceZl;
	
	
		pre_torque_R[0]=torquexr;
		pre_torque_R[1]=torqueyr;
		pre_torque_R[2]=torquezr;
	
		pre_torque_L[0]=torquexl;
		pre_torque_L[1]=torqueyl;
		pre_torque_L[2]=torquezl;
	
		pre_force_F_R[0]=force_F_R[0];
		pre_force_F_R[1]=force_F_R[1];
		pre_force_F_R[2]=force_F_R[2];
	
		pre_force_F_L[0]=force_F_L[0];
		pre_force_F_L[1]=force_F_L[1];
		pre_force_F_L[2]=force_F_L[2];
	
	
		pre_torque_F_R[0]= torque_F_R[0];
		pre_torque_F_R[1]= torque_F_R[1];
		pre_torque_F_R[2]= torque_F_R[2];
	
		pre_torque_F_L[0]= torque_F_L[0];
		pre_torque_F_L[1]= torque_F_L[1];
		pre_torque_F_L[2]= torque_F_L[2];

		if(i==2){
             
			force_F_R[0]=forceXr;
			force_F_R[1]=forceYr;
			force_F_R[2]=forceZr;
		
			force_F_L[0]=forceXl;
			force_F_L[1]=forceYl;
			force_F_L[2]=forceZl;
		
		
			torque_F_R[0]=torquexr;
			torque_F_R[1]=torqueyr;
			torque_F_R[2]=torquezr;
		
			torque_F_L[0]=torquexl;
			torque_F_L[1]=torqueyl;
			torque_F_L[2]=torquezl;
		
		}
		
		else if(i>=3){
			LPF(10,pre_force_R,pre_force_F_R,force_F_R);
			LPF(10,pre_force_L,pre_force_F_L,force_F_L);
		
			LPF(10,pre_torque_R,pre_torque_F_R,torque_F_R);
			LPF(10,pre_torque_L,pre_torque_F_L,torque_F_L);
		}
		
		else{
		
			force_F_R[0]=forceXr;
			force_F_R[1]=forceYr;
			force_F_R[2]=forceZr;
		
			force_F_L[0]=forceXl;
			force_F_L[1]=forceYl;
			force_F_L[2]=forceZl;
		
		
			torque_F_R[0]=torquexr;
			torque_F_R[1]=torqueyr;
			torque_F_R[2]=torquezr;
		
			torque_F_L[0]=torquexl;
			torque_F_L[1]=torqueyl;
			torque_F_L[2]=torquezl;
		}

		if(i>(ps_stand/2)+1){
			ps_sensing_count++;
			ps_RforceZ_avg +=forceZr;
			ps_RtorqueX_avg+=torquexr;
			ps_RtorqueY_avg+=torqueyr;
			ps_LforceZ_avg +=forceZl;
			ps_LtorqueX_avg+=torquexl;
			ps_LtorqueY_avg+=torqueyl;
		}
		ps_RforceZ_last =forceZr;
		ps_RtorqueX_last=torquexr;
		ps_RtorqueY_last=torqueyr;
		ps_LforceZ_last =forceZl;
		ps_LtorqueX_last=torquexl;
		ps_LtorqueY_last=torqueyl;

		_th_encoder[0] = -mPositionSensors[ankleRollR]->getValue();
		_th_encoder[1] = -mPositionSensors[anklePitchR]->getValue();
		_th_encoder[2] = mPositionSensors[kneePitchR]->getValue();
		_th_encoder[3] = mPositionSensors[hipPitchR]->getValue();
		_th_encoder[4] = mPositionSensors[hipRollR]->getValue();
		_th_encoder[5] = mPositionSensors[hipYawR]->getValue();
		_th_encoder[6] = -mPositionSensors[hipYawL]->getValue();
		_th_encoder[7] = -mPositionSensors[hipRollL]->getValue();
		_th_encoder[8] = mPositionSensors[hipPitchL]->getValue();
		_th_encoder[9] = mPositionSensors[kneePitchL]->getValue();
		_th_encoder[10] = -mPositionSensors[anklePitchL]->getValue();
		_th_encoder[11] = mPositionSensors[ankleRollL]->getValue();

		// printf("\nankleroll  /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf",_th_torque[0]*180/pi, _th_torque[11]*180/pi,_th_encoder[0]*180/pi,_th_encoder[11]*180/pi,_error_th_1.x[0][0],_error_th_1.x[11][0]);
		// printf("\nanklepitch /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf",_th_torque[1]*180/pi, _th_torque[10]*180/pi,_th_encoder[1]*180/pi,_th_encoder[10]*180/pi,_error_th_1.x[1][0],_error_th_1.x[10][0]);
		// printf("\nkneepitch  /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf",_th_torque[2]*180/pi, _th_torque[9]*180/pi,_th_encoder[2]*180/pi,_th_encoder[9]*180/pi,_error_th_1.x[2][0],_error_th_1.x[9][0]);
		// printf("\nhippitch   /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf",_th_torque[3]*180/pi, _th_torque[8]*180/pi,_th_encoder[3]*180/pi,_th_encoder[8]*180/pi,_error_th_1.x[3][0],_error_th_1.x[8][0]);
		// printf("\nhiproll    /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf",_th_torque[4]*180/pi, _th_torque[7]*180/pi,_th_encoder[4]*180/pi,_th_encoder[7]*180/pi,_error_th_1.x[4][0],_error_th_1.x[7][0]);
		// printf("\nhipyaw     /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf",_th_torque[5]*180/pi, _th_torque[6]*180/pi,_th_encoder[5]*180/pi,_th_encoder[6]*180/pi,_error_th_1.x[5][0],_error_th_1.x[6][0]);
		for(int a=0;a<12;a++)
		{
			_th_printf[a]=_th_position[a];
		}
		printf("\n step count = %d",datacountft);
		datacountft++;	
		// printf("\n time  = %lf", walkingdatatime*samplingtime);
		walkingdatatime++;
		// fprintf(rne,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		// 	_th_printf[0], _th_printf[1], _th_printf[2], _th_printf[3], _th_printf[4],
		// 	_th_printf[5],_th_printf[6], _th_printf[7], _th_printf[8], _th_printf[9],
		// 	_th_printf[10],_th_printf[11],_th_encoder[0],_th_encoder[1],_th_encoder[2],
		// 	_th_encoder[3],_th_encoder[4],_th_encoder[5],_th_encoder[6],_th_encoder[7],
		// 	_th_encoder[8],_th_encoder[9],_th_encoder[10],_th_encoder[11],_error_th_1.x[0][0],
		// 	_error_th_1.x[1][0],_error_th_1.x[2][0],_error_th_1.x[3][0],_error_th_1.x[4][0],_error_th_1.x[5][0],
		// 	_error_th_1.x[6][0],_error_th_1.x[7][0],_error_th_1.x[8][0],_error_th_1.x[9][0],_error_th_1.x[10][0],
		// 	_error_th_1.x[11][0],_error_dth_1.x[0][0],_error_dth_1.x[1][0],_error_dth_1.x[2][0],_error_dth_1.x[3][0],
		// 	_error_dth_1.x[4][0],_error_dth_1.x[5][0],_error_dth_1.x[6][0],_error_dth_1.x[7][0],_error_dth_1.x[8][0],
		// 	_error_dth_1.x[9][0],_error_dth_1.x[10][0],_error_dth_1.x[11][0]);
		
		// fprintf(slipcontrol, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf\n",
		// 	frefY_print, fslipy_print, forceYr, forceYl, local_gpsrz, pre_ycom+pre_ycom_c, ftzmpy, forceZr, forceZl, phaseflag,
		// 	realphaseflag, ssptime, dsptime, stepcheck, sspgpsrz, sspgpsrx ,frefX_print, fslipx_print, footdegreeR,sspdegreeR,
		// 	localFootDegree, yComVFinalPrint, none, none, _ycom_acc_measure, comDegree, footdegreeL);
		// fprintf(footprint, "%lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %d\n",
		// 	gpsrx, gpslx, gpsrz, gpslz, footradR, footradL, sspflag,local_gpsrx, local_gpsrz,footRY_acc,
		// 	footLY_acc, footRYAccLpf, footLYAccLpf,new_phase_num);
		// fprintf(cpt,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf\n",
		// ZMP_c[0]+_px, ZMP_c[1]+_py, phaseflag, realphaseflag,_px + _xcom_CPT,
		//  _py+_ycom_CPT, _xcom_nominal_global, _ycom_nominal_global, ftzmpx + _px, ftzmpy + _py,
		//  _px,_py,ZMP_global_d[0],ZMP_global_d[1],ZMP_c[0],
		//   ZMP_c[1],CP_cur[0],CP_cur[1],CP_cur_global_ssp[0],CP_cur_global_ssp[1],
		//   _targetxcom, _targetycom,CP_ref[0],CP_ref[1],CP_E[0],
		//   CP_E[1],CP_ref_global_ssp[0],CP_ref_global_ssp[1],phaseflag,ZMP_global_d[0],
		//   ZMP_global_d[1],CP_cur_global_dsp[0],CP_cur_global_dsp[1],CP_ref_global_dsp[0],CP_ref_global_dsp[1],
		//   inv_pos.x[0][0],inv_pos.x[1][0]);
		myStep();
	}
	ps_RforceZ_avg =ps_RforceZ_avg /ps_sensing_count;
	ps_RtorqueX_avg=ps_RtorqueX_avg/ps_sensing_count;
	ps_RtorqueY_avg=ps_RtorqueY_avg/ps_sensing_count;
	ps_LforceZ_avg =ps_LforceZ_avg /ps_sensing_count;
	ps_LtorqueX_avg=ps_LtorqueX_avg/ps_sensing_count;
	ps_LtorqueY_avg=ps_LtorqueY_avg/ps_sensing_count;
	_sn=0;
	
	// printf("\n------------------------POSITION MODE STAND OFF-------------------------");
	// printf("\n------------------------------------------------------------------------");
	// printf("\n---------------------------TORQUE MODE STAND ON-------------------------");
	// printf("\n------------------------------------------------------------------------");
	for(int h=0;h<tq_stand;h++) {	
		// printf("\n%c#############%c @@@@@@@@@@@@@@@@", gpscoordinateR, gpscoordinateL);
		gpsr = mGPSR->getValues();
		gpsl = mGPSL->getValues();
		// gpscoordinateR= = mGPSR -> getCoordinateSystem();
		// gpscoordinateL= = mGPSL -> getCoordinateSystem();
		compassr = mCompassR->getValues();
		compassl = mCompassL->getValues();
		compassc=mCompassC->getValues();

		gpsrx = gpsr[0];
		gpsry = gpsr[1];
		gpsrz = gpsr[2];
		gpslx = gpsl[0];
		gpsly = gpsl[1];
		gpslz = gpsl[2];

		compassrx = compassr[0];
		compassry = compassr[1];
		compassrz = compassr[2];
		compasslx = compassl[0];
		compassly = compassl[1];
		compasslz = compassl[2];
		compasscx = compassc[0];
		compasscy = compassc[1];
		compasscz = compassc[2];

		gpsrxoffset = gpsrx;
		gpsryoffset = gpsry;
		gpsrzoffset = gpsrz;
		gpslxoffset = gpslx;
		gpslyoffset = gpsly;
		gpslzoffset = gpslz;

		footradR = atan2(compassrx, compassry);
		footradL = atan2(compasslx, compassly);
		comRad=atan2(compasscx,compasscy);
		footdegreeR = (footradR - 1.5708) / M_PI * 180.0;if (footdegreeR<-180){footdegreeR+=360; }   
		footdegreeL = (footradL - 1.5708) / M_PI * 180.0;if (footdegreeL<-180){footdegreeL+=360; }   
		comDegree=(comRad-1.5708)/M_PI*180.0;if(comDegree<-180){comDegree+=360;}
                        
		footdegreeRoffset = footdegreeR;
		footdegreeLoffset = footdegreeL;
		comdegreeOffset=comDegree;

		// printf("\n gpsrx : %lf ", gpsrx);
		// printf("\t gpsry : %lf ", gpsry);
		// printf("\t gpsrz : %lf ", gpsrz);
		// printf("\t\t gpslx : %lf ", gpslx);
		// printf("\t gpsly : %lf ", gpsly);
		// printf("\t gpslz : %lf ", gpslz);
		// printf("\n compassrx : %lf ", compassrx);
		// printf("\t compassry : %lf ", compassry);
		// printf("\t compassrz : %lf ", compassrz);
		// printf("\t\t compasslx : %lf ", compasslx);
		// printf("\t compassly : %lf ", compassly);
		// printf("\t compasslz : %lf ", compasslz);
        // printf("\t footradR : %lf ", footradR);
        // printf("\t footdegreeR : %lf ", footdegreeR);

		// printf("\n================footradR : %lf\t footradL : %lf=====================", footradR, footradL);
		// printf("\t gpscoordinateR : %c ", gpscoordinateR);
		// printf("\t gpscoordinateL : %c ", gpscoordinateL);

		acc = mAccelerometer->getValues();
		_xcom_acc_measure = acc[0];
		_ycom_acc_measure = acc[1];
		_zcom_acc_measure = acc[2];
		footRX_acc = accR[0];
		footRY_acc = accR[1];
		footRZ_acc = accR[2];
		footLX_acc = accL[0];
		footLY_acc = accL[1];
		footLZ_acc = accL[2];

		ftMotors[0]->setPosition(0);
		ftMotors[1]->setPosition(0);
		ftMotors[2]->setPosition(0);
		ftMotors[3]->setPosition(0);
		ftMotors[4]->setPosition(0);
		ftMotors[5]->setPosition(0);

		torquexr = ftMotors[0]->getTorqueFeedback();
		torqueyr = ftMotors[1]->getTorqueFeedback();
		torquezr = ftMotors[2]->getTorqueFeedback();
		torquexl = ftMotors[3]->getTorqueFeedback();
		torqueyl = ftMotors[4]->getTorqueFeedback();
		torquezl = ftMotors[5]->getTorqueFeedback();
		// printf("\n ptorquerollR : %lf ",torquexr);
		// printf("\t ptorquepitchR : %lf ",torqueyr);
		// printf("\t ptorqueyawR : %lf ",torquezr);
		// printf("\t\t ptorquerollL : %lf ",torquexl);
		// printf("\t ptorquepitchL : %lf ",torqueyl);
		// printf("\t ptorqueyawL : %lf ",torquezl);

		F_Lu = mFlu->getValues();
		F_Ld = mFld->getValues();
		F_Ru = mFru->getValues();
		F_Rd = mFrd->getValues();

		Frxu = F_Ru[0];
		Fryu = F_Ru[1];
		Frzu = F_Ru[2];
		Frxd = F_Rd[2];
		Fryd = F_Rd[1];
		Frzd = F_Rd[0];

		Flxu = F_Lu[0];
		Flyu = F_Lu[1];
		Flzu = F_Lu[2];
		Flxd = F_Ld[2];
		Flyd = F_Ld[1];
		Flzd = F_Ld[0];

		Flxu >= Flxd ? FXl = Flxu : FXl = -Flxd;
		Flyu >= Flyd ? FYl = Flyu : FYl = -Flyd;
		Flzu >= Flzd ? FZl = Flzu : FZl = -Flzd;
		
		Frxu >= Frxd ? FXr = Frxu : FXr = -Frxd;
		Fryu >= Fryd ? FYr = Fryu : FYr = -Fryd;
		Frzu >= Frzd ? FZr = Frzu : FZr = -Frzd;
		
		// printf("\n Flxu : %lf ",Flxu);
		// printf("\t Flyu : %lf ",Flyu);
		// printf("\t Flzu : %lf ",Flzu);
		// printf("\t Flxd : %lf ",Flxd);
		// printf("\t Flyd : %lf ",Flyd);
		// printf("\t Flzd : %lf ",Flzd); 
		// printf("\t Frxu : %lf ",Flxu);
		// printf("\t Fryu : %lf ",Flyu);
		// printf("\t Frzu : %lf ",Frzu);
		// printf("\t Frxd : %lf ",Flxd);
		// printf("\t Fryd : %lf ",Flyd);
		// printf("\t Frzd : %lf \n",Frzd);
		ftzmpxr = (-torqueyr) / Frzu * 10;
		ftzmpxl = (-torqueyl) / Flzu * 10;
		ftzmpyr = (torquexr) / Frzu * 10;
		ftzmpyl = (torquexl) / Flzu * 10;

		forceZr = (FZr / 10);
		forceZl = (FZl / 10);
		forceXr = (FXr / 10);
		forceXl = (FXl / 10);
		forceYr = (FYr / 10);
		forceYl = (FYl / 10);
		
		// printf("\n PForceXr : %lf ",FXr/10);
		// printf("\t PForceYr : %lf ",FYr/10);
		// printf("\t PForceZr : %lf ",FZr/10);
		// printf("\t\t\t\t PForceXl : %lf ",FXl/10);
		// printf("\t PForceYl : %lf ",FYl/10);
		// printf("\t PForceZl : %lf ",FZl/10);
		
		phaseflag = 0;
		realphaseflag = 0;

		if (Frzu == 0) {
			ftzmpxr = 0;
			ftzmpyr = 0;
		}
		if (Flzu == 0) {
			ftzmpxl = 0;
			ftzmpyl = 0;
		}
		footup = 0;
		
		if (new_phase_num == 2)//DSP_R
		{
			local_gpsrx_ssp_begin=gpsrx;
			local_gpsrz_ssp_begin=gpsrz;

			local_gpsrx=gpslx-local_gpslx_ssp_begin;
			local_gpsrz=gpslz-local_gpslz_ssp_begin;

			localFootDegreeRSspI=footdegreeR;
			localFootDegree=footdegreeL-localFootDegreeLSspI;
			// printf("localFootDegree=%lf footdegreeL=%lf localFootDegreeLSspI=%lf\n",localFootDegree, footdegreeL,localFootDegreeLSspI);
			ftzmpx = ((Flzu/10)*(ftzmpxl - pre_local_stride_F) + (Frzu/10) * ftzmpxr) / ((Frzu/10) + (Flzu/10));
			ftzmpy = ((Flzu/10)*(ftzmpyl + pre_local_stride_L) + (Frzu/10) * ftzmpyr) / ((Frzu/10) + (Flzu/10));
			// printf("222222222222222222222222222222");
			
		}

		else if (new_phase_num == 4)//DSP_L
		{
            local_gpslx_ssp_begin=gpslx;
			local_gpslz_ssp_begin=gpslz;
			local_gpsrx=gpsrx-local_gpsrx_ssp_begin;
			local_gpsrz=gpsrz-local_gpsrz_ssp_begin;

			localFootDegreeLSspI=footdegreeL;
			localFootDegree=footdegreeR-localFootDegreeRSspI;
			// printf("localFootDegree=%lf footdegreeR=%lf localFootDegreeRSspI=%lf\n",localFootDegree, footdegreeR,localFootDegreeRSspI);
			ftzmpx = ((Flzu/10)*(ftzmpxl)+(Frzu/10) * (ftzmpxr - pre_local_stride_F)) / ((Frzu/10) + (Flzu/10));
			// printf("ftzmpxl=%lf ftzmpxr=%lf",ftzmpxl,ftzmpxr);
			ftzmpy = ((Flzu/10)*(ftzmpyl)+(Frzu/10) * (ftzmpyr - pre_local_stride_L)) / ((Frzu/10) + (Flzu/10));

			// printf("4444@@%lf %lf\n",(Flzd/10)*ftzmpxl, (Frzd/10)*(ftzmpxr-pre_local_stride_F));
			// printf("4444444444444444444444444444444444444444");
			
		}
		
		else if (new_phase_num == 1)//SSP_R
		{
			double FT_denominator_lx = (Flzu/10) * (ftzmpxl + (_lsx_des - _px));
			double FT_denominator_ly = (Flzu/10) * (ftzmpyl + (_lsy_des - _py));

			local_gpsrx=gpsrx-local_gpsrx_ssp_begin;
			local_gpsrz=gpsrz-local_gpsrz_ssp_begin;
 
			localFootDegree=footdegreeR-localFootDegreeRSspI;
			
			ftzmpx = (FT_denominator_lx + (Frzu/10) * ftzmpxr) / ((Frzu/10) + (Flzu/10));
			ftzmpy = (FT_denominator_ly + (Frzu/10) * ftzmpyr) / ((Frzu/10) + (Flzu/10));
			// printf("localFootDegree=%lf footdegreeR=%lf localFootDegreeRSspI=%lf\n",localFootDegree, footdegreeR,localFootDegreeRSspI);
			// printf("111111111111111111111111111111");
			
		}
		else if (new_phase_num == 3)//SSP_L
		{
			
			double FT_denominator_rx = (Frzu/10) * (ftzmpxr + (_rsx_des - _px));
			double FT_denominator_ry = (Frzu/10) * (ftzmpyl + (_rsy_des - _py));
			//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!rsx des=%lf   rsy des = %lf   _py=%lf\n", _rsx_des, _rsy_des,_py);
			
			local_gpsrx=gpslx-local_gpslx_ssp_begin;
			local_gpsrz=gpslz-local_gpslz_ssp_begin;
			
			localFootDegree=footdegreeL-localFootDegreeLSspI;
			
			ftzmpx = ((Flzu/10)*(ftzmpxl)+FT_denominator_rx) / ((Frzu/10) + (Flzu/10));
			ftzmpy = ((Flzu/10)*(ftzmpyl)+FT_denominator_ry) / ((Frzu/10) + (Flzu/10));
			
			// printf("localFootDegree=%lf footdegreeL=%lf localFootDegreeLSspI=%lf\n",localFootDegree, footdegreeL,localFootDegreeLSspI);
			// printf("3333333333333333333333333333333");
		}
		
		phase = DSP_L;
		if (phase == DSP_L)
		{
			new_phase_num = 4;
			phaseflag = 1;
		}
		// FsrSensing(_i-1, new_phase_num, _px, _py, _ppx, _ppy);
		
		pre_force_R[0]=forceXr;
		pre_force_R[1]=forceYr;
		pre_force_R[2]=forceZr;
	
		pre_force_L[0]=forceXl;
		pre_force_L[1]=forceYl;
		pre_force_L[2]=forceZl;
	
	
		pre_torque_R[0]=torquexr;
		pre_torque_R[1]=torqueyr;
		pre_torque_R[2]=torquezr;
	
		pre_torque_L[0]=torquexl;
		pre_torque_L[1]=torqueyl;
		pre_torque_L[2]=torquezl;

		pre_force_F_R[0]=force_F_R[0];
		pre_force_F_R[1]=force_F_R[1];
		pre_force_F_R[2]=force_F_R[2];
	
		pre_force_F_L[0]=force_F_L[0];
		pre_force_F_L[1]=force_F_L[1];
		pre_force_F_L[2]=force_F_L[2];
	
	
		pre_torque_F_R[0]= torque_F_R[0];
		pre_torque_F_R[1]= torque_F_R[1];
		pre_torque_F_R[2]= torque_F_R[2];
	
		pre_torque_F_L[0]= torque_F_L[0];
		pre_torque_F_L[1]= torque_F_L[1];
		pre_torque_F_L[2]= torque_F_L[2];
	
		LPF(10,pre_force_R,pre_force_F_R,force_F_R);
		LPF(10,pre_force_L,pre_force_F_L,force_F_L);

		LPF(10,pre_torque_R,pre_torque_F_R,torque_F_R);
		LPF(10,pre_torque_L,pre_torque_F_L,torque_F_L);

		//Matrix_12X1 _theta_p;

	

		_th_encoder_p[0]  = _th_encoder[0];
		_th_encoder_p[1]  = _th_encoder[1];
		_th_encoder_p[2]  = _th_encoder[2];
		_th_encoder_p[3]  = _th_encoder[3];
		_th_encoder_p[4]  = _th_encoder[4];
		_th_encoder_p[5]  = _th_encoder[5];
		_th_encoder_p[6]  = _th_encoder[6];
		_th_encoder_p[7]  = _th_encoder[7];
		_th_encoder_p[8]  = _th_encoder[8];
		_th_encoder_p[9]  = _th_encoder[9];
		_th_encoder_p[10] = _th_encoder[10];
		_th_encoder_p[11] = _th_encoder[11];

		_th_encoder[0] = -mPositionSensors[ankleRollR]->getValue();
		_th_encoder[1] = -mPositionSensors[anklePitchR]->getValue();
		_th_encoder[2] = mPositionSensors[kneePitchR]->getValue();
		_th_encoder[3] = mPositionSensors[hipPitchR]->getValue();
		_th_encoder[4] = mPositionSensors[hipRollR]->getValue();
		_th_encoder[5] = mPositionSensors[hipYawR]->getValue();
		_th_encoder[6] = -mPositionSensors[hipYawL]->getValue();
		_th_encoder[7] = -mPositionSensors[hipRollL]->getValue();
		_th_encoder[8] = mPositionSensors[hipPitchL]->getValue();
		_th_encoder[9] = mPositionSensors[kneePitchL]->getValue();
		_th_encoder[10] = -mPositionSensors[anklePitchL]->getValue();
		_th_encoder[11] = mPositionSensors[ankleRollL]->getValue();

#if GravityCompensation	
		
		if(_sn==0)
		{
			_th_encoder[0] =0;
			_th_encoder[1] = 0;
			_th_encoder[2] =0;
			_th_encoder[3] = 0;
			_th_encoder[4] = 0;
			_th_encoder[5] = 0;
			_th_encoder[6] = 0;
			_th_encoder[7] = 0;
			_th_encoder[8] = 0;
			_th_encoder[9] = 1.57;
			_th_encoder[10] = 0;
			_th_encoder[11] = 0;
			forwardkinematics(_th_encoder);
		}
		else
		{
			_th_encoder[0] =0;
			_th_encoder[1] = 0;
			_th_encoder[2] =0;
			_th_encoder[3] = 0;
			_th_encoder[4] = 0;
			_th_encoder[5] = 0;
			_th_encoder[6] = 0;
			_th_encoder[7] = 0;
			_th_encoder[8] = 0;
			_th_encoder[9] = 1.57-_sn*0.001;
			_th_encoder[10] = 0;
			_th_encoder[11] = 0;
			forwardkinematics(_th_encoder);
		}

		_theta_p.x[0][0] = _th_encoder[0];
		_theta_p.x[1][0] = _th_encoder[1];
		_theta_p.x[2][0] = _th_encoder[2];
		_theta_p.x[3][0] = _th_encoder[3];
		_theta_p.x[4][0] = _th_encoder[4];
		_theta_p.x[5][0] = _th_encoder[5];
		_theta_p.x[6][0] = _th_encoder[6];
		_theta_p.x[7][0] = _th_encoder[7];
		_theta_p.x[8][0] = _th_encoder[8];
		_theta_p.x[9][0] = _th_encoder[9];
		_theta_p.x[10][0] = _th_encoder[10];
		_theta_p.x[11][0] = _th_encoder[11];
		// else
		// {
		// 	th_error=0;
		// 	th_errorsum=0;
			
		// 	for(int errorcheck =0;errorcheck<12;errorcheck++)
		// 	{
		// 		th_error=abs(_th_encoder[errorcheck]-_th_encoder_p[errorcheck]);
		// 		th_errorsum=th_errorsum+th_error;
		// 	}
		// 	if(th_errorsum>0.004)
		// 	{
		// 		if(th_errorsum_p>th_errorsum)
		// 		{
		// 			printf("\n!!!!!!!!!!!!!!!!!!!encoder update");
		// 			forwardkinematics(_th_encoder_p);
		// 		}
		// 	}
		// 	th_errorsum_p=th_errorsum;
		// 	printf("\nerrorsum=%lf\n",th_errorsum);
		// }
		
#elif (GravityCompensation==0)
		forwardkinematics(_th_torque);		

		_theta_p.x[0][0] = _th_torque[0];
		_theta_p.x[1][0] = _th_torque[1];
		_theta_p.x[2][0] = _th_torque[2];
		_theta_p.x[3][0] = _th_torque[3];
		_theta_p.x[4][0] = _th_torque[4];
		_theta_p.x[5][0] = _th_torque[5];
		_theta_p.x[6][0] = _th_torque[6];
		_theta_p.x[7][0] = _th_torque[7];
		_theta_p.x[8][0] = _th_torque[8];
		_theta_p.x[9][0] = _th_torque[9];
		_theta_p.x[10][0] = _th_torque[10];
		_theta_p.x[11][0] = _th_torque[11];
#endif		
		
		RR01.x[0][0] = TRB1[0][0];
		RR01.x[0][1] = TRB1[0][1];
		RR01.x[0][2] = TRB1[0][2];

		RR01.x[1][0] = TRB1[1][0];
		RR01.x[1][1] = TRB1[1][1];
		RR01.x[1][2] = TRB1[1][2];

		RR01.x[2][0] = TRB1[2][0];
		RR01.x[2][1] = TRB1[2][1];
		RR01.x[2][2] = TRB1[2][2];

		RR12.x[0][0] = TR12[0][0];
		RR12.x[0][1] = TR12[0][1];
		RR12.x[0][2] = TR12[0][2];

		RR12.x[1][0] = TR12[1][0];
		RR12.x[1][1] = TR12[1][1];
		RR12.x[1][2] = TR12[1][2];

		RR12.x[2][0] = TR12[2][0];
		RR12.x[2][1] = TR12[2][1];
		RR12.x[2][2] = TR12[2][2];

		RR23.x[0][0] = TR23[0][0];
		RR23.x[0][1] = TR23[0][1];
		RR23.x[0][2] = TR23[0][2];

		RR23.x[1][0] = TR23[1][0];
		RR23.x[1][1] = TR23[1][1];
		RR23.x[1][2] = TR23[1][2];

		RR23.x[2][0] = TR23[2][0];
		RR23.x[2][1] = TR23[2][1];
		RR23.x[2][2] = TR23[2][2];

		RR34.x[0][0] = TR34[0][0];
		RR34.x[0][1] = TR34[0][1];
		RR34.x[0][2] = TR34[0][2];

		RR34.x[1][0] = TR34[1][0];
		RR34.x[1][1] = TR34[1][1];
		RR34.x[1][2] = TR34[1][2];

		RR34.x[2][0] = TR34[2][0];
		RR34.x[2][1] = TR34[2][1];
		RR34.x[2][2] = TR34[2][2];

		RR45.x[0][0] = TR45[0][0];
		RR45.x[0][1] = TR45[0][1];
		RR45.x[0][2] = TR45[0][2];

		RR45.x[1][0] = TR45[1][0];
		RR45.x[1][1] = TR45[1][1];
		RR45.x[1][2] = TR45[1][2];

		RR45.x[2][0] = TR45[2][0];
		RR45.x[2][1] = TR45[2][1];
		RR45.x[2][2] = TR45[2][2];

		RR56.x[0][0] = TR56[0][0];
		RR56.x[0][1] = TR56[0][1];
		RR56.x[0][2] = TR56[0][2];

		RR56.x[1][0] = TR56[1][0];
		RR56.x[1][1] = TR56[1][1];
		RR56.x[1][2] = TR56[1][2];

		RR56.x[2][0] = TR56[2][0];
		RR56.x[2][1] = TR56[2][1];
		RR56.x[2][2] = TR56[2][2];

		RR67.x[0][0] = TR6E[0][0];
		RR67.x[0][1] = TR6E[0][1];
		RR67.x[0][2] = TR6E[0][2];

		RR67.x[1][0] = TR6E[1][0];
		RR67.x[1][1] = TR6E[1][1];
		RR67.x[1][2] = TR6E[1][2];

		RR67.x[2][0] = TR6E[2][0];
		RR67.x[2][1] = TR6E[2][1];
		RR67.x[2][2] = TR6E[2][2];

		RL01.x[0][0] = TLB1[0][0];
		RL01.x[0][1] = TLB1[0][1];
		RL01.x[0][2] = TLB1[0][2];

		RL01.x[1][0] = TLB1[1][0];
		RL01.x[1][1] = TLB1[1][1];
		RL01.x[1][2] = TLB1[1][2];

		RL01.x[2][0] = TLB1[2][0];
		RL01.x[2][1] = TLB1[2][1];
		RL01.x[2][2] = TLB1[2][2];

		RL12.x[0][0] = TL12[0][0];
		RL12.x[0][1] = TL12[0][1];
		RL12.x[0][2] = TL12[0][2];

		RL12.x[1][0] = TL12[1][0];
		RL12.x[1][1] = TL12[1][1];
		RL12.x[1][2] = TL12[1][2];

		RL12.x[2][0] = TL12[2][0];
		RL12.x[2][1] = TL12[2][1];
		RL12.x[2][2] = TL12[2][2];

		RL23.x[0][0] = TL23[0][0];
		RL23.x[0][1] = TL23[0][1];
		RL23.x[0][2] = TL23[0][2];

		RL23.x[1][0] = TL23[1][0];
		RL23.x[1][1] = TL23[1][1];
		RL23.x[1][2] = TL23[1][2];

		RL23.x[2][0] = TL23[2][0];
		RL23.x[2][1] = TL23[2][1];
		RL23.x[2][2] = TL23[2][2];

		RL34.x[0][0] = TL34[0][0];
		RL34.x[0][1] = TL34[0][1];
		RL34.x[0][2] = TL34[0][2];

		RL34.x[1][0] = TL34[1][0];
		RL34.x[1][1] = TL34[1][1];
		RL34.x[1][2] = TL34[1][2];

		RL34.x[2][0] = TL34[2][0];
		RL34.x[2][1] = TL34[2][1];
		RL34.x[2][2] = TL34[2][2];

		RL45.x[0][0] = TL45[0][0];
		RL45.x[0][1] = TL45[0][1];
		RL45.x[0][2] = TL45[0][2];

		RL45.x[1][0] = TL45[1][0];
		RL45.x[1][1] = TL45[1][1];
		RL45.x[1][2] = TL45[1][2];

		RL45.x[2][0] = TL45[2][0];
		RL45.x[2][1] = TL45[2][1];
		RL45.x[2][2] = TL45[2][2];

		RL56.x[0][0] = TL56[0][0];
		RL56.x[0][1] = TL56[0][1];
		RL56.x[0][2] = TL56[0][2];

		RL56.x[1][0] = TL56[1][0];
		RL56.x[1][1] = TL56[1][1];
		RL56.x[1][2] = TL56[1][2];

		RL56.x[2][0] = TL56[2][0];
		RL56.x[2][1] = TL56[2][1];
		RL56.x[2][2] = TL56[2][2];

		RL67.x[0][0] = TL6E[0][0];
		RL67.x[0][1] = TL6E[0][1];
		RL67.x[0][2] = TL6E[0][2];

		RL67.x[1][0] = TL6E[1][0];
		RL67.x[1][1] = TL6E[1][1];
		RL67.x[1][2] = TL6E[1][2];

		RL67.x[2][0] = TL6E[2][0];
		RL67.x[2][1] = TL6E[2][1];
		RL67.x[2][2] = TL6E[2][2];

		//Matrix_M33X7 RR, RL;

		RR.x[0][0] = RR01;
		RR.x[1][0] = RR12;
		RR.x[2][0] = RR23;
		RR.x[3][0] = RR34;
		RR.x[4][0] = RR45;
		RR.x[5][0] = RR56;
		RR.x[6][0] = RR67;

		RL.x[0][0] = RL01;
		RL.x[1][0] = RL12;
		RL.x[2][0] = RL23;
		RL.x[3][0] = RL34;
		RL.x[4][0] = RL45;
		RL.x[5][0] = RL56;
		RL.x[6][0] = RL67;

		//Matrix_3X3 RR10, RR21, RR32, RR43, RR54, RR65, RR76;
		//Matrix_3X3 RL10, RL21, RL32, RL43, RL54, RL65, RL76;

		RR10 = transposeMatrix_33(RR01);
		RR21 = transposeMatrix_33(RR12);
		RR32 = transposeMatrix_33(RR23);
		RR43 = transposeMatrix_33(RR34);
		RR54 = transposeMatrix_33(RR45);
		RR65 = transposeMatrix_33(RR56);
		RR76 = transposeMatrix_33(RR67);

		RL10 = transposeMatrix_33(RL01);
		RL21 = transposeMatrix_33(RL12);
		RL32 = transposeMatrix_33(RL23);
		RL43 = transposeMatrix_33(RL34);
		RL54 = transposeMatrix_33(RL45);
		RL65 = transposeMatrix_33(RL56);
		RL76 = transposeMatrix_33(RL67);
		//Matrix_M33X7 invRR, invRL;

		invRR.x[0][0] = RR10;
		invRR.x[1][0] = RR21;
		invRR.x[2][0] = RR32;
		invRR.x[3][0] = RR43;
		invRR.x[4][0] = RR54;
		invRR.x[5][0] = RR65;
		invRR.x[6][0] = RR76;

		invRL.x[0][0] = RL10;
		invRL.x[1][0] = RL21;
		invRL.x[2][0] = RL32;
		invRL.x[3][0] = RL43;
		invRL.x[4][0] = RL54;
		invRL.x[5][0] = RL65;
		invRL.x[6][0] = RL76;

		PR01.x[0][0] = TRB1[0][3];
		PR01.x[1][0] = TRB1[1][3];
		PR01.x[2][0] = TRB1[2][3];
	
		PR12.x[0][0] = TR12[0][3];
		PR12.x[1][0] = TR12[1][3];
		PR12.x[2][0] = TR12[2][3];

		PR23.x[0][0] = TR23[0][3];
		PR23.x[1][0] = TR23[1][3];
		PR23.x[2][0] = TR23[2][3];

		PR34.x[0][0] = TR34[0][3];
		PR34.x[1][0] = TR34[1][3];
		PR34.x[2][0] = TR34[2][3];

		PR45.x[0][0] = TR45[0][3];
		PR45.x[1][0] = TR45[1][3];
		PR45.x[2][0] = TR45[2][3];
	
		PR56.x[0][0] = TR56[0][3];
		PR56.x[1][0] = TR56[1][3];
		PR56.x[2][0] = TR56[2][3];

		PR67.x[0][0] = TR6E[0][3];
		PR67.x[1][0] = TR6E[1][3];
		PR67.x[2][0] = TR6E[2][3];
	
		PL01.x[0][0] = TLB1[0][3];
		PL01.x[1][0] = TLB1[1][3];
		PL01.x[2][0] = TLB1[2][3];
	
		PL12.x[0][0] = TL12[0][3];
		PL12.x[1][0] = TL12[1][3];
		PL12.x[2][0] = TL12[2][3];

		PL23.x[0][0] = TL23[0][3];
		PL23.x[1][0] = TL23[1][3];
		PL23.x[2][0] = TL23[2][3];

		PL34.x[0][0] = TL34[0][3];
		PL34.x[1][0] = TL34[1][3];
		PL34.x[2][0] = TL34[2][3];

		PL45.x[0][0] = TL45[0][3];
		PL45.x[1][0] = TL45[1][3];
		PL45.x[2][0] = TL45[2][3];
	
		PL56.x[0][0] = TL56[0][3];
		PL56.x[1][0] = TL56[1][3];
		PL56.x[2][0] = TL56[2][3];

		PL67.x[0][0] = TL6E[0][3];
		PL67.x[1][0] = TL6E[1][3];
		PL67.x[2][0] = TL6E[2][3];
	
		//Matrix_M31X7 PositionppR;

		PositionppR.x[0][0] = PR01;
		PositionppR.x[0][1] = PR12;
		PositionppR.x[0][2] = PR23;
		PositionppR.x[0][3] = PR34;
		PositionppR.x[0][4] = PR45;
		PositionppR.x[0][5] = PR56;
		PositionppR.x[0][6] = PR67;
	
		//Matrix_M31X7 PositionppL;

		PositionppL.x[0][0] = PL01;
		PositionppL.x[0][1] = PL12;
		PositionppL.x[0][2] = PL23;
		PositionppL.x[0][3] = PL34;
		PositionppL.x[0][4] = PL45;
		PositionppL.x[0][5] = PL56;
		PositionppL.x[0][6] = PL67;

		ddq_Jdot_R_p.x[0][0] = 0.0;
		ddq_Jdot_R_p.x[1][0] = 0.0;
		ddq_Jdot_R_p.x[2][0] = 0.0;
		ddq_Jdot_R_p.x[3][0] = 0.0;
		ddq_Jdot_R_p.x[4][0] = 0.0;
		ddq_Jdot_R_p.x[5][0] = 0.0;
  
		ddq_Jdot_R_p.x[0][1] = 0.0;
		ddq_Jdot_R_p.x[1][1] = 0.0;
		ddq_Jdot_R_p.x[2][1] = 0.0;
		ddq_Jdot_R_p.x[3][1] = 0.0;
		ddq_Jdot_R_p.x[4][1] = 0.0;
		ddq_Jdot_R_p.x[5][1] = 0.0;

		ddq_Jdot_R_p.x[0][2] = 0.0;
		ddq_Jdot_R_p.x[1][2] = 0.0;
		ddq_Jdot_R_p.x[2][2] = 0.0;
		ddq_Jdot_R_p.x[3][2] = 0.0;
		ddq_Jdot_R_p.x[4][2] = 0.0;
		ddq_Jdot_R_p.x[5][2] = 0.0;

		ddq_Jdot_R_p.x[0][3] = 0.0;
		ddq_Jdot_R_p.x[1][3] = 0.0;
		ddq_Jdot_R_p.x[2][3] = 0.0;
		ddq_Jdot_R_p.x[3][3] = 0.0;
		ddq_Jdot_R_p.x[4][3] = 0.0;
		ddq_Jdot_R_p.x[5][3] = 0.0;

		ddq_Jdot_R_p.x[0][4] = 0.0;
		ddq_Jdot_R_p.x[1][4] = 0.0;
		ddq_Jdot_R_p.x[2][4] = 0.0;
		ddq_Jdot_R_p.x[3][4] = 0.0;
		ddq_Jdot_R_p.x[4][4] = 0.0;
		ddq_Jdot_R_p.x[5][4] = 0.0;

		ddq_Jdot_R_p.x[0][5] = 0.0;
		ddq_Jdot_R_p.x[1][5] = 0.0;
		ddq_Jdot_R_p.x[2][5] = 0.0;
		ddq_Jdot_R_p.x[3][5] = 0.0;
		ddq_Jdot_R_p.x[4][5] = 0.0;
		ddq_Jdot_R_p.x[5][5] = 0.0;

		_w_LE_n.x[0][0]=0;
		_w_LE_n.x[1][0]=0;
		_w_LE_n.x[2][0]=0;

		_w_RE_n.x[0][0]=0;
		_w_RE_n.x[1][0]=0;
		_w_RE_n.x[2][0]=0;

		_dw_LE_n.x[0][0]=0;
		_dw_LE_n.x[1][0]=0;
		_dw_LE_n.x[2][0]=0;

		_dw_RE_n.x[0][0]=0;
		_dw_RE_n.x[1][0]=0;
		_dw_RE_n.x[2][0]=0;

		Mdth_d.x[0][0]=0.0;
		Mdth_d.x[1][0]=0.0;
		Mdth_d.x[2][0]=0.0;
		Mdth_d.x[3][0]=0.0;
		Mdth_d.x[4][0]=0.0;
		Mdth_d.x[5][0]=0.0;
		Mdth_d.x[6][0]=0.0;
		Mdth_d.x[7][0]=0.0;
		Mdth_d.x[8][0]=0.0;
		Mdth_d.x[9][0]=0.0;
		Mdth_d.x[10][0]=0.0;
		Mdth_d.x[11][0]=0.0;

		// printf("\nrne encodor %lf %lf %lf %lf %lf %lf $$$$ %lf %lf %lf %lf %lf %lf",_th_encoder[0]*180/pi,_th_encoder[1]*180/pi,_th_encoder[2]*180/pi,_th_encoder[3]*180/pi,_th_encoder[4]*180/pi,_th_encoder[5]*180/pi,_th_encoder[6]*180/pi,_th_encoder[7]*180/pi,_th_encoder[8]*180/pi,_th_encoder[9]*180/pi,_th_encoder[10]*180/pi,_th_encoder[11]*180/pi);
		// printf("\ndes:hipyawR=    %lf   hipyawL=    %lf ,enc:hipyawR=      %lf   hipyawL=    %lf",_th_torque[0]*180/pi, _th_torque[11]*180/pi,_th_encoder[0]*180/pi,_th_encoder[11]*180/pi);
		// printf("\ndes=hiprollR=   %lf   hiprollL=   %lf ,enc:hiprollR=     %lf   hiprollL=   %lf",_th_torque[1]*180/pi, _th_torque[10]*180/pi,_th_encoder[1]*180/pi,_th_encoder[10]*180/pi);
		// printf("\ndes:hippitchR=  %lf   hippitchL=  %lf ,enc:hippitchR=    %lf   hippitchL=  %lf",_th_torque[2]*180/pi, _th_torque[9]*180/pi,_th_encoder[2]*180/pi,_th_encoder[9]*180/pi);
		// printf("\ndes:kneepitchR= %lf   kneepitchL= %lf ,enc:kneepitchR=   %lf   kneepitchL= %lf",_th_torque[3]*180/pi, _th_torque[8]*180/pi,_th_encoder[3]*180/pi,_th_encoder[8]*180/pi);
		// printf("\ndes=anklepitchR=%lf   anklepitchL=%lf ,enc:anklepitchR=  %lf   anklepitchL=%lf",_th_torque[4]*180/pi, _th_torque[7]*180/pi,_th_encoder[4]*180/pi,_th_encoder[7]*180/pi);
		// printf("\ndes=anklerollR= %lf   anklerollL= %lf ,enc:anklerollR=   %lf   anklerollL= %lf",_th_torque[5]*180/pi, _th_torque[6]*180/pi,_th_encoder[5]*180/pi,_th_encoder[6]*180/pi);
		
		// forwardkinematics_FTB(_th_encoder);

		// if (_sn==0)
		// {
		// 	_th_current_p =_theta_p;
		// }

		// else{
		// 	_th_current_p =_th_current_n;
		// }
		_th_current_p =_th_current_n;	

		_th_current_n.x[0][0]=_th_encoder[0];
		_th_current_n.x[1][0]=_th_encoder[1];
		_th_current_n.x[2][0]=_th_encoder[2];
		_th_current_n.x[3][0]=_th_encoder[3];
		_th_current_n.x[4][0]=_th_encoder[4];
		_th_current_n.x[5][0]=_th_encoder[5];
		_th_current_n.x[6][0]=_th_encoder[6];
		_th_current_n.x[7][0]=_th_encoder[7];
		_th_current_n.x[8][0]=_th_encoder[8];
		_th_current_n.x[9][0]=_th_encoder[9];
		_th_current_n.x[10][0]=_th_encoder[10];
		_th_current_n.x[11][0]=_th_encoder[11];

      	
		if (_sn==0)
		{
			Mdth.x[0][0]=0.0;
			Mdth.x[1][0]=0.0;
			Mdth.x[2][0]=0.0;
			Mdth.x[3][0]=0.0;
			Mdth.x[4][0]=0.0;
			Mdth.x[5][0]=0.0;
			Mdth.x[6][0]=0.0;
			Mdth.x[7][0]=0.0;
			Mdth.x[8][0]=0.0;
			Mdth.x[9][0]=0.0;
			Mdth.x[10][0]=0.0;
			Mdth.x[11][0]=0.0;
		}

		else
		{
			Mdth = s_vectorminus121_121(_th_current_n, _th_current_p);
		}

#if (GravityCompensation==1)		
		_kv=0;
		_kp=0;
#endif
		// double Tr_anklePitch=0.015;//0;//0.02;//
		// double _kv_anklePitch=2*(1.8/Tr_anklePitch);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
		// double _kp_anklePitch=(1.8/Tr_anklePitch)*(1.8/Tr_anklePitch);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
		// double Tr_ankleRoll=0.01;//0;//0.02;//
		// double _kv_ankleRoll=2*(1.8/Tr_ankleRoll);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
		// double _kp_ankleRoll=(1.8/Tr_ankleRoll)*(1.8/Tr_ankleRoll);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
		_error_dth_1=vectorminus121_121(Mdth_d,Mdth);
		_error_dth=productmatrix121_d(_kv,_error_dth_1);
		// _error_dth.x[1][0]=_error_dth.x[1][0]/_kv*_kv_anklePitch;
		// _error_dth.x[10][0]=_error_dth.x[10][0]/_kv*_kv_anklePitch;
		// _error_dth.x[0][0]=_error_dth.x[0][0]/_kv*_kv_ankleRoll;
		// _error_dth.x[11][0]=_error_dth.x[11][0]/_kv*_kv_ankleRoll;

		_error_th_1=vectorminus121_121(_theta_p,_th_current_n);
		_error_th=productmatrix121_d(_kp,_error_th_1);
		// _error_th.x[1][0]=_error_th.x[1][0]/_kp*_kp_anklePitch;
		// _error_th.x[10][0]=_error_th.x[10][0]/_kp*_kp_anklePitch;
		// _error_th.x[0][0]=_error_th.x[0][0]/_kp*_kp_ankleRoll;
		// _error_th.x[11][0]=_error_th.x[11][0]/_kp*_kp_ankleRoll;

		Mddth =vectorplus121_121_121(Mddth_d,_error_dth,_error_th);
		
		//RNE 초기 조건
		wr00.x[0][0] = 0;
		wr00.x[1][0] = 0;
		wr00.x[2][0] = 0;

		dwr00.x[0][0] = 0;
		dwr00.x[1][0] = 0;
		dwr00.x[2][0] = 0;

		dvr00.x[0][0] = 0;
		dvr00.x[1][0] = 0;
		dvr00.x[2][0] = 9.81; //0

		dvcr00.x[0][0] = 0;
		dvcr00.x[1][0] = 0;
		dvcr00.x[2][0] = 0;

		//Matrix_3X1 Fr00, Nr00, fr77, nr77;

		Fr00.x[0][0] = 0;
		Fr00.x[1][0] = 0;
		Fr00.x[2][0] = 0;

		Nr00.x[0][0] = 0;
		Nr00.x[1][0] = 0;
		Nr00.x[2][0] = 0;

		//f,n 

		fr77.x[0][0] =  0;
		fr77.x[1][0] =  0;
		fr77.x[2][0] =  0;//-11.672488;//-ps_RforceZ_last;//-force_F_R[2];//-ps_RforceZ_last;//-forceZr;//-force_F_R[2];//-d_Fz_DSP;//
		// printf("\nforce_F_R[2]=%d",force_F_R[2]);


		nr77.x[0][0] =  0;//0.142319;//-ps_RtorqueX_last;//-torque_F_R[0];//-ps_RtorqueX_last;//-torquexr;//-torque_F_R[0];//0;//
		nr77.x[1][0] =  0;//0.053947;//-ps_RtorqueY_last;//-torque_F_R[1];//-ps_RtorqueY_last;//-torqueyr;//-torque_F_R[1];//0;//
		nr77.x[2][0] =	0;
		// printf("\nR : fz=%lf tx=%lf ty=%lf",ps_RforceZ_last,ps_RtorqueX_last,ps_RtorqueY_last);

		//RNE
		//outward iteration
		//Matrix_M31X7 RNEwr, RNEdwr, RNEdvr, RNEdvcr, RNEFr, RNENr, RNEfr, RNEnr;

		RNEwr.x[0][0] = wr00;
		RNEdwr.x[0][0] = dwr00;
		RNEdvr.x[0][0] = dvr00;
		RNEdvcr.x[0][0] = dvcr00;
		RNEFr.x[0][0] = Fr00;
		RNENr.x[0][0] = Nr00;

		RNEfr.x[0][6] = fr77;
		RNEnr.x[0][6] = nr77;

		for (k = 0; k < 6; k++) { ///outward iteration

				rotationr = invRR.x[k][0]; //3x3
				omegar = RNEwr.x[0][k]; //3x1
				productwr = productmatrix33_31(rotationr, omegar);//3x1

				ta_zr=ta.x[0][5-k];//3x1
				wr_m2_1=Mdth.x[5-k][0];//1x1(double)
#if GravityCompensation	
				wr_m2_1=0;
#endif
				wr_m2 = productmatrix11_31(wr_m2_1, ta_zr);//3x1


				RNEwr.x[0][k + 1] = vectorplus31_31(productwr, wr_m2); // 3x1


				domegar = RNEdwr.x[0][k];//3x1
				productdwr1 = productmatrix33_31(rotationr, domegar);//3x1

				cproductdwr1 = crossproductmatrix31_31(productwr, wr_m2);//3x1


				ta_zr=ta.x[0][5-k];
				dwr_m2_1=Mddth.x[5-k][0];
#if GravityCompensation	
				dwr_m2_1=0;
#endif				
				dwr_m2 = productmatrix11_31(dwr_m2_1, ta_zr);//3x1
				RNEdwr.x[0][k + 1] = vectorplus31_31_31(productdwr1, cproductdwr1, dwr_m2);//3x1

				vpr1 = crossproductmatrix31_31(domegar, PositionppR.x[0][k]);//3x1
				vpr2_1 = crossproductmatrix31_31(omegar, PositionppR.x[0][k]);//3x1
				vpr2 = crossproductmatrix31_31(omegar, vpr2_1);//3x1
				vpr3 = RNEdvr.x[0][k];//3x1
				vectorplusdvr1 = vectorplus31_31_31(vpr1, vpr2, vpr3);//3x1

				RNEdvr.x[0][k + 1] = productmatrix33_31(rotationr, vectorplusdvr1);//3x1

				positionCr = PositionC.x[0][5-k];//3x1
				Rdvcr1 = crossproductmatrix31_31(RNEdwr.x[0][k + 1], positionCr);//3x1
				Rdvcr2_1 = crossproductmatrix31_31(RNEwr.x[0][k + 1], positionCr);//3x1
				Rdvcr2 = crossproductmatrix31_31(RNEwr.x[0][k + 1], Rdvcr2_1);//3x1

				RNEdvcr.x[0][k + 1] = vectorplus31_31_31(Rdvcr1, Rdvcr2, RNEdvr.x[0][k + 1]);//3x1

				helpRNEdvcr = RNEdvcr.x[0][k + 1];//3x1
				helpRNEFr.x[0][0] = RNEmass[5-k] * helpRNEdvcr.x[0][0];
				helpRNEFr.x[1][0] = RNEmass[5-k] * helpRNEdvcr.x[1][0];
				helpRNEFr.x[2][0] = RNEmass[5-k] * helpRNEdvcr.x[2][0];

				RNEFr.x[0][k + 1] = helpRNEFr;//3x1

				RNENr_1_1 = Inertia.x[0][5-k];//3x3
				RNENr_1 = productmatrix33_31(RNENr_1_1, RNEdwr.x[0][k + 1]);//3x1
				RNENr_2 = productmatrix33_31(RNENr_1_1, RNEwr.x[0][k + 1]);//3x1
				RNENr_3 = crossproductmatrix31_31(RNEwr.x[0][k + 1], RNENr_2);//3x1

				RNENr.x[0][k + 1] = vectorplus31_31(RNENr_1, RNENr_3);//3x1
			}

			for (k = 6; k > 0; k = k - 1) {  ///inward iteration

				RNEfr_1 = productmatrix33_31(RR.x[k][0], RNEfr.x[0][k]);//3x1
				RNEfr.x[0][k - 1] = vectorplus31_31(RNEfr_1, RNEFr.x[0][k]);//3x1

				inwardpositionCr = PositionC.x[0][6-k];//3x1
				RNEnr_1 = productmatrix33_31(RR.x[k][0], RNEnr.x[0][k]);//3x1
				RNEnr_2 = crossproductmatrix31_31(inwardpositionCr, RNEFr.x[0][k]);//3x1
				RNEnr_3 = crossproductmatrix31_31(PositionppR.x[0][k], RNEfr_1);//3x1

				RNEnr.x[0][k - 1] = vectorplus31_31_31_31(RNENr.x[0][k], RNEnr_1, RNEnr_2, RNEnr_3);//3x1

				helpFinaltorquer = RNEnr.x[0][k - 1];//3x1
				helpFinaltorquer_1= transposeMatrix_31(helpFinaltorquer);//1x3
				ta_zr_1 = ta.x[0][6-k];//3x1
				Finaltorque.x[6-k][0] = productmatrix13_31(helpFinaltorquer_1,ta_zr_1);//double
			}

		//RNE 초기 조건 
		wl00.x[0][0] = 0;
		wl00.x[1][0] = 0;
		wl00.x[2][0] = 0;

		dwl00.x[0][0] = 0;
		dwl00.x[1][0] = 0;
		dwl00.x[2][0] = 0;

		dvl00.x[0][0] = 0;
		dvl00.x[1][0] = 0;
		dvl00.x[2][0] = 9.81; //0

		dvcl00.x[0][0] = 0;
		dvcl00.x[1][0] = 0;
		dvcl00.x[2][0] = 0;

		//Matrix_3X1 Fl00, Nl00, fl77, nl77;

		Fl00.x[0][0] = 0;
		Fl00.x[1][0] = 0;
		Fl00.x[2][0] = 0;

		Nl00.x[0][0] = 0;
		Nl00.x[1][0] = 0;
		Nl00.x[2][0] = 0;

		//f,n

		fl77.x[0][0] = 0;
		fl77.x[1][0] = 0;
		fl77.x[2][0] = 0;//-11.884312;//-ps_LforceZ_last;//-force_F_L[2];//-ps_LforceZ_last;//-forceZl;//-force_F_L[2];//-d_Fz_DSP;//

		nl77.x[0][0] = 0;//-0.155095;//-ps_LtorqueX_last;//-torque_F_L[0];//-ps_LtorqueX_last;//-torquexl;//-torque_F_L[0];//0;//
		nl77.x[1][0] = 0;//0.054012;//-ps_LtorqueY_last;//-torque_F_L[1];//-ps_LtorqueY_last;//-torqueyl;//-torque_F_L[1];//0;//
		nl77.x[2][0] = 0;
		// printf("\nL : fz=%lf tx=%lf ty=%lf",ps_LforceZ_last,ps_LtorqueX_last,ps_LtorqueY_last);

		RNEwl.x[0][0] = wl00;
		RNEdwl.x[0][0] = dwl00;
		RNEdvl.x[0][0] = dvl00;
		RNEdvcl.x[0][0] = dvcl00;
		RNEFl.x[0][0] = Fl00;
		RNENl.x[0][0] = Nl00;

		RNEfl.x[0][6] = fl77;
		RNEnl.x[0][6] = nl77;

		// B->R
			for (k = 0; k < 6; k++) { ///outward iteration

				rotationl = invRL.x[k][0]; //3x3
				omegal = RNEwl.x[0][k]; //3x1
				productwl = productmatrix33_31(rotationl, omegal);//3x1

				ta_zl=ta.x[0][k+6];//3x1
				wl_m2_1=Mdth.x[k+6][0];//double
#if GravityCompensation	
				wl_m2_1=0;
#endif
				wl_m2 = productmatrix11_31(wl_m2_1, ta_zl);//3x1

				RNEwl.x[0][k + 1] = vectorplus31_31(productwl, wl_m2); // w(i+1)

				domegal = RNEdwl.x[0][k];//3x1
				productdwl1 = productmatrix33_31(rotationl, domegal);//3x1

				cproductdwl1 = crossproductmatrix31_31(productwl, wl_m2);//3x1

				ta_zl=ta.x[0][k+6];//3x1
				dwl_m2_1=Mddth.x[k+6][0];//double
#if GravityCompensation	
				dwl_m2_1=0;
#endif
				dwl_m2 = productmatrix11_31(dwl_m2_1, ta_zl);

				RNEdwl.x[0][k + 1] = vectorplus31_31_31(productdwl1, cproductdwl1, dwl_m2);//3x1

				vpl1 = crossproductmatrix31_31(domegal, PositionppL.x[0][k]);//3x1
				vpl2_1 = crossproductmatrix31_31(omegal, PositionppL.x[0][k]);//3x1
				vpl2 = crossproductmatrix31_31(omegal, vpl2_1);//3x1
				vpl3 = RNEdvl.x[0][k];//3x1
				vectorplusdvl1 = vectorplus31_31_31(vpl1, vpl2, vpl3);//3x1

				RNEdvl.x[0][k + 1] = productmatrix33_31(rotationl, vectorplusdvl1);//3x1

				positionCl = PositionC.x[0][k+6];//3x1
				Rdvcl1 = crossproductmatrix31_31(RNEdwl.x[0][k + 1], positionCl);//3x1
				Rdvcl2_1 = crossproductmatrix31_31(RNEwl.x[0][k + 1], positionCl);//3x1
				Rdvcl2 = crossproductmatrix31_31(RNEwl.x[0][k + 1], Rdvcl2_1);//3x1

				RNEdvcl.x[0][k + 1] = vectorplus31_31_31(Rdvcl1, Rdvcl2, RNEdvl.x[0][k + 1]);//3x1

				helpRNEdvcl = RNEdvcl.x[0][k + 1];//3x1
				helpRNEFl.x[0][0] = RNEmass[k+6] * helpRNEdvcl.x[0][0];//m*dvc
				helpRNEFl.x[1][0] = RNEmass[k+6] * helpRNEdvcl.x[1][0];
				helpRNEFl.x[2][0] = RNEmass[k+6] * helpRNEdvcl.x[2][0];

				RNEFl.x[0][k + 1] = helpRNEFl;//3x1

				RNENl_1_1 = Inertia.x[0][k+6];//3x3
				RNENl_1 = productmatrix33_31(RNENl_1_1, RNEdwl.x[0][k + 1]);//3x1
				RNENl_2 = productmatrix33_31(RNENl_1_1, RNEwl.x[0][k + 1]);//3x1
				RNENl_3 = crossproductmatrix31_31(RNEwl.x[0][k + 1], RNENl_2);//3x1

				RNENl.x[0][k + 1] = vectorplus31_31(RNENl_1, RNENl_3);//3x1
			}

			for (k = 6; k > 0; k = k - 1) {  ///inward iteration

				RNEfl_1 = productmatrix33_31(RL.x[k][0], RNEfl.x[0][k]);//3x1
				RNEfl.x[0][k - 1] = vectorplus31_31(RNEfl_1, RNEFl.x[0][k]);//3x1

				inwardpositionCl = PositionC.x[0][k+5];//3x1
				RNEnl_1 = productmatrix33_31(RL.x[k][0], RNEnl.x[0][k]);//3x1
				RNEnl_2 = crossproductmatrix31_31(inwardpositionCl, RNEFl.x[0][k]);//3x1
				RNEnl_3 = crossproductmatrix31_31(PositionppL.x[0][k], RNEfl_1);//3x1

				RNEnl.x[0][k - 1] = vectorplus31_31_31_31(RNENl.x[0][k], RNEnl_1, RNEnl_2, RNEnl_3);//3x1

			/*
			helpFinaltorque = RNEn.x[0][i - 1];//n(i)-1
			Finaltorque.x[6-i][0] = productmatrix11_31(helpFinaltorque_1,ta_z);
			*/

				helpFinaltorquel = RNEnl.x[0][k - 1];//3x1
				helpFinaltorquel_1= transposeMatrix_31(helpFinaltorquel);//1x3
				ta_zl_1 = ta.x[0][k+5];//3x1
				Finaltorque.x[k+5][0] = productmatrix13_31(helpFinaltorquel_1,ta_zl_1);//double
				// printf("Finaltorque.x[%d][0]=%lf",k+5,Finaltorque.x[k+5][0]);
			}

		torque1 = Finaltorque.x[0][0];
		torque2 = Finaltorque.x[1][0];
		torque3 = Finaltorque.x[2][0];
		torque4 = Finaltorque.x[3][0];
		torque5 = Finaltorque.x[4][0];
		torque6 = Finaltorque.x[5][0];
		torque7 = Finaltorque.x[6][0];
		torque8 = Finaltorque.x[7][0];
		torque9 = Finaltorque.x[8][0];
		torque10 = Finaltorque.x[9][0];
		torque11 = Finaltorque.x[10][0];
		torque12 = Finaltorque.x[11][0];
	
		double torque[12]={torque1,torque2,torque3,torque4,torque5,torque6,torque7,torque8,torque9,torque10,torque11,torque12};

		// printf("\nankleroll  /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:-R=%6lf ,L=%6lf",_th_torque[0]*180/pi, _th_torque[11]*180/pi,_th_encoder[0]*180/pi,_th_encoder[11]*180/pi,_error_th_1.x[0][0],_error_th_1.x[11][0],_error_dth_1.x[0][0],_error_dth_1.x[11][0],-torque[0],torque[11]);
		// printf("\nanklepitch /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:-R=%6lf ,-L=%6lf",_th_torque[1]*180/pi, _th_torque[10]*180/pi,_th_encoder[1]*180/pi,_th_encoder[10]*180/pi,_error_th_1.x[1][0],_error_th_1.x[10][0],_error_dth_1.x[1][0],_error_dth_1.x[10][0],-torque[1],-torque[10]);
		// printf("\nkneepitch  /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,L=%6lf",_th_torque[2]*180/pi, _th_torque[9]*180/pi,_th_encoder[2]*180/pi,_th_encoder[9]*180/pi,_error_th_1.x[2][0],_error_th_1.x[9][0],_error_dth_1.x[2][0],_error_dth_1.x[9][0],torque[2],torque[9]);
		// printf("\nhippitch   /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,L=%6lf",_th_torque[3]*180/pi, _th_torque[8]*180/pi,_th_encoder[3]*180/pi,_th_encoder[8]*180/pi,_error_th_1.x[3][0],_error_th_1.x[8][0],_error_dth_1.x[3][0],_error_dth_1.x[8][0],torque[3],torque[8]);
		// printf("\nhiproll    /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,-L=%6lf",_th_torque[4]*180/pi, _th_torque[7]*180/pi,_th_encoder[4]*180/pi,_th_encoder[7]*180/pi,_error_th_1.x[4][0],_error_th_1.x[7][0],_error_dth_1.x[4][0],_error_dth_1.x[7][0],torque[4],-torque[7]);
		// printf("\nhipyaw     /des :R=%6lf ,L=%6lf /enc:R=%6lf ,L=%6lf /errth:R=%6lf ,L=%6lf /errdth:R=%6lf ,L=%6lf /torque:R=%6lf ,-L=%6lf",_th_torque[5]*180/pi, _th_torque[6]*180/pi,_th_encoder[5]*180/pi,_th_encoder[6]*180/pi,_error_th_1.x[5][0],_error_th_1.x[6][0],_error_dth_1.x[5][0],_error_dth_1.x[6][0],torque[5],-torque[6]);
		
#if (onlyOneMotorTorque==0)		
		
		mMotors[ankleRollR]->setTorque(-torque[0]);
		mMotors[anklePitchR]->setTorque(-torque[1]);
		mMotors[kneePitchR]->setTorque(torque[2]);
		mMotors[hipPitchR]->setTorque(torque[3]);
		mMotors[hipRollR]->setTorque(torque[4]);
		mMotors[hipYawR]->setTorque(torque[5]);
		mMotors[hipYawL]->setTorque(-torque[6]);
		mMotors[hipRollL]->setTorque(-torque[7]);
		mMotors[hipPitchL]->setTorque(torque[8]);
		mMotors[kneePitchL]->setTorque(torque[9]);
		mMotors[anklePitchL]->setTorque(-torque[10]);
		mMotors[ankleRollL]->setTorque(torque[11]);
#elif (onlyOneMotorTorque==1)
		if(_sn==0)
		{
			mMotors[ankleRollR]->setTorque(0);
			mMotors[anklePitchR]->setTorque(0);
			mMotors[kneePitchR]->setTorque(0);
			mMotors[hipPitchR]->setTorque(0);
			mMotors[hipRollR]->setTorque(0);
			mMotors[hipYawR]->setTorque(0);
			mMotors[hipYawL]->setTorque(0);
			mMotors[hipRollL]->setTorque(0);
			mMotors[hipPitchL]->setTorque(0);
			mMotors[kneePitchL]->setTorque(0);
			mMotors[anklePitchL]->setTorque(0);
			mMotors[ankleRollL]->setTorque(0);
			
			mMotors[ankleRollR]->setPosition(0);
			mMotors[anklePitchR]->setPosition(0);
			mMotors[kneePitchR]->setPosition(0);
			mMotors[hipPitchR]->setPosition(0);
			mMotors[hipRollR]->setPosition(0);
			mMotors[hipYawR]->setPosition(0);
			mMotors[hipYawL]->setPosition(0);
			mMotors[hipRollL]->setPosition(0);
			mMotors[hipPitchL]->setPosition(0);
			mMotors[kneePitchL]->setPosition(1.57);
			mMotors[anklePitchL]->setPosition(0);
			mMotors[ankleRollL]->setPosition(0);
		}
		else
		{
			mMotors[ankleRollR]->setPosition(0);
			mMotors[anklePitchR]->setPosition(0);
			mMotors[kneePitchR]->setPosition(0);
			mMotors[hipPitchR]->setPosition(0);
			mMotors[hipRollR]->setPosition(0);
			mMotors[hipYawR]->setPosition(0);
			mMotors[hipYawL]->setPosition(0);
			mMotors[hipRollL]->setPosition(0);
			mMotors[hipPitchL]->setPosition(0);
			mMotors[kneePitchL]->setPosition(0);
			mMotors[kneePitchL]->setTorque(torque[9]);
			mMotors[anklePitchL]->setPosition(0);
			mMotors[ankleRollL]->setPosition(0);
		}
		printf("1.57-_sn*0.001=%lf\n",1.57-_sn*0.001);
#endif
		printf("\n-torque[0]= %lf,-torque[1]= %lf,torque[2]= %lf,torque[3]= %lf, torque[4]= %lf,torque[5]= %lf",
		-torque[0],-torque[1],torque[2],torque[3],torque[4],torque[5]);
		printf("\ntorque[11]= %lf,-torque[10]= %lf,torque[9]= %lf, torque[8]= %lf,-torque[7]= %lf,-torque[6]= %lf",
		torque[11],-torque[10],torque[9],torque[8],-torque[7],-torque[6]);

		rtorque1 = mMotors[ankleRollR] -> getTorqueFeedback(); 
		rtorque2 = mMotors[anklePitchR] -> getTorqueFeedback();
		rtorque3 = mMotors[kneePitchR] -> getTorqueFeedback();
		rtorque4 = mMotors[hipPitchR] -> getTorqueFeedback();
		rtorque5 = mMotors[hipRollR] -> getTorqueFeedback();
		rtorque6 = mMotors[hipYawR] -> getTorqueFeedback();
		rtorque7 = mMotors[hipYawL] -> getTorqueFeedback();
		rtorque8 = mMotors[hipRollL] -> getTorqueFeedback();
		rtorque9 = mMotors[hipPitchL] -> getTorqueFeedback();
		rtorque10 = mMotors[kneePitchL] -> getTorqueFeedback();
		rtorque11 = mMotors[anklePitchL] -> getTorqueFeedback();
		rtorque12 = mMotors[ankleRollL] -> getTorqueFeedback();
		for(int a=0;a<12;a++)
		{
			_th_printf[a]=_th_torque[a];
		}
		printf("\n step count = %d",datacountft);
		datacountft++;	
		printf("\n time  = %lf", walkingdatatime*samplingtime);
		walkingdatatime++;
		fprintf(rne,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			_th_printf[0], _th_printf[1], _th_printf[2], _th_printf[3], _th_printf[4],
			_th_printf[5],_th_printf[6], _th_printf[7], _th_printf[8], _th_printf[9],
			_th_printf[10],_th_printf[11],_th_encoder[0],_th_encoder[1],_th_encoder[2],
			_th_encoder[3],_th_encoder[4],_th_encoder[5],_th_encoder[6],_th_encoder[7],
			_th_encoder[8],_th_encoder[9],_th_encoder[10],_th_encoder[11],_error_th_1.x[0][0],
			_error_th_1.x[1][0],_error_th_1.x[2][0],_error_th_1.x[3][0],_error_th_1.x[4][0],_error_th_1.x[5][0],
			_error_th_1.x[6][0],_error_th_1.x[7][0],_error_th_1.x[8][0],_error_th_1.x[9][0],_error_th_1.x[10][0],
			_error_th_1.x[11][0],_error_dth_1.x[0][0],_error_dth_1.x[1][0],_error_dth_1.x[2][0],_error_dth_1.x[3][0],
			_error_dth_1.x[4][0],_error_dth_1.x[5][0],_error_dth_1.x[6][0],_error_dth_1.x[7][0],_error_dth_1.x[8][0],
			_error_dth_1.x[9][0],_error_dth_1.x[10][0],_error_dth_1.x[11][0]);
		
		// fprintf(slipcontrol, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf\n",
		// 	frefY_print, fslipy_print, forceYr, forceYl, local_gpsrz, pre_ycom+pre_ycom_c, ftzmpy, forceZr, forceZl, phaseflag,
		// 	realphaseflag, ssptime, dsptime, stepcheck, sspgpsrz, sspgpsrx ,frefX_print, fslipx_print, footdegreeR,sspdegreeR,
		// 	localFootDegree, yComVFinalPrint, none, none, _ycom_acc_measure, comDegree, footdegreeL);
		// fprintf(footprint, "%lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %d\n",
		// 	gpsrx, gpslx, gpsrz, gpslz, footradR, footradL, sspflag,local_gpsrx, local_gpsrz,footRY_acc,
		// 	footLY_acc, footRYAccLpf, footLYAccLpf,new_phase_num);
		// fprintf(cpt,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf\n",
		// ZMP_c[0]+_px, ZMP_c[1]+_py, phaseflag, realphaseflag,_px + _xcom_CPT,
		//  _py+_ycom_CPT, _xcom_nominal_global, _ycom_nominal_global, ftzmpx + _px, ftzmpy + _py,
		//  _px,_py,ZMP_global_d[0],ZMP_global_d[1],ZMP_c[0],
		//   ZMP_c[1],CP_cur[0],CP_cur[1],CP_cur_global_ssp[0],CP_cur_global_ssp[1],
		//   _targetxcom, _targetycom,CP_ref[0],CP_ref[1],CP_E[0],
		//   CP_E[1],CP_ref_global_ssp[0],CP_ref_global_ssp[1],phaseflag,ZMP_global_d[0],
		//   ZMP_global_d[1],CP_cur_global_dsp[0],CP_cur_global_dsp[1],CP_ref_global_dsp[0],CP_ref_global_dsp[1],
		//   inv_pos.x[0][0],inv_pos.x[1][0]);

		myStep();
        _sn++;
	}

	_sn=0;
    // printf("\n------------------------------------------------------------------------");
    // printf("\n--------------------------TORQUE MODE STAND OFF-------------------------");
    // printf("\n------------------------------------------------------------------------");
	double _dds= 0.1257;
	while (1) {
		// printf("main while _sn = %d",_sn);
		// start1 = clock();
		//printf("\n%c#############%c @@@@@@@@@@@@@@@@", gpscoordinateR, gpscoordinateL);
		gpsr = mGPSR->getValues();
		gpsl = mGPSL->getValues();
		// gpscoordinateR= = mGPSR -> getCoordinateSystem();
		// gpscoordinateL= = mGPSL -> getCoordinateSystem();
		compassr = mCompassR->getValues();
		compassl = mCompassL->getValues();
		compassc=mCompassC->getValues();

		gpsrx = gpsr[0] - gpsrxoffset;
		gpsry = gpsr[1] - gpsryoffset;
		gpsrz = -(gpsr[2] - gpsrzoffset);
		gpslx = gpsl[0] - gpsrxoffset;
		gpsly = gpsl[1] - gpsryoffset;
		gpslz = -(gpsl[2] - gpsrzoffset);

		compassrx = compassr[0];
		compassry = compassr[1];
		compassrz = compassr[2];
		compasslx = compassl[0]; 
		compassly = compassl[1];
		compasslz = compassl[2];
		compasscx = compassc[0];
		compasscy = compassc[1];
		compasscz = compassc[2];

		footradR = atan2(compassrx, compassry);
		footradL = atan2(compasslx, compassly);
		comRad=atan2(compasscx,compasscy);
		footdegreeR = (footradR - 1.5708) / M_PI * 180.0;if (footdegreeR<-180){footdegreeR+=360; } 
		footdegreeR=footdegreeR-footdegreeRoffset;
		footdegreeL = (footradL - 1.5708) / M_PI * 180.0;if (footdegreeL<-180){footdegreeL+=360; } 
		footdegreeL=footdegreeL-footdegreeLoffset;
		comDegree=(comRad-1.5708)/M_PI*180.0;if(comDegree<-180){comDegree+=360;}
		comDegree=comDegree-comdegreeOffset;


		// printf("\n gpsrx : %lf ", gpsrx);
		// printf("\t gpsry : %lf ", gpsry);
		// printf("\t gpsrz : %lf ", gpsrz);
		// printf("\t\t gpslx : %lf ", gpslx);
		// printf("\t gpsly : %lf ", gpsly);
		// printf("\t gpslz : %lf ", gpslz);
		// printf("\n compassrx : %lf ", compassrx);
		// printf("\t compassry : %lf ", compassry);
		// printf("\t compassrz : %lf ", compassrz);
		// printf("\t\t compasslx : %lf ", compasslx);
		// printf("\t compassly : %lf ", compassly);
		// printf("\t compasslz : %lf ", compasslz);

		// printf("\n================footradR : %lf\t footradL : %lf=====================", footradR, footradL);
		// printf("\t gpscoordinateR : %c ", gpscoordinateR);
		// printf("\t gpscoordinateL : %c ", gpscoordinateL);

		ftMotors[0]->setPosition(0);
		ftMotors[1]->setPosition(0);
		ftMotors[2]->setPosition(0);
		ftMotors[3]->setPosition(0);
		ftMotors[4]->setPosition(0);
		ftMotors[5]->setPosition(0);
//dd
		torquexr = ftMotors[0]->getTorqueFeedback();
		torqueyr = ftMotors[1]->getTorqueFeedback();
		torquezr = ftMotors[2]->getTorqueFeedback();
		torquexl = ftMotors[3]->getTorqueFeedback();
		torqueyl = ftMotors[4]->getTorqueFeedback();
		torquezl = ftMotors[5]->getTorqueFeedback();

		mMotors[ankleRollR]-> enableTorqueFeedback(samplingTime);
		mMotors[anklePitchR]-> enableTorqueFeedback(samplingTime);
		mMotors[kneePitchR]-> enableTorqueFeedback(samplingTime);
		mMotors[hipPitchR]-> enableTorqueFeedback(samplingTime);
		mMotors[hipRollR]-> enableTorqueFeedback(samplingTime);
		mMotors[hipYawR]-> enableTorqueFeedback(samplingTime);
		mMotors[hipYawL]-> enableTorqueFeedback(samplingTime);
		mMotors[hipRollL]-> enableTorqueFeedback(samplingTime);
		mMotors[hipPitchL]-> enableTorqueFeedback(samplingTime);
		mMotors[kneePitchL]-> enableTorqueFeedback(samplingTime);
		mMotors[anklePitchL]-> enableTorqueFeedback(samplingTime);
		mMotors[ankleRollL]-> enableTorqueFeedback(samplingTime);
  
		F_Lu = mFlu->getValues();
		F_Ld = mFld->getValues();
		F_Ru = mFru->getValues();
		F_Rd = mFrd->getValues();
		Frxu = F_Ru[0];
		Fryu = F_Ru[1];
		Frzu = F_Ru[2];
		Frxd = F_Rd[2];
		Fryd = F_Rd[1];
		Frzd = F_Rd[0];
		Flxu = F_Lu[0];
		Flyu = F_Lu[1];
		Flzu = F_Lu[2]; 
		Flxd = F_Ld[2];
		Flyd = F_Ld[1];
		Flzd = F_Ld[0];
		FXl = 0, FYl = 0, FZl = 0, FXr = 0, FYr = 0, FZr = 0;
		Flxu >= Flxd ? FXl = Flxu : FXl = -Flxd;
		Flyu >= Flyd ? FYl = Flyu : FYl = -Flyd;
		Flzu >= Flzd ? FZl = Flzu : FZl = -Flzd;
		
		Frxu >= Frxd ? FXr = Frxu : FXr = -Frxd;
		Fryu >= Fryd ? FYr = Fryu : FYr = -Fryd;
		Frzu >= Frzd ? FZr = Frzu : FZr = -Frzd;
		
		// printf("\n torquerollR : %lf ",torquexr);
		// printf("\t torquepitchR : %lf ",torqueyr);
		// printf("\t torqueyawR : %lf ",torquezr);
		// printf("\t\t torquerollL : %lf ",torquexl);
		// printf("\t torquepitchL : %lf ",torqueyl);
		// printf("\t torqueyawL : %lf ",torquezl);

		acc = mAccelerometer->getValues();
		_xcom_acc_measure = acc[0];
		_ycom_acc_measure = acc[1];
		_zcom_acc_measure = acc[2];
		footRX_acc = accR[0];
		footRY_acc = accR[1];
		footRZ_acc = accR[2];
		footLX_acc = accL[0];
		footLY_acc = accL[1];
		footLZ_acc = accL[2];
		
		//printf("@@@@@@@@@@@@@@@@@@@%lf\t%lf\t%lf\n",_xcom_acc_measure, _ycom_acc_measure, _zcom_acc_measure);

		// printf("\n ForceXr : %lf ",FXr/10);
		// printf("\t ForceYr : %lf ",FYr/10);
		// printf("\t ForceZr : %lf ",FZr/10);
		// printf("\t\t\t\t ForceXl : %lf ",FXl/10);
		// printf("\t ForceYl : %lf ",FYl/10);
		// printf("\t ForceZl : %lf ",FZl/10);
		// ////////////////////////////////////////////
		// printf("\nFLxu : %lf ",Flxu);
		// printf("\t Flyu : %lf ",Flyu);
		// printf("\t Flzu : %lf ",Flzu);
		// printf("\t Flxd : %lf ",Flxd);
		// printf("\t Flyd : %lf ",Flyd);
		// printf("\t Flzd : %lf ",Flzd);
		// printf("\t Frxu : %lf ",Flxu);
		// printf("\t Fryu : %lf ",Flyu);
		// printf("\t Frzu : %lf ",Frzu);
		// printf("\t Frxd : %lf ",Flxd);
		// printf("\t Fryd : %lf ",Flyd);
		// printf("\t Frzd : %lf \n",Frzd);

		// ftzmpxr=(-torqueyr-FXr/10*0.013)/Frzd/10;
		// ftzmpxl=(-torqueyl-FXl/10*0.013)/Flzd/10;
		// ftzmpyr=(torquexr-FYr/10*0.013)/Frzd/10;
		// ftzmpyl=(torquexl-FYl/10 *0.013)/Flzd/10;
		///////////////////////////////////////////
		ftzmpxr = (-torqueyr) / Frzu * 10;
		ftzmpxl = (-torqueyl) / Flzu * 10;
		ftzmpyr = (torquexr) / Frzu * 10;
		ftzmpyl = (torquexl) / Flzu * 10;
		///////////////////////////////////
		// printf("\t ftzmpxR : %lf, ftzmpyR : %lf",ftzmpxr, ftzmpyr);
		// printf("\t ftzmpxL : %lf, ftzmpyL : %lf",ftzmpxl, ftzmpyl);
		///////////////////////////////
		// ftzmpx=(-torqueyr-(footup*(FXr/10))-torqueyl-(footup*(FXl/10)))/((Frzd/10)+(Flzd/10));
		// ftzmpy=(+torquexr+(footup*(FYr/10))+torquexl+(footup*(FYl/10)))/((Frzd/10)+(Flzd/10));
		///////////////////////////////////////////////////////////////////////////
		// ftzmpx=((Frzd/10)*(ftzmpxr-FS/2)+(Flzd/10)*(ftzmpxl+FS/2))/((Frzd/10)+(Flzd/10));
		// ftzmpy=((Frzd/10)*(ftzmpyr-0.0475)+(Flzd/10)*(ftzmpyl+0.0475))/((Frzd/10)+(Flzd/10));
		// printf("new_phase_num=%d\n", new_phase_num);
		forceZr = (FZr / 10);
		forceZl = (FZl / 10);
		forceXr = (FXr / 10);
		forceXl = (FXl / 10);
		forceYr = (FYr / 10);
		forceYl = (FYl / 10);

		if (Frzu == 0) {
			ftzmpxr = 0;
			ftzmpyr = 0;
		}
		if (Flzu == 0) {
			ftzmpxl = 0;
			ftzmpyl = 0;
		}
		footup = 0;
		// printf("\nprelocalstrideF=%lf  prelocalstrideL=%lf, lsxdes=%lf px=%lf lsydes=%lf py=%lf\n", pre_local_stride_F,pre_local_stride_L,_lsx_des,_px,_lsy_des,_py);
		// printf("------------------------------------------------------------------------\n");
		// printf("--------------------------new_phase_num = %d------------------------\n",new_phase_num);
		// printf("------------------------------------------------------------------------\n");
		if (new_phase_num == 2)//DSP_R
		{
			local_gpsrx_ssp_begin=gpsrx;
			local_gpsrz_ssp_begin=gpsrz;

			local_gpsrx=gpslx-local_gpslx_ssp_begin;
			local_gpsrz=gpslz-local_gpslz_ssp_begin;

			localFootDegreeRSspI=footdegreeR;
			localFootDegree=footdegreeL-localFootDegreeLSspI;
			// printf("localFootDegree=%lf footdegreeL=%lf localFootDegreeLSspI=%lf\n",localFootDegree, footdegreeL,localFootDegreeLSspI);
			ftzmpx = ((Flzu/10)*(ftzmpxl - pre_local_stride_F) + (Frzu/10) * ftzmpxr) / ((Frzu/10) + (Flzu/10));
			ftzmpy = ((Flzu/10)*(ftzmpyl + pre_local_stride_L) + (Frzu/10) * ftzmpyr) / ((Frzu/10) + (Flzu/10));
			// printf("222222222222222222222222222222");
		}

		else if (new_phase_num == 4)//DSP_L
		{
            local_gpslx_ssp_begin=gpslx;
			local_gpslz_ssp_begin=gpslz;
			local_gpsrx=gpsrx-local_gpsrx_ssp_begin;
			local_gpsrz=gpsrz-local_gpsrz_ssp_begin;

			localFootDegreeLSspI=footdegreeL;
			localFootDegree=footdegreeR-localFootDegreeRSspI;
			// printf("localFootDegree=%lf footdegreeR=%lf localFootDegreeRSspI=%lf\n",localFootDegree, footdegreeR,localFootDegreeRSspI);
			ftzmpx = ((Flzu/10)*(ftzmpxl)+(Frzu/10) * (ftzmpxr - pre_local_stride_F)) / ((Frzu/10) + (Flzu/10));
			// printf("ftzmpxl=%lf ftzmpxr=%lf",ftzmpxl,ftzmpxr);
			ftzmpy = ((Flzu/10)*(ftzmpyl)+(Frzu/10) * (ftzmpyr - pre_local_stride_L)) / ((Frzu/10) + (Flzu/10));

			// printf("4444@@%lf %lf\n",(Flzd/10)*ftzmpxl, (Frzd/10)*(ftzmpxr-pre_local_stride_F));
			// printf("4444444444444444444444444444444444444444");
			
		}
		
		else if (new_phase_num == 1)//SSP_R
		{
			double FT_denominator_lx = (Flzu/10) * (ftzmpxl + (_lsx_des - _px));
			double FT_denominator_ly = (Flzu/10) * (ftzmpyl + (_lsy_des - _py));

			local_gpsrx=gpsrx-local_gpsrx_ssp_begin;
			local_gpsrz=gpsrz-local_gpsrz_ssp_begin;
 
			localFootDegree=footdegreeR-localFootDegreeRSspI;
			
			ftzmpx = (FT_denominator_lx + (Frzu/10) * ftzmpxr) / ((Frzu/10) + (Flzu/10));
			ftzmpy = (FT_denominator_ly + (Frzu/10) * ftzmpyr) / ((Frzu/10) + (Flzu/10));
			// printf("localFootDegree=%lf footdegreeR=%lf localFootDegreeRSspI=%lf\n",localFootDegree, footdegreeR,localFootDegreeRSspI);
			// printf("111111111111111111111111111111");
		}
		else if (new_phase_num == 3)//SSP_L
		{
			
			double FT_denominator_rx = (Frzu/10) * (ftzmpxr + (_rsx_des - _px));
			double FT_denominator_ry = (Frzu/10) * (ftzmpyl + (_rsy_des - _py));
			//printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!rsx des=%lf   rsy des = %lf   _py=%lf\n", _rsx_des, _rsy_des,_py);
			
			local_gpsrx=gpslx-local_gpslx_ssp_begin;
			local_gpsrz=gpslz-local_gpslz_ssp_begin;
			
			localFootDegree=footdegreeL-localFootDegreeLSspI;
			
			ftzmpx = ((Flzu/10)*(ftzmpxl)+FT_denominator_rx) / ((Frzu/10) + (Flzu/10));
			ftzmpy = ((Flzu/10)*(ftzmpyl)+FT_denominator_ry) / ((Frzu/10) + (Flzu/10));
			
			// printf("localFootDegree=%lf footdegreeL=%lf localFootDegreeLSspI=%lf\n",localFootDegree, footdegreeL,localFootDegreeLSspI);
			// printf("3333333333333333333333333333333");
		}
		// printf("\nzmpmeasured : _lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);

		if (phase == DSP_R || phase == DSP_L) {
			phaseflag = 1;
		}
		else {
			phaseflag = 0;
		}
		if (((Frzu/10)>0 && (Flzu/10) >0) || ((Frzu/10) == 0 && (Flzu/10) == 0))
		{
			realphaseflag = 1;
		}
		else if ((Frzu/10)>0)
		{
			sspflag = 2;
			realphaseflag = 0;
		}
		else if ((Flzu/10)>0)
		{
			sspflag = 3;
			realphaseflag = 0;
		}
		else
		{
			realphaseflag = 0;
		}
		
		if(_i!=_pi)
		{
			stepcheck=~stepcheck;
			_sspgpsrz=gpsrz;
			_sspgpsrx=gpsrx;
			_sspdegreeR=footdegreeR;
		}
		// printf("\nstepcheck=%d",stepcheck);
		_pi=_i;
		
		if(stepcheck==0)
        {
			sspgpsrz=gpsrz-_sspgpsrz;
			sspgpsrx=gpsrx-_sspgpsrx;
			sspdegreeR=footdegreeR-_sspdegreeR;
        }
        if(stepcheck==-1)
        {
			sspgpsrz=0;
			sspgpsrx=0;      
			sspdegreeR=0;  
        }

		pre_force_R[0]=forceXr;
		pre_force_R[1]=forceYr;
		pre_force_R[2]=forceZr;
		
		pre_force_L[0]=forceXl;
		pre_force_L[1]=forceYl;
		pre_force_L[2]=forceZl;
		
		pre_torque_R[0]=torquexr;
		pre_torque_R[1]=torqueyr;
		pre_torque_R[2]=torquezr;
		
		pre_torque_L[0]=torquexl;
		pre_torque_L[1]=torqueyl;
		pre_torque_L[2]=torquezl;
		
		pre_force_F_R[0]=force_F_R[0];
		pre_force_F_R[1]=force_F_R[1];
		pre_force_F_R[2]=force_F_R[2];
		
		pre_force_F_L[0]=force_F_L[0];
		pre_force_F_L[1]=force_F_L[1];
		pre_force_F_L[2]=force_F_L[2];
		
		pre_torque_F_R[0]=torque_F_R[0];
		pre_torque_F_R[1]=torque_F_R[1];
		pre_torque_F_R[2]=torque_F_R[2];
		
		pre_torque_F_L[0]=torque_F_L[0];
		pre_torque_F_L[1]=torque_F_L[1];
		pre_torque_F_L[2]=torque_F_L[2];

		LPF(10,pre_force_R,pre_force_F_R,force_F_R);
		LPF(10,pre_force_L,pre_force_F_L,force_F_L);
		
		LPF(10,pre_torque_R,pre_torque_F_R,torque_F_R);
		LPF(10,pre_torque_L,pre_torque_F_L,torque_F_L);

		i_th_encoder[0] = -mPositionSensors[ankleRollR]->getValue();
		i_th_encoder[1] = -mPositionSensors[anklePitchR]->getValue();
		i_th_encoder[2] = mPositionSensors[kneePitchR]->getValue();
		i_th_encoder[3] = mPositionSensors[hipPitchR]->getValue();
		i_th_encoder[4] = mPositionSensors[hipRollR]->getValue();
		i_th_encoder[5] = mPositionSensors[hipYawR]->getValue();
		i_th_encoder[6] = -mPositionSensors[hipYawL]->getValue();
		i_th_encoder[7] = -mPositionSensors[hipRollL]->getValue();
		i_th_encoder[8] = mPositionSensors[hipPitchL]->getValue();
		i_th_encoder[9] = mPositionSensors[kneePitchL]->getValue();
		i_th_encoder[10] = -mPositionSensors[anklePitchL]->getValue();
		i_th_encoder[11] = mPositionSensors[ankleRollL]->getValue();

		j_th_encoder[0] = i_th_encoder[0];
		j_th_encoder[1] = i_th_encoder[1];
		j_th_encoder[2] = i_th_encoder[2];
		j_th_encoder[3] = i_th_encoder[3];
		j_th_encoder[4] = i_th_encoder[4];
		j_th_encoder[5] = i_th_encoder[5];
		j_th_encoder[6] = i_th_encoder[6];
		j_th_encoder[7] = i_th_encoder[7];
		j_th_encoder[8] = i_th_encoder[8];
		j_th_encoder[9] = i_th_encoder[9];
		j_th_encoder[10] = i_th_encoder[10];
		j_th_encoder[11] = i_th_encoder[11];

        // forwardkinematics_FTB(j_th_encoder);

		if (_sn == 0)
		{
			i_th_current_p.x[0][0] = _th_position[0];
			i_th_current_p.x[1][0] = _th_position[1];
			i_th_current_p.x[2][0] = _th_position[2];
			i_th_current_p.x[3][0] = _th_position[3];
			i_th_current_p.x[4][0] = _th_position[4];
			i_th_current_p.x[5][0] = _th_position[5];
			i_th_current_p.x[6][0] = _th_position[6];
			i_th_current_p.x[7][0] = _th_position[7];
			i_th_current_p.x[8][0] = _th_position[8];
			i_th_current_p.x[9][0] = _th_position[9];
			i_th_current_p.x[10][0] = _th_position[10];
			i_th_current_p.x[11][0] = _th_position[11];
			//민하 누나의 i_th_current_p.x[]=_th_i[] 에서 변경

		}

		else {
			i_th_current_p = i_th_current_n;
		}
		i_th_current_n.x[0][0] = i_th_encoder[0];
		i_th_current_n.x[1][0] = i_th_encoder[1];
		i_th_current_n.x[2][0] = i_th_encoder[2];
		i_th_current_n.x[3][0] = i_th_encoder[3];
		i_th_current_n.x[4][0] = i_th_encoder[4];
		i_th_current_n.x[5][0] = i_th_encoder[5];
		i_th_current_n.x[6][0] = i_th_encoder[6];
		i_th_current_n.x[7][0] = i_th_encoder[7];
		i_th_current_n.x[8][0] = i_th_encoder[8];
		i_th_current_n.x[9][0] = i_th_encoder[9];
		i_th_current_n.x[10][0] = i_th_encoder[10];
		i_th_current_n.x[11][0] = i_th_encoder[11];

		iMdth = s_vectorminus121_121(i_th_current_n, i_th_current_p);
		
		if (_s[_i][6] == 0 && _s[_i][5] + samplingtime / 2 < _t) {
			// printf("------------------------------WALKING SET-------------------------------\n");
			// printf("------------------------------------------------------------------------\n");
			
			rot_th = rot_th + _s[_i - 1][4];//0
			_C = cosh(_s[_i + 1][2] / _Tc);
			_S = sinh(_s[_i + 1][2] / _Tc);
			_ppx = _px;
			_ppy = _py;
			_pvx_ter = _vx_ter;
			_pvy_ter = _vy_ter;
			_px = _px + cos(rot_th) * _s[_i][0] + sin(rot_th) * pow(-1.0, _i + 1.0) * _s[_i][1];
			_py = _py + sin(rot_th) * _s[_i][0] - cos(rot_th) * pow(-1.0, _i + 1.0) * _s[_i][1];
			_x_rot = cos(rot_th) * _s[_i][0] *sspf_coeff/2 - sin(rot_th) * pow(-1.0, _i) * _s[_i][1] * sspf_coeff/2;
			_y_rot = -sin(rot_th) * _s[_i][0] * sspf_coeff/2 + cos(rot_th) * pow(-1.0, _i + 1.0) * _s[_i][1] * sspf_coeff/2;
			_x_ter = cos(rot_th) * _x_rot - sin(rot_th) * _y_rot;
			_y_ter = sin(rot_th) * _x_rot + cos(rot_th) * _y_rot;
			if(onlySSPon==1)
			{
				_x_ter = cos(rot_th) * _x_rot - sin(rot_th) * _y_rot+pre_xcom_c;
				_y_ter = sin(rot_th) * _x_rot + cos(rot_th) * _y_rot+pre_ycom_c;
				fslipy=0;
				fslipx=0;
			}
			if(_i>2){
				_xcom_CPT=-_px+_xcom_CPT_global;
				_ycom_CPT=-_py+_ycom_CPT_global;
				// _xcom_CPT_v=-_xcom_CPT_v;
				// _ycom_CPT_v=-_ycom_CPT_v;
			}
			
			pre_fslipy = 0;
			pre_fslipx=0;
			
			// fslipy_print=0;
			// fslipx_print=0;
			// frefY_print=0;
			// frefX_print=0;

			CPx_ddot=0;
			CPy_ddot=0;
			pre_CPx_ddot=0;
			pre_CPy_ddot=0;
			if(CpSwitch==1||CpSwitch==2)
			{
				if(DspInitSwitch!=2)
				{
					CP_ref[0]=0;
					CP_ref[1]=0;
					ZMP_d[0]=0;
					ZMP_d[1]=0;
					pre_ycom_c = 0;
					pre_xcom_c = 0;
					pre_ycom_cv = 0;
					pre_xcom_cv=0;
				}
			}
			
			footLYAccLpf=0, footRYAccLpf=0;

			// printf("\tI = %d, _step_length=%d \n", _i,_step_length);
			
			if (_i == 2) {
				_y_ter = -L3 * (1 - dspf_coeff);
			}

			_x_rotf = cos(rot_th + _s[_i][4]) * _s[_i + 1][0] * sspf_coeff/2 - sin(rot_th + _s[_i][4]) * pow(-1.0, _i + 1.0) * _s[_i + 1][1] * sspf_coeff/2;
			_y_rotf = sin(rot_th + _s[_i][4]) * _s[_i + 1][0] * sspf_coeff/2 + cos(rot_th + _s[_i][4]) * pow(-1.0, _i + 1.0) * _s[_i + 1][1] * sspf_coeff/2;

			if (_i == _step_length && _i % 2 == 1) {
				_x_rotf = -sin(rot_th)*L3 * sspf_coeff/2;
				_y_rotf = cos(rot_th) * L3 * sspf_coeff/2;
			}
			else if (_i == _step_length && _i % 2 == 0) {
				_x_rotf = sin(rot_th) * L3 * sspf_coeff/2;
				_y_rotf = -cos(rot_th) * L3 * sspf_coeff/2;
			}

			_x_terf = cos(-rot_th) * _x_rotf - sin(-rot_th) * _y_rotf;
			_y_terf = sin(-rot_th) * _x_rotf + cos(-rot_th) * _y_rotf;

			_vx_ter = (_C * _x_ter + _x_terf) / (_Tc * _S);
			_vy_ter = (_C * _y_ter - _y_terf) / (_Tc * _S);
			_s[_i][6] = 1;
			_theta = 0;
			_swingtime = 2 * pi / (int)(_s[_i + 1][2] / samplingtime);
			
            dsp_x_i = cos(rot_th) * _s[_i][0] * sspf_coeff/2 - sin(rot_th) * pow(-1.0, _i)* _s[_i][1] * sspf_coeff/2 + _ppx;
			dsp_x_f = cos(rot_th) * _s[_i][0] * (sspf_coeff/2+dspf_coeff) - sin(rot_th) * pow(-1.0, _i) * _s[_i][1] * (sspf_coeff/2+dspf_coeff) + _ppx;
			dsp_dx_i = cos(-rot_th) * _vx_ter - sin(-rot_th) * _vy_ter;
			dsp_dx_f = cos(-rot_th) * _vx_ter - sin(-rot_th) * _vy_ter;
			dsp_y_i = sin(rot_th) * _s[_i][0] * sspf_coeff/2 + cos(rot_th) * pow(-1.0, _i) * _s[_i][1] * sspf_coeff/2 + _ppy;
			dsp_y_f = sin(rot_th) * _s[_i][0] * (sspf_coeff/2+dspf_coeff) + cos(rot_th) * pow(-1.0, _i) * _s[_i][1] * (sspf_coeff/2+dspf_coeff) + _ppy;
			dsp_dy_i = -sin(-rot_th) * _vx_ter - cos(-rot_th) * _vy_ter;
			dsp_dy_f = -sin(-rot_th) * _vx_ter - cos(-rot_th) * _vy_ter;
			// printf("\nI>2CPSwitch&&dspinitswitch==1\ndsp_x_i_c = %lf dsp_dx_i_c = %lf dsp_y_i_c = %lf  dsp_dy_i_c = %lf GLOBAL dsp_x_i_c = %lf GLOBAL dsp_y_i_c = %lf\n",dsp_x_i_c, dsp_dx_i_c, dsp_y_i_c, dsp_dy_i_c,dspXICGlobal,dspYICGlobal);
			// printf("\n dsp_x_i=%lf,dsp_x_f=%lf,dsp_dx_i=%lf,dsp_dx_f=%lf",dsp_x_i,dsp_x_f,dsp_dx_i,dsp_dx_f);
			// printf("\n dsp_y_i=%lf,dsp_y_f=%lf,dsp_dy_i=%lf,dsp_dy_f=%lf",dsp_y_i,dsp_y_f,dsp_dy_i,dsp_dy_f);
			if(_i>2 && (CpSwitch==1||CpSwitch==2) && DspInitSwitch==1){
				printf("\n dsp init change");
				dsp_x_i = dsp_x_i_c;
				dsp_y_i = dsp_y_i_c;
				dsp_dx_i = dsp_dx_i_c;
				dsp_dy_i = dsp_dy_i_c;
			}
			dsp_x_i_nominal = cos(rot_th) * _s[_i][0] * sspf_coeff/2 - sin(rot_th) * pow(-1.0, _i)* _s[_i][1] * sspf_coeff/2 + _ppx;
			dsp_x_f_nominal = cos(rot_th) * _s[_i][0] * (sspf_coeff/2+dspf_coeff) - sin(rot_th) * pow(-1.0, _i) * _s[_i][1] * (sspf_coeff/2+dspf_coeff) + _ppx;
			dsp_dx_i_nominal = cos(-rot_th) * _vx_ter - sin(-rot_th) * _vy_ter;
			dsp_dx_f_nominal = cos(-rot_th) * _vx_ter - sin(-rot_th) * _vy_ter;
			dsp_y_i_nominal = sin(rot_th) * _s[_i][0] * sspf_coeff/2 + cos(rot_th) * pow(-1.0, _i) * _s[_i][1] * sspf_coeff/2 + _ppy;
			dsp_y_f_nominal = sin(rot_th) * _s[_i][0] * (sspf_coeff/2+dspf_coeff) + cos(rot_th) * pow(-1.0, _i) * _s[_i][1] * (sspf_coeff/2+dspf_coeff) + _ppy;
			dsp_dy_i_nominal = -sin(-rot_th) * _vx_ter - cos(-rot_th) * _vy_ter;
			dsp_dy_f_nominal = -sin(-rot_th) * _vx_ter - cos(-rot_th) * _vy_ter;
			
			if (_i == 2) {
				dsp_y_i = L3;
				dsp_y_i_nominal=L3;
				// dsp_y_i=0;
				dsp_y_f = L3 + L3 * dspf_coeff;
				dsp_y_f_nominal=L3 + L3 * dspf_coeff;
				// dsp_y_f = L3;
			}
			
			if(CpSwitch==0 && dspDecelerator ==1 && (_i>(dspDeceleratorOn-2))){
				dsp_dx_i=dsp_dx_i/dspConstant;
            	dsp_dx_f=dsp_dx_f/dspConstant;
				dsp_dy_i=dsp_dy_i/dspConstant;
				dsp_dy_f=dsp_dy_f/dspConstant;
				// printf("%d, dspDeceleratoron###############################\n##########################################",_i);
			}
			cubicPolynomialInterpolationInitForDsp(dsp_x_i,dsp_x_f,dsp_dx_i,dsp_dx_f,dsp_y_i,dsp_y_f,dsp_dy_i,dsp_dy_f,_s[_i + 1][3],3,0);
			cubicPolynomialInterpolationInitForDsp(dsp_x_i_nominal,dsp_x_f_nominal,dsp_dx_i_nominal,dsp_dx_f_nominal,dsp_y_i_nominal,dsp_y_f_nominal,dsp_dy_i_nominal,dsp_dy_f_nominal,_s[_i + 1][3],3,1);
            
			_swingrot_time = (_s[_i + 1][4] + _s[_i][4]) / (int)(_s[_i + 1][2] / samplingtime);

			_baserot_time = _s[_i][4] / (int)(_s[_i + 1][2] / samplingtime);

			_i = _i + 1;
		}
		printf("_i = %d",_i);
		if (_s[_step_length + 1][5] + samplingtime / 2 < _t) {// 마지막 걸음
			// printf("------------------------------------------------------------------------\n");
			// printf("---------------------------LAST STEP START------------------------------\n");
			// printf("------------------------------------------------------------------------\n");
			rot_th = rot_th + _s[_i - 1][4];

			if (_i % 2 == 1) {
				dsp_y_i = cos(rot_th)*(-0.0740 + L3 * sspf_coeff/2) + _py;
				dsp_x_i = -sin(rot_th) * (-0.0740 + L3 * sspf_coeff/2) + _px;
				dsp_y_f = cos(rot_th) * (-L3) + _py;
				dsp_x_f = -sin(rot_th) * (-L3) + _px;
			}
			else if (_i % 2 == 0) {
				dsp_y_i = cos(rot_th) * (0.0740 - L3 * sspf_coeff/2) + _py;
				dsp_x_i = -sin(rot_th) * (0.0740 - L3 * sspf_coeff/2) + _px;
				dsp_y_f = cos(rot_th) * (L3)+_py;
				dsp_x_f = -sin(rot_th) * (L3)+_px;
			}

			dsp_dx_i = -sin(rot_th) * _pvy_ter;
			dsp_dx_f = 0;
			dsp_x_a0 = dsp_x_i;
			dsp_x_a1 = dsp_dx_i;
			dsp_x_a2 = 3 / pow(_s[_i][3], 2) * (dsp_x_f - dsp_x_i) - 2 / _s[_i][3] * dsp_dx_i - dsp_dx_f / _s[_i][3];
			dsp_x_a3 = -2 / pow(_s[_i][3], 3) * (dsp_x_f - dsp_x_i) + 1 / pow(_s[_i][3], 2) * (dsp_dx_f + dsp_dx_i);


			dsp_dy_i = cos(rot_th) * _pvy_ter;
			dsp_dy_f = 0;
			dsp_y_a0 = dsp_y_i;
			dsp_y_a1 = dsp_dy_i;
			dsp_y_a2 = 3 / pow(_s[_i][3], 2) * (dsp_y_f - dsp_y_i) - 2 / _s[_i][3] * dsp_dy_i - dsp_dy_f / _s[_i][3];
			dsp_y_a3 = -2 / pow(_s[_i][3], 3) * (dsp_y_f - dsp_y_i) + 1 / pow(_s[_i][3], 2) * (dsp_dy_f + dsp_dy_i);

			_dsp_x_des = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5] + samplingtime) + dsp_x_a2 * pow((_t - _s[_i - 1][5] + samplingtime), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5] + samplingtime), 3);
			_dsp_y_des = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5] + samplingtime) + dsp_y_a2 * pow((_t - _s[_i - 1][5] + samplingtime), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5] + samplingtime), 3);

			if (_i == 3) {
				_rsx_des = 0;
				_rsy_des = 0;
				_rsz_des = 0;
			}
			else if (_i % 2 == 0) {
				_lsx_des = _ppx;
				_lsy_des = _ppy;
				_lsz_des = 0;
			}
			else if (_i % 2 == 1) {
				_rsx_des = _ppx;
				_rsy_des = _ppy;
				_rsz_des = 0;
			}

			_targetxcom = _dsp_x_des;
			_targetycom = _dsp_y_des;

			
		}
		else if (_i % 2 == 0) {// 오른발 지지 왼발 스윙
			if ((_t - _s[_i - 1][5]) > _s[_i][3] + samplingtime / 2) {
                // printf("------------------------right foot supporting---------------------------\n");
				// printf("------------------------------------------------------------------------\n");
				localtime = _t - (_s[_i - 1][5] + _s[_i][3]);
				Tr=Tr_walk_ssp;
				_kv=2*(1.8/Tr);
				_kp=(1.8/Tr)*(1.8/Tr);
				footLYAccLpf=0, footRYAccLpf=0;
				DSPDelCheck=0;
				phase = SSP_R;
				if(fslipInitCheck==0 && sspStartInit==1)
				{
					pre_xcom_c=0, pre_ycom_c=0;
					pre_xcom_cv=0, pre_ycom_cv=0;
					fslipInitCheck=1;
				}
				if (phase == SSP_R) // right foot supporting
					new_phase_num = 1;
				// FsrSensing(_i, new_phase_num, _px, _py, _ppx, _ppy);
				/////////////////////////////////////////////////////
				// printf("\n\tI = %d, _step_length=%d \n", _i,_step_length);
				// printf("\n_px=%f _py=%f _ppx = %f _ppy = %f  \n ", _px, _py, _ppx, _ppy);
				_baserot = _baserot + _baserot_time;
				_lsr = _lsr + _swingrot_time;
				_theta = _theta + _swingtime;

				_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
				_ycom = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
				_xcom_origin=_xcom;
				_ycom_origin=_ycom;
				_xcom_v = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_ycom_v = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_xcom_acc = (-_x_ter / pow(_Tc, 2)) * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + (_vx_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_ycom_acc = (_y_ter / pow(_Tc, 2)) * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - (_vy_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
                _x_des = _px + _xcom;
				_y_des = _py + _ycom;
				pre_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc));
				pre_ycom = cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc)); 
				_lsx_des = (cos(rot_th) * _s[_i - 1][0] - sin(rot_th) * pow(-1.0, _i - 1.0) * _s[_i - 1][1] + cos(rot_th + _s[_i - 1][4]) * _s[_i][0] - sin(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) / (2 * pi) * (_theta - sin(_theta)) + _ppx;
				_lsy_des = (_py + (sin(rot_th + _s[_i - 1][4]) * _s[_i][0] + cos(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) - _ppy) / (2 * pi) * (_theta - sin(_theta)) + _ppy;
				_lsz_des = _stepheight / 2 * (1 - cos(_theta));
				_dd_lsx_des = (cos(rot_th) * _s[_i - 1][0] - sin(rot_th) * pow(-1.0, _i - 1.0) * _s[_i - 1][1] + cos(rot_th + _s[_i - 1][4]) * _s[_i][0] - sin(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) / (2 * pi) * pow(_dds, 2)*(sin(_theta));
				_dd_lsy_des = (_py + (sin(rot_th + _s[_i - 1][4]) * _s[_i][0] + cos(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) - _ppy) / (2 * pi) * pow(_dds, 2)*(sin(_theta));
				_dd_lsz_des = _stepheight / 2 * (cos(_theta))*pow(_dds, 2);
				
				if(CpSwitch==2){
					ZMP_c[0] = 0.0;
					ZMP_c[1] = 0.0;
#if (cpZmpCurrentOn==1)
						ZMP_c[0] = tq_yr_p1_pos/f_zr_p1_pos;
						ZMP_c[1] = tq_xr_p1_pos/f_zr_p1_pos;
						// printf("\ncpZmpCurrentOn : ZMP_c[0]=%lf,ZMP_c[1]=%lf_lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",ZMP_c[0],ZMP_c[1],_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);
#endif
					//Pc 계산

					//forwardkinematics_RTL(j_th_encoder);
					inversekinematicsRtoB(j_th_encoder);
					inv_jac_R = saveMatrix_66(i_jacobi);
					forwardkinematics_FTB(_th_encoder);
					inv_th_R.x[0][0] = iMdth.x[0][0];
					inv_th_R.x[1][0] = iMdth.x[1][0];
					inv_th_R.x[2][0] = iMdth.x[2][0];
					inv_th_R.x[3][0] = iMdth.x[3][0];
					inv_th_R.x[4][0] = iMdth.x[4][0];
					inv_th_R.x[5][0] = iMdth.x[5][0];

					inv_pos.x[0][0] = ITRBE[0][3] + _px;
					inv_pos.x[1][0] = ITRBE[1][3] + _py;
					inv_pos.x[2][0] = ITRBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_R, inv_th_R);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_R, inv_th_R);
					
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}
					else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					}                   
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					_xcom_nominal = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_c[0]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_ycom_nominal = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_c[1]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));

					_vxcom_nominal = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + ZMP_c[0]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_vycom_nominal = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)  + ZMP_c[1]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					DCM_ver3(0,Kf_S[0],_xcom_nominal,_vxcom_nominal,inv_pos.x[0][0],inv_vel.x[0][0],_px,_i,localtime);
					DCM_ver3(1,Kf_S[1],_ycom_nominal,_vycom_nominal,inv_pos.x[1][0],inv_vel.x[1][0],_py,_i,localtime);

#if protectZmpEscapeOn
					protectZmpEscape(new_phase_num);
#endif
					//pd 계산
					_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_d[0]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_ycom = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_d[1]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) ;
					_vxcom = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + ZMP_d[0]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_vycom = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)  + ZMP_d[1]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					//update된 com 궤적
					_x_des = _px + _xcom; 
					_y_des = _py + _ycom;
					// printf("\norigin com x=%lf y=%lf control com x=%lf y=%lf",_xcom_origin,_ycom_origin,_xcom,_ycom);
					//target com 궤적에 xcom ycom 대입
					dsp_y_i_c = _ycom+_py;
					dsp_x_i_c = _xcom+_px;
					dsp_dy_i_c = _vycom;
					dsp_dx_i_c = _vxcom;
					//dsp 시작 위치를 옮겨주기 위한 컨트롤값 저장
					_xcom_nominal_global=_xcom_nominal+_px;
					_ycom_nominal_global=_ycom_nominal+_py;
					CP_cur_global_ssp[0] = CP_cur[0] + _px;
					CP_cur_global_ssp[1] = CP_cur[1] + _py;
					CP_cur_global_dsp[0] = 0;//CP_E[0] + _px;
					CP_cur_global_dsp[1] = 0;//CP_E[1] + _py;
					CP_ref_global_ssp[0] = CP_ref[0] + _px;
					CP_ref_global_ssp[1] = CP_ref[1] + _py;
					CP_ref_global_dsp[0] = 0;//CP_ref[0] + _px;
					CP_ref_global_dsp[1] = 0;//CP_ref[1] + _py;
					//matlab plot 을 위한 과정
				}
				
				// fslipy = fslipKy*(-(M * _g /E _zc * (pre_ycom+pre_ycom_c - ftzmpy)) + forceYr);
				// fslipy = fslipKy*(-(M * _g / _zc * (pre_ycom - ftzmpy)) + forceYr);
				// frefY_print = M * _g / _zc * (pre_ycom+pre_ycom_c - ftzmpy);
				// frefY_print = M * _g / _zc * (pre_ycom - ftzmpy);
				if(fslipAddFoot==1)
				{
					fslipy = fslipKy*(-M * ( _ycom_acc_measure-footRY_acc)+forceYr); 
					fslipx = fslipKx*(-M*(_xcom_acc_measure-footRX_acc)+forceXr);
					frefY_print = M * ( _ycom_acc_measure-footRY_acc);
					frefX_print = M* ( _xcom_acc_measure-footRX_acc);
				}
				else
				{
					fslipy = fslipKy*(-M * (_ycom_acc_measure)+forceYr); 
					fslipx = fslipKx*(-M * (_xcom_acc_measure)+forceXr);
					frefY_print = M * ( _ycom_acc_measure);
					frefX_print = M* ( _xcom_acc_measure);
				}
				fslipy_print=fslipy/fslipKy;
				fslipx_print=fslipx/fslipKx;
				if(controlswitchYR==0 || _i<sspYFslipControlOn){fslipy=0;}
                if(controlswitchXR==0 || _i<sspXFslipControlOn){fslipx=0;}
				
				if(fslipControlOn==1){
					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*fslipy*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + fslipy * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*fslipx*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+fslipx*samplingtime;
					

					pre_ycom_c = _ycom_c;
					pre_xcom_c = _xcom_c;
					pre_ycom_cv = _ycom_cv;
					pre_xcom_cv = _xcom_cv;

					_ycom_fslip = _ycom + _ycom_c;
					_xcom_fslip = _xcom+_xcom_c;

					// _x_des = _px + _xcom; 
					// _y_des = _py + _ycom;
					_x_des = _px + _xcom_fslip;
					_y_des = _py + _ycom_fslip;
					// dspYICGlobal=dsp_y_i_c+_py;
					// dspXICGlobal=dsp_x_i_c+_px;
					dsp_y_i_c = _ycom_fslip+_py;
					dsp_x_i_c = _xcom_fslip+_px;
					// dsp_y_i_c =_targetycom;
					// dsp_x_i_c=_targetxcom;
					dsp_dy_i_c = _ycom_v+_ycom_cv;
					dsp_dx_i_c = _xcom_v+_xcom_cv;
					yComVFinalPrint=_ycom_v+_ycom_cv;
				}
				else if(CpSwitch==1){
					//forwardkinematics_RTL(j_th_encoder);
					inversekinematicsRtoB(j_th_encoder);
					inv_jac_R = saveMatrix_66(i_jacobi);
					forwardkinematics_FTB(_th_encoder);
					inv_th_R.x[0][0] = iMdth.x[0][0];
					inv_th_R.x[1][0] = iMdth.x[1][0];
					inv_th_R.x[2][0] = iMdth.x[2][0];
					inv_th_R.x[3][0] = iMdth.x[3][0];
					inv_th_R.x[4][0] = iMdth.x[4][0];
					inv_th_R.x[5][0] = iMdth.x[5][0];

					inv_pos.x[0][0] = ITRBE[0][3] + _px;
					inv_pos.x[1][0] = ITRBE[1][3] + _py;
					inv_pos.x[2][0] = ITRBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_R, inv_th_R);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_R, inv_th_R);
					
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}
					else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					}                   
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					
					CPT_ver2_ssp(new_phase_num,localtime,_i,Kf_S[0],Kf_S[1],rot_th,_x_ter,_vx_ter,_y_ter,_vy_ter,inv_pos.x[0][0],inv_vel.x[0][0],inv_pos.x[1][0],inv_vel.x[1][0],_px,_py);

					CP_cur_global_ssp[0] = CP_cur[0] + _px;
					CP_cur_global_ssp[1] = CP_cur[1] + _py;
					CP_cur_global_dsp[0] = 0;//CP_E[0] + _px;
					CP_cur_global_dsp[1] = 0;//CP_E[1] + _py;
					CP_ref_global_ssp[0] = CP_ref[0] + _px;
					CP_ref_global_ssp[1] = CP_ref[1] + _py;
					CP_ref_global_dsp[0] = 0;//CP_ref[0] + _px;
					CP_ref_global_dsp[1] = 0;//CP_ref[1] + _py;
					
					ZMP_global_d[0] = ZMP_d[0] + _px;
					ZMP_global_d[1] = ZMP_d[1] + _py;

					CPx_ddot=a_desired[0];
					CPy_ddot=a_desired[1];

					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*CPx_ddot*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+CPx_ddot*samplingtime;
					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*CPy_ddot*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + CPy_ddot * samplingtime;
					
					pre_CPx_ddot=CPx_ddot;
					pre_CPy_ddot=CPy_ddot;
					pre_ycom_c = _ycom_c;
					pre_xcom_c = _xcom_c;
					pre_ycom_cv = _ycom_cv;
					pre_xcom_cv = _xcom_cv;

					_xcom_CPT=_xcom+_xcom_c;
					_ycom_CPT=_ycom+_ycom_c;
					_xcom_CPT_global=_xcom_CPT+_px;
					_ycom_CPT_global=_ycom_CPT+_py;
					_xcom_CPT_v=_xcom_v+_xcom_cv;
					_ycom_CPT_v=_ycom_v+_ycom_cv;

					_x_des = _px + _xcom_CPT;
					_y_des = _py + _ycom_CPT;

					dsp_y_i_c = _ycom_CPT_global;
					dsp_x_i_c = _xcom_CPT_global;
					dsp_dy_i_c = _ycom_CPT_v;
					dsp_dx_i_c = _xcom_CPT_v;
					_xcom_nominal_global=_xcom+_px;
					_ycom_nominal_global=_ycom+_py;
				}
				_targetxcom = _x_des;
				_targetycom = _y_des;
				// printf("_targetxcom=%lf _targetycom=%lf \n",_x_des, _y_des);
				pre_fslipy=fslipy;
				pre_fslipx=fslipx;
                // printf("\ndsp_x_i_c = %lf dsp_y_i_c = %lf dsp_dx_i_c = %lf dsp_dy_i_c = %lf\n",dsp_x_i_c, dsp_y_i_c, dsp_dx_i_c, dsp_dy_i_c);
				
			}

			else {
				// printf("--------------------------------right foot DSP--------------------------\n");
				// printf("------------------------------------------------------------------------\n");
				localtime = _t - _s[_i - 1][5];
				Tr=Tr_walk_dsp;
				_kv=2*(1.8/Tr);
				_kp=(1.8/Tr)*(1.8/Tr);
				phase = DSP_R;
				fslipInitCheck=0;
				if (phase == DSP_R)
					new_phase_num = 2;
				// FsrSensing(_i, new_phase_num, _px, _py, _ppx, _ppy);
				// printf("_px=%f _py=%f _ppx = %f _ppy = %f _i=%d \n ", _px, _py, _ppx, _ppy,_i);
				
				_dsp_x_des = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5]) + dsp_x_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5]), 3);
				_dsp_y_des = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5]) + dsp_y_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5]), 3);
				_xcom = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5]) + dsp_x_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5]), 3);
				_ycom = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5]) + dsp_y_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5]), 3);
				_xcom_origin_global = dsp_x_a0_nominal + dsp_x_a1_nominal * (_t - _s[_i - 1][5]) + dsp_x_a2_nominal * pow((_t - _s[_i - 1][5]), 2) + dsp_x_a3_nominal * pow((_t - _s[_i - 1][5]), 3);
				_ycom_origin_global = dsp_y_a0_nominal + dsp_y_a1_nominal * (_t - _s[_i - 1][5]) + dsp_y_a2_nominal * pow((_t - _s[_i - 1][5]), 2) + dsp_y_a3_nominal * pow((_t - _s[_i - 1][5]), 3);
				
				_xcom_v = dsp_x_a1 + 2 * dsp_x_a2 *(_t - _s[_i - 1][5] ) + 3 * dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 2);
				_ycom_v = dsp_y_a1 + 2 * dsp_y_a2 * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 2);
				_xcom_v_nominal = dsp_x_a1_nominal + 2 * dsp_x_a2_nominal *(_t - _s[_i - 1][5]) + 3 * dsp_x_a3_nominal * pow((_t - _s[_i - 1][5]), 2);
				_ycom_v_nominal = dsp_y_a1_nominal + 2 * dsp_y_a2_nominal * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3_nominal * pow((_t - _s[_i - 1][5]), 2);
				_xcom_acc = 2 * dsp_x_a2 + 6 * dsp_x_a3 * (_t - _s[_i - 1][5] );
				_ycom_acc = 2 * dsp_y_a2 + 6 * dsp_y_a3 * (_t - _s[_i - 1][5] );
				
				if(CpSwitch==2)
				{
					_xcom_nominal_global=_xcom;
					_ycom_nominal_global=_ycom;
					//global com traj

					//forwardkinematics_FTB(j_th_encoder);
					//forwardkinematics_RTL(j_th_encoder);
					inversekinematicsRtoB(j_th_encoder);
					inv_jac_R = saveMatrix_66(i_jacobi);
					forwardkinematics_FTB(_th_encoder);
					inv_th_R.x[0][0] = iMdth.x[0][0];
					inv_th_R.x[1][0] = iMdth.x[1][0];
					inv_th_R.x[2][0] = iMdth.x[2][0];
					inv_th_R.x[3][0] = iMdth.x[3][0];
					inv_th_R.x[4][0] = iMdth.x[4][0];
					inv_th_R.x[5][0] = iMdth.x[5][0];

					inv_pos.x[0][0] = ITRBE[0][3] + _px;
					inv_pos.x[1][0] = ITRBE[1][3] + _py;
					inv_pos.x[2][0] = ITRBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_R, inv_th_R);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_R, inv_th_R);
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}

					else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					}                   
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					//current com traj
				}
				if(fslipAddFoot==1)
				{
					fslipy = fslipKyDsp *(-M * ( _ycom_acc_measure-((footLY_acc+footRY_acc)/2))+forceYl+forceYr);
					fslipx = fslipKxDsp *(-M * ( _xcom_acc_measure-((footLX_acc+footRX_acc)/2))+forceXl+forceXr);
					frefY_print = M * ( _ycom_acc_measure-((footLY_acc+footRY_acc)/2));
					frefX_print = M * ( _xcom_acc_measure-((footLX_acc+footRX_acc)/2));
				}
				else
				{
					fslipy = fslipKyDsp*(-M * ( _ycom_acc_measure)+forceYl+forceYr); 
					fslipx = fslipKxDsp*(-M * (_xcom_acc_measure)+forceXl+forceXr);
					frefY_print = M * ( _ycom_acc_measure);
					frefX_print = M* ( _xcom_acc_measure);
				}
				if(DSPDelFslipNoiseOn==1 && DSPDelCheck==0)
				{
					if(forceZr>0 && forceZl>0)
					{
						fslipy=0;
						frefY_print=0;
						DSPDelCheck=1;
					}
				} 
                
				fslipy_print=fslipy/fslipKyDsp;
				fslipx_print=fslipx/fslipKxDsp;
				// printf("\nfslipy_print=%lf, frefY_print=%lf _ycom_acc_measure= %lf footLY_acc=%lf,footRY_acc=%lf  (footLY_acc+footRY_acc)/2=%lf \n",fslipy_print,frefY_print,_ycom_acc_measure,footLY_acc,footRY_acc,(footLY_acc+footRY_acc)/2);
				// printf("\n_ycom_v=%lf\n",_ycom_v);
				
				if(fslipControlOn==1)
				{
					fslipx=0;
					
					
					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*fslipy*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + fslipy * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*fslipx*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+fslipx*samplingtime;
					if(dspFSlipControl==0 || _i<dspFslipControlOn)
					{
						_ycom_c=0;_ycom_cv=0;_xcom_c=0;_xcom_cv=0;
						// printf("dspFslipOFF!!!!!!!!!!!!!!!!!!");
					}

					
					pre_ycom_c = _ycom_c;
					pre_xcom_c=_xcom_c;
					pre_ycom_cv =_ycom_cv;
					pre_xcom_cv=_xcom_cv;

					_ycom_fslip = _ycom + _ycom_c;
					_xcom_fslip=_xcom+_xcom_c;
					// printf("_ycom_c=%lf _xcom_c=%lf _ycom_cv=%lf_xcom_cv= %lf",_ycom_c,_xcom_c ,_ycom_cv,_xcom_cv);
					
					_dsp_x_des = _xcom_fslip;
					_dsp_y_des = _ycom_fslip;
				}
				yComVFinalPrint=_ycom_v+_ycom_cv;
				if(CpSwitch==1 && cpDspSwitch==2)
				{
					CP_ref[0] = _xcom-_px+_Tc*_xcom_v;
					CP_ref[1] = _ycom-_py+_Tc*_ycom_v; 
					if(CpSwitch==1 && DspInitSwitch==1){
						CP_ref[0] = _xcom_origin_global-_px+_Tc*_xcom_v_nominal;
						CP_ref[1] = _ycom_origin_global-_py+_Tc*_ycom_v_nominal; 
					}
					CP_ref_global_ssp[0] = 0;//CP_ref[0] + _px;
					CP_ref_global_ssp[1] = 0;//CP_ref[1] + _py;
					CP_ref_global_dsp[0] = CP_ref[0] + _px;
					CP_ref_global_dsp[1] = CP_ref[1] + _py;
					
					//forwardkinematics_FTB(j_th_encoder);
					//forwardkinematics_RTL(j_th_encoder);
					inversekinematicsRtoB(j_th_encoder);
					inv_jac_R = saveMatrix_66(i_jacobi);
					forwardkinematics_FTB(_th_encoder);
					inv_th_R.x[0][0] = iMdth.x[0][0];
					inv_th_R.x[1][0] = iMdth.x[1][0];
					inv_th_R.x[2][0] = iMdth.x[2][0];
					inv_th_R.x[3][0] = iMdth.x[3][0];
					inv_th_R.x[4][0] = iMdth.x[4][0];
					inv_th_R.x[5][0] = iMdth.x[5][0];

					inv_pos.x[0][0] = ITRBE[0][3] + _px;
					inv_pos.x[1][0] = ITRBE[1][3] + _py;
					inv_pos.x[2][0] = ITRBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_R, inv_th_R);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_R, inv_th_R);

					
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}

					else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					}                   
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					CP_cur[0] = (inv_pos.x[0][0] - _px) + _Tc * (inv_vel.x[0][0]);
					CP_cur[1] = (inv_pos.x[1][0] - _py) + _Tc * (inv_vel.x[1][0]);
					CP_cur_global_ssp[0] = 0;//CP_cur[0] + _px;
					CP_cur_global_ssp[1] = 0;//CP_cur[1] + _py;
					CP_cur_global_dsp[0] = CP_cur[0] + _px;
					CP_cur_global_dsp[1] = CP_cur[1] + _py;

					ZMP_c[0] = 0.0;
					ZMP_c[1] = 0.0;
					if(cpZmpCurrentOn==1)
					{
						ZMP_c[0] = (tq_yr_p2_pos+(((tq_yl_p2_pos/f_zl_p2_pos) - pre_local_stride_F)*f_zl_p2_pos ))/(f_zr_p2_pos+f_zl_p2_pos);
						ZMP_c[1] = (tq_xr_p2_pos+(((tq_xl_p2_pos/f_zl_p2_pos) - pre_local_stride_L)*f_zl_p2_pos ))/(f_zr_p2_pos+f_zl_p2_pos);
						// printf("\ncpZmpCurrentOn : ZMP_c[0]=%lf,ZMP_c[1]=%lf_lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",ZMP_c[0],ZMP_c[1],_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);
					}
					if(_i<cptOn)
					{
						Kf_D[0]=0;
						Kf_D[1]=0;
					}
					else{
						Kf_D[0]=Kf_D_origin[0];
						Kf_D[1]=Kf_D_origin[1];	
					}
					// ZMP_d[0] = (CP_ref[0] - (exp(samplingtime / _Tc)*CP_cur[0])) / (1 - exp(samplingtime / _Tc));
					// ZMP_d[1] = (CP_ref[1] - (exp(samplingtime / _Tc)*CP_cur[1])) / (1 - exp(samplingtime / _Tc));
					// a_desired[0] = Kf_D[0] * ((_m*_g) / _zc) * (ZMP_c[0] - ZMP_d[0]);
					// a_desired[1] = Kf_D[1] * ((_m*_g) / _zc) * (ZMP_c[1] - ZMP_d[1]);
	
					ZMP_d[0] = ZMP_c[0] + (1 + Kf_S[0]*sqrt(_zc / _g))*(CP_cur[0] - CP_ref[0]);
					ZMP_d[1] = ZMP_c[1] + (1 + Kf_S[1]*sqrt(_zc / _g))*(CP_cur[1] - CP_ref[1]);
					a_desired[0] = (_m*_g / _zc) * ( - (1 + Kf_S[0]*sqrt(_zc / _g))*(CP_cur[0] - CP_ref[0]));
					a_desired[1] = (_m*_g / _zc) * ( - (1 + Kf_S[1]*sqrt(_zc / _g))*(CP_cur[1] - CP_ref[1]));
					
					if(Kf_D[0]==0 && Kf_D[1]==0)
					{
						a_desired[0]=0;
						a_desired[1]=0;
					}
					ZMP_global_d[0] = ZMP_d[0] + _px;
					ZMP_global_d[1] = ZMP_d[1] + _py;

					CPx_ddot=a_desired[0];
					CPy_ddot=a_desired[1];

					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*CPy_ddot*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + CPy_ddot * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*CPx_ddot*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+CPx_ddot*samplingtime;
					pre_CPx_ddot=CPx_ddot;
					pre_CPy_ddot=CPy_ddot;
					pre_ycom_c = _ycom_c;
					pre_xcom_c = _xcom_c;
					pre_ycom_cv = _ycom_cv;
					pre_xcom_cv = _xcom_cv;

					_xcom_CPT=_xcom-_px+_xcom_c;
					_ycom_CPT=_ycom-_py+_ycom_c;
					_xcom_CPT_global=_xcom_CPT+_px;
					_ycom_CPT_global=_ycom_CPT+_py;
					_xcom_CPT_v=_xcom_v+_xcom_cv;
					_ycom_CPT_v=_ycom_v+_ycom_cv;

					_dsp_x_des=_xcom_CPT_global;
					_dsp_y_des=_ycom_CPT_global;
					_xcom_nominal_global=_xcom_nominal;
					_ycom_nominal_global=_ycom_nominal;
					// printf("\n_ycom_c=%lf, _xcom_c=%lf, _ycom_cv=%lf, _xcom_cv=%lf",_ycom_c,_xcom_c,_ycom_cv,_xcom_cv);
					// printf("\n_ycom=%lf, _xcom=%lf, _ycom_nominal=%lf, _xcom_nominal=%lf, _dsp_x_des=%lf,_dsp_y_des=%lf",_ycom,_xcom,_ycom_nominal,_xcom_nominal,_dsp_x_des,_dsp_y_des);
#if debugPrintOn
					printf("---------------------------CP DSP CONTROL END---------------------------\n");
					printf("------------------------------------------------------------------------\n");
#endif				
				}
				pre_fslipy=fslipy;
				// printf("R_dsp_x_des = %lf R_dsp_y_des =%lf _x_des = %lf _xcom_v=%lf  _y_des=%lf _ycom_v=%lf\n", _dsp_x_des,_dsp_y_des,_x_des,_xcom_v,_y_des,_ycom_v);
				_targetxcom = _dsp_x_des;
				_targetycom = _dsp_y_des;
				if (_i == 3) {

					_lsx_des = 0;
					_lsy_des = 0;
					_lsz_des = 0;

					_dd_lsx_des = 0;
					_dd_lsy_des = 0;
					_dd_lsz_des = 0;


				}
				else {

					_lsx_des = _ppx;
					_lsy_des = _ppy;
					_lsz_des = 0;

					_dd_lsx_des = 0;
					_dd_lsy_des = 0;
					_dd_lsz_des = 0;

				}
#if debugPrintOn
				printf("\n-----------------------------RIGHT foot DSP END-------------------------");
				printf("\n------------------------------------------------------------------------");
#endif			
			}
		}

		else {     // 왼발 지지, 오른발 스윙(첫 스윙)
			if ((_t - _s[_i - 1][5]) > _s[_i][3] + samplingtime / 2) {
                // printf("\n-------------------------left foot supporting---------------------------");
				// printf("\n------------------------------------------------------------------------");
				localtime= _t - (_s[_i - 1][5] + _s[_i][3]);
				Tr=Tr_walk_ssp;
				_kv=2*(1.8/Tr);
				_kp=(1.8/Tr)*(1.8/Tr);
				footLYAccLpf=0, footRYAccLpf=0;
				DSPDelCheck=0;
				if(fslipInitCheck==0 && sspStartInit==1)
				{
					pre_xcom_c=0, pre_ycom_c=0;
					pre_xcom_cv=0, pre_ycom_cv=0;
					fslipInitCheck=1;
				}
				phase = SSP_L;
				if (phase == SSP_L)
				{
					new_phase_num = 3;
				}
				// FsrSensing(_i, new_phase_num, _px, _py, _ppx, _ppy);		

				_baserot = _baserot + _baserot_time;

				_rsr = _rsr + _swingrot_time;
				_theta = _theta + _swingtime;
				
				_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
				_ycom = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
				_xcom_origin=_xcom;
				_ycom_origin=_ycom;
				_xcom_v = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_ycom_v = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_xcom_acc = (-_x_ter / pow(_Tc, 2)) * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + (_vx_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_ycom_acc = (_y_ter / pow(_Tc, 2)) * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - (_vy_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_x_des = _px + _xcom;
                _y_des = _py + _ycom;
                _rsx_des = (cos(rot_th) * _s[_i - 1][0] - sin(rot_th) * pow(-1.0, _i - 1.0) * _s[_i - 1][1] + cos(rot_th + _s[_i - 1][4]) * _s[_i][0] - sin(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) / (2 * pi) * (_theta - sin(_theta)) + _ppx;
				_rsy_des = (_py + (sin(rot_th + _s[_i - 1][4]) * _s[_i][0] + cos(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) - _ppy) / (2 * pi) * (_theta - sin(_theta)) + _ppy;
				_rsz_des = _stepheight / 2 * (1 - cos(_theta));
				_dd_rsx_des = (cos(rot_th) * _s[_i - 1][0] - sin(rot_th) * pow(-1.0, _i - 1.0) * _s[_i - 1][1] + cos(rot_th + _s[_i - 1][4]) * _s[_i][0] - sin(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) / (2 * pi)* pow(_dds,2) * (sin(_theta));
				_dd_rsy_des = (_py + (sin(rot_th + _s[_i - 1][4]) * _s[_i][0] + cos(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) - _ppy) / (2 * pi)* pow(_dds,2) * (sin(_theta));
				_dd_rsz_des = _stepheight / 2 * (cos(_theta))*pow(_dds,2);
				// printf("\n^^^^^^^^^^^^^^^^M*_ycom_acc = %lf\tforceXr = %lf\t fslipy = %lf\t", (M * _g / _zc * (pre_ycom - ftzmpy)), forceYl, -(M * _g / _zc * (pre_ycom - ftzmpy)) + forceYl);
				pre_ycom = cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc));
				
				if(CpSwitch==2){
					ZMP_c[0] = 0.0;
					ZMP_c[1] = 0.0;
					if(cpZmpCurrentOn==1)
					{
						ZMP_c[0] = tq_yr_p1_pos/f_zr_p1_pos;
						ZMP_c[1] = tq_xr_p1_pos/f_zr_p1_pos;
						// printf("\ncpZmpCurrentOn : ZMP_c[0]=%lf,ZMP_c[1]=%lf_lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",ZMP_c[0],ZMP_c[1],_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);
					}
					//Pc 계산
					//forwardkinematics_FTB(j_th_encoder);
			        //forwardkinematics_LTR(j_th_encoder);
					inversekinematicsLtoB(j_th_encoder);
					inv_jac_L = saveMatrix_66(i_jacobi);
					//inv_jac_L = changeMatrix_66(inv_jac_L_1);
					forwardkinematics_FTB(_th_encoder);
					inv_th_L.x[0][0] = iMdth.x[11][0];
					inv_th_L.x[1][0] = iMdth.x[10][0];
					inv_th_L.x[2][0] = iMdth.x[9][0];
					inv_th_L.x[3][0] = iMdth.x[8][0];
					inv_th_L.x[4][0] = iMdth.x[7][0];
					inv_th_L.x[5][0] = iMdth.x[6][0];

					inv_pos.x[0][0] = ITLBE[0][3] + _px;
					inv_pos.x[1][0] = ITLBE[1][3] + _py;
					inv_pos.x[2][0] = ITLBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_L, inv_th_L);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_L, inv_th_L);
                   
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}
					else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					} 
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif

					_xcom_nominal = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_c[0]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_ycom_nominal = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_c[1]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));

					_vxcom_nominal = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + ZMP_c[0]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_vycom_nominal = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)  + ZMP_c[1]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					DCM_ver3(0,Kf_S[0],_xcom_nominal,_vxcom_nominal,inv_pos.x[0][0],inv_vel.x[0][0],_px,_i,localtime);
					DCM_ver3(1,Kf_S[1],_ycom_nominal,_vycom_nominal,inv_pos.x[1][0],inv_vel.x[1][0],_py,_i,localtime);


#if protectZmpEscapeOn
					protectZmpEscape(new_phase_num);
#endif
					
					//pd 계산
					_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_d[0]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_ycom = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + ZMP_d[1]*(1 - cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) ;
					_vxcom = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + ZMP_d[0]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					_vycom = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)  + ZMP_d[1]*(- (1 / _Tc)*sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
					//update된 com 궤적
					_x_des = _px + _xcom; 
					_y_des = _py + _ycom;
					// printf("\norigin com x=%lf y=%lf control com x=%lf y=%lf",_xcom_origin,_ycom_origin,_xcom,_ycom);
					//target com 궤적에 xcom ycom 대입
					dsp_y_i_c = _ycom+_py;
					dsp_x_i_c = _xcom+_px;
					dsp_dy_i_c = _vycom;
					dsp_dx_i_c = _vxcom;
					//dsp 시작 위치를 옮겨주기 위한 컨트롤값 저장
					_xcom_nominal_global=_xcom_nominal+_px;
					_ycom_nominal_global=_ycom_nominal+_py;
					CP_cur_global_ssp[0] = CP_cur[0] + _px;
					CP_cur_global_ssp[1] = CP_cur[1] + _py;
					CP_cur_global_dsp[0] = 0;//CP_E[0] + _px;
					CP_cur_global_dsp[1] = 0;//CP_E[1] + _py;
					CP_ref_global_ssp[0] = CP_ref[0] + _px;
					CP_ref_global_ssp[1] = CP_ref[1] + _py;
					CP_ref_global_dsp[0] = 0;//CP_ref[0] + _px;
					CP_ref_global_dsp[1] = 0;//CP_ref[1] + _py;
					//matlab plot 을 위한 과정
				}
				// fslipy = fslipKy*(-(M * _g / _zc * (pre_ycom+pre_ycom - ftzmpy)) + forceYl);
				// fslipy = fslipKy*(-(M * _g / _zc * (pre_ycom - ftzmpy)) + forceYl);
				
				// fslipy = fslipKy*(-M * _ycom_acc_measure+forceYl);
				// frefY_print = M * _g / _zc * (pre_ycom+pre_ycom_c - ftzmpy);
				// frefY_print = M * _g / _zc * (pre_ycom - ftzmpy);
				if(fslipAddFoot==1)
				{
					fslipy = fslipKy * (-M * ( _ycom_acc_measure - footLY_acc)+forceYl);
					fslipx = fslipKx * (-M * ( _xcom_acc_measure - footLX_acc)+forceXl);
					frefY_print = M * ( _ycom_acc_measure-footLY_acc);
					frefX_print = M * ( _xcom_acc_measure-footLX_acc);
				}
				else
				{
					fslipy = fslipKy*(-M * ( _ycom_acc_measure)+forceYl); 
					fslipx = fslipKx*(-M*(_xcom_acc_measure)+forceXl);
					frefY_print = M * ( _ycom_acc_measure);
					frefX_print = M* ( _xcom_acc_measure);
				}
				fslipy_print=fslipy/fslipKy; 
				fslipx_print=fslipx/fslipKx;
				// printf("\nfrefY_print=%lf,_ycom_acc_measure= %lf footLY_acc=%lf \n",frefY_print,_ycom_acc_measure,footLY_acc);
				if(controlswitchYL==0 || _i<sspYFslipControlOn){fslipy=0;}
                if(controlswitchXL==0 || _i<sspXFslipControlOn){fslipx=0;}
				
				if(fslipControlOn==1){
					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*fslipy*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + fslipy * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*fslipx*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+fslipx*samplingtime;
					

					pre_ycom_c = _ycom_c;
					pre_xcom_c=_xcom_c;
					pre_ycom_cv = _ycom_cv;
					pre_xcom_cv=_xcom_cv;

					_ycom_fslip = _ycom + _ycom_c;
					_xcom_fslip=_xcom+_xcom_c;
					_x_des = _px + _xcom_fslip;
					_y_des = _py + _ycom_fslip;

					// printf("\tfslipy = %lf\t_ycom_v = %lf\t _ycom_cv = %lf \t_ycom = %lf\t_ycom_fslip = %lf _py = %lf \n",fslipy, _ycom_v,_ycom_cv, _ycom, _ycom_fslip, _py);
					// printf("\tfslipx = %lf\t_xcom_v = %lf\t _xcom_cv = %lf \t_xcom = %lf\t_xcom_fslip = %lf _px = %lf \n",fslipx, _xcom_v,_xcom_cv, _xcom, _xcom_fslip, _px);
					
					// dspYICGlobal=dsp_y_i_c+_py;
					// dspXICGlobal=dsp_x_i_c+_px;

					dsp_y_i_c = _ycom_fslip+_py;
					dsp_x_i_c= _xcom_fslip+_px;
					dsp_dy_i_c = _ycom_v+_ycom_cv;
					dsp_dx_i_c = _xcom_v+_xcom_cv;
					yComVFinalPrint=_ycom_v+_ycom_cv;
					// printf("\nDsp_dy_i_c=%lf\n",dsp_dy_i_c);
				}
				else if(CpSwitch==1){
					//forwardkinematics_FTB(j_th_encoder);
			        //forwardkinematics_LTR(j_th_encoder);
					inversekinematicsLtoB(j_th_encoder);
					inv_jac_L = saveMatrix_66(i_jacobi);
					//inv_jac_L = changeMatrix_66(inv_jac_L_1);
					forwardkinematics_FTB(_th_encoder);
					inv_th_L.x[0][0] = iMdth.x[11][0];
					inv_th_L.x[1][0] = iMdth.x[10][0];
					inv_th_L.x[2][0] = iMdth.x[9][0];
					inv_th_L.x[3][0] = iMdth.x[8][0];
					inv_th_L.x[4][0] = iMdth.x[7][0];
					inv_th_L.x[5][0] = iMdth.x[6][0];

					inv_pos.x[0][0] = ITLBE[0][3] + _px;
					inv_pos.x[1][0] = ITLBE[1][3] + _py;
					inv_pos.x[2][0] = ITLBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_L, inv_th_L);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_L, inv_th_L);

                   
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}

						else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					} 
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					CPT_ver2_ssp(new_phase_num,localtime,_i,Kf_S[0],Kf_S[1],rot_th,_x_ter,_vx_ter,_y_ter,_vy_ter,inv_pos.x[0][0],inv_vel.x[0][0],inv_pos.x[1][0],inv_vel.x[1][0],_px,_py);
					
					CP_cur_global_ssp[0] = CP_E[0] + _px;
					CP_cur_global_ssp[1] = CP_E[1] + _py;
					CP_cur_global_dsp[0] = 0;//CP_E[0] + _px;
					CP_cur_global_dsp[1] = 0;//CP_E[1] + _py;
					CP_ref_global_ssp[0] = CP_ref[0] + _px;
					CP_ref_global_ssp[1] = CP_ref[1] + _py;
					CP_ref_global_dsp[0] = 0;//CP_ref[0] + _px;
					CP_ref_global_dsp[1] = 0;//CP_ref[1] + _py;
					
					ZMP_global_d[0] = ZMP_d[0] + _px;
					ZMP_global_d[1] = ZMP_d[1] + _py;

					CPx_ddot=a_desired[0];
					CPy_ddot=a_desired[1];

					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*CPy_ddot*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + CPy_ddot * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*CPx_ddot*pow(samplingtime,2);
					// printf("pre_xcom_c=%lf pre_xcom_cv=%lf CPx_ddot=%lf \n",pre_xcom_c, pre_xcom_cv,CPx_ddot);
					_xcom_cv=pre_xcom_cv+CPx_ddot*samplingtime;
					pre_CPx_ddot=CPx_ddot;
					pre_CPy_ddot=CPy_ddot;
					pre_ycom_c = _ycom_c;
					pre_xcom_c=_xcom_c;
					pre_ycom_cv = _ycom_cv;
					pre_xcom_cv=_xcom_cv;

					_xcom_CPT=_xcom+_xcom_c;
					_ycom_CPT=_ycom+_ycom_c;
					// printf("_xcom_CPT=%lf _xcom=%lf _xcom_c=%lf \n",_xcom_CPT, _xcom,_xcom_c);
					_xcom_CPT_global=_xcom_CPT+_px;
					_ycom_CPT_global=_ycom_CPT+_py;
					_xcom_CPT_v=_xcom_v+_xcom_cv;
					_ycom_CPT_v=_ycom_v+_ycom_cv;
					
					_x_des = _px + _xcom_CPT;
					_y_des = _py + _ycom_CPT;

					dsp_y_i_c = _ycom_CPT_global;
					dsp_x_i_c = _xcom_CPT_global;
					dsp_dy_i_c = _ycom_CPT_v;
					dsp_dx_i_c = _xcom_CPT_v;
					_xcom_nominal_global=_xcom+_px;
					_ycom_nominal_global=_ycom+_py;
				}
				_targetxcom = _x_des;
				_targetycom = _y_des;
				// printf("_targetxcom=%lf _targetycom=%lf \n",_x_des, _y_des);
				
				pre_fslipy=fslipy;
				pre_fslipx=fslipx;
                // printf("\nleftfootsupport\ndsp_x_i_c = %lf dsp_y_i_c = %lf dsp_dx_i_c = %lf dsp_dy_i_c = %lf\n",dsp_x_i_c, -dsp_y_i_c, dsp_dx_i_c, dsp_dy_i_c);
			}

			else {
				// printf("--------------------------------left foot DSP---------------------------\n");
				// printf("------------------------------------------------------------------------\n");
				localtime = _t - _s[_i - 1][5] ;	
				Tr=Tr_walk_dsp;
				_kv=2*(1.8/Tr);
				_kp=(1.8/Tr)*(1.8/Tr);

				phase = DSP_L;
				fslipInitCheck=0;
				if (phase == DSP_L)
					new_phase_num = 4;
				// FsrSensing(_i, new_phase_num, _px, _py, _ppx, _ppy);		
				
				// printf("_px=%f _py=%f _ppx = %f _ppy = %f _i=%d \n ", _px, _py, _ppx, _ppy,_i);
				_dsp_x_des = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5] ) + dsp_x_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 3);
				_dsp_y_des = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5] ) + dsp_y_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 3);
				_xcom = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5] ) + dsp_x_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 3);
				_ycom = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5] ) + dsp_y_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 3);
				_xcom_origin_global = dsp_x_a0_nominal + dsp_x_a1_nominal * (_t - _s[_i - 1][5] ) + dsp_x_a2_nominal * pow((_t - _s[_i - 1][5] ), 2) + dsp_x_a3_nominal * pow((_t - _s[_i - 1][5] ), 3);
				_ycom_origin_global = dsp_y_a0_nominal + dsp_y_a1_nominal * (_t - _s[_i - 1][5] ) + dsp_y_a2_nominal * pow((_t - _s[_i - 1][5] ), 2) + dsp_y_a3_nominal * pow((_t - _s[_i - 1][5] ), 3);
				_xcom_v = dsp_x_a1 + 2 * dsp_x_a2 *(_t - _s[_i - 1][5] ) + 3 * dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 2);
				_ycom_v = dsp_y_a1 + 2 * dsp_y_a2 * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 2);
				_xcom_v_nominal = dsp_x_a1_nominal + 2 * dsp_x_a2_nominal * (_t - _s[_i - 1][5] ) + 3 * dsp_x_a3_nominal * pow((_t - _s[_i - 1][5] ), 2);
				_ycom_v_nominal = dsp_y_a1_nominal + 2 * dsp_y_a2_nominal * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3_nominal * pow((_t - _s[_i - 1][5] ), 2);
				
				_xcom_acc = 2 * dsp_x_a2 + 6 * dsp_x_a3 * (_t - _s[_i - 1][5]);
				_ycom_acc = 2 * dsp_y_a2 + 6 * dsp_y_a3 * (_t - _s[_i - 1][5]);

				if(CpSwitch==2)
				{
					_xcom_nominal_global=_xcom;
					_ycom_nominal_global=_ycom;
					//global com traj
					forwardkinematics_FTB(j_th_encoder);
			        //forwardkinematics_LTR(j_th_encoder);
					inversekinematicsLtoB(j_th_encoder);
					inv_jac_L = saveMatrix_66(i_jacobi);
					//inv_jac_L = changeMatrix_66(inv_jac_L_1);
					// forwardkinematics_FTB(_th_encoder);
					inv_th_L.x[0][0] = iMdth.x[11][0];
					inv_th_L.x[1][0] = iMdth.x[10][0];
					inv_th_L.x[2][0] = iMdth.x[9][0];
					inv_th_L.x[3][0] = iMdth.x[8][0];
					inv_th_L.x[4][0] = iMdth.x[7][0];
					inv_th_L.x[5][0] = iMdth.x[6][0];

					inv_pos.x[0][0] = ITLBE[0][3] + _px;
					inv_pos.x[1][0] = ITLBE[1][3] + _py;
					inv_pos.x[2][0] = ITLBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_L, inv_th_L);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_L, inv_th_L);

                   
					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}

						else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					} 
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					//current com traj
				}

				if(fslipAddFoot==1)
				{
					fslipy = fslipKyDsp *(-M * ( _ycom_acc_measure-((footLY_acc+footRY_acc)/2))+forceYl+forceYr);
					fslipx = fslipKxDsp *(-M * ( _xcom_acc_measure-((footLX_acc+footRX_acc)/2))+forceXl+forceXr);
					frefY_print = M * ( _ycom_acc_measure-((footLY_acc+footRY_acc)/2));
					frefX_print = M * ( _xcom_acc_measure-((footLX_acc+footRX_acc)/2));
				}
				else
				{
					fslipy = fslipKyDsp*(-M * ( _ycom_acc_measure)+forceYl+forceYr); 
					fslipx = fslipKxDsp*(-M*(_xcom_acc_measure)+forceXl+forceXr);
					frefY_print = M * ( _ycom_acc_measure);
					frefX_print = M* ( _xcom_acc_measure);
				}
				if(DSPDelFslipNoiseOn==1 && DSPDelCheck==0)
				{
					if(forceZr>0 && forceZl>0)
					{
						fslipy=0;
						frefY_print=0;
						DSPDelCheck=1;
					}
				}
                
				fslipy_print=fslipy/fslipKyDsp;
				fslipx_print=fslipx/fslipKxDsp;

				// printf("\nfslipy_print=%lf, frefY_print=%lf,_ycom_acc_measure= %lf footLY_acc=%lf,footRY_acc=%lf  (footLY_acc+footRY_acc)/2=%lf \n",fslipy_print,frefY_print,_ycom_acc_measure,footLY_acc,footRY_acc,(footLY_acc+footRY_acc)/2);
				// printf("\n_ycom_v=%lf\n",_ycom_v);
				
				if(fslipControlOn==1)
				{
					fslipx=0;
					
					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*fslipy*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + fslipy * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*fslipx*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+fslipx*samplingtime;
                    if(dspFSlipControl==0 || _i<dspFslipControlOn)
					{
						_ycom_c=0;_ycom_cv=0;_xcom_c=0;_xcom_cv=0;
						// printf("!!!!!!!!!!!!!!!!!dspFslipOFF!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
					}

					pre_ycom_c = _ycom_c;
					pre_xcom_c=_xcom_c;
					pre_ycom_cv =_ycom_cv;
					pre_xcom_cv=_xcom_cv;

					_ycom_fslip = _ycom + _ycom_c;
					_xcom_fslip=_xcom+_xcom_c;
					// printf("_ycom_c=%lf _xcom_c=%lf _ycom_cv=%lf_xcom_cv= %lf",_ycom_c,_xcom_c ,_ycom_cv,_xcom_cv);
					_dsp_x_des = _xcom_fslip;
					_dsp_y_des = _ycom_fslip;
				}
				yComVFinalPrint=_ycom_v+_ycom_cv;
				if(CpSwitch==1 && cpDspSwitch==2)
				{
					CP_ref[0] = _xcom-_px+_Tc*_xcom_v;
					CP_ref[1] = _ycom-_py+_Tc*_ycom_v; 
					if(CpSwitch==1 && DspInitSwitch==1){
						CP_ref[0] = _xcom_nominal-_px+_Tc*_xcom_v_nominal;
						CP_ref[1] =  _ycom_nominal-_py+_Tc*_ycom_v_nominal;
					}
					CP_ref_global_ssp[0] = 0;//CP_ref[0] + _px;
					CP_ref_global_ssp[1] = 0;//CP_ref[1] + _py;
					CP_ref_global_dsp[0] = CP_ref[0] + _px;
					CP_ref_global_dsp[1] = CP_ref[1] + _py;

					forwardkinematics_FTB(j_th_encoder);
			        //forwardkinematics_LTR(j_th_encoder);
					inversekinematicsLtoB(j_th_encoder);
					inv_jac_L = saveMatrix_66(i_jacobi);
					//inv_jac_L = changeMatrix_66(inv_jac_L_1);
					// forwardkinematics_FTB(_th_encoder);
					inv_th_L.x[0][0] = iMdth.x[11][0];
					inv_th_L.x[1][0] = iMdth.x[10][0];
					inv_th_L.x[2][0] = iMdth.x[9][0];
					inv_th_L.x[3][0] = iMdth.x[8][0];
					inv_th_L.x[4][0] = iMdth.x[7][0];
					inv_th_L.x[5][0] = iMdth.x[6][0];

					inv_pos.x[0][0] = ITLBE[0][3] + _px;
					inv_pos.x[1][0] = ITLBE[1][3] + _py;
					inv_pos.x[2][0] = ITLBE[2][3];

					inv_vel = productmatrix66_61(inv_jac_L, inv_th_L);
					pre_inv_vel_1 = productmatrix66_61(inv_jac_L, inv_th_L);

					pre_inv_vel[0]=pre_inv_vel_1.x[0][0];
					pre_inv_vel[1]=pre_inv_vel_1.x[1][0];

					pre_inv_F_vel[0]=inv_F_vel[0];
					pre_inv_F_vel[1]=inv_F_vel[1];

					if(_sn==0){
					inv_F_vel[0]=pre_inv_vel_1.x[0][0];
					inv_F_vel[1]=pre_inv_vel_1.x[1][0];
					}

						else {   
					LPF(10,pre_inv_vel,pre_inv_F_vel,inv_F_vel);   
					} 
#if (cpVelFilterOn==1)
						inv_vel.x[0][0] = inv_F_vel[0];
                   		inv_vel.x[1][0] = inv_F_vel[1];
#endif
					CP_cur[0] = (inv_pos.x[0][0] - _px) + _Tc * (inv_vel.x[0][0]);
					CP_cur[1] = (inv_pos.x[1][0] - _py) + _Tc * (inv_vel.x[1][0]);
					CP_cur_global_ssp[0] =0;// CP_cur[0] + _px;
					CP_cur_global_ssp[1] =0;// CP_cur[1] + _py;
					CP_cur_global_dsp[0] = CP_cur[0] + _px;
					CP_cur_global_dsp[1] = CP_cur[1] + _py;
					
					ZMP_c[0] = 0.0;
					ZMP_c[1] = 0.0;
					if(cpZmpCurrentOn==1)
					{
						ZMP_c[0] = ((((tq_yr_p3_pos/f_zr_p3_pos) - pre_local_stride_F)*f_zr_p3_pos)+tq_yl_p3_pos)/(f_zr_p3_pos+f_zl_p3_pos);
						ZMP_c[1] = ((((tq_xr_p3_pos/f_zr_p3_pos) - pre_local_stride_L)*f_zr_p3_pos)+tq_xl_p3_pos)/(f_zr_p3_pos+f_zl_p3_pos);
						// printf("\ncpZmpCurrentOn : ZMP_c[0]=%lf,ZMP_c[1]=%lf_lsx_des=%lf, _lsy_des=%lf pre_local_stride_F=%lf pre_local_stride_L=%lf\n",ZMP_c[0],ZMP_c[1],_lsx_des,_lsy_des,pre_local_stride_F,pre_local_stride_L);
					}
					if(_i<cptOn)
					{
						Kf_D[0]=0;
						Kf_D[1]=0;
					}
					else{
						Kf_D[0]=Kf_D_origin[0];
						Kf_D[1]=Kf_D_origin[1];	
					}
					// ZMP_d[0] = (CP_ref[0] - (exp(samplingtime / _Tc)*CP_cur[0])) / (1 - exp(samplingtime / _Tc));
					// ZMP_d[1] = (CP_ref[1] - (exp(samplingtime / _Tc)*CP_cur[1])) / (1 - exp(samplingtime / _Tc));
					// a_desired[0] = Kf_D[0] * ((_m*_g) / _zc) * (ZMP_c[0] - ZMP_d[0]);
					// a_desired[1] = Kf_D[1] * ((_m*_g) / _zc) * (ZMP_c[1] - ZMP_d[1]);

					ZMP_d[0] = ZMP_c[0] + (1 + Kf_S[0]*sqrt(_zc / _g))*(CP_cur[0] - CP_ref[0]);
					ZMP_d[1] = ZMP_c[1] + (1 + Kf_S[1]*sqrt(_zc / _g))*(CP_cur[1] - CP_ref[1]);
					a_desired[0] = (_m*_g / _zc) * ( - (1 + Kf_S[0]*sqrt(_zc / _g))*(CP_cur[0] - CP_ref[0]));
					a_desired[1] = (_m*_g / _zc) * ( - (1 + Kf_S[1]*sqrt(_zc / _g))*(CP_cur[1] - CP_ref[1]));

					if(Kf_D[0]==0 && Kf_D[1]==0)
					{
						a_desired[0]=0;
						a_desired[1]=0;
					}
					ZMP_global_d[0] = ZMP_d[0] + _px;
					ZMP_global_d[1] = ZMP_d[1] + _py;

					CPx_ddot=a_desired[0];
					CPy_ddot=a_desired[1];

					_ycom_c = pre_ycom_c + pre_ycom_cv * samplingtime + 0.5*CPy_ddot*pow(samplingtime, 2);
					_ycom_cv = pre_ycom_cv + CPy_ddot * samplingtime;
					_xcom_c= pre_xcom_c+pre_xcom_cv*samplingtime+ 0.5*CPx_ddot*pow(samplingtime,2);
					_xcom_cv=pre_xcom_cv+CPx_ddot*samplingtime;
					pre_CPx_ddot=CPx_ddot;
					pre_CPy_ddot=CPy_ddot;
					pre_ycom_c = _ycom_c;
					pre_xcom_c=_xcom_c;
					pre_ycom_cv = _ycom_cv;
					pre_xcom_cv=_xcom_cv;
					
					_xcom_CPT=_xcom-_px+_xcom_c;
					_ycom_CPT=_ycom-_py+_ycom_c;
					_xcom_CPT_global=_xcom_CPT+_px;
					_ycom_CPT_global=_ycom_CPT+_py;
					_xcom_CPT_v=_xcom_v+_xcom_cv;
					_ycom_CPT_v=_ycom_v+_ycom_cv;
					_xcom_nominal_global=_xcom_nominal;
					_ycom_nominal_global=_ycom_nominal;
					_dsp_x_des=_xcom_CPT_global;
					_dsp_y_des=_ycom_CPT_global;
					// printf("\n_ycom_c=%lf, _xcom_c=%lf, _ycom_cv=%lf, _xcom_cv=%lf",_ycom_c,_xcom_c,_ycom_cv,_xcom_cv);
					// printf("\n_ycom=%lf, _xcom=%lf, _ycom_nominal=%lf, _xcom_nominal=%lf, _dsp_x_des=%lf,_dsp_y_des=%lf",_ycom,_xcom,_ycom_nominal,_xcom_nominal,_dsp_x_des,_dsp_y_des);
					// printf("------------------------------------------------------------------------\n");
					// printf("---------------------------CP DSP CONTROL END---------------------------\n");
					// printf("------------------------------------------------------------------------\n");
				}
				pre_fslipy=fslipy;
				
				_targetxcom = _dsp_x_des;
				_targetycom = _dsp_y_des;
				if (_i == 3) {
					_rsx_des = 0;
					_rsy_des = 0;
					_rsz_des = 0;
		
					_dd_rsx_des = 0;
					_dd_rsy_des = 0;
					_dd_rsz_des = 0;

				}
				else {
					_rsx_des = _ppx;
					_rsy_des = _ppy;
					_rsz_des = 0;
		
					_dd_rsx_des = 0;
					_dd_rsy_des = 0;
					_dd_rsz_des = 0;
				}
				
				// printf("\n------------------------------------------------------------------------\n");
				// printf("------------------------------left foot DSP END-------------------------\n");
				// printf("------------------------------------------------------------------------\n");
			}
		}

		btargetr[0][0] = cos(_baserot);
		btargetr[0][1] = -sin(_baserot);
		btargetr[0][2] = 0.0;
		btargetr[1][0] = sin(_baserot);
		btargetr[1][1] = cos(_baserot);
		btargetr[1][2] = 0.0;
		btargetr[2][0] = 0.0;
		btargetr[2][1] = 0.0;
		btargetr[2][2] = 1.0;

		TWB[0][0] = btargetr[0][0];
		TWB[0][1] = btargetr[0][1];
		TWB[0][2] = btargetr[0][2];
		TWB[1][0] = btargetr[1][0];
		TWB[1][1] = btargetr[1][1];
		TWB[1][2] = btargetr[1][2];
		TWB[2][0] = btargetr[2][0];
		TWB[2][1] = btargetr[2][1];
		TWB[2][2] = btargetr[2][2];
		TWB[0][3] = _targetxcom;
		TWB[1][3] = _targetycom;
		TWB[2][3] = _zc;

		ltargetr[0][0] = cos(_lsr);
		ltargetr[0][1] = -sin(_lsr);
		ltargetr[0][2] = 0.0;
		ltargetr[1][0] = sin(_lsr);
		ltargetr[1][1] = cos(_lsr);
		ltargetr[1][2] = 0.0;
		ltargetr[2][0] = 0.0;
		ltargetr[2][1] = 0.0;
		ltargetr[2][2] = 1.0;

		targetr[0][0] = ltargetr[0][0];
		targetr[0][1] = ltargetr[0][1];
		targetr[0][2] = ltargetr[0][2];
		targetr[1][0] = ltargetr[1][0];
		targetr[1][1] = ltargetr[1][1];
		targetr[1][2] = ltargetr[1][2];
		targetr[2][0] = ltargetr[2][0];
		targetr[2][1] = ltargetr[2][1];
		targetr[2][2] = ltargetr[2][2];

		targetp[0] = _lsx_des;
		targetp[1] = _lsy_des;
		targetp[2] = _lsz_des;
		// double _th[12] = {0.007324, 0.852565, -1.573681, 0.721117, 0.007352, 0.000000, -0.000000, -0.007352, -0.721255, 1.573706, -0.852451, 0.007352};
		// printf("\n_lsx_des= %lf _lsy_des = %lf ,_lsz_des=%lf, _lsr=%lf,_baserot=%lf,targetxcom = %lf targetycom = %lf",
		// _lsx_des,_lsy_des,_lsz_des,_lsr,_baserot,_targetxcom,_targetycom);
		
		

		
		/*ORIGIN 0630
		for (_j = 0; _j < 1000; _j++) {
			inversekinematicsBtoL(_th);
			forwardkinematics(_th);
			if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5] < 0.00000001)) {

				break;
			}
		}

		rtargetr[0][0] = cos(_rsr);
		rtargetr[0][1] = -sin(_rsr);
		rtargetr[0][2] = 0.0;
		rtargetr[1][0] = sin(_rsr);
		rtargetr[1][1] = cos(_rsr);
		rtargetr[1][2] = 0.0;
		rtargetr[2][0] = 0.0;
		rtargetr[2][1] = 0.0;
		rtargetr[2][2] = 1.0;

		targetr[0][0] = rtargetr[0][0];
		targetr[0][1] = rtargetr[0][1];
		targetr[0][2] = rtargetr[0][2];
		targetr[1][0] = rtargetr[1][0];
		targetr[1][1] = rtargetr[1][1];
		targetr[1][2] = rtargetr[1][2];
		targetr[2][0] = rtargetr[2][0];
		targetr[2][1] = rtargetr[2][1];
		targetr[2][2] = rtargetr[2][2];

		targetp[0] = _rsx_des;
		targetp[1] = _rsy_des;
		targetp[2] = _rsz_des;

		for (_j = 0; _j < 1000; _j++) {
			inversekinematicsBtoR(_th);
			forwardkinematics(_th);
			if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5] < 0.00000001)) {

				break;
			}
		}
		*/


		//add 0630
		
			//Matrix_12X1 _theta_p;
		 if (_sn==0){

			_q_LE_n.x[0][0]=0;
			_q_LE_n.x[1][0]=0;
			_q_LE_n.x[2][0]=_lsr;
		}

		else{
			_q_LE_p.x[0][0]=0;
			_q_LE_p.x[1][0]=0;
			_q_LE_p.x[2][0]=_q_LE_n.x[2][0];
	
			_q_LE_n.x[0][0]=0;
			_q_LE_n.x[1][0]=0;
			_q_LE_n.x[2][0]=_lsr;
		}

		_ddp_LE.x[0][0]=_dd_lsx_des;
		_ddp_LE.x[1][0]=_dd_lsy_des;
		_ddp_LE.x[2][0]=_dd_lsz_des;

		forwardkinematics(_th);
		
		_theta_p.x[0][0] = _th[0];
		_theta_p.x[1][0] = _th[1];
		_theta_p.x[2][0] = _th[2];
		_theta_p.x[3][0] = _th[3];
		_theta_p.x[4][0] = _th[4];
		_theta_p.x[5][0] = _th[5];
		_theta_p.x[6][0] = _th[6];
		_theta_p.x[7][0] = _th[7];
		_theta_p.x[8][0] = _th[8];
		_theta_p.x[9][0] = _th[9];
		_theta_p.x[10][0] = _th[10];
		_theta_p.x[11][0] = _th[11];
		
		RR01.x[0][0] = TRB1[0][0];
		RR01.x[0][1] = TRB1[0][1];
		RR01.x[0][2] = TRB1[0][2];

		RR01.x[1][0] = TRB1[1][0];
		RR01.x[1][1] = TRB1[1][1];
		RR01.x[1][2] = TRB1[1][2];

		RR01.x[2][0] = TRB1[2][0];
		RR01.x[2][1] = TRB1[2][1];
		RR01.x[2][2] = TRB1[2][2];

		RR12.x[0][0] = TR12[0][0];
		RR12.x[0][1] = TR12[0][1];
		RR12.x[0][2] = TR12[0][2];

		RR12.x[1][0] = TR12[1][0];
		RR12.x[1][1] = TR12[1][1];
		RR12.x[1][2] = TR12[1][2];

		RR12.x[2][0] = TR12[2][0];
		RR12.x[2][1] = TR12[2][1];
		RR12.x[2][2] = TR12[2][2];

		RR23.x[0][0] = TR23[0][0];
		RR23.x[0][1] = TR23[0][1];
		RR23.x[0][2] = TR23[0][2];

		RR23.x[1][0] = TR23[1][0];
		RR23.x[1][1] = TR23[1][1];
		RR23.x[1][2] = TR23[1][2];

		RR23.x[2][0] = TR23[2][0];
		RR23.x[2][1] = TR23[2][1];
		RR23.x[2][2] = TR23[2][2];

		RR34.x[0][0] = TR34[0][0];
		RR34.x[0][1] = TR34[0][1];
		RR34.x[0][2] = TR34[0][2];

		RR34.x[1][0] = TR34[1][0];
		RR34.x[1][1] = TR34[1][1];
		RR34.x[1][2] = TR34[1][2];

		RR34.x[2][0] = TR34[2][0];
		RR34.x[2][1] = TR34[2][1];
		RR34.x[2][2] = TR34[2][2];

		RR45.x[0][0] = TR45[0][0];
		RR45.x[0][1] = TR45[0][1];
		RR45.x[0][2] = TR45[0][2];

		RR45.x[1][0] = TR45[1][0];
		RR45.x[1][1] = TR45[1][1];
		RR45.x[1][2] = TR45[1][2];

		RR45.x[2][0] = TR45[2][0];
		RR45.x[2][1] = TR45[2][1];
		RR45.x[2][2] = TR45[2][2];


		RR56.x[0][0] = TR56[0][0];
		RR56.x[0][1] = TR56[0][1];
		RR56.x[0][2] = TR56[0][2];

		RR56.x[1][0] = TR56[1][0];
		RR56.x[1][1] = TR56[1][1];
		RR56.x[1][2] = TR56[1][2];

		RR56.x[2][0] = TR56[2][0];
		RR56.x[2][1] = TR56[2][1];
		RR56.x[2][2] = TR56[2][2];


		RR67.x[0][0] = TR6E[0][0];
		RR67.x[0][1] = TR6E[0][1];
		RR67.x[0][2] = TR6E[0][2];

		RR67.x[1][0] = TR6E[1][0];
		RR67.x[1][1] = TR6E[1][1];
		RR67.x[1][2] = TR6E[1][2];

		RR67.x[2][0] = TR6E[2][0];
		RR67.x[2][1] = TR6E[2][1];
		RR67.x[2][2] = TR6E[2][2];


		RL01.x[0][0] = TLB1[0][0];
		RL01.x[0][1] = TLB1[0][1];
		RL01.x[0][2] = TLB1[0][2];

		RL01.x[1][0] = TLB1[1][0];
		RL01.x[1][1] = TLB1[1][1];
		RL01.x[1][2] = TLB1[1][2];

		RL01.x[2][0] = TLB1[2][0];
		RL01.x[2][1] = TLB1[2][1];
		RL01.x[2][2] = TLB1[2][2];

		RL12.x[0][0] = TL12[0][0];
		RL12.x[0][1] = TL12[0][1];
		RL12.x[0][2] = TL12[0][2];

		RL12.x[1][0] = TL12[1][0];
		RL12.x[1][1] = TL12[1][1];
		RL12.x[1][2] = TL12[1][2];

		RL12.x[2][0] = TL12[2][0];
		RL12.x[2][1] = TL12[2][1];
		RL12.x[2][2] = TL12[2][2];

		RL23.x[0][0] = TL23[0][0];
		RL23.x[0][1] = TL23[0][1];
		RL23.x[0][2] = TL23[0][2];

		RL23.x[1][0] = TL23[1][0];
		RL23.x[1][1] = TL23[1][1];
		RL23.x[1][2] = TL23[1][2];

		RL23.x[2][0] = TL23[2][0];
		RL23.x[2][1] = TL23[2][1];
		RL23.x[2][2] = TL23[2][2];

		RL34.x[0][0] = TL34[0][0];
		RL34.x[0][1] = TL34[0][1];
		RL34.x[0][2] = TL34[0][2];

		RL34.x[1][0] = TL34[1][0];
		RL34.x[1][1] = TL34[1][1];
		RL34.x[1][2] = TL34[1][2];

		RL34.x[2][0] = TL34[2][0];
		RL34.x[2][1] = TL34[2][1];
		RL34.x[2][2] = TL34[2][2];

		RL45.x[0][0] = TL45[0][0];
		RL45.x[0][1] = TL45[0][1];
		RL45.x[0][2] = TL45[0][2];

		RL45.x[1][0] = TL45[1][0];
		RL45.x[1][1] = TL45[1][1];
		RL45.x[1][2] = TL45[1][2];

		RL45.x[2][0] = TL45[2][0];
		RL45.x[2][1] = TL45[2][1];
		RL45.x[2][2] = TL45[2][2];


		RL56.x[0][0] = TL56[0][0];
		RL56.x[0][1] = TL56[0][1];
		RL56.x[0][2] = TL56[0][2];

		RL56.x[1][0] = TL56[1][0];
		RL56.x[1][1] = TL56[1][1];
		RL56.x[1][2] = TL56[1][2];

		RL56.x[2][0] = TL56[2][0];
		RL56.x[2][1] = TL56[2][1];
		RL56.x[2][2] = TL56[2][2];


		RL67.x[0][0] = TL6E[0][0];
		RL67.x[0][1] = TL6E[0][1];
		RL67.x[0][2] = TL6E[0][2];

		RL67.x[1][0] = TL6E[1][0];
		RL67.x[1][1] = TL6E[1][1];
		RL67.x[1][2] = TL6E[1][2];

		RL67.x[2][0] = TL6E[2][0];
		RL67.x[2][1] = TL6E[2][1];
		RL67.x[2][2] = TL6E[2][2];

		//Matrix_M33X7 RR, RL;

		RR.x[0][0] = RR01;
		RR.x[1][0] = RR12;
		RR.x[2][0] = RR23;
		RR.x[3][0] = RR34;
		RR.x[4][0] = RR45;
		RR.x[5][0] = RR56;
		RR.x[6][0] = RR67;

		RL.x[0][0] = RL01;
		RL.x[1][0] = RL12;
		RL.x[2][0] = RL23;
		RL.x[3][0] = RL34;
		RL.x[4][0] = RL45;
		RL.x[5][0] = RL56;
		RL.x[6][0] = RL67;



		//Matrix_3X3 RR10, RR21, RR32, RR43, RR54, RR65, RR76;
		//Matrix_3X3 RL10, RL21, RL32, RL43, RL54, RL65, RL76;

		RR10 = transposeMatrix_33(RR01);
		RR21 = transposeMatrix_33(RR12);
		RR32 = transposeMatrix_33(RR23);
		RR43 = transposeMatrix_33(RR34);
		RR54 = transposeMatrix_33(RR45);
		RR65 = transposeMatrix_33(RR56);
		RR76 = transposeMatrix_33(RR67);

		RL10 = transposeMatrix_33(RL01);
		RL21 = transposeMatrix_33(RL12);
		RL32 = transposeMatrix_33(RL23);
		RL43 = transposeMatrix_33(RL34);
		RL54 = transposeMatrix_33(RL45);
		RL65 = transposeMatrix_33(RL56);
		RL76 = transposeMatrix_33(RL67);

		//Matrix_M33X7 invRR, invRL;

		invRR.x[0][0] = RR10;
		invRR.x[1][0] = RR21;
		invRR.x[2][0] = RR32;
		invRR.x[3][0] = RR43;
		invRR.x[4][0] = RR54;
		invRR.x[5][0] = RR65;
		invRR.x[6][0] = RR76;

		invRL.x[0][0] = RL10;
		invRL.x[1][0] = RL21;
		invRL.x[2][0] = RL32;
		invRL.x[3][0] = RL43;
		invRL.x[4][0] = RL54;
		invRL.x[5][0] = RL65;
		invRL.x[6][0] = RL76;

		PR01.x[0][0] = TRB1[0][3];
		PR01.x[1][0] = TRB1[1][3];
		PR01.x[2][0] = TRB1[2][3];

		PR12.x[0][0] = TR12[0][3];
		PR12.x[1][0] = TR12[1][3];
		PR12.x[2][0] = TR12[2][3];

		PR23.x[0][0] = TR23[0][3];
		PR23.x[1][0] = TR23[1][3];
		PR23.x[2][0] = TR23[2][3];

		PR34.x[0][0] = TR34[0][3];
		PR34.x[1][0] = TR34[1][3];
		PR34.x[2][0] = TR34[2][3];

		PR45.x[0][0] = TR45[0][3];
		PR45.x[1][0] = TR45[1][3];
		PR45.x[2][0] = TR45[2][3];

		PR56.x[0][0] = TR56[0][3];
		PR56.x[1][0] = TR56[1][3];
		PR56.x[2][0] = TR56[2][3];

		PR67.x[0][0] = TR6E[0][3];
		PR67.x[1][0] = TR6E[1][3];
		PR67.x[2][0] = TR6E[2][3];


		PL01.x[0][0] = TLB1[0][3];
		PL01.x[1][0] = TLB1[1][3];
		PL01.x[2][0] = TLB1[2][3];

		PL12.x[0][0] = TL12[0][3];
		PL12.x[1][0] = TL12[1][3];
		PL12.x[2][0] = TL12[2][3];

		PL23.x[0][0] = TL23[0][3];
		PL23.x[1][0] = TL23[1][3];
		PL23.x[2][0] = TL23[2][3];

		PL34.x[0][0] = TL34[0][3];
		PL34.x[1][0] = TL34[1][3];
		PL34.x[2][0] = TL34[2][3];

		PL45.x[0][0] = TL45[0][3];
		PL45.x[1][0] = TL45[1][3];
		PL45.x[2][0] = TL45[2][3];

		PL56.x[0][0] = TL56[0][3];
		PL56.x[1][0] = TL56[1][3];
		PL56.x[2][0] = TL56[2][3];

		PL67.x[0][0] = TL6E[0][3];
		PL67.x[1][0] = TL6E[1][3];
		PL67.x[2][0] = TL6E[2][3];

		//Matrix_M31X7 PositionppR;

		PositionppR.x[0][0] = PR01;
		PositionppR.x[0][1] = PR12;
		PositionppR.x[0][2] = PR23;
		PositionppR.x[0][3] = PR34;
		PositionppR.x[0][4] = PR45;
		PositionppR.x[0][5] = PR56;
		PositionppR.x[0][6] = PR67;

		//Matrix_M31X7 PositionppL;

		PositionppL.x[0][0] = PL01;
		PositionppL.x[0][1] = PL12;
		PositionppL.x[0][2] = PL23;
		PositionppL.x[0][3] = PL34;
		PositionppL.x[0][4] = PL45;
		PositionppL.x[0][5] = PL56;
		PositionppL.x[0][6] = PL67;
		
    	// std::chrono::system_clock::time_point StartTime_IKL = std::chrono::system_clock::now();


		if(inverse_type==0)
		{
			
			for (_j = 0; _j < 1000; _j++) {//inverse B->L
				inversekinematicsBtoL(_th);
				forwardkinematics(_th);
				//forwardkinematics(_th, TR6E, TR56, TR45, TR34, TR23, TR12, TRB1, TL6E, TL56, TL45, TL34, TL23, TL12, TLB1);
				if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5]) < 0.00000001) {
					//origin condition  < 0.00000001
					break;
				}
			}
		}
		else if(inverse_type==1)
		{
			inversekinematicsBtoL(_th);
			forwardkinematics(_th);
			// printf("error sum of sqrt =%lf",sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5]));
		}
		// std::chrono::system_clock::time_point EndTime_IKL = std::chrono::system_clock::now();
		// std::chrono::duration<double> sec = EndTime_IKL - StartTime_IKL;
		// std::cout << "Test() function operation time (sec) : " << sec.count() << " seconds" << std::endl;
		// std::cout << "Test() function operation time (ms) : " << sec.count()*(10^3) << " ms" << std::endl;

		
		_jacobi_L_p =_jacobi_L_n;

		_jacobi_L_n = saveMatrix_66(_jacobi);

		_n_L = cin_function(_a_L, _jacobi);   // read function
									//cout_function(a, n, print_matrix);   // read function
		_deter_L = (double)det(_a_L, _n_L);   // read function
		inverse(_a_L, _d_L, _n_L, _deter_L);   // read function

		_invjacobi_L_p = saveMatrix_66(_d_L); //inverse matrix(J^-1) save for ddq

		rtargetr[0][0] = cos(_rsr);
		rtargetr[0][1] = -sin(_rsr);
		rtargetr[0][2] = 0.0;
		rtargetr[1][0] = sin(_rsr);
		rtargetr[1][1] = cos(_rsr);
		rtargetr[1][2] = 0.0;
		rtargetr[2][0] = 0.0;
		rtargetr[2][1] = 0.0;
		rtargetr[2][2] = 1.0;

		targetr[0][0] = rtargetr[0][0];
		targetr[0][1] = rtargetr[0][1];
		targetr[0][2] = rtargetr[0][2];
		targetr[1][0] = rtargetr[1][0];
		targetr[1][1] = rtargetr[1][1];
		targetr[1][2] = rtargetr[1][2];
		targetr[2][0] = rtargetr[2][0];
		targetr[2][1] = rtargetr[2][1];
		targetr[2][2] = rtargetr[2][2];

		targetp[0] = _rsx_des;
		targetp[1] = _rsy_des;
		targetp[2] = _rsz_des;
			
		if (_sn==0){
			_q_RE_n.x[0][0]=0;
			_q_RE_n.x[1][0]=0;
			_q_RE_n.x[2][0]=_rsr;
		}

		else{
			_q_RE_p.x[0][0]=0;
			_q_RE_p.x[1][0]=0;
			_q_RE_p.x[2][0]=_q_RE_n.x[2][0];
			_q_RE_n.x[0][0]=0;
			_q_RE_n.x[1][0]=0;
			_q_RE_n.x[2][0]=_rsr;
		}

		Matrix_3X1 _ddp_RE;

		_ddp_RE.x[0][0]=_dd_rsx_des;
		_ddp_RE.x[1][0]=_dd_rsy_des;
		_ddp_RE.x[2][0]=_dd_rsz_des;

		//inverse 
		if(inverse_type==0)
		{
			for (_j = 0; _j < 1000; _j++) {//inverse B->R
				inversekinematicsBtoR(_th);
				forwardkinematics(_th);
				//forwardkinematics(_th, TR6E, TR56, TR45, TR34, TR23, TR12, TRB1, TL6E, TL56, TL45, TL34, TL23, TL12, TLB1);
				if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5]) < 0.00000001) {
					break;
				}
			}
		}
		else if(inverse_type==1)
		{
			inversekinematicsBtoR(_th);
			forwardkinematics(_th);
		}

		_jacobi_R_p =_jacobi_R_n ;

		_jacobi_R_n_1 = saveMatrix_66(_jacobi);
		_jacobi_R_n = changeMatrix_66(_jacobi_R_n_1); // 1->6 TO 6->1

			//_jacobi_n=savedouble_66(_jacobi);
		savedouble_66(_jacobi,_jacobi_n);
		_n_R = cin_function(_a_R, _jacobi_n);   // read function		
									//cout_function(a, n, print_matrix);   // read function
		_deter_R = (double)det(_a_R, _n_R);   // read function
		inverse(_a_R, _d_R, _n_R, _deter_R);   // read function
		_invjacobi_R_p = saveMatrix_66(_d_R); //inverse matrix 

			//_invjacobi_R_p = changeMatrix_66(_invjacobi_R_p_1); // 1->6 TO 6->1 ,inverse matrix save for ddq

		_theta_n.x[0][0] = _th[0];
		_theta_n.x[1][0] = _th[1];
		_theta_n.x[2][0] = _th[2];
		_theta_n.x[3][0] = _th[3];
		_theta_n.x[4][0] = _th[4];
		_theta_n.x[5][0] = _th[5];
		_theta_n.x[6][0] = _th[6];
		_theta_n.x[7][0] = _th[7];
		_theta_n.x[8][0] = _th[8];
		_theta_n.x[9][0] = _th[9];
		_theta_n.x[10][0] = _th[10];
		_theta_n.x[11][0] = _th[11];

		
		if (_sn==0){


			_dth_p.x[0][0]=0;
			_dth_p.x[1][0]=0;
			_dth_p.x[2][0]=0;
			_dth_p.x[3][0]=0;
			_dth_p.x[4][0]=0;
			_dth_p.x[5][0]=0;
			_dth_p.x[6][0]=0;
			_dth_p.x[7][0]=0;
			_dth_p.x[8][0]=0;
			_dth_p.x[9][0]=0;
			_dth_p.x[10][0]=0;
			_dth_p.x[11][0]=0;

			_dth_n = s_vectorminus121_121(_theta_n,_theta_p);



			ddq_Jdot_L_p.x[0][0] = 0.0;
			ddq_Jdot_L_p.x[1][0] = 0.0;
			ddq_Jdot_L_p.x[2][0] = 0.0;
			ddq_Jdot_L_p.x[3][0] = 0.0;
			ddq_Jdot_L_p.x[4][0] = 0.0;
			ddq_Jdot_L_p.x[5][0] = 0.0;


			ddq_Jdot_L_p.x[0][1] = 0.0;
			ddq_Jdot_L_p.x[1][1] = 0.0;
			ddq_Jdot_L_p.x[2][1] = 0.0;
			ddq_Jdot_L_p.x[3][1] = 0.0;
			ddq_Jdot_L_p.x[4][1] = 0.0;
			ddq_Jdot_L_p.x[5][1] = 0.0;
 
  
			ddq_Jdot_L_p.x[0][2] = 0.0;
			ddq_Jdot_L_p.x[1][2] = 0.0;
			ddq_Jdot_L_p.x[2][2] = 0.0;
			ddq_Jdot_L_p.x[3][2] = 0.0;
			ddq_Jdot_L_p.x[4][2] = 0.0;
			ddq_Jdot_L_p.x[5][2] = 0.0;


			ddq_Jdot_L_p.x[0][3] = 0.0;
			ddq_Jdot_L_p.x[1][3] = 0.0;
			ddq_Jdot_L_p.x[2][3] = 0.0;
			ddq_Jdot_L_p.x[3][3] = 0.0;
			ddq_Jdot_L_p.x[4][3] = 0.0;
			ddq_Jdot_L_p.x[5][3] = 0.0;


			ddq_Jdot_L_p.x[0][4] = 0.0;
			ddq_Jdot_L_p.x[1][4] = 0.0;
			ddq_Jdot_L_p.x[2][4] = 0.0;
			ddq_Jdot_L_p.x[3][4] = 0.0;
			ddq_Jdot_L_p.x[4][4] = 0.0;
			ddq_Jdot_L_p.x[5][4] = 0.0;

			ddq_Jdot_L_p.x[0][5] = 0.0;
			ddq_Jdot_L_p.x[1][5] = 0.0;
			ddq_Jdot_L_p.x[2][5] = 0.0;
			ddq_Jdot_L_p.x[3][5] = 0.0;
			ddq_Jdot_L_p.x[4][5] = 0.0;
			ddq_Jdot_L_p.x[5][5] = 0.0;

			ddq_Jdot_R_p.x[0][0] = 0.0;
			ddq_Jdot_R_p.x[1][0] = 0.0;
			ddq_Jdot_R_p.x[2][0] = 0.0;
			ddq_Jdot_R_p.x[3][0] = 0.0;
			ddq_Jdot_R_p.x[4][0] = 0.0;
			ddq_Jdot_R_p.x[5][0] = 0.0;


			ddq_Jdot_R_p.x[0][1] = 0.0;
			ddq_Jdot_R_p.x[1][1] = 0.0;
			ddq_Jdot_R_p.x[2][1] = 0.0;
			ddq_Jdot_R_p.x[3][1] = 0.0;
			ddq_Jdot_R_p.x[4][1] = 0.0;
			ddq_Jdot_R_p.x[5][1] = 0.0;


			ddq_Jdot_R_p.x[0][2] = 0.0;
			ddq_Jdot_R_p.x[1][2] = 0.0;


			ddq_Jdot_R_p.x[2][2] = 0.0;
			ddq_Jdot_R_p.x[3][2] = 0.0;
			ddq_Jdot_R_p.x[4][2] = 0.0;
			ddq_Jdot_R_p.x[5][2] = 0.0;


			ddq_Jdot_R_p.x[0][3] = 0.0;
			ddq_Jdot_R_p.x[1][3] = 0.0;
			ddq_Jdot_R_p.x[2][3] = 0.0;
			ddq_Jdot_R_p.x[3][3] = 0.0;
			ddq_Jdot_R_p.x[4][3] = 0.0;
			ddq_Jdot_R_p.x[5][3] = 0.0;

  
			ddq_Jdot_R_p.x[0][4] = 0.0;
			ddq_Jdot_R_p.x[1][4] = 0.0;
			ddq_Jdot_R_p.x[2][4] = 0.0;
			ddq_Jdot_R_p.x[3][4] = 0.0;
			ddq_Jdot_R_p.x[4][4] = 0.0;
			ddq_Jdot_R_p.x[5][4] = 0.0;


			ddq_Jdot_R_p.x[0][5] = 0.0;
			ddq_Jdot_R_p.x[1][5] = 0.0;
			ddq_Jdot_R_p.x[2][5] = 0.0;
			ddq_Jdot_R_p.x[3][5] = 0.0;
			ddq_Jdot_R_p.x[4][5] = 0.0;
			ddq_Jdot_R_p.x[5][5] = 0.0;

			//ddq_Jdot_L_n = s_vectorminus66_66(_jacobi_L_n,_jacobi_L_p);
				//ddq_Jdot_R_n = s_vectorminus66_66(_jacobi_R_n,_jacobi_R_p);


			_w_LE_n.x[0][0]=0;
			_w_LE_n.x[1][0]=0;
			_w_LE_n.x[2][0]=0;


			_w_RE_n.x[0][0]=0;
			_w_RE_n.x[1][0]=0;
			_w_RE_n.x[2][0]=0;


			_dw_LE_n.x[0][0]=0;
			_dw_LE_n.x[1][0]=0;
			_dw_LE_n.x[2][0]=0;

	
			_dw_RE_n.x[0][0]=0;
			_dw_RE_n.x[1][0]=0;
			_dw_RE_n.x[2][0]=0;
		}
		else{

			_dth_p.x[0][0]=_dth_n.x[0][0];
			_dth_p.x[1][0]=_dth_n.x[1][0];
			_dth_p.x[2][0]=_dth_n.x[2][0];
			_dth_p.x[3][0]=_dth_n.x[3][0];
			_dth_p.x[4][0]=_dth_n.x[4][0];
			_dth_p.x[5][0]=_dth_n.x[5][0];
			_dth_p.x[6][0]=_dth_n.x[6][0];
			_dth_p.x[7][0]=_dth_n.x[7][0];
			_dth_p.x[8][0]=_dth_n.x[8][0];
			_dth_p.x[9][0]=_dth_n.x[9][0];
			_dth_p.x[10][0]=_dth_n.x[10][0];
			_dth_p.x[11][0]=_dth_n.x[11][0];

			//ddq_Jdot_L_p= ddq_Jdot_L_n;
			//ddq_Jdot_R_p= ddq_Jdot_R_n;

			_dth_n = s_vectorminus121_121(_theta_n,_theta_p);

			ddq_Jdot_L_p = s_vectorminus66_66(_jacobi_L_n,_jacobi_L_p);
			ddq_Jdot_R_p = s_vectorminus66_66(_jacobi_R_n,_jacobi_R_p);

       	
			_w_LE_p.x[0][0]=0;
			_w_LE_p.x[1][0]=0;
			_w_LE_p.x[2][0]=_w_LE_n.x[2][0];


			_w_RE_p.x[0][0]=0;
			_w_RE_p.x[1][0]=0;
			_w_RE_p.x[2][0]=_w_RE_n.x[2][0];


			_w_LE_n=s_vectorminus31_31(_q_LE_n,_q_LE_p);
			_w_RE_n=s_vectorminus31_31(_q_RE_n,_q_RE_p);

			_dw_LE_n=s_vectorminus31_31(_w_LE_n,_w_LE_p);
			_dw_RE_n=s_vectorminus31_31(_w_RE_n,_w_RE_p);
		}
		
		
		Mdth_d.x[0][0] = _dth_p.x[0][0];
		Mdth_d.x[1][0] = _dth_p.x[1][0];
		Mdth_d.x[2][0] = _dth_p.x[2][0];
		Mdth_d.x[3][0] = _dth_p.x[3][0];
		Mdth_d.x[4][0] = _dth_p.x[4][0];
		Mdth_d.x[5][0] = _dth_p.x[5][0];
		Mdth_d.x[6][0] = _dth_p.x[6][0];
		Mdth_d.x[7][0] = _dth_p.x[7][0];
		Mdth_d.x[8][0] = _dth_p.x[8][0];
		Mdth_d.x[9][0] = _dth_p.x[9][0];
		Mdth_d.x[10][0] = _dth_p.x[10][0];
		Mdth_d.x[11][0] = _dth_p.x[11][0];


		//Matrix_6X1 ddq_ddx_L, ddq_ddx_R;

		ddq_ddx_R.x[0][0] = _ddp_RE.x[0][0];
		ddq_ddx_R.x[1][0] = _ddp_RE.x[1][0];
		ddq_ddx_R.x[2][0] = _ddp_RE.x[2][0];
		ddq_ddx_R.x[3][0] = _dw_RE_n.x[0][0];
		ddq_ddx_R.x[4][0] = _dw_RE_n.x[1][0];
		ddq_ddx_R.x[5][0] = _dw_RE_n.x[2][0];

		ddq_ddx_L.x[0][0] = _ddp_LE.x[0][0];
		ddq_ddx_L.x[1][0] = _ddp_LE.x[1][0];
		ddq_ddx_L.x[2][0] = _ddp_LE.x[2][0];
		ddq_ddx_L.x[3][0] = _dw_LE_n.x[0][0];
		ddq_ddx_L.x[4][0] = _dw_LE_n.x[1][0];
		ddq_ddx_L.x[5][0] = _dw_LE_n.x[2][0];
	
		ddq_dq_R.x[0][0] = Mdth_d.x[0][0];
		ddq_dq_R.x[1][0] = Mdth_d.x[1][0];
		ddq_dq_R.x[2][0] = Mdth_d.x[2][0];
		ddq_dq_R.x[3][0] = Mdth_d.x[3][0];
		ddq_dq_R.x[4][0] = Mdth_d.x[4][0];
		ddq_dq_R.x[5][0] = Mdth_d.x[5][0];

		ddq_dq_L.x[0][0] = Mdth_d.x[6][0];
		ddq_dq_L.x[1][0] = Mdth_d.x[7][0];
		ddq_dq_L.x[2][0] = Mdth_d.x[8][0];
		ddq_dq_L.x[3][0] = Mdth_d.x[9][0];
		ddq_dq_L.x[4][0] = Mdth_d.x[10][0];
		ddq_dq_L.x[5][0] = Mdth_d.x[11][0];

		ddq_help_1_L = productmatrix66_61(ddq_Jdot_L_p,ddq_dq_L);
		ddq_help_2_L = vectorminus61_61(ddq_ddx_L,ddq_help_1_L);
		ddq_L=productmatrix66_61(_invjacobi_L_p,ddq_help_2_L);

		ddq_help_1_R = productmatrix66_61(ddq_Jdot_R_p,ddq_dq_R);
		ddq_help_2_R = vectorminus61_61(ddq_ddx_R,ddq_help_1_R);
		ddq_R=productmatrix66_61(_invjacobi_R_p,ddq_help_2_R);

		Mddth_d.x[0][0] = ddq_R.x[0][0];
		Mddth_d.x[1][0] = ddq_R.x[1][0];
		Mddth_d.x[2][0] = ddq_R.x[2][0];
		Mddth_d.x[3][0] = ddq_R.x[3][0];
		Mddth_d.x[4][0] = ddq_R.x[4][0];
		Mddth_d.x[5][0] = ddq_R.x[5][0];
		Mddth_d.x[6][0] = ddq_L.x[0][0];
		Mddth_d.x[7][0] = ddq_L.x[1][0];
		Mddth_d.x[8][0] = ddq_L.x[2][0];
		Mddth_d.x[9][0] = ddq_L.x[3][0];
		Mddth_d.x[10][0] = ddq_L.x[4][0];
		Mddth_d.x[11][0] = ddq_L.x[5][0];

			_th_encoder[0] = -mPositionSensors[ankleRollR]->getValue();
			_th_encoder[1] = -mPositionSensors[anklePitchR]->getValue();
			_th_encoder[2] = mPositionSensors[kneePitchR]->getValue();
			_th_encoder[3] = mPositionSensors[hipPitchR]->getValue();
			_th_encoder[4] = mPositionSensors[hipRollR]->getValue();
			_th_encoder[5] = mPositionSensors[hipYawR]->getValue();
			_th_encoder[6] = -mPositionSensors[hipYawL]->getValue();
			_th_encoder[7] = -mPositionSensors[hipRollL]->getValue();
			_th_encoder[8] = mPositionSensors[hipPitchL]->getValue();
			_th_encoder[9] = mPositionSensors[kneePitchL]->getValue();
			_th_encoder[10] = -mPositionSensors[anklePitchL]->getValue();
			_th_encoder[11] = mPositionSensors[ankleRollL]->getValue();
		
			if (_sn==0)
			{
				_th_current_p =_theta_p;
			}

			else{
				_th_current_p =_th_current_n;
			}

			_th_current_n.x[0][0]=_th_encoder[0];
			_th_current_n.x[1][0]=_th_encoder[1];
			_th_current_n.x[2][0]=_th_encoder[2];
			_th_current_n.x[3][0]=_th_encoder[3];
			_th_current_n.x[4][0]=_th_encoder[4];
			_th_current_n.x[5][0]=_th_encoder[5];
			_th_current_n.x[6][0]=_th_encoder[6];
			_th_current_n.x[7][0]=_th_encoder[7];
			_th_current_n.x[8][0]=_th_encoder[8];
			_th_current_n.x[9][0]=_th_encoder[9];
			_th_current_n.x[10][0]=_th_encoder[10];
			_th_current_n.x[11][0]=_th_encoder[11];
      	
			if (_sn==0)
			{
				Mdth.x[0][0]=0.0;
				Mdth.x[1][0]=0.0;
				Mdth.x[2][0]=0.0;
				Mdth.x[3][0]=0.0;
				Mdth.x[4][0]=0.0;
				Mdth.x[5][0]=0.0;
				Mdth.x[6][0]=0.0;
				Mdth.x[7][0]=0.0;
				Mdth.x[8][0]=0.0;
				Mdth.x[9][0]=0.0;
				Mdth.x[10][0]=0.0;
				Mdth.x[11][0]=0.0;
			}
			else
			{
				Mdth = s_vectorminus121_121(_th_current_n, _th_current_p);
			}

			// double Tr_anklePitch=0.013;//0;//0.02;//
			// double _kv_anklePitch=2*(1.8/Tr_anklePitch);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// double _kp_anklePitch=(1.8/Tr_anklePitch)*(1.8/Tr_anklePitch);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
			
			// double Tr_ankleRoll=0.013;//0;//0.02;//
			// double _kv_ankleRoll=2*(1.8/Tr_ankleRoll);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// double _kp_ankleRoll=(1.8/Tr_ankleRoll)*(1.8/Tr_ankleRoll);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
			
			// double Tr_kneePitch=0.02;//0;//0.02;//
			// double _kv_kneePitch=2*(1.8/Tr_kneePitch);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// double _kp_kneePitch=(1.8/Tr_kneePitch)*(1.8/Tr_kneePitch);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);

			// double Tr_hipPitch=0.035;//0;//0.02;//
			// double _kv_hipPitch=2*(1.8/Tr_hipPitch);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// double _kp_hipPitch=(1.8/Tr_hipPitch)*(1.8/Tr_hipPitch);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);

			// double Tr_hipRoll=0.035;//0;//0.02;//
			// double _kv_hipRoll=2*(1.8/Tr_hipRoll);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// double _kp_hipRoll=(1.8/Tr_hipRoll)*(1.8/Tr_hipRoll);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);

			_error_dth_1=vectorminus121_121(Mdth_d,Mdth) ;
			_error_dth=productmatrix121_d(_kv,_error_dth_1);

			// _error_dth.x[0][0]=_error_dth.x[0][0]/_kv*_kv_ankleRoll;
			// _error_dth.x[11][0]=_error_dth.x[11][0]/_kv*_kv_ankleRoll;

			// _error_dth.x[1][0]=_error_dth.x[1][0]/_kv*_kv_anklePitch;
			// _error_dth.x[10][0]=_error_dth.x[10][0]/_kv*_kv_anklePitch;

			// _error_dth.x[2][0]=_error_dth.x[2][0]/_kv*_kv_kneePitch;
			// _error_dth.x[9][0]=_error_dth.x[9][0]/_kv*_kv_kneePitch;

			// _error_dth.x[3][0]=_error_dth.x[3][0]/_kv*_kv_hipPitch;
			// _error_dth.x[8][0]=_error_dth.x[8][0]/_kv*_kv_hipPitch;

			// _error_dth.x[4][0]=_error_dth.x[4][0]/_kv*_kv_hipRoll;
			// _error_dth.x[7][0]=_error_dth.x[7][0]/_kv*_kv_hipRoll;


			_error_th_1=vectorminus121_121(_theta_p,_th_current_n) ;
			_error_th=productmatrix121_d(_kp,_error_th_1);

			// _error_th.x[0][0]=_error_th.x[0][0]/_kp*_kp_ankleRoll;
			// _error_th.x[11][0]=_error_th.x[11][0]/_kp*_kp_ankleRoll;

			// _error_th.x[1][0]=_error_th.x[1][0]/_kp*_kp_anklePitch;
			// _error_th.x[10][0]=_error_th.x[10][0]/_kp*_kp_anklePitch;

			// _error_th.x[2][0]=_error_th.x[2][0]/_kp*_kp_kneePitch;
			// _error_th.x[9][0]=_error_th.x[9][0]/_kp*_kp_kneePitch;

			// _error_th.x[3][0]=_error_th.x[3][0]/_kp*_kp_hipPitch;
			// _error_th.x[8][0]=_error_th.x[8][0]/_kp*_kp_hipPitch;

			// _error_th.x[4][0]=_error_th.x[4][0]/_kp*_kp_hipRoll;
			// _error_th.x[7][0]=_error_th.x[7][0]/_kp*_kp_hipRoll;

#if iCtrlOn
		if (_sn==0)
		{
			pre_error_th_1 = _error_th_1;
			i_error=productmatrix121_d_d(0.5, samplingtime, _error_th_1);
			i_control=productmatrix121_d(iCtrlGain, i_error);
			pre_i_control=i_control;
			Mddth =vectorplus121_121_121_121(Mddth_d,_error_dth,_error_th,i_control);
		}
		else
		{
			
			i_error =vectorplus121_121(pre_error_th_1,_error_th_1);
			pre_error_th_1 = _error_th_1;
			i_error=productmatrix121_d_d(0.5, samplingtime, i_error);
			i_control=productmatrix121_d(iCtrlGain, i_error);
			i_control=vectorplus121_121(pre_i_control,i_control);
			pre_i_control=i_control;
			Mddth =vectorplus121_121_121_121(Mddth_d,_error_dth,_error_th,i_control);
#if debugPrintOn
			printf("\n icontrol %lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/",
			i_control.x[0][0],i_control.x[1][0],i_control.x[2][0],i_control.x[3][0],
			i_control.x[4][0],i_control.x[5][0],i_control.x[6][0],i_control.x[7][0],
			i_control.x[8][0],i_control.x[9][0],i_control.x[10][0],i_control.x[11][0]);
			printf("\n i_error %lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/%lf/",
			i_error.x[0][0],i_error.x[1][0],i_error.x[2][0],i_error.x[3][0],
			i_error.x[4][0],i_error.x[5][0],i_error.x[6][0],i_error.x[7][0],
			i_error.x[8][0],i_error.x[9][0],i_error.x[10][0],i_error.x[11][0]);
#endif
		}
		
#else
		Mddth =vectorplus121_121_121(Mddth_d,_error_dth,_error_th);
#endif	


			//RNE 초기 조건 
			wr00.x[0][0] = 0;
			wr00.x[1][0] = 0;
			wr00.x[2][0] = 0;

			dwr00.x[0][0] = 0;
			dwr00.x[1][0] = 0;
			dwr00.x[2][0] = 0;

			dvr00.x[0][0] = 0;
			dvr00.x[1][0] = 0;
			dvr00.x[2][0] = 9.81; //0

			dvcr00.x[0][0] = 0;
			dvcr00.x[1][0] = 0;
			dvcr00.x[2][0] = 0;

			//Matrix_3X1 Fr00, Nr00, fr77, nr77;

			Fr00.x[0][0] = 0;
			Fr00.x[1][0] = 0;
			Fr00.x[2][0] = 0;

			Nr00.x[0][0] = 0;
			Nr00.x[1][0] = 0;
			Nr00.x[2][0] = 0;


			if(sensorOn==0){
				f_Xr = 0.0;
				f_Yr = 0.0;
				tq_Zr = 0.0;
				tq_Xr = 0.0;
				tq_Yr = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zr = d_Fz_SSP;
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zr = 0.0;
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zr = d_Fz_DSP;//ps_RforceZ_last;//
				}
				else if (phase == 2) // DSP_L
				{
					f_Zr = d_Fz_DSP;//ps_RforceZ_last;//
				}
			}
			else if(sensorOn==4){
				f_Xr = 0.0;
				f_Yr = 0.0;
				f_Zr = forceZr;
				tq_Xr = 0;
				tq_Yr = 0;
				tq_Zr = 0.0;
			}
			else if(sensorOn==5){
				f_Xr = 0.0;
				f_Yr = 0.0;
				f_Zr = force_F_R[2];
				tq_Xr = 0;
				tq_Yr = 0;
				tq_Zr = 0.0;
			}
            else if(sensorOn==6){
				f_Xr = 0.0;
				f_Yr = 0.0;
				tq_Xr = torque_F_R[0];//d_tqX_SSP;
				tq_Yr = torque_F_R[1];//d_tqY_SSP;
				tq_Zr = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zr = d_Fz_SSP;//
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zr = 0.0;
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zr = d_Fz_DSP;//20*0.5;//ps_RforceZ_last;//
				}
				else if (phase == 2) // DSP_L
				{
					f_Zr = d_Fz_DSP;//20*0.5;//ps_RforceZ_last;//
				}
            }
            else if(sensorOn==7){
				f_Xr = 0.0;
				f_Yr = 0.0;
				tq_Zr = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zr = f_zr_p1_pos;
					tq_Xr = tq_xr_p1_pos;
					tq_Yr = tq_yr_p1_pos;
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zr = f_zr_p0_pos;
					tq_Xr = tq_xr_p0_pos;
					tq_Yr = tq_yr_p0_pos;
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zr = f_zr_p3_pos;//ps_RforceZ_last;//
					tq_Xr = tq_xr_p3_pos;//ps_RtorqueX_last;//
					tq_Yr = tq_yr_p3_pos;//ps_RtorqueY_last;//
				}
				else if (phase == 2) // DSP_L
				{
					f_Zr = f_zr_p2_pos;//ps_RforceZ_last;//
					tq_Xr = tq_xr_p2_pos;//ps_RtorqueX_last;//
					tq_Yr = tq_yr_p2_pos;//ps_RtorqueY_last;//
				}
			}
			else if(sensorOn==1){
				f_Xr = 0.0;
				f_Yr = 0.0;
				tq_Xr = torquexr;//d_tqX_SSP;
				tq_Yr = torqueyr;//d_tqY_SSP;
				tq_Zr = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zr = d_Fz_SSP;//
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zr = 0.0;
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zr = d_Fz_DSP;//20*0.5;//ps_RforceZ_last;//
				}
				else if (phase == 2) // DSP_L
				{
					f_Zr = d_Fz_DSP;//20*0.5;//ps_RforceZ_last;//
				}
            }
			else if(sensorOn==2){
				f_Zr = force_F_R[2];
				tq_Xr = torque_F_R[0];
				tq_Yr = torque_F_R[1];
            }
			else if(sensorOn==3){
				f_Xr = force_F_R[0];
				f_Yr = force_F_R[1];
				f_Zr = force_F_R[2];
				tq_Xr = torque_F_R[0];
				tq_Yr = torque_F_R[1];
				tq_Zr = torque_F_R[2];
			}
			

			fr77.x[0][0] =  -f_Xr;
			fr77.x[1][0] =  -f_Yr;
			fr77.x[2][0] =  -f_Zr;

			nr77.x[0][0] = -tq_Xr;
			nr77.x[1][0] = -tq_Yr;
			nr77.x[2][0] = -tq_Zr;

			RNEwr.x[0][0] = wr00;
			RNEdwr.x[0][0] = dwr00;
			RNEdvr.x[0][0] = dvr00;
			RNEdvcr.x[0][0] = dvcr00;
			RNEFr.x[0][0] = Fr00;
			RNENr.x[0][0] = Nr00;

			RNEfr.x[0][6] = fr77;
			RNEnr.x[0][6] = nr77;

			for (k = 0; k < 6; k++) { ///outward iteration

				rotationr = invRR.x[k][0]; //3x3
				omegar = RNEwr.x[0][k]; //3x1
				productwr = productmatrix33_31(rotationr, omegar);//3x1

				ta_zr=ta.x[0][5-k];//3x1
				wr_m2_1=Mdth.x[5-k][0];//1x1(double)
				wr_m2 = productmatrix11_31(wr_m2_1, ta_zr);//3x1


				RNEwr.x[0][k + 1] = vectorplus31_31(productwr, wr_m2); // 3x1


				domegar = RNEdwr.x[0][k];//3x1
				productdwr1 = productmatrix33_31(rotationr, domegar);//3x1

				cproductdwr1 = crossproductmatrix31_31(productwr, wr_m2);//3x1


				ta_zr=ta.x[0][5-k];
				dwr_m2_1=Mddth.x[5-k][0];
				dwr_m2 = productmatrix11_31(dwr_m2_1, ta_zr);//3x1


				RNEdwr.x[0][k + 1] = vectorplus31_31_31(productdwr1, cproductdwr1, dwr_m2);//3x1


				vpr1 = crossproductmatrix31_31(domegar, PositionppR.x[0][k]);//3x1
				vpr2_1 = crossproductmatrix31_31(omegar, PositionppR.x[0][k]);//3x1
				vpr2 = crossproductmatrix31_31(omegar, vpr2_1);//3x1
				vpr3 = RNEdvr.x[0][k];//3x1
				vectorplusdvr1 = vectorplus31_31_31(vpr1, vpr2, vpr3);//3x1

				RNEdvr.x[0][k + 1] = productmatrix33_31(rotationr, vectorplusdvr1);//3x1

				positionCr = PositionC.x[0][5-k];//3x1
				Rdvcr1 = crossproductmatrix31_31(RNEdwr.x[0][k + 1], positionCr);//3x1
				Rdvcr2_1 = crossproductmatrix31_31(RNEwr.x[0][k + 1], positionCr);//3x1
				Rdvcr2 = crossproductmatrix31_31(RNEwr.x[0][k + 1], Rdvcr2_1);//3x1

				RNEdvcr.x[0][k + 1] = vectorplus31_31_31(Rdvcr1, Rdvcr2, RNEdvr.x[0][k + 1]);//3x1

				helpRNEdvcr = RNEdvcr.x[0][k + 1];//3x1
				helpRNEFr.x[0][0] = RNEmass[5-k] * helpRNEdvcr.x[0][0];
				helpRNEFr.x[1][0] = RNEmass[5-k] * helpRNEdvcr.x[1][0];
				helpRNEFr.x[2][0] = RNEmass[5-k] * helpRNEdvcr.x[2][0];

				RNEFr.x[0][k + 1] = helpRNEFr;//3x1

				RNENr_1_1 = Inertia.x[0][5-k];//3x3
				RNENr_1 = productmatrix33_31(RNENr_1_1, RNEdwr.x[0][k + 1]);//3x1
				RNENr_2 = productmatrix33_31(RNENr_1_1, RNEwr.x[0][k + 1]);//3x1
				RNENr_3 = crossproductmatrix31_31(RNEwr.x[0][k + 1], RNENr_2);//3x1

				RNENr.x[0][k + 1] = vectorplus31_31(RNENr_1, RNENr_3);//3x1


			}

			for (k = 6; k > 0; k = k - 1) {  ///inward iteration

				RNEfr_1 = productmatrix33_31(RR.x[k][0], RNEfr.x[0][k]);//3x1
				RNEfr.x[0][k - 1] = vectorplus31_31(RNEfr_1, RNEFr.x[0][k]);//3x1

				inwardpositionCr = PositionC.x[0][6-k];//3x1
				RNEnr_1 = productmatrix33_31(RR.x[k][0], RNEnr.x[0][k]);//3x1
				RNEnr_2 = crossproductmatrix31_31(inwardpositionCr, RNEFr.x[0][k]);//3x1
				RNEnr_3 = crossproductmatrix31_31(PositionppR.x[0][k], RNEfr_1);//3x1

				RNEnr.x[0][k - 1] = vectorplus31_31_31_31(RNENr.x[0][k], RNEnr_1, RNEnr_2, RNEnr_3);//3x1

				helpFinaltorquer = RNEnr.x[0][k - 1];//3x1
				helpFinaltorquer_1= transposeMatrix_31(helpFinaltorquer);//1x3
				ta_zr_1 = ta.x[0][6-k];//3x1
				Finaltorque.x[6-k][0] = productmatrix13_31(helpFinaltorquer_1,ta_zr_1);//double
			}



			//RNE 초기 조건 
			wl00.x[0][0] = 0;
			wl00.x[1][0] = 0;
			wl00.x[2][0] = 0;

			dwl00.x[0][0] = 0;
			dwl00.x[1][0] = 0;
			dwl00.x[2][0] = 0;

			dvl00.x[0][0] = 0;
			dvl00.x[1][0] = 0;
			dvl00.x[2][0] = 9.81; //0

			dvcl00.x[0][0] = 0;
			dvcl00.x[1][0] = 0;
			dvcl00.x[2][0] = 0;



			Fl00.x[0][0] = 0;
			Fl00.x[1][0] = 0;
			Fl00.x[2][0] = 0;

			Nl00.x[0][0] = 0;
			Nl00.x[1][0] = 0;
			Nl00.x[2][0] = 0;
			//f,n
            
			if(sensorOn==0)
			{
                f_Xl = 0.0;
                f_Yl = 0.0;
                tq_Xl = 0.0;
                tq_Yl = 0.0;
                tq_Zl = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zl = 0.0;
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zl = d_Fz_SSP;
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zl =  d_Fz_DSP;//ps_LforceZ_last;//
				}
				else if (phase == 2) // DSP_L
				{
					f_Zl = d_Fz_DSP;//ps_LforceZ_last;//
				}
			}
			else if(sensorOn==4){
				f_Xl = 0.0;
				f_Yl = 0.0;
				f_Zl = forceZl;
				tq_Xl = 0.0;
				tq_Yl = 0.0;
				tq_Zl = 0.0;
			}
			else if(sensorOn==5){
				f_Xl = 0.0;
				f_Yl = 0.0;
				f_Zl = force_F_L[2];
				tq_Xl = 0.0;
				tq_Yl = 0.0;
				tq_Zl = 0.0;
			}
            else if(sensorOn==6){
				f_Xl = 0.0;
				f_Yl = 0.0;
				tq_Xl = torque_F_L[0];//d_tqX_SSP;
				tq_Yl = torque_F_L[1];//d_tqY_SSP;
				tq_Zl = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zl = 0.0;
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zl = d_Fz_SSP;//
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zl = d_Fz_DSP;//ps_RforceZ_last;//d_Fz_DSP;
				}
				else if (phase == 2) // DSP_L
				{
					f_Zl = d_Fz_DSP;//ps_RforceZ_last;//d_Fz_DSP;
				}
            }
            else if(sensorOn==7)
            {
                f_Xl = 0.0;
                f_Yl = 0.0;
                tq_Zl = 0.0;
                if (phase == 1) // SSP_R 
                {
                    f_Zl = f_zl_p1_pos;
                    tq_Xl = tq_xl_p1_pos;
                    tq_Yl = tq_yl_p1_pos;
                }
                else if (phase == 0) // SSP_L 
                {
                    f_Zl = f_zl_p0_pos;
                    tq_Xl = tq_xl_p0_pos;
                    tq_Yl = tq_yl_p0_pos;
                }
                else if (phase == 3) // DSP_R 
                {
                    f_Zl =  f_zl_p3_pos;//ps_LforceZ_last;//
                    tq_Xl = tq_xl_p3_pos;//ps_LtorqueX_last;//
                    tq_Yl = tq_yl_p3_pos;//ps_LtorqueY_last;//
                }
                else if (phase == 2) // DSP_L
                {
                    f_Zl = f_zl_p2_pos;//ps_LforceZ_last;//
                    tq_Xl = tq_xl_p2_pos;//ps_LtorqueX_last;//
                    tq_Yl = tq_yl_p2_pos;//ps_LtorqueY_last;//
                }
            }
			else if(sensorOn==1){
				f_Xl = 0.0;
				f_Yl = 0.0;
				tq_Xl = torquexl;//d_tqX_SSP;
				tq_Yl = torqueyl;//d_tqY_SSP;
				tq_Zl = 0.0;
				if (phase == 1) // SSP_R 
				{
					f_Zl = 0.0;
				}
				else if (phase == 0) // SSP_L 
				{
					f_Zl = d_Fz_SSP;//
				}
				else if (phase == 3) // DSP_R 
				{
					f_Zl = d_Fz_DSP;//ps_RforceZ_last;//d_Fz_DSP;
				}
				else if (phase == 2) // DSP_L
				{
					f_Zl = d_Fz_DSP;//ps_RforceZ_last;//d_Fz_DSP;
				}
            }
			
            else if(sensorOn==2){
				f_Zl = force_F_L[2];
				tq_Xl = torque_F_L[0];
				tq_Yl = torque_F_L[1];
            }
			else if(sensorOn==3){
				f_Xl = force_F_L[0];
				f_Yl = force_F_L[1];
				f_Zl = force_F_L[2];
				tq_Xl = torque_F_L[0];
				tq_Yl = torque_F_L[1];
				tq_Zl = torque_F_L[2];
            }


			fl77.x[0][0] =  -f_Xl;
			fl77.x[1][0] =  -f_Yl;
			fl77.x[2][0] =  -f_Zl;

			nl77.x[0][0] = -tq_Xl;
			nl77.x[1][0] = -tq_Yl;
			nl77.x[2][0] = -tq_Zl;
		// printf("\n phase=%d\n",phase);


			///RNE
			//outward iteration

			RNEwl.x[0][0] = wl00;
			RNEdwl.x[0][0] = dwl00;
			RNEdvl.x[0][0] = dvl00;
			RNEdvcl.x[0][0] = dvcl00;
			RNEFl.x[0][0] = Fl00;
			RNENl.x[0][0] = Nl00;

			RNEfl.x[0][6] = fl77;
			RNEnl.x[0][6] = nl77;

			// B->R
			for (k = 0; k < 6; k++) { ///outward iteration

				rotationl = invRL.x[k][0]; //3x3
				omegal = RNEwl.x[0][k]; //3x1
				productwl = productmatrix33_31(rotationl, omegal);//3x1

				ta_zl=ta.x[0][k+6];//3x1
				wl_m2_1=Mdth.x[k+6][0];//double
				wl_m2 = productmatrix11_31(wl_m2_1, ta_zl);//3x1

				RNEwl.x[0][k + 1] = vectorplus31_31(productwl, wl_m2); // w(i+1)

				domegal = RNEdwl.x[0][k];//3x1
				productdwl1 = productmatrix33_31(rotationl, domegal);//3x1

				cproductdwl1 = crossproductmatrix31_31(productwl, wl_m2);//3x1


				ta_zl=ta.x[0][k+6];//3x1
				dwl_m2_1=Mddth.x[k+6][0];//double
				dwl_m2 = productmatrix11_31(dwl_m2_1, ta_zl);

				RNEdwl.x[0][k + 1] = vectorplus31_31_31(productdwl1, cproductdwl1, dwl_m2);//3x1

				vpl1 = crossproductmatrix31_31(domegal, PositionppL.x[0][k]);//3x1
				vpl2_1 = crossproductmatrix31_31(omegal, PositionppL.x[0][k]);//3x1
				vpl2 = crossproductmatrix31_31(omegal, vpl2_1);//3x1
				vpl3 = RNEdvl.x[0][k];//3x1
				vectorplusdvl1 = vectorplus31_31_31(vpl1, vpl2, vpl3);//3x1

				RNEdvl.x[0][k + 1] = productmatrix33_31(rotationl, vectorplusdvl1);//3x1

				positionCl = PositionC.x[0][k+6];//3x1
				Rdvcl1 = crossproductmatrix31_31(RNEdwl.x[0][k + 1], positionCl);//3x1
				Rdvcl2_1 = crossproductmatrix31_31(RNEwl.x[0][k + 1], positionCl);//3x1
				Rdvcl2 = crossproductmatrix31_31(RNEwl.x[0][k + 1], Rdvcl2_1);//3x1

				RNEdvcl.x[0][k + 1] = vectorplus31_31_31(Rdvcl1, Rdvcl2, RNEdvl.x[0][k + 1]);//3x1

				helpRNEdvcl = RNEdvcl.x[0][k + 1];//3x1
				helpRNEFl.x[0][0] = RNEmass[k+6] * helpRNEdvcl.x[0][0];//m*dvc
				helpRNEFl.x[1][0] = RNEmass[k+6] * helpRNEdvcl.x[1][0];
				helpRNEFl.x[2][0] = RNEmass[k+6] * helpRNEdvcl.x[2][0];

				RNEFl.x[0][k + 1] = helpRNEFl;//3x1

				RNENl_1_1 = Inertia.x[0][k+6];//3x3
				RNENl_1 = productmatrix33_31(RNENl_1_1, RNEdwl.x[0][k + 1]);//3x1
				RNENl_2 = productmatrix33_31(RNENl_1_1, RNEwl.x[0][k + 1]);//3x1
				RNENl_3 = crossproductmatrix31_31(RNEwl.x[0][k + 1], RNENl_2);//3x1

				RNENl.x[0][k + 1] = vectorplus31_31(RNENl_1, RNENl_3);//3x1
			}
			for (k = 6; k > 0; k = k - 1) {  ///inward iteration

				RNEfl_1 = productmatrix33_31(RL.x[k][0], RNEfl.x[0][k]);//3x1
				RNEfl.x[0][k - 1] = vectorplus31_31(RNEfl_1, RNEFl.x[0][k]);//3x1

				inwardpositionCl = PositionC.x[0][k+5];//3x1
				RNEnl_1 = productmatrix33_31(RL.x[k][0], RNEnl.x[0][k]);//3x1
				RNEnl_2 = crossproductmatrix31_31(inwardpositionCl, RNEFl.x[0][k]);//3x1
				RNEnl_3 = crossproductmatrix31_31(PositionppL.x[0][k], RNEfl_1);//3x1

				RNEnl.x[0][k - 1] = vectorplus31_31_31_31(RNENl.x[0][k], RNEnl_1, RNEnl_2, RNEnl_3);//3x1

			/*
			helpFinaltorque = RNEn.x[0][i - 1];//n(i)-1
			Finaltorque.x[6-i][0] = productmatrix11_31(helpFinaltorque_1,ta_z);
			*/

				helpFinaltorquel = RNEnl.x[0][k - 1];//3x1
				helpFinaltorquel_1= transposeMatrix_31(helpFinaltorquel);//1x3
				ta_zl_1 = ta.x[0][k+5];//3x1
				Finaltorque.x[k+5][0] = productmatrix13_31(helpFinaltorquel_1,ta_zl_1);//double
			}

			//printf("_sn:%d\n\n\n",_sn);
			torque1 = Finaltorque.x[0][0];
			torque2 = Finaltorque.x[1][0];
			torque3 = Finaltorque.x[2][0];
			torque4 = Finaltorque.x[3][0];
			torque5 = Finaltorque.x[4][0];
			torque6 = Finaltorque.x[5][0];
			torque7 = Finaltorque.x[6][0];
			torque8 = Finaltorque.x[7][0];
			torque9 = Finaltorque.x[8][0];
			torque10 = Finaltorque.x[9][0];
			torque11 = Finaltorque.x[10][0];
			torque12 = Finaltorque.x[11][0];

			// double  y_zmpboundary_high,y_zmpboundary_low;
			// if(new_phase_num==2)//DSP_R
			// {
			// 	y_zmpboundary_high = _py + 0.38 + 0.074;
			// 	y_zmpboundary_low = _py - 0.38;
			// }
			// else if(new_phase_num==4)//DSP_L
			// {
			// 	y_zmpboundary_high = _py + 0.38;
			// 	y_zmpboundary_low = _py-(+0.38 + 0.074);

			// }
			// else if(new_phase_num==1)//SSP_R
			// {
			// 	y_zmpboundary_high = _py + 0.16;
			// 	y_zmpboundary_low = _py - 0.38;

			// }
			// else if(new_phase_num==3)//SSP_L
			// {
			// 	y_zmpboundary_high = _py + 0.38;
			// 	y_zmpboundary_low = _py - 0.16;
			// }
//추후 변경 예정

		// printf("y_zmpboundary_high,y_zmpboundary_low: %lf, %lf\n\n\n\n",y_zmpboundary_high,y_zmpboundary_low);

			_sn=_sn + 1;
			
			printf("torque:  %lf  %lf  %lf  %lf  %lf  %lf &&& %lf  %lf  %lf  %lf  %lf  %lf \n  ",
			torque1, torque2, torque3, torque4, torque5, torque6,
			torque7, torque8, torque9, torque10, torque11, torque12);
#if debugPrintOn
            printf("\n hipYawtorque     : R=%lf L=%lf errorR=%lf L=%lf",torque6,-torque7,_error_th_1.x[5][0]*180/pi,_error_th_1.x[6][0]*180/pi);
            printf("\n hipRolltorque    : R=%lf L=%lf errorR=%lf L=%lf",torque5,-torque8,_error_th_1.x[4][0]*180/pi,_error_th_1.x[7][0]*180/pi);
            printf("\n hipPitchtorque   : R=%lf L=%lf errorR=%lf L=%lf",torque4,torque9,_error_th_1.x[3][0]*180/pi,_error_th_1.x[8][0]*180/pi);
            printf("\n kneePitchtorque  : R=%lf L=%lf errorR=%lf L=%lf",torque3,torque10,_error_th_1.x[2][0]*180/pi,_error_th_1.x[9][0]*180/pi);
            printf("\n anklepitchtorque : R=%lf L=%lf errorR=%lf L=%lf",-torque2,-torque11,_error_th_1.x[1][0]*180/pi,_error_th_1.x[10][0]*180/pi);
            printf("\n anklerolltorque  : R=%lf L=%lf errorR=%lf L=%lf",-torque1,torque12,_error_th_1.x[0][0]*180/pi,_error_th_1.x[11][0]*180/pi);   
#endif
		
			double torque[12]={torque1,torque2,torque3,torque4,torque5,torque6,torque7,torque8,torque9,torque10,torque11,torque12};

		// printf( "_theta_p    = %lf  %lf  %lf  %lf  %lf  %f &&& %f  %lf  %lf  %lf  %lf  %lf \n  ", _theta_p.x[0][0], _theta_p.x[1][0], _theta_p.x[2][0], _theta_p.x[3][0], _theta_p.x[4][0], _theta_p.x[5][0], _theta_p.x[6][0], _theta_p.x[7][0], _theta_p.x[8][0], _theta_p.x[9][0], _theta_p.x[10][0], _theta_p.x[11][0]);
		// printf( "_th_encoder = %lf  %lf  %lf  %lf  %lf  %f &&& %f  %lf  %lf  %lf  %lf  %lf \n  ", _th_encoder[0], _th_encoder[1], _th_encoder[2], _th_encoder[3], _th_encoder[4], _th_encoder[5], _th_encoder[6], _th_encoder[7], _th_encoder[8], _th_encoder[9], _th_encoder[10], _th_encoder[11]);

						// printf("rtorque:  %lf  %lf  %lf  %lf  %lf  %f  %f  %lf  %lf  %lf  %lf  %lf  \n ", rtorque1, rtorque2, rtorque3, rtorque4, rtorque5, rtorque6, rtorque7, rtorque8, rtorque9, rtorque10, rtorque11, rtorque12);							 
		
		err_th1 = _error_th_1.x[0][0];
		err_th2 = _error_th_1.x[1][0];
		err_th3 = _error_th_1.x[2][0];
		err_th4 = _error_th_1.x[3][0];
		err_th5 = _error_th_1.x[4][0];
		err_th6 = _error_th_1.x[5][0];
		err_th7 = _error_th_1.x[6][0];
		err_th8 = _error_th_1.x[7][0];						   
		err_th9 = _error_th_1.x[8][0];
		err_th10 = _error_th_1.x[9][0];
		err_th11 = _error_th_1.x[10][0];
		err_th12 = _error_th_1.x[11][0];

		// err_dth1 = _error_th_1.x[0][0]/samplingtime;
		// err_dth2 = _error_th_1.x[1][0]/samplingtime;
		// err_dth3 = _error_th_1.x[2][0]/samplingtime;
		// err_dth4 = _error_th_1.x[3][0]/samplingtime;
		// err_dth5 = _error_th_1.x[4][0]/samplingtime;
		// err_dth6 = _error_th_1.x[5][0]/samplingtime;
		// err_dth7 = _error_th_1.x[6][0]/samplingtime;
		// err_dth8 = _error_th_1.x[7][0]/samplingtime;
		// err_dth9 = _error_th_1.x[8][0]/samplingtime;
		// err_dth10 = _error_th_1.x[9][0]/samplingtime;
		// err_dth11 = _error_th_1.x[10][0]/samplingtime;
		// err_dth12 = _error_th_1.x[11][0]/samplingtime;

		err_dth1 = Mdth.x[0][0];
		err_dth2 = Mdth.x[1][0];
		err_dth3 = Mdth.x[2][0];
		err_dth4 = Mdth.x[3][0];
		err_dth5 = Mdth.x[4][0];
		err_dth6 = Mdth.x[5][0];
		err_dth7 = Mdth.x[6][0];
		err_dth8 = Mdth.x[7][0];
		err_dth9 = Mdth.x[8][0];
		err_dth10 = Mdth.x[9][0];
		err_dth11 = Mdth.x[10][0];
		err_dth12 = Mdth.x[11][0];

		// err_dth1 = _error_dth_1.x[0][0];
		// err_dth2 = _error_dth_1.x[1][0];
		// err_dth3 = _error_dth_1.x[2][0];
		// err_dth4 = _error_dth_1.x[3][0];
		// err_dth5 = _error_dth_1.x[4][0];
		// err_dth6 = _error_dth_1.x[5][0];
		// err_dth7 = _error_dth_1.x[6][0];
		// err_dth8 = _error_dth_1.x[7][0];						   
		// err_dth9 = _error_dth_1.x[8][0];
		// err_dth10 = _error_dth_1.x[9][0];
		// err_dth11 = _error_dth_1.x[10][0];
		// err_dth12 = _error_dth_1.x[11][0];

#if (torqueModeOn==1)
        
		//  mMotors[16]->setTorque(-torque[0]);
		//  mMotors[14]->setTorque(torque[1]);
		//  mMotors[12]->setTorque(-torque[2]);
		//  mMotors[10]->setTorque(-torque[3]);
		//  mMotors[8]->setTorque(torque[4]);
		//  mMotors[6]->setTorque(torque[5]);
		//  mMotors[7]->setTorque(-torque[6]);
		//  mMotors[9]->setTorque(-torque[7]);
		//  mMotors[11]->setTorque(-torque[8]);
		//  mMotors[13]->setTorque(-torque[9]);
		//  mMotors[15]->setTorque(torque[10]);
		//  mMotors[17]->setTorque(torque[11]);
		//민하 original
	#if (jointCtrlOn==1)
		{
			mMotors[ankleRollR]->setTorque(-torque[0]-jointCtrlPGain[0]*err_th1*180/pi+jointDamperGain*err_dth1);
			mMotors[anklePitchR]->setTorque(-torque[1]-jointCtrlPGain[1]*err_th2*180/pi+jointDamperGain*err_dth2);
			mMotors[kneePitchR]->setTorque(torque[2]+jointCtrlPGain[2]*err_th3*180/pi-jointDamperGain*err_dth3);
			mMotors[hipPitchR]->setTorque(torque[3]+jointCtrlPGain[3]*err_th4*180/pi-jointDamperGain*err_dth4);
			mMotors[hipRollR]->setTorque(torque[4]+jointCtrlPGain[4]*err_th5*180/pi-jointDamperGain*err_dth5);
			mMotors[hipYawR]->setTorque(torque[5]+jointCtrlPGain[5]*err_th6*180/pi-jointDamperGain*err_dth6);
			mMotors[hipYawL]->setTorque(-torque[6]-jointCtrlPGain[6]*err_th7*180/pi+jointDamperGain*err_dth7);
			mMotors[hipRollL]->setTorque(-torque[7]-jointCtrlPGain[7]*err_th8*180/pi+jointDamperGain*err_dth8);
			mMotors[hipPitchL]->setTorque(torque[8]+jointCtrlPGain[8]*err_th9*180/pi-jointDamperGain*err_dth9);
			mMotors[kneePitchL]->setTorque(torque[9]+jointCtrlPGain[9]*err_th10*180/pi-jointDamperGain*err_dth10);
			mMotors[anklePitchL]->setTorque(-torque[10]-jointCtrlPGain[10]*err_th11*180/pi+jointDamperGain*err_dth11);
			mMotors[ankleRollL]->setTorque(torque[11]+jointCtrlPGain[11]*err_th12*180/pi-jointDamperGain*err_dth12);
		}
	#else
		{
			mMotors[ankleRollR]->setTorque(-torque[0]);
			mMotors[anklePitchR]->setTorque(-torque[1]);
			mMotors[kneePitchR]->setTorque(torque[2]);
			mMotors[hipPitchR]->setTorque(torque[3]);
			mMotors[hipRollR]->setTorque(torque[4]);
			mMotors[hipYawR]->setTorque(torque[5]);
			mMotors[hipYawL]->setTorque(-torque[6]);
			mMotors[hipRollL]->setTorque(-torque[7]);
			mMotors[hipPitchL]->setTorque(torque[8]);
			mMotors[kneePitchL]->setTorque(torque[9]);
			mMotors[anklePitchL]->setTorque(-torque[10]);
			mMotors[ankleRollL]->setTorque(torque[11]);
		}
	#endif            
		
#elif (torqueModeOn==0)
			// mMotors[16]->setPosition(-_th[0]);
			// mMotors[14]->setPosition(_th[1]);
			// mMotors[12]->setPosition(-_th[2]);
			// mMotors[10]->setPosition(-_th[3]);
			// mMotors[8]->setPosition(_th[4]);
			// mMotors[6]->setPosition(_th[5]);
			// mMotors[7]->setPosition(-_th[6]);
			// mMotors[9]->setPosition(-_th[7]);
			// mMotors[11]->setPosition(-_th[8]);
			// mMotors[13]->setPosition(-_th[9]);
			// mMotors[15]->setPosition(_th[10]);
			// mMotors[17]->setPosition(_th[11]);

			//ORIGIN 0630
			mMotors[ankleRollR]->setPosition(-_th[0]);
			mMotors[anklePitchR]->setPosition(-_th[1]);
			mMotors[kneePitchR]->setPosition(_th[2]);
			mMotors[hipPitchR]->setPosition(_th[3]);
			mMotors[hipRollR]->setPosition(_th[4]);
			mMotors[hipYawR]->setPosition(_th[5]);
			mMotors[hipYawL]->setPosition(-_th[6]);
			mMotors[hipRollL]->setPosition(-_th[7]);
			mMotors[hipPitchL]->setPosition(_th[8]);
			mMotors[kneePitchL]->setPosition(_th[9]);
			mMotors[anklePitchL]->setPosition(-_th[10]);
			mMotors[ankleRollL]->setPosition(_th[11]);

			// mMotors[16]->setPosition(-0.008386);
			// mMotors[14]->setPosition(-0.722485);
			// mMotors[12]->setPosition(-1.573324);
			// mMotors[10]->setPosition(0.850839);
			// mMotors[8]->setPosition(0.008386);
			// mMotors[6]->setPosition(0);
			// mMotors[7]->setPosition(0);
			// mMotors[9]->setPosition(0.008386);
			// mMotors[11]->setPosition(-0.850839);
			// mMotors[13]->setPosition(1.573324);
			// mMotors[15]->setPosition(0.722485);
			// mMotors[17]->setPosition(0.008386);
			
			// mMotors[16]->setPosition(0);
			// mMotors[14]->setPosition(0);
			// mMotors[12]->setPosition(0);
			// mMotors[10]->setPosition(0);
			// mMotors[8]->setPosition(0);
			// mMotors[6]->setPosition(0);
			// mMotors[7]->setPosition(0);
			// mMotors[9]->setPosition(0);
			// mMotors[11]->setPosition(0);
			// mMotors[13]->setPosition(0);
			// mMotors[15]->setPosition(0);
			// mMotors[17]->setPosition(0);
#endif
		
		// printf("%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", _th[0], _th[1], _th[2], _th[3], _th[4], _th[5], _th[6], _th[7], _th[8], _th[9], _th[10], _th[11]);
		
		// //printf("%f %f %f %f %f %f\n ", TLW1[0][0], TLW1[0][1], TLW1[0][2], TLW12[0][0], TLW12[0][1], TLW12[0][2]);
		// //printf("%f %f %f\n ", targetr[0][0], targetr[0][1], targetr[0][2]);
		for(int a=0;a<12;a++)
		{
			_th_printf[a]=_th[a];
		}
		printf("\n step count = %d",datacountft);
		datacountft++;	
		// printf("\n time  = %lf", walkingdatatime*samplingtime);
		walkingdatatime++;
#if debugFprintOn
		fprintf(ft,"%lf %lf %lf %lf %lf %lf\n",forceZr, forceZl,torquexr,torquexl,-torqueyr,-torqueyl);
		fprintf(rne,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			_th_printf[0], _th_printf[1], _th_printf[2], _th_printf[3], _th_printf[4],
			_th_printf[5],_th_printf[6], _th_printf[7], _th_printf[8], _th_printf[9],
			_th_printf[10],_th_printf[11],_th_encoder[0],_th_encoder[1],_th_encoder[2],
			_th_encoder[3],_th_encoder[4],_th_encoder[5],_th_encoder[6],_th_encoder[7],
			_th_encoder[8],_th_encoder[9],_th_encoder[10],_th_encoder[11],_error_th_1.x[0][0],
			_error_th_1.x[1][0],_error_th_1.x[2][0],_error_th_1.x[3][0],_error_th_1.x[4][0],_error_th_1.x[5][0],
			_error_th_1.x[6][0],_error_th_1.x[7][0],_error_th_1.x[8][0],_error_th_1.x[9][0],_error_th_1.x[10][0],
			_error_th_1.x[11][0],_error_dth_1.x[0][0],_error_dth_1.x[1][0],_error_dth_1.x[2][0],_error_dth_1.x[3][0],
			_error_dth_1.x[4][0],_error_dth_1.x[5][0],_error_dth_1.x[6][0],_error_dth_1.x[7][0],_error_dth_1.x[8][0],
			_error_dth_1.x[9][0],_error_dth_1.x[10][0],_error_dth_1.x[11][0]);
		fprintf(slipcontrol, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf\n",
			frefY_print, fslipy_print, forceYr, forceYl, local_gpsrz, pre_ycom+pre_ycom_c, ftzmpy, forceZr, forceZl, phaseflag,
			realphaseflag, ssptime, dsptime, stepcheck, sspgpsrz, sspgpsrx ,frefX_print, fslipx_print, footdegreeR,sspdegreeR,
			localFootDegree, yComVFinalPrint, none, none, _ycom_acc_measure, comDegree, footdegreeL);
		fprintf(footprint, "%lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %d\n",
			gpsrx, gpslx, gpsrz, gpslz, footradR, footradL, sspflag,local_gpsrx, local_gpsrz,footRY_acc,
			footLY_acc, footRYAccLpf, footLYAccLpf,new_phase_num);
		fprintf(cpt,"%lf %lf %d %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf\n",
		ZMP_c[0]+_px, ZMP_c[1]+_py, phaseflag, realphaseflag,CP_err[0],
		 CP_err[1], _xcom_origin, _ycom_origin, ftzmpx + _px, ftzmpy + _py,
		 _px,_py,ZMP_global_d[0],ZMP_global_d[1],ZMP_c[0],
		  ZMP_c[1],CP_cur[0],CP_cur[1],CP_cur_global_ssp[0],CP_cur_global_ssp[1],
		  _targetxcom, _targetycom,CP_ref[0],CP_ref[1],_xcom_nominal_global,
		  _ycom_nominal_global,CP_ref_global_ssp[0],CP_ref_global_ssp[1],phaseflag,ZMP_global_d[0],
		  ZMP_global_d[1],CP_cur_global_dsp[0],CP_cur_global_dsp[1],CP_ref_global_dsp[0],CP_ref_global_dsp[1],
		  inv_pos.x[0][0],inv_pos.x[1][0]);
#endif
		
		myStep();
		_t = _t + samplingtime;
	}
	fclose(footprint);
	fclose(slipcontrol);
	fclose(cpt);
	fclose(rne);
	fclose(ft);
}
void Walk::checkIfFallen() {
	static int fup = 0;
	static int fdown = 0;
	static const double acc_tolerance = 80.0;
	static const double acc_step = 100;

	// count how many steps the accelerometer
	// says that the robot is down
	const double *acc = mAccelerometer->getValues();
	if (acc[1] < 512.0 - acc_tolerance)
		fup++;
	else
		fup = 0;

	if (acc[1] > 512.0 + acc_tolerance)
		fdown++;
	else
		fdown = 0;

	// the robot face is down
	if (fup > acc_step) {
		mMotionManager->playPage(10);  //T f_up
		mMotionManager->playPage(9);   // init position
		fup = 0;
	}
	// the back face is down
	else if (fdown > acc_step) {
		mMotionManager->playPage(11);  // b_up
		mMotionManager->playPage(9);   // init position
		fdown = 0;
	}
}