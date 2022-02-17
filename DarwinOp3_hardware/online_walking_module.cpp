/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: SCH */
#include "/home/robotis/catkin_ws/src/ROBOTIS-Framework/robotis_controller/include/robotis_controller/robotis_controller.h"
#include "/home/robotis/catkin_ws/src/ROBOTIS-Framework/robotis_device/include/robotis_device/robot.h"
#include "/home/robotis/catkin_ws/src/ROBOTIS-Framework/robotis_device/include/robotis_device/dynamixel.h"
#include "dynamixel_sdk/group_bulk_read.h"
#include "dynamixel_sdk/group_sync_write.h"
#include "op3_online_walking_module/online_walking_module.h"
#include <fstream>
using namespace std;
using namespace robotis_op;
using namespace dynamixel;
using namespace robotis_framework;
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
#define debugFprintOn 1
ofstream wpg;
int samplingTime; // integer sampling time
double samplingtime;//0.008;
int samplingPeriod = samplingTime;
int inverse_type = 1; // 0 : direct inverse from C , 1 : inverse solved from Matlab
int control_on = 0; // 1 : ZMP control , 0 : nominal value
int tq_standing=0;
int tq_stand = 10000;//0;//3000;//4000/samplingTime;
//////////////////////////////////////control pannel torque
//////////////////////////////////////////////////////////
#define torqueModeOn 1
#define GravityCompensationRNE 0
#define initPoseCheck 0
#define jointCtrlOn 0
#define iCtrlOn 0
#define CTMOn 1
int present_control_mode=0;//1=torquemode 0=positionmode
double Tr=0.2;//0;//0.02;//
double Tr_walk_ssp=0.015;
double Tr_walk_dsp=0.015;
double _kv=2*(1.8/Tr);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
double _kp=(1.8/Tr)*(1.8/Tr);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
// double pointmass= 1.5;//1.0;
// double _m = 1+1.18888;//3.01;//1.1655 + pointmass; Jinu kang kang
// double _m = 1.34928+1.0502;
double _m = 1.34928+1.18888;
// double _m = 1.34928+0.23061*2+0.14807+1.18888;
int sensorOn=0;
//(=0 fz: constant)(=1 fz: constant tx,ty: measured)//(=2 fz tx ty sensorLPF live)(=3 all sensorLPF live)//(=4 fz: measured)(5=fz: measured(LPF))//(=6 fz: constant tx,ty: measured(LPF))//(=7 fz tx ty : constant)
double jointDamperGain=0.0;
double iCtrlGain=8000;
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
//cpZmpCurrentOn =0 -> zmp.cur=0 / =1 -> zmp.cur=EEfxtxtyÂ¡Ã­oÂ¨Ã¹oÂ¡Â¾aÂ©Ã¶Yzmp
double cp_cur_limit=0.007;
double cpgain_all=0.0;
double cpgain_ssp=0.0;
double Kf_D[2] = {0.0+cpgain_all,0.0+cpgain_all}; ///CP gain 
double Kf_S[2] = {0.0+cpgain_all+cpgain_ssp,0.0+cpgain_all+cpgain_ssp}; ///CP gain 
double Kf_D_origin[2]={Kf_D[0],Kf_D[1]};
double Kf_S_origin[2]= {Kf_S[0],Kf_S[1]};
int DspInitSwitch=1; 
//0=>dsp start is defined by fomula
//1=>ssp last point = dsp first point , cpref is xcom origin

double result_th[12]={0,};
double begin=0;
double end=0;
double time_duration=0;
int loopcount=0;
int phaseflag = 0, realphaseflag = 0, sspflag = 0, stepcheck=0;
double fsrtorquexr = 0, fsrtorquexl = 0, fsrtorqueyr = 0, fsrtorqueyl = 0;
double footup = 0;
int torqueStandingCount = 0;
int torqueStandCount=0;
double local_stride_F, local_stride_L;
double pre_local_stride_F, pre_local_stride_L;
double _t = 0;
double force[8], pos_x[8], pos_y[8], zmp[2], zmp_global[2];
double force_left, force_right, force_sum;
double sumofzmp_r, sumofzmp_l;
double fsr_boundary_x = 0.05; // [cm] sagittal FSR boundary
double fsr_boundary_y = 0.0265;//0.027; // [cm] lateral FSR boundary (with offset -> +3.8 -1.6)
double fsr_offset = 0.0125;//1.1; // [cm] lateral FSR offset (length between the foot base coordinate to middle of the foot)

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

//phase0 SSP_L Â¢Â¯Â¨Â­Â©Ã¶Â©Â¬ AoAo
double f_zr_p0_pos=0.0;
double tq_xr_p0_pos=0.0;
double tq_yr_p0_pos=0.0;
double f_zl_p0_pos=d_Fz_SSP;
double tq_xl_p0_pos=-0.24;
double tq_yl_p0_pos=-0.15;//-0.05;//0;//0.1;//0.1;//0;//
//phase1 SSP_R Â¢Â¯AÂ¢Â¬Â¡ÃÂ©Ã¶Â©Â¬ AoAo
double f_zr_p1_pos=d_Fz_SSP;
double tq_xr_p1_pos=0.24;
double tq_yr_p1_pos=-0.15;//-0.05;//0;//0.1;//0;//
double f_zl_p1_pos=0.0;
double tq_xl_p1_pos=0.0;
double tq_yl_p1_pos=0.0;
//phase2 DSP_L Â¢Â¯AÂ¢Â¬Â¡ÃÂ©Ã¶Â©Â¬ AoAo AIEA Â¥Ã¬IÂ©Ã¶Â©Â¬ AoAo
double f_zr_p2_pos=d_Fz_DSP;
double tq_xr_p2_pos=-0.12;
double tq_yr_p2_pos=0.08;//0.18;//
double f_zl_p2_pos=d_Fz_DSP;
double tq_xl_p2_pos=0.2;
double tq_yl_p2_pos=-0.1;//0.0;//
//phase3 DSP_R Â¢Â¯Â¨Â­Â©Ã¶Â©Â¬ AoAo AIEA Â¥Ã¬IÂ©Ã¶Â©Â¬ AoAo
double f_zr_p3_pos=d_Fz_DSP;
double tq_xr_p3_pos=-0.2;
double tq_yr_p3_pos=-0.1;//0;//
double f_zl_p3_pos=d_Fz_DSP;
double tq_xl_p3_pos=0.12;
double tq_yl_p3_pos=0.08;//0.18;//

clock_t tick1, tick2, tick3, tick4, tick5, tick6, tick7, tick8;
clock_t tock1, tock2, tock3, tock4, tock5, tock6, tock7, tock8;
float result1, result2, result3, result4, result5, result6, result7, result8;

double LF = 0.0305;//0.0265+0.004=0.0305//0.02715;//0.035;
double BX = -0.01501;//0;//+0.01;//-0.01;//-0.01;//0.01;
double BZ = 0.06582;//0.01;//0.01;
double L1 = 0.11;//0.093;
double L2 = 0.11;//0.093;
double L3 = 0.035;//0.037;
double pi = 3.141592;

double _zc = 0.27582;//0.22;//0.2;//0.09 * 2.0;//0.2;//0.098 * 2.0 ;
//0.22+0.05582=0.27582 because CoM move 0.01 to 0.06582

int step_number = 0;
int sw = 0;

enum _phase { SSP_L, SSP_R, DSP_L, DSP_R };
int phase_num = 0; // 1 : SSP , 0 : DSP
int new_phase_num = 0;
double FS = 0; // Frontal stride
double LS = 0; // Lateral stride

_phase phase;

double _px = 0;
double _py = 0;
double _ppx = 0;
double _ppy = 0;
//double _rsx_des = 0;
//double _rsy_des = 0;
//double _lsx_des = 0;
//double _lsy_des = 2 * L3;

double t = 0;
double C = 0;
double S = 0;
double px = 0;
double py = 0;
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
double sx = 0;
double sy = 0;
double sz = 0;
double theta = 0;

double Fz = 2.8 * 9.81;

double Kx_control = 0.25;
double Ky_control = 1.5;

double x_com_control = 0;
double y_com_control = 0;
double x_com_nominal = 0;
double y_com_nominal = 0;
double x_com_input = 0;
double y_com_input = 0;
double desiredCOMX = 0;
double desiredCOMY = 0;

double x_zmp_current = 0;
double y_zmp_current = 0;

double x_zmp_desired = 0;
double y_zmp_desired = 0;

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

//add 0630 281 - 890 : matrix AÂ¢Â´ACCIÂ¡Ãi Â¡ÃeÂ¡Ã­eCIÂ¡Â¤AÂ¡Ãi Â¨ÃºÂ¢Â¥ Â¡Â¾Â¢Â¬AÂ¢Ã

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
	rMatrix.x[3][0] = m1.x[3][0]*m2.x[0][0] + m1.x[3][1]*m2.x[1][0] + m1.x[3][2]*m2.x[2][0] +m1.x[3][3]*m2.x[3][0] +m1.x[3][4]*m2.x[4][0] +m1.x[3][5]*m2.x[5][0];;
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
///////////////////////////////////////////////////////////////////////add -Â©Ã¶ICI

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

//B to R AACÂ¡ÃÂ¡Ãe

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
//add Â©Ã¶ICI 917 - 982 : forward Â¢ÃÂ¡Ã Â¨ÃºÂ©Ã·Â¡Â¤AÂ¡Ãi Â¨Ã¹Â¡Â¾Â¨ÃºÂ©Â£Â¥Ã¬A AOÂ¢Â¥AÂ¡ÃCÂ¥Ã¬Â¡Ã EÂ¢Â´Â¨Ã¶A Â¨ÃºÂ©ÂªAÂ¢Â¬Â¢Â¬e AÂ©Â¬Â¡ÃÂ¢Â®CIÂ¢Â¬e Â¥Ã¬A
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
///////////////////////////////////////////////////////////////////add -Â©Ã¶ICI

//B to l AACÂ¡ÃÂ¡Ãe

double TLWE[4][4] = { 0, };
double TLW6[4][4] = { 0, };
double TLW5[4][4] = { 0, };
double TLW4[4][4] = { 0, };
double TLW3[4][4] = { 0, };
double TLW2[4][4] = { 0, };
double TLW1[4][4] = { 0, };

//add 0630 917 - 982 : forward Â¢ÃÂ¡Ã Â¨ÃºÂ©Ã·Â¡Â¤AÂ¡Ãi Â¨Ã¹Â¡Â¾Â¨ÃºÂ©Â£Â¥Ã¬A AOÂ¢Â¥AÂ¡ÃCÂ¥Ã¬Â¡Ã EÂ¢Â´Â¨Ã¶A Â¨ÃºÂ©ÂªAÂ¢Â¬Â¢Â¬e AÂ©Â¬Â¡ÃÂ¢Â®CIÂ¢Â¬e Â¥Ã¬A
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

void minor1(double b[6][6], double a[6][6], int i, int n) {
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
			minor1(b, a, i, n);   // read function
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

//Â¨ÃºOÂ¢Â¯Â¢Â®Â©Ã·Â¡Â§Â¢Â¥A AÂ¨ÃÂ¡Â¾aÂ¡ÃÂ¢Â® 4*4AÂ¡ÃÂ¢Â¬Â¢Ã§Â¢Â¬Â¢Â¬ Â¡ÃÂ¢Â®Â¢Â¥E
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
	//add 0630 1325 - 1647 : forward Â¥Ã¬Â¥Ã¬ Â¨ÃºÂ¨Â¡Â¢Â¬Â¢Ã AÂ©ÂªÂ¢Â¯iÂ¡ÃÂ¢Â® Â¡ÃÂ¢Â®AoÂ¡Ãi AOÂ¢Â¥A Â¡Â¾aAÂ¢Â¬ Â¡Â¾Â¢Â¬Co Â©Ã¶Â©Â¡Â©Ã¶yAIÂ¢Ãu Â¢Â¥Â¨Â­Â¢ÃoAÂ©Ã¸AÂ¡Ã­ Â¡ÃI Â¡ÃÂ¡ÃÂ¨ÃºÂ¨Â¡
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

	
	///////////////////////////////////////////////////////////////////////////////////////
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
///////////////////INVERSE  AACÂ¡ÃÂ¡Ãe///////////////


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

//B to l AACÂ¡ÃÂ¡Ãe
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
	ITRB1[2][3] = 0;//LF;

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
	ITR6E[0][3] = BX;     //AcEÂ¢Ã§AI

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
	ITLB1[2][3] = 0;//LF;

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




//////////////////////2860 - 3124 : Â¡ÃÂ¡ÃÂ¡ÃÂ¢Â®Â¨Ã¹OÂ¥Ã¬Â¥Ã¬ Â¡Â¾Â¢Â¬CIÂ¢Â¥A Â¡ÃI Â¢ÃÂ¡ÃÂ¢Â¬A AUAUÂ¨Â¬nÂ¨ÃºEÂ¢Â¬Â¢Â¬ Â¡Â¾Â¢Â¬CIÂ¢Â¥A COÂ¨Ã¹o Â¡Ã­oÂ¡Â¤I Â¡Â¾Â¢Â¬Co


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

//printf("fsr_boundary_x : %lf , FS : %lf\n", fsr_boundary_x, FS);

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
	int fslipAddFoot=1, sspStartInit=0;//1AIÂ¢Â¬e AEÂ¡Â¾aEÂ¡Â©
	int onlySSPon=0;
	double onlySSPTime=0.6;
	int LPFSwitchDsp=0, LPFSwitchSsp=0, LPFSwitchFootDsp=0;
	int dspFootLPFOn=6, dspFslipLPFOn=6;
////////////////////////////control pannel//////////////
    int fslipInitCheck=0;
	double fslipy_print=0, fslipx_print=0;
    double frefY_print=0,frefX_print=0;
	int DSPDelCheck=0; 
	double fslipy = 0, fslipx = 0;
	double pre_fslipy=0, pre_fslipx=0;
	double pre2_fslipy=0, pre2_fslipx=0;
	double fslipyLpf=0, fslipxLpf=0;
	double fslipyLpfPrint=0, fslipxLpfPrint=0;
	double pre_fslipyLpf=0, pre_fslipxLpf=0;
	double pre2_fslipyLpf=0, pre2_fslipxLpf=0;
	double footLYAccLpf=0, footRYAccLpf=0;
	double pre_footLYAccLpf=0,pre_footRYAccLpf=0;
	double pre_footLY_acc=0, pre_footRY_acc=0;
	double lamda, lamdaFoot;
	double alpha, alphaFoot;
	double fc=3, fcFoot=10, Qfactor=0.707;
	
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
	double pre_xcom_v;
	double pre_ycom_v;
	double pre_xcom_acc;
	double pre_ycom_acc;

	/////////////////////for cpt/////////////////////
	double pre_ssp_cp[2] ={0,};       //global CP    

	double CP_global[2] ={0,};       //global CP    
	double CP_global_dsp[2]={0,};
	double CP_global_ref[2] ={0,};       //global CP   
	double CP_global_ref_dsp[2]={0,}; 
	double CP_c[2]={0,};            //current local CP
	double CP_ref[2]={0,};            //reference local CP
	double CP_d[2]={0,};            //desired local CP
	double CP_init[2]={0,};            // initial local CP
	double CP_sspinit[2]={0,};            //ssp initial local CP
	double CP_err[2]={0,};
	double CP_E[2] = { 0, };           //Encoder local CP

	double ZMP_c[2]={0,};           //current local ZMP
	double ZMP_d[2]={0,};           //desired local ZMP
	double ZMP_global[2]={0,};      //current global ZMP
	double ZMP_global_d[2]={0,};      //current global ZMP

	double p_sspinit[2]={0,};       //ssp initial position
	double v_sspinit[2]={0,};       //ssp initial velocity

	double p_current[2]={0,};       //local current position, final COM
	double v_current[2]={0,};       //local current velocity

	double p_nominal[2]={0,};       //local nominal position, final COM
	double v_nominal[2]={0,};       //local nominal velocity

	double next_p_nominal[2]={0,};       //local nominal position, final COM
	double next_v_nominal[2]={0,};       //local nominal velocity

	double COM_c[2]={0,};
	double COM_n[2]={0,};

	double p_control[2]={0,};       //local control position
	double v_control[2]={0,};       //local control velocity

	double next_p_control[2]={0,};       //local control position(t+dt)
	double next_v_control[2]={0,};       //local control velocity(t+dt)

	double global_next_p_control[2]={0,};       //global control position(t+dt)

	double dp_control[2]={0,};       //delta control position
	double dv_control[2]={0,}; //delta control velocity

	double next_dp_control[2]={0,};       //global delta control position
	double next_dv_control[2] = {0,0}; //delta control velocity

	double p_desired[2]={0,};       //desired position
	double v_desired[2]={0,};       //desired velocity
	double a_desired[2]={0,};       //desired acc

	double localzmpx, localzmpy;

	double _vxcom;
	double _vycom;

	double CP_DSP_init[2] ;
	double CP_DSP_fin[2] ;

	double CP_SSP_fin[2] ;

	double d_a0[2];
	double d_a1[2];
	double d_a2[2];
	double d_a3[2];

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
	double cpOffsetX=0, cpOffsetY=0;
	double cpDspXF=0, cpDspYF=0;
	double cpDspTX=0, cpDspTY=0;
	double cpDspVX=0, cpDspVY=0;
	double targetXDsp=0, targetYDsp=0;
	double _xcom_cd = 0;
	double _ycom_cd = 0;
	double _ycom_cdv = 0, _xcom_cdv=0;
	double pre_ycom_cd = 0, pre_xcom_cd=0;
	double pre_ycom_cdv = 0, pre_xcom_cdv=0;

	double _targetxcom;
	double _targetycom;
	double _xcom;
	double _ycom;
	double _xcom_nominal;
	double _ycom_nominal;
	double _xcom_global_nominal;
	double _ycom_global_nominal;
	double _xcom_v = 0;
	double _ycom_v = 0;
	double _xcom_v_nominal = 0;
	double _ycom_v_nominal = 0;
	double _xcom_acc = 0;
	double _ycom_acc = 0;


	double forceZr;
	double forceZl;
	double forceXr;
	double forceXl;
	double forceYr;
	double forceYl;
	//double ftzmpxr = 0, ftzmpyr = 0, ftzmpxl = 0, ftzmpyl = 0, ftzmpx = 0, ftzmpy = 0;
	//double fgroundX = 0, fgroundY = 0, fswitchZ = 0;

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
  	double _th[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_i[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_interpolation[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_position[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_torque[12] = {-0.006594, 0.909073, -1.619199, 0.710126, 0.006594, 0.000000, -0.000000, -0.006594, -0.710126, 1.619199, -0.909073, 0.006594};
	double _th_printf[12]={0,};
	int _i=0;
	int _j;
	int _sn=0;
	int _pi;
	double _tempp[3] = { 0, };
	double _tempr[3][3] = { 0, };

	double _g = 9.81;
	double _Tc = sqrt(_zc / _g);
	double dsptime = 0.8;
	double ssptime = 0.8;
	
	double stride_beg = 0.06;
	double stride_mid = 0.06;
	double stride_end = 0.06;
	double stridey_beg = 0.00;
	double stridey_mid = 0.00;
	double stridey_end = 0.00;
  double _s[100][7] = {//footstep command
		{ 0,    0,  0,0,0,0,0 },
	{ 0,    0,  0,0,0,0,0 },
	{ 0   , L3 * 2, 0,  0,0,0,0 },
	{ stride_beg, L3 * 2+stridey_beg, ssptime,0.2,0*pi / 180,0,0 },//I=3
	{ stride_beg, L3 * 2+stridey_beg, ssptime,dsptime,0 * pi / 180,0,0 },//I=4
  
	{ stride_beg, L3 * 2+stridey_beg, ssptime,dsptime,0 * pi / 180,0,0 },//I=5
	{ stride_mid, L3 * 2+stridey_mid, ssptime,dsptime, 0* pi / 180,0,0 },//I=6
	{ stride_mid, L3 * 2+stridey_mid, ssptime,dsptime,0 * pi / 180,0,0 },//Â¢Â¯Â¨ÃÂ¡Â¾aÂ¨Ã¹Â¡Â©Â¨Â¬IAIdsptime
	{ stride_mid, L3 * 2+stridey_mid, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
  /*
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },//Â¢Â¯Â¨ÃÂ¡Â¾aÂ¨Â¬IAI
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
	{ stride_end, L3 * 2+stridey_end, ssptime,dsptime,0 * pi / 180,0,0 },
  */
	{ 0   , L3 * 2, ssptime,dsptime,0 * pi / 180,0,0 },
	{ 0   , 0     ,   0,dsptime,0,0,0 } };
	int _step_length = 10;//11
	double _stepheight = 0.02;
	double _swingtime = 0;
	double sspf_coeff = 0.5;//34,66//3 7 0.375
	double dspf_coeff = 0.5;
/*	if(onlySSPon==1)
	{
		sspf_coeff=0.5;
		dspf_coeff=0.5;
	}
	*/
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

	double _sx = 0;
	double _sy = 0;
	double _sz = 0;
	double _theta = 0;

	double _footsize_sf = 0.04;
	double _footsize_sl = 0.04;
	double _footsize_l1 = 0.038;
	double _footsize_l2 = 0.016;

	double dsp_x_i;
	double dsp_x_f; 
	double dsp_dx_i;
	double dsp_dx_f;
	double dsp_x_a0;
	double dsp_x_a1;
	double dsp_x_a2;
	double dsp_x_a3;

	double ndsp_x_a0;
	double ndsp_x_a1;
	double ndsp_x_a2;
	double ndsp_x_a3;
    double ndsp_x_a4;
    double ndsp_x_a5;

	double dsp_y_i;
	double dsp_y_f;
	double dsp_dy_i;
	double dsp_dy_f;
	double dsp_y_a0;
	double dsp_y_a1;
	double dsp_y_a2;
	double dsp_y_a3;

	double ndsp_y_a0;
	double ndsp_y_a1;
	double ndsp_y_a2;
	double ndsp_y_a3;
    double ndsp_y_a4;
    double ndsp_y_a5;


	double dsp_x_i_nominal;
	double dsp_x_f_nominal; 
	double dsp_dx_i_nominal;
	double dsp_dx_f_nominal;
	double dsp_x_a0_nominal;
	double dsp_x_a1_nominal;
	double dsp_x_a2_nominal;
	double dsp_x_a3_nominal;
	double dsp_y_i_nominal;
	double dsp_y_f_nominal;
	double dsp_dy_i_nominal;
	double dsp_dy_f_nominal;
	double dsp_y_a0_nominal;
	double dsp_y_a1_nominal;
	double dsp_y_a2_nominal;
	double dsp_y_a3_nominal;

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
	double _swingrot = 0;
	double _baserot = 0;
	double _swingrot_time = 0;
	double _baserot_time = 0;

	

OnlineWalkingModule::OnlineWalkingModule()
  : control_cycle_sec_(0.004),
    is_moving_(false),
    is_balancing_(false),
    is_offset_updating_(false),
    goal_initialize_(false),
    balance_control_initialize_(false),
    body_offset_initialize_(false),
    joint_control_initialize_(false),
    wholebody_initialize_(false),
    walking_initialize_(false),
    is_foot_step_2d_(false),
    walking_phase_(DSP),
    total_mass_(3.5),
    foot_distance_(0.07)
{
  enable_       = false;
  module_name_  = "online_walking_module";
//    control_mode_ = robotis_framework::PositionControl;
  control_mode_ = robotis_framework::TorqueControl;
  control_type_ = NONE;
  balance_type_ = OFF;

  ROS_INFO("online_walking_module");

  op3_kdl_ = new OP3Kinematics();
	_t=samplingtime;
  /* leg */
  result_["r_hip_yaw"]    = new robotis_framework::DynamixelState();
  result_["r_hip_roll"]   = new robotis_framework::DynamixelState();
  result_["r_hip_pitch"]  = new robotis_framework::DynamixelState();
  result_["r_knee"]       = new robotis_framework::DynamixelState();
  result_["r_ank_pitch"]  = new robotis_framework::DynamixelState();
  result_["r_ank_roll"]   = new robotis_framework::DynamixelState();
  result_["l_hip_yaw"]    = new robotis_framework::DynamixelState();
  result_["l_hip_roll"]   = new robotis_framework::DynamixelState();
  result_["l_hip_pitch"]  = new robotis_framework::DynamixelState();
  result_["l_knee"]       = new robotis_framework::DynamixelState();
  result_["l_ank_pitch"]  = new robotis_framework::DynamixelState();
  result_["l_ank_roll"]   = new robotis_framework::DynamixelState();

  /* leg */
  joint_name_to_id_["r_hip_yaw"]    = 1;
  joint_name_to_id_["l_hip_yaw"]    = 2;
  joint_name_to_id_["r_hip_roll"]   = 3;
  joint_name_to_id_["l_hip_roll"]   = 4;
  joint_name_to_id_["r_hip_pitch"]  = 5;
  joint_name_to_id_["l_hip_pitch"]  = 6;
  joint_name_to_id_["r_knee"]       = 7;
  joint_name_to_id_["l_knee"]       = 8;
  joint_name_to_id_["r_ank_pitch"]  = 9;
  joint_name_to_id_["l_ank_pitch"]  = 10;
  joint_name_to_id_["r_ank_roll"]   = 11;
  joint_name_to_id_["l_ank_roll"]   = 12;

  /* parameter */
   
 samplingtime = control_cycle_sec_;//0.008;
 samplingTime = control_cycle_sec_*1000; // integer sampling time
 samplingPeriod = control_cycle_sec_*1000;
 number_of_joints_ = 12;

  curr_joint_accel_.resize(number_of_joints_, 0.0);
  curr_joint_vel_.resize(number_of_joints_, 0.0);
  curr_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_accel_.resize(number_of_joints_, 0.0);
  des_joint_vel_.resize(number_of_joints_, 0.0);
  des_joint_pos_.resize(number_of_joints_, 0.0);
  des_joint_torque_.resize(number_of_joints_,0.0);

  goal_joint_accel_.resize(number_of_joints_, 0.0);
  goal_joint_vel_.resize(number_of_joints_, 0.0);
  goal_joint_pos_.resize(number_of_joints_, 0.0);

  des_joint_feedback_.resize(number_of_joints_, 0.0);
  des_joint_feedforward_.resize(number_of_joints_, 0.0);
  des_joint_pos_to_robot_.resize(number_of_joints_, 0.0);

  joint_feedforward_gain_.resize(number_of_joints_, 0.0);

  // body position default
  des_body_pos_.resize(3, 0.0);
  des_body_vel_.resize(3, 0.0);
  des_body_accel_.resize(3, 0.0);
  des_body_Q_.resize(4, 0.0);

  // left foot position default
  des_l_leg_pos_.resize(3, 0.0);
  des_l_leg_vel_.resize(3, 0.0);
  des_l_leg_accel_.resize(3, 0.0);
  des_l_leg_Q_.resize(4, 0.0);

  // right foot position default
  des_r_leg_pos_.resize(3, 0.0);
  des_r_leg_vel_.resize(3, 0.0);
  des_r_leg_accel_.resize(3, 0.0);
  des_r_leg_Q_.resize(4, 0.0);

  x_lipm_.resize(3, 0.0);
  y_lipm_.resize(3, 0.0);

  resetBodyPose();

  // walking parameter default
  walking_param_.dsp_ratio = 0.2;
  walking_param_.lipm_height = 0.12;
  walking_param_.foot_height_max = 0.05;
  walking_param_.zmp_offset_x = 0.0; // not applied
  walking_param_.zmp_offset_y = 0.0;

  des_balance_gain_ratio_.resize(1, 0.0);
  goal_balance_gain_ratio_.resize(1, 0.0);

  balance_control_.initialize(control_cycle_sec_*1000.0);
  balance_control_.setGyroBalanceEnable(false); // Gyro
  balance_control_.setOrientationBalanceEnable(false); // IMU
  balance_control_.setForceTorqueBalanceEnable(false); // FT

  balance_l_foot_force_x_   = 0.0;
  balance_l_foot_force_y_   = 0.0;
  balance_l_foot_force_z_   = 0.0;
  balance_l_foot_torque_x_  = 0.0;
  balance_l_foot_torque_y_  = 0.0;
  balance_l_foot_torque_z_  = 0.0;

  balance_r_foot_force_x_   = 0.0;
  balance_r_foot_force_y_   = 0.0;
  balance_r_foot_force_z_   = 0.0;
  balance_r_foot_torque_x_  = 0.0;
  balance_r_foot_torque_y_  = 0.0;
  balance_r_foot_torque_z_  = 0.0;

  // Body Offset
  des_body_offset_.resize(3, 0.0);
  goal_body_offset_.resize(3, 0.0);

  std::string balance_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  std::string joint_feedback_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedback_gain.yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  std::string joint_feedforward_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedforward_gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);
  
}

OnlineWalkingModule::~OnlineWalkingModule()
{
  queue_thread_.join();
}

void OnlineWalkingModule::initialize(const int control_cycle_msec, robotis_framework::Robot *robot)
{
  control_cycle_sec_ = control_cycle_msec * 0.001;
  queue_thread_      = boost::thread(boost::bind(&OnlineWalkingModule::queueThread, this));

  ros::NodeHandle ros_node;

  // Publisher
  status_msg_pub_       = ros_node.advertise<robotis_controller_msgs::StatusMsg>("/robotis/status", 1);
  online_walking_param_pub_		= ros_node.advertise<robotis_controller_msgs::OnlineWalkingParam>("/robotis/online_param",1);
  movement_done_pub_    = ros_node.advertise<std_msgs::String>("/robotis/movement_done", 1);
  goal_joint_state_pub_ = ros_node.advertise<sensor_msgs::JointState>("/robotis/online_walking/goal_joint_states", 1);
  pelvis_pose_pub_      = ros_node.advertise<geometry_msgs::PoseStamped>("/robotis/pelvis_pose", 1);

  // Service
//  get_preview_matrix_client_ = ros_node.serviceClient<op3_online_walking_module_msgs::GetPreviewMatrix>("/robotis/online_walking/get_preview_matrix", 0);
}

void OnlineWalkingModule::queueThread()
{
  ros::NodeHandle     ros_node;
  ros::CallbackQueue  callback_queue;

  ros_node.setCallbackQueue(&callback_queue);

  // Subscriber
  ros::Subscriber reset_body_sub_ = ros_node.subscribe("/robotis/online_walking/reset_body", 5,
                                                       &OnlineWalkingModule::setResetBodyCallback, this);
  ros::Subscriber joint_pose_sub_ = ros_node.subscribe("/robotis/online_walking/goal_joint_pose", 5,
                                                       &OnlineWalkingModule::goalJointPoseCallback, this);
  ros::Subscriber kinematics_pose_sub_ = ros_node.subscribe("/robotis/online_walking/goal_kinematics_pose", 5,
                                                            &OnlineWalkingModule::goalKinematicsPoseCallback, this);
  ros::Subscriber foot_step_command_sub_ = ros_node.subscribe("/robotis/online_walking/foot_step_command", 5,
                                                              &OnlineWalkingModule::footStepCommandCallback, this);
  ros::Subscriber walking_param_sub_ = ros_node.subscribe("/robotis/online_walking/walking_param", 5,
                                                          &OnlineWalkingModule::walkingParamCallback, this);
  ros::Subscriber wholebody_balance_msg_sub = ros_node.subscribe("/robotis/online_walking/wholebody_balance_msg", 5,
                                                                 &OnlineWalkingModule::setWholebodyBalanceMsgCallback, this);
  ros::Subscriber body_offset_msg_sub = ros_node.subscribe("/robotis/online_walking/body_offset", 5,
                                                           &OnlineWalkingModule::setBodyOffsetCallback, this);
  ros::Subscriber foot_distance_msg_sub = ros_node.subscribe("/robotis/online_walking/foot_distance", 5,
                                                             &OnlineWalkingModule::setFootDistanceCallback, this);

  ros::Subscriber footsteps_sub = ros_node.subscribe("/robotis/online_walking/footsteps_2d", 5,
                                                     &OnlineWalkingModule::footStep2DCallback, this);

//  ros::Subscriber imu_data_sub = ros_node.subscribe("/robotis/sensor/imu/imu", 5,
//                                                    &OnlineWalkingModule::imuDataCallback, this);
//  ros::Subscriber l_foot_ft_sub = ros_node.subscribe("/robotis/sensor/l_foot_ft", 3,
//                                                     &OnlineWalkingModule::leftFootForceTorqueOutputCallback, this);
//  ros::Subscriber r_foot_ft_sub = ros_node.subscribe("/robotis/sensor/r_foot_ft", 3,
//                                                     &OnlineWalkingModule::rightFootForceTorqueOutputCallback, this);

  // Service
  ros::ServiceServer get_joint_pose_server = ros_node.advertiseService("/robotis/online_walking/get_joint_pose",
                                                                       &OnlineWalkingModule::getJointPoseCallback, this);
  ros::ServiceServer get_kinematics_pose_server = ros_node.advertiseService("/robotis/online_walking/get_kinematics_pose",
                                                                            &OnlineWalkingModule::getKinematicsPoseCallback, this);

  ros::WallDuration duration(control_cycle_sec_);
  while(ros_node.ok())
    callback_queue.callAvailable(duration);
}

void OnlineWalkingModule::resetBodyPose()
{
  des_body_pos_[0] = 0.0;
  des_body_pos_[1] = 0.0;
  des_body_pos_[2] = 0.3402256;

  des_body_Q_[0] = 0.0;
  des_body_Q_[1] = 0.0;
  des_body_Q_[2] = 0.0;
  des_body_Q_[3] = 1.0;

  des_r_leg_pos_[0] = 0.0;
  des_r_leg_pos_[1] = -0.5 * foot_distance_; //-0.045; //-0.035;
  des_r_leg_pos_[2] = 0.0;

  des_r_leg_Q_[0] = 0.0;
  des_r_leg_Q_[1] = 0.0;
  des_r_leg_Q_[2] = 0.0;
  des_r_leg_Q_[3] = 1.0;

  des_l_leg_pos_[0] = 0.0;
  des_l_leg_pos_[1] = 0.5 * foot_distance_; //0.045; //0.035;
  des_l_leg_pos_[2] = 0.0;

  des_l_leg_Q_[0] = 0.0;
  des_l_leg_Q_[1] = 0.0;
  des_l_leg_Q_[2] = 0.0;
  des_l_leg_Q_[3] = 1.0;

  x_lipm_[0] = des_body_pos_[0];
  x_lipm_[1] = 0.0;
  x_lipm_[2] = 0.0;

  y_lipm_[0] = des_body_pos_[1];
  y_lipm_[1] = 0.0;
  y_lipm_[2] = 0.0;

  walking_param_.zmp_offset_x = des_body_pos_[0];
}

void OnlineWalkingModule::parseBalanceGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  //  ROS_INFO("Parse Balance Gain Data");

  foot_roll_gyro_p_gain_  = doc["foot_roll_gyro_p_gain"].as<double>();
  foot_roll_gyro_d_gain_  = doc["foot_roll_gyro_d_gain"].as<double>();
  foot_pitch_gyro_p_gain_ = doc["foot_pitch_gyro_p_gain"].as<double>();
  foot_pitch_gyro_d_gain_ = doc["foot_pitch_gyro_d_gain"].as<double>();

  foot_roll_angle_p_gain_   = doc["foot_roll_angle_p_gain"].as<double>();
  foot_roll_angle_d_gain_   = doc["foot_roll_angle_d_gain"].as<double>();
  foot_pitch_angle_p_gain_  = doc["foot_pitch_angle_p_gain"].as<double>();
  foot_pitch_angle_d_gain_  = doc["foot_pitch_angle_d_gain"].as<double>();

  foot_x_force_p_gain_ = doc["foot_x_force_p_gain"].as<double>();
  foot_x_force_d_gain_ = doc["foot_x_force_d_gain"].as<double>();
  foot_y_force_p_gain_ = doc["foot_y_force_p_gain"].as<double>();
  foot_y_force_d_gain_ = doc["foot_y_force_d_gain"].as<double>();
  foot_z_force_p_gain_ = doc["foot_z_force_p_gain"].as<double>();
  foot_z_force_d_gain_ = doc["foot_z_force_d_gain"].as<double>();

  foot_roll_torque_p_gain_  = doc["foot_roll_torque_p_gain"].as<double>();
  foot_roll_torque_d_gain_  = doc["foot_roll_torque_d_gain"].as<double>();
  foot_pitch_torque_p_gain_ = doc["foot_pitch_torque_p_gain"].as<double>();
  foot_pitch_torque_d_gain_ = doc["foot_pitch_torque_d_gain"].as<double>();

  roll_gyro_cut_off_frequency_  = doc["roll_gyro_cut_off_frequency"].as<double>();
  pitch_gyro_cut_off_frequency_ = doc["pitch_gyro_cut_off_frequency"].as<double>();

  roll_angle_cut_off_frequency_   = doc["roll_angle_cut_off_frequency"].as<double>();
  pitch_angle_cut_off_frequency_  = doc["pitch_angle_cut_off_frequency"].as<double>();

  foot_x_force_cut_off_frequency_ = doc["foot_x_force_cut_off_frequency"].as<double>();
  foot_y_force_cut_off_frequency_ = doc["foot_y_force_cut_off_frequency"].as<double>();
  foot_z_force_cut_off_frequency_ = doc["foot_z_force_cut_off_frequency"].as<double>();

  foot_roll_torque_cut_off_frequency_   = doc["foot_roll_torque_cut_off_frequency"].as<double>();
  foot_pitch_torque_cut_off_frequency_  = doc["foot_pitch_torque_cut_off_frequency"].as<double>();

  balance_hip_roll_gain_    = doc["balance_hip_roll_gain"].as<double>();
  balance_knee_gain_        = doc["balance_knee_gain"].as<double>();
  balance_ankle_roll_gain_  = doc["balance_ankle_roll_gain"].as<double>();
  balance_ankle_pitch_gain_ = doc["balance_ankle_pitch_gain"].as<double>();
}

void OnlineWalkingModule::parseJointFeedbackGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  joint_feedback_[joint_name_to_id_["r_hip_yaw"]-1].p_gain_     = doc["r_hip_yaw_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_yaw"]-1].d_gain_     = doc["r_hip_yaw_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_roll"]-1].p_gain_    = doc["r_hip_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_roll"]-1].d_gain_    = doc["r_hip_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_pitch"]-1].p_gain_   = doc["r_hip_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_hip_pitch"]-1].d_gain_   = doc["r_hip_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_knee"]-1].p_gain_        = doc["r_knee_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_knee"]-1].d_gain_        = doc["r_knee_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_pitch"]-1].p_gain_   = doc["r_ank_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_pitch"]-1].d_gain_   = doc["r_ank_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_roll"]-1].p_gain_    = doc["r_ank_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["r_ank_roll"]-1].d_gain_    = doc["r_ank_roll_d_gain"].as<double>();

  joint_feedback_[joint_name_to_id_["l_hip_yaw"]-1].p_gain_     = doc["l_hip_yaw_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_yaw"]-1].d_gain_     = doc["l_hip_yaw_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_roll"]-1].p_gain_    = doc["l_hip_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_roll"]-1].d_gain_    = doc["l_hip_roll_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_pitch"]-1].p_gain_   = doc["l_hip_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_hip_pitch"]-1].d_gain_   = doc["l_hip_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_knee"]-1].p_gain_        = doc["l_knee_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_knee"]-1].d_gain_        = doc["l_knee_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_pitch"]-1].p_gain_   = doc["l_ank_pitch_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_pitch"]-1].d_gain_   = doc["l_ank_pitch_d_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_roll"]-1].p_gain_    = doc["l_ank_roll_p_gain"].as<double>();
  joint_feedback_[joint_name_to_id_["l_ank_roll"]-1].d_gain_    = doc["l_ank_roll_d_gain"].as<double>();
}

void OnlineWalkingModule::parseJointFeedforwardGainData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  joint_feedforward_gain_[joint_name_to_id_["r_hip_yaw"]-1]   = doc["r_hip_yaw_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_hip_roll"]-1]  = doc["r_hip_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_hip_pitch"]-1] = doc["r_hip_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_knee"]-1]      = doc["r_knee_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_ank_pitch"]-1] = doc["r_ank_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["r_ank_roll"]-1]  = doc["r_ank_roll_gain"].as<double>();

  joint_feedforward_gain_[joint_name_to_id_["l_hip_yaw"]-1]   = doc["l_hip_yaw_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_hip_roll"]-1]  = doc["l_hip_roll_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_hip_pitch"]-1] = doc["l_hip_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_knee"]-1]      = doc["l_knee_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_ank_pitch"]-1] = doc["l_ank_pitch_gain"].as<double>();
  joint_feedforward_gain_[joint_name_to_id_["l_ank_roll"]-1]  = doc["l_ank_roll_gain"].as<double>();
}

void OnlineWalkingModule::setWholebodyBalanceMsgCallback(const std_msgs::String::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  std::string balance_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/balance_gain.yaml";
  parseBalanceGainData(balance_gain_path);

  std::string joint_feedback_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedback_gain.yaml";
  parseJointFeedbackGainData(joint_feedback_gain_path);

  std::string joint_feedforward_gain_path = ros::package::getPath("op3_online_walking_module") + "/config/joint_feedforward_gain.yaml";
  parseJointFeedforwardGainData(joint_feedforward_gain_path);

  if (msg->data == "balance_on")
    goal_balance_gain_ratio_[0] = 1.0;
  else if(msg->data == "balance_off")
    goal_balance_gain_ratio_[0] = 0.0;

  balance_control_initialize_ = false;
  balance_type_ = ON;
  walking_phase_ = DSP;
  
}

void OnlineWalkingModule::initBalanceControl()
{
  if (balance_control_initialize_ == true)
    return;

  balance_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = 1.0;

  balance_step_ = 0;
  balance_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> balance_zero;
  balance_zero.resize(1, 0.0);

  balance_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_balance_gain_ratio_, balance_zero, balance_zero,
                                         goal_balance_gain_ratio_, balance_zero, balance_zero);

  if (is_balancing_ == true)
    ROS_INFO("[UPDATE] Balance Gain");
  else
  {
    is_balancing_ = true;
    ROS_INFO("[START] Balance Gain");
  }
}

void OnlineWalkingModule::calcBalanceControl()
{
  if (is_balancing_ == true)
  {
    double cur_time = (double) balance_step_ * control_cycle_sec_;
    des_balance_gain_ratio_ = balance_tra_->getPosition(cur_time);

    if (balance_step_ == balance_size_-1)
    {
      balance_step_ = 0;
      is_balancing_ = false;
      delete balance_tra_;

      if (des_balance_gain_ratio_[0] == 0.0)
      {
        control_type_ = NONE;
        balance_type_ = OFF;
      }

      ROS_INFO("[END] Balance Gain");
    }
    else
      balance_step_++;
  }
}

void OnlineWalkingModule::imuDataCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imu_data_mutex_lock_.lock();

  imu_data_msg_ = *msg;

  imu_data_msg_.angular_velocity.x *= -1.0;
  imu_data_msg_.angular_velocity.y *= -1.0;

  imu_data_mutex_lock_.unlock();
}

void OnlineWalkingModule::leftFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3,1);
  force.coeffRef(0,0) = msg->wrench.force.x;
  force.coeffRef(1,0) = msg->wrench.force.y;
  force.coeffRef(2,0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3,1);
  torque.coeffRef(0,0) = msg->wrench.torque.x;
  torque.coeffRef(1,0) = msg->wrench.torque.y;
  torque.coeffRef(2,0) = msg->wrench.torque.z;

  Eigen::MatrixXd force_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*force;
  Eigen::MatrixXd torque_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*torque;

  double l_foot_fx_N  = force_new.coeff(0,0);
  double l_foot_fy_N  = force_new.coeff(1,0);
  double l_foot_fz_N  = force_new.coeff(2,0);
  double l_foot_Tx_Nm = torque_new.coeff(0,0);
  double l_foot_Ty_Nm = torque_new.coeff(1,0);
  double l_foot_Tz_Nm = torque_new.coeff(2,0);


  l_foot_fx_N = robotis_framework::sign(l_foot_fx_N) * fmin( fabs(l_foot_fx_N), 2000.0);
  l_foot_fy_N = robotis_framework::sign(l_foot_fy_N) * fmin( fabs(l_foot_fy_N), 2000.0);
  l_foot_fz_N = robotis_framework::sign(l_foot_fz_N) * fmin( fabs(l_foot_fz_N), 2000.0);
  l_foot_Tx_Nm = robotis_framework::sign(l_foot_Tx_Nm) * fmin(fabs(l_foot_Tx_Nm), 300.0);
  l_foot_Ty_Nm = robotis_framework::sign(l_foot_Ty_Nm) * fmin(fabs(l_foot_Ty_Nm), 300.0);
  l_foot_Tz_Nm = robotis_framework::sign(l_foot_Tz_Nm) * fmin(fabs(l_foot_Tz_Nm), 300.0);

  l_foot_ft_data_msg_.force.x = l_foot_fx_N;
  l_foot_ft_data_msg_.force.y = l_foot_fy_N;
  l_foot_ft_data_msg_.force.z = l_foot_fz_N;
  l_foot_ft_data_msg_.torque.x = l_foot_Tx_Nm;
  l_foot_ft_data_msg_.torque.y = l_foot_Ty_Nm;
  l_foot_ft_data_msg_.torque.z = l_foot_Tz_Nm;
}

void OnlineWalkingModule::rightFootForceTorqueOutputCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  Eigen::MatrixXd force = Eigen::MatrixXd::Zero(3,1);
  force.coeffRef(0,0) = msg->wrench.force.x;
  force.coeffRef(1,0) = msg->wrench.force.y;
  force.coeffRef(2,0) = msg->wrench.force.z;

  Eigen::MatrixXd torque = Eigen::MatrixXd::Zero(3,1);
  torque.coeffRef(0,0) = msg->wrench.torque.x;
  torque.coeffRef(1,0) = msg->wrench.torque.y;
  torque.coeffRef(2,0) = msg->wrench.torque.z;

  Eigen::MatrixXd force_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*force;
  Eigen::MatrixXd torque_new = robotis_framework::getRotationX(M_PI)*robotis_framework::getRotationZ(-0.5*M_PI)*torque;

  double r_foot_fx_N  = force_new.coeff(0,0);
  double r_foot_fy_N  = force_new.coeff(1,0);
  double r_foot_fz_N  = force_new.coeff(2,0);
  double r_foot_Tx_Nm = torque_new.coeff(0,0);
  double r_foot_Ty_Nm = torque_new.coeff(1,0);
  double r_foot_Tz_Nm = torque_new.coeff(2,0);

  r_foot_fx_N = robotis_framework::sign(r_foot_fx_N) * fmin( fabs(r_foot_fx_N), 2000.0);
  r_foot_fy_N = robotis_framework::sign(r_foot_fy_N) * fmin( fabs(r_foot_fy_N), 2000.0);
  r_foot_fz_N = robotis_framework::sign(r_foot_fz_N) * fmin( fabs(r_foot_fz_N), 2000.0);
  r_foot_Tx_Nm = robotis_framework::sign(r_foot_Tx_Nm) *fmin(fabs(r_foot_Tx_Nm), 300.0);
  r_foot_Ty_Nm = robotis_framework::sign(r_foot_Ty_Nm) *fmin(fabs(r_foot_Ty_Nm), 300.0);
  r_foot_Tz_Nm = robotis_framework::sign(r_foot_Tz_Nm) *fmin(fabs(r_foot_Tz_Nm), 300.0);

  r_foot_ft_data_msg_.force.x = r_foot_fx_N;
  r_foot_ft_data_msg_.force.y = r_foot_fy_N;
  r_foot_ft_data_msg_.force.z = r_foot_fz_N;
  r_foot_ft_data_msg_.torque.x = r_foot_Tx_Nm;
  r_foot_ft_data_msg_.torque.y = r_foot_Ty_Nm;
  r_foot_ft_data_msg_.torque.z = r_foot_Tz_Nm;
}

void OnlineWalkingModule::setResetBodyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == true)
  {
    des_body_offset_[0] = 0.0;
    des_body_offset_[1] = 0.0;
    des_body_offset_[2] = 0.0;

    resetBodyPose();
  }
}

void OnlineWalkingModule::walkingParamCallback(const op3_online_walking_module_msgs::WalkingParam& msg)
{
  walking_param_ = msg;
}

void OnlineWalkingModule::goalJointPoseCallback(const op3_online_walking_module_msgs::JointPose& msg)
{
  if (enable_ == false)
    return;

  size_t joint_size = msg.pose.name.size();

  if (control_type_ == NONE || control_type_ == JOINT_CONTROL)
  {
    mov_time_ = msg.mov_time;

    for (size_t i = 0; i < msg.pose.name.size(); i++)
    {
      std::string joint_name = msg.pose.name[i];
      goal_joint_pos_[joint_name_to_id_[joint_name] - 1] = msg.pose.position[i];
    }

    joint_control_initialize_ = false;
    control_type_ = JOINT_CONTROL;
    balance_type_ = OFF;
    des_balance_gain_ratio_[0] = 0.0;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::initJointControl()
{
  if (joint_control_initialize_ == true)
    return;

  joint_control_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  joint_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_joint_pos_, des_joint_vel_, des_joint_accel_,
                                         goal_joint_pos_, goal_joint_vel_, goal_joint_accel_);
  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Joint Control");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Joint Control");
	ROS_INFO("walking_initalize_:%d",walking_initialize_);
	ROS_INFO("control_type:%d",control_type_);
	ROS_INFO("walking_phase_:%d",walking_phase_);
	ROS_INFO("balance_type_:%d",balance_type_);
	ROS_INFO("is_moving_:%d",is_moving_);
  }
}

void OnlineWalkingModule::calcJointControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_joint_pos_ = joint_tra_->getPosition(cur_time);
    des_joint_vel_ = joint_tra_->getVelocity(cur_time);
    des_joint_accel_ = joint_tra_->getAcceleration(cur_time);

    queue_mutex_.unlock();

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      delete joint_tra_;

      control_type_ = NONE;

      ROS_INFO("[END] Joint Control");

    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::setBodyOffsetCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  if (control_type_ == NONE || control_type_ == OFFSET_CONTROL)
  {
    goal_body_offset_[0] = msg->position.x;
    goal_body_offset_[1] = msg->position.y;
    goal_body_offset_[2] = msg->position.z;

    body_offset_initialize_ = false;
    control_type_ = OFFSET_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::setFootDistanceCallback(const std_msgs::Float64::ConstPtr& msg)
{
  if (enable_ == false)
    return;

  foot_distance_ = msg->data;

  resetBodyPose();
}

void OnlineWalkingModule::initOffsetControl()
{
  if (body_offset_initialize_ == true)
    return;

  body_offset_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = 1.0;

  body_offset_step_ = 0;
  body_offset_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  std::vector<double_t> offset_zero;
  offset_zero.resize(3, 0.0);

  body_offset_tra_ =
      new robotis_framework::MinimumJerk(ini_time, mov_time,
                                         des_body_offset_, offset_zero, offset_zero,
                                         goal_body_offset_, offset_zero, offset_zero);

  if (is_moving_ == true)
    ROS_INFO("[UPDATE] Body Offset");
  else
  {
    is_moving_ = true;
    ROS_INFO("[START] Body Offset");
  }
}

void OnlineWalkingModule::calcOffsetControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) body_offset_step_ * control_cycle_sec_;

    queue_mutex_.lock();

    des_body_offset_ = body_offset_tra_->getPosition(cur_time);

    queue_mutex_.unlock();

    if (body_offset_step_ == mov_size_-1)
    {
      body_offset_step_ = 0;
      is_moving_ = false;
      delete body_offset_tra_;

      control_type_ = NONE;

      ROS_INFO("[END] Body Offset");
    }
    else
      body_offset_step_++;
  }
}

void OnlineWalkingModule::goalKinematicsPoseCallback(const op3_online_walking_module_msgs::KinematicsPose& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  if (control_type_ == NONE || control_type_ == WHOLEBODY_CONTROL)
  {
    if (is_moving_ == true)
    {
      if (wholegbody_control_group_!=msg.name)
      {
        ROS_WARN("[WARN] Control group is different!");
        return;
      }
    }
    mov_time_ = msg.mov_time;
    wholegbody_control_group_ = msg.name;
    wholebody_goal_msg_ = msg.pose;

    wholebody_initialize_ = false;
    control_type_ = WHOLEBODY_CONTROL;
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::initWholebodyControl()
{
  if (wholebody_initialize_ == true)
    return;

  wholebody_initialize_ = true;

  double ini_time = 0.0;
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  wholebody_control_ =
      new WholebodyControl(wholegbody_control_group_,
                           ini_time, mov_time,
                           wholebody_goal_msg_);

  if (is_moving_ == true)
  {
    // TODO
  }
  else
  {
    ROS_INFO("[START] Wholebody Control");

    wholebody_control_->initialize(des_body_pos_, des_body_Q_,
                                   des_r_leg_pos_, des_r_leg_Q_,
                                   des_l_leg_pos_, des_l_leg_Q_);
    is_moving_ = true;
  }
}

void OnlineWalkingModule::calcWholebodyControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;

    wholebody_control_->set(cur_time);

    wholebody_control_->getTaskPosition(des_l_leg_pos_,
                                        des_r_leg_pos_,
                                        des_body_pos_);
    wholebody_control_->getTaskOrientation(des_l_leg_Q_,
                                           des_r_leg_Q_,
                                           des_body_Q_);

    if (mov_step_ == mov_size_-1)
    {
      mov_step_ = 0;
      is_moving_ = false;
      wholebody_control_->finalize();

      control_type_ = NONE;

      ROS_INFO("[END] Wholebody Control");
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::footStep2DCallback(const op3_online_walking_module_msgs::Step2DArray& msg)
{
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  Eigen::Quaterniond body_Q(des_body_Q_[3],des_body_Q_[0],des_body_Q_[1],des_body_Q_[2]);
  Eigen::MatrixXd body_R = robotis_framework::convertQuaternionToRotation(body_Q);
  Eigen::MatrixXd body_rpy = robotis_framework::convertQuaternionToRPY(body_Q);
  Eigen::MatrixXd body_T = Eigen::MatrixXd::Identity(4,4);
  body_T.block(0,0,3,3) = body_R;
  body_T.coeffRef(0,3) = des_body_pos_[0];
  body_T.coeffRef(1,3) = des_body_pos_[1];

  op3_online_walking_module_msgs::Step2DArray foot_step_msg;

  int old_size = msg.footsteps_2d.size();
  int new_size = old_size + 3;

  op3_online_walking_module_msgs::Step2D first_msg;
  op3_online_walking_module_msgs::Step2D second_msg;

  first_msg.moving_foot = msg.footsteps_2d[0].moving_foot - 1;
  second_msg.moving_foot = first_msg.moving_foot + 1;

  if (first_msg.moving_foot == LEFT_LEG)
  {
    first_msg.step2d.x = des_l_leg_pos_[0];
    first_msg.step2d.y = des_l_leg_pos_[1];
    first_msg.step2d.theta = body_rpy.coeff(2,0); //0.0;

    second_msg.step2d.x = des_r_leg_pos_[0];
    second_msg.step2d.y = des_r_leg_pos_[1];
    second_msg.step2d.theta = body_rpy.coeff(2,0); //0.0;
  }
  else if (first_msg.moving_foot == RIGHT_LEG)
  {
    first_msg.step2d.x = des_r_leg_pos_[0];
    first_msg.step2d.y = des_r_leg_pos_[1];
    first_msg.step2d.theta = body_rpy.coeff(2,0); //0.0;

    second_msg.step2d.x = des_l_leg_pos_[0];
    second_msg.step2d.y = des_l_leg_pos_[1];
    second_msg.step2d.theta = body_rpy.coeff(2,0); //0.0;
  }

  foot_step_msg.footsteps_2d.push_back(first_msg);
  foot_step_msg.footsteps_2d.push_back(second_msg);

  double step_final_theta;

  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    for (int i=0; i<old_size; i++)
    {
      op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[i];
      step_msg.moving_foot -= 1;

      Eigen::MatrixXd step_R = robotis_framework::convertRPYToRotation(0.0,0.0,step_msg.step2d.theta);
      Eigen::MatrixXd step_T = Eigen::MatrixXd::Identity(4,4);
      step_T.block(0,0,3,3) = step_R;
      step_T.coeffRef(0,3) = step_msg.step2d.x;
      step_T.coeffRef(1,3) = step_msg.step2d.y;

      Eigen::MatrixXd step_T_new = body_T*step_T;
      Eigen::MatrixXd step_R_new = step_T_new.block(0,0,3,3);

      double step_new_x = step_T_new.coeff(0,3);
      double step_new_y = step_T_new.coeff(1,3);
      Eigen::MatrixXd step_new_rpy = robotis_framework::convertRotationToRPY(step_R_new);
      double step_new_theta = step_new_rpy.coeff(2,0);

      step_msg.step2d.x = step_new_x;
      step_msg.step2d.y = step_new_y;
      step_msg.step2d.theta = step_new_theta;

      if (i == old_size-1)
        step_final_theta = step_new_theta;

      foot_step_msg.footsteps_2d.push_back(step_msg);
    }

    op3_online_walking_module_msgs::Step2D step_msg = msg.footsteps_2d[old_size-1];

    if (step_msg.moving_foot - 1 == LEFT_LEG)
      first_msg.moving_foot = RIGHT_LEG;
    else
      first_msg.moving_foot = LEFT_LEG;

    first_msg.step2d.x      = 0.0;
    first_msg.step2d.y      = 0.0;
    first_msg.step2d.theta  = step_final_theta; //step_msg.step2d.theta;

    foot_step_msg.footsteps_2d.push_back(first_msg);

    foot_step_2d_ = foot_step_msg;
    foot_step_2d_.step_time = msg.step_time;

    walking_size_ = new_size;
    mov_time_ = msg.step_time; //1.0;
    is_foot_step_2d_ = true;
    control_type_ = WALKING_CONTROL;

    if (is_moving_ == false)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}
int torque_button_count=0;
void OnlineWalkingModule::footStepCommandCallback(const op3_online_walking_module_msgs::FootStepCommand& msg)
{
	const int BAUD_RATE = 2000000;
	const double PROTOCOL_VERSION = 2.0;
	const int SUB_CONTROLLER_ID = 200;
	const int DXL_BROADCAST_ID = 254;
	const int DEFAULT_DXL_ID = 1;
	const std::string SUB_CONTROLLER_DEVICE = "/dev/ttyUSB0";
	const int POWER_CTRL_TABLE = 24;
	const int RGB_LED_CTRL_TABLE = 26;
	const int TORQUE_ON_CTRL_TABLE = 64;

	bool g_is_simulation = false;
	int g_baudrate;
	std::string g_offset_file;
	std::string g_robot_file;
	std::string g_init_file;
	std::string g_device_name;
	
	ROS_INFO("footstepcommandcallback = %s", msg.command.c_str());
	if(msg.command =="torquemode")
	{

		control_mode_ = robotis_framework::TorqueControl;
		present_control_mode=1;
		// if (controller->initialize("/home/robotis/catkin_ws/src/ROBOTIS-OP3/op3_manager/config/OP3.robot", "/home/robotis/catkin_ws/src/ROBOTIS-OP3/op3_manager/config/dxl_init_OP3_torque.yaml") == false)
		// {
		// 	ROS_ERROR("ROBOTIS Controller Initialize Fail!");
		// 	return;
		// }
	}
	else if(msg.command == "positionmode")
	{
		control_mode_ = robotis_framework::PositionControl;
		present_control_mode=0;
		// if (controller->initialize("/home/robotis/catkin_ws/src/ROBOTIS-OP3/op3_manager/config/OP3.robot", "/home/robotis/catkin_ws/src/ROBOTIS-OP3/op3_manager/config/dxl_init_OP3_position.yaml") == false)
		// {
		// 	ROS_ERROR("ROBOTIS Controller Initialize Fail!");
		// 	return;
		// }
	}


	// if(msg.command =="torquemode")
	// {
	// 	PortHandler *port_handler = (PortHandler *) PortHandler::getPortHandler("/dev/ttyUSB0");
	// 	// port_to_sync_write_current_test=new dynamixel::GroupSyncWrite()
	// 	bool set_port_result = port_handler->setBaudRate(2000000);
	// 	// RobotisController *controller = RobotisController::getInstance();
	// 	// controller->stopTimer();
    // 	// controller->setCtrlModule("none");
	// 	if (set_port_result == false)
	// 	{
	// 		ROS_ERROR("Error Set port");
	// 		return;
	// 	}
	// 	else
	// 		ROS_INFO("Set port");
		
	// 	PacketHandler *packet_handler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);


	// 	// for(uint8_t dxl_idx=7;dxl_idx=<=18;dxl_idx++)
	// 	// {
	// 	// 	ROS_INFO("torque off [%d]",dxl_idx);
	// 	// 	packet_handler->write1ByteTxRx(port_handler, dxl_idx, TORQUE_ON_CTRL_TABLE, 0);
	// 	// }
	// 	// for(uint8_t dxl_idx=7;dxl_idx==18;dxl_idx++)
	// 	// {
	// 	// 	ROS_INFO("torque mode change [%d]",dxl_idx);
	// 	// 	packet_handler->write1ByteTxRx(port_handler, dxl_idx, 11, 0);
	// 	// }
	// 	// for(uint8_t dxl_idx=7;dxl_idx==18;dxl_idx++)
	// 	// {
	// 	// 	ROS_INFO("torque on [%d]",dxl_idx);
	// 	// 	packet_handler->write1ByteTxRx(port_handler, dxl_idx, TORQUE_ON_CTRL_TABLE, 1);
	// 	// }
		

	// 	// torque off only one dxl for test
	// 	// int torque_off_count=0;
	// 	// int torque_mode_count=0;
	// 	// int torque_on_count=0;
	// 	// for(uint8_t dxl_idx=7;dxl_idx<=18;dxl_idx++)
	// 	// {	
	// 	// 	ROS_INFO("dxl_idx=%d",dxl_idx);
	// 	// 	torque_off_count=0;
	// 	// 	torque_mode_count=0;
	// 	// 	torque_on_count=0;			
	// 	// 	while(torque_off_count<5 )
	// 	// 	{
	// 	// 		ROS_INFO("torque off %d",torque_off_count);
	// 	// 		int _return_torque = packet_handler->write1ByteTxRx(port_handler, dxl_idx, TORQUE_ON_CTRL_TABLE, 0);
	// 	// 		if(_return_torque != 0)
	// 	// 		{
	// 	// 			ROS_ERROR("Torque off dxl failed [%s]", packet_handler->getRxPacketError(_return_torque));
	// 	// 		}
	// 	// 		else
	// 	// 		{
	// 	// 			ROS_INFO("Torque off dxl");
	// 	// 		}
	// 	// 		if(_return_torque ==0)
	// 	// 			break;
	// 	// 		else
	// 	// 			torque_off_count++;
	// 	// 	}
	// 	// 	if(torque_off_count==5)
	// 	// 	{
	// 	// 		ROS_ERROR("torque off failed");
	// 	// 		return;
	// 	// 	}
	// 	// 	// usleep(100 * 1000);
	// 	// 	//positionmode->torquemode
			
	// 	// 	while(torque_mode_count<5 )
	// 	// 	{
	// 	// 		ROS_INFO("torque mode change %d",torque_mode_count);
	// 	// 		int _return_torque = packet_handler->write1ByteTxRx(port_handler, dxl_idx, 11, 0);
	// 	// 		if(_return_torque != 0)
	// 	// 		{
	// 	// 			ROS_ERROR("Torque mode change dxl failed [%s]", packet_handler->getRxPacketError(_return_torque));
	// 	// 		}
	// 	// 		else
	// 	// 		{
	// 	// 			ROS_INFO("Torque mode change dxl");
	// 	// 		}
	// 	// 		if(_return_torque ==0)
	// 	// 			break;
	// 	// 		else
	// 	// 			torque_mode_count++;
	// 	// 	}
	// 	// 	if(torque_mode_count==5)
	// 	// 	{
	// 	// 		ROS_ERROR("torque mode change failed");
	// 	// 		return;
	// 	// 	}
	// 	// 	// usleep(100 * 1000);
	// 	// 	// torque on only one dxl for test
			
	// 	// 	while(torque_on_count<5 )
	// 	// 	{
	// 	// 		ROS_INFO("torque on %d",torque_on_count);
	// 	// 		int _return_torque = packet_handler->write1ByteTxRx(port_handler, dxl_idx, TORQUE_ON_CTRL_TABLE, 1);
	// 	// 		if(_return_torque != 0)
	// 	// 		{
	// 	// 			ROS_ERROR("Torque on dxl failed [%s]", packet_handler->getRxPacketError(_return_torque));
	// 	// 		}
	// 	// 		else
	// 	// 		{
	// 	// 			ROS_INFO("Torque on dxl");
	// 	// 		}
	// 	// 		if(_return_torque ==0)
	// 	// 			break;
	// 	// 		else
	// 	// 			torque_on_count++;
	// 	// 	}
	// 	// 	if(torque_on_count==5)
	// 	// 	{
	// 	// 		ROS_ERROR("torque on failed");
	// 	// 		return;
	// 	// 	}
	// 	// 	// usleep(100 * 1000);
	// 	// }

	// 	// torque off only one dxl for test last
	// 	// torque_off_count=0;
	// 	// while(torque_off_count<5 && torque_button_count==2)
	// 	// {
	// 	// 	int _return_torque = packet_handler->write1ByteTxRx(port_handler, 12, TORQUE_ON_CTRL_TABLE, 0);
	// 	// 	if(_return_torque != 0)
	// 	// 	{
	// 	// 		ROS_ERROR("Torque off dxl failed [%s]", packet_handler->getRxPacketError(_return_torque));
	// 	// 	}
	// 	// 	else
	// 	// 	{
	// 	// 		ROS_INFO("Torque off dxl");
	// 	// 	}
	// 	// 	if(_return_torque ==0)
	// 	// 		break;
	// 	// 	else
	// 	// 		torque_off_count++;
	// 	// }
	// 	// if(torque_off_count==5)
	// 	// {
	// 	// 	ROS_ERROR("torque off failed");
	// 	// 	return;
	// 	// }
	// 	// usleep(100 * 1000);
		
	// 	ROS_INFO("torque button callback count = %d",torque_button_count);
	// 	torque_button_count++;
	// }
	
  if (enable_ == false)
    return;

  if (balance_type_ == OFF)
  {
    ROS_WARN("[WARN] Balance is off!");
    return;
  }

  is_foot_step_2d_ = false;

  if (control_type_ == NONE || control_type_ == WALKING_CONTROL)
  {
    walking_size_ = msg.step_num + 3; //msg.step_num + 2;
    mov_time_ = msg.step_time;

    foot_step_command_ = msg;
    foot_step_command_.step_num = walking_size_;

    control_type_ = WALKING_CONTROL;

    if (is_moving_ == false)
      initWalkingControl();
    else
      ROS_WARN("[WARN] Previous task is alive!");
  }
  else
    ROS_WARN("[WARN] Control type is different!");
}

void OnlineWalkingModule::initWalkingControl()
{
  double mov_time = mov_time_;

  mov_step_ = 0;
  mov_size_ = (int) (mov_time / control_cycle_sec_) + 1;

  walking_step_ = 0;

  walking_control_ = new WalkingControl(control_cycle_sec_,
                                        walking_param_.dsp_ratio, walking_param_.lipm_height, walking_param_.foot_height_max,
                                        walking_param_.zmp_offset_x, walking_param_.zmp_offset_y,
                                        x_lipm_, y_lipm_,
                                        foot_distance_);

  double lipm_height = walking_control_->getLipmHeight();
  preview_request_.lipm_height = lipm_height;
  preview_request_.control_cycle = control_cycle_sec_;

  bool get_preview_matrix = false;
  get_preview_matrix = definePreviewMatrix();

  if (get_preview_matrix == true)
  {
    if (is_moving_ == true)
    {
      // TODO
    }
    else
    {
      if (is_foot_step_2d_ == true)
      {
        walking_control_->initialize(foot_step_2d_,
                                     des_body_pos_, des_body_Q_,
                                     des_r_leg_pos_, des_r_leg_Q_,
                                     des_l_leg_pos_, des_l_leg_Q_);
      }
      else
      {
        walking_control_->initialize(foot_step_command_,
                                     des_body_pos_, des_body_Q_,
                                     des_r_leg_pos_, des_r_leg_Q_,
                                     des_l_leg_pos_, des_l_leg_Q_);
      }

      walking_control_->calcPreviewParam(preview_response_K_,preview_response_K_row_,preview_response_K_col_,
                                         preview_response_P_,preview_response_P_row_,preview_response_P_row_);

      is_moving_ = true;

      initFeedforwardControl();
#if debugFprintOn
	wpg.open("/home/robotis/catkin_ws/src/ROBOTIS-OP3/op3_online_walking_module/src/wpg.txt");
#endif
		
      ROS_INFO("Sampling time: %lfms",control_cycle_sec_*1000);
      ROS_INFO("[START] wWalking Control (%d/%d)", walking_step_+1, walking_size_);
    }
    
    walking_initialize_ = true;
  }
  else
    ROS_WARN("[FAIL] Cannot get preview matrix");
}

void OnlineWalkingModule::calcWalkingControl()
{
  if (is_moving_ == true)
  {
    double cur_time = (double) mov_step_ * control_cycle_sec_;
    walking_control_->set(cur_time, walking_step_,is_foot_step_2d_);

    walking_control_->getWalkingPosition(des_l_leg_pos_,
                                         des_r_leg_pos_,
                                         des_body_pos_);
    walking_control_->getWalkingOrientation(des_l_leg_Q_,
                                            des_r_leg_Q_,
                                            des_body_Q_);

    walking_control_->getLIPM(x_lipm_, y_lipm_);

    walking_control_->getWalkingState(walking_leg_, walking_phase_);


    if (mov_step_ == mov_size_-1)
    {
      ROS_INFO("[END] Walking Control (%d/%d)", walking_step_+1, walking_size_);

      mov_step_ = 0;
      walking_control_->next();

      if (walking_step_ == walking_size_-1)
      {
        is_moving_ = false;
        is_foot_step_2d_ = false;
        walking_control_->finalize();

        control_type_ = NONE;
        walking_phase_ = DSP;
      }
      else
      {
        walking_step_++;
        ROS_INFO("[START] Walking Control (%d/%d)", walking_step_+1, walking_size_);
      }
    }
    else
      mov_step_++;
  }
}

void OnlineWalkingModule::initFeedforwardControl()
{
  // feedforward trajectory
  
  std::vector<double_t> zero_vector;
  zero_vector.resize(1,0.0);

  std::vector<double_t> via_pos;
  via_pos.resize(3, 0.0);
  via_pos[0] = 1.0 * DEGREE2RADIAN;

  double init_time = 0.0;
  double fin_time = mov_time_;
  double via_time = 0.5 * (init_time + fin_time);
  double dsp_ratio = walking_param_.dsp_ratio;

  feed_forward_tra_ =
      new robotis_framework::MinimumJerkViaPoint(init_time, fin_time, via_time, dsp_ratio,
                                                 zero_vector, zero_vector, zero_vector,
                                                 zero_vector, zero_vector, zero_vector,
                                                 via_pos, zero_vector, zero_vector);
                                                 
}

void OnlineWalkingModule::calcRobotPose()
{
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3,1);
  des_body_pos.coeffRef(0,0) = des_body_pos_[0];
  des_body_pos.coeffRef(1,0) = des_body_pos_[1];
  des_body_pos.coeffRef(2,0) = des_body_pos_[2];

  Eigen::Quaterniond des_body_Q(des_body_Q_[3],des_body_Q_[0],des_body_Q_[1],des_body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q);

  // Forward Kinematics
  op3_kdl_->initialize(des_body_pos, des_body_rot);

  Eigen::VectorXd r_leg_joint_pos, l_leg_joint_pos;

  r_leg_joint_pos.resize(6);
  r_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1];
  r_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1];
  r_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1];
  r_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["r_knee"]-1];
  r_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1];
  r_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1];

  l_leg_joint_pos.resize(6);
  l_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1];
  l_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1];
  l_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1];
  l_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["l_knee"]-1];
  l_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1];
  l_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1];

  op3_kdl_->setJointPosition(r_leg_joint_pos, l_leg_joint_pos);

  std::vector<double_t> r_leg_pos, r_leg_Q;
  r_leg_pos.resize(3,0.0);
  r_leg_Q.resize(4,0.0);

  std::vector<double_t> l_leg_pos, l_leg_Q;
  l_leg_pos.resize(3,0.0);
  l_leg_Q.resize(4,0.0);

  op3_kdl_->solveForwardKinematics(r_leg_pos, r_leg_Q,
                                   l_leg_pos, l_leg_Q);

  Eigen::Quaterniond curr_r_leg_Q(r_leg_Q[3],r_leg_Q[0],r_leg_Q[1],r_leg_Q[2]);
  Eigen::MatrixXd curr_r_leg_rot = robotis_framework::convertQuaternionToRotation(curr_r_leg_Q);

  Eigen::MatrixXd g_to_r_leg = Eigen::MatrixXd::Identity(4,4);
  g_to_r_leg.block(0,0,3,3) = curr_r_leg_rot;
  g_to_r_leg.coeffRef(0,3) = r_leg_pos[0];
  g_to_r_leg.coeffRef(1,3) = r_leg_pos[1];
  g_to_r_leg.coeffRef(2,3) = r_leg_pos[2];

  Eigen::Quaterniond curr_l_leg_Q(l_leg_Q[3],l_leg_Q[0],l_leg_Q[1],l_leg_Q[2]);
  Eigen::MatrixXd curr_l_leg_rot = robotis_framework::convertQuaternionToRotation(curr_l_leg_Q);

  Eigen::MatrixXd g_to_l_leg = Eigen::MatrixXd::Identity(4,4);
  g_to_l_leg.block(0,0,3,3) = curr_l_leg_rot;
  g_to_l_leg.coeffRef(0,3) = l_leg_pos[0];
  g_to_l_leg.coeffRef(1,3) = l_leg_pos[1];
  g_to_l_leg.coeffRef(2,3) = l_leg_pos[2];

  op3_kdl_->finalize();
}

void OnlineWalkingModule::setTargetForceTorque()
{
  if (walking_phase_ == DSP)
  {
    balance_r_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_r_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_r_foot_force_z_ = -0.5 * total_mass_ * 9.81;

    balance_l_foot_force_x_ = -0.5 * total_mass_ * x_lipm_[2];
    balance_l_foot_force_y_ = -0.5 * total_mass_ * y_lipm_[2];
    balance_l_foot_force_z_ = -0.5 * total_mass_ * 9.81;
  }
  else if (walking_phase_ == SSP)
  {
    if (walking_leg_ == LEFT_LEG)
    {
      balance_r_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_r_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_r_foot_force_z_ = -1.0 * total_mass_ * 9.81;

      balance_l_foot_force_x_ = 0.0;
      balance_l_foot_force_y_ = 0.0;
      balance_l_foot_force_z_ = 0.0;
    }
    else if (walking_leg_ == RIGHT_LEG)
    {
      balance_r_foot_force_x_ = 0.0;
      balance_r_foot_force_y_ = 0.0;
      balance_r_foot_force_z_ = 0.0;

      balance_l_foot_force_x_ = -1.0 * total_mass_ * x_lipm_[2];
      balance_l_foot_force_y_ = -1.0 * total_mass_ * y_lipm_[2];
      balance_l_foot_force_z_ = -1.0 * total_mass_ * 9.81;
    }
  }
}

void OnlineWalkingModule::setBalanceControlGain()
{
  //// set gain
  //gyro
  balance_control_.foot_roll_gyro_ctrl_.p_gain_ = foot_roll_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_gyro_ctrl_.d_gain_ = foot_roll_gyro_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.p_gain_ = foot_pitch_gyro_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_gyro_ctrl_.d_gain_ = foot_pitch_gyro_d_gain_ * des_balance_gain_ratio_[0];

  //orientation
  balance_control_.foot_roll_angle_ctrl_.p_gain_  = foot_roll_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_roll_angle_ctrl_.d_gain_  = foot_roll_angle_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.p_gain_ = foot_pitch_angle_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.foot_pitch_angle_ctrl_.d_gain_ = foot_pitch_angle_d_gain_ * des_balance_gain_ratio_[0];

  //force torque
  balance_control_.right_foot_force_x_ctrl_.p_gain_      = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.p_gain_      = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.p_gain_      = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.p_gain_  = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_x_ctrl_.d_gain_      = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_y_ctrl_.d_gain_      = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_force_z_ctrl_.d_gain_      = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_roll_ctrl_.d_gain_  = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.right_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  balance_control_.left_foot_force_x_ctrl_.p_gain_      = foot_x_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.p_gain_      = foot_y_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.p_gain_      = foot_z_force_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.p_gain_  = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.p_gain_ = foot_roll_torque_p_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_x_ctrl_.d_gain_      = foot_x_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_y_ctrl_.d_gain_      = foot_y_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_force_z_ctrl_.d_gain_      = foot_z_force_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_roll_ctrl_.d_gain_  = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];
  balance_control_.left_foot_torque_pitch_ctrl_.d_gain_ = foot_roll_torque_d_gain_ * des_balance_gain_ratio_[0];

  //// set cut off freq
  balance_control_.roll_gyro_lpf_.setCutOffFrequency(roll_gyro_cut_off_frequency_);
  balance_control_.pitch_gyro_lpf_.setCutOffFrequency(pitch_gyro_cut_off_frequency_);
  balance_control_.roll_angle_lpf_.setCutOffFrequency(roll_angle_cut_off_frequency_);
  balance_control_.pitch_angle_lpf_.setCutOffFrequency(pitch_angle_cut_off_frequency_);

  balance_control_.right_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.right_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.right_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.right_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.right_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);

  balance_control_.left_foot_force_x_lpf_.setCutOffFrequency(foot_x_force_cut_off_frequency_);
  balance_control_.left_foot_force_y_lpf_.setCutOffFrequency(foot_y_force_cut_off_frequency_);
  balance_control_.left_foot_force_z_lpf_.setCutOffFrequency(foot_z_force_cut_off_frequency_);
  balance_control_.left_foot_torque_roll_lpf_.setCutOffFrequency(foot_roll_torque_cut_off_frequency_);
  balance_control_.left_foot_torque_pitch_lpf_.setCutOffFrequency(foot_pitch_torque_cut_off_frequency_);
}

bool OnlineWalkingModule::setBalanceControl()
{
  // Set Balance Control
  balance_control_.setGyroBalanceEnable(true);
  balance_control_.setOrientationBalanceEnable(true);
  balance_control_.setForceTorqueBalanceEnable(true);

  balance_control_.setCOBManualAdjustment(des_body_offset_[0], des_body_offset_[1], des_body_offset_[2]);

  setBalanceControlGain();
  setTargetForceTorque();

  bool ik_success = true;

  // Body Pose
  Eigen::MatrixXd des_body_pos = Eigen::MatrixXd::Zero(3,1);
  des_body_pos.coeffRef(0,0) = des_body_pos_[0];
  des_body_pos.coeffRef(1,0) = des_body_pos_[1];
  des_body_pos.coeffRef(2,0) = des_body_pos_[2];

  Eigen::Quaterniond des_body_Q(des_body_Q_[3],des_body_Q_[0],des_body_Q_[1],des_body_Q_[2]);
  Eigen::MatrixXd des_body_rot = robotis_framework::convertQuaternionToRotation(des_body_Q);
  Eigen::MatrixXd des_body_rpy = robotis_framework::convertQuaternionToRPY(des_body_Q);

  // Right Leg Pose
  Eigen::MatrixXd des_r_foot_pos = Eigen::MatrixXd::Zero(3,1);
  des_r_foot_pos.coeffRef(0,0) = des_r_leg_pos_[0];
  des_r_foot_pos.coeffRef(1,0) = des_r_leg_pos_[1];
  des_r_foot_pos.coeffRef(2,0) = des_r_leg_pos_[2];

  Eigen::Quaterniond des_r_foot_Q(des_r_leg_Q_[3],des_r_leg_Q_[0],des_r_leg_Q_[1],des_r_leg_Q_[2]);
  Eigen::MatrixXd des_r_foot_rot = robotis_framework::convertQuaternionToRotation(des_r_foot_Q);

  // Left Leg Pose
  Eigen::MatrixXd des_l_foot_pos = Eigen::MatrixXd::Zero(3,1);
  des_l_foot_pos.coeffRef(0,0) = des_l_leg_pos_[0];
  des_l_foot_pos.coeffRef(1,0) = des_l_leg_pos_[1];
  des_l_foot_pos.coeffRef(2,0) = des_l_leg_pos_[2];

  Eigen::Quaterniond des_l_foot_Q(des_l_leg_Q_[3],des_l_leg_Q_[0],des_l_leg_Q_[1],des_l_leg_Q_[2]);
  Eigen::MatrixXd des_l_foot_rot = robotis_framework::convertQuaternionToRotation(des_l_foot_Q);

  // Set Desired Value for Balance Control
  Eigen::MatrixXd body_pose = Eigen::MatrixXd::Identity(4,4);
  body_pose.block<3,3>(0,0) = des_body_rot;
  body_pose.block<3,1>(0,3) = des_body_pos;

  Eigen::MatrixXd l_foot_pose = Eigen::MatrixXd::Identity(4,4);
  l_foot_pose.block<3,3>(0,0) = des_l_foot_rot;
  l_foot_pose.block<3,1>(0,3) = des_l_foot_pos;

  Eigen::MatrixXd r_foot_pose = Eigen::MatrixXd::Identity(4,4);
  r_foot_pose.block<3,3>(0,0) = des_r_foot_rot;
  r_foot_pose.block<3,1>(0,3) = des_r_foot_pos;

  // ===== Transformation =====
  Eigen::MatrixXd robot_to_body = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd robot_to_l_foot = body_pose.inverse() * l_foot_pose;
  Eigen::MatrixXd robot_to_r_foot = body_pose.inverse() * r_foot_pose;
  // =====

  // Set IMU
  imu_data_mutex_lock_.lock();

  balance_control_.setCurrentGyroSensorOutput(imu_data_msg_.angular_velocity.x, imu_data_msg_.angular_velocity.y);

  Eigen::Quaterniond imu_quaternion(imu_data_msg_.orientation.w,
                                    imu_data_msg_.orientation.x,
                                    imu_data_msg_.orientation.y,
                                    imu_data_msg_.orientation.z);
  Eigen::MatrixXd imu_rpy =
      robotis_framework::convertRotationToRPY(robotis_framework::getRotationX(M_PI) * imu_quaternion.toRotationMatrix() * robotis_framework::getRotationZ(M_PI));

  imu_data_mutex_lock_.unlock();

  // Set FT
  Eigen::MatrixXd robot_to_r_foot_force =
      robot_to_r_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.force.x, r_foot_ft_data_msg_.force.y, r_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_r_foot_torque =
      robot_to_r_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(r_foot_ft_data_msg_.torque.x, r_foot_ft_data_msg_.torque.y, r_foot_ft_data_msg_.torque.z);

  Eigen::MatrixXd robot_to_l_foot_force =
      robot_to_l_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.force.x, l_foot_ft_data_msg_.force.y, l_foot_ft_data_msg_.force.z);

  Eigen::MatrixXd robot_to_l_foot_torque =
      robot_to_l_foot.block(0,0,3,3) * robotis_framework::getRotationX(M_PI) *
      robotis_framework::getTransitionXYZ(l_foot_ft_data_msg_.torque.x, l_foot_ft_data_msg_.torque.y, l_foot_ft_data_msg_.torque.z);

  balance_control_.setCurrentOrientationSensorOutput(imu_rpy.coeff(0,0), imu_rpy.coeff(1,0));

  balance_control_.setCurrentFootForceTorqueSensorOutput(robot_to_r_foot_force.coeff(0,0),  robot_to_r_foot_force.coeff(1,0),  robot_to_r_foot_force.coeff(2,0),
                                                         robot_to_r_foot_torque.coeff(0,0), robot_to_r_foot_torque.coeff(1,0), robot_to_r_foot_torque.coeff(2,0),
                                                         robot_to_l_foot_force.coeff(0,0),  robot_to_l_foot_force.coeff(1,0),  robot_to_l_foot_force.coeff(2,0),
                                                         robot_to_l_foot_torque.coeff(0,0), robot_to_l_foot_torque.coeff(1,0), robot_to_l_foot_torque.coeff(2,0));

  balance_control_.setDesiredCOBGyro(0.0,0.0);

  balance_control_.setDesiredCOBOrientation(des_body_rpy.coeff(0,0),des_body_rpy.coeff(1,0));

  balance_control_.setDesiredFootForceTorque(balance_r_foot_force_x_, balance_r_foot_force_y_, balance_r_foot_force_z_,
                                             balance_r_foot_torque_x_, balance_r_foot_torque_y_, balance_r_foot_torque_z_,
                                             balance_l_foot_force_x_, balance_l_foot_force_y_, balance_l_foot_force_z_,
                                             balance_l_foot_torque_x_, balance_l_foot_torque_y_, balance_l_foot_torque_z_);

  balance_control_.setDesiredPose(robot_to_body, robot_to_r_foot, robot_to_l_foot);

  int error;
  Eigen::MatrixXd robot_to_body_mod, robot_to_r_foot_mod, robot_to_l_foot_mod;
  balance_control_.process(&error, &robot_to_body_mod, &robot_to_r_foot_mod, &robot_to_l_foot_mod);

  // ===== Transformation =====
  Eigen::MatrixXd body_pose_mod = body_pose * robot_to_body_mod;
  Eigen::MatrixXd r_foot_pose_mod = body_pose * robot_to_r_foot_mod;
  Eigen::MatrixXd l_foot_pose_mod = body_pose * robot_to_l_foot_mod;
  // =====

  Eigen::MatrixXd des_body_rot_mod = body_pose_mod.block<3,3>(0,0);
  Eigen::MatrixXd des_body_pos_mod = body_pose_mod.block<3,1>(0,3);

  Eigen::MatrixXd des_r_foot_rot_mod = r_foot_pose_mod.block<3,3>(0,0);
  Eigen::MatrixXd des_r_foot_pos_mod = r_foot_pose_mod.block<3,1>(0,3);
  Eigen::MatrixXd des_l_foot_rot_mod = l_foot_pose_mod.block<3,3>(0,0);
  Eigen::MatrixXd des_l_foot_pos_mod = l_foot_pose_mod.block<3,1>(0,3);

  // ======= ======= //
  op3_kdl_->initialize(des_body_pos_mod, des_body_rot_mod);

  Eigen::VectorXd r_leg_joint_pos, l_leg_joint_pos;

  r_leg_joint_pos.resize(6);
  r_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1];
  r_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1];
  r_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1];
  r_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["r_knee"]-1];
  r_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1];
  r_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1];

  l_leg_joint_pos.resize(6);
  l_leg_joint_pos(0) = des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1];
  l_leg_joint_pos(1) = des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1];
  l_leg_joint_pos(2) = des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1];
  l_leg_joint_pos(3) = des_joint_pos_[joint_name_to_id_["l_knee"]-1];
  l_leg_joint_pos(4) = des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1];
  l_leg_joint_pos(5) = des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1];

  op3_kdl_->setJointPosition(r_leg_joint_pos, l_leg_joint_pos);

  std::vector<double_t> r_leg_output, l_leg_output;

  Eigen::Quaterniond des_r_foot_Q_mod = robotis_framework::convertRotationToQuaternion(des_r_foot_rot_mod);
  Eigen::Quaterniond des_l_foot_Q_mod = robotis_framework::convertRotationToQuaternion(des_l_foot_rot_mod);

  ik_success = op3_kdl_->solveInverseKinematics(r_leg_output,
                                                des_r_foot_pos_mod,des_r_foot_Q_mod,
                                                l_leg_output,
                                                des_l_foot_pos_mod,des_l_foot_Q_mod);

  op3_kdl_->finalize();

  if (ik_success == true)
  {
    des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]      = r_leg_output[0];
    des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]     = r_leg_output[1];
    des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]    = r_leg_output[2];
    des_joint_pos_[joint_name_to_id_["r_knee"]-1]         = r_leg_output[3];
    des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]  = r_leg_output[4];
    des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]   = r_leg_output[5];

    des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]      = l_leg_output[0];
    des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]     = l_leg_output[1];
    des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]    = l_leg_output[2];
    des_joint_pos_[joint_name_to_id_["l_knee"]-1]         = l_leg_output[3];
    des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  = l_leg_output[4];
    des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]   = l_leg_output[5];
  }

  return ik_success;
}

void OnlineWalkingModule::setFeedbackControl()
{
  for (int i=0; i<number_of_joints_; i++)
  {
    des_joint_pos_to_robot_[i] = des_joint_pos_[i] + des_joint_feedforward_[i];

    joint_feedback_[i].desired_ = des_joint_pos_[i];
    des_joint_feedback_[i] = joint_feedback_[i].getFeedBack(curr_joint_pos_[i]);

    des_joint_pos_to_robot_[i] += des_joint_feedback_[i];
  }
}

void OnlineWalkingModule::setFeedforwardControl()
{
  double cur_time = (double) mov_step_ * control_cycle_sec_;

  std::vector<double_t> feed_forward_value = feed_forward_tra_->getPosition(cur_time);

  if (walking_phase_ == DSP)
    feed_forward_value[0] = 0.0;

  std::vector<double_t> support_leg_gain;
  support_leg_gain.resize(number_of_joints_, 0.0);

  if (walking_leg_ == LEFT_LEG)
  {
    support_leg_gain[joint_name_to_id_["r_hip_yaw"]-1]    = 1.0;
    support_leg_gain[joint_name_to_id_["r_hip_roll"]-1]   = 1.0;
    support_leg_gain[joint_name_to_id_["r_hip_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["r_knee"]-1]       = 1.0;
    support_leg_gain[joint_name_to_id_["r_ank_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["r_ank_roll"]-1]   = 1.0;

    support_leg_gain[joint_name_to_id_["l_hip_yaw"]-1]    = 1.0;
    support_leg_gain[joint_name_to_id_["l_hip_roll"]-1]   = 1.0;
    support_leg_gain[joint_name_to_id_["l_hip_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["l_knee"]-1]       = 1.0;
    support_leg_gain[joint_name_to_id_["l_ank_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["l_ank_roll"]-1]   = 1.0;
  }
  else if (walking_leg_ == RIGHT_LEG)
  {
    support_leg_gain[joint_name_to_id_["r_hip_yaw"]-1]    = 1.0;
    support_leg_gain[joint_name_to_id_["r_hip_roll"]-1]   = 1.0;
    support_leg_gain[joint_name_to_id_["r_hip_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["r_knee"]-1]       = 1.0;
    support_leg_gain[joint_name_to_id_["r_ank_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["r_ank_roll"]-1]   = 1.0;

    support_leg_gain[joint_name_to_id_["l_hip_yaw"]-1]    = 1.0;
    support_leg_gain[joint_name_to_id_["l_hip_roll"]-1]   = 1.0;
    support_leg_gain[joint_name_to_id_["l_hip_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["l_knee"]-1]       = 1.0;
    support_leg_gain[joint_name_to_id_["l_ank_pitch"]-1]  = 1.0;
    support_leg_gain[joint_name_to_id_["l_ank_roll"]-1]   = 1.0;
  }

  for (int i=0; i<number_of_joints_; i++)
    des_joint_feedforward_[i] = joint_feedforward_gain_[i] * feed_forward_value[0] * support_leg_gain[i];
}

void OnlineWalkingModule::sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle)
{
  // adjust balance offset
  double internal_gain = 0.05;

  balance_angle[joint_name_to_id_["r_hip_roll"]-1] =
      -1.0 * internal_gain * rlGyroErr * balance_hip_roll_gain_;  // R_HIP_ROLL
  balance_angle[joint_name_to_id_["l_hip_roll"]-1] =
      -1.0 * internal_gain * rlGyroErr * balance_hip_roll_gain_;  // L_HIP_ROLL

  balance_angle[joint_name_to_id_["r_knee"]-1] =
      1.0 * internal_gain * fbGyroErr * balance_knee_gain_;  // R_KNEE
  balance_angle[joint_name_to_id_["l_knee"]-1] =
      -1.0 * internal_gain * fbGyroErr * balance_knee_gain_;  // L_KNEE

  balance_angle[joint_name_to_id_["r_ank_pitch"]-1] =
      -1.0 * internal_gain * fbGyroErr * balance_ankle_pitch_gain_;  // R_ANKLE_PITCH
  balance_angle[joint_name_to_id_["l_ank_pitch"]-1] =
      1.0 * internal_gain * fbGyroErr * balance_ankle_pitch_gain_;  // L_ANKLE_PITCH

  balance_angle[joint_name_to_id_["r_ank_roll"]-1] =
      -1.0 * internal_gain * rlGyroErr * balance_ankle_roll_gain_;  // R_ANKLE_ROLL
  balance_angle[joint_name_to_id_["l_ank_roll"]-1] =
      -1.0 * internal_gain * rlGyroErr * balance_ankle_roll_gain_;  // L_ANKLE_ROLL
}


void OnlineWalkingModule::jointLimitCheck()
{
	// motor5 hip yaw r
	if( des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1] >= 0.3)
	{
		des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]=0.3;
	}
	else if( des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1] <= -0.3)
	{
		des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]=-0.3;
	}
	// motor4 hip roll r
	if( des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]  >= 0.3)
	{
		des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1] =0.3;
	}
	else if( des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]  <= -0.3)
	{
		des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1] =-0.3;
	}
	// motor3 hip pitch r
	if( des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]  >= 1.1)
	{
		des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1] =1.1;
	}
	else if( des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]  <= 0.4)
	{
		des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1] =0.4;
	}
	// motor2 knee pitch r
	if( des_joint_pos_[joint_name_to_id_["r_knee"]-1]   >= -1.4)
	{
		des_joint_pos_[joint_name_to_id_["r_knee"]-1]  =-1.4;
	}
	else if( des_joint_pos_[joint_name_to_id_["r_knee"]-1]   <= -1.9)
	{
		des_joint_pos_[joint_name_to_id_["r_knee"]-1]  =-1.9;
	}
	// motor1 ankle pitch r
	if( des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]  >= -0.4)
	{
		des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1] = -0.4;
	}
	else if( des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]  <= -1.2)
	{
		des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1] = -1.2;
	}
	// motor0 ankle roll r
	if( des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]  >= 0.3)
	{
		des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1] =0.3;
	}
	else if( des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]  <= -0.3)
	{
		des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1] =-0.3;
	}
	// motor6 hip yaw l
	if( des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1] >= 0.3)
	{
		des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]=0.3;
	}
	else if( des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1] <= -0.3)
	{
		des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]=-0.3;
	}
	// motor7 hip roll l
	if( des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]  >= 0.3)
	{
		des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1] =0.3;
	}
	else if( des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]  <= -0.3)
	{
		des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1] =-0.3;
	}
	// motor8 hip pitch l
	if( des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]  >= -0.4)
	{
		des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1] =-0.4;
	}
	else if( des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]  <= -1.1)
	{
		des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1] = -1.1;
	}
	// motor9 knee pitch l
	if( des_joint_pos_[joint_name_to_id_["l_knee"]-1]   >= 1.9)
	{
		des_joint_pos_[joint_name_to_id_["l_knee"]-1]  =1.9;
	}
	else if( des_joint_pos_[joint_name_to_id_["l_knee"]-1]   <= 1.4)
	{
		des_joint_pos_[joint_name_to_id_["l_knee"]-1]  = 1.4;
	}
	// motor10 ankle pitch l
	if( des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  >= 1.2)
	{
		des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1] = 1.2;
	}
	else if( des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  <= 0.5)
	{
		des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1] = 0.5;
	}
	// motor11 ankle roll l
	if(des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]  >= 0.3)
	{
		des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1] =0.3;
	}
	else if( des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]  <= -0.3)
	{
		des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1] =-0.3;
	}
}

void OnlineWalkingModule::jointTorqueLimitCheck(double limit)
{
	// motor5 hip yaw r
	if( des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1] >= limit)
	{
		des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]=limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1] <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]=-limit;
	}
	// motor4 hip roll r
	if( des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1] =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1] =-limit;
	}
	// motor3 hip pitch r
	if( des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1] =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1] = -limit;
	}
	// motor2 knee pitch r
	if( des_joint_torque_[joint_name_to_id_["r_knee"]-1]   >= limit)
	{
		des_joint_torque_[joint_name_to_id_["r_knee"]-1]  =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["r_knee"]-1]   <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["r_knee"]-1]  =-limit;
	}
	// motor1 ankle pitch r
	if( des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1] = limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1] = -limit;
	}
	// motor0 ankle roll r
	if( des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1] =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1] =-limit;
	}
	// motor6 hip yaw l
	if( des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1] >= limit)
	{
		des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]=limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1] <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]=-limit;
	}
	// motor7 hip roll l
	if( des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1] =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1] =-limit;
	}
	// motor8 hip pitch l
	if( des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1] =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1] = -limit;
	}
	// motor9 knee pitch l
	if( des_joint_torque_[joint_name_to_id_["l_knee"]-1]   >= limit)
	{
		des_joint_torque_[joint_name_to_id_["l_knee"]-1]  =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["l_knee"]-1]   <=-limit)
	{
		des_joint_torque_[joint_name_to_id_["l_knee"]-1]  = -limit;
	}
	// motor10 ankle pitch l
	if( des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1] = limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]  <=-limit)
	{
		des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1] =-limit;
	}
	// motor11 ankle roll l
	if(des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]  >= limit)
	{
		des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1] =limit;
	}
	else if( des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]  <= -limit)
	{
		des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1] =-limit;
	}
}

void OnlineWalkingModule::process(std::map<std::string, robotis_framework::Dynamixel *> dxls,
                              std::map<std::string, double> sensors)
{
	if (enable_ == false)
	return;

	double balance_angle[number_of_joints_];

	for (int i=0; i<number_of_joints_; i++)
	balance_angle[i] = 0.0;

	double rl_gyro_err = 0.0 - sensors["gyro_x"];
	double fb_gyro_err = 0.0 - sensors["gyro_y"];

	//sensoryFeedback(rl_gyro_err, fb_gyro_err, balance_angle);

	/*----- write curr position -----*/
	for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
		state_iter != result_.end(); state_iter++)
	{
	std::string joint_name = state_iter->first;

	robotis_framework::Dynamixel *dxl = NULL;
	std::map<std::string, robotis_framework::Dynamixel*>::iterator dxl_it = dxls.find(joint_name);
	if (dxl_it != dxls.end())
		dxl = dxl_it->second;
	else
		continue;

	double curr_joint_pos = dxl->dxl_state_->present_position_;
	double goal_joint_pos = dxl->dxl_state_->goal_position_;

	if (goal_initialize_ == false)
		des_joint_pos_[joint_name_to_id_[joint_name]-1] = goal_joint_pos;

		curr_joint_pos_[joint_name_to_id_[joint_name]-1] = curr_joint_pos;
	}

	goal_initialize_ = true;

	/* Trajectory Calculation */
	/* if(_sn!=0)
	{
	end = ros::Time::now().toSec();
	time_duration=end-begin;
	}*/
	begin = ros::Time::now().toSec();
 
	if (control_type_ == JOINT_CONTROL)
	{
#if debugPrintOn	  
		ROS_INFO("control_type_ == JOINT_CONTROL");
#endif	
		initJointControl();
		calcJointControl();
  	}
	else if (control_type_ == WHOLEBODY_CONTROL)
	{
#if debugPrintOn	  
		ROS_INFO("control_type_ == WHOLEBODY_CONTROL");
#endif	
		initWholebodyControl();
		calcWholebodyControl();
	}
	else if (control_type_ == WALKING_CONTROL)
	{
    // ROS_INFO("control_type_ == WALKING_CONTROL");
    if(walking_initialize_ == true)
    {
		//ROS_INFO("CONTROLL");
		samplingtime= control_cycle_sec_;
		samplingTime = samplingtime*1000; // integer sampling time
		samplingPeriod = samplingtime*1000;
		//_t = samplingtime;

		//clock_t start1, start2, start3, start4, end1, end2, end3, end4;

		double rot_th = 0;
		//add 0630  4685 - 5143 : RNEÂ¢Â¯Â¢Â® CEÂ¢Â¯aCN Â©Ã¶Â¡ÃÂ¨Ã¹Â¨Â¬AÂ¢Â® AÂ¢Â´ACÂ¢Ãu matrix Â¨Â¬?Â¨Ã¹o Â¨Ã¹Â¡Â¾Â¨ÃºÂ©Â£ Â¨Â¬IÂ¨Â¬Â¨Â¢AIÂ¥Ã¬Â¡Ã EÂ¢Ã§AICIÂ¢Â¬eÂ¨Ã¹Â¡Â© AÂ©Â¬Â¡ÃÂ¢Â®CIÂ¢Â¬e Â¥Ã¬E Â¡ÃI Â¡ÃÂ¡ÃÂ¨ÃºÂ¨Â¡
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
			//Â©Ã¶Â¡Ã¬Â¡ÃOAÂ©Â¬Â¨Ã¶E AOÂ¡Â¤A
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
		//Â©Ã¶ICI original

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

		//Â¢Â¬Â¥Ã¬AÂ¨ÃAuÂ¡Â¤Â¢Ã§
		double m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12;

    	m1 = 0.06934;//0.0794462; //0.06934// Â¢Â¥UAÂ¡Ã : kg
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
	
		Matrix_MM33X12 Inertia;
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
		Matrix_12X1 _th_current_n, _th_current_p, Mdth;
		Matrix_12X1 _error_dth_1, _error_dth, _error_th_1, _error_th, Mddth;
		
		double i_th_encoder[12];
		double j_th_encoder[12];
		Matrix_12X1 i_th_current_n, i_th_current_p,  iMdth;
		
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

		double RNEmass[12] = { m1, m2, m3, m4 , m5, m6, m7, m8 , m9, m10, m11, m12 };//AuÂ¡Â¤Â¢Ã§
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

		if(_i==0)
		{
#if debugPrintOn			
			ROS_INFO("walk init !!! _i=%d , _sn = %d",_i,_sn);
#endif			
			loopcount=0;
			_ppx=0;
			_ppy=0;
			_px=0;
			_py=0;
			_baserot=0;
			_lsr=0;
			_rsr=0;
			_sn=0;
			_s[0][5] = 0;
			_i=0;
			_rsx_des = 0;
			_rsy_des = 0;
			_rsz_des = 0;
			_lsx_des = 0;
			_lsy_des = 2 * L3;
			_lsz_des = 0;
			torqueStandingCount=0;
			torqueStandCount=0;
			for(int k=0 ;k<12;k++)
			{
				_th[k]=_th_i[k];
			}
			forwardkinematics(_th);
			_theta=0;
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
			_i = 2;
			_t=samplingtime;
		}

		if(torqueStandingCount<tq_standing)
		{
			_th_encoder[0] = -curr_joint_pos_[joint_name_to_id_["r_ank_roll"] - 1];
			_th_encoder[1] = -curr_joint_pos_[joint_name_to_id_["r_ank_pitch"] - 1];
			_th_encoder[2] = curr_joint_pos_[joint_name_to_id_["r_knee"] - 1];
			_th_encoder[3] = curr_joint_pos_[joint_name_to_id_["r_hip_pitch"] - 1];
			_th_encoder[4] = curr_joint_pos_[joint_name_to_id_["r_hip_roll"] - 1];
			_th_encoder[5] = curr_joint_pos_[joint_name_to_id_["r_hip_yaw"] - 1];
			_th_encoder[6] = -curr_joint_pos_[joint_name_to_id_["l_hip_yaw"] - 1];
			_th_encoder[7] = -curr_joint_pos_[joint_name_to_id_["l_hip_roll"] - 1];
			_th_encoder[8] = curr_joint_pos_[joint_name_to_id_["l_hip_pitch"] - 1];
			_th_encoder[9] = curr_joint_pos_[joint_name_to_id_["l_knee"] - 1];
			_th_encoder[10] = -curr_joint_pos_[joint_name_to_id_["l_ank_pitch"] - 1];
			_th_encoder[11] = curr_joint_pos_[joint_name_to_id_["l_ank_roll"] - 1];

			if(_sn==0)
			{
				result_th[0] = -curr_joint_pos_[joint_name_to_id_["r_ank_roll"] - 1];
				result_th[1] = -curr_joint_pos_[joint_name_to_id_["r_ank_pitch"] - 1];
				result_th[2] = curr_joint_pos_[joint_name_to_id_["r_knee"] - 1];
				result_th[3] = curr_joint_pos_[joint_name_to_id_["r_hip_pitch"] - 1];
				result_th[4] = curr_joint_pos_[joint_name_to_id_["r_hip_roll"] - 1];
				result_th[5] = curr_joint_pos_[joint_name_to_id_["r_hip_yaw"] - 1];
				result_th[6] = -curr_joint_pos_[joint_name_to_id_["l_hip_yaw"] - 1];
				result_th[7] = -curr_joint_pos_[joint_name_to_id_["l_hip_roll"] - 1];
				result_th[8] = curr_joint_pos_[joint_name_to_id_["l_hip_pitch"] - 1];
				result_th[9] = curr_joint_pos_[joint_name_to_id_["l_knee"] - 1];
				result_th[10] = -curr_joint_pos_[joint_name_to_id_["l_ank_pitch"] - 1];
				result_th[11] = curr_joint_pos_[joint_name_to_id_["l_ank_roll"] - 1];
				forwardkinematics(result_th);
			}
			else
			{
				forwardkinematics(_th_encoder);
			}
			
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
			double _tf=(tq_standing/2)*samplingtime;

			double poly_t = torqueStandingCount*samplingtime;

			if(torqueStandingCount<(tq_standing/2)+1){
				highpoly(poly_th_i,poly_th_f,poly_dth_i,poly_dth_f,poly_ddth_i,poly_ddth_f, _tf, poly_t, result_th);
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
			
			//RNE ÃÃÂ±Ã¢ ÃÂ¶Â°Ã
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


			//RNE ÃÃÂ±Ã¢ ÃÂ¶Â°Ã 
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

//#if debugPrintOn
			ROS_INFO("TORQUE R [0]=%6lf [1]=%6lf [2]=%6lf [3]=%6lf [4]=%6lf [5]=%6lf", -torque[0],-torque[1], torque[2],torque[3], torque[4],torque[5]);
			ROS_INFO("TORQUE L [11]=%6lf [10]=%6lf [9]=%6lf [8]=%6lf [7]=%6lf [6]=%6lf", torque[11],-torque[10], torque[9],torque[8], -torque[7],-torque[6]);
// #endif

#if torqueModeOn
			des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]      = torque[5]/149.795*227.509;//_th[5]//encoder[0]
			des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]     = torque[4]/149.795*227.509;//_th[4]//encoder[2]
			des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]    = torque[3]/149.795*227.509;//_th[3]//encoder[4]
			des_joint_torque_[joint_name_to_id_["r_knee"]-1]         = torque[2]/149.795*227.509;//_th[2]//encoder[6]
			des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]    = -torque[1]/149.795*227.509;//_th[1]//encoder[8]
			des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]   	 = -torque[0]/149.795*227.509;//_th[0]///encoder[10]
			
			des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]      = -torque[6]/149.795*227.509;
			des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]     = -torque[7]/149.795*227.509;
			des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]    = torque[8]/149.795*227.509;//-0.4;//
			des_joint_torque_[joint_name_to_id_["l_knee"]-1]         = torque[9]/149.795*227.509;//0;//
			des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]    = -torque[10]/149.795*227.509;
			des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]     = torque[11]/149.795*227.509;
			
			// des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//_th[5]//encoder[0]
			// des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]     = 0;//_th[4]//encoder[2]
			// des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]    = 0;//_th[3]//encoder[4]
			// des_joint_torque_[joint_name_to_id_["r_knee"]-1]         = 0;//_th[2]//encoder[6]
			// des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]    = 0;//_th[1]//encoder[8]
			// des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]   = 0;//_th[0]///encoder[10]
			
			// des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;
			// des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]     = 0;
			// des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]    = 0.4;
			// des_joint_torque_[joint_name_to_id_["l_knee"]-1]         = 0;
			// des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]  = 0;
			// des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]   = 0;
#endif
			// pre_force_R[0]=forceXr;
			// pre_force_R[1]=forceYr;
			// pre_force_R[2]=forceZr;
		
			// pre_force_L[0]=forceXl;
			// pre_force_L[1]=forceYl;
			// pre_force_L[2]=forceZl;
		
		
			// pre_torque_R[0]=torquexr;
			// pre_torque_R[1]=torqueyr;
			// pre_torque_R[2]=torquezr;
		
			// pre_torque_L[0]=torquexl;
			// pre_torque_L[1]=torqueyl;
			// pre_torque_L[2]=torquezl;
		
			// pre_force_F_R[0]=force_F_R[0];
			// pre_force_F_R[1]=force_F_R[1];
			// pre_force_F_R[2]=force_F_R[2];
		
			// pre_force_F_L[0]=force_F_L[0];
			// pre_force_F_L[1]=force_F_L[1];
			// pre_force_F_L[2]=force_F_L[2];
		
		
			// pre_torque_F_R[0]= torque_F_R[0];
			// pre_torque_F_R[1]= torque_F_R[1];
			// pre_torque_F_R[2]= torque_F_R[2];
		
			// pre_torque_F_L[0]= torque_F_L[0];
			// pre_torque_F_L[1]= torque_F_L[1];
			// pre_torque_F_L[2]= torque_F_L[2];

			// if(torqueStandingCount==2){
             
			// 	force_F_R[0]=forceXr;
			// 	force_F_R[1]=forceYr;
			// 	force_F_R[2]=forceZr;
			
			// 	force_F_L[0]=forceXl;
			// 	force_F_L[1]=forceYl;
			// 	force_F_L[2]=forceZl;
			
			
			// 	torque_F_R[0]=torquexr;
			// 	torque_F_R[1]=torqueyr;
			// 	torque_F_R[2]=torquezr;
			
			// 	torque_F_L[0]=torquexl;
			// 	torque_F_L[1]=torqueyl;
			// 	torque_F_L[2]=torquezl;
		
			// }
		
			// else if(torqueStandingCount>=3){
			// 	LPF(10,pre_force_R,pre_force_F_R,force_F_R);
			// 	LPF(10,pre_force_L,pre_force_F_L,force_F_L);
			
			// 	LPF(10,pre_torque_R,pre_torque_F_R,torque_F_R);
			// 	LPF(10,pre_torque_L,pre_torque_F_L,torque_F_L);
			// }
		
			// else{
		
			// 	force_F_R[0]=forceXr;
			// 	force_F_R[1]=forceYr;
			// 	force_F_R[2]=forceZr;
			
			// 	force_F_L[0]=forceXl;
			// 	force_F_L[1]=forceYl;
			// 	force_F_L[2]=forceZl;
			
			
			// 	torque_F_R[0]=torquexr;
			// 	torque_F_R[1]=torqueyr;
			// 	torque_F_R[2]=torquezr;
			
			// 	torque_F_L[0]=torquexl;
			// 	torque_F_L[1]=torqueyl;
			// 	torque_F_L[2]=torquezl;
			// }

			// if(i>(tq_standing/2)+1){
			// 	ps_sensing_count++;
			// 	ps_RforceZ_avg +=forceZr;
			// 	ps_RtorqueX_avg+=torquexr;
			// 	ps_RtorqueY_avg+=torqueyr;
			// 	ps_LforceZ_avg +=forceZl;
			// 	ps_LtorqueX_avg+=torquexl;
			// 	ps_LtorqueY_avg+=torqueyl;
			// }
			// ps_RforceZ_last =forceZr;
			// ps_RtorqueX_last=torquexr;
			// ps_RtorqueY_last=torqueyr;
			// ps_LforceZ_last =forceZl;
			// ps_LtorqueX_last=torquexl;
			// ps_LtorqueY_last=torqueyl;

			torqueStandingCount++;
			loopcount++;
			ROS_INFO("torque standing loopcount = %d",loopcount);
			if(torqueStandingCount==tq_standing)
			{
				_sn=0;
				torqueStandingCount++;
			}
		}
		
		else if(torqueStandCount<tq_stand)//torque mode standing
		{
			_th_encoder[0] = -curr_joint_pos_[joint_name_to_id_["r_ank_roll"] - 1];
			_th_encoder[1] = -curr_joint_pos_[joint_name_to_id_["r_ank_pitch"] - 1];
			_th_encoder[2] = curr_joint_pos_[joint_name_to_id_["r_knee"] - 1];
			_th_encoder[3] = curr_joint_pos_[joint_name_to_id_["r_hip_pitch"] - 1];
			_th_encoder[4] = curr_joint_pos_[joint_name_to_id_["r_hip_roll"] - 1];
			_th_encoder[5] = curr_joint_pos_[joint_name_to_id_["r_hip_yaw"] - 1];
			_th_encoder[6] = -curr_joint_pos_[joint_name_to_id_["l_hip_yaw"] - 1];
			_th_encoder[7] = -curr_joint_pos_[joint_name_to_id_["l_hip_roll"] - 1];
			_th_encoder[8] = curr_joint_pos_[joint_name_to_id_["l_hip_pitch"] - 1];
			_th_encoder[9] = curr_joint_pos_[joint_name_to_id_["l_knee"] - 1];
			_th_encoder[10] = -curr_joint_pos_[joint_name_to_id_["l_ank_pitch"] - 1];
			_th_encoder[11] = curr_joint_pos_[joint_name_to_id_["l_ank_roll"] - 1];

#if initPoseCheck
			if(_sn==0)
			{
				// _th_torque[0]	=_th_encoder[0];
				// _th_torque[1]	=_th_encoder[1];
				// _th_torque[2]	=_th_encoder[2];
				// _th_torque[3]	=_th_encoder[3];
				// _th_torque[4]	=_th_encoder[4];
				// _th_torque[5]	=_th_encoder[5];
				// _th_torque[6]	=_th_encoder[6];
				// _th_torque[7]	=_th_encoder[7];
				// _th_torque[8]	=_th_encoder[8];
				// _th_torque[9]	=_th_encoder[9];
				// _th_torque[10]	=_th_encoder[10];
				// _th_torque[11]	=_th_encoder[11];

				_th_torque[0]	=0.0;
				_th_torque[1]	=0.0;
				_th_torque[2]	=0.0;
				_th_torque[3]	=0.0;
				_th_torque[4]	=0.0;
				_th_torque[5]	=0.0;
				_th_torque[6]	=0.0;
				_th_torque[7]	=0.0;
				_th_torque[8]	=0.0;
				_th_torque[9]	=-1.57;
				_th_torque[10]	=0.0;
				_th_torque[11]	=0.0;
			}

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

#if GravityCompensationRNE	
			// _th_encoder[0] =0;
			// _th_encoder[1] = 0;
			// _th_encoder[2] =0;
			// _th_encoder[3] = 0;
			// _th_encoder[4] = 0;
			// _th_encoder[5] = 0;
			// _th_encoder[6] = 0;
			// _th_encoder[7] = 0;
			// _th_encoder[8] = 0;
			// _th_encoder[9] = 1.57;
			// _th_encoder[10] = 0;
			// _th_encoder[11] = 0;

			// _th_encoder[0] = -curr_joint_pos_[0];
			// _th_encoder[1] = -curr_joint_pos_[1];
			// _th_encoder[2] = curr_joint_pos_[2];
			// _th_encoder[3] = curr_joint_pos_[3];
			// _th_encoder[4] = curr_joint_pos_[4];
			// _th_encoder[5] = curr_joint_pos_[5];
			// _th_encoder[6] = -curr_joint_pos_[6];
			// _th_encoder[7] = -curr_joint_pos_[7];
			// _th_encoder[8] = curr_joint_pos_[8];
			// _th_encoder[9] = curr_joint_pos_[9];
			// _th_encoder[10] = -curr_joint_pos_[10];
			// _th_encoder[11] = curr_joint_pos_[11];

			printf("curr_joint_pos_[%d]=%lf\n",joint_name_to_id_["l_knee"]-1,curr_joint_pos_[joint_name_to_id_["l_knee"]-1]);

			// _th_encoder[0] =0;
			// _th_encoder[1] = 0;
			// _th_encoder[2] =0;
			// _th_encoder[3] = 0;
			// _th_encoder[4] = 0;
			// _th_encoder[5] = 0;
			// _th_encoder[6] = 0;
			// _th_encoder[7] = 0;
			// _th_encoder[8] = 0;
			// _th_encoder[9] = curr_joint_pos_[9];
			// _th_encoder[10] = 0;
			// _th_encoder[11] = 0;



			forwardkinematics(_th_encoder);

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
#else
		forwardkinematics(_th_torque);		
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

#if (CTMOn==0)		
			_kv=0;
			_kp=0;
#endif
			// // double Tr_anklePitch=0.015;//0;//0.02;//
			// // double _kv_anklePitch=2*(1.8/Tr_anklePitch);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// // double _kp_anklePitch=(1.8/Tr_anklePitch)*(1.8/Tr_anklePitch);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
			// // double Tr_ankleRoll=0.01;//0;//0.02;//
			// // double _kv_ankleRoll=2*(1.8/Tr_ankleRoll);//0;//2*(1.8/Tr);//180;//2*(1.8/Tr);
			// // double _kp_ankleRoll=(1.8/Tr_ankleRoll)*(1.8/Tr_ankleRoll);//0;//(1.8/Tr)*(1.8/Tr);//8100;//(1.8/Tr)*(1.8/Tr);
			_error_dth_1=vectorminus121_121(Mdth_d,Mdth);
			_error_dth=productmatrix121_d(_kv,_error_dth_1);
			// // _error_dth.x[1][0]=_error_dth.x[1][0]/_kv*_kv_anklePitch;
			// // _error_dth.x[10][0]=_error_dth.x[10][0]/_kv*_kv_anklePitch;
			// // _error_dth.x[0][0]=_error_dth.x[0][0]/_kv*_kv_ankleRoll;
			// // _error_dth.x[11][0]=_error_dth.x[11][0]/_kv*_kv_ankleRoll;

			_error_th_1=vectorminus121_121(_theta_p,_th_current_n);
			_error_th=productmatrix121_d(_kp,_error_th_1);
			// // _error_th.x[1][0]=_error_th.x[1][0]/_kp*_kp_anklePitch;
			// // _error_th.x[10][0]=_error_th.x[10][0]/_kp*_kp_anklePitch;
			// // _error_th.x[0][0]=_error_th.x[0][0]/_kp*_kp_ankleRoll;
			// // _error_th.x[11][0]=_error_th.x[11][0]/_kp*_kp_ankleRoll;

			Mddth =vectorplus121_121_121(Mddth_d,_error_dth,_error_th);
			
			//RNE AEÂ¡Â¾a AÂ¢ÃÂ¡ÃC
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
#if GravityCompensationRNE
					wr_m2_1=0;
#endif
					wr_m2 = productmatrix11_31(wr_m2_1, ta_zr);//3x1


					RNEwr.x[0][k + 1] = vectorplus31_31(productwr, wr_m2); // 3x1


					domegar = RNEdwr.x[0][k];//3x1
					productdwr1 = productmatrix33_31(rotationr, domegar);//3x1

					cproductdwr1 = crossproductmatrix31_31(productwr, wr_m2);//3x1


					ta_zr=ta.x[0][5-k];
					dwr_m2_1=Mddth.x[5-k][0];
#if GravityCompensationRNE
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

			//RNE EE init
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
#if GravityCompensationRNE
					wl_m2_1=0;
#endif
					wl_m2 = productmatrix11_31(wl_m2_1, ta_zl);//3x1

					RNEwl.x[0][k + 1] = vectorplus31_31(productwl, wl_m2); // w(i+1)

					domegal = RNEdwl.x[0][k];//3x1
					productdwl1 = productmatrix33_31(rotationl, domegal);//3x1

					cproductdwl1 = crossproductmatrix31_31(productwl, wl_m2);//3x1

					ta_zl=ta.x[0][k+6];//3x1
					dwl_m2_1=Mddth.x[k+6][0];//double
#if GravityCompensationRNE
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
			
			if(loopcount%10==0)
			{
				printf("th_d[0]=%lf, th_d[1]=%lf,th_d[2]=%lf,th_d[3]=%lf,th_d[4]=%lf,th_d[5]=%lf,th_d[6]=%lf,th_d[7]=%lf,th_d[8]=%lf,th_d[9]=%lf,th_d[10]=%lf,th_d[11]=%lf",
				_th_torque[0],_th_torque[1],_th_torque[2],_th_torque[3],_th_torque[4],_th_torque[5],
				_th_torque[6],_th_torque[7],_th_torque[8],_th_torque[9],_th_torque[10],_th_torque[11]);
				printf("\n-torque[0]= %lf,-torque[1]= %lf,torque[2]= %lf,torque[3]= %lf, torque[4]= %lf,torque[5]= %lf",
				-torque[0],-torque[1],torque[2],torque[3],torque[4],torque[5]);
				printf("\ntorque[11]= %lf,-torque[10]= %lf,torque[9]= %lf, torque[8]= %lf,-torque[7]= %lf,-torque[6]= %lf",
				torque[11],-torque[10],torque[9],torque[8],-torque[7],-torque[6]);
				printf("torque stand loopcount = %d",loopcount);
			}
			// printf("torque[9]L Knee pitch=%lf\n",torque[9]);
			// printf("curr_joint_pos_[l_knee]=%lf",curr_joint_pos_[joint_name_to_id_["l_knee"]-1]);
			// printf("\n");
			
			// printf("\n-torque[0]= %lf,-torque[1]= %lf,torque[2]= %lf,torque[3]= %lf, torque[4]= %lf,torque[5]= %lf",
			// -torque[0],-torque[1],torque[2],torque[3],torque[4],torque[5]);
			// printf("\ntorque[11]= %lf,-torque[10]= %lf,torque[9]= %lf, torque[8]= %lf,-torque[7]= %lf,-torque[6]= %lf",
			// torque[11],-torque[10],torque[9],torque[8],-torque[7],-torque[6]);

			// des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//torque[5];//_th[5]//encoder[0]
			// des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]     = 0;//torque[4];//_th[4]//encoder[2]
			// des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]    = 0;//torque[3];//_th[3]//encoder[4]
			// des_joint_torque_[joint_name_to_id_["r_knee"]-1]         = 0;//torque[2];//_th[2]//encoder[6]
			// des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]    = 0;//-torque[1];//_th[1]//encoder[8]
			// des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]   	 = 0;//-torque[0];//_th[0]///encoder[10]
			
			// des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;//-torque[6];
			// des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]     = 0;//-torque[7];
			// des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]    = 0;//torque[8]/149.795*227.509;//-0.4;//
			// des_joint_torque_[joint_name_to_id_["l_knee"]-1]         = torque[9]/149.795*227.509;//0;//
			// des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]    = 0;//-torque[10];
			// des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]     = 0;//torque[11];

			des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]      = torque[5]/149.795*227.509;//_th[5]//encoder[0]
			des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]     = torque[4]/149.795*227.509;//_th[4]//encoder[2]
			des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]    = torque[3]/149.795*227.509;//_th[3]//encoder[4]
			des_joint_torque_[joint_name_to_id_["r_knee"]-1]         = torque[2]/149.795*227.509;//_th[2]//encoder[6]
			des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]    = -torque[1]/149.795*227.509;//_th[1]//encoder[8]
			des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]   	 = -torque[0]/149.795*227.509;//_th[0]///encoder[10]
			
			des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]      = -torque[6]/149.795*227.509;
			des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]     = -torque[7]/149.795*227.509;
			des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]    = torque[8]/149.795*227.509;//-0.4;//
			des_joint_torque_[joint_name_to_id_["l_knee"]-1]         = torque[9]/149.795*227.509;//0;//
			des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]    = -torque[10]/149.795*227.509;
			des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]     = torque[11]/149.795*227.509;

#if torqueModeOn
			jointTorqueLimitCheck(0.5);
#endif

			torqueStandCount++;
			loopcount++;
			_sn++;
			
#if debugFprintOn
			for(int i=0; i<12; i++)
				wpg<<_th[i]<<" ";
			for(int i=0; i<12; i++)
				wpg<<curr_joint_pos_[i]<<" ";
			wpg<<endl;
#endif
			if(torqueStandCount==tq_standing)
			{
				_sn=0;
				torqueStandCount++;
			}
		}
		else //(loopcount >= tq_stand)&& (loopcount>=tq_standing) WPG
		{
			double _dds= 0.1257;
			//while (1) {
			if (phase == DSP_R || phase == DSP_L) {
				phaseflag = 1;
			}
			else {
				phaseflag = 0;
			}
			

			i_th_encoder[0]  = -curr_joint_pos_[0];
			i_th_encoder[1]  = -curr_joint_pos_[1];
			i_th_encoder[2]  = curr_joint_pos_[2];
			i_th_encoder[3]  = curr_joint_pos_[3];
			i_th_encoder[4]  = curr_joint_pos_[4];
			i_th_encoder[5]  = curr_joint_pos_[5];
			i_th_encoder[6]  = -curr_joint_pos_[6];
			i_th_encoder[7]  = -curr_joint_pos_[7];
			i_th_encoder[8]  = curr_joint_pos_[8];
			i_th_encoder[9]  = curr_joint_pos_[9];
			i_th_encoder[10] = -curr_joint_pos_[10];
			i_th_encoder[11] = curr_joint_pos_[11];

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
			// printf("------------------------------------------------------------------------\n");
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
      		}
			if(_i>2){
				_xcom_CPT=-_px+_xcom_CPT_global;
				_ycom_CPT=-_py+_ycom_CPT_global;
				// _xcom_CPT_v=-_xcom_CPT_v;
				// _ycom_CPT_v=-_ycom_CPT_v;
			}
			

			if (_i == 2) {
				_y_ter = -L3 * (1 - dspf_coeff);
				if(onlySSPon==1)
				{
					_y_ter = -L3;//only ssp
				}
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
			
			dsp_x_a0 = dsp_x_i;
			dsp_x_a1 = dsp_dx_i;
			dsp_x_a2 = 3 / pow(_s[_i + 1][3], 2) * (dsp_x_f - dsp_x_i) - 2 / _s[_i + 1][3] * dsp_dx_i - dsp_dx_f / _s[_i + 1][3];
			dsp_x_a3 = -2 / pow(_s[_i + 1][3], 3) * (dsp_x_f - dsp_x_i) + 1 / pow(_s[_i + 1][3], 2) * (dsp_dx_f + dsp_dx_i);
            
			dsp_x_a0_nominal = dsp_x_i_nominal;
			dsp_x_a1_nominal = dsp_dx_i_nominal;
			dsp_x_a2_nominal = 3 / pow(_s[_i + 1][3], 2) * (dsp_x_f_nominal - dsp_x_i_nominal) - 2 / _s[_i + 1][3] * dsp_dx_i_nominal - dsp_dx_f_nominal / _s[_i + 1][3];
			dsp_x_a3_nominal = -2 / pow(_s[_i + 1][3], 3) * (dsp_x_f_nominal - dsp_x_i_nominal) + 1 / pow(_s[_i + 1][3], 2) * (dsp_dx_f_nominal + dsp_dx_i_nominal);
            
			ndsp_x_a0=dsp_x_i;
            ndsp_x_a1=dsp_dx_i;
            ndsp_x_a2=0;
            ndsp_x_a3=(20*(dsp_x_f - dsp_x_i)-(8*dsp_dx_f+12*dsp_dx_i)* _s[_i + 1][3])/(2*pow(_s[_i + 1][3], 3));
            ndsp_x_a4=(30*(dsp_x_i - dsp_x_f)+(14*dsp_dx_f+16*dsp_dx_i)*_s[_i + 1][3])/(2*pow(_s[_i + 1][3], 4));
			ndsp_x_a5=(12*(dsp_x_f - dsp_x_i)-(6*dsp_dx_f+6*dsp_dx_i)*_s[_i + 1][3])/(2*pow(_s[_i + 1][3], 5));
			
			dsp_y_a0 = dsp_y_i;
			dsp_y_a1 = dsp_dy_i;
			dsp_y_a2 = 3 / pow(_s[_i + 1][3], 2) * (dsp_y_f - dsp_y_i) - 2 / _s[_i + 1][3] * dsp_dy_i - dsp_dy_f / _s[_i + 1][3];
			dsp_y_a3 = -2 / pow(_s[_i + 1][3], 3) * (dsp_y_f - dsp_y_i) + 1 / pow(_s[_i + 1][3], 2) * (dsp_dy_f + dsp_dy_i);
			
			dsp_y_a0_nominal = dsp_y_i_nominal;
			dsp_y_a1_nominal = dsp_dy_i_nominal;
			dsp_y_a2_nominal = 3 / pow(_s[_i + 1][3], 2) * (dsp_y_f_nominal - dsp_y_i_nominal) - 2 / _s[_i + 1][3] * dsp_dy_i_nominal - dsp_dy_f_nominal / _s[_i + 1][3];
			dsp_y_a3_nominal = -2 / pow(_s[_i + 1][3], 3) * (dsp_y_f_nominal - dsp_y_i_nominal) + 1 / pow(_s[_i + 1][3], 2) * (dsp_dy_f_nominal + dsp_dy_i_nominal);
			
			_swingrot_time = (_s[_i + 1][4] + _s[_i][4]) / (int)(_s[_i + 1][2] / samplingtime);

			_baserot_time = _s[_i][4] / (int)(_s[_i + 1][2] / samplingtime);

			_i = _i + 1;
		}

    	if (_s[_step_length + 1][5] + samplingtime / 2 < _t) {// Â¢Â¬Â¢ÃAoÂ¢Â¬Â¡Â¤ Â¡ÃEAÂ¨Ã¶
			// printf("---------------------------LAST STEP START------------------------------\n");
			// printf("------------------------------------------------------------------------\n");
#if debugPrintOn			
			ROS_INFO("last step");
#endif			
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
    	else if (_i % 2 == 0) {// Â¢Â¯AÂ¢Â¬Â¡ÃÂ©Ã¶Â©Â¬ AoAo Â¢Â¯Â¨Â­Â©Ã¶Â©Â¬ Â¨Ã¶Â¨Â¬AÂ¢Ã§
			if ((_t - _s[_i - 1][5]) > _s[_i][3] + samplingtime / 2) {
                // printf("------------------------------------------------------------------------\n");
				// printf("------------------------right foot supporting---------------------------\n");
				// printf("------------------------------------------------------------------------\n");
				Tr=Tr_walk_ssp;
				_kv=2*(1.8/Tr);
				_kp=(1.8/Tr)*(1.8/Tr);
        		footLYAccLpf=0, footRYAccLpf=0;
				DSPDelCheck=0;
				phase = SSP_R;
        		_baserot = _baserot + _baserot_time;
				_lsr = _lsr + _swingrot_time;
				_theta = _theta + _swingtime;

				_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
				_ycom = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
				_xcom_v = (-_x_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _vx_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_ycom_v = (_y_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _vy_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_xcom_acc = (-_x_ter / pow(_Tc, 2)) * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + (_vx_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
				_ycom_acc = (_y_ter / pow(_Tc, 2)) * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - (_vy_ter / _Tc) * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc);
                pre_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc));
				pre_ycom = cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3]) - samplingtime) / _Tc));
                _x_des = _px + _xcom;
				_y_des = _py + _ycom;
				_lsx_des = (cos(rot_th) * _s[_i - 1][0] - sin(rot_th) * pow(-1.0, _i - 1.0) * _s[_i - 1][1] + cos(rot_th + _s[_i - 1][4]) * _s[_i][0] - sin(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) / (2 * pi) * (_theta - sin(_theta)) + _ppx;
				_lsy_des = (_py + (sin(rot_th + _s[_i - 1][4]) * _s[_i][0] + cos(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) - _ppy) / (2 * pi) * (_theta - sin(_theta)) + _ppy;
				_lsz_des = _stepheight / 2 * (1 - cos(_theta));

				_dd_lsx_des = (cos(rot_th) * _s[_i - 1][0] - sin(rot_th) * pow(-1.0, _i - 1.0) * _s[_i - 1][1] + cos(rot_th + _s[_i - 1][4]) * _s[_i][0] - sin(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) / (2 * pi) * pow(_dds, 2)*(sin(_theta));
				_dd_lsy_des = (_py + (sin(rot_th + _s[_i - 1][4]) * _s[_i][0] + cos(rot_th + _s[_i - 1][4]) * pow(-1.0, _i) * _s[_i][1]) - _ppy) / (2 * pi) * pow(_dds, 2)*(sin(_theta));
				_dd_lsz_des = _stepheight / 2 * (cos(_theta))*pow(_dds, 2);

        
        		_targetxcom = _x_des;
				_targetycom = _y_des;
				targetXDsp=_targetxcom;
				targetYDsp=_targetycom;
      		}
        else {
			// printf("------------------------------------------------------------------------\n");
			// printf("--------------------------------right foot DSP--------------------------\n");
			// printf("------------------------------------------------------------------------\n");
			Tr=Tr_walk_dsp;
			_kv=2*(1.8/Tr);
			_kp=(1.8/Tr)*(1.8/Tr);
			phase = DSP_R;
			fslipInitCheck=0;

			_dsp_x_des = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5]) + dsp_x_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5]), 3);
			_dsp_y_des = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5]) + dsp_y_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5]), 3);
			_xcom = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5]) + dsp_x_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5]), 3);
			_ycom = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5]) + dsp_y_a2 * pow((_t - _s[_i - 1][5]), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5]), 3);
			
			_xcom_v = dsp_x_a1 + 2 * dsp_x_a2 *(_t - _s[_i - 1][5] ) + 3 * dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 2);
			_ycom_v = dsp_y_a1 + 2 * dsp_y_a2 * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 2);
			_xcom_v_nominal = dsp_x_a1_nominal + 2 * dsp_x_a2_nominal *(_t - _s[_i - 1][5]) + 3 * dsp_x_a3_nominal * pow((_t - _s[_i - 1][5]), 2);
			_ycom_v_nominal = dsp_y_a1_nominal + 2 * dsp_y_a2_nominal * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3_nominal * pow((_t - _s[_i - 1][5]), 2);
			_xcom_acc = 2 * dsp_x_a2 + 6 * dsp_x_a3 * (_t - _s[_i - 1][5] );
			_ycom_acc = 2 * dsp_y_a2 + 6 * dsp_y_a3 * (_t - _s[_i - 1][5] );
				
      
				
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
				
		}
	}

    else {     // Â¢Â¯Â¨Â­Â©Ã¶Â©Â¬ AoAo, Â¢Â¯AÂ¢Â¬Â¡ÃÂ©Ã¶Â©Â¬ Â¨Ã¶Â¨Â¬AÂ¢Ã§(AÂ©Ã¶ Â¨Ã¶Â¨Â¬AÂ¢Ã§)
		if ((_t - _s[_i - 1][5]) > _s[_i][3] + samplingtime / 2) {
			// printf("\n-------------------------left foot supporting---------------------------");
			// printf("\n------------------------------------------------------------------------");
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
			_baserot = _baserot + _baserot_time;

			_rsr = _rsr + _swingrot_time;
			_theta = _theta + _swingtime;
			
			_xcom = cos(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) - sin(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
			_ycom = sin(rot_th) * (-_x_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) + _Tc * _vx_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc)) + cos(rot_th) * (_y_ter * cosh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc) - _Tc * _vy_ter * sinh((_t - (_s[_i - 1][5] + _s[_i][3])) / _Tc));
			
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

			_targetxcom = _x_des;
			_targetycom = _y_des;
      	}
		else {
			// printf("\n------------------------------------------------------------------------\n");
			// printf("--------------------------------left foot DSP---------------------------\n");
			// printf("------------------------------------------------------------------------\n");	
			Tr=Tr_walk_dsp;
			_kv=2*(1.8/Tr);
			_kp=(1.8/Tr)*(1.8/Tr);

			phase = DSP_L;
			fslipInitCheck=0;
			_dsp_x_des = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5] ) + dsp_x_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 3);
			_dsp_y_des = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5] ) + dsp_y_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 3);
			_xcom = dsp_x_a0 + dsp_x_a1 * (_t - _s[_i - 1][5] ) + dsp_x_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 3);
			_ycom = dsp_y_a0 + dsp_y_a1 * (_t - _s[_i - 1][5] ) + dsp_y_a2 * pow((_t - _s[_i - 1][5] ), 2) + dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 3);
			_xcom_v = dsp_x_a1 + 2 * dsp_x_a2 *(_t - _s[_i - 1][5] ) + 3 * dsp_x_a3 * pow((_t - _s[_i - 1][5] ), 2);
			_ycom_v = dsp_y_a1 + 2 * dsp_y_a2 * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3 * pow((_t - _s[_i - 1][5] ), 2);
			_xcom_v_nominal = dsp_x_a1_nominal + 2 * dsp_x_a2_nominal * (_t - _s[_i - 1][5] ) + 3 * dsp_x_a3_nominal * pow((_t - _s[_i - 1][5] ), 2);
			_ycom_v_nominal = dsp_y_a1_nominal + 2 * dsp_y_a2_nominal * (_t - _s[_i - 1][5] ) + 3 * dsp_y_a3_nominal * pow((_t - _s[_i - 1][5] ), 2);
			
			_xcom_acc = 2 * dsp_x_a2 + 6 * dsp_x_a3 * (_t - _s[_i - 1][5]);
			_ycom_acc = 2 * dsp_y_a2 + 6 * dsp_y_a3 * (_t - _s[_i - 1][5]);

			
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

    	if(inverse_type==0)
			{
				
				for (_j = 0; _j < 1000; _j++) {//inverse B->L
					inversekinematicsBtoL(_th);
					forwardkinematics(_th);
					//forwardkinematics(_th, TR6E, TR56, TR45, TR34, TR23, TR12, TRB1, TL6E, TL56, TL45, TL34, TL23, TL12, TLB1);
					if (sqrt(err[0] * err[0] + err[1] * err[1] + err[2] * err[2] + err[3] * err[3] + err[4] * err[4] + err[5] * err[5]) < 0.01) {
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

			_th_encoder[0] = -curr_joint_pos_[0];
			_th_encoder[1] = -curr_joint_pos_[1];
			_th_encoder[2] = curr_joint_pos_[2];
			_th_encoder[3] = curr_joint_pos_[3];
			_th_encoder[4] = curr_joint_pos_[4];
			_th_encoder[5] = curr_joint_pos_[5];
			_th_encoder[6] = -curr_joint_pos_[6];
			_th_encoder[7] = -curr_joint_pos_[7];
			_th_encoder[8] = curr_joint_pos_[8];
			_th_encoder[9] = curr_joint_pos_[9];
			_th_encoder[10] = -curr_joint_pos_[10];
			_th_encoder[11] = curr_joint_pos_[11];

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

			Mddth =vectorplus121_121_121(Mddth_d,_error_dth,_error_th);


			//RNE AEÂ¡Â¾a AÂ¢ÃÂ¡ÃC 
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



			//RNE AEÂ¡Â¾a AÂ¢ÃÂ¡ÃC 
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
      
     _sn=_sn + 1;
     double torque[12]={torque1,torque2,torque3,torque4,torque5,torque6,torque7,torque8,torque9,torque10,torque11,torque12};
     int checktime;
#if (torqueModeOn==1)
			des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//_th[5]//encoder[0]
			des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]     = 0;//_th[4]//encoder[2]
			des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]    = 0;//_th[3]//encoder[4]
			des_joint_torque_[joint_name_to_id_["r_knee"]-1]         = 0;//_th[2]//encoder[6]
			des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]    = 0;//_th[1]//encoder[8]
			des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]   = 0;//_th[0]///encoder[10]
			
			des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;
			des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]     = 0;
			des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]    = -0.4;
			des_joint_torque_[joint_name_to_id_["l_knee"]-1]         = 0;
			des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]  = 0;
			des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]   = 0;

			

#elif (torqueModeOn==0)	
			
			// des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//6
			// des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]     = 0.006594;//5
			// des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]    = 0.710126;//4
			// des_joint_pos_[joint_name_to_id_["r_knee"]-1]         = -1.619199;//3
			// des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]    = -0.2;//2
			// des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]   = -0.006594;//1

			// des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;//7
			// des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]     = -0.006594;//8
			// des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]    = -0.710126;//9
			// des_joint_pos_[joint_name_to_id_["l_knee"]-1]         = 1.619199;//10
			// des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  = 0.2;//11
			// des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]   = 0.006594;//12*/

			// position mod walking
			 des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]      = _th[5];//6
			 des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]     = _th[4];//5
			 des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]    = _th[3];//4
			 des_joint_pos_[joint_name_to_id_["r_knee"]-1]         = _th[2];//3
			 des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]    = -_th[1];//2
			 des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]   = -_th[0];//1

			 des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]      = -_th[6];//7
			 des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]     = -_th[7];//8
			 des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]    = _th[8];//9
			 des_joint_pos_[joint_name_to_id_["l_knee"]-1]         = _th[9];//10
			 des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  = -_th[10];//11
			 des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]   = _th[11];//12

			// des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//6
			// des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]     = 0.006594;//5
			// des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]    = 0.710126;//4
			// des_joint_pos_[joint_name_to_id_["r_knee"]-1]         = -1.619199;//3
			// des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]    = -0.2;//2
			// des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]   = -0.006594;//1

			// des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;//7
			// des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]     = -0.006594;//8
			// des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]    = -0.710126;//9
			// des_joint_pos_[joint_name_to_id_["l_knee"]-1]         = 1.619199;//10
			// des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  = 0.2;//11
			// des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]   = 0.006594;//12

	#if debugPrintOn	
			for(int i =0;i<6;i++)
			{
				// ROS_INFO("th[%d]=%lf",checktime,_th[checktime]);
				printf("th[%d]=%lf",i,_th[i]);
			}
			ROS_INFO(" ");
			for(int j=11;j>5;j--)
			{
				printf("th[%d]=%lf",j,_th[j]);
			}
			ROS_INFO(" ");
			ROS_INFO("_i=%d , _sn = %d",_i,_sn);
	#endif			
			// for(checktime=0;checktime<12;checktime++)
			// {
			// 	ROS_INFO("encoder[%d]=%f",checktime,curr_joint_pos_[checktime]);
			// }
#endif
			//ROS_INFO("_step_length/0.008= %lf",_step_length/0.008);
				// printf("\n time  = %lf", walkingDataCount*samplingtime);
			
			
			// des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//_th[5]//encoder[0]
			// des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]     = 0.006594;//_th[4]//encoder[2]
			// des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]    = 0.710126;//_th[3]//encoder[4]
			// des_joint_pos_[joint_name_to_id_["r_knee"]-1]         = -1.619199;//_th[2]//encoder[6]
			// des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]    = -0.2;//_th[1]//encoder[8]
			// des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]   = -0.006594;//_th[0]///encoder[10]
			
			// des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;//_th[6]//encoder[1]
			// des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]     = -0.006594;//_th[7]//encoder[3]
			// des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]    = -0.710126;//_th[8]//encoder[5]
			// des_joint_pos_[joint_name_to_id_["l_knee"]-1]         = 1.619199;//_th[9]//encoder[7]
			// des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  = 0.2;//_th[10]//encoder[9]
			// des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]   = 0.006594;//_th[11]//encoder[11]

			// des_joint_pos_[joint_name_to_id_["r_hip_yaw"]-1]      = 0.0;//_th[5]//encoder[0]
			// des_joint_pos_[joint_name_to_id_["r_hip_roll"]-1]     = 0.0;//_th[4]//encoder[2]
			// des_joint_pos_[joint_name_to_id_["r_hip_pitch"]-1]    = 0.0;//_th[3]//encoder[4]
			// des_joint_pos_[joint_name_to_id_["r_knee"]-1]         = 0.0;//_th[2]//encoder[6]
			// des_joint_pos_[joint_name_to_id_["r_ank_pitch"]-1]    = 0.0;//_th[1]//encoder[8]
			// des_joint_pos_[joint_name_to_id_["r_ank_roll"]-1]   = 0.0;//_th[0]///encoder[10]
			
			// des_joint_pos_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;//_th[6]//encoder[1]
			// des_joint_pos_[joint_name_to_id_["l_hip_roll"]-1]     = 0.0;//_th[7]//encoder[3]
			// des_joint_pos_[joint_name_to_id_["l_hip_pitch"]-1]    = 0.0;//_th[8]//encoder[5]
			// des_joint_pos_[joint_name_to_id_["l_knee"]-1]         = 0.0;//_th[9]//encoder[7]
			// des_joint_pos_[joint_name_to_id_["l_ank_pitch"]-1]  = 0.0;//_th[10]//encoder[9]
			// des_joint_pos_[joint_name_to_id_["l_ank_roll"]-1]   = 0.0;//_th[11]//encoder[11]
			
#if (torqueModeOn==0)			
			jointLimitCheck();
#elif torqueModeOn
			jointTorqueLimitCheck(0.5);
#endif
			
#if debugFprintOn
			for(int i=0; i<12; i++)
				wpg<<_th[i]<<" ";
			for(int i=0; i<12; i++)
				wpg<<curr_joint_pos_[i]<<" ";
			wpg<<endl;
#endif

			_t = _t + samplingtime;
			loopcount++;
			//ROS_INFO("walking loopcount = %d",loopcount);
			//ROS_INFO("_t=%lf,_px=%lf,_ppx=%lf,_py=%lf,_ppy_%lf",_t,_px,_ppx,_py,_ppy);
			//ROS_INFO("_rsx_des=%lf,_rsy_des=%lf,_rsz_des=%lf",_rsx_des,_rsy_des,_rsz_des);
			//ROS_INFO("_lsx_des=%lf,_lsy_des=%lf,_lsz_des=%lf",_lsx_des,_lsy_des,_lsz_des);
			// ROS_INFO("time:%lf,samplingtime=%lf",_t,samplingtime);
			// ROS_INFO("loopcount:%d",loopcount);

#if debugPrintOn		
		 ROS_INFO("Calc Time: %f", time_duration);
#endif		 
			//ROS_INFO("Calc Time: %f", time_duration.toSec());
			// ROS_INFO("walking_initalize_:%d",walking_initialize_);
			// ROS_INFO("control_type:%d",control_type_);
			// ROS_INFO("walking_phase_:%d",walking_phase_);
			// ROS_INFO("balance_type_:%d",balance_type_);
			// ROS_INFO("is_moving_:%d",is_moving_);
			//fclose(thetacompare);
			
			if(loopcount==tq_standing+tq_stand+(((_step_length-1)/samplingtime)*(dsptime+ssptime)+(0.2/samplingtime)))
			{ 
				
					ROS_INFO("CONTROL ENd _i=%d , _sn = %d",_i,_sn);
				//	goal_initialize_ = false;
				// is_balancing_ = false;
				// joint_control_initialize_   = false;
				// wholebody_initialize_       = false;
				walking_initialize_         = false;
				control_type_ = NONE;
				walking_phase_ = DSP;
				balance_type_ = OFF;	
				is_moving_    = false;
				balance_control_initialize_ = false;
				_i=0;
				
				des_joint_torque_[joint_name_to_id_["r_hip_yaw"]-1]      = 0;//_th[5]//encoder[0]
				des_joint_torque_[joint_name_to_id_["r_hip_roll"]-1]     = 0;//_th[4]//encoder[2]
				des_joint_torque_[joint_name_to_id_["r_hip_pitch"]-1]    = 0;//_th[3]//encoder[4]
				des_joint_torque_[joint_name_to_id_["r_knee"]-1]         = 0;//_th[2]//encoder[6]
				des_joint_torque_[joint_name_to_id_["r_ank_pitch"]-1]    = 0;//_th[1]//encoder[8]
				des_joint_torque_[joint_name_to_id_["r_ank_roll"]-1]   = 0;//_th[0]///encoder[10]

				des_joint_torque_[joint_name_to_id_["l_hip_yaw"]-1]      = 0;
				des_joint_torque_[joint_name_to_id_["l_hip_roll"]-1]     = 0;
				des_joint_torque_[joint_name_to_id_["l_hip_pitch"]-1]    = 0;
				des_joint_torque_[joint_name_to_id_["l_knee"]-1]         = 0;
				des_joint_torque_[joint_name_to_id_["l_ank_pitch"]-1]  = 0;
				des_joint_torque_[joint_name_to_id_["l_ank_roll"]-1]   = 0;

				
				
	#if debugFprintOn
				wpg.close();
	#endif

			}
		}
    }
    else
	{
      ROS_INFO("WARNING!! Check Walking intialize");
      /*
		is_moving_ = false;
        is_foot_step_2d_ = false;
        walking_control_->finalize();
        control_type_ = NONE;
        walking_phase_ = DSP;
        balance_type_ == OFF;
        control_type_ = NONE;
        walking_initialize_ = false;
        */
    }
    
  }
  else if (control_type_ == OFFSET_CONTROL)
  {
	ROS_INFO("control_type_ == OFFSET_CONTROL");
    initOffsetControl();
    calcOffsetControl();
  }

  //  calcRobotPose();

  if (balance_type_ == ON)
  {
	// ROS_INFO("balance_type_ == ON");
    
    // initBalanceControl();
    // calcBalanceControl();

    // if (setBalanceControl() == false)
    // {
    //   is_moving_ = false;
    //   is_balancing_ = false;
    //   is_foot_step_2d_ = false;

    //   balance_type_ = OFF;
    //   control_type_ = NONE;

    //   resetBodyPose();

    //   ROS_INFO("[FAIL] Task Space Control");
    // }
    
  }

  //setFeedbackControl();
  
/*
  for (int i=0; i<number_of_joints_; i++)
    des_joint_pos_to_robot_[i] += balance_angle[i];
*/
  sensor_msgs::JointState goal_joint_msg;
  geometry_msgs::PoseStamped pelvis_pose_msg;
  robotis_controller_msgs::OnlineWalkingParam msg_walkingparam;

  goal_joint_msg.header.stamp = ros::Time::now();
  pelvis_pose_msg.header.stamp = ros::Time::now();

  pelvis_pose_msg.pose.position.x = des_body_pos_[0];
  pelvis_pose_msg.pose.position.y = des_body_pos_[1];
  pelvis_pose_msg.pose.position.z = des_body_pos_[2] - 0.0907;

  pelvis_pose_msg.pose.orientation.x = des_body_Q_[0];
  pelvis_pose_msg.pose.orientation.y = des_body_Q_[1];
  pelvis_pose_msg.pose.orientation.z = des_body_Q_[2];
  pelvis_pose_msg.pose.orientation.w = des_body_Q_[3];

  /*----- set joint data -----*/
  for (std::map<std::string, robotis_framework::DynamixelState *>::iterator state_iter = result_.begin();
       state_iter != result_.end(); state_iter++)
  {
    std::string joint_name = state_iter->first;
        result_[joint_name]->goal_position_ = des_joint_pos_[joint_name_to_id_[joint_name]-1];
		result_[joint_name]->goal_torque_ = des_joint_torque_[joint_name_to_id_[joint_name]-1];
    // result_[joint_name]->goal_position_ = des_joint_pos_to_robot_[joint_name_to_id_[joint_name]-1];
	
    goal_joint_msg.name.push_back(joint_name);
	// if(torqueModeOn==0)
    	goal_joint_msg.position.push_back(des_joint_pos_[joint_name_to_id_[joint_name]-1]);
	// else if(torqueModeOn==1)
	// 	goal_joint_msg.torque.push_back(des_joint_torque_[joint_name_to_id_[joint_name]-1]);
  }
  msg_walkingparam.msgtest=(int)present_control_mode;

 end = ros::Time::now().toSec();
 time_duration=end-begin;
  pelvis_pose_pub_.publish(pelvis_pose_msg);
  goal_joint_state_pub_.publish(goal_joint_msg);
  online_walking_param_pub_.publish(msg_walkingparam);
}

void OnlineWalkingModule::stop()
{
  for (int i=0; i<number_of_joints_; i++)
  {
    des_joint_pos_[i]   = 0.0;
    des_joint_vel_[i]   = 0.0;
    des_joint_accel_[i] = 0.0;
  }

  goal_initialize_ = false;

  is_moving_    = false;
  is_balancing_ = false;

  joint_control_initialize_   = false;
  wholebody_initialize_       = false;
  walking_initialize_         = false;
  balance_control_initialize_ = false;

  control_type_ = NONE;

  return;
}

bool OnlineWalkingModule::isRunning()
{
  return is_moving_;
}

void OnlineWalkingModule::publishStatusMsg(unsigned int type, std::string msg)
{
  robotis_controller_msgs::StatusMsg status;
  status.header.stamp = ros::Time::now();
  status.type         = type;
  status.module_name  = "Wholebody";
  status.status_msg   = msg;

  status_msg_pub_.publish(status);
}

bool OnlineWalkingModule::getJointPoseCallback(op3_online_walking_module_msgs::GetJointPose::Request &req,
                                           op3_online_walking_module_msgs::GetJointPose::Response &res)
{
  for (int i=0; i<number_of_joints_; i++)
  {
    res.pose.pose.name.push_back(joint_name_[i]);
    res.pose.pose.position.push_back(des_joint_pos_[i]);
  }

  return true;
}

bool OnlineWalkingModule::getKinematicsPoseCallback(op3_online_walking_module_msgs::GetKinematicsPose::Request &req,
                                                op3_online_walking_module_msgs::GetKinematicsPose::Response &res)
{
  std::string group_name = req.name;

  geometry_msgs::Pose msg;

  if (group_name == "body")
  {
    msg.position.x = des_body_pos_[0];
    msg.position.y = des_body_pos_[1];
    msg.position.z = des_body_pos_[2];

    msg.orientation.x = des_body_Q_[0];
    msg.orientation.y = des_body_Q_[1];
    msg.orientation.z = des_body_Q_[2];
    msg.orientation.w = des_body_Q_[3];
  }
  else if (group_name == "left_leg")
  {
    msg.position.x = des_l_leg_pos_[0];
    msg.position.y = des_l_leg_pos_[1];
    msg.position.z = des_l_leg_pos_[2];

    msg.orientation.x = des_l_leg_Q_[0];
    msg.orientation.y = des_l_leg_Q_[1];
    msg.orientation.z = des_l_leg_Q_[2];
    msg.orientation.w = des_l_leg_Q_[3];
  }
  else if (group_name == "right_leg")
  {
    msg.position.x = des_r_leg_pos_[0];
    msg.position.y = des_r_leg_pos_[1];
    msg.position.z = des_r_leg_pos_[2];

    msg.orientation.x = des_r_leg_Q_[0];
    msg.orientation.y = des_r_leg_Q_[1];
    msg.orientation.z = des_r_leg_Q_[2];
    msg.orientation.w = des_r_leg_Q_[3];
  }

  res.pose.pose = msg;

  return true;
}

bool OnlineWalkingModule::definePreviewMatrix()
{
  std::vector<double_t> K;
  K.push_back(739.200064);
  K.push_back(24489.822984);
  K.push_back(3340.410380);
  K.push_back(69.798325);

  preview_response_K_ = K;
  preview_response_K_row_ = 1;
  preview_response_K_col_ = 4;

  std::vector<double_t> P;
  P.push_back(33.130169);
  P.push_back(531.738962);
  P.push_back(60.201291);
  P.push_back(0.327533);
  P.push_back(531.738962);
  P.push_back(10092.440286);
  P.push_back(1108.851055);
  P.push_back(7.388990);
  P.push_back(60.201291);
  P.push_back(1108.851055);
  P.push_back(130.194694);
  P.push_back(0.922502);
  P.push_back(0.327533);
  P.push_back(7.388990);
  P.push_back(0.922502);
  P.push_back(0.012336);

  preview_response_P_ = P;
  preview_response_P_row_ = 4;
  preview_response_P_col_ = 4;

  return true;
}



