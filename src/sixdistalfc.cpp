#include "sixdistalfc.h"
#include <math.h>
#include"kaanh.h"
#include <algorithm>
#include"robotconfig.h"
#include"sixdistaldynamics.h"
#include"jointdynamics.h"
#include <vector>
//using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;
using namespace CONFIG;
using namespace sixDistalDynamicsInt;
using namespace JointDynamicsInt;
/// \brief

robotconfig robotDemo;
sixdistaldynamics sixDistalMatrix;
jointdynamics JointMatrixFT;


std::vector<double> PositionList_vec(6 * SampleNum);
auto PositionList = PositionList_vec.data();
std::vector<double> SensorList_vec(6 * SampleNum);
auto SensorList = SensorList_vec.data();

double ForceToMeng = 0;
double TimeToMeng = 0;

double Vol2FTCoef[36]={0.000133, 	-0.029834, 	0.000031, 	0.029925, 	-0.000006, 	-0.000034,
                       0.000046, 	-0.017260, 	0.000137, 	-0.017237, 	0.000217, 	0.034588,
                       -0.045122, 	0.000344, 	-0.045692, 	0.000084, 	-0.047397, 	-0.000011,
                       -0.001332, 	0.000020, 	0.001212, 	0.000010, 	-0.000098, 	-0.000024,
                       -0.000712, 	-0.000020, 	-0.000783, 	0.000034, 	0.001539, 	-0.000004,
                       -0.000004, 	0.001104, 	-0.000001, 	0.001079, 	0.000007, 	0.001099};


void crossVector(const double* a, const double* b, double* c)
{

    c[0] = a[1] * b[2] - b[1] * a[2];
    c[1] = -(a[0] * b[2] - b[0] * a[2]);
    c[2] = a[0] * b[1] - b[0] * a[1];

}


void GetATI(PlanTarget &target,double* FT)
{
    int32_t FTint[6],status_code,sample_counter;

    #ifdef UNIX
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);

    conSensor->slavePool().at(6).readPdo(0x6000, 0x01, &FTint[0] ,32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x02, &FTint[1], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x03, &FTint[2], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x04, &FTint[3], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x05, &FTint[4], 32);
    conSensor->slavePool().at(6).readPdo(0x6000, 0x06, &FTint[5], 32);
    conSensor->slavePool().at(6).readPdo(0x6010, 0x00, &status_code, 32);
    conSensor->slavePool().at(6).readPdo(0x6020, 0x00, &sample_counter, 32);
    #endif

    double ATIscale=1000000.0;
    FT[0] = FTint[0]/ ATIscale;
    FT[1] = FTint[1] / ATIscale;
    FT[2] = FTint[2]/ ATIscale;
    FT[3] = FTint[3]/ ATIscale;
    FT[4] = FTint[4]/ ATIscale;
    FT[5] = FTint[5] / ATIscale;

}

void GetYuLi(PlanTarget &target,double* FT)
{

    int16_t FTnum;
    float FTtemp[6];

    #ifdef UNIX
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x00, &FTnum ,16);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x01, &FTtemp[0] ,32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x02, &FTtemp[1], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x03, &FTtemp[2], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x04, &FTtemp[3], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x05, &FTtemp[4], 32);
    conSensor->slavePool().at(6).readPdo(0x6030, 0x06, &FTtemp[5], 32);
    #endif
   //std::cout<<FTtemp[0]<<"***"<<FTtemp[1]<<std::endl;
    for(int i=0;i<6;i++)
        FT[i]=FTtemp[i];

}

void BeiFu(PlanTarget &target,double* FT)
{

    int16_t FTint[6];
    double FTReal[6];

    #ifdef UNIX
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
    conSensor->slavePool().at(7).readPdo(0x6000, 0x11, &FTint[0] ,16);
    conSensor->slavePool().at(7).readPdo(0x6010, 0x11, &FTint[1], 16);
    conSensor->slavePool().at(7).readPdo(0x6020, 0x11, &FTint[2], 16);
    conSensor->slavePool().at(7).readPdo(0x6030, 0x11, &FTint[3], 16);
    conSensor->slavePool().at(8).readPdo(0x6000, 0x11, &FTint[4], 16);
    conSensor->slavePool().at(8).readPdo(0x6010, 0x11, &FTint[5], 16);
    #endif

    for (int i=0;i<6;i++)
    {
         FTReal[i] = FTint[i]*20.0 / 65536.0*1000.0;
    }

    s_mm(6, 1, 6, Vol2FTCoef, FTReal, FT);


}

double Premax_pos[6]={0};
double Premin_pos[6]={0};
double Premax_vel[6]={0};
double Premin_vel[6]={0};
void SetLimit(PlanTarget &target, double ratio)
{
    for(int i=0;i<6;i++)
    {
       Premax_pos[i]=target.controller->motionPool()[i].maxPos();
       Premin_pos[i]=target.controller->motionPool()[i].minPos();

       Premax_vel[i]=target.controller->motionPool()[i].maxVel();
       Premin_vel[i]=target.controller->motionPool()[i].minVel();

    }

        double max_pos[6]={2.96706/1, 1.57,1.0,  2.96706/3,1.57,6.28};
        double min_pos[6]={-2.96706/1,-0.5,     -1,  -2.96706/3,-1.57,-6.28};
        for(int i=0;i<6;i++)
        {
            target.controller->motionPool()[i].setMaxVel(target.controller->motionPool()[i].maxVel()/ratio);
            target.controller->motionPool()[i].setMinVel(target.controller->motionPool()[i].minVel()/ratio);
            target.controller->motionPool()[i].setMaxPos(max_pos[i]);
            target.controller->motionPool()[i].setMinPos(min_pos[i]);
        }

}

void ReSetLimit(PlanTarget &target)
{
    for(int i=0;i<6;i++)
    {
        target.controller->motionPool()[i].setMaxVel(Premax_vel[i]);
        target.controller->motionPool()[i].setMinVel(Premin_vel[i]);
        target.controller->motionPool()[i].setMaxPos(Premax_pos[i]);
        target.controller->motionPool()[i].setMinPos(Premin_pos[i]);
    }

}

void FT2World(PlanTarget &target,const double *FT, double *FmInWorld)
{
    double FT_YANG[6];
    FT_YANG[0] = -FT[0];FT_YANG[1] = -FT[1];FT_YANG[2] = FT[2];
    FT_YANG[3] = -FT[3];FT_YANG[4] = -FT[4];FT_YANG[5] = FT[5];

    double TransVector[16];
    target.model->generalMotionPool().at(0).getMpm(TransVector);
    double TransMatrix[4][4];
    for (int i = 0;i < 4;i++)
        for (int j = 0;j < 4;j++)
            TransMatrix[i][j] = TransVector[4 * i + j];

    double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
    double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
    double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };

    //FT[0] = 0;FT[1] = 0;FT[2] = 1;FT[3] = 0;FT[4] = 0;FT[5] = 0;
    FmInWorld[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
    FmInWorld[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
    FmInWorld[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
    FmInWorld[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
    FmInWorld[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
    FmInWorld[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];

}


void RepeatTrapezoidal(PlanTarget &target,double *begin_pjs,double *step_pjs)
{
    static bool flag[6] = { true,true,true,true,true,true };
    double PosLimit[6] = { 0.000,0.40,0,0,0,0};
    double NegLimit[6] = { -0.000,-0.40,0,0,0,0};
    static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.001,0.4,0.001,0.001,0.001,0.001 };
    static aris::Size t_count[6] = { 0 };
    static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1 };
    int temp[6] = { 0 };
    if(target.count==1)
    {
        for(int i=0;i<6;i++)
        {
            flag[i]=true;
            t_count[i]=0;
            CountOffsetPos[i]=1;
            CountOffsetNeg[i]=1;
        }
    }
    for (int i = 0;i < 6;i++)
    {

        if (flag[i])
        {
            if (step_pjs[i] < PosLimit[i])
            {
                aris::plan::moveAbsolute(target.count - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

                step_pjs[i] = step_pjs[i] + vArc[i];
            }
            //std::cout << vArc << "  ";
            if ((t_count[i] - (target.count - CountOffsetNeg[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetNeg[i] + 1)) > -0.5)
            {
                CountOffsetPos[i] = target.count;
                flag[i] = false;
                begin_pjs[i] = step_pjs[i];//target.model->motionPool()[i].mp();
            }


        }
        if (flag[i] == false)
        {
            if (step_pjs[i] > NegLimit[i])
            {
                aris::plan::moveAbsolute(target.count - CountOffsetPos[i] + 1, 0, begin_pjs[i] - NegLimit[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

                step_pjs[i] = step_pjs[i] - vArc[i];
            }

            temp[i] = t_count[i] - (target.count - CountOffsetPos[i] + 1);
            if ((t_count[i] - (target.count - CountOffsetPos[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetPos[i] + 1)) > -0.5)
            {
                CountOffsetNeg[i] = target.count;
                flag[i] = true;
                begin_pjs[i] = step_pjs[i];//target.model->motionPool()[i].mp();
            }

        }
    }

}


void OneOrderFilter(const double *raw, const double *new0, double *new1, double CutFreq)
{

     for (int j = 0; j < 6; j++)
     {
         double intDT = 0.001;
         new1[j] = new0[j] + intDT * (raw[j]-new0[j])*CutFreq;
     }
}

void SecondOrderFilter(const double *raw, const double new0[][3], double new1[][3],double CutFreq)
{

    for (int j = 0; j < 6; j++)
    {
        double A[3][3], B[3];//SHANGHAI DIANQI EXP
        //CutFreq = 85;
        A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
        A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
        A[2][0] = -CutFreq * CutFreq * CutFreq;
        A[2][1] = -2 * CutFreq * CutFreq;
        A[2][2] = -2 * CutFreq;
        B[0] = 0; B[1] = 0;
        B[2] = -A[2][0];
        double intDT = 0.001;
        new1[j][0] = new0[j][0] + intDT * (A[0][0] * new0[j][0] + A[0][1] * new0[j][1] + A[0][2] * new0[j][2] + B[0] * raw[j]);
        new1[j][1] = new0[j][1] + intDT * (A[1][0] * new0[j][0] + A[1][1] * new0[j][1] + A[1][2] * new0[j][2] + B[1] * raw[j]);
        new1[j][2] = new0[j][2] + intDT * (A[2][0] * new0[j][0] + A[2][1] * new0[j][1] + A[2][2] * new0[j][2] + B[2] * raw[j]);
    }
}

void q2dqt(double *ActPq, const double *DesPq, double *dqt, const double KPP)
{
    double dQuar[4] = { 0 };
    //姿态误差2
    double cos_theta = ActPq[3] * DesPq[3] + ActPq[4] * DesPq[4] + ActPq[5] * DesPq[5] + ActPq[6] * DesPq[6];
    if (cos_theta < 0)
    {
        ActPq[3] = -ActPq[3];
        ActPq[4] = -ActPq[4];
        ActPq[5] = -ActPq[5];
        ActPq[6] = -ActPq[6];
    }
    cos_theta = ActPq[3] * DesPq[3] + ActPq[4] * DesPq[4] + ActPq[5] * DesPq[5] + ActPq[6] * DesPq[6];

    cos_theta = std::max(-1.0, cos_theta);
    cos_theta = std::min(1.0, cos_theta);

    double theta = std::acos(cos_theta);
    double sin_theta = std::sin(theta);


    if (theta < 0.03)
    {
        dQuar[0] = DesPq[3] - ActPq[3];
        dQuar[1] = DesPq[4] - ActPq[4];
        dQuar[2] = DesPq[5] - ActPq[5];
        dQuar[3] = DesPq[6] - ActPq[6];
    }
    else
    {
        dQuar[0] = -(DesPq[3] * cos_theta*(-theta) + theta * ActPq[3]) / sin_theta;
        dQuar[1] = -(DesPq[4] * cos_theta*(-theta) + theta * ActPq[4]) / sin_theta;
        dQuar[2] = -(DesPq[5] * cos_theta*(-theta) + theta * ActPq[5]) / sin_theta;
        dQuar[3] = -(DesPq[6] * cos_theta*(-theta) + theta * ActPq[6]) / sin_theta;
    }

    double norm_dQuar=std::max(1e-7, std::sqrt(aris::dynamic::s_vv(4,dQuar,dQuar)));
    double unit_dQuar[4]={0};

    for(int i=0;i<4;i++)
        unit_dQuar[i]=dQuar[i]/norm_dQuar;

    for(int i=0;i<4;i++)
    {
        double dt = std::min(KPP*theta, 0.5);// protect angular velocity target, theta always positive
        dqt[i]=unit_dQuar[i]*dt;
    }
}


void dX2dTheta(PlanTarget &target, const double *dXX, double *dTheta)
{
    double PqEnd[7];
    double EndW[3], EndP[3], BaseV[3];
    target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0 };
	for (int i = 0;i < 6;i++)
		dX[i] = dXX[i];

    ///* Using Jacobian, TransMatrix from ARIS
    for (int i = 0;i < 3;i++)
        EndW[i] = dX[i + 3];

    for (int i = 0;i < 3;i++)
        EndP[i] = PqEnd[i];
    crossVector(EndP, EndW, BaseV);

    for (int i = 0;i < 3;i++)
        dX[i + 3] = dX[i + 3];
    for (int i = 0;i < 3;i++)
        dX[i] = dX[i] + BaseV[i];


    auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
    fwd.cptJacobi();
    double pinv[36];

    // 所需的中间变量，请对U的对角线元素做处理
    double U[36], tau[6];
    aris::Size p[6];
    aris::Size rank;

    // 根据 A 求出中间变量，相当于做 QR 分解 //
    // 请对 U 的对角线元素做处理
    s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-10);
    for (int i = 0;i < 6;i++)
        if (U[7 * i] >= 0)
            U[7 * i] = U[7 * i] + 0.001;
        else
            U[7 * i] = U[7 * i] - 0.001;
    // 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
    s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
    double tau2[6];
    s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

    // 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
    s_mm(6, 1, 6, pinv, dX, dTheta);




}


struct MoveXYZParam
{
	double damp[6];

};
auto MoveXYZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveXYZParam param;


	for (auto &p : params)
	{

		if (p.first == "damp")
		{
			for (int i = 0; i < 6; i++)
				param.damp[i] = std::stod(p.second);
		}

	}
	target.param = param;

      for(auto &option:target.mot_options) option|=
		Plan::USE_TARGET_POS |
#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;


}



auto MoveXYZ::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveXYZParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3];
    static double FT0[6], FT_be[6];

	// 访问主站 //
	auto controller = target.controller;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;
	

	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
    target.model->generalMotionPool().at(0).getMpm(TransVector);
    target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0.00001, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

    int16_t FTint[6];
    double FTReal[6],FT[6];
    auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	
    conSensor->slavePool().at(7).readPdo(0x6000, 0x11, &FTint[0] ,16);
    conSensor->slavePool().at(7).readPdo(0x6010, 0x11, &FTint[1], 16);
    conSensor->slavePool().at(7).readPdo(0x6020, 0x11, &FTint[2], 16);
    conSensor->slavePool().at(7).readPdo(0x6030, 0x11, &FTint[3], 16);
    conSensor->slavePool().at(8).readPdo(0x6000, 0x11, &FTint[4], 16);
    conSensor->slavePool().at(8).readPdo(0x6010, 0x11, &FTint[5], 16);

    for (int i=0;i<6;i++)
    {
         FTReal[i] = FTint[i]*20.0 / 65536.0*1000.0;
    }
	
    s_mm(6, 1, 6, Vol2FTCoef, FTReal, FT);
	


	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
			FT_be[j] = FT[j];
		}
	}


	for (int j = 0; j < 6; j++)
	{
        double A[3][3], B[3], CutFreq = 6;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = stateTor1[i][0];
	}
	double estFT[6] = { 0 };
    sixDistalMatrix.sixDistalCollision(RobotPosition, RobotVelocity, RobotAcceleration, TorqueSensor, sixDistalMatrix.estParasFT, estFT);

    // 获取当前起始点位置 //

    if (target.count == 1)
    {
        for (int j = 0; j < 6; j++)
        {
            FT0[j] = FT[j];
        }
    }

    if (target.count == 500)
    {
        for (int j = 0; j < 6; j++)
        {
            FT0[j] = stateTor1[j][0];
             //FT0[j] = FT[j];
        }
    }


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
        FT_KAI[i] = stateTor1[i][0] - FT0[i];
	}



	for (int i = 0; i < 3; i++)
	{
        if (abs(FT_KAI[i]) < 0.08)
			FT_KAI[i] = 0;
	}
	for (int i = 3; i < 6; i++)
	{
        if (abs(FT_KAI[i]) < 0.01)
			FT_KAI[i] = 0;
	}

	double FT_YANG[6];
    FT_YANG[0] = -FT_KAI[0];FT_YANG[1] = -FT_KAI[1];FT_YANG[2] = FT_KAI[2];
    FT_YANG[3] = -FT_KAI[3];FT_YANG[4] = -FT_KAI[4];FT_YANG[5] = FT_KAI[5];

	double FmInWorld[6];

	double TransMatrix[4][4];
	for (int i = 0;i < 4;i++)
		for (int j = 0;j < 4;j++)
			TransMatrix[i][j] = TransVector[4 * i + j];

	double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
	double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
	double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };

	//FT[0] = 0;FT[1] = 0;FT[2] = 1;FT[3] = 0;FT[4] = 0;FT[5] = 0;
	FmInWorld[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
	FmInWorld[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
	FmInWorld[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
	FmInWorld[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
	FmInWorld[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
	FmInWorld[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];



    dX[0] = 0 * FmInWorld[0] / 4000;
    dX[1] = 1 * FmInWorld[1] / 16000;
    dX[2] = 0 * FmInWorld[2] / 16000;
    dX[3] = 0 * FmInWorld[3] / 3000;
    dX[4] = 0 * FmInWorld[4] / 3000;
    dX[5] = 0 * FmInWorld[5] / 3000;




	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// 打印电流 //
	auto &cout = controller->mout();





	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";


    lout << FT[0] << ",";lout << FT[1] << ",";
    lout << FT[2] << ",";lout << FT[3] << ",";
    lout << FT[4] << ",";lout << FT[5] << ",";
	lout << stateTor0[0][0] << ",";lout << stateTor0[1][0] << ",";
	lout << stateTor0[2][0] << ",";lout << stateTor0[3][0] << ",";
	lout << stateTor0[4][0] << ",";lout << stateTor0[5][0] << ",";

	//lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
   // lout << dX[0] << ",";

   // lout << dX[0] << ",";
   // lout << FT[1] << ",";lout << FT[2] << ",";
   // lout << FT[3] << ",";lout << FT[4] << ",";
   // lout << FT[5] << ",";lout << FT[6] << ",";
	lout << std::endl;



	///* Using Jacobian, TransMatrix from ARIS
	for (int i = 0;i < 3;i++)
		EndW[i] = dX[i + 3];

	for (int i = 0;i < 3;i++)
		EndP[i] = PqEnd[i];
	crossVector(EndP, EndW, BaseV);

	for (int i = 0;i < 3;i++)
		dX[i + 3] = dX[i + 3];
	for (int i = 0;i < 3;i++)
		dX[i] = dX[i] + BaseV[i];


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
	fwd.cptJacobi();
	double pinv[36];

	// 所需的中间变量，请对U的对角线元素做处理
	double U[36], tau[6];
	aris::Size p[6];
	aris::Size rank;

	// 根据 A 求出中间变量，相当于做 QR 分解 //
	// 请对 U 的对角线元素做处理
	s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-10);
	for (int i = 0;i < 6;i++)
		if (U[7 * i] >= 0)
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
	}

	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}



    if (target.count % 300 == 0)
    {
        //for (int i = 0; i < 6; i++)
        {
            cout << stateTor1[2][0] << "**" << FT_KAI[2]<< "**" << FT0[2];
//cout << dX[0] << "**" << dX[1]<< "**" << dX[2]<<dX[3]<<dX[4]<<dX[5];
        }

        cout << std::endl;

    }


	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}
/*

	for (int i = 0; i < 6; i++)
	{
		stateDm0[i, 0] = stateDm1[i, 0];
		stateDm0[i, 1] = stateDm1[i, 1];
		stateDm0[i, 2] = stateDm1[i, 2];
		stateTor0[i, 0] = stateTor1[i, 0];
		stateTor0[i, 1] = stateTor1[i, 1];
		stateTor0[i, 2] = stateTor1[i, 2];
    }
*/


	return 150000000 - target.count;
}

MoveXYZ::MoveXYZ(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"mvXYZ\">"
		"	<GroupParam>"
		"		<Param name=\"damp\" default=\"1.0\"/>"
		"	</GroupParam>"
		"</Command>");


}



struct MoveDistalParam
{
	double A5P, A6P;
	double A5N, A6N, VEL, SensorType;

};
auto MoveDistal::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveDistalParam param;

	for (auto &p : params)
	{
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);
		if (p.first == "A5P")
			param.A5P = std::stod(p.second);
		if (p.first == "A5N")
			param.A5N = std::stod(p.second);
		if (p.first == "A6P")
			param.A6P = std::stod(p.second);
		if (p.first == "A6N")
			param.A6N = std::stod(p.second);
		if (p.first == "VEL")
			param.VEL = std::stod(p.second);

	}

	target.param = param;

     for(auto &option:target.mot_options) option|=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR|
        Plan::NOT_CHECK_ENABLE;


}
auto MoveDistal::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveDistalParam&>(target.param);

	static double begin_pjs[6];
	static double step_pjs[6];
	static double perVar = 0;
	static double ampVar = 0;
    static int CollectNum = 1;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
        CollectNum = 1;
		for (int i = 0; i < 6; i++)
		{
			begin_pjs[i] = target.model->motionPool()[i].mp();
			step_pjs[i] = target.model->motionPool()[i].mp();
		}
	}

    static bool flag[6] = {true,true,true,true,true,true};
    double PosLimit[6] = { 1,0.5,0.5,param.A5P,param.A6P};
    double NegLimit[6] = { -1,-0.5,-0.5,-1,param.A5N,param.A6N};
    double dTheta = 0.00001;
    double vel_base = 0.15 / 100;
    static double pArc[6], vArc[6], aArc[6], vArcMax[6] =  { param.VEL*vel_base,param.VEL*vel_base,param.VEL*vel_base,param.VEL*vel_base,param.VEL*vel_base,param.VEL*vel_base };
    static aris::Size t_count[6] = { 0 };

    static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1 };

	if (CollectNum < SampleNum)
	{
		for (int i = 0;i < 6;i++)
		{

			if (flag[i])
			{
				if (step_pjs[i] < PosLimit[i])
				{
					aris::plan::moveAbsolute(target.count - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

					step_pjs[i] = step_pjs[i] + vArc[i];
				}
				//std::cout << vArc << "  ";
				if ((t_count[i] - (target.count - CountOffsetNeg[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetNeg[i] + 1)) > -0.5)
				{
					CountOffsetPos[i] = target.count;
					flag[i] = false;
					begin_pjs[i] = step_pjs[i];
				}


			}
			if (flag[i] == false)
			{
				if (step_pjs[i] > NegLimit[i])
				{
					aris::plan::moveAbsolute(target.count - CountOffsetPos[i] + 1, 0, begin_pjs[i] - NegLimit[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

					step_pjs[i] = step_pjs[i] - vArc[i];
				}

				if ((t_count[i] - (target.count - CountOffsetPos[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetPos[i] + 1)) > -0.5)
				{
					CountOffsetNeg[i] = target.count;
					flag[i] = true;
					begin_pjs[i] = step_pjs[i];
				}

			}
			if (i == 4 || i == 5)
				target.model->motionPool().at(i).setMp(step_pjs[i]);
		}
	}
	else
	{
		for (int i = 0;i < 6;i++)
		{

			if (flag[i])
			{
				if (vArc[i] > 0.0001*1e-4)
				{
					vArc[i] = vArc[i] - 0.001*1e-4;
					step_pjs[i] = step_pjs[i] + vArc[i];
				}

			}
			if (flag[i] == false)
			{

				if (vArc[i] > 0.0001*1e-4)
				{
					vArc[i] = vArc[i] - 0.001*1e-4;
					step_pjs[i] = step_pjs[i] - vArc[i];
				}
			}
			if (i == 4 || i == 5)
				target.model->motionPool().at(i).setMp(step_pjs[i]);
		}
	}



	if (target.model->solverPool().at(1).kinPos())return -1;


    double FT[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);


    // 访问主站 //
    auto controller = target.controller;

	// 打印电流 //
	auto &cout = controller->mout();
	if (target.count % 100 == 0)
	{
		//for (int i = 0; i < 6; i++)
		{
            cout << FT[2]<< "  ";
			//cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
			//cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
		}
		//   cout << target.count << "  ";
		cout << std::endl;
	}

	auto &lout = controller->lout();

	if (target.count % 8 == 0 && CollectNum < SampleNum)
    {
        for (int i = 0; i < 6; i++)
        {
            PositionList[6 * (CollectNum - 1) + i] = target.model->motionPool()[i].mp();
            SensorList[6 * (CollectNum - 1) + i] = FT[i];
        }

        lout << target.count << ",";
        lout << PositionList[6 * (CollectNum - 1) + 0] << ",";lout << PositionList[6 * (CollectNum - 1) + 1] << ",";
        lout << PositionList[6 * (CollectNum - 1) + 2] << ",";lout << PositionList[6 * (CollectNum - 1) + 3] << ",";
        lout << PositionList[6 * (CollectNum - 1) + 4] << ",";lout << PositionList[6 * (CollectNum - 1) + 5] << ",";
        lout << SensorList[6 * (CollectNum - 1) + 0] << ",";lout << SensorList[6 * (CollectNum - 1) + 1] << ",";
        lout << SensorList[6 * (CollectNum - 1) + 2] << ",";lout << SensorList[6 * (CollectNum - 1) + 3] << ",";
        lout << SensorList[6 * (CollectNum - 1) + 4] << ",";lout << SensorList[6 * (CollectNum - 1) + 5] << ",";

        lout << std::endl;
        CollectNum = CollectNum + 1;
    }

	if (target.count % 8 == 0 && CollectNum > SampleNum - 1)
	{
		CollectNum = CollectNum + 1;

	}

    return (150+SampleNum) - CollectNum;
}


auto MoveDistal::collectNrt(aris::plan::PlanTarget &target)->void
{

    double estParas[GroupDim] = { 0 };
    double StatisError[6] = { 0,0,0,0,0,0 };
    auto controller = target.controller;
    auto &cout = controller->mout();
	 // auto &lout = controller->lout();
	std::cout << "collect" << std::endl;

    sixDistalMatrix.RLS(PositionList, SensorList, sixDistalMatrix.estParasFT, StatisError);
	//std::cout<<"collect"<<std::endl;
	for (int i = 0;i < GroupDim;i++)
        cout << sixDistalMatrix.estParasFT[i] << ",";

	std::cout << "*****************************Statictic Model Error*****************************************" << std::endl;
	for (int i = 0;i < 6;i++)
        cout << StatisError[i] << std::endl;

	std::vector<double> load_params, torque_error;
	load_params.resize(16, 0.0);
	torque_error.resize(6, 0.0);

	for (int i = 0;i < 16;i++)
		load_params[i] = sixDistalMatrix.estParasFT[i];
	for (int i = 0;i < 6;i++)
		torque_error[i] = StatisError[i];

	std::string calib_info = "Calibration Is Completed";

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("load_params", load_params));
	out_param.push_back(std::make_pair<std::string, std::any>("torque_error", torque_error));
	target.ret = out_param;
 
}


MoveDistal::MoveDistal(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvDistal\">"
		"	<GroupParam>"
        "		<Param name=\"SensorType\"default=\"-1.0\"/>"
        "		<Param name=\"A5P\"default=\"1.5\"/>"
        "		<Param name=\"A5N\" default=\"-1.5\"/>"
        "		<Param name=\"A6P\"default=\"2.0\"/>"
        "		<Param name=\"A6N\" default=\"-2.0\"/>"
        "		<Param name=\"VEL\" default=\"100\"/>"
		"	</GroupParam>"
		"</Command>");

}


struct MoveDistalSaveParam
{
	double P1, P2, P3, P4, P5, P6, P7, P8, P9, P10;
	double P11, P12, P13, P14, P15, P16;
};
auto MoveDistalSave::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveDistalSaveParam param;

	for (auto &p : params)
	{
		if (p.first == "P1")
			param.P1 = std::stod(p.second);
		if (p.first == "P2")
			param.P2 = std::stod(p.second);
		if (p.first == "P3")
			param.P3 = std::stod(p.second);
		if (p.first == "P4")
			param.P4 = std::stod(p.second);
		if (p.first == "P5")
			param.P5 = std::stod(p.second);
		if (p.first == "P6")
			param.P6 = std::stod(p.second);
		if (p.first == "P7")
			param.P7 = std::stod(p.second);
		if (p.first == "P8")
			param.P8 = std::stod(p.second);
		if (p.first == "P9")
			param.P9 = std::stod(p.second);
		if (p.first == "P10")
			param.P10 = std::stod(p.second);
		if (p.first == "P11")
			param.P11 = std::stod(p.second);
		if (p.first == "P12")
			param.P12 = std::stod(p.second);
		if (p.first == "P13")
			param.P13 = std::stod(p.second);
		if (p.first == "P14")
			param.P14 = std::stod(p.second);
		if (p.first == "P15")
			param.P15 = std::stod(p.second);
		if (p.first == "P16")
			param.P16 = std::stod(p.second);

	}

	target.param = param;
	double link_params[16] = { 0 };
	link_params[0] = param.P1;
	link_params[1] = param.P2;
	link_params[2] = param.P3;
	link_params[3] = param.P4;
	link_params[4] = param.P5;
	link_params[5] = param.P6;
	link_params[6] = param.P7;
	link_params[7] = param.P8;
	link_params[8] = param.P9;
	link_params[9] = param.P10;
	link_params[10] = param.P11;
	link_params[11] = param.P12;
	link_params[12] = param.P13;
	link_params[13] = param.P14;
	link_params[14] = param.P15;
	link_params[15] = param.P16;

	aris::core::Matrix mat0(1, GroupDim, link_params);
	if (target.model->variablePool().findByName("estParasFT") !=
		target.model->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(
			&*target.model->variablePool().findByName("estParasFT"))->data() = mat0;
	}
	else
	{
		target.model->variablePool().add<aris::dynamic::MatrixVariable>("estParasFT", mat0);
	}

	std::string calib_info = "Load Identification Is Completed";

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", calib_info));
	target.ret = out_param;

}
MoveDistalSave::MoveDistalSave(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"MoveDistalSave\">"
		"	<GroupParam>"
		"		<Param name=\"P1\"default=\"0.0\"/>"
		"		<Param name=\"P2\" default=\"0.0\"/>"
		"		<Param name=\"P3\"default=\"0.0\"/>"
		"		<Param name=\"P4\" default=\"0.0\"/>"
		"		<Param name=\"P5\"default=\"0.0\"/>"
		"		<Param name=\"P6\" default=\"0.0\"/>"
		"		<Param name=\"P7\"default=\"0.0\"/>"
		"		<Param name=\"P8\" default=\"0.0\"/>"
		"		<Param name=\"P9\"default=\"0.0\"/>"
		"		<Param name=\"P10\" default=\"0.0\"/>"
		"		<Param name=\"P11\"default=\"0.0\"/>"
		"		<Param name=\"P12\" default=\"0.0\"/>"
		"		<Param name=\"P13\"default=\"0.0\"/>"
		"		<Param name=\"P14\" default=\"0.0\"/>"
		"		<Param name=\"P15\"default=\"0.0\"/>"
		"		<Param name=\"P16\" default=\"0.0\"/>"
		"	</GroupParam>"
		"</Command>");

}



struct DistalTestParam
{
    double SensorType;
    double amplitude;

};
auto DistalTest::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    DistalTestParam param;

    for (auto &p : params)
    {
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);
        if (p.first == "amplitude")
            param.amplitude = std::stod(p.second);

    }

    target.param = param;

     for(auto &option:target.mot_options) option|=
        Plan::USE_TARGET_POS |
        //#ifdef WIN32
        Plan::NOT_CHECK_POS_MIN |
        Plan::NOT_CHECK_POS_MAX |
        Plan::NOT_CHECK_POS_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
        Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
        //#endif
        Plan::NOT_CHECK_VEL_MIN |
        Plan::NOT_CHECK_VEL_MAX |
        Plan::NOT_CHECK_VEL_CONTINUOUS |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR|
        Plan::NOT_CHECK_ENABLE;



    //读取动力学参数
    auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasFT"));
    for (int i = 0;i < GroupDim;i++)
        sixDistalMatrix.estParasFT[i] = mat0->data().data()[i];


}
auto DistalTest::executeRT(PlanTarget &target)->int
{
    auto &param = std::any_cast<DistalTestParam&>(target.param);

    static double begin_pjs[6];
    static double step_pjs[6];
    static double perVar = 0;
    static double ampVar = 0;
   static int CollectNum = 1;
    if (target.count < 1000)
    {
        ampVar = ampVar + param.amplitude / 1000;
    }
    // 获取当前起始点位置 //
    if (target.count == 1)
    {
        for (int i = 0; i < 6; i++)
        {
            begin_pjs[i] = target.model->motionPool()[i].mp();
            step_pjs[i] = target.model->motionPool()[i].mp();
        }
    } 

    static bool flag[6] = {true,true,true,true,true,true};
    double PosLimit[6] = { 0.5,0.6,0.4,1,1,1 };
    double NegLimit[6] = { -0.5,-0.6,-0.4,-1,-1,-1 };
    double dTheta = 0.00001;
    static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.05,0.05,0.05,0.05,0.05,0.15 };
    static aris::Size t_count[6] = { 0 };

    static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1 };


    for (int i = 0;i < 6;i++)
    {

        if (flag[i])
        {
            if (step_pjs[i] < PosLimit[i])
            {
                aris::plan::moveAbsolute(target.count - CountOffsetNeg[i] + 1, 0, PosLimit[i] - begin_pjs[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

                step_pjs[i] = step_pjs[i] + vArc[i];
            }
            //std::cout << vArc << "  ";
            if ((t_count[i] - (target.count - CountOffsetNeg[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetNeg[i] + 1)) > -0.5)
            {
                CountOffsetPos[i] = target.count;
                flag[i] = false;
                begin_pjs[i] = target.model->motionPool()[i].mp();
            }


        }
        if (flag[i] == false)
        {
            if (step_pjs[i] > NegLimit[i])
            {
                aris::plan::moveAbsolute(target.count - CountOffsetPos[i] + 1, 0, begin_pjs[i] - NegLimit[i], vArcMax[i] / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc[i], vArc[i], aArc[i], t_count[i]);

                step_pjs[i] = step_pjs[i] - vArc[i];
            }

            if ((t_count[i] - (target.count - CountOffsetPos[i] + 1)) < 0.5 && (t_count[i] - (target.count - CountOffsetPos[i] + 1)) > -0.5)
            {
                CountOffsetNeg[i] = target.count;
                flag[i] = true;
                begin_pjs[i] = target.model->motionPool()[i].mp();
            }

        }
        //if(i==4||i==5)
            target.model->motionPool().at(i).setMp(step_pjs[i]);
    }



    if (target.model->solverPool().at(1).kinPos())return -1;

    double FT[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);


    // 访问主站 //
    auto controller = target.controller;

    double CollisionFT[6],q[6],dq[6],ddq[6],ts[6];
    double omega = 0;
    for (int i = 0; i < 6; i++)
    {
        q[i]= target.model->motionPool()[i].mp();
        dq[i] =0;
        ddq[i] =0;
    }

    sixDistalMatrix.sixDistalCollision(q, dq, ddq, FT, sixDistalMatrix.estParasFT, CollisionFT);

    for (int j = 0; j < 6; j++)
    {
        FT[j]=FT[j]-CollisionFT[j];


    }

    static double FT0[6]={0};
    if (target.count == 1)
        for (int j = 0; j < 6; j++)
            FT0[j]=FT[j];

    for (int j = 0; j < 6; j++)
        FT[j]=FT[j]-FT0[j];


    // 打印电流 //
    auto &cout = controller->mout();
    if (target.count % 100 == 0)
    {
        //for (int i = 0; i < 6; i++)
        {
            //cout << CollisionFT[0]<< "  "<<CollisionFT[1]<<"  "<<CollisionFT[2];
            cout << FT[0]<< "  "<<FT[1]<<"  "<<FT[2];
            cout << FT0[0]<< "  "<<FT0[1]<<"  "<<FT0[2];
            //cout << "vel" << i + 1 << ":" << target.model->motionPool()[i].mv() << "  ";
            //cout << "cur" << i + 1 << ":" << target.model->motionPool()[i].ma() << "  ";
        }
        //   cout << target.count << "  ";
        cout << std::endl;
    }



    auto &lout = controller->lout();

    lout << target.count << ",";
    lout << CollisionFT[0] << ",";lout << CollisionFT[1] << ",";
    lout << CollisionFT[2] << ",";lout << CollisionFT[3] << ",";
    lout << CollisionFT[4] << ",";lout << CollisionFT[5] << ",";
    lout << FT[0] << ",";lout << FT[1] << ",";
    lout << FT[2] << ",";lout << FT[3] << ",";
    lout << FT[4] << ",";lout << FT[5] << ",";

    lout << std::endl;

    return 1000000 - CollectNum;
}


DistalTest::DistalTest(const std::string &name) :Plan(name)
{

    command().loadXmlStr(
        "<Command name=\"DistalTest\">"
        "	<GroupParam>"
        "		<Param name=\"SensorType\"default=\"20.0\"/>"
        "		<Param name=\"amplitude\" default=\"0.2\"/>"
        "	</GroupParam>"
        "</Command>");

}



struct MovePressureParam
{
	double PressF;


};

auto MovePressure::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;




}
auto MovePressure::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3];
    static double FT0[6], FT_be[6];
	static double SumFtErr[6];
	// 访问主站 //
	auto controller = target.controller;
    auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			SumFtErr[i] = 0;
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

    double FT[6];
	uint16_t FTnum;
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x00, &FTnum, 16);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x01, &FT[0], 32);  //Fx
	conSensor->slavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
	conSensor->slavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
	conSensor->slavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
	FT[0] = -FT[0];FT[3] = -FT[3];

	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
			FT_be[j] = FT[j];
		}
	}

	for (int j = 0; j < 6; j++)
	{
		if (abs(FT[j]) < 0.0001)
			FT[j] = FT_be[j];
	}


	for (int j = 0; j < 6; j++)
	{
		double A[3][3], B[3], CutFreq = 35;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}


	for (int i = 0; i < 3; i++)
	{
		if (FT_KAI[i] < 1.0&&FT_KAI[i]>0)
			FT_KAI[i] = FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-1.0)
			FT_KAI[i] = -FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	for (int i = 3; i < 6; i++)
	{
		if (FT_KAI[i] < 0.05&&FT_KAI[i]>0)
			FT_KAI[i] = 20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-0.05)
			FT_KAI[i] = -20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}


	double FT_YANG[6];
	FT_YANG[0] = FT_KAI[2];FT_YANG[1] = -FT_KAI[1];FT_YANG[2] = FT_KAI[0];
	FT_YANG[3] = FT_KAI[5];FT_YANG[4] = -FT_KAI[4];FT_YANG[5] = FT_KAI[3];

	double FmInWorld[6];

	double TransMatrix[4][4];
	for (int i = 0;i < 4;i++)
		for (int j = 0;j < 4;j++)
			TransMatrix[i][j] = TransVector[4 * i + j];

	double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
	double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
	double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };

	//FT[0] = 0;FT[1] = 0;FT[2] = 1;FT[3] = 0;FT[4] = 0;FT[5] = 0;
	FmInWorld[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
	FmInWorld[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
	FmInWorld[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
	FmInWorld[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
	FmInWorld[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
	FmInWorld[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];

	//for(int i=0;i<6;i++)
		//SumFtErr[i]=SumFtErr[i]+
	if (abs(FmInWorld[2]) > 2)
		SumFtErr[2] = SumFtErr[2] + (FmInWorld[2] - (5))*DT;

	dX[2] = 1 * (FmInWorld[2] - (5) + 0 * SumFtErr[2]) / 820000;
	dX[3] = 1 * (FmInWorld[3]) / 4000;
	dX[4] = 1 * (FmInWorld[4]) / 4000;
	dX[5] = 1 * (FmInWorld[5]) / 4000;

	/*//  SHANGHAI DIANQI EXP
	dX[0] = 1 * (FmInWorld[0]) / 40000;
	dX[1] = 1 * (FmInWorld[1]) / 40000;
	dX[2] = 1 * (FmInWorld[2]) / 40000;
	dX[3] = 1 * (FmInWorld[3]) / 4000;
	dX[4] = 1 * (FmInWorld[4]) / 4000;
	dX[5] = 1 * (FmInWorld[5]) / 4000;
*/


	if (target.count > 23000)
		dX[0] = -0.00001;
	if (target.count > 45000)
		dX[0] = 0.00001;
	if (target.count > 67000)
		dX[0] = -0.00001;
	if (target.count > 89000)
		dX[0] = 0.00001;
	if (target.count > 111000)
		dX[0] = -0.00001;
	if (target.count > 133000)
		dX[0] = 0.00001;

	if (target.count % 100 == 0)
	{

        cout << FmInWorld[2] << "***" << SumFtErr[2] << "***" << dX[4] << "***" << dX[5] << "***" << FT0[2] << std::endl;

		//cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";

	lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	lout << FmInWorld[0] << ",";lout << FmInWorld[1] << ",";
	lout << FmInWorld[2] << ",";lout << FmInWorld[3] << ",";
	lout << FmInWorld[4] << ",";
	// lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
	// lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
	 //lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";




	 //lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
	// lout << dX[0] << ",";

	// lout << dX[0] << ",";
	// lout << FT[1] << ",";lout << FT[2] << ",";
	// lout << FT[3] << ",";lout << FT[4] << ",";
	// lout << FT[5] << ",";lout << FT[6] << ",";
	lout << std::endl;



	///* Using Jacobian, TransMatrix from ARIS
	for (int i = 0;i < 3;i++)
		EndW[i] = dX[i + 3];

	for (int i = 0;i < 3;i++)
		EndP[i] = PqEnd[i];
	crossVector(EndP, EndW, BaseV);

	for (int i = 0;i < 3;i++)
		dX[i + 3] = dX[i + 3];
	for (int i = 0;i < 3;i++)
		dX[i] = dX[i] + BaseV[i];


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
	fwd.cptJacobi();
	double pinv[36];

	// 所需的中间变量，请对U的对角线元素做处理
	double U[36], tau[6];
	aris::Size p[6];
	aris::Size rank;

	// 根据 A 求出中间变量，相当于做 QR 分解 //
	// 请对 U 的对角线元素做处理
	s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-10);
	for (int i = 0;i < 6;i++)
		if (U[7 * i] >= 0)
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}

	for (int j = 0; j < 6; j++)
	{
		FT_be[j] = FT[j];
	}
	return 150000000 - target.count;

}

MovePressure::MovePressure(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvPre\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");

}


struct MovePressureToolYZParam
{
	double PressF;
    double SensorType;


};


auto MovePressureToolYZ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureToolYZParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);

	}

	target.param = param;

    target.option |=  Plan::USE_TARGET_POS;

    for(auto &option:target.mot_options) option|=
		Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
        Plan::NOT_CHECK_ENABLE;

    //读取动力学参数
    auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasFT"));
    for (int i = 0;i < GroupDim;i++)
        sixDistalMatrix.estParasFT[i] = mat0->data().data()[i];


}

auto MovePressureToolYZ::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureToolYZParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
	static double sT0[6][3], sT1[6][3];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;
    auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
	double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
	double CrossNormalZbase[3] = { 0 };

	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);
    NormalVector[0] = TransVector[2];NormalVector[1] = TransVector[6];NormalVector[2] = TransVector[10];

	crossVector(NormalVector, ZBase, CrossNormalZbase);
	CosNormalAng = NormalVector[2] / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
	SinNormalAng = sqrt(CrossNormalZbase[0] * CrossNormalZbase[0] + CrossNormalZbase[1] * CrossNormalZbase[1] + CrossNormalZbase[2] * CrossNormalZbase[2]) / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
	NormalAng = atan2(SinNormalAng, CosNormalAng);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

    double FTReal[6],FT[6];
    if(param.SensorType>0)
        GetATI(target,FTReal);
    else
        GetYuLi(target,FTReal);


	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}

    double CollisionFT[6],q[6],dq[6],ddq[6],ts[6];

    for (int i = 0; i < 6; i++)
    {
        q[i]= target.model->motionPool()[i].mp();
        dq[i] =0;
        ddq[i] =0;
    }

    sixDistalMatrix.sixDistalCollision(q, dq, ddq, FTReal, sixDistalMatrix.estParasFT, CollisionFT);



    for (int j = 0; j < 6; j++)
    {
        FT[j]=FTReal[j]-CollisionFT[j];

    }


	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}




    for (int j = 0; j < 3; j++)
	{
        double A[3][3], B[3], CutFreq = 105;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}

    for (int j = 3; j < 6; j++)
    {
        double A[3][3], B[3], CutFreq = 105;//SHANGHAI DIANQI EXP
        //CutFreq = 85;
        A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
        A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
        A[2][0] = -CutFreq * CutFreq * CutFreq;
        A[2][1] = -2 * CutFreq * CutFreq;
        A[2][2] = -2 * CutFreq;
        B[0] = 0; B[1] = 0;
        B[2] = -A[2][0];
        double intDT = 0.001;
        stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
        stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
        stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
    }



	double FT_KAI[6];

    if (target.count == 1)
    {
        for (int i = 0; i < 6; ++i)
        {
            FT0[i] = stateTor1[i][0];

        }
    }


	for (int i = 0; i < 6; i++)
	{
        FT_KAI[i] = stateTor1[i][0]-FT0[i];//In KAI Coordinate
	}





	for (int i = 0; i < 3; i++)
	{
		if (FT_KAI[i] < 1.0&&FT_KAI[i]>0)
			FT_KAI[i] = FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-1.0)
			FT_KAI[i] = -FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	for (int i = 3; i < 6; i++)
	{
        if (FT_KAI[i] < 0.2&&FT_KAI[i]>0)
            FT_KAI[i] = 5 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
        else if (FT_KAI[i]<0 && FT_KAI[i]>-0.2)
            FT_KAI[i] = -5 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}


double preF=-5;
	double dXpid[6] = { 0,0,0,0,0,0 };

    dXpid[2] = 1 * (FT_KAI[2] - (preF)) / 650000;
    dXpid[3] = 1 * (FT_KAI[3]) / 4000;
    dXpid[4] = 1 * (FT_KAI[4]) / 4000;
    dXpid[5] = 0 * (FT_KAI[5]) / 8000;


    double TangentArc[3] = { 0 };
    static double TangentArc0[3] = { 0 };
    static double TangentArc1[3] = { 0 };
    static double TangentArc2[3] = { 0 };
    static bool MoveDirection = true;
    static bool MoveDirectionT = true, MoveDirectionF = false;
    static bool MoveDirectionChange = false;
    static int StartCount = 12000;
    double CosTheta1, CosTheta2;


/*
    if (target.count > StartCount)
    {
        dXpid[2] = 1 * (FT_KAI[2] - (preF)) / 350000;
        dXpid[3] = 1 * (FT_KAI[3]) / 20000;
        dXpid[4] = 1 * (FT_KAI[4]) / 20000;
        dXpid[5] = 0 * (FT_KAI[5]) / 8000;
    }

*/
    if (target.count > StartCount&&MoveDirectionF == true)
    {
        dXpid[2] = 1 * (FT_KAI[2] - (preF)) / 650000;
        dXpid[3] = 0;
        dXpid[4] = 0;
        dXpid[5] = 0;
    }



	double FT_YANG[6];
    FT_YANG[0] = -dXpid[0];FT_YANG[1] = -dXpid[1];FT_YANG[2] = dXpid[2];
    FT_YANG[3] = -dXpid[3];FT_YANG[4] = -dXpid[4];FT_YANG[5] = dXpid[5];



	double TransMatrix[4][4];
	for (int i = 0;i < 4;i++)
		for (int j = 0;j < 4;j++)
			TransMatrix[i][j] = TransVector[4 * i + j];

	double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
	double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
	double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };

	//FT[0] = 0;FT[1] = 0;FT[2] = 1;FT[3] = 0;FT[4] = 0;FT[5] = 0;
    dX[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
    dX[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
    dX[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
    dX[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
    dX[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
    dX[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];


    FT_YANG[0] = -FT_KAI[0];FT_YANG[1] = -FT_KAI[1];FT_YANG[2] = FT_KAI[2];
    FT_YANG[3] = -FT_KAI[3];FT_YANG[4] = -FT_KAI[4];FT_YANG[5] = FT_KAI[5];


    double FmInWorld[6];
    FmInWorld[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
    FmInWorld[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
    FmInWorld[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
    FmInWorld[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
    FmInWorld[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
    FmInWorld[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];




    static double pArc, vArc, aArc, vArcMax = 0.03;
	aris::Size t_count;

   // double Square[4][3] = { {0,-0.04,0.73},
     //                       {0,-0.04,0.3},
     //                       {0,-0.25,0.3},
     //                       {0,-0.25,0.73}};

    double Square[4][3] = { {0,-0.06,0.35},
                            {0,-0.06,0.76},
                            {0,-0.27,0.76},
                            {0,-0.27,0.35}};

	static double MoveLength = 0;
    static double DecLength = 0.01, LengthT = 0.2, LengthF = 0.45;//LengthT>LengthF

	LengthT = sqrt((Square[0][1] - Square[1][1])*(Square[0][1] - Square[1][1]) + (Square[0][2] - Square[1][2])*(Square[0][2] - Square[1][2]));
	double CountFmax = sqrt((Square[2][1] - Square[1][1])*(Square[2][1] - Square[1][1]) + (Square[2][2] - Square[1][2])*(Square[2][2] - Square[1][2])) / LengthF;

CountFmax=4;
	double DecTime = 0, Dec = 0;
	static int count_offsetT = StartCount, count_offsetF = StartCount;
	static double vArcEndT = 0, vArcEndF = 0;
	static int CountT = 0, CountF = 0;

	double Ktemp, temp0, temp1;
	double CrossSurface[3] = { 0,1,0 };//YZ
	double ExtendSurface[3] = { 0,0,-1 };
	temp0 = Square[1][1] - Square[0][1];temp1 = Square[1][2] - Square[0][2];
	ExtendSurface[1] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[2] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[0] = 0;


	temp0 = Square[2][1] - Square[1][1];temp1 = Square[2][2] - Square[1][2];
	CrossSurface[1] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[2] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[0] = 0;





	if (target.count > StartCount&&MoveDirectionT == true && MoveDirectionF == false)
	{
		if (CountT % 2 == 0)
            MoveDirection = true;
		else
            MoveDirection = false;

		if (abs(NormalVector[0]) < 0.01)
		{
			TangentArc1[0] = ExtendSurface[0]; TangentArc1[1] = ExtendSurface[1]; TangentArc1[2] = ExtendSurface[2];

			TangentArc2[0] = ExtendSurface[0]; TangentArc2[1] = -ExtendSurface[1]; TangentArc2[2] = -ExtendSurface[2];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
			{
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];

			}

		}
		else
		{
			temp0 = ExtendSurface[1] * ExtendSurface[1] + ExtendSurface[2] * ExtendSurface[2];
			temp1 = (ExtendSurface[1] * NormalVector[1] + ExtendSurface[2] * NormalVector[2]) / NormalVector[0];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[1] = Ktemp * ExtendSurface[1];
			TangentArc1[2] = Ktemp * ExtendSurface[2];
			TangentArc1[0] = -(NormalVector[1] * TangentArc0[1] + NormalVector[2] * TangentArc0[2]) / NormalVector[0];

			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			temp0 = ExtendSurface[1] * ExtendSurface[1] + ExtendSurface[2] * ExtendSurface[2];
			temp1 = (ExtendSurface[1] * NormalVector[1] + ExtendSurface[2] * NormalVector[2]) / NormalVector[0];
			Ktemp = -1 / sqrt(temp0 + temp1 * temp1);

			TangentArc2[1] = Ktemp * ExtendSurface[1];
			TangentArc2[2] = Ktemp * ExtendSurface[2];
			TangentArc2[0] = -(NormalVector[1] * TangentArc0[1] + NormalVector[2] * TangentArc0[2]) / NormalVector[0];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];

		}


		if (MoveDirection)
			if (MoveLength < LengthT - DecLength)
			{
				aris::plan::moveAbsolute(target.count - count_offsetF + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndT = vArc;
			}
			else
			{

				vArc = vArcEndT - 1 * (DecLength - (LengthT - MoveLength)) / DecLength * vArcEndT;

				if (abs(vArc) < 0.0001)
				{
					MoveDirectionT = false;
					MoveDirectionF = true;
					count_offsetT = target.count;
					MoveDirectionChange = true;
					CountT = CountT + 1;
				}
			}

		if (!MoveDirection)
			if (MoveLength > (DecLength))
			{
				aris::plan::moveAbsolute(target.count - count_offsetF + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndF = vArc;
			}
			else
			{
				vArc = vArcEndF - 1 * (DecLength - MoveLength) / DecLength * vArcEndF;

				if (abs(vArc) < 0.0001)
				{
					MoveDirectionT = false;
					MoveDirectionF = true;
					count_offsetT = target.count;
					MoveDirectionChange = true;
					CountT = CountT + 1;
				}
			}

	}


	if (target.count > StartCount&&MoveDirectionT == false && MoveDirectionF == true && CountF < CountFmax)
	{
		if (CountF % 2 == 0)
		{
			MoveDirection = true;
		}
		else
		{
			MoveDirection = true;
		}
		if (abs(NormalVector[0]) < 0.01)
		{
			TangentArc1[0] = CrossSurface[0]; TangentArc1[1] = CrossSurface[1]; TangentArc1[2] = CrossSurface[2];
			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			for (int i = 0;i < 3;i++)
				TangentArc[i] = TangentArc1[i];
		}
		else
		{
			temp0 = CrossSurface[1] * CrossSurface[1] + CrossSurface[2] * CrossSurface[2];
			temp1 = (CrossSurface[1] * NormalVector[1] + CrossSurface[2] * NormalVector[2]) / NormalVector[0];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[1] = Ktemp * CrossSurface[1];
			TangentArc1[2] = Ktemp * CrossSurface[2];
			TangentArc1[0] = -(NormalVector[1] * TangentArc0[1] + NormalVector[2] * TangentArc0[2]) / NormalVector[0];

			for (int i = 0;i < 3;i++)
				TangentArc[i] = TangentArc1[i];

		}


		if (MoveDirection)
		{
			aris::plan::moveAbsolute(target.count - count_offsetT + 1, 0, LengthF, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
			vArc = vArc * 1000;
			vArcEndT = vArc;
		}

		if (abs(vArc) < 0.0001&&aArc < 0)
		{
			MoveDirectionT = true;
			MoveDirectionF = false;
			count_offsetF = target.count;
			MoveDirectionChange = true;
			CountF = CountF + 1;
		}


	}
	if (CountF > CountFmax - 1)
	{
		vArc = 0;
	}

//vArc=0;
	if (target.count > StartCount)
	{
		if (MoveDirection)
		{
            dX[0] = dX[0]+1 * vArc * TangentArc[0] / 1000;
            dX[1] = dX[1]+1*vArc * TangentArc[1] / 1000;
            dX[2] = dX[2]+1*vArc * TangentArc[2] / 1000;

		}
		else
		{
            dX[0] = dX[0]+1 * vArc * TangentArc[0] / 1000;
            dX[1] = dX[1]+1*vArc * TangentArc[1] / 1000;
            dX[2] = dX[2]+1*vArc * TangentArc[2] / 1000;
		}
		if (target.count > StartCount&&MoveDirectionT == true && MoveDirectionF == false)
			if (MoveDirection)
                MoveLength = MoveLength + sqrt(dX[0] * dX[0] + dX[2] * dX[2]);
			else
                MoveLength = MoveLength - sqrt(dX[0] * dX[0] + dX[2] * dX[2]);
	}





	//if(FT_KAI[2]<-12.5)
	  //  ForceToMeng =9.38;
   // else
/*
	for (int j = 0; j < 6; j++)
	{
		double A[3][3], B[3], CutFreq = 10;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		if(target.count > start&&target.count < (start+1*interval-StopInt))
		{
		if(FT_KAI[j]<-14)
			FT_KAI[j]=-10.59+sin(target.count);
		if(FT_KAI[j]>-7)
			FT_KAI[j]=-9.37+sin(target.count);
		}
		sT1[j][0] = sT0[j][0] + intDT * (A[0][0] * sT0[j][0] + A[0][1] * sT0[j][1] + A[0][2] * sT0[j][2] + B[0] * FT_KAI[j]);
		sT1[j][1] = sT0[j][1] + intDT * (A[1][0] * sT0[j][0] + A[1][1] * sT0[j][1] + A[1][2] * sT0[j][2] + B[1] * FT_KAI[j]);
		sT1[j][2] = sT0[j][2] + intDT * (A[2][0] * sT0[j][0] + A[2][1] * sT0[j][1] + A[2][2] * sT0[j][2] + B[2] * FT_KAI[j]);
	}*/

	ForceToMeng = sT1[2][0];
	if (ForceToMeng < -14)
		ForceToMeng = -11.59;
	if (ForceToMeng > -8)
		ForceToMeng = -9.37;

    //ForceToMeng = vArc;
	TimeToMeng = target.count / 1000.0;


	for (int j = 0; j < 6; j++)
	{
        if (dX[j] > 0.00025)
            dX[j] = 0.00025;
        if (dX[j] < -0.00025)
            dX[j] = -0.00025;

	}


/*
    if(PqEnd[2]>0.5&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00001;

    if(PqEnd[2]>0.53&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00006;
    if(PqEnd[2]>0.55&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00004;
    if(PqEnd[2]>0.6&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00002;


    if(PqEnd[2]>0.5&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00001;
    if(PqEnd[2]>0.53&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00006;
    if(PqEnd[2]>0.55&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00004;
    if(PqEnd[2]>0.6&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00002;
*/






     /* shuzhi
    if(PqEnd[2]>0.5&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00006;

    if(PqEnd[2]>0.53&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00008;
    if(PqEnd[2]>0.55&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00005;
    if(PqEnd[2]>0.6&&MoveDirectionT == true&&MoveDirection==true)
        dX[4]=0.00002;


    if(PqEnd[2]>0.5&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00006;
    if(PqEnd[2]>0.53&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00005;
    if(PqEnd[2]>0.55&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00008;
    if(PqEnd[2]>0.6&&MoveDirectionT == true&&MoveDirection==false)
        dX[4]=-0.00002;
    */




	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";
    //lout << vArc << endl;
	//lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	//lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
	//lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
   // lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";
    lout << FT[0] << ",";lout << FT[1] << ",";
    lout << FT[2] << ",";lout << FT[3] << ",";
    lout << FT[4] << ",";lout << FT[5] << ",";


     lout << CollisionFT[0] << ",";lout << CollisionFT[1] << ",";
     lout << CollisionFT[2] << ",";lout << CollisionFT[3] << ",";
     lout << CollisionFT[4] << ",";lout << CollisionFT[5] << ",";

     //lout << TangentArc[0] << ",";lout << TangentArc[1] << ",";
     //lout << TangentArc[2] << ",";

     lout << FTReal[0] << ",";lout << FTReal[1] << ",";
     lout << FTReal[2] << ",";lout << FTReal[3] << ",";
     lout << FTReal[4] << ",";lout << FTReal[5] << ",";

    lout << std::endl;



	///* Using Jacobian, TransMatrix from ARIS
	for (int i = 0;i < 3;i++)
		EndW[i] = dX[i + 3];

	for (int i = 0;i < 3;i++)
		EndP[i] = PqEnd[i];
	crossVector(EndP, EndW, BaseV);

	for (int i = 0;i < 3;i++)
		dX[i + 3] = dX[i + 3];
	for (int i = 0;i < 3;i++)
		dX[i] = dX[i] + BaseV[i];


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
	fwd.cptJacobi();
	double pinv[36];

	// 所需的中间变量，请对U的对角线元素做处理
	double U[36], tau[6];
	aris::Size p[6];
	aris::Size rank;

	// 根据 A 求出中间变量，相当于做 QR 分解 //
	// 请对 U 的对角线元素做处理
	s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-10);
	for (int i = 0;i < 6;i++)
		if (U[7 * i] >= 0)
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}



    if(abs(FT_KAI[2])<60)
    {
        for (int i = 0; i < 6; i++)
        {
            step_pjs[i] = step_pjs[i] + dTheta[i];
            target.model->motionPool().at(i).setMp(step_pjs[i]);
        }
    }


    if (target.count % 300 == 0)
    {

        //cout <<FT_KAI[2] << "****"<<TangentArc[0] << "****" << TangentArc[1] << "****" <<TangentArc[2]<< endl;

        cout << FT_KAI[2] << "*" <<FT[2] << "*" << FTReal[2] <<"*"<<dX[0]<< "*"<<dX[1]<<"*"<<PqEnd[2]<<std::endl;


                //cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;


    }

	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

		sT0[i][0] = sT1[i][0];
		sT0[i][1] = sT1[i][1];
		sT0[i][2] = sT1[i][2];

	}


	for (int j = 0; j < 3; j++)
	{
		TangentArc0[j] = TangentArc[j];
	}

	return 150000000 - target.count;

}

MovePressureToolYZ::MovePressureToolYZ(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
        "<Command name=\"mvPreTYZ\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}



struct MovePressureToolXYParam
{
	double PressF;
    double SensorType;


};


auto MovePressureToolXY::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureToolXYParam param;
	for (auto &p : params)
	{
        if (p.first == "PressF")
            param.PressF = std::stod(p.second);
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);
	}

	target.param = param;

     for(auto &option:target.mot_options) option|=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR|
        Plan::NOT_CHECK_ENABLE;

        SetLimit(target,6.0);


}
auto MovePressureToolXY::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureToolXYParam&>(target.param);

	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
	static double sT0[6][3], sT1[6][3];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;
    auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
	double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
	double CrossNormalZbase[3] = { 0 };

	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);
    NormalVector[0] = TransVector[2];NormalVector[1] = TransVector[6];NormalVector[2] = TransVector[10];

	crossVector(NormalVector, ZBase, CrossNormalZbase);
	CosNormalAng = NormalVector[2] / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
	SinNormalAng = sqrt(CrossNormalZbase[0] * CrossNormalZbase[0] + CrossNormalZbase[1] * CrossNormalZbase[1] + CrossNormalZbase[2] * CrossNormalZbase[2]) / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
	NormalAng = atan2(SinNormalAng, CosNormalAng);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

    double FT[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}


   SecondOrderFilter(FT,stateTor0,stateTor1,80);

	for (int j = 0; j < 6; j++)
	{
        double A[3][3], B[3], CutFreq = 85;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}


	for (int i = 0; i < 3; i++)
	{
		if (FT_KAI[i] < 1.0&&FT_KAI[i]>0)
			FT_KAI[i] = FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-1.0)
			FT_KAI[i] = -FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	for (int i = 3; i < 6; i++)
	{
		if (FT_KAI[i] < 0.05&&FT_KAI[i]>0)
			FT_KAI[i] = 20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-0.05)
			FT_KAI[i] = -20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}



	double dXpid[6] = { 0,0,0,0,0,0 };
    dXpid[2] = 1 * (FT_KAI[2] - (-5)) / 420000;
    dXpid[3] = 1 * (FT_KAI[3]) / 2000;
    dXpid[4] = 1 * (FT_KAI[4]) / 2000;
    dXpid[5] = 0 * (FT_KAI[5]) / 2000;

	double FmInWorld[6];

    FT2World(target,dXpid,FmInWorld);

	for (int i = 0;i < 6;i++)
        dX[i] = 0;//*FmInWorld[i];


	double TangentArc[3] = { 0 };
	static double TangentArc0[3] = { 0 };
	static double TangentArc1[3] = { 0 };
	static double TangentArc2[3] = { 0 };
	static bool MoveDirection = true;
	static bool MoveDirectionT = true, MoveDirectionF = false;
	static bool MoveDirectionChange = false;
    static int StartCount = 1500;
    double CosTheta1;


    static double pArc, vArc, aArc, vArcMax = 0.05;
	aris::Size t_count;

    double Square[4][3] = { {EndP0[0],0,0},
                            {EndP0[0]+0.04,0,0},
                            {EndP0[0]+0.04,0,0},
                            {EndP0[0],0,0} };


	static double MoveLength = 0;
    static double DecLength = 0.01, LengthT = 0.2, LengthF = 0.00005;//LengthT>LengthF

	LengthT = sqrt((Square[0][0] - Square[1][0])*(Square[0][0] - Square[1][0]) + (Square[0][1] - Square[1][1])*(Square[0][1] - Square[1][1]));
	double CountFmax = sqrt((Square[2][0] - Square[1][0])*(Square[2][0] - Square[1][0]) + (Square[2][1] - Square[1][1])*(Square[2][1] - Square[1][1])) / LengthF;


	static int count_offsetT = StartCount, count_offsetF = StartCount;
	static double vArcEndT = 0, vArcEndF = 0;
	static int CountT = 0, CountF = 0;

	double Ktemp, temp0, temp1;
	double CrossSurface[3] = { 0,0,0 };
	double ExtendSurface[3] = { 0,0,0 };
	temp0 = Square[1][0] - Square[0][0];temp1 = Square[1][1] - Square[0][1];
	ExtendSurface[0] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[1] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[2] = 0;

	temp0 = Square[2][0] - Square[1][0];temp1 = Square[2][1] - Square[1][1];
	CrossSurface[0] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[1] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[2] = 0;





	if (target.count > StartCount&&MoveDirectionT == true && MoveDirectionF == false)
	{
		if (CountT % 2 == 0)
			MoveDirection = true;
		else
			MoveDirection = false;
		/*
		if (MoveDirection == true)
			if (LengthT < 0)
				LengthT = -LengthT;

		if (MoveDirection == false)
			if (LengthT > 0)
				LengthT = -LengthT;
		 */
		if (abs(NormalVector[2]) < 0.01)
		{
			TangentArc1[0] = ExtendSurface[0]; TangentArc1[1] = ExtendSurface[1]; TangentArc1[2] = ExtendSurface[2];

			TangentArc2[0] = -ExtendSurface[0]; TangentArc2[1] = -ExtendSurface[1]; TangentArc2[2] = ExtendSurface[2];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
			{
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];

			}

		}
		else
		{
			temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
			temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[0] = Ktemp * ExtendSurface[0];
			TangentArc1[1] = Ktemp * ExtendSurface[1];
			TangentArc1[2] = -(NormalVector[0] * TangentArc1[0] + NormalVector[1] * TangentArc1[1]) / NormalVector[2];

			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
			temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
			Ktemp = -1 / sqrt(temp0 + temp1 * temp1);

			TangentArc2[0] = Ktemp * ExtendSurface[0];
			TangentArc2[1] = Ktemp * ExtendSurface[1];
			TangentArc2[2] = -(NormalVector[0] * TangentArc2[0] + NormalVector[1] * TangentArc2[1]) / NormalVector[2];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];

		}


		if (MoveDirection)
			if (MoveLength < LengthT - DecLength)
			{
				aris::plan::moveAbsolute(target.count - count_offsetF + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndT = vArc;
			}
			else
			{

				vArc = vArcEndT - 1 * (DecLength - (LengthT - MoveLength)) / DecLength * vArcEndT;

				if (abs(vArc) < 0.0001)
				{
					MoveDirectionT = false;
					MoveDirectionF = true;
					count_offsetT = target.count;
					MoveDirectionChange = true;
					CountT = CountT + 1;
				}
			}

		if (!MoveDirection)
			if (MoveLength > (DecLength))
			{
				aris::plan::moveAbsolute(target.count - count_offsetF + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndF = vArc;
			}
			else
			{
				vArc = vArcEndF - 1 * (DecLength - MoveLength) / DecLength * vArcEndF;

				if (abs(vArc) < 0.0001)
				{
					MoveDirectionT = false;
					MoveDirectionF = true;
					count_offsetT = target.count;
					MoveDirectionChange = true;
					CountT = CountT + 1;
				}
			}

	}


	if (target.count > StartCount&&MoveDirectionT == false && MoveDirectionF == true && CountF < CountFmax)
	{
		if (CountF % 2 == 0)
		{
			MoveDirection = true;
		}
		else
		{
			MoveDirection = true;
		}
		if (abs(NormalVector[2]) < 0.01)
		{
			TangentArc1[0] = CrossSurface[0]; TangentArc1[1] = CrossSurface[1]; TangentArc1[2] = CrossSurface[2];
			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			for (int i = 0;i < 3;i++)
				TangentArc[i] = TangentArc1[i];
		}
		else
		{
			temp0 = CrossSurface[0] * CrossSurface[0] + CrossSurface[1] * CrossSurface[1];
			temp1 = (CrossSurface[0] * NormalVector[0] + CrossSurface[1] * NormalVector[1]) / NormalVector[2];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[0] = Ktemp * CrossSurface[0];
			TangentArc1[1] = Ktemp * CrossSurface[1];
			TangentArc1[2] = -(NormalVector[0] * TangentArc1[0] + NormalVector[1] * TangentArc1[1]) / NormalVector[2];

			for (int i = 0;i < 3;i++)
				TangentArc[i] = TangentArc1[i];

		}


		if (MoveDirection)
		{
			aris::plan::moveAbsolute(target.count - count_offsetT + 1, 0, LengthF, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
			vArc = vArc * 1000;
			vArcEndT = vArc;
		}

		if (abs(vArc) < 0.0001&&aArc < 0)
		{
			MoveDirectionT = true;
			MoveDirectionF = false;
			count_offsetF = target.count;
			MoveDirectionChange = true;
			CountF = CountF + 1;
		}


	}



		if (MoveDirection)
		{
            dX[0] = dX[0]+vArc * TangentArc[0] / 1000;
            dX[1] = dX[1]+vArc * TangentArc[1] / 1000;
            dX[2] = dX[2]+vArc * TangentArc[2] / 1000;

		}
		else
		{
            dX[0] = dX[0]+vArc * TangentArc[0] / 1000;
            dX[1] = dX[1]+vArc * TangentArc[1] / 1000;
            dX[2] = dX[2]+vArc * TangentArc[2] / 1000;

		}
		if (target.count > StartCount&&MoveDirectionT == true && MoveDirectionF == false)
			if (MoveDirection)
                MoveLength = MoveLength + sqrt(dX[0] * dX[0] + dX[2] * dX[2]);
			else
                MoveLength = MoveLength - sqrt(dX[0] * dX[0] + dX[2] * dX[2]);




	//if(FT_KAI[2]<-12.5)
	  //  ForceToMeng =9.38;
   // else
/*
	for (int j = 0; j < 6; j++)
	{
		double A[3][3], B[3], CutFreq = 10;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;

		if(target.count > start&&target.count < (start+1*interval-StopInt))
		{
		if(FT_KAI[j]<-14)
			FT_KAI[j]=-10.59+sin(target.count);
		if(FT_KAI[j]>-7)
			FT_KAI[j]=-9.37+sin(target.count);
		}
		sT1[j][0] = sT0[j][0] + intDT * (A[0][0] * sT0[j][0] + A[0][1] * sT0[j][1] + A[0][2] * sT0[j][2] + B[0] * FT_KAI[j]);
		sT1[j][1] = sT0[j][1] + intDT * (A[1][0] * sT0[j][0] + A[1][1] * sT0[j][1] + A[1][2] * sT0[j][2] + B[1] * FT_KAI[j]);
		sT1[j][2] = sT0[j][2] + intDT * (A[2][0] * sT0[j][0] + A[2][1] * sT0[j][1] + A[2][2] * sT0[j][2] + B[2] * FT_KAI[j]);
	}*/

	ForceToMeng = sT1[2][0];


	ForceToMeng = vArc;
	TimeToMeng = target.count / 1000.0;



	for (int j = 0; j < 6; j++)
	{
        if (dX[j] > 0.00025)
            dX[j] = 0.00025;
        if (dX[j] < -0.00025)
            dX[j] = -0.00025;
	}


	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";
    //lout << vArc << endl;
    //lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	//lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
	//lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
   // lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";
    lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
    lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
    lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";


     lout << FT[0] << ",";lout << FT[1] << ",";
     lout << FT[2] << ",";lout << FT[3] << ",";
     lout << FT[4] << ",";lout << FT[5] << ",";
    lout << std::endl;



    dX2dTheta(target,dX,dTheta);

	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
	}


    if (target.count % 300 == 0)
    {

        cout << step_pjs[0]<<"*"<<step_pjs[1] << "*" << step_pjs[2] <<"*"<<step_pjs[3]<< "*" << step_pjs[4] << "*" <<step_pjs[5] << std::endl;

        //cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[0] << "*" << TransVector[1] << "*" << TransVector[2] << "*" << FT0[2] << endl;
        //cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[4] << "*" << TransVector[5] << "*" << TransVector[6] << "*" << FT0[2] << endl;
        //cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[8] << "*" << TransVector[9] << "*" << TransVector[10] << "*" << FT0[2] << endl;

                //cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

        cout << std::endl;

    }



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

		sT0[i][0] = sT1[i][0];
		sT0[i][1] = sT1[i][1];
		sT0[i][2] = sT1[i][2];

	}


	for (int j = 0; j < 3; j++)
	{
		TangentArc0[j] = TangentArc[j];
	}

	return 150000000 - target.count;

}

auto MovePressureToolXY::collectNrt(aris::plan::PlanTarget &target)->void
{

    ReSetLimit(target);

}

MovePressureToolXY::MovePressureToolXY(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvPreTXY\">"
		"	<GroupParam>"
        "       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}


struct MovePressureToolXLineParam
{
    double PressF;
    double SensorType;


};

auto MovePressureToolXLine::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MovePressureToolXLineParam param;
    for (auto &p : params)
    {
        if (p.first == "PressF")
            param.PressF = std::stod(p.second);
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);
    }

    target.param = param;

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_ENABLE;

        SetLimit(target,6.0);


}
auto MovePressureToolXLine::executeRT(PlanTarget &target)->int
{
    auto &param = std::any_cast<MovePressureToolXLineParam&>(target.param);

    static double step_pjs[6];
    static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
    static double sT0[6][3], sT1[6][3];
    static double FT0[6];

    // 访问主站 //
    auto controller = target.controller;
    auto &cout = controller->mout();
    // 获取当前起始点位置 //
    if (target.count == 1)
    {
        for (int i = 0; i < 6; ++i)
        {
            step_pjs[i] = target.model->motionPool()[i].mp();
            // controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
        }
    }


    if (target.model->solverPool().at(1).kinPos())return -1;


    double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
    double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
    double CrossNormalZbase[3] = { 0 };

    target.model->generalMotionPool().at(0).getMpm(TransVector);
    target.model->generalMotionPool().at(0).getMpq(PqEnd);
    NormalVector[0] = TransVector[2];NormalVector[1] = TransVector[6];NormalVector[2] = TransVector[10];

    crossVector(NormalVector, ZBase, CrossNormalZbase);
    CosNormalAng = NormalVector[2] / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
    SinNormalAng = sqrt(CrossNormalZbase[0] * CrossNormalZbase[0] + CrossNormalZbase[1] * CrossNormalZbase[1] + CrossNormalZbase[2] * CrossNormalZbase[2]) / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
    NormalAng = atan2(SinNormalAng, CosNormalAng);

    double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
    double dTheta[6] = { 0 };

    double FT[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);



    // 获取当前起始点位置 //
    if (target.count == 1)
    {
        for (int j = 0; j < 6; j++)
        {
            stateTor0[j][0] = FT[j];
            FT0[j] = FT[j];
        }
        for (int i = 0;i < 3;i++)
            EndP0[i] = PqEnd[i];
    }


   SecondOrderFilter(FT,stateTor0,stateTor1,80);


    double FT_KAI[6];
    for (int i = 0; i < 6; i++)
    {
        FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
    }

	double zero_check[6] = { 1,1,1,0.05,0.05,0.05 };
    for (int i = 0; i < 6; i++)
    {
        if (FT_KAI[i] < zero_check[i] &&FT_KAI[i]>0)
            FT_KAI[i] = 1/zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
        else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
            FT_KAI[i] = -1 / zero_check[i]*FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
    }


    double dXpid[6] = { 0,0,0,0,0,0 };
    dXpid[2] = 1 * (FT_KAI[2] - (-5)) / 220000;
    dXpid[3] = 0 * (FT_KAI[3]) / 2000;
    dXpid[4] = 0 * (FT_KAI[4]) / 2000;
    dXpid[5] = 0 * (FT_KAI[5]) / 2000;

    double FmInWorld[6];

    FT2World(target,dXpid,FmInWorld);

	for (int i = 0;i < 6;i++)
		dX[i] = 0;//FmInWorld[i];


    double TangentArc[3] = { 0 };
    static double TangentArc0[3] = { 0 };
    static double TangentArc1[3] = { 0 };
    static double TangentArc2[3] = { 0 };
    static bool MoveDirection = true;
    static bool MoveDirectionX = true;
    static bool MoveDirectionChange = false;
    static int StartCount = 1500;
    double CosTheta1;


    static double pArc, vArc, aArc, vArcMax = 0.05;
    aris::Size t_count;

    double Square[4][3] = { {EndP0[0],0,0},
                            {EndP0[0]+0.04,0,0},
                            {EndP0[0]+0.04,0,0},
                            {EndP0[0],0,0} };


    static double MoveLength = 0;
    static double DecLength = 0.01, LengthX = 0.2;//LengthT>LengthF

    LengthX = sqrt((Square[0][0] - Square[1][0])*(Square[0][0] - Square[1][0]) + (Square[0][1] - Square[1][1])*(Square[0][1] - Square[1][1]));

    static int count_offsetX = StartCount;
    static double vArcEndX = 0;
    static int CountX = 0;

    double Ktemp, temp0, temp1;
    double CrossSurface[3] = { 0,0,0 };
    double ExtendSurface[3] = { 0,0,0 };
    temp0 = Square[1][0] - Square[0][0];temp1 = Square[1][1] - Square[0][1];
    ExtendSurface[0] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[1] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[2] = 0;

    temp0 = Square[2][0] - Square[1][0];temp1 = Square[2][1] - Square[1][1];
    CrossSurface[0] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[1] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); CrossSurface[2] = 0;





    if (target.count > StartCount)
    {
        if (CountX % 2 == 0)
            MoveDirection = true;
        else
            MoveDirection = false;

        if (abs(NormalVector[2]) < 0.01)
        {
            TangentArc1[0] = ExtendSurface[0]; TangentArc1[1] = ExtendSurface[1]; TangentArc1[2] = ExtendSurface[2];

            TangentArc2[0] = -ExtendSurface[0]; TangentArc2[1] = -ExtendSurface[1]; TangentArc2[2] = ExtendSurface[2];

            if (MoveDirection == true)
                for (int i = 0;i < 3;i++)
                    TangentArc[i] = TangentArc1[i];

            if (MoveDirection == false)
            {
                for (int i = 0;i < 3;i++)
                    TangentArc[i] = TangentArc2[i];

            }

        }
        else
        {
            temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
            temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
            Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

            TangentArc1[0] = Ktemp * ExtendSurface[0];
            TangentArc1[1] = Ktemp * ExtendSurface[1];
            TangentArc1[2] = -(NormalVector[0] * TangentArc1[0] + NormalVector[1] * TangentArc1[1]) / NormalVector[2];

            CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

            temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
            temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
            Ktemp = -1 / sqrt(temp0 + temp1 * temp1);

            TangentArc2[0] = Ktemp * ExtendSurface[0];
            TangentArc2[1] = Ktemp * ExtendSurface[1];
            TangentArc2[2] = -(NormalVector[0] * TangentArc2[0] + NormalVector[1] * TangentArc2[1]) / NormalVector[2];

            if (MoveDirection == true)
                for (int i = 0;i < 3;i++)
                    TangentArc[i] = TangentArc1[i];

            if (MoveDirection == false)
                for (int i = 0;i < 3;i++)
                    TangentArc[i] = TangentArc2[i];

        }


        if (MoveDirection)
            if (MoveLength < LengthX - DecLength)
            {
                aris::plan::moveAbsolute(target.count - count_offsetX + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
                vArc = vArc * 1000;
                vArcEndX = vArc;
            }
            else
            {

                vArc = vArcEndX - 1 * (DecLength - (LengthX - MoveLength)) / DecLength * vArcEndX;

                if (abs(vArc) < 0.0001)
                {
                    count_offsetX = target.count;
                    MoveDirectionChange = true;
                    CountX = CountX + 1;
                }
            }

        if (!MoveDirection)
            if (MoveLength > (DecLength))
            {
                aris::plan::moveAbsolute(target.count - count_offsetX + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
                vArc = vArc * 1000;
                vArcEndX = vArc;
            }
            else
            {
                vArc = vArcEndX - 1 * (DecLength - MoveLength) / DecLength * vArcEndX;

                if (abs(vArc) < 0.0001)
                {

                    count_offsetX = target.count;
                    MoveDirectionChange = true;
                    CountX = CountX + 1;
                }
            }

    }



        if (MoveDirection)
        {
            dX[0] = dX[0]+vArc * TangentArc[0] / 1000;
            dX[1] = dX[1]+vArc * TangentArc[1] / 1000;
            dX[2] = dX[2]+vArc * TangentArc[2] / 1000;

        }
        else
        {
            dX[0] = dX[0]+vArc * TangentArc[0] / 1000;
            dX[1] = dX[1]+vArc * TangentArc[1] / 1000;
            dX[2] = dX[2]+vArc * TangentArc[2] / 1000;

        }
        if (target.count > StartCount)
            if (MoveDirection)
                MoveLength = MoveLength + sqrt(dX[0] * dX[0] + dX[2] * dX[2]);
            else
                MoveLength = MoveLength - sqrt(dX[0] * dX[0] + dX[2] * dX[2]);


    for (int j = 0; j < 6; j++)
    {
        if (dX[j] > 0.00025)
            dX[j] = 0.00025;
        if (dX[j] < -0.00025)
            dX[j] = -0.00025;
    }


    // log 电流 //
    auto &lout = controller->lout();

    // lout << target.model->motionPool()[0].mp() << ",";
     //lout << target.model->motionPool()[1].mp() << ",";
     //lout << target.model->motionPool()[2].mp() << ",";
     //lout << target.model->motionPool()[3].mp() << ",";
     //lout << target.model->motionPool()[4].mp() << ",";
     //lout << target.model->motionPool()[5].mp() << ",";
    //lout << vArc << endl;
    //lout << FTnum << ",";
    //lout << FT[2] << ",";lout << dX[2] << ",";
    //lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

    //lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
    //lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
   // lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";
    lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
    lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
    lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";


     lout << dX[0] << ",";lout << dX[1] << ",";
     lout << dX[2] << ",";lout << dX[3] << ",";
     lout << dX[4] << ",";lout << dX[5] << ",";
    lout << std::endl;



    dX2dTheta(target,dX,dTheta);

    for (int i = 0; i < 6; i++)
    {
        if (dTheta[i] > 0.003)
            dTheta[i] = 0.003;
        if (dTheta[i] < -0.003)
            dTheta[i] = -0.003;
        //lout << dTheta[i] << ",";
    }


    //lout << std::endl;
    for (int i = 0; i < 6; i++)
    {
        dTheta[i] = dTheta[i] * DirectionFlag[i];

    }

    for (int i = 0; i < 6; i++)
    {
        step_pjs[i] = step_pjs[i] + dTheta[i];
        target.model->motionPool().at(i).setMp(step_pjs[i]);
    }


    if (target.count % 300 == 0)
    {

        cout << FmInWorld[0]<<"*"<<FmInWorld[1] << "*" << FmInWorld[2] <<"*"<<step_pjs[3]<< "*" << step_pjs[4] << "*" <<FT_KAI[2] << std::endl;

        //cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[0] << "*" << TransVector[1] << "*" << TransVector[2] << "*" << FT0[2] << endl;
        //cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[4] << "*" << TransVector[5] << "*" << TransVector[6] << "*" << FT0[2] << endl;
        //cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[8] << "*" << TransVector[9] << "*" << TransVector[10] << "*" << FT0[2] << endl;

                //cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

        cout << std::endl;

    }



    for (int i = 0; i < 6; i++)
    {

        stateTor0[i][0] = stateTor1[i][0];
        stateTor0[i][1] = stateTor1[i][1];
        stateTor0[i][2] = stateTor1[i][2];

        sT0[i][0] = sT1[i][0];
        sT0[i][1] = sT1[i][1];
        sT0[i][2] = sT1[i][2];

    }


    for (int j = 0; j < 3; j++)
    {
        TangentArc0[j] = TangentArc[j];
    }

    return 15000 - target.count;

}

auto MovePressureToolXLine::collectNrt(aris::plan::PlanTarget &target)->void
{

    ReSetLimit(target);

}

MovePressureToolXLine::MovePressureToolXLine(const std::string &name) :Plan(name)
{

    command().loadXmlStr(
        "<Command name=\"mvPreTXLine\">"
        "	<GroupParam>"
        "       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"-20.0\"/>"
        "   </GroupParam>"
        "</Command>");

}



struct MovePressureToolYLineParam
{
	double PressF;
	double SensorType;


};

auto MovePressureToolYLine::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureToolYLineParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);
		if (p.first == "SensorType")
			param.SensorType = std::stod(p.second);
	}

	target.param = param;

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_ENABLE;

	SetLimit(target, 6.0);


}
auto MovePressureToolYLine::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureToolYLineParam&>(target.param);

	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
	static double sT0[6][3], sT1[6][3];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;
	auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
	double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
	double CrossNormalZbase[3] = { 0 };

	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);
	NormalVector[0] = TransVector[2];NormalVector[1] = TransVector[6];NormalVector[2] = TransVector[10];

	crossVector(NormalVector, ZBase, CrossNormalZbase);
	CosNormalAng = NormalVector[2] / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
	SinNormalAng = sqrt(CrossNormalZbase[0] * CrossNormalZbase[0] + CrossNormalZbase[1] * CrossNormalZbase[1] + CrossNormalZbase[2] * CrossNormalZbase[2]) / sqrt(NormalVector[0] * NormalVector[0] + NormalVector[1] * NormalVector[1] + NormalVector[2] * NormalVector[2]);
	NormalAng = atan2(SinNormalAng, CosNormalAng);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	double FT[6];
	if (param.SensorType > 0)
		GetATI(target, FT);
	else
		GetYuLi(target, FT);



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}


	SecondOrderFilter(FT, stateTor0, stateTor1, 80);


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}

	double zero_check[6] = { 1,1,1,0.05,0.05,0.05 };
	for (int i = 0; i < 6; i++)
	{
		if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
			FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
			FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}


	double dXpid[6] = { 0,0,0,0,0,0 };
	dXpid[2] = 1 * (FT_KAI[2] - (-5)) / 220000;
	dXpid[3] = 0 * (FT_KAI[3]) / 2000;
	dXpid[4] = 0 * (FT_KAI[4]) / 2000;
	dXpid[5] = 0 * (FT_KAI[5]) / 2000;

	double FmInWorld[6];

	FT2World(target, dXpid, FmInWorld);

	for (int i = 0;i < 6;i++)
		dX[i] = 0;//FmInWorld[i];


	double TangentArc[3] = { 0 };
	static double TangentArc0[3] = { 0 };
	static double TangentArc1[3] = { 0 };
	static double TangentArc2[3] = { 0 };
	static bool MoveDirection = true;
	static bool MoveDirectionY = true;
	static bool MoveDirectionChange = false;
	static int StartCount = 1500;
	double CosTheta1;


	static double pArc, vArc, aArc, vArcMax = 0.05;
	aris::Size t_count;

	double Square[4][3] = { {EndP0[0],EndP0[1],0},
							{EndP0[0],EndP0[1]+0.04,0},
							{EndP0[0],EndP0[1]+0.04,0},
							{EndP0[0],EndP0[1],0} };


	static double MoveLength = 0;
	static double DecLength = 0.01, LengthY = 0.2;//LengthT>LengthF

	LengthY = sqrt((Square[0][0] - Square[1][0])*(Square[0][0] - Square[1][0]) + (Square[0][1] - Square[1][1])*(Square[0][1] - Square[1][1]));

	static int count_offsetY = StartCount;
	static double vArcEndY = 0;
	static int CountY = 0;

	double Ktemp, temp0, temp1;
	double CrossSurface[3] = { 0,0,0 };
	double ExtendSurface[3] = { 0,0,0 };
	temp0 = Square[1][0] - Square[0][0];temp1 = Square[1][1] - Square[0][1];
	ExtendSurface[0] = temp0 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[1] = temp1 / sqrt(temp0*temp0 + temp1 * temp1); ExtendSurface[2] = 0;


	if (target.count > StartCount)
	{
		if (CountY % 2 == 0)
			MoveDirection = true;
		else
			MoveDirection = false;

		if (abs(NormalVector[2]) < 0.01)
		{
			TangentArc1[0] = ExtendSurface[0]; TangentArc1[1] = ExtendSurface[1]; TangentArc1[2] = ExtendSurface[2];

			TangentArc2[0] = -ExtendSurface[0]; TangentArc2[1] = -ExtendSurface[1]; TangentArc2[2] = ExtendSurface[2];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
			{
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];

			}

		}
		else
		{
			temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
			temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
			Ktemp = 1 / sqrt(temp0 + temp1 * temp1);

			TangentArc1[0] = Ktemp * ExtendSurface[0];
			TangentArc1[1] = Ktemp * ExtendSurface[1];
			TangentArc1[2] = -(NormalVector[0] * TangentArc1[0] + NormalVector[1] * TangentArc1[1]) / NormalVector[2];

			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
			temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
			Ktemp = -1 / sqrt(temp0 + temp1 * temp1);

			TangentArc2[0] = Ktemp * ExtendSurface[0];
			TangentArc2[1] = Ktemp * ExtendSurface[1];
			TangentArc2[2] = -(NormalVector[0] * TangentArc2[0] + NormalVector[1] * TangentArc2[1]) / NormalVector[2];

			if (MoveDirection == true)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc1[i];

			if (MoveDirection == false)
				for (int i = 0;i < 3;i++)
					TangentArc[i] = TangentArc2[i];

		}


		if (MoveDirection)
			if (MoveLength < LengthY - DecLength)
			{
				aris::plan::moveAbsolute(target.count - count_offsetY + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndY = vArc;
			}
			else
			{

				vArc = vArcEndY - 1 * (DecLength - (LengthY - MoveLength)) / DecLength * vArcEndY;

				if (abs(vArc) < 0.0001)
				{
					count_offsetY = target.count;
					MoveDirectionChange = true;
					CountY = CountY + 1;
				}
			}

		if (!MoveDirection)
			if (MoveLength > (DecLength))
			{
				aris::plan::moveAbsolute(target.count - count_offsetY + 1, 0, 1000, vArcMax / 1000, 0.05 / 1000 / 1000, 0.05 / 1000 / 1000, pArc, vArc, aArc, t_count);
				vArc = vArc * 1000;
				vArcEndY = vArc;
			}
			else
			{
				vArc = vArcEndY - 1 * (DecLength - MoveLength) / DecLength * vArcEndY;

				if (abs(vArc) < 0.0001)
				{

					count_offsetY = target.count;
					MoveDirectionChange = true;
					CountY = CountY + 1;
				}
			}

	}





	if (MoveDirection)
	{
		dX[0] = dX[0] + vArc * TangentArc[0] / 1000;
		dX[1] = dX[1] + vArc * TangentArc[1] / 1000;
		dX[2] = dX[2] + vArc * TangentArc[2] / 1000;

	}
	else
	{
		dX[0] = dX[0] + vArc * TangentArc[0] / 1000;
		dX[1] = dX[1] + vArc * TangentArc[1] / 1000;
		dX[2] = dX[2] + vArc * TangentArc[2] / 1000;

	}
	if (target.count > StartCount)
		if (MoveDirection)
			MoveLength = MoveLength + sqrt(dX[1] * dX[1] + dX[2] * dX[2]);
		else
			MoveLength = MoveLength - sqrt(dX[1] * dX[1] + dX[2] * dX[2]);


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}


	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";
	//lout << vArc << endl;
	//lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	//lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
	//lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
   // lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";
	lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
	lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
	lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";


	lout << FT[0] << ",";lout << FT[1] << ",";
	lout << FT[2] << ",";lout << FT[3] << ",";
	lout << FT[4] << ",";lout << FT[5] << ",";
	lout << std::endl;



	dX2dTheta(target, dX, dTheta);

	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}


	if (target.count % 300 == 0)
	{

		cout << FmInWorld[0] << "*" << FmInWorld[1] << "*" << FmInWorld[2] << "*" << step_pjs[3] << "*" << step_pjs[4] << "*" << FT_KAI[2] << std::endl;

		//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[0] << "*" << TransVector[1] << "*" << TransVector[2] << "*" << FT0[2] << endl;
		//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[4] << "*" << TransVector[5] << "*" << TransVector[6] << "*" << FT0[2] << endl;
		//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[8] << "*" << TransVector[9] << "*" << TransVector[10] << "*" << FT0[2] << endl;

				//cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

		sT0[i][0] = sT1[i][0];
		sT0[i][1] = sT1[i][1];
		sT0[i][2] = sT1[i][2];

	}


	for (int j = 0; j < 3; j++)
	{
		TangentArc0[j] = TangentArc[j];
	}

	return 150000000 - target.count;

}

auto MovePressureToolYLine::collectNrt(aris::plan::PlanTarget &target)->void
{

	ReSetLimit(target);

}

MovePressureToolYLine::MovePressureToolYLine(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvPreTYLine\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}





struct MoveFeedParam
{
	double PressF;
    double SensorType;


};

auto MoveFeed::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveFeedParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
            param.PressF = std::stod(p.second);

	}

	target.param = param;

	target.option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_MIN |
		Plan::NOT_CHECK_VEL_MAX |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;




}
auto MoveFeed::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveFeedParam&>(target.param);

	double RobotPosition[6];
	double RobotPositionJ[6];
	double RobotVelocity[6];
	double RobotAcceleration[6];
	double TorqueSensor[6];
	double X1[3];
	double X2[3];
	static double begin_pjs[6];
	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3];
    static double FT0[6], FT_be[6];
	static double SumFtErr[6];
	// 访问主站 //
	auto controller = target.controller;
    auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
			SumFtErr[i] = 0;
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

    double FT[6];
	uint16_t FTnum;
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x00, &FTnum, 16);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x01, &FT[0], 32);  //Fx
	conSensor->slavePool().at(6).readPdo(0x6030, 0x02, &FT[1], 32);  //Fy
	conSensor->slavePool().at(6).readPdo(0x6030, 0x03, &FT[2], 32);  //Fz
	conSensor->slavePool().at(6).readPdo(0x6030, 0x04, &FT[3], 32);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x05, &FT[4], 32);
	conSensor->slavePool().at(6).readPdo(0x6030, 0x06, &FT[5], 32);
	FT[0] = -FT[0];FT[3] = -FT[3];

	for (int i = 0;i < 6;i++)
	{
		RobotPositionJ[i] = target.model->motionPool()[i].mp();
		RobotPosition[i] = target.model->motionPool()[i].mp();
		RobotVelocity[i] = 0;
		RobotAcceleration[i] = 0;
		TorqueSensor[i] = FT[i];
	}



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
			FT_be[j] = FT[j];
		}
	}

	for (int j = 0; j < 6; j++)
	{
		if (abs(FT[j]) < 0.0001)
			FT[j] = FT_be[j];
	}


	for (int j = 0; j < 6; j++)
	{
		double A[3][3], B[3], CutFreq = 35;//SHANGHAI DIANQI EXP
		//CutFreq = 85;
		A[0][0] = 0; A[0][1] = 1; A[0][2] = 0;
		A[1][0] = 0; A[1][1] = 0; A[1][2] = 1;
		A[2][0] = -CutFreq * CutFreq * CutFreq;
		A[2][1] = -2 * CutFreq * CutFreq;
		A[2][2] = -2 * CutFreq;
		B[0] = 0; B[1] = 0;
		B[2] = -A[2][0];
		double intDT = 0.001;
		stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + A[0][2] * stateTor0[j][2] + B[0] * FT[j]);
		stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + A[1][2] * stateTor0[j][2] + B[1] * FT[j]);
		stateTor1[j][2] = stateTor0[j][2] + intDT * (A[2][0] * stateTor0[j][0] + A[2][1] * stateTor0[j][1] + A[2][2] * stateTor0[j][2] + B[2] * FT[j]);
	}


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}


	for (int i = 0; i < 3; i++)
	{
		if (FT_KAI[i] < 1.0&&FT_KAI[i]>0)
			FT_KAI[i] = FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-1.0)
			FT_KAI[i] = -FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	for (int i = 3; i < 6; i++)
	{
		if (FT_KAI[i] < 0.05&&FT_KAI[i]>0)
			FT_KAI[i] = 20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-0.05)
			FT_KAI[i] = -20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}


	double FT_YANG[6];
	FT_YANG[0] = FT_KAI[2];FT_YANG[1] = -FT_KAI[1];FT_YANG[2] = FT_KAI[0];
	FT_YANG[3] = FT_KAI[5];FT_YANG[4] = -FT_KAI[4];FT_YANG[5] = FT_KAI[3];

	double FmInWorld[6];

	double TransMatrix[4][4];
	for (int i = 0;i < 4;i++)
		for (int j = 0;j < 4;j++)
			TransMatrix[i][j] = TransVector[4 * i + j];

	double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
	double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
	double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };

	//FT[0] = 0;FT[1] = 0;FT[2] = 1;FT[3] = 0;FT[4] = 0;FT[5] = 0;
	FmInWorld[0] = n[0] * FT_YANG[0] + n[1] * FT_YANG[1] + n[2] * FT_YANG[2];
	FmInWorld[1] = o[0] * FT_YANG[0] + o[1] * FT_YANG[1] + o[2] * FT_YANG[2];
	FmInWorld[2] = a[0] * FT_YANG[0] + a[1] * FT_YANG[1] + a[2] * FT_YANG[2];
	FmInWorld[3] = n[0] * FT_YANG[3] + n[1] * FT_YANG[4] + n[2] * FT_YANG[5];
	FmInWorld[4] = o[0] * FT_YANG[3] + o[1] * FT_YANG[4] + o[2] * FT_YANG[5];
	FmInWorld[5] = a[0] * FT_YANG[3] + a[1] * FT_YANG[4] + a[2] * FT_YANG[5];


	//if (abs(FmInWorld[2]) > 2)
	//	SumFtErr[2] = SumFtErr[2] + (FmInWorld[2] - (5))*DT;
	double ContactForceXY = 0, ThetaXY = 0, dXY = 0;
	static bool FreeFlag = true;
	ContactForceXY = sqrt(FmInWorld[0] * FmInWorld[0] + FmInWorld[1] * FmInWorld[1]);

	if (ContactForceXY > 1 || FreeFlag == false)
	{
		if (abs(FmInWorld[0]) > 0.01)
			ThetaXY = atan(FmInWorld[1] / FmInWorld[0]);
		else
			ThetaXY = 1.57;
		dXY = 1 * (FmInWorld[1] - (-5)) / 820000;

		dX[0] = 0.00001;
		dX[1] = dXY;
		dX[2] = 0 * (FmInWorld[2] - (5) + 0 * SumFtErr[2]) / 820000;
		dX[3] = 0 * (FmInWorld[3]) / 4000;
		dX[4] = 0 * (FmInWorld[4]) / 4000;
		dX[5] = 0 * (FmInWorld[5]) / 4000;
		FreeFlag = false;
	}

	if (ContactForceXY < 1 && FreeFlag)
	{
		dX[0] = 0.00001;
		dX[1] = 0.00001;
	}


	/*
		if (target.count > 23000)
			dX[0] = -0.00001;
		if (target.count > 45000)
			dX[0] = 0.00001;
		if (target.count > 67000)
			dX[0] = -0.00001;
		if (target.count > 89000)
			dX[0] = 0.00001;
		if (target.count > 111000)
			dX[0] = -0.00001;
		if (target.count > 133000)
			dX[0] = 0.00001;
	*/
	if (target.count % 100 == 0)
	{
        cout << "******" << std::endl;
        cout << FmInWorld[1] << "***" << ThetaXY << "***" << dX[0] << "***" << dX[1] << "***" << FT0[2] << std::endl;

		//cout <<  FT_KAI[0]<<"***"<<FmInWorld[2]<<endl;

		cout << std::endl;

	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}

	// log 电流 //
	auto &lout = controller->lout();

	// lout << target.model->motionPool()[0].mp() << ",";
	 //lout << target.model->motionPool()[1].mp() << ",";
	 //lout << target.model->motionPool()[2].mp() << ",";
	 //lout << target.model->motionPool()[3].mp() << ",";
	 //lout << target.model->motionPool()[4].mp() << ",";
	 //lout << target.model->motionPool()[5].mp() << ",";

	lout << FTnum << ",";
	//lout << FT[2] << ",";lout << dX[2] << ",";
	//lout << FmInWorld[2] << ",";lout << FT0[2] << ",";

	lout << FmInWorld[0] << ",";lout << FmInWorld[1] << ",";
	lout << FmInWorld[2] << ",";lout << FmInWorld[3] << ",";
	lout << FmInWorld[4] << ",";
	// lout << FT_KAI[0] << ",";lout << FT_KAI[1] << ",";
	// lout << FT_KAI[2] << ",";lout << FT_KAI[3] << ",";
	 //lout << FT_KAI[4] << ",";lout << FT_KAI[5] << ",";




	 //lout << stateTor1[2][0] << ",";lout << FT0[3] << ",";
	// lout << dX[0] << ",";

	// lout << dX[0] << ",";
	// lout << FT[1] << ",";lout << FT[2] << ",";
	// lout << FT[3] << ",";lout << FT[4] << ",";
	// lout << FT[5] << ",";lout << FT[6] << ",";
	lout << std::endl;



	///* Using Jacobian, TransMatrix from ARIS
	for (int i = 0;i < 3;i++)
		EndW[i] = dX[i + 3];

	for (int i = 0;i < 3;i++)
		EndP[i] = PqEnd[i];
	crossVector(EndP, EndW, BaseV);

	for (int i = 0;i < 3;i++)
		dX[i + 3] = dX[i + 3];
	for (int i = 0;i < 3;i++)
		dX[i] = dX[i] + BaseV[i];


	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
	fwd.cptJacobi();
	double pinv[36];

	// 所需的中间变量，请对U的对角线元素做处理
	double U[36], tau[6];
	aris::Size p[6];
	aris::Size rank;

	// 根据 A 求出中间变量，相当于做 QR 分解 //
	// 请对 U 的对角线元素做处理
	s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-10);
	for (int i = 0;i < 6;i++)
		if (U[7 * i] >= 0)
			U[7 * i] = U[7 * i] + 0.1;
		else
			U[7 * i] = U[7 * i] - 0.1;
	// 根据QR分解的结果求x，相当于Matlab中的 x = A\b //
	s_householder_utp_sov(6, 6, 1, rank, U, tau, p, dX, dTheta, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
	double tau2[6];
	s_householder_utp2pinv(6, 6, rank, U, tau, p, pinv, tau2, 1e-10);

	// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
	s_mm(6, 1, 6, pinv, dX, dTheta);



	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}



	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}

	for (int j = 0; j < 6; j++)
	{
		FT_be[j] = FT[j];
	}
	return 150000000 - target.count;

}

MoveFeed::MoveFeed(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvFe\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");

}



int signV(double x)
{
    double margin = 0.01;
	if (x > margin) return 1;
	if (abs(x) < margin || abs(x) == margin) return 0;
	if (x < -margin) return -1;
}

//系统传递函数H(s)=1/(ms)
void PIDcalOne(double m,double ts,double *KP)
{
	double T = ts / 3.0;
	KP[0] = m / T;
}

//系统传递函数H(s)=1/(ms+h)
void PIDcalTwo(double m, double h, double ts, double overshoot,double *KP, double *KI)
{
	double temp = log(overshoot);
	double kesi = 1 / sqrt(1 + aris::PI*aris::PI / temp / temp);
	double omega = 4 / kesi / ts;

	KI[0] = omega * omega * m;
	KP[0] = 2 * kesi *omega * m - h;
}

auto f(aris::dynamic::Model *m, double *A)->void
{
    auto &s = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool()[1]);

    // J_inv
    double U[36], tau[6], J_inv[36], tau2[6];
    aris::Size p[6], rank;	s_householder_utp(6, 6, s.Jf(), U, tau, p, rank, 1e-4);
    s_householder_utp2pinv(6, 6, rank, U, tau, p, J_inv, tau2, 1e-7);
    // M = (M + I) * J_inv
    double M[36], tem[36];
    s_mc(6, 6, s.M(), s.nM(), M, 6);
    for (int i = 0; i < 6; ++i)M[at(i, i, 6)] += m->motionPool()[i].frcCoe()[2];
    s_mm(6, 6, 6, M, J_inv, tem);
    s_mm(6, 6, 6, J_inv, T(6), tem, 6, A, 6);
}



static std::atomic_bool enable_FCPressL = true;
struct ForceDirectParam
{
	double PressF;
	double SensorType;

};


auto ForceDirect::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	ForceDirectParam param;
    for (auto &p : params)
    {
        if (p.first == "PressF")
            param.PressF = std::stod(p.second);
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);

    }
	target.param = param;
    target.ret = std::vector<std::pair<std::string, std::any>>();
    enable_FCPressL=true;

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS|
		Plan::NOT_CHECK_ENABLE;





}
auto ForceDirect::executeRT(PlanTarget &target)->int
{
    auto &param = std::any_cast<ForceDirectParam&>(target.param);
    auto &lout = target.controller->lout();

    static double begin_pjs[6]={0};
    static double step_pjs[6]={0};
    static double begin_t0[6];
    static double stateTor0[6], stateTor1[6];

    static double FT0[6];
    static double PqEnd0[7] = { 0 }, PqEnd[7] = { 0 },pe0[6] = { 0 },pe[6] = { 0 };
	// 访问主站 //
	auto controller = target.controller;
    auto &cout = controller->mout();
    target.model->generalMotionPool().at(0).getMpq(PqEnd);

	// 获取当前起始点位置 //
    double FT[6],FTemp[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);

    double q[6],dq[6],ddq[6],CollisionFT[6];
    for (int i = 0; i < 6; i++)
    {
        q[i]= controller->motionPool()[i].actualPos();
        dq[i] =0;
        ddq[i] =0;
        FTemp[i]=FT[i];
    }

   // sixDistalMatrix.sixDistalCollision(q, dq, ddq, FT, sixDistalMatrix.estParasFT, CollisionFT);
    //for (int j = 0; j < 6; j++)
        //FT[j]=FT[j]-CollisionFT[j];


    if (target.count == 1)
    {
        enable_FCPressL=true;
        SetLimit(target,4.0);
        for (int j = 0; j < 6; j++)
        {
            FT0[j]=FT[j];
        }
    }

    for (int j = 0; j < 6; j++)
    {
        FT[j]=FT[j]-FT0[j];
    }


    double FmInWorld[6]={0};
    FT2World(target,FT,FmInWorld);

    if (target.count == 1)
    {
        for (int j = 0; j < 6; j++)
            stateTor0[j] = FmInWorld[j];
    }

    OneOrderFilter(FmInWorld,stateTor0,stateTor1,20);


	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
            controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
		for (int i = 0; i < 7; ++i)
		{
			PqEnd0[i] = PqEnd[i];
		}
		for (int i = 0; i < 6; ++i)
        {
            step_pjs[i] = 0;
            begin_t0[i] = 0;
            begin_pjs[i] = 0;
		}

        target.model->generalMotionPool().at(0).getMpe(pe0);

	}	


    RepeatTrapezoidal(target,begin_pjs,step_pjs);


    aris::dynamic::s_vc(6, pe0, pe);
    pe[0]-=step_pjs[1]*0.08;
    //pe[3]+=step_pjs[1];
    //pe[4]+=step_pjs[1]*0.8;
    //pe[5]-=step_pjs[1]*0.2;
    aris::dynamic::s_pe2pq(pe, PqEnd0);




    //PqEnd0[1] = step_pjs[1];
    //PqEnd0[2] = begin_t0[2]+step_pjs[1]*0.2;
    //PqEnd0[2]=-15;

    ////////////////////////////////////////////////////compute dX(ee actual velocity)////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dynamics //
    for (int i = 0; i < 6; ++i)
    {
        target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
        target.model->motionPool()[i].setMv(controller->motionAtAbs(i).actualVel());
        target.model->motionPool()[i].setMa(0.0);
    }
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
    if (fwd.kinPos())return -1;
    fwd.kinVel();
    fwd.cptJacobiWrtEE();
    fwd.cptGeneralInverseDynamicMatrix();
    target.model->solverPool()[2].dynAccAndFce();


	double va[6] = { 0 }, dX[6] = { 0 };
	for (int i = 0;i < 6;i++)
	{
		va[i] = controller->motionAtAbs(i).actualVel();
	}
	s_mm(6, 1, 6, fwd.Jf(), va, dX);


    ////////////////////////////////////////////////////step 1. compute vt(ee target velocity, pos loop)//////////////////////////////////////////////////////////////////////////////////////////////////
    double vt[6] = { 0 };
    double KPP[6] = {50,50,50,10,10,10};


    // linear vt //
	for (int i = 0; i < 3; ++i)
	{
        vt[i] = KPP[i] * (PqEnd0[i]- PqEnd[i]);
	}

    // angular vt //
    double dqt[4]={0};
    q2dqt(PqEnd, PqEnd0, dqt, KPP[4]);
    aris::dynamic::s_wq2wa(PqEnd + 3, dqt, vt+3);

    // protect vt max and min //
    const double vt_max[6]{0.2,0.2,0.2,0.1,0.1,0.1};
    const double vt_min[6]{-0.2,-0.2,-0.2,-0.1,-0.1,-0.1};
    for (int i = 0; i < 6; ++i)
    {
        vt[i] = std::max(vt_min[i], vt[i]);  // protect vt
        vt[i] = std::min(vt_max[i], vt[i]);  // protect vt
    }

    // force control vt //


    ////////////////////////////////////////////////////step 2. compute at(ee target acceleration, vel loop)//////////////////////////////////////////////////////////////////////////////////////////////////
    double at[6] = { 0 },ft[6] = { 0 };

    const double KPV[6] = {100, 100, 100, 200, 200, 200};
    const double KIV[6] = {1000, 1000, 1000, 10000, 10000, 10000};

    static double ErrSumVt[7] = { 0 };
    const double err_sum_max[6]{0.2, 0.2, 0.2, 0.5, 0.5, 0.5};
    const double err_sum_min[6]{-0.2, -0.2, -0.2, -0.5, -0.5, -0.5};

    // acc control //
	for (int i = 0; i < 6; ++i)
	{
		ErrSumVt[i] = ErrSumVt[i] + (vt[i]-dX[i])*0.001;
        ErrSumVt[i] = std::min(ErrSumVt[i], err_sum_max[i]);  // protect integral value
        ErrSumVt[i] = std::max(ErrSumVt[i], err_sum_min[i]);  // protect integral value
        at[i] = KPV[i] * (vt[i]-dX[i]) + KIV[i]*ErrSumVt[i];
	}

    // fce control //
    double vt_motion_max = 0.05;
    static double err_sum_fce_vt = 0.0;

    if(target.count < 1000) vt_motion_max = 0.05;
    else vt_motion_max = 0.5;


    // protect max and min velocity //
    const int motion = 2;
   if(dX[motion] > vt_motion_max)
    {
        err_sum_fce_vt += (vt_motion_max - dX[motion]) * 0.001;
        at[motion] = KPV[motion] * (vt_motion_max-dX[motion]) + KIV[motion] * err_sum_fce_vt;
    }
    else if(dX[motion] < -vt_motion_max)
    {
        err_sum_fce_vt += (-vt_motion_max - dX[motion]) * 0.001;
        at[motion] = KPV[motion] * (-vt_motion_max-dX[motion]) + KIV[motion] * err_sum_fce_vt;
    }
    else
    {
        if(err_sum_fce_vt < 0.0)
        {
            err_sum_fce_vt += (vt_motion_max - dX[motion]) * 0.001;
            err_sum_fce_vt = std::min(0.0, err_sum_fce_vt);
        }
        else
        {
            err_sum_fce_vt += (-vt_motion_max - dX[motion]) * 0.001;
            err_sum_fce_vt = std::max(0.0, err_sum_fce_vt);
        }

        at[motion] = KIV[motion] * err_sum_fce_vt;
    }

    ////////////////////////////////////////////////////step 3. compute ft(ee target force, acc 2 fce)//////////////////////////////////////////////////////////////////////////////////////////////////
    // inertia matrix //
    double A[36]={0};
    f(target.model, A);
    s_mm(6, 1, 6, A, at,ft);

    // fce control //
    static double SumdX=0, SumFt=0;

    if(target.count==1)
    {
        SumdX=0;
        SumFt=0;
    }

    double Vmin=-0.02;
    const double KPF=5, KIF=5, vis = 500;
    //const double KPF=15, KIF=5, vis = 1000;
    double target_f=10;
    SumdX=SumdX+(Vmin-dX[motion])*0.001;
    SumFt = SumFt+(target_f - stateTor1[motion])*0.001;
    ft[motion]-=KPF * (target_f - stateTor1[motion]) + KIF*SumFt + vis*dX[motion];
    //ft[motion]-=KPF * (target_f - stateTor1[motion])+KIF*SumFt+vis*dX[motion];
    //ft[motion]=KPF * (target_f - stateTor1[motion])+KIF*SumFt+0*dXKI*SumdX+vis*dX[motion];
    //ft[motion]=-ft[motion]+0*dXKI*SumdX;

/*
    //Y-Direction
    static double SumdX=0, SumFt=0;
    double Vmin=-0.02,dXKI=2000,vis=000;
    double KPF=5,KIF=5;
    double target_f=-10;

    SumdX=SumdX+(Vmin-dX[motion])*0.001;
    SumFt = SumFt+(target_f - stateTor1[motion])*0.001;
    ft[motion]-=KPF * (target_f - stateTor1[motion])+KIF*SumFt+vis*dX[motion];
    //ft[motion]=-ft[motion]+0*dXKI*SumdX;
*/
    ////////////////////////////////////////////////////step 4. compute tau(caused by ft)//////////////////////////////////////////////////////////////////////////////////////////////////
	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

    double f_static[6] = { 9,9,5,3,2,2 };
	double f_vel_JRC[6] = { 0,0,0,0,0,0 };

	double JoinTau[6] = { 0 };
	s_mm(6, 1, 6, fwd.Jf(), T(6), ft, 1, JoinTau, 1);

    ////////////////////////////////////////////////////step 5. dynamic compensation//////////////////////////////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < 6; i++)
	{
        JoinTau[i] = JoinTau[i] + target.model->motionPool()[i].mfDyn() + f_vel_JRC[i] * va[i] + 0 * f_static[i] * signV(va[i]);
        JoinTau[i] = JoinTau[i] * f2c_index[i];
	}





    ////////////////////////////////////////////////////others//////////////////////////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i < 6; i++)
    {
        JoinTau[i] = std::max(-500.0, JoinTau[i]);
        JoinTau[i] = std::min(500.0, JoinTau[i]);

        controller->motionAtAbs(i).setTargetToq(JoinTau[i]);
    }

    for (int i = 0; i < 6; i++)
        stateTor0[i] = stateTor1[i];




    if (target.count % 300 == 0)
    {
        double err=(PqEnd0[2] - PqEnd[2]);
        cout<<step_pjs[1]<<"****"<<stateTor1[motion]<<"****"<<dX[motion]<<"****"<<at[motion]<<"****"<<err_sum_fce_vt<<std::endl;
    }

    lout << target.count << ","
         << step_pjs[1] << ","
         << PqEnd[1] << ","
         << ft[motion] << ","
         << dX[motion] << ","
         << step_pjs[1] << ","
         << std::endl;

    bool ds_is_all_finished{ true };
    if (!enable_FCPressL)
        for (int i = 0; i < 6; ++i)
        {
            auto ret = controller->motionPool().at(i).disable();
            if (ret)
            {
                ds_is_all_finished = false;
            }
        }

    //将目标电机由电流模式切换到位置模式
    if (target.count==28000)
    {
        for (int i = 0; i < 6; ++i)
        {

                auto &cm = controller->motionPool().at(i);
                controller->motionPool().at(i).setModeOfOperation(8);
                auto ret = cm.mode(8);
                cm.setTargetPos(cm.actualPos());

        }
    }

    return 28000 - target.count;
}

auto ForceDirect::collectNrt(aris::plan::PlanTarget &target)->void
{

    ReSetLimit(target);

}


ForceDirect::ForceDirect(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
        "<Command name=\"FCPressL\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}




static std::atomic_bool enable_mvJoint = true;
struct MoveJointParam
{
    double PressF;
    double SensorType;


};

auto MoveJoint::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveJointParam param;
    for (auto &p : params)
    {
        if (p.first == "PressF")
            param.PressF = std::stod(p.second);
        if (p.first == "SensorType")
            param.SensorType = std::stod(p.second);

    }

    target.param = param;
    target.ret = std::vector<std::pair<std::string, std::any>>();

    for (auto &option : target.mot_options) option |=
        Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS;


    enable_mvJoint=true;

    //读取动力学参数
    auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasFT"));
    for (int i = 0;i < GroupDim;i++)
        sixDistalMatrix.estParasFT[i] = mat0->data().data()[i];

    auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasJoint"));
    for (int i = 0;i < JointGroupDim + 12;i++)
        JointMatrixFT.estParasJoint[i] = mat1->data().data()[i];

    //auto mat3 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("LoadParas"));
    for (int i = 0;i < 10;i++)
        JointMatrixFT.LoadParas[i] = 0;//1 * mat3->data().data()[i];


}
auto MoveJoint::executeRT(PlanTarget &target)->int
{
    auto &param = std::any_cast<MoveJointParam&>(target.param);

    static double step_pjs[6],begin_pjs[6];
    static double FT0[6];

    // 访问主站 //
    auto controller = target.controller;
    // 打印电流 //
    auto &cout = controller->mout();
    // log 电流 //
    auto &lout = controller->lout();

    // 获取当前起始点位置 //
    if (target.count == 1)
    {
        SetLimit(target,4.0);
        for (int i = 0; i < 6; ++i)
        {
            step_pjs[i] = target.model->motionPool()[i].mp();
            begin_pjs[i] = target.model->motionPool()[i].mp();
            controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
        }
        target.controller->logFileRawName("motion_replay");
    }

    if (target.model->solverPool().at(1).kinPos())return -1;

    double dTheta[6] = { 0 };

    double FT[6],FTemp[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);


    double pa[6],va[6],aa[6],ta[6],idealToq[6],idealFT[6];

    for (int i = 0; i < 6; i++)
    {
        pa[i] = controller->motionAtAbs(i).actualPos();
        va[i] = controller->motionAtAbs(i).actualVel();
        aa[i] = 0;
        ta[i] = controller->motionAtAbs(i).actualToq() / f2c_index[i];
    }

    double Acv[12] = {0.8,0.0,0.8,0.0,0.8,0.0,0.5,0.0,0.5,0.0,0.5,0.0};

    JointMatrixFT.JointDragYang(pa, va, aa, ta, JointMatrixFT.estParasJoint, JointMatrixFT.LoadParas, idealToq, Acv);


    sixDistalMatrix.sixDistalCollision(pa, va, aa, FT, sixDistalMatrix.estParasFT, idealFT);
    for (int j = 0; j < 6; j++)
    {
        FTemp[j]=FT[j];
        FT[j]=FT[j]-idealFT[j];
    }

    if (target.count == 1)
    {
        for (int j = 0; j < 6; j++)
        {
            FT0[j] = FT[j];
        }
    }

    double FT_KAI[6];
    for (int i = 0; i < 6; i++)
    {
        FT_KAI[i] = FT[i] - FT0[i];//In KAI Coordinate
    }


    double zero_check[6] = { 0.2,0.2,0.2,0.05,0.05,0.05 };
    for (int i = 0; i < 6; i++)
    {
        if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
            FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
        else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
            FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
    }

    double FmInWorld[6];
    FT2World(target, FT_KAI, FmInWorld);


    ///* Using Jacobian, TransMatrix from ARIS
    auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
    fwd.cptJacobiWrtEE();
    //FmInWorld[2] = 0; FmInWorld[3] = 0; FmInWorld[4] = 0; FmInWorld[5] = 0;
    double JoinTau[6] = { 0 };
    s_mm(6, 1, 6,fwd.Jf() , T(6), FmInWorld, 1, JoinTau, 1);


    double ft_offset[6] = { 0 };
    double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
    double f_static[6] = { 9,9,5,3,2,2 };
    double f_vel_JRC[6] = { 10,10,10,10,10,10 };
    double ExternTau[6] = { 0 };

/*
    //动力学
    for (int i = 0; i < 6; ++i)
    {
        target.model->motionPool()[i].setMp(controller->motionPool()[i].actualPos());
        target.model->motionPool().at(i).setMv(controller->motionAtAbs(i).actualVel());
        target.model->motionPool().at(i).setMa(0.0);
    }

    target.model->solverPool()[1].kinPos();
    target.model->solverPool()[1].kinVel();
    target.model->solverPool()[2].dynAccAndFce();
*/

    for (int i = 0; i < 6; i++)
    {
        //ExternTau[i] = ta[i] -JoinTau[i]- target.model->motionPool()[i].mfDyn() - f_vel_JRC[i] * va[i] - 1 * f_static[i] * signV(va[i]);
        ExternTau[i] = ta[i] -JoinTau[i]- idealToq[i];
        ExternTau[i] = -ExternTau[i];
    }

    double rate=0.05;
    dTheta[0] = JoinTau[0] / 300/rate + ExternTau[0] / 1000/rate;
    dTheta[1] = JoinTau[1] / 500/rate + ExternTau[1] / 1000/rate;
    dTheta[2] = JoinTau[2] / 500/rate + ExternTau[2] / 1000/rate;
    dTheta[3] = JoinTau[3] / 200/rate + ExternTau[3] / 1000/rate;
    dTheta[4] = JoinTau[4] / 300/rate + ExternTau[4] / 1000/rate;
    dTheta[5] = JoinTau[5] / 300/rate + ExternTau[5] / 1000/rate;

    for (int i = 0; i < 6; i++)
    {
        if (dTheta[i] > 0.0006)
            dTheta[i] = 0.0006;
        if (dTheta[i] < -0.0006)
            dTheta[i] = -0.0006;
        //lout << dTheta[i] << ",";
    }

    for (int i = 0; i < 6; i++)
    {
        step_pjs[i] = step_pjs[i] + dTheta[i];
       // target.model->motionPool().at(i).setMp(step_pjs[i]);
    }

    double KP[6]={8,10,10,1,2,0.1};

    double torque_max[6]={300,500,500,300,300,400};
    double torque_min[6]={-300,-500,-500,-300,-300,-400};
    for (int i = 0; i < 6; i++)
    {

     ft_offset[i]=(30*KP[i]*(step_pjs[i]-pa[i])+idealToq[i])*f2c_index[i];
     ft_offset[i] = std::max(torque_min[i], ft_offset[i]);
     ft_offset[i] = std::min(torque_max[i], ft_offset[i]);
     controller->motionAtAbs(i).setTargetToq(ft_offset[i]);
     
    }
/*
    lout << FTemp[0] << ",";lout << FTemp[1] << ",";
    lout << FTemp[2] << ",";lout << FTemp[3] << ",";
    lout << FTemp[4] << ",";lout << FTemp[5] << ",";

    lout << FT_YANG[0] << ",";lout << FT_YANG[1] << ",";
    lout << FT_YANG[2] << ",";lout << FT_YANG[3] << ",";
    lout << FT_YANG[4] << ",";lout << FT_YANG[5] << ","
*/
    lout << pa[0] << " ";lout << pa[1] << " ";
    lout << pa[2] << " ";lout << pa[3] << " ";
    lout << pa[4] << " ";lout << pa[5] << " ";
    lout << std::endl;


    if (target.count % 300 == 0)
    {
        cout<<ft_offset[0]<<"***"<< ft_offset[1]<<"***"<< ft_offset[2]<<"***"<<ft_offset[3]<<"***"<<ft_offset[4]<<std::endl;
    }


    if (!enable_mvJoint)
        for (int i = 0; i < 6; ++i)
            auto ret = controller->motionPool().at(i).disable();




    return 150000000 - target.count;
}


auto MoveJoint::collectNrt(aris::plan::PlanTarget &target)->void
{

    ReSetLimit(target);

}
MoveJoint::MoveJoint(const std::string &name) :Plan(name)
{

    command().loadXmlStr(
        "<Command name=\"mvJoint\">"
        "	<GroupParam>"
        "       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"-20.0\"/>"
        "   </GroupParam>"
        "</Command>");

}


//复现文件位置//
struct ReplayParam
{
    std::vector<aris::Size> total_count_vec;
    std::vector<double> axis_begin_pos_vec;
    std::vector<double> axis_first_pos_vec;
    std::vector<std::vector<double>> pos_vec;
    double vel, acc, dec;
    std::string path;
};
auto Replay::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    ReplayParam param;
    param.vel = std::stod(params.at("vel"));
    param.acc = std::stod(params.at("acc"));
    param.dec = std::stod(params.at("dec"));
    param.path = params.at("path");

    param.total_count_vec.resize(6, 0);
    param.axis_begin_pos_vec.resize(6, 0.0);
    param.axis_first_pos_vec.resize(6, 0.0);

    param.pos_vec.resize(6, std::vector<double>(1, 0.0));
    //std::cout << "size:" << param.pos_vec.size() << std::endl;
    //初始化pos_vec//
    for (int j = 0; j < param.pos_vec.size(); j++)
    {
        param.pos_vec[j].clear();
    }

    //定义读取log文件的输入流oplog//
    std::ifstream oplog;
    int cal = 0;
    oplog.open(param.path);

    //以下检查是否成功读取文件//
    if (!oplog)
    {
        throw std::runtime_error("fail to open the file");
    }
    while (!oplog.eof())
    {
        for (int j = 0; j < param.pos_vec.size(); j++)
        {
            double data;
            oplog >> data;
            param.pos_vec[j].push_back(data);
        }
    }
    oplog.close();
    //oplog.clear();
    for (int j = 0; j < param.pos_vec.size(); j++)
    {
        param.pos_vec[j].pop_back();
        param.axis_first_pos_vec[j] = param.pos_vec[j][0];
    }

    target.param = param;
    std::fill(target.mot_options.begin(), target.mot_options.end(),
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
        Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
    std::vector<std::pair<std::string, std::any>> ret;
    target.ret = ret;
}
auto Replay::executeRT(PlanTarget &target)->int
{
    auto controller = target.controller;
    auto &param = std::any_cast<ReplayParam&>(target.param);

    double p, v, a;
    aris::Size t_count;
    static aris::Size first_total_count = 1;
    aris::Size total_count = 1;
    aris::Size return_value = 0;

    // 获取6个电机初始位置 //
    if (target.count == 1)
    {
        for (int i = 0; i < 6; i++)
        {
            param.axis_begin_pos_vec[i] = target.model->motionPool().at(i).mp();
        }
        for (int i = 0; i < 6; i++)
        {
            // 梯形规划到log开始点 //
            aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
            first_total_count = std::max(first_total_count, t_count);
        }
    }

    // 机械臂走到log开始点 //
    if (target.count <= first_total_count)
    {
        for (int i = 0; i < 6; i++)
        {
            // 在第一个周期走梯形规划复位
            aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
            controller->motionAtAbs(i).setTargetPos(p);
            target.model->motionPool().at(i).setMp(p);
        }
    }

    // 机械臂开始从头到尾复现log中点 //
    if (target.count > first_total_count)
    {
        for (int i = 0; i < 6; i++)
        {
            controller->motionAtAbs(i).setTargetPos(param.pos_vec[i][target.count - first_total_count]);
            target.model->motionPool().at(i).setMp(param.pos_vec[i][target.count - first_total_count]);
        }
    }
    if (target.model->solverPool().at(1).kinPos())return -1;

    //输出6个轴的实时位置log文件//
    auto &lout = controller->lout();
    for (int i = 0; i < 6; i++)
    {
        lout << controller->motionAtAbs(i).actualPos() << " ";//第一列数字必须是位置
    }
    lout << std::endl;

    return target.count > (first_total_count - 2 + param.pos_vec[0].size()) ? 0 : 1;

}
auto Replay::collectNrt(PlanTarget &target)->void {}
Replay::Replay(const std::string &name) :Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"MotionReplay\">"
        "	<GroupParam>"
        "		<Param name=\"path\" default=\"C:\\Users\\kevin\\Desktop\\file\\rt_log--2019-09-06--19-14-16--moveJR.txt\"/>"
        "		<Param name=\"vel\" default=\"0.05\" abbreviation=\"v\"/>"
        "		<Param name=\"acc\" default=\"0.1\" abbreviation=\"a\"/>"
        "		<Param name=\"dec\" default=\"0.1\" abbreviation=\"d\"/>"
        "	</GroupParam>"
        "</Command>");
}










struct MovePressureToolXSineParam
{
	double PressF;
	double SensorType;


};

auto MovePressureToolXSine::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MovePressureToolXSineParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);
		if (p.first == "SensorType")
			param.SensorType = std::stod(p.second);
	}

	target.param = param;
    target.ret = std::vector<std::pair<std::string, std::any>>();

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
		Plan::NOT_CHECK_ENABLE;

	SetLimit(target, 6.0);


}
auto MovePressureToolXSine::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureToolXSineParam&>(target.param);

	static double step_pjs[6];
    static double stateTor0[6], stateTor1[6], EndP0[3];
    static double sT0[6]={0}, sT1[6]={0};
    static double FT0[6];

    int FTnum=50;
    static double FTten[50]={0};

	// 访问主站 //
	auto controller = target.controller;
	auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
	double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
	double CrossNormalZbase[3] = { 0 };

	

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	double FT[6];
	if (param.SensorType > 0)
		GetATI(target, FT);
	else
		GetYuLi(target, FT);



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
            stateTor0[j] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}


    //SecondOrderFilter(FT, stateTor0, stateTor1, 80);

    OneOrderFilter(FT,stateTor0,stateTor1,80);

	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
        FT_KAI[i] = stateTor1[i] - FT0[i];//In KAI Coordinate
	}

	double zero_check[6] = { 1,1,1,0.05,0.05,0.05 };
	for (int i = 0; i < 6; i++)
	{
		if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
			FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
			FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}


	double dXpid[6] = { 0,0,0,0,0,0 };
    static double sumFor[6]={0};
    for (int i=0;i<6;i++)
        sumFor[i]=sumFor[i]+(FT_KAI[2] - (-5))*0.001;

    bool flag=true;
    for(int i=0;i<FTnum;i++)
        if(abs(FTten[i])>4)
            flag=false;


    if(flag)
       dXpid[2] = 1 * (FT_KAI[2] - (-5)) / 80000;
    else
    {
       //flag=false;
        sumFor[2]=0;
       dXpid[2] = 1 * (FT_KAI[2] - (-5)) / 80000;
       //if(abs(FT_KAI[2])<2)
          // flag=true;
    }

	dXpid[3] = 0 * (FT_KAI[3]) / 2000;
	dXpid[4] = 0 * (FT_KAI[4]) / 2000;
	dXpid[5] = 0 * (FT_KAI[5]) / 2000;


    if (target.count == 1)
        for (int j = 0; j < 6; j++)
            sT0[j] = dXpid[j];


    OneOrderFilter(dXpid,sT0,sT1,40);




	double FmInWorld[6];

    FT2World(target, sT1, FmInWorld);

	static double amp = 0;
    if (target.count < 10000)
        amp = amp + 0.000003;

    dX[0] = amp*sin(2 * aris::PI / 4 * target.count / 1000.0)/1000.0;
	dX[1] = 0;
    dX[2] = FmInWorld[2];
	dX[3] = 0; dX[4] = 0; dX[5] = 0;



	for (int j = 0; j < 6; j++)
	{
        if (dX[j] > 0.00025)
            dX[j] = 0.00025;
        if (dX[j] < -0.00025)
            dX[j] = -0.00025;
	}

	// log 电流 //
	auto &lout = controller->lout();
    lout << FT[0] << ",";lout << FT[1] << ",";
    lout << FT[2] << ",";lout << FT[3] << ",";
    lout << FT[4] << ",";lout << FT[5] << ",";

    lout << stateTor1[0] << ",";lout << stateTor1[1] << ",";
    lout << stateTor1[2] << ",";lout << stateTor1[3] << ",";
    lout << stateTor1[4] << ",";lout << stateTor1[5] << ",";

	lout << dX[0] << ",";lout << dX[1] << ",";
	lout << dX[2] << ",";lout << dX[3] << ",";
	lout << dX[4] << ",";lout << dX[5] << ",";
	lout << std::endl;



	dX2dTheta(target, dX, dTheta);

	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
        step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}


    if (target.count % 100 == 0)
	{

        cout << FTten[0] << "*" << FTten[1] <<"*"<<dX[2]<<"*"<<flag;
		cout << std::endl;

	}

	for (int i = 0; i < 6; i++)
	{

        stateTor0[i] = stateTor1[i];
        sT0[i] = sT1[i];


	}

    if(target.count<FTnum+1)
       FTten[target.count-1]=FT_KAI[2];
    else
    {
        for(int i=0;i<FTnum-1;i++)
            FTten[i]=FTten[i+1];

       FTten[FTnum-1]=FT_KAI[2];

    }


    return 15000000 - target.count;

}

auto MovePressureToolXSine::collectNrt(aris::plan::PlanTarget &target)->void
{

	ReSetLimit(target);

}

MovePressureToolXSine::MovePressureToolXSine(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvPreTXSine\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}


struct MoveForceXSineParam
{
	double PressF;
	double SensorType;


};

auto MoveForceXSine::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveForceXSineParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);
		if (p.first == "SensorType")
			param.SensorType = std::stod(p.second);
	}

	target.param = param;
	target.ret = std::vector<std::pair<std::string, std::any>>();

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_ENABLE;

	SetLimit(target, 6.0);


}
auto MoveForceXSine::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveForceXSineParam&>(target.param);

	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
	static double sT0[6][3], sT1[6][3];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;
	auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
	double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
	double CrossNormalZbase[3] = { 0 };



	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	double FT[6];
	if (param.SensorType > 0)
		GetATI(target, FT);
	else
		GetYuLi(target, FT);



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}


    SecondOrderFilter(FT, stateTor0, stateTor1, 80);


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}

	double zero_check[6] = { 1,1,1,0.05,0.05,0.05 };
	for (int i = 0; i < 6; i++)
	{
		if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
			FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
			FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

    double FmInWorld[6];

	FT2World(target, FT_KAI, FmInWorld);

	double dXpid[6] = { 0,0,0,0,0,0 };
    const int motion = 1;
    dXpid[motion] = 1 * (FmInWorld[motion] - (-5)) / 420000;
	dXpid[3] = 0 * (FmInWorld[3]) / 2000;
	dXpid[4] = 0 * (FmInWorld[4]) / 2000;
	dXpid[5] = 0 * (FmInWorld[5]) / 2000;


	static double amp = 0;
	if (target.count < 10000)
		amp = amp + 0.000003;

	dX[0] = amp*sin(2 * aris::PI / 4 * target.count / 1000.0) / 1000.0;
    dX[1] = dXpid[1];
    dX[2] = dXpid[2];

	dX[3] = 0; dX[4] = 0; dX[5] = 0;



	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}

	// log 电流 //
	auto &lout = controller->lout();
	lout << FT[0] << ",";lout << FT[1] << ",";
	lout << FT[2] << ",";lout << FT[3] << ",";
	lout << FT[4] << ",";lout << FT[5] << ",";

	lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
	lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
	lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";

	lout << dX[0] << ",";lout << dX[1] << ",";
	lout << dX[2] << ",";lout << dX[3] << ",";
	lout << dX[4] << ",";lout << dX[5] << ",";
	lout << std::endl;



	dX2dTheta(target, dX, dTheta);

	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}


	if (target.count % 300 == 0)
	{

        cout << FmInWorld[0] << "*" << FmInWorld[1] << "*" << FmInWorld[2] << "*" << step_pjs[3] << "*" << step_pjs[4] << "*" << FmInWorld[motion] << std::endl;
		cout << std::endl;

	}

	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

	}

	return 15000000 - target.count;

}

auto MoveForceXSine::collectNrt(aris::plan::PlanTarget &target)->void
{

	ReSetLimit(target);

}

MoveForceXSine::MoveForceXSine(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvForXSine\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}




struct MoveForceCircleParam
{
	double PressF;
	double SensorType;


};

auto MoveForceCircle::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveForceCircleParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);
		if (p.first == "SensorType")
			param.SensorType = std::stod(p.second);
	}

	target.param = param;
	target.ret = std::vector<std::pair<std::string, std::any>>();

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_ENABLE;

	SetLimit(target, 6.0);


}
auto MoveForceCircle::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveForceCircleParam&>(target.param);

	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
	static double sT0[6][3], sT1[6][3];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;
	auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
			// controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
		}
	}


	if (target.model->solverPool().at(1).kinPos())return -1;


	double PqEnd[7], TransVector[16], NormalVector[3], CosNormalAng, SinNormalAng, NormalAng;
	double XBase[3] = { 1,0,0 }, YBase[3] = { 0,1,0 }, ZBase[3] = { 0,0,1 };
	double CrossNormalZbase[3] = { 0 };



	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	double FT[6];
	if (param.SensorType > 0)
		GetATI(target, FT);
	else
		GetYuLi(target, FT);



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}


	SecondOrderFilter(FT, stateTor0, stateTor1, 80);


	double FT_KAI[6];
	for (int i = 0; i < 6; i++)
	{
		FT_KAI[i] = stateTor1[i][0] - FT0[i];//In KAI Coordinate
	}

	double zero_check[6] = { 1,1,1,0.05,0.05,0.05 };
	for (int i = 0; i < 6; i++)
	{
		if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
			FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
			FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
	}

	double FmInWorld[6];

	FT2World(target, FT_KAI, FmInWorld);

	double dXpid[6] = { 0,0,0,0,0,0 };
	const int motion = 1;
    double ForRadius=-sqrt(FmInWorld[0]*FmInWorld[0]+FmInWorld[1]*FmInWorld[1]);
    double RadiusAdd = (ForRadius - (-3)) / 620000;
    RadiusAdd = std::max(-0.000025, RadiusAdd);
    RadiusAdd = std::min(0.000025, RadiusAdd);


	static double radius = 0;

    if (radius < 10000)
    radius = radius + RadiusAdd;
	
	double time = 1 * target.count / 1000.0;

    dX[0] = (RadiusAdd * sin(time)+radius * cos(time)) / 1000.0;
    dX[1] = (RadiusAdd * cos(time)-radius * sin(time)) / 1000.0;
	dX[2] = 0;

	dX[3] = 0; dX[4] = 0; dX[5] = 0;



	for (int j = 0; j < 6; j++)
	{
        if (dX[j] > 0.00025)
            dX[j] = 0.00025;
        if (dX[j] < -0.00025)
            dX[j] = -0.00025;
	}

	// log 电流 //
	auto &lout = controller->lout();
	//lout << FT[0] << ",";lout << FT[1] << ",";
	//lout << FT[2] << ",";lout << FT[3] << ",";
	//lout << FT[4] << ",";lout << FT[5] << ",";

	//lout << stateTor1[0][0] << ",";lout << stateTor1[1][0] << ",";
	//lout << stateTor1[2][0] << ",";lout << stateTor1[3][0] << ",";
	//lout << stateTor1[4][0] << ",";lout << stateTor1[5][0] << ",";

	lout << dX[0] << ",";lout << dX[1] << ",";
	lout << dX[2] << ",";lout << dX[3] << ",";
	lout << dX[4] << ",";lout << dX[5] << ",";
	lout << std::endl;



	dX2dTheta(target, dX, dTheta);

	for (int i = 0; i < 6; i++)
	{
		if (dTheta[i] > 0.003)
			dTheta[i] = 0.003;
		if (dTheta[i] < -0.003)
			dTheta[i] = -0.003;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}


	if (target.count % 300 == 0)
	{

        cout << FmInWorld[0] << "*" << FmInWorld[1] << "*" << dX[0] << "*" << radius << "*" << ForRadius << "*" << RadiusAdd << std::endl;
		cout << std::endl;

	}

	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

	}

	return 35000 - target.count;

}

auto MoveForceCircle::collectNrt(aris::plan::PlanTarget &target)->void
{

	ReSetLimit(target);

}

MoveForceCircle::MoveForceCircle(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvForC\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}



double P1[7] = {  0.2957561877690258,-0.4453456380218208,0.2771034,-0.707070,-0.70714,0,0 };
double P2[7] = { 0.34893612065625784,-0.41170589555940784,0.27707736,-0.7070685,-0.7071450,0,0};
double P3[7] = { 0.30910799,-0.34915346,0.27707300,-0.707084,-0.7071295,0,0 };
double P4[7] = { 0.2563542,-0.3824533,0.2770473,-0.707082,-0.70713,0,0};
double P5[7] = { 0.25856243,-0.42001145,0.284604726,-0.7070756,-0.707137,0,0};
static long start_count = 0;

void PressLine(PlanTarget &target, const double *FmInWorld, const double *P1, const double *P2, const double addLength,double *dX,bool &flag)
{
    double pArc, vArc, aArc, vArcMax = 0.025;
    aris::Size t_count = 0;

    double dir[3] = { 0 }, vertic[3] = { 0 }, zbase[3] = { 0,0,1 };
    double length = 0;

    length = sqrt((P2[0] - P1[0])*(P2[0] - P1[0]) + (P2[1] - P1[1])*(P2[1] - P1[1]));
    dir[0] = (P2[0] - P1[0]) / length;
    dir[1] = (P2[1] - P1[1]) / length;
    length = sqrt((P2[0] - P1[0])*(P2[0] - P1[0]) + (P2[1] - P1[1])*(P2[1] - P1[1])) + addLength;
    aris::plan::moveAbsolute(target.count-start_count+1, 0, length, vArcMax / 1000, 0.02 / 1000 / 1000, 0.02 / 1000 / 1000, pArc, vArc, aArc, t_count);

    if ((target.count - start_count-t_count) < 0.5&&(target.count - start_count-t_count) >- 0.5)
      {
        flag = true;
    }

    double dX0[6] = { 0 };
    dX0[0] = vArc * dir[0];
    dX0[1] = vArc * dir[1];
    dX0[2] = 0;


    crossVector(zbase, dir, vertic);
    double xy_desired[2] = { 0 };
    xy_desired[0] = 10 * vertic[0];
    xy_desired[1] = 10 * vertic[1];

    //if(PqEnd[1]>-0.393&&PqEnd[1]<-0.385)
        //xy_desired[0] = 0;


    double dXpid[6] = { 0,0,0,0,0,0 };
    dXpid[0] = 1 * (FmInWorld[0] - xy_desired[0]) / 720000;
    dXpid[1] = 1 * (FmInWorld[1] - xy_desired[1]) / 720000;

    for (int i = 0;i < 6;i++)
        dX[i] = dX0[i] + dXpid[i];


}


static std::atomic_bool enable_FCPressP = true;
struct MoveForceCurveParam
{
	double PressF;
	double SensorType;

	double p1x, p1y, p2x, p2y;
};

auto MoveForceCurve::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveForceCurveParam param;
	for (auto &p : params)
	{
		if (p.first == "PressF")
			param.PressF = std::stod(p.second);
		if (p.first == "SensorType")
			param.SensorType = std::stod(p.second);

        if (p.first == "PP1")
			param.p1x = std::stod(p.second);
        if (p.first == "PP2")
			param.p1y = std::stod(p.second);
        if (p.first == "PP3")
			param.p2x = std::stod(p.second);
        if (p.first == "PP4")
			param.p2y = std::stod(p.second);
        if (p.first == "PP5")
			param.p2y = std::stod(p.second);
	}

	target.param = param;
	target.ret = std::vector<std::pair<std::string, std::any>>();
    enable_FCPressP=true;
	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		Plan::NOT_CHECK_VEL_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
            Plan::NOT_CHECK_ENABLE;





}
auto MoveForceCurve::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveForceCurveParam&>(target.param);

	static double step_pjs[6];
	static double stateTor0[6][3], stateTor1[6][3], EndP0[3];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;
	auto &cout = controller->mout();
	// 获取当前起始点位置 //
	if (target.count == 1)
		for (int i = 0; i < 6; ++i)
			step_pjs[i] = target.model->motionPool()[i].mp();


	if (target.model->solverPool().at(1).kinPos())return -1;


    double PqEnd[7];
    target.model->generalMotionPool().at(0).getMpq(PqEnd);

	double dX[6] = { 0.00000, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

//////////////////////////////////////////////////Get FT in World Framework, Filter////////////////////////////////////////////
    double FT[6]={0};
	if (param.SensorType > 0)
		GetATI(target, FT);
	else
		GetYuLi(target, FT);


	// 获取当前起始点位置 //
	if (target.count == 1)
	{
        enable_FCPressP=true;
         SetLimit(target, 4.0);
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}


    //SecondOrderFilter(FT, stateTor0, stateTor1, 280);


    double FT_KAI[6]={0};
	for (int i = 0; i < 6; i++)
	{
        FT_KAI[i] = FT[i] - FT0[i];//In KAI Coordinate
	}
/*
	double zero_check[6] = { 1,1,1,0.05,0.05,0.05 };
	for (int i = 0; i < 6; i++)
	{
		if (FT_KAI[i] < zero_check[i] && FT_KAI[i]>0)
			FT_KAI[i] = 1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
		else if (FT_KAI[i]<0 && FT_KAI[i]>-zero_check[i])
			FT_KAI[i] = -1 / zero_check[i] * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
    }*/

	double FmInWorld[6];
	FT2World(target, FT_KAI, FmInWorld);
	
    ////////////////////////////////////////////////////////Press Profile Determined by P1-P2-P3-P4-P5////////////////////////////
        static char line_mark = 'A';
        static bool begin_flag = true;
        if(target.count==1)
        {
            line_mark = 'A';
            begin_flag = true;
            start_count=0;
        }


        bool finish_flag = false;
        //static double addLength[5] = { -0.018,-0.025,-0.010,-0.015,0.01 };
        //static double addLength[5] = { 0.008,0.001,0.023,0.005,0.025};
        double addLength[5] = { -0.00,-0.00,0.000,0.000,0.000};
        switch (line_mark)
        {
        case 'A':
            if (begin_flag)
            {
                start_count = target.count;
                begin_flag = false;
            }
                PressLine(target, FmInWorld, P1, P2, addLength[0], dX,finish_flag);
                if (finish_flag)
                {
                    begin_flag = true;
                    finish_flag = false;
                    line_mark = 'B';
                 }
                break;
        case 'B':
            if (begin_flag)
            {
                start_count = target.count;
                begin_flag = false;
            }
            PressLine(target, FmInWorld, P2, P3, addLength[1], dX, finish_flag);
            if (finish_flag)
            {
                begin_flag = true;
                finish_flag = false;
                line_mark = 'C';
            }
                break;
        case 'C':
            if (begin_flag)
            {
                start_count = target.count;
                begin_flag = false;
            }
            PressLine(target, FmInWorld, P3, P4, addLength[2], dX, finish_flag);
            if (finish_flag)
            {
                begin_flag = true;
                finish_flag = false;
                line_mark = 'D';
            }
                break;
        case 'D':
            if (begin_flag)
            {
                start_count = target.count;
                begin_flag = false;
            }
            PressLine(target, FmInWorld, P4, P5, addLength[3], dX, finish_flag);
            if (finish_flag)
            {
                begin_flag = true;
                finish_flag = false;
                line_mark = 'E';
            }
                break;
        case 'E':
            if (begin_flag)
            {
                start_count = target.count;
                begin_flag = false;
            }
            PressLine(target, FmInWorld, P5, P1, addLength[4], dX, finish_flag);
            if (finish_flag)
            {
                begin_flag = true;
                finish_flag = false;
                line_mark = 'A';
            }
                break;
        default:
            cout << "curve finished" << std::endl;

        }



/////////////////////////////////////////////////////dX to dTheta, Generate Motor Position/////////////////////////////////////////////////
	for (int j = 0; j < 6; j++)
	{
        if (dX[j] > 0.00025)
            dX[j] = 0.00025;
        if (dX[j] < -0.00025)
            dX[j] = -0.00025;
	}

	dX2dTheta(target, dX, dTheta);

	for (int i = 0; i < 6; i++)
	{
        if (dTheta[i] > 0.0025)
            dTheta[i] = 0.0025;
        if (dTheta[i] < -0.0025)
            dTheta[i] = -0.0025;
		//lout << dTheta[i] << ",";
	}


	//lout << std::endl;
	for (int i = 0; i < 6; i++)
	{
		dTheta[i] = dTheta[i] * DirectionFlag[i];

	}

	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}

////////////////////////////////////////////////////////////////Display Real Press Force/////////////////////////////////////
    static double PressF0[1] = { 0 }, PressF1[1] = { 0 };
	double normal_force[1];
	normal_force[0]= sqrt(FmInWorld[0] * FmInWorld[0] + FmInWorld[1] * FmInWorld[1]);
	if (target.count == 1)
		PressF0[0] = normal_force[0];

    OneOrderFilter(normal_force, PressF0, PressF1, 5);


///////////////////////////////////////////////////////////////Filter Management/////////////////////////////////////////////
	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];

	}
	PressF0[0] = PressF1[0];
///////////////////////////////////////////////////////////////Print and Save/////////////////////////////////////////////////
	// log 电流 //
	auto &lout = controller->lout();

    lout << target.count << ",";lout << FT[1] << ",";
	lout << dX[0] << ",";lout << dX[1] << ",";
	lout << FmInWorld[0] << ",";lout << FmInWorld[1] << ",";
	lout << PqEnd[0] << ",";lout << PqEnd[1] << ",";
	lout << std::endl;
    //if (target.count % 100 == 0)
	{

        cout << target.count  << "*" << FmInWorld[0]  << "**"<<FmInWorld[1]<<"**"<<FmInWorld[2]<<"*" << FT0[0]  << "**"<<FT0[1]<<"**"<<FT0[2]<<std::endl;
        //cout << std::endl;

	}

	

/////////////////////////////////////////////////////////////Function Exit//////////////////////////////////////////////////
	if (line_mark == 'F')
		return 0;


    if (!enable_FCPressP)
        for (int i = 0; i < 6; ++i)
            auto ret = controller->motionPool().at(i).disable();

    return 28000 - target.count;

}

auto MoveForceCurve::collectNrt(aris::plan::PlanTarget &target)->void
{

	ReSetLimit(target);

}

MoveForceCurve::MoveForceCurve(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"FCPressP\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
		"		<Param name=\"SensorType\"default=\"-20.0\"/>"
        "		<Param name=\"PP1\"default=\"0.29342\"/>"
        "		<Param name=\"PP2\"default=\"-0.43863428\"/>"
        "		<Param name=\"PP3\"default=\"0.34742376\"/>"
        "		<Param name=\"PP4\"default=\"-0.406617\"/>"
        "		<Param name=\"PP5\"default=\"-0.406617\"/>"
		"   </GroupParam>"
		"</Command>");

}




// 力控停止指令——停止FCStop，去使能电机 //

auto FCStop::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        enable_mvJoint = false;
        enable_FCPressL=false;
        enable_FCPressP=false;
        target.option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

    }
FCStop::FCStop(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"FCStop\">"
            "</Command>");
    }






// 获取part_pq，end_pq，end_pe等 //
struct GetForceParam
{

    double set_force,press_force;

};
auto GetForce::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    GetForceParam par;

    std::any param = par;
    //std::any param = std::make_any<GetParam>();


    target.server->getRtData([&](aris::server::ControlServer& cs, const aris::plan::PlanTarget *target, std::any& data)->void
    {

        auto ec = dynamic_cast<aris::control::EthercatController*>(&cs.controller());


    }, param);

    auto out_data = std::any_cast<GetForceParam &>(param);


    std::vector<std::pair<std::string, std::any>> out_param;

    out_param.push_back(std::make_pair<std::string, std::any>("set_force", out_data.set_force));
    out_param.push_back(std::make_pair<std::string, std::any>("press_force", out_data.press_force));

    target.ret = out_param;
    target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
auto GetForce::collectNrt(PlanTarget &target)->void {}
GetForce::GetForce(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"get\">"
        "</Command>");
}



