#include "sixdistalfc.h"
#include <math.h>
#include"kaanh.h"
#include <algorithm>
#include"robotconfig.h"
#include"sixdistaldynamics.h"
#include <vector>
//using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;
using namespace CONFIG;
using namespace sixDistalDynamicsInt;
/// \brief

robotconfig robotDemo;
sixdistaldynamics sixDistalMatrix;


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


void crossVector(const double* a, const double* b, double* c)
{

	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = -(a[0] * b[2] - b[0] * a[2]);
	c[2] = a[0] * b[1] - b[0] * a[1];

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
	static float FT0[6], FT_be[6];

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
    static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { param.VEL,param.VEL,param.VEL,param.VEL,param.VEL,param.VEL };
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
        "		<Param name=\"SensorType\"default=\"1.0\"/>"
		"		<Param name=\"A5P\"default=\"0.0\"/>"
		"		<Param name=\"A5N\" default=\"0.0\"/>"
		"		<Param name=\"A6P\"default=\"0.0\"/>"
		"		<Param name=\"A6N\" default=\"0.0\"/>"
		"		<Param name=\"VEL\" default=\"0.15\"/>"
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
	static float FT0[6], FT_be[6];
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

	float FT[6];
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




}
auto MovePressureToolXY::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MovePressureToolXYParam&>(target.param);

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
	static float FT0[6], FT_be[6];

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

    double FT[6];
    if(param.SensorType>0)
        GetATI(target,FT);
    else
        GetYuLi(target,FT);



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
		for (int i = 0;i < 3;i++)
			EndP0[i] = PqEnd[i];
	}




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

	double FT_YANG[6];
    FT_YANG[0] = -dXpid[0];FT_YANG[1] = -dXpid[1];FT_YANG[2] = dXpid[2];
    FT_YANG[3] = -dXpid[3];FT_YANG[4] = -dXpid[4];FT_YANG[5] = dXpid[5];


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

	for (int i = 0;i < 6;i++)
        dX[i] = FmInWorld[i];


	double TangentArc[3] = { 0 };
	static double TangentArc0[3] = { 0 };
	static double TangentArc1[3] = { 0 };
	static double TangentArc2[3] = { 0 };
	static bool MoveDirection = true;
	static bool MoveDirectionT = true, MoveDirectionF = false;
	static bool MoveDirectionChange = false;
    static int StartCount = 15000;
	double CosTheta1, CosTheta2;




    static double pArc, vArc, aArc, vArcMax = 0.05;
	aris::Size t_count;

    double Square[4][3] = { {-0.1,0.46,0},
                            {0.2,0.46,0},
                            {0.2,0.50,0},
                            {-0.1,0.50,0} };


	static double MoveLength = 0;
    static double DecLength = 0.01, LengthT = 0.2, LengthF = 0.00005;//LengthT>LengthF

	LengthT = sqrt((Square[0][0] - Square[1][0])*(Square[0][0] - Square[1][0]) + (Square[0][1] - Square[1][1])*(Square[0][1] - Square[1][1]));
	double CountFmax = sqrt((Square[2][0] - Square[1][0])*(Square[2][0] - Square[1][0]) + (Square[2][1] - Square[1][1])*(Square[2][1] - Square[1][1])) / LengthF;


	double DecTime = 0, Dec = 0;
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

			TangentArc2[0] = -ExtendSurface[0]; TangentArc2[1] = ExtendSurface[1]; TangentArc2[2] = ExtendSurface[2];

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
			TangentArc1[2] = -(NormalVector[0] * TangentArc0[0] + NormalVector[1] * TangentArc0[1]) / NormalVector[2];

			CosTheta1 = TangentArc1[0] * TangentArc0[0] + TangentArc1[1] * TangentArc0[1] + TangentArc1[2] * TangentArc0[2];

			temp0 = ExtendSurface[0] * ExtendSurface[0] + ExtendSurface[1] * ExtendSurface[1];
			temp1 = (ExtendSurface[0] * NormalVector[0] + ExtendSurface[1] * NormalVector[1]) / NormalVector[2];
			Ktemp = -1 / sqrt(temp0 + temp1 * temp1);

			TangentArc2[0] = Ktemp * ExtendSurface[0];
			TangentArc2[1] = Ktemp * ExtendSurface[1];
			TangentArc2[2] = -(NormalVector[0] * TangentArc0[0] + NormalVector[1] * TangentArc0[1]) / NormalVector[2];

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
			TangentArc1[2] = -(NormalVector[0] * TangentArc0[0] + NormalVector[1] * TangentArc0[1]) / NormalVector[2];

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
            dX[2] = dX[2]+vArc * TangentArc[2] / 1000;
            dX[1] = dX[1]+vArc * TangentArc[1] / 1000;
		}
		else
		{
            dX[0] = dX[0]+vArc * TangentArc[0] / 1000;
            dX[2] = dX[2]+vArc * TangentArc[2] / 1000;
            dX[1] = dX[1]+vArc * TangentArc[1] / 1000;
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
	if (target.count % 300 == 0)
	{

        cout << FT_KAI[2] << "*" << TangentArc[0] <<"*"<<TangentArc[1] << "*" << TangentArc[2] << "*" << FT0[2] << std::endl;

		//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[0] << "*" << TransVector[1] << "*" << TransVector[2] << "*" << FT0[2] << endl;
		//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[4] << "*" << TransVector[5] << "*" << TransVector[6] << "*" << FT0[2] << endl;
		//cout << FT_KAI[2] << "*" << NormalAng << "*" << TransVector[8] << "*" << TransVector[9] << "*" << TransVector[10] << "*" << FT0[2] << endl;

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

		sT0[i][0] = sT1[i][0];
		sT0[i][1] = sT1[i][1];
		sT0[i][2] = sT1[i][2];

	}

	for (int j = 0; j < 6; j++)
	{
		FT_be[j] = FT[j];
	}
	for (int j = 0; j < 3; j++)
	{
		TangentArc0[j] = TangentArc[j];
	}

	return 150000000 - target.count;

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




auto GetForce::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{

	double FT[2] = { 0,0 };

	std::any cur_a = double(0);
    /*target.server->getRtData([&](aris::server::ControlServer& cs, std::any &data)->void

	{
		FT[0] = TimeToMeng;
		FT[1] = ForceToMeng;
		//std::any_cast<double&>(data) = cs.controller().motionPool().at(i).actualCur();
    }, cur_a);*/


	std::string ret(reinterpret_cast<char*>(&FT), 2 * sizeof(double));
	target.ret = ret;
	target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
auto GetForce::collectNrt(PlanTarget &target)->void {}
GetForce::GetForce(const std::string &name) : Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"GetForce\">"
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
	static float FT0[6], FT_be[6];
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

	float FT[6];
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
void PIDcalTeo(double m, double h, double ts, double overshoot,double *KP, double *KI)
{
	double temp = log(overshoot);
	double kesi = 1 / sqrt(1 + aris::PI*aris::PI / temp / temp);
	double omega = 4 / kesi / ts;

	KI[0] = omega * omega * m;
	KP[0] = 2 * kesi *omega * m - h;
}

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

	for (auto &option : target.mot_options) option |=
		Plan::USE_TARGET_POS |
		//#ifdef WIN32
		Plan::NOT_CHECK_POS_MIN |
		Plan::NOT_CHECK_POS_MAX |
		Plan::NOT_CHECK_POS_CONTINUOUS |
		Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
		Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
		//#endif
		Plan::NOT_CHECK_VEL_CONTINUOUS |
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
		Plan::NOT_CHECK_ENABLE;


}
auto ForceDirect::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<ForceDirectParam&>(target.param);

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
	static float FT0[6], FT_be[6];

	// 访问主站 //
	auto controller = target.controller;
    auto &cout = controller->mout();

	static double PqEnd0[7] = { 0 }, PqEnd[7] = { 0 };
	static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
	double end_pm[16];
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
        for (int j = 0; j < 6; j++)
        {
            FT0[j]=FT[j];
        }
    }

    for (int j = 0; j < 6; j++)
    {
        FT[j]=FT[j]-FT0[j];
    }

    if (target.count == 1)
    {
        for (int j = 0; j < 6; j++)
        {
            stateTor0[j][0] = FT[j];
        }
    }

   // /* One-Order Filter
    for (int j = 0; j < 6; j++)
    {
        double CutFreq = 20;
        double intDT = 0.001;
        stateTor1[j][0] = stateTor0[j][0] + intDT * (FT[j]-stateTor0[j][0])*CutFreq;

    }




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
			step_pjs[i] = PqEnd[i];
		}


		PqEnd0[2] = -5;
        //PqEnd0[0] = 0.398;PqEnd0[1] = 0;PqEnd0[2] = 0.6295;PqEnd0[3] = 0;PqEnd0[4] = 0.7071;PqEnd0[5] = 0;PqEnd0[6] = 0.7071;

        aris::dynamic::s_pq2pm(PqEnd0, begin_pm);
		target.model->generalMotionPool()[0].getMpm(begin_pm);
	}	

	static bool flag[6] = { true,true,true,true,true,true };
    double PosLimit[6] = { 0.030,0.030,0,0,0,0};
    double NegLimit[6] = { -0.030,-0.030,0,0,0,0};
    static double pArc[6], vArc[6], aArc[6], vArcMax[6] = { 0.001,0.005,0.001,0.001,0.001,0.001 };
	static aris::Size t_count[6] = { 0 };
	static int CountOffsetPos[6] = { 1,1,1,1,1,1 }, CountOffsetNeg[6] = { 1,1,1,1,1,1 };
	int temp[6] = { 0 };
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


    for (int i = 0; i < 6; ++i)
    {
        PqEnd0[i] = step_pjs[i];
    }
    PqEnd0[2]=-5;



	double TransMatrix[4][4];
	for (int i = 0;i < 4;i++)
		for (int j = 0;j < 4;j++)
			TransMatrix[i][j] = begin_pm[4 * i + j];

	double n[3] = { TransMatrix[0][0], TransMatrix[0][1], TransMatrix[0][2] };
	double o[3] = { TransMatrix[1][0], TransMatrix[1][1], TransMatrix[1][2] };
	double a[3] = { TransMatrix[2][0], TransMatrix[2][1], TransMatrix[2][2] };
	


	if (target.model->solverPool().at(1).kinPos())return -1;

	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);
	fwd.cptJacobiWrtEE();
	double va[6] = { 0 }, dX[6] = { 0 };
	for (int i = 0;i < 6;i++)
	{
		va[i] = controller->motionAtAbs(i).actualVel();
	}
	s_mm(6, 1, 6, fwd.Jf(), va, dX);

	/*
	target.model->generalMotionPool()[0].getMpm(pmtemp);
	Pm[0] = pmtemp[0];Pm[1] = pmtemp[1];Pm[2] = pmtemp[2];
	Pm[3] = pmtemp[4];Pm[4] = pmtemp[5];Pm[5] = pmtemp[6];
	Pm[6] = pmtemp[8];Pm[7] = pmtemp[9];Pm[8] = pmtemp[10];
	double PmTrans[9] = { 0 };
	s_mm(3, 3, 3, Pm0, Pm, PmTrans);*/

    //PqEnd[0] = 0.398;PqEnd[1] = 0;PqEnd[2] = 0.6295;PqEnd[3] = 0;PqEnd[4] =0;PqEnd[5] = 0;PqEnd[6] = 1;

    aris::dynamic::s_pq2pm(PqEnd, end_pm);
	//target.model->generalMotionPool()[0].getMpm(end_pm);
	aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);
	// relative_pa //
	double pq[7];
	aris::dynamic::s_pm2pq(relative_pm, pq);


	double vt[6] = { 0 }, ftemp[6] = { 0 };
    double KPP[7] = {200,200,8,-15,-15,-15,0};
    double KPV[7] = {100,100,0,4,4,4,0};
    double KIV[7] = {50,50,  0,1,1,1,0};
    static double ErrSum[7]={0};
	for (int i = 0; i < 3; ++i)
	{
        ErrSum[i]=ErrSum[i]+(PqEnd0[i] - PqEnd[i])*0.001;
        vt[i] = KPP[i] * (PqEnd0[i]- PqEnd[i]);
	}


	/*
	//姿态误差1
	double omega = 2*acos(pq[6]);
	for (int i = 3; i < 6; ++i)
	{
		ErrSum[i] = ErrSum[i] + (pq[i])*0.001;
		ftemp[i] = KP[i] * (pq[i]) + KI[i] * ErrSum[i];
	}
	ft[3] = n[0] * ftemp[3] + n[1] * ftemp[4] + n[2] * ftemp[5];
	ft[4] = o[0] * ftemp[3] + o[1] * ftemp[4] + o[2] * ftemp[5];
	ft[5] = a[0] * ftemp[3] + a[1] * ftemp[4] + a[2] * ftemp[5];
	*/



    double dQuar[4] = { 0 };
	//姿态误差2
	double cos_theta = PqEnd[3] * PqEnd0[3] + PqEnd[4] * PqEnd0[4] + PqEnd[5] * PqEnd0[5] + PqEnd[6] * PqEnd0[6];

    auto &lout = controller->lout();


	if (cos_theta < 0)
	{
		PqEnd[3] = -PqEnd[3];
		PqEnd[4] = -PqEnd[4];
		PqEnd[5] = -PqEnd[5];
		PqEnd[6] = -PqEnd[6];
	}
	cos_theta = PqEnd[3] * PqEnd0[3] + PqEnd[4] * PqEnd0[4] + PqEnd[5] * PqEnd0[5] + PqEnd[6] * PqEnd0[6];
	
	cos_theta = std::max(-1.0, cos_theta);
	cos_theta = std::min(1.0, cos_theta);
	
	double theta = acos(cos_theta);
	double sin_theta = sin(theta);


    if (theta < 0.03)
	{
        dQuar[0] = -PqEnd0[3] + PqEnd[3];
        dQuar[1] = -PqEnd0[4] + PqEnd[4];
        dQuar[2] = -PqEnd0[5] + PqEnd[5];
        dQuar[3] = -PqEnd0[6] + PqEnd[6];
	}
	else
	{ 
        dQuar[0] = (PqEnd0[3] * cos_theta*(-theta) + theta * PqEnd[3]) / sin_theta;
        dQuar[1] = (PqEnd0[4] * cos_theta*(-theta) + theta * PqEnd[4]) / sin_theta;
        dQuar[2] = (PqEnd0[5] * cos_theta*(-theta) + theta * PqEnd[5]) / sin_theta;
        dQuar[3] = (PqEnd0[6] * cos_theta*(-theta) + theta * PqEnd[6]) / sin_theta;
	}
	
	for (int i = 3; i < 6; ++i)
	{
		ErrSum[i] = ErrSum[i] + (pq[i])*0.001;
        vt[i] = KPP[i] * (dQuar[i-3]);
	}

	static double ErrSumVt[7] = { 0 };
	double ft[6] = { 0 };
	for (int i = 0; i < 6; ++i)
	{
		ErrSumVt[i] = ErrSumVt[i] + (vt[i]-dX[i])*0.001;
		ft[i] = KPV[i] * (vt[i]-dX[i])+KIV[i]*ErrSumVt[i];
	}

    ErrSumVt[2] = ErrSumVt[2]+(PqEnd0[2] - FT[2])*0.001;
    ft[2]=KPP[2] * (PqEnd0[2] - FT[2])+KIV[2]*ErrSumVt[2];



	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };

    double f_static[6] = { 9,9,5,3,2,2 };
	double f_vel_JRC[6] = { 0,0,0,0,0,0 };

	double JoinTau[6] = { 0 };
	s_mm(6, 1, 6, fwd.Jf(), T(6), ft, 1, JoinTau, 1);

    if (target.count % 300 == 0)
    {
        cout<<ft[2]<<"****"<<FT[2]<<"****"<<step_pjs[1]<<"****"<<PqEnd0[1]<<"****"<<temp[1]<<std::endl;
    }

    lout << ft[2] << ",";lout << FT[2] << ",";lout<<step_pjs[1] << ",";lout << temp[1] << ",";

	double pa[6] = { 0 }, ta[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		pa[i] = controller->motionAtAbs(i).actualPos();
		va[i] = controller->motionAtAbs(i).actualVel();
        //ta[i] = controller->motionAtAbs(i).actualCur() / f2c_index[i];
	}
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


	for (int i = 0; i < 6; i++)
	{
        JoinTau[i] = JoinTau[i] + target.model->motionPool()[i].mfDyn() + f_vel_JRC[i] * va[i] + 0 * f_static[i] * signV(va[i]);
        JoinTau[i] = JoinTau[i] * f2c_index[i];
	}


    lout << PqEnd[0] << ",";lout << PqEnd[1] << ",";
    lout << PqEnd[2] << ",";lout << PqEnd[3] << ",";
    lout << PqEnd[4] << ",";lout << PqEnd[5] << ",";lout << PqEnd[6] << ",";


    lout << JoinTau[0] << ",";lout << JoinTau[1] << ",";
    lout << JoinTau[2] << ",";lout << JoinTau[3] << ",";
    lout << JoinTau[4] << ",";lout << JoinTau[5] << ",";lout << std::endl;

	for (int i = 0; i < 6; i++)
	{
        JoinTau[i] = std::max(-300.0, JoinTau[i]);
        JoinTau[i] = std::min(300.0, JoinTau[i]);

        controller->motionAtAbs(i).setTargetToq(JoinTau[i]);
	}



    for (int i = 0; i < 6; i++)
    {

        stateTor0[i][0] = stateTor1[i][0];
        stateTor0[i][1] = stateTor1[i][1];

    }


    return 35000 - target.count;

}

ForceDirect::ForceDirect(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
        "<Command name=\"ForceDir\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"-20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}





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
    //auto mat0 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*target.model->variablePool().findByName("estParasFT"));
    //for (int i = 0;i < GroupDim;i++)
        //sixDistalMatrix.estParasFT[i] = mat0->data().data()[i];

}
auto MoveJoint::executeRT(PlanTarget &target)->int
{
	auto &param = std::any_cast<MoveJointParam&>(target.param);

    static double step_pjs[6],begin_pjs[6];
    static double stateTor0[6][2], stateTor1[6][2];
    static double FT0[6];

	// 访问主站 //
	auto controller = target.controller;

	// 获取当前起始点位置 //
	if (target.count == 1)
	{
        for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
            begin_pjs[i] = target.model->motionPool()[i].mp();
            //controller->motionPool().at(i).setModeOfOperation(10);	//切换到电流控制
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
    double dThetaFil[6] = { 0 };



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

    sixDistalMatrix.sixDistalCollision(q, dq, ddq, FT, sixDistalMatrix.estParasFT, CollisionFT);
    for (int j = 0; j < 6; j++)
        FT[j]=FT[j]-CollisionFT[j];




    if (target.count == 1)
    {
        for (int j = 0; j < 6; j++)
        {
            stateTor0[j][0] = FT[j];
        }
    }


    /* One-Order Filter
	for (int j = 0; j < 6; j++)
	{
        double CutFreq = 5;//SHANGHAI DIANQI EXP


		double intDT = 0.001;
        stateTor1[j][0] = stateTor0[j][0] + intDT * (FT[j]-stateTor0[j][0])*CutFreq;

    }*/


    // /*Second-Order Filter
    for (int j = 0; j < 6; j++)
    {
        double A[2][2], B[2], CutFreq = 100;//SHANGHAI DIANQI EXP
        //CutFreq = 85;
        A[0][0] = 0; A[0][1] = 1;
        A[1][0] = -CutFreq*CutFreq; A[1][1] = -sqrt(2)*CutFreq;

        B[0] = 0; B[1] = CutFreq*CutFreq;

        double intDT = 0.001;
        stateTor1[j][0] = stateTor0[j][0] + intDT * (A[0][0] * stateTor0[j][0] + A[0][1] * stateTor0[j][1] + B[0] * FT[j]);
        stateTor1[j][1] = stateTor0[j][1] + intDT * (A[1][0] * stateTor0[j][0] + A[1][1] * stateTor0[j][1] + B[1] * FT[j]);

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



    for (int i = 0; i < 3; i++)
    {
        if (FT_KAI[i] < 0.1&&FT_KAI[i]>0)
            FT_KAI[i] = 10 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
        else if (FT_KAI[i]<0 && FT_KAI[i]>-0.1)
            FT_KAI[i] = -10 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
    }


    for (int i = 3; i < 6; i++)
    {
        if (FT_KAI[i] < 0.05&&FT_KAI[i]>0)
            FT_KAI[i] = 20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
        else if (FT_KAI[i]<0 && FT_KAI[i]>-0.05)
            FT_KAI[i] = -20 * FT_KAI[i] * FT_KAI[i];//In KAI Coordinate
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


	///* Using Jacobian, TransMatrix from ARIS
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(target.model->solverPool()[1]);

    fwd.cptJacobiWrtEE();
    //FmInWorld[2] = 0; FmInWorld[3] = 0; FmInWorld[4] = 0; FmInWorld[5] = 0;
	double JoinTau[6] = { 0 };
    s_mm(6, 1, 6,fwd.Jf() , T(6), FmInWorld, 1, JoinTau, 1);

	


	double pa[6] = { 0 }, va[6] = { 0 }, ta[6] = { 0 };
	double ft_offset[6] = { 0 };
	double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
    double f_static[6] = { 9,9,5,3,2,2 };
    double f_vel_JRC[6] = { 10,10,10,10,10,10 };
	double ExternTau[6] = { 0 };

	for (int i = 0; i < 6; i++)
	{
		pa[i] = controller->motionAtAbs(i).actualPos();
		va[i] = controller->motionAtAbs(i).actualVel();
		ta[i] = controller->motionAtAbs(i).actualCur()/ f2c_index[i];
	}
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


	for (int i = 0; i < 6; i++)
	{
        ExternTau[i] = ta[i] -JoinTau[i]- target.model->motionPool()[i].mfDyn() - f_vel_JRC[i] * va[i] - 1 * f_static[i] * signV(va[i]);
        ExternTau[i] = -ExternTau[i];
    }


    double rate=6.0;
    dTheta[0] = JoinTau[0] / 500/rate+ ExternTau[0] / 1000/rate;
    dTheta[1] = JoinTau[1] / 500/rate+ ExternTau[1] / 1000/rate;
    dTheta[2] = JoinTau[2] / 500/rate+ ExternTau[2] / 1000/rate;
    dTheta[3] = JoinTau[3] / 100/rate+ ExternTau[3] / 1000/rate;
    dTheta[4] = JoinTau[4] / 500/rate+ ExternTau[4] / 1000/rate;
    dTheta[5] = JoinTau[5] / 100/rate+ ExternTau[5] / 1000/rate;

	for (int i = 0; i < 6; i++)
	{
        if (dTheta[i] > 0.0006)
            dTheta[i] = 0.0006;
        if (dTheta[i] < -0.0006)
            dTheta[i] = -0.0006;
		//lout << dTheta[i] << ",";
	}



     static double StateDtheta0[6][2], StateDtheta1[6][2];

     if (target.count == 1)
     {
         for (int j = 0; j < 6; j++)
         {
             stateTor0[j][0] = dTheta[j];
         }
     }
     // /*Second-Order Filter
     for (int j = 0; j < 6; j++)
     {
         double A[2][2], B[2], CutFreq = 20;//SHANGHAI DIANQI EXP
         //CutFreq = 85;
         A[0][0] = 0; A[0][1] = 1;
         A[1][0] = -CutFreq*CutFreq; A[1][1] = -sqrt(2)*CutFreq;

         B[0] = 0; B[1] = CutFreq*CutFreq;

         double intDT = 0.001;
         StateDtheta1[j][0] = StateDtheta0[j][0] + intDT * (A[0][0] * StateDtheta0[j][0] + A[0][1] * StateDtheta0[j][1] + B[0] * dTheta[j]);
         StateDtheta1[j][1] = StateDtheta0[j][1] + intDT * (A[1][0] * StateDtheta0[j][0] + A[1][1] * StateDtheta0[j][1] + B[1] * dTheta[j]);

     }







	for (int i = 0; i < 6; i++)
	{
        dThetaFil[i] = StateDtheta1[i][0] * DirectionFlag[i];

	}



	for (int i = 0; i < 6; i++)
	{
        step_pjs[i] = step_pjs[i] + dTheta[i];
       // target.model->motionPool().at(i).setMp(step_pjs[i]);
	}




    double KP[6]={8,8,8,1,8,1};

   



    for (int i = 0; i < 6; i++)
    {
    pa[i] = controller->motionAtAbs(i).actualPos();
    va[i] = controller->motionAtAbs(i).actualVel();
    //ta[i] = controller->motionAtAbs(i).actualTor();
    ft_offset[i]=(10*KP[i]*(step_pjs[i]-pa[i])+target.model->motionPool()[i].mfDyn()+0*f_vel_JRC[i]*va[i] + 0*f_static[i]*signV(va[i]))*f2c_index[i];
    ft_offset[i] = std::max(-500.0, ft_offset[i]);
    ft_offset[i] = std::min(500.0, ft_offset[i]);
    //if(abs(pa[i])<1)

        //controller->motionAtAbs(i).setTargetCur(ft_offset[i]);
    }

    lout << FTemp[0] << ",";lout << FTemp[1] << ",";
    lout << FTemp[2] << ",";lout << FTemp[3] << ",";
    lout << FTemp[4] << ",";lout << FTemp[5] << ",";
/*
    lout << FT_YANG[0] << ",";lout << FT_YANG[1] << ",";
    lout << FT_YANG[2] << ",";lout << FT_YANG[3] << ",";
    lout << FT_YANG[4] << ",";lout << FT_YANG[5] << ",";

    lout << dTheta[0] << ",";lout << dTheta[1] << ",";
    lout << dTheta[2] << ",";lout << dTheta[3] << ",";
    lout << dTheta[4] << ",";lout << dTheta[5] << ",";

    lout << step_pjs[0] << ",";lout << step_pjs[1] << ",";
    lout << step_pjs[2] << ",";lout << step_pjs[3] << ",";
    lout << step_pjs[4] << ",";lout << step_pjs[5] << ",";

    lout << va[0] << ",";lout << va[1] << ",";
    lout << va[2] << ",";lout << va[3] << ",";
    lout << va[4] << ",";lout << va[5] << ",";*/
    lout << std::endl;


    if (target.count % 300 == 0)
    {

        //cout << step_pjs[2] << "***" << ft_offset[2] << "***" << step_pjs[2] << endl;

        //cout <<FT_KAI[0]<<"***"<<FT_KAI[1]<<"***"<<FT_KAI[2]<<endl;
        cout <<ta[0]<<"***"<<ExternTau[0]<<"***"<<JoinTau[0]<<"***"<<FmInWorld[1]<<"***"<<va[0]<<std::endl;
    }



	for (int i = 0; i < 6; i++)
	{

        stateTor0[i][0] = stateTor1[i][0];
        stateTor0[i][1] = stateTor1[i][1];

        StateDtheta0[i][0] = StateDtheta1[i][0];
        StateDtheta0[i][1] = StateDtheta1[i][1];

	}


	return 150000000 - target.count;

}

MoveJoint::MoveJoint(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"mvJoint\">"
		"	<GroupParam>"
		"       <Param name=\"PressF\" default=\"0\"/>"
        "		<Param name=\"SensorType\"default=\"20.0\"/>"
		"   </GroupParam>"
		"</Command>");

}
