
#include <math.h>
#include <algorithm>

#include"move_series.h"

//using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;


void crossVector1(const double* a, const double* b, double* c)
{

	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = -(a[0] * b[2] - b[0] * a[2]);
	c[2] = a[0] * b[1] - b[0] * a[1];

}


struct MoveSeriesGKParam
{
	std::vector<double> t, x, xp1, xp2, xp3, y, yp1, yp2, yp3;
};
auto MoveSeriesGK::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveSeriesGKParam param;

	auto x_mat = target.model->calculator().calculateExpression(params.at("x"));
	auto y_mat = target.model->calculator().calculateExpression(params.at("y"));
	if (x_mat.size() != y_mat.size())throw std::runtime_error("x and y size not correct");

	param.t.resize(x_mat.size() + 6);
	param.t[0] = -3;
	param.t[1] = -2;
	param.t[2] = -1;
	param.t[3] = 0;

	param.x.resize(x_mat.size() + 6);
	std::copy(x_mat.begin(), x_mat.end(), param.x.begin() + 3);
	param.x[0] = param.x[3];
	param.x[1] = param.x[3];
	param.x[2] = param.x[3];
	*(param.x.end() - 1) = *(param.x.end() - 4);
	*(param.x.end() - 2) = *(param.x.end() - 4);
	*(param.x.end() - 3) = *(param.x.end() - 4);

	param.y.resize(y_mat.size() + 6);
	std::copy(y_mat.begin(), y_mat.end(), param.y.begin() + 3);
	param.y[0] = param.y[3];
	param.y[1] = param.y[3];
	param.y[2] = param.y[3];
	*(param.y.end() - 1) = *(param.y.end() - 4);
	*(param.y.end() - 2) = *(param.y.end() - 4);
	*(param.y.end() - 3) = *(param.y.end() - 4);


	auto scale = std::stod(params.at("scale"));

	for (int i = 1; i < x_mat.size(); ++i)
	{
		auto diff_x = x_mat.data()[i] - x_mat.data()[i - 1];
		auto diff_y = y_mat.data()[i] - y_mat.data()[i - 1];

		param.t.data()[i + 3] = param.t.data()[i + 2] + std::max(std::sqrt(diff_x * diff_x + diff_y * diff_y), 1e-10) * scale;
	}

	*(param.t.end() - 3) = *(param.t.end() - 4) + 1;
	*(param.t.end() - 2) = *(param.t.end() - 4) + 2;
	*(param.t.end() - 1) = *(param.t.end() - 4) + 3;

	param.xp1.resize(x_mat.size() + 6);
	param.xp2.resize(x_mat.size() + 6);
	param.xp3.resize(x_mat.size() + 6);
	param.yp1.resize(x_mat.size() + 6);
	param.yp2.resize(x_mat.size() + 6);
	param.yp3.resize(x_mat.size() + 6);

	aris::dynamic::s_akima(param.t.size(), param.t.data(), param.x.data(), param.xp1.data(), param.xp2.data(), param.xp3.data(), 1e-10);
	aris::dynamic::s_akima(param.t.size(), param.t.data(), param.y.data(), param.yp1.data(), param.yp2.data(), param.yp3.data(), 1e-10);

	target.param = param;

	for (auto &option : target.mot_options) option |=
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
		Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
		Plan::NOT_CHECK_ENABLE;



}
auto MoveSeriesGK::executeRT(PlanTarget &target)->int
{
	static double stateTor0[6][3], stateTor1[6][3];
	static float FT0[6];
    auto &param = std::any_cast<MoveSeriesGKParam&>(target.param);
	auto controller = target.controller;
	auto &cout = controller->mout();

	auto x = aris::dynamic::s_akima_at(param.t.size(), param.t.data(), param.x.data(), param.xp1.data(), param.xp2.data(), param.xp3.data(), target.count / 1000.0);
	auto y = aris::dynamic::s_akima_at(param.t.size(), param.t.data(), param.y.data(), param.yp1.data(), param.yp2.data(), param.yp3.data(), target.count / 1000.0);

	//cout << x << "  " << y << endl;




	///* Using Jacobian, TransMatrix from ARIS
	double EndW[3], EndP[3], BaseV[3];
	double PqEnd[7], TransVector[16];
	target.model->generalMotionPool().at(0).getMpm(TransVector);
	target.model->generalMotionPool().at(0).getMpq(PqEnd);

	static double X0[6] = { 0 }, X1[6] = { 0 };
	static double step_pjs[6], sumDx[6] = { 0 };
	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			step_pjs[i] = target.model->motionPool()[i].mp();
		}
		X0[0] = x;X0[1] = y;
		X1[0] = x;X1[1] = y;

	}

	X1[0] = x;X1[1] = y;

	if (target.model->solverPool().at(1).kinPos())return -1;

	double dX[6] = { 0.00001, -0.0000, -0.0000, -0.0000, -0.0000, -0.0000 };
	double dTheta[6] = { 0 };

	for (int i = 0; i < 6; ++i)
	{
		dX[i] = X1[i] - X0[i];
	}


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00025)
			dX[j] = 0.00025;
		if (dX[j] < -0.00025)
			dX[j] = -0.00025;
	}

	dX[2] = 0;dX[3] = 0;dX[4] = 0;dX[5] = 0;


	int16_t FTint[6];
	double FTReal[6], FT[6];
	auto conSensor = dynamic_cast<aris::control::EthercatController*>(target.controller);

	conSensor->slavePool().at(7).readPdo(0x6000, 0x11, &FTint[0], 16);
	conSensor->slavePool().at(7).readPdo(0x6010, 0x11, &FTint[1], 16);
	conSensor->slavePool().at(7).readPdo(0x6020, 0x11, &FTint[2], 16);
	conSensor->slavePool().at(7).readPdo(0x6030, 0x11, &FTint[3], 16);
	conSensor->slavePool().at(8).readPdo(0x6000, 0x11, &FTint[4], 16);
	conSensor->slavePool().at(8).readPdo(0x6010, 0x11, &FTint[5], 16);

	for (int i = 0;i < 6;i++)
	{
		FTReal[i] = FTint[i] * 20.0 / 65536.0*1000.0;
	}

	double Vol2FTCoef[36] = { 0.000133, 	-0.029834, 	0.000031, 	0.029925, 	-0.000006, 	-0.000034,
						   0.000046, 	-0.017260, 	0.000137, 	-0.017237, 	0.000217, 	0.034588,
						   -0.045122, 	0.000344, 	-0.045692, 	0.000084, 	-0.047397, 	-0.000011,
						   -0.001332, 	0.000020, 	0.001212, 	0.000010, 	-0.000098, 	-0.000024,
						   -0.000712, 	-0.000020, 	-0.000783, 	0.000034, 	0.001539, 	-0.000004,
						   -0.000004, 	0.001104, 	-0.000001, 	0.001079, 	0.000007, 	0.001099 };
	s_mm(6, 1, 6, Vol2FTCoef, FTReal, FT);



	// 获取当前起始点位置 //
	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			stateTor0[j][0] = FT[j];
			FT0[j] = FT[j];
		}
	}


	for (int j = 0; j < 6; j++)
	{
		double A[3][3], B[3], CutFreq = 23;
		if (target.count > 23000)
			CutFreq = 53;
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



	if (target.count == 1)
	{
		for (int j = 0; j < 6; j++)
		{
			FT0[j] = stateTor1[j][0];
		}
	}

	if (target.count == 500)
	{
		for (int j = 0; j < 6; j++)
		{
			FT0[j] = stateTor1[j][0];
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



	double Ftotal = 1.0;

	if (sumDx[0] > 0.014&&sumDx[1] < -0.006)
		Ftotal = 0.3;

	if (sumDx[0] > 0.014&&sumDx[1] > 0 - 0.001)
		Ftotal = 0.3;

	if (sumDx[0] < 0.0016&&sumDx[1] < 0 - 0.0047)
		Ftotal = 0.3;

	if (sumDx[1] < 0 - 0.01)
		Ftotal = 0.3;
	//Ftotal=sqrt(1.2*FmInWorld[0]*FmInWorld[0]+1.2*FmInWorld[1]*FmInWorld[1]+FmInWorld[2]*FmInWorld[2]);
	dX[2] = 1 * (FmInWorld[2] - (Ftotal)) / 626000;


	if (dX[2] > 0)
		dX[2] *= 1.5;

	if (dX[2] < 0)
		dX[2] *= 0.5;
	//dX[2] = std::min(dX[2], -0.5);

	dX[3] = 0;dX[4] = 0;dX[5] = 0;

	for (int j = 0; j < 6; j++)
	{
		sumDx[j] = sumDx[j] + dX[j];
	}


	if (target.count % 300 == 0)
		cout << FmInWorld[2] << "***" << Ftotal << std::endl;
	// log 电流 //
	auto &lout = controller->lout();


	for (int j = 0; j < 6; j++)
	{
		if (dX[j] > 0.00015)
			dX[j] = 0.00015;
		if (dX[j] < -0.00006)
			dX[j] = -0.00006;
	}


	lout << dX[0] << "," << dX[1] << ",";
	lout << std::endl;




	for (int i = 0;i < 3;i++)
		EndW[i] = dX[i + 3];

	for (int i = 0;i < 3;i++)
		EndP[i] = PqEnd[i];
	crossVector1(EndP, EndW, BaseV);

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



	for (int i = 0; i < 6; i++)
	{
		step_pjs[i] = step_pjs[i] + dTheta[i];
		target.model->motionPool().at(i).setMp(step_pjs[i]);
	}

	for (int i = 0; i < 6; ++i)
	{
		X0[i] = X1[i];
	}


	for (int i = 0; i < 6; i++)
	{

		stateTor0[i][0] = stateTor1[i][0];
		stateTor0[i][1] = stateTor1[i][1];
		stateTor0[i][2] = stateTor1[i][2];
	}






	//return target.count / 1000.0 > (*(param.t.end() - 4) + 1000) ? 0 : 1;

	return 10000000 - target.count;

}
MoveSeriesGK::~MoveSeriesGK() = default;
MoveSeriesGK::MoveSeriesGK(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"mvs\">"
		"	<GroupParam>"
		"	    <Param name=\"x\" default=\"0\" abbreviation=\"x\" />"
		"	    <Param name=\"y\" default=\"0\" abbreviation=\"y\" />"
		"	    <Param name=\"scale\" default=\"10\" />"
		"	</GroupParam>"
		"</Command>");
}
