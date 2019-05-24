
#include <math.h>
#include <algorithm>

#include"move_series.h"

using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;


void crossVector1(const double* a, const double* b, double* c)
{

	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = -(a[0] * b[2] - b[0] * a[2]);
	c[2] = a[0] * b[1] - b[0] * a[1];

}


struct MoveSeriesParam
{
	std::vector<double> t, x, xp1, xp2, xp3, y, yp1, yp2, yp3;
};
auto MoveSeries::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	MoveSeriesParam param;

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

		param.t.data()[i + 3] = param.t.data()[i + 2] + std::max(std::sqrt(diff_x * diff_x + diff_y * diff_y), 1e-7) * scale;
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

	aris::dynamic::s_akima(param.t.size(), param.t.data(), param.x.data(), param.xp1.data(), param.xp2.data(), param.xp3.data(), 1e-7);
	aris::dynamic::s_akima(param.t.size(), param.t.data(), param.y.data(), param.yp1.data(), param.yp2.data(), param.yp3.data(), 1e-7);

	target.param = param;

    for(auto &option:target.mot_options) option|=
      Plan::USE_TARGET_POS |
#ifdef WIN32
      Plan::NOT_CHECK_POS_MIN |
      Plan::NOT_CHECK_POS_MAX |
      Plan::NOT_CHECK_POS_CONTINUOUS |
      Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
      Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
      Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
      Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
      Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
      Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
      Plan::NOT_CHECK_VEL_MIN |
      Plan::NOT_CHECK_VEL_MAX |
      Plan::NOT_CHECK_VEL_CONTINUOUS |
      Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
      Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;



}
	auto MoveSeries::executeRT(PlanTarget &target)->int 
	{
		auto &param = std::any_cast<MoveSeriesParam&>(target.param);


		auto x = aris::dynamic::s_akima_at(param.t.size(), param.t.data(), param.x.data(), param.xp1.data(), param.xp2.data(), param.xp3.data(), target.count / 1000.0);
		auto y = aris::dynamic::s_akima_at(param.t.size(), param.t.data(), param.y.data(), param.yp1.data(), param.yp2.data(), param.yp3.data(), target.count / 1000.0);

		std::cout << x << "  " << y << std::endl;






		static double X0[6] = { 0 }, X1[6] = { 0 };
		static double step_pjs[6];
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


		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();

		// log 电流 //
		auto &lout = controller->lout();


	



		///* Using Jacobian, TransMatrix from ARIS
		double EndW[3], EndP[3], BaseV[3];
		double PqEnd[7], TransVector[16];
		target.model->generalMotionPool().at(0).getMpm(TransVector);
		target.model->generalMotionPool().at(0).getMpq(PqEnd);

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
            lout << dTheta[i] << ",";
		}

        lout << x << ","<<y<<",";
       lout << endl;

		for (int i = 0; i < 6; i++)
		{
			step_pjs[i] = step_pjs[i] + dTheta[i];
            //target.model->motionPool().at(i).setMp(step_pjs[i]);
		}

		for (int i = 0; i < 6; ++i)
		{
			X0[i] = X1[i];
		}

		return target.count / 1000.0 > *(param.t.end() - 4) ? 0 : 1;

	}
	MoveSeries::~MoveSeries() = default;
	MoveSeries::MoveSeries(const std::string &name) :Plan(name)
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
