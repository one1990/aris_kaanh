#include "kinematics.h"
#include <math.h>
#include"kaanh.h"
#include <algorithm>
#include"robotconfig.h"
#include <vector>
using namespace std;
using namespace aris::plan;
using namespace aris::dynamic;
using namespace CONFIG;
/// \brief

int RecorToolPoints = 0;
double TransMatrix[6][16] = { 0 };

void crossVectorKine(const double* a, const double* b, double* c)
{

	c[0] = a[1] * b[2] - b[1] * a[2];
	c[1] = -(a[0] * b[2] - b[0] * a[2]);
	c[2] = a[0] * b[1] - b[0] * a[1];

}




struct FourPointsParam
{
	int Number;
};

auto FourPoints::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	FourPointsParam param;
	for (auto &p : params)
	{
		if (p.first == "Number")
			param.Number = std::stod(p.second);

	}

	double TransVector[16] = { 0 };
	target.model->generalMotionPool().at(0).getMpm(TransVector);

	if (RecorToolPoints < 6)
	{
		for (int i = 0;i < 16;i++)
			TransMatrix[RecorToolPoints][i] = TransVector[i];

		cout << RecorToolPoints;
		RecorToolPoints++;
	}

	target.option |= NOT_RUN_COLLECT_FUNCTION;
	target.option |= NOT_RUN_EXECUTE_FUNCTION;
}

FourPoints::FourPoints(const std::string &name) :Plan(name)
{
command().loadXmlStr(
		"<Command name=\"FourPoints\">"
		"	<GroupParam>"
		"       <Param name=\"Number\" default=\"6\"/>"
		"   </GroupParam>"
		"</Command>");
}







struct SetToolParam
{
	double period;
	double amplitude;

};
auto SetTool::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
	SetToolParam param;

	for (auto &p : params)
	{
		if (p.first == "period")
			param.period = std::stod(p.second);
		if (p.first == "amplitude")
			param.amplitude = std::stod(p.second);

	}

	if (RecorToolPoints == 6)
	{

		double TransMat[6][12] = { { -0.6492,0.2770,0.7084,-0.2863,0.7738,-0.5650,-0.7046,-0.5697,-0.4231,1028.29,233.29,1432.98},
		{-0.8469, 0.1182, 0.5184, 0.4204, 0.7458, 0.5168, -0.3256, 0.6556, -0.6813,1118.69,-231.99,1478.85},
		{-0.8680, -0.0425, 0.4947, -0.0744, 0.9962, -0.0451, -0.4909, -0.0759, -0.8679,1129.45,25,1564},
		{-0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835,983.98,0.39,1502.31},
		{ -0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835,605.47,0.39,1502.31},
		{ -0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835,983.98,0.39,1122.24} };

		for (int i = 0;i < 6;i++)
			for (int j = 0;j < 12;j++)
				TransMat[i][j] = TransMatrix[i][j];


		double Atemp[5][9], Btemp[5][3];

		//target.model->generalMotionPool().at(0).getMpm(TransVector);
		for (int i = 0;i < 5;i++)
			for (int j = 0;j < 9;j++)
				Atemp[i][j] = TransMat[i][j] - TransMat[i + 1][j];

		double L[3][3] = { 0 };
		for (int i = 0;i < 5;i++)
		{
			for (int m = 0;m < 3;m++)
				for (int n = 0;n < 3;n++)
					for (int j = 0;j < 3;j++)
						L[m][n] = L[m][n] + Atemp[i][j + 3 * m] * Atemp[i][j * 3 + n];
		}

		for (int i = 0;i < 5;i++)
			for (int j = 0;j < 3;j++)
				Btemp[i][j] = TransMat[i + 1][j + 9] - TransMat[i][j + 9];

		double R[3] = { 0 };
		for (int i = 0;i < 5;i++)
		{
			for (int m = 0;m < 3;m++)
				for (int j = 0;j < 3;j++)
					R[m] = R[m] + Atemp[i][j + 3 * m] * Btemp[i][j];
		}

		// Pinv(L)*R,求取位置偏移Epos
		double Lvec[9];
		for (int i = 0;i < 3;i++)
			for (int j = 0;j < 3;j++)
				Lvec[i * 3 + j] = L[i][j];

		double U[9], tau[3], pinv[9], Epos[3];
		aris::Size p[3];
		aris::Size rank;

		// 根据 A 求出中间变量，相当于做 QR 分解 //
		// 请对 U 的对角线元素做处理
		s_householder_utp(3, 3, Lvec, U, tau, p, rank, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
		double tau2[3];
		s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
		s_mm(3, 1, 3, pinv, R, Epos);

		// 求取姿态偏移矩阵Epos
		double Trans4[9] = { -0.5835, 0.0035, 0.8121, 0.0020, 1.0000, -0.0028, -0.8121, 0.0000, -0.5835 };
		double Dis4[3] = { 983.98,0.39,1502.31 };
		double Dis5[3] = { 605.47,0.39,1502.31 };
		double Dis6[3] = { 983.98,0.39,1122.24 };
		double x1[3], x2[3], x12[3];
		s_mm(3, 1, 3, Trans4, Epos, x1);
		for (int i = 0;i < 3;i++)
			x1[i] = x1[i] + Dis4[i];
		s_mm(3, 1, 3, Trans4, Epos, x2);
		for (int i = 0;i < 3;i++)
			x2[i] = x2[i] + Dis5[i];

		//计算X方向的方向余弦
		double norm_x12 = 0, n_TE[3];
		for (int i = 0;i < 3;i++)
		{
			x12[i] = x2[i] - x1[i];
			norm_x12 = norm_x12 + x12[i] * x12[i];
		}
		norm_x12 = sqrt(norm_x12);
		for (int i = 0;i < 3;i++)
		{
			x12[i] = x12[i] / norm_x12;
		}
		// 根据 A 求出中间变量，相当于做 QR 分解 //
		s_householder_utp(3, 3, Trans4, U, tau, p, rank, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
		s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
		s_mm(3, 1, 3, pinv, x12, n_TE);

		//计算Z方向的方向余弦
		s_mm(3, 1, 3, Trans4, Epos, x2);
		for (int i = 0;i < 3;i++)
			x2[i] = x2[i] + Dis6[i];
		double a_TE[3];
		for (int i = 0;i < 3;i++)
		{
			x12[i] = x2[i] - x1[i];
			norm_x12 = norm_x12 + x12[i] * x12[i];
		}
		norm_x12 = sqrt(norm_x12);
		for (int i = 0;i < 3;i++)
		{
			x12[i] = x12[i] / norm_x12;
		}
		// 根据 A 求出中间变量，相当于做 QR 分解 //
		s_householder_utp(3, 3, Trans4, U, tau, p, rank, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A) //
		s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);

		// 根据QR分解的结果求广义逆，相当于Matlab中的 pinv(A)*b //
		s_mm(3, 1, 3, pinv, x12, a_TE);

		//计算Y方向的方向余弦
		double o_TE[3];
		crossVectorKine(a_TE, n_TE, o_TE);
	}



}

SetTool::SetTool(const std::string &name) :Plan(name)
{

	command().loadXmlStr(
		"<Command name=\"STool\">"
		"	<GroupParam>"
		"		<Param name=\"period\"default=\"1.0\"/>"
		"		<Param name=\"amplitude\"default=\"0.2\"/>"
		"	</GroupParam>"
		"</Command>");


}

