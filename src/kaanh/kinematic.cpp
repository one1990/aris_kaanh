#include "kaanh/kinematic.h"
#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>


using namespace aris::plan;
using namespace aris::dynamic;

/*
采用4点标定工具坐标系的坐标原点TCP，方法是沿4个不同方向将TCP与一个固定点接触，根据对应的法兰坐标系的位姿计算TCP
采用5点标定工具坐标系的坐标原点TCP和和TCF的Z轴方向
采用6点标定工具坐标系的坐标原点TCP和和TCF的Z轴、Y轴方向
*/

//向量叉乘
auto crossVector(double *a, double *s, double *n)->int
{
	//n=a×s
	n[0] = a[1] * s[2] - s[1] * a[2];
	n[1] = s[0] * a[2] - a[0] * s[2];
	n[2] = a[0] * s[1] - s[0] * a[1];
	return 0;
}

//将位置和欧拉角组成的位姿的单位由mm和°转换为m和rad
auto mmdeg2mrad(std::string datastr, double *data, size_t data_size)->int
{
	std::string posestr = datastr.substr(datastr.find("{") + 1, datastr.find("}") - datastr.find("{") - 1);
	std::string::size_type pos1 = 0;
	std::string::size_type pos2 = posestr.find(",");
	std::vector<std::string> tempvec;
	while (pos2 != std::string::npos)
	{
		tempvec.push_back(posestr.substr(pos1, pos2 - pos1));
		pos1 = pos2 + 1;
		pos2 = posestr.find(",", pos1);
	}
	tempvec.push_back(posestr.substr(pos1, posestr.length() - 1));
	if (tempvec.size() == data_size)
	{
		for (size_t i = 0; i < data_size; i++)
		{
			size_t j = i % 6;
			if (j<3)
			{ 
				data[i] = std::stod(tempvec.at(i))/1000; 
			}
			else
			{
				data[i] = std::stod(tempvec.at(i)) * PI / 180;
			}
		}
		return 0;
	}
	else
	{
		throw std::runtime_error("The input data of teaching point's pose is wrong！");		//"输入的示教点位姿数据有误！"
		return -1;
	}
}

//获取示教点位姿数据
auto get_teachpt_data(std::string datastr, double *data, size_t data_size)->int
{
	std::string posestr = datastr.substr(datastr.find("{") + 1, datastr.find("}") - datastr.find("{") - 1);
	std::string::size_type pos1 = 0;
	std::string::size_type pos2 = posestr.find(",");
	std::vector<std::string> tempvec;
	while (pos2 != std::string::npos)
	{
		tempvec.push_back(posestr.substr(pos1, pos2 - pos1));
		pos1 = pos2 + 1;
		pos2 = posestr.find(",", pos1);
	}
	tempvec.push_back(posestr.substr(pos1, posestr.length() - 1));
	if (tempvec.size() == data_size)
	{
		for (size_t i = 0; i < data_size; i++)
		{
			data[i] = std::stod(tempvec.at(i));
		}
		return 0;
	}
	else
	{
		throw std::runtime_error("The input data of teaching point's pose is wrong！");		//"输入的示教点位姿数据有误！"
		return -1;
	}
}

//4点标定法
struct CalibT4PParam
{
	double pe_4pt[24];
	std::vector<double> tool_pe;
	std::string calib_info;
	
};
auto CalibT4P::prepairNrt()->void
{
	//参数初始化
	CalibT4PParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			std::string tempstr=std::string(p.second);
			int ret1 = get_teachpt_data(tempstr, param.pe_4pt, 24);
			if (ret1 != 0) return;
		}
	}
	this->param() = param;
	double pm_4pt[64];
	double tcp[3];		//计算获得的tcp
	double tcp_error = 0;		//计算tcp的误差
	//double tcf[9];		//计算获得的tcf
	for (int i = 0; i < 4; i++)
	{
		double temp_pe[6];
		double temp_pm[16];
		for (int j = 0; j < 6; j++)
		{
			temp_pe[j] = param.pe_4pt[6 * i + j];
		}
		s_pe2pm(temp_pe, temp_pm, "321");
		for (int k = 0; k < 16; k++)
		{
			pm_4pt[16 * i + k] = temp_pm[k];
		}
	}
	int ret2 = cal_TCP(pm_4pt, tcp, tcp_error);
	if (ret2 == 0)
	{
		//将标定结果转换为欧拉角形式
		//const double pose[6] = { tcp[0] *1000, tcp[1] * 1000, tcp[2] * 1000, 0, 0, 0 };
		const double pose[6] = { tcp[0], tcp[1], tcp[2], 0, 0, 0 };
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe.push_back(pose[i]);
		}
		//const std::string calib_info = "工具坐标系4点标定完成！工具中心点（TCP）的拟合误差是：" + std::to_string(tcp_error * 1000) + "mm";
		const std::string calib_info = "The calculation is done! The calibration error of TCP is:" + std::to_string(tcp_error * 1000) + "mm";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("The calculation process was aborted!");		//"无法计算标定结果，获取的示教点异常，请重新执行标定过程。"
		param.calib_info = std::string("The calculation process was aborted!").c_str();
		//return;
	}
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("tool_pe", param.tool_pe));
	ret() = out_param;

	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibT4P::CalibT4P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_t4p\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}\"/>"
		"   </GroupParam>"
		"</Command>");
}
auto CalibT4P::cal_TCP(double transmatric[64], double tcp[3], double &tcp_error)->int
{
	//计算R(i)-R(i+1),P(i+1)-P(i)
	double R[36], P[12];
	tm2RP_4Pt(transmatric, R, P);
	/*for (int i = 0; i < 12; i++)
	{
		std::cout << P[i] << ",";
	}*/
	double deltaR[27], deltaP[9];
	deltaRP_4Pt(R, P, deltaR, deltaP);

	//计算tcp和tcp_error
	double U[27] = { 0 };
	double tau[9] = { 0 };
	aris::Size p[9];
	aris::Size rank;
	s_householder_utp(9, 3, deltaR, U, tau, p, rank, 1e-10);
	////判断是否列满秩
	if (rank < 3)
	{
		//throw std::runtime_error("示教点数据异常，请重新获取示教点！");
		return -1;
	}
	double tau2[9], pinv[27];
	s_householder_utp2pinv(9, 3, rank, U, tau, p, pinv, tau2, 1e-10);
	s_mm(3, 1, 9, pinv, deltaP, tcp);
	//计算tcp_error
	double new_deltaP[9], error_vec[9];
	s_mm(9, 1, 3, deltaR, tcp, new_deltaP);
	/*for (int i = 0; i < 9; i++)
	{
		std::cout << new_deltaP[i] << ","<<std::endl;
	}*/
	for (int i = 0; i < 9; i++)
	{
		error_vec[i] = new_deltaP[i] - deltaP[i];
		//std::cout << error_vec[i] << "," << std::endl;
	}
	double prod_error[3] = { 0.0, 0.0, 0.0 };
	double sum_prod_error = 0.0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			prod_error[i] = prod_error[i] + error_vec[i * 3 + j] * error_vec[i * 3 + j];
		}
		sum_prod_error = sum_prod_error + prod_error[i];
	}
	tcp_error = std::sqrt(sum_prod_error);
	//std::cout << "误差" << tcp_error;
	return 0;
}
auto CalibT4P::tm2RP_4Pt(double tm[64], double *R, double *P)->int
{
	//将n_pt*16坐标变换矩阵中提取n_pt*9旋转矩阵和n_pt*3平移向量
	int num = 64;
	int n_pt = 4;
	double temp[4][16] = { 0 };
	for (int i = 0; i < num; i++)
	{
		int row = (int)floor(i / 16);
		int col = i % 16;
		for (int j = 0; j < 16; j++)
		{
			temp[row][col] = tm[i];
		}
	}
	for (int i = 0; i < n_pt; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i * 9 + j] = temp[i][j];
			P[i * 3 + j] = temp[i][j * 4 + 3];
		}
		for (int j = 3; j < 6; j++)
		{
			R[i * 9 + j] = temp[i][j + 1];
		}
		for (int j = 6; j < 9; j++)
		{
			R[i * 9 + j] = temp[i][j + 2];
		}
	}
	return 0;
}
auto CalibT4P::deltaRP_4Pt(double R[36], double P[12], double * deltaR, double * deltaP)->int
{
	//计算R(i)-R(i+1),P(i+1)-P(i),i=示教点总数n_pt减1
	//deltaR[27],deltaP[9]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			deltaR[i * 9 + j] = R[i * 9 + j] - R[(i + 1) * 9 + j];
		}
		for (int j = 0; j < 3; j++)
		{
			deltaP[3 * i + j] = P[(i + 1) * 3 + j] - P[i * 3 + j];
		}
	}
	return 0;
}


//5点标定法
struct CalibT5PParam
{
	double pe_5pt[30];
	std::vector<double> tool_pe;
	std::string calib_info;
};
auto CalibT5P::prepairNrt()->void
{
	//参数初始化
	CalibT5PParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			std::string tempstr = std::string(p.second);
			int ret1 = get_teachpt_data(tempstr, param.pe_5pt, 30);
			if (ret1 != 0) return;
		}
	}
	this->param() = param;
	double pm_5pt[80];
	double tcp[3];		//计算获得的tcp
	double tcp_error;		//计算tcp的误差
	double tcf[9];		//计算获得的tcf
	for (int i = 0; i < 5; i++)
	{
		double temp_pe[6];
		double temp_pm[16];
		for (int j = 0; j < 6; j++)
		{
			temp_pe[j] = param.pe_5pt[6 * i + j];
		}
		s_pe2pm(temp_pe, temp_pm, "321");
		for (int k = 0; k < 16; k++)
		{
			pm_5pt[16 * i + k] = temp_pm[k];
		}
	}
	int ret = cal_TCP_Z(pm_5pt, tcp, tcp_error, tcf);
	if (ret == 0)
	{
		//将标定结果转换为欧拉角形式
		double re321[3];
		s_rm2re(tcf, re321, "321");
		//const double pose[6] = { tcp[0] * 1000, tcp[1] * 1000, tcp[2] * 1000, re321[0] *180 / PI, re321[1] * 180 / PI, re321[2] * 180 / PI };
		const double pose[6] = { tcp[0], tcp[1], tcp[2], re321[0], re321[1], re321[2] };
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe.push_back(pose[i]);
		}
		//param.calib_info = "工具坐标系5点标定完成！工具中心点（TCP）的拟合误差是：" + std::to_string(tcp_error * 1000) + "mm";
		const std::string calib_info = "The calculation is done! The calibration error of TCP is:" + std::to_string(tcp_error * 1000) + "mm";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("The calculation process was aborted!");		//无法计算标定结果，获取的示教点异常，请重新执行标定过程。
		param.calib_info = std::string("The calculation process was aborted!").c_str();
		//return;
	}
	
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("tool_pe", param.tool_pe));
	this->ret()= out_param;
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibT5P::CalibT5P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_t5p\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}\"/>"
		"   </GroupParam>"
		"</Command>");
}
auto CalibT5P::cal_TCP_Z(double transmatric[80], double tcp[3], double &tcp_error, double tcf[9])->int
{
	//计算R(i)-R(i+1),P(i+1)-P(i)
	double R[45], P[15];
	tm2RP_5Pt(transmatric, R, P);
	double tcp_R[36], tcp_P[12], tcf_R[18], tcf_P[6];
	//获取计算tcp的数据
	for (int i = 0; i < 36; i++)
	{
		tcp_R[i] = R[i];
	}
	for (int i = 0; i < 12; i++)
	{
		tcp_P[i] = P[i];
	}
	//获取计算tcf的数据
	for (int i = 0; i < 18; i++)
	{
		tcf_R[i] = R[45 - 18 + i];
	}
	for (int i = 0; i < 6; i++)
	{
		tcf_P[i] = P[15 - 6 + i];
	}

	//计算tcp
	double deltaR[27], deltaP[9];
	deltaRP_5Pt(tcp_R, tcp_P, deltaR, deltaP);
	double U[27], tau[9];
	aris::Size p[9];
	aris::Size rank;
	s_householder_utp(9, 3, deltaR, U, tau, p, rank, 1e-10);
	if (rank < 3)
	{
		//throw std::runtime_error("示教点数据异常，请重新获取示教点！");
		return -1;
	}
	double tau2[9], pinv[27];
	s_householder_utp2pinv(9, 3, rank, U, tau, p, pinv, tau2, 1e-10);
	s_mm(3, 1, 9, pinv, deltaP, tcp);
	//计算tcp_error
	double new_deltaP[9], error_vec[9];
	s_mm(9, 1, 3, deltaR, tcp, new_deltaP);
	for (int i = 0; i < 9; i++)
	{
		error_vec[i] = new_deltaP[i] - deltaP[i];
	}
	double prod_error[3] = { 0 };
	double sum_prod_error = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			prod_error[i] = prod_error[i] + error_vec[i * 3 + j] * error_vec[i * 3 + j];
		}
		sum_prod_error = sum_prod_error + prod_error[i];
	}
	tcp_error = std::sqrt(sum_prod_error);

	//计算tcf
	/*double tcf_deltaR[9], tcf_deltaP[3];
	deltaRP_5Pt(tcf_R, tcf_P, tcf_deltaR, tcf_deltaP);
	double _tcf_deltaR[9], temp_vec[3], oz_vec[3];
	for (int i = 0; i < 9; i++)
	{
		_tcf_deltaR[i] = -tcf_deltaR[i];
	}*/
	double tcf_deltaRz[9], tcf_deltaPz[3];
	double temp_vec[3], oz_vec[3];
	for (int i = 0; i < 9; i++)
	{
		tcf_deltaRz[i] = tcf_R[9 + i] - tcf_R[i];
	}
	for (int i = 0; i < 3; i++)
	{
		tcf_deltaPz[i] = tcf_P[i + 3] - tcf_P[i];
	}
	s_mm(3, 1, 3, tcf_deltaRz, tcp, temp_vec);
	for (int i = 0; i < 3; i++)
	{
		oz_vec[i] = temp_vec[i] + tcf_deltaPz[i];
	}
	double prod_oz = 0;
	double len_oz = 0;
	for (int i = 0; i < 3; i++)
	{
		prod_oz = prod_oz + oz_vec[i] * oz_vec[i];
	}
	len_oz = std::sqrt(prod_oz);
	double tcf_R0[9];
	for (int i = 0; i < 9; i++)
	{
		tcf_R0[i] = tcf_R[i];
	}
	double U_tcf[9], tau_tcf[3];
	aris::Size p_tcf[3];
	aris::Size rank_tcf;
	s_householder_utp(3, 3, tcf_R0, U_tcf, tau_tcf, p_tcf, rank_tcf, 1e-10);
	double tau2_tcf[3], pinv_tcf[9];
	s_householder_utp2pinv(3, 3, rank_tcf, U_tcf, tau_tcf, p_tcf, pinv_tcf, tau2_tcf, 1e-10);
	double a_Tz[3];
	s_mm(3, 1, 3, pinv_tcf, oz_vec, a_Tz);
	for (int i = 0; i < 3; i++)
	{
		a_Tz[i] = a_Tz[i] / len_oz;
	}
	//末端Flange坐标系的Z轴（0，0，1）经过旋转后变为a_Tz,则Fz×a_Tz为它们所在平面的法向量
	double Fz[3] = { 0,0,1 };
	double s_Ty[3], n_Tx[3];
	double prod_oy = 0;
	double prod_ox = 0;
	//叉乘获得Y轴方向向量及向量单位化
	crossVector(a_Tz, Fz, s_Ty);
	for (int i = 0; i < 3; i++)
	{
		prod_oy = prod_oy + s_Ty[i] * s_Ty[i];
	}
	for (int i = 0; i < 3; i++)
	{
		s_Ty[i] = s_Ty[i] / std::sqrt(prod_oy);
	}
	//叉乘获得X轴方向向量及向量单位化
	crossVector(s_Ty, a_Tz, n_Tx);
	for (int i = 0; i < 3; i++)
	{
		prod_ox = prod_ox + n_Tx[i] * n_Tx[i];
	}
	for (int i = 0; i < 3; i++)
	{
		n_Tx[i] = n_Tx[i] / std::sqrt(prod_ox);
	}
	for (int i = 0; i < 3; i++)
	{
		tcf[i * 3 + 0] = n_Tx[i];
		tcf[i * 3 + 1] = s_Ty[i];
		tcf[i * 3 + 2] = a_Tz[i];
	}
	return 0;
}
auto CalibT5P::tm2RP_5Pt(double tm[80], double *R, double *P)->int
{
	//将n_pt*16坐标变换矩阵中提取n_pt*9旋转矩阵和n_pt*3平移向量
	int num = 80;
	int n_pt = 5;
	double temp[5][16] = { 0 };
	for (int i = 0; i < num; i++)
	{
		int row = (int)floor(i / 16);
		int col = i % 16;
		for (int j = 0; j < 16; j++)
		{
			temp[row][col] = tm[i];
		}
	}
	for (int i = 0; i < n_pt; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i * 9 + j] = temp[i][j];
			P[i * 3 + j] = temp[i][j * 4 + 3];
		}
		for (int j = 3; j < 6; j++)
		{
			R[i * 9 + j] = temp[i][j + 1];
		}
		for (int j = 6; j < 9; j++)
		{
			R[i * 9 + j] = temp[i][j + 2];
		}
	}
	return 0;
}
auto CalibT5P::deltaRP_5Pt(double R[45], double P[15], double * deltaR, double * deltaP)->int
{
	//计算R(i)-R(i+1),P(i+1)-P(i),i=示教点总数n_pt减1
	//deltaR[27],deltaP[9]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			deltaR[i * 9 + j] = R[i * 9 + j] - R[(i + 1) * 9 + j];
		}
		for (int j = 0; j < 3; j++)
		{
			deltaP[3 * i + j] = P[(i + 1) * 3 + j] - P[i * 3 + j];
		}
	}
	return 0;
}


//6点标定法
struct CalibT6PParam
{
	double pe_6pt[36];
	std::vector<double> tool_pe;
	std::string calib_info;
};
auto CalibT6P::prepairNrt()->void
{
	//参数初始化，获取当前示教点位姿数据
	CalibT6PParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "pose")
		{
			std::string tempstr = std::string(p.second);
			int ret1 = get_teachpt_data(tempstr, param.pe_6pt, 36);
			if (ret1 != 0) return;
		}
	}
	this->param() = param;
	double pm_6pt[96];
	double tcp[3];		//计算获得的tcp
	double tcp_error;		//计算tcp的误差
	double tcf[9];		//计算获得的tcf
	for (int i = 0; i < 6; i++)
	{
		double temp_pe[6];
		double temp_pm[16];
		for (int j = 0; j < 6; j++)
		{
			temp_pe[j] = param.pe_6pt[6 * i + j];
		}
		s_pe2pm(temp_pe, temp_pm, "321");
		for (int k = 0; k < 16; k++)
		{
			pm_6pt[16 * i + k] = temp_pm[k];
		}
	}
	int ret = cal_TCP_TCF(pm_6pt, tcp, tcp_error, tcf);
	if (ret == 0)
	{
		//将标定结果转换为欧拉角形式
		double re321[3];
		s_rm2re(tcf, re321, "321");
		//const double pose[6] = { tcp[0] * 1000, tcp[1] * 1000, tcp[2] * 1000, re321[0] * 180 / PI, re321[1] * 180 / PI, re321[2] * 180 / PI };
		const double pose[6] = { tcp[0], tcp[1], tcp[2], re321[0], re321[1], re321[2] };
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe.push_back(pose[i]);
		}
		
		//param.calib_info = "工具坐标系6点标定完成！工具中心点（TCP）的拟合误差是：" + std::to_string(tcp_error * 1000) + "mm";
		const std::string calib_info = "The calculation is done! The calibration error of TCP is:" + std::to_string(tcp_error * 1000) + "mm";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("The calculation process was aborted!");		//无法计算标定结果，获取的示教点异常，请重新执行标定过程。
		const std::string calib_info = "The calculation process was aborted!";
		param.calib_info = calib_info.c_str();
		//return;
	}
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	out_param.push_back(std::make_pair<std::string, std::any>("tool_pe", param.tool_pe));
	this->ret() = out_param;

	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibT6P::CalibT6P(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_t6p\">"
		"	<GroupParam>"
		"		<Param name=\"pose\" default=\"{0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}\"/>"
		"   </GroupParam>"
		"</Command>");
}
auto CalibT6P::cal_TCP_TCF(double transmatric[96], double tcp[3], double &tcp_error, double tcf[9])->int
{
	//计算R(i)-R(i+1),P(i+1)-P(i)
	double R[54], P[18];
	tm2RP_6Pt(transmatric, R, P);
	double tcp_R[36], tcp_P[12], tcf_R[27], tcf_P[9];
	//获取计算tcp的数据
	for (int i = 0; i < 36; i++)
	{
		tcp_R[i] = R[i];
	}
	for (int i = 0; i < 12; i++)
	{
		tcp_P[i] = P[i];
	}
	//获取计算tcf的数据
	for (int i = 0; i < 27; i++)
	{
		tcf_R[i] = R[54 - 27 + i];
	}
	for (int i = 0; i < 9; i++)
	{
		tcf_P[i] = P[18 - 9 + i];
	}

	//计算tcp
	double deltaR[27], deltaP[9];
	deltaRP_6Pt(tcp_R, tcp_P, deltaR, deltaP);
	double U[27], tau[9];
	aris::Size p[9];
	aris::Size rank;
	s_householder_utp(9, 3, deltaR, U, tau, p, rank, 1e-10);
	if (rank < 3)
	{
		//throw std::runtime_error("示教点数据异常，请重新获取示教点！");
		return -1;
	}
	double tau2[9], pinv[27];
	s_householder_utp2pinv(9, 3, rank, U, tau, p, pinv, tau2, 1e-10);
	s_mm(3, 1, 9, pinv, deltaP, tcp);
	//计算tcp_error
	double new_deltaP[9], error_vec[9];
	s_mm(9, 1, 3, deltaR, tcp, new_deltaP);
	for (int i = 0; i < 9; i++)
	{
		error_vec[i] = new_deltaP[i] - deltaP[i];
	}
	double prod_error[3] = { 0 };
	double sum_prod_error = 0;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			prod_error[i] = prod_error[i] + error_vec[i * 3 + j] * error_vec[i * 3 + j];
		}
		sum_prod_error = sum_prod_error + prod_error[i];
	}
	tcp_error = std::sqrt(sum_prod_error);

	//计算tcf
	double tcf_deltaRz[9], tcf_deltaPz[3], tcf_deltaRy[9], tcf_deltaPy[3];
	for (int i = 0; i < 9; i++)
	{
		tcf_deltaRz[i] = tcf_R[9 + i] - tcf_R[i];
		tcf_deltaRy[i] = tcf_R[18 + i] - tcf_R[i];
	}
	for (int i = 0; i < 3; i++)
	{
		tcf_deltaPz[i] = tcf_P[i + 3] - tcf_P[i];
		tcf_deltaPy[i] = tcf_P[i + 6] - tcf_P[i];
	}
	double temp_vecZ[3], oz_vec[3];
	double temp_vecY[3], oy_vec[3];
	s_mm(3, 1, 3, tcf_deltaRz, tcp, temp_vecZ);
	s_mm(3, 1, 3, tcf_deltaRy, tcp, temp_vecY);
	for (int i = 0; i < 3; i++)
	{
		oz_vec[i] = temp_vecZ[i] + tcf_deltaPz[i];
		oy_vec[i] = temp_vecY[i] + tcf_deltaPy[i];
	}
	double prod_oz = 0;
	double len_oz = 0;
	double prod_oy = 0;
	double len_oy = 0;
	for (int i = 0; i < 3; i++)
	{
		prod_oz = prod_oz + oz_vec[i] * oz_vec[i];
		prod_oy = prod_oy + oy_vec[i] * oy_vec[i];
	}
	len_oz = std::sqrt(prod_oz);
	len_oy = std::sqrt(prod_oy);
	double tcf_R0[9];
	for (int i = 0; i < 9; i++)
	{
		tcf_R0[i] = tcf_R[i];
	}
	double U_tcf[9], tau_tcf[3];
	aris::Size p_tcf[3];
	aris::Size rank_tcf;
	s_householder_utp(3, 3, tcf_R0, U_tcf, tau_tcf, p_tcf, rank_tcf, 1e-10);
	double tau2_tcf[3], pinv_tcf[9];
	s_householder_utp2pinv(3, 3, rank_tcf, U_tcf, tau_tcf, p_tcf, pinv_tcf, tau2_tcf, 1e-10);
	double s_Ty[3], a_Tz[3];
	s_mm(3, 1, 3, pinv_tcf, oz_vec, a_Tz);
	s_mm(3, 1, 3, pinv_tcf, oy_vec, s_Ty);
	for (int i = 0; i < 3; i++)
	{
		a_Tz[i] = a_Tz[i] / len_oz;
		s_Ty[i] = s_Ty[i] / len_oy;
	}
	double n_Tx[3];
	double prod_ox = 0;
	//叉乘获得X轴方向向量及向量单位化
	crossVector(s_Ty, a_Tz, n_Tx);
	for (int i = 0; i < 3; i++)
	{
		prod_ox = prod_ox + n_Tx[i] * n_Tx[i];
	}
	for (int i = 0; i < 3; i++)
	{
		n_Tx[i] = n_Tx[i] / std::sqrt(prod_ox);
	}
	/*std::cout << a_Tz[0] << "," << a_Tz[1] << "," << a_Tz[2] << std::endl;
	std::cout << s_Ty[0] << "," << s_Ty[1] << "," << s_Ty[2] << std::endl;
	std::cout << n_Tx[0] << "," << n_Tx[1] << "," << n_Tx[2] << std::endl;*/
	for (int i = 0; i < 3; i++)
	{
		tcf[i * 3 + 0] = n_Tx[i];
		tcf[i * 3 + 1] = s_Ty[i];
		tcf[i * 3 + 2] = a_Tz[i];
	}
	return 0;
}
auto CalibT6P::tm2RP_6Pt(double tm[96], double *R, double *P)->int
{
	//将n_pt*16坐标变换矩阵中提取n_pt*9旋转矩阵和n_pt*3平移向量
	int num = 96;
	int n_pt = 6;
	double temp[6][16] = { 0 };
	for (int i = 0; i < num; i++)
	{
		int row = (int)floor(i / 16);
		int col = i % 16;
		for (int j = 0; j < 16; j++)
		{
			temp[row][col] = tm[i];
		}
	}
	for (int i = 0; i < n_pt; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R[i * 9 + j] = temp[i][j];
			P[i * 3 + j] = temp[i][j * 4 + 3];
		}
		for (int j = 3; j < 6; j++)
		{
			R[i * 9 + j] = temp[i][j + 1];
		}
		for (int j = 6; j < 9; j++)
		{
			R[i * 9 + j] = temp[i][j + 2];
		}
	}
	return 0;
}
auto CalibT6P::deltaRP_6Pt(double R[54], double P[18], double * deltaR, double * deltaP)->int
{
	//计算R(i)-R(i+1),P(i+1)-P(i),i=示教点总数n_pt减1
	//deltaR[27],deltaP[9]
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 9; j++)
		{
			deltaR[i * 9 + j] = R[i * 9 + j] - R[(i + 1) * 9 + j];
		}
		for (int j = 0; j < 3; j++)
		{
			deltaP[3 * i + j] = P[(i + 1) * 3 + j] - P[i * 3 + j];
		}
	}
	return 0;
}


//设定工具坐标系的标定结果
struct SetTFParam
{
	//size_t tool_id;
	std::string tool_name;
	double tool_pe[6];
	std::string calib_info;
};
auto SetTF::prepairNrt()->void
{
	//参数初始化
	SetTFParam param;
	std::vector<std::string> tempvec;
	for (auto &p : cmdParams())
	{
		/*if (p.first == "tool_id")
		{
			param.tool_id = int32Param(p.first);
		}*/
		if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
		else if (p.first == "tool_pe")
		{
			std::string tempstr=std::string(p.second);
			std::string posestr = tempstr.substr(tempstr.find("{") + 1, tempstr.find("}") - tempstr.find("{") - 1);
			std::string::size_type pos1 = 0;
			std::string::size_type pos2 = posestr.find(",");
			
			while (pos2 != std::string::npos)
			{
				tempvec.push_back(posestr.substr(pos1, pos2 - pos1));
				pos1 = pos2 + 1;
				pos2 = posestr.find(",", pos1);
			}
			tempvec.push_back(posestr.substr(pos1, posestr.length() - 1));
		}
	}
	if (tempvec.size() != 6)
	{
		throw std::runtime_error("The input data of teaching point's pose is wrong！");		//工具坐标系位姿的输入数据有误！
		param.calib_info = std::string("The input data of teaching point's pose is wrong！");
	}
	else
	{
		//获取工具坐标系相对于法兰坐标系的位姿
		for (int i = 0; i < 6; i++)
		{
			param.tool_pe[i] = std::stod(tempvec.at(i));
		}
		double tool_pm_f[16];
		s_pe2pm(param.tool_pe, tool_pm_f, "321");
		//获取法兰坐标系相对于底座坐标系的位姿
		double tool0_pm_g[16];
		try
		{
			auto mat1 = model()->partPool().findByName("L6")->markerPool().findByName("tool0")->prtPm();
			for (size_t i = 0; i < 4; i++)
			{
				for (size_t j = 0; j < 4; j++)
				{
					tool0_pm_g[4 * i + j] = mat1[i][j];
				}
			}
		}
		catch (std::exception)
		{
			throw std::runtime_error("cann't find \"tool0\" node in partPool.");
		}
		
		//计算工具坐标系相对于底座坐标系的位姿
		double tool_pm_g[16];
		double tool_pe_g[6];
		s_mm(4, 4, 4, tool0_pm_g, tool_pm_f, tool_pm_g);
		s_pm2pe(tool_pm_g, tool_pe_g, "313");
		try
		{
			model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->setPrtPe(tool_pe_g);
		}
		catch(std::exception)
		{
			throw std::runtime_error("cann't find \"" + param.tool_name + "\" node in partPool.");
		}
		/*
		auto mat2 = model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->prtPm();
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				std::cout << mat2[i][j] << ",";
			}
			std::cout << std::endl;
		}
		*/
		//param.calib_info = "工具坐标系位姿的配置节点已生成。";
		const std::string calib_info = "The configuration node of " + param.tool_name +"'s pose is created or updated.";
		param.calib_info = calib_info.c_str();
	}
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
SetTF::SetTF(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"settf\">"
		"	<GroupParam>"
		"		<Param name=\"tool_name\" default=\"Tool0\"/>"
		"		<Param name=\"tool_pe\" default=\"{0,0,0,0,0,0}\"/>"
		"   </GroupParam>"
		"</Command>");
}


//首次或无负载零点标定（机器人上无工具负载、附加负载及外部负载时进行的标定，将此时各轴编码器的值作为基准值）
//无负载零点标定与工具无关
//机器人工作时仅有一种固定或波动较小的负载时也可以采用此方法进行零点标定
struct CalibZFParam 
{
	size_t axis_id;		//当前标定轴的索引值0-5
	std::string calib_info;
};
double CalibZF::zeroVal[6] = { 0 };
auto CalibZF::prepairNrt()->void
{
	//参数初始化
	CalibZFParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "axis_id")
		{
			param.axis_id = int32Param(p.first);
		}
	}
	////从控制器配置文件里读取待标定轴原来的零点位置
	
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			CalibZF::zeroVal[i] = mat->data().data()[i];
		}
	}
	
	this->param() = param;

	//获取当前标定轴的零位数值
	CalibZF::zeroVal[param.axis_id] = model()->motionPool().at(param.axis_id).mp();
	
	//将新的标定轴零点位置写入控制器配置文件
	aris::core::Matrix mat1(1, 6, CalibZF::zeroVal);
	//model()->variablePool().findByName("tool0_axis_home")

	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"))->data() = mat1;
	}
	else
	{
		model()->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", mat1);
	}
	
	const std::string calib_info = "The current axis's zero positon has been updated.";
	param.calib_info = calib_info.c_str();
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibZF::CalibZF(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_zf\">"
		"	<GroupParam>"
		"		<Param name=\"axis_id\" default=\"0\"/>"
		"   </GroupParam>"
		"</Command>");
}


//有负载下的零点偏移量标定（机器人上仅有内部负载，如：工具负载及关节上的附加负载时，获得的是不同工具对应的零点偏移量，它是相对于首次标定基准值的增量）
//当工具作业时的外部负载较大时，需要对不同的外部负载进行零点偏移量标定。
//适用场景——“机器人+末端执行器”的精确定位，例如：焊接、激光切割、钻/铣削、打磨
struct CalibZOParam
{
	size_t axis_id;		//当前标定轴的索引值0-5
	std::string tool_name;		//工具名称
	//bool calib_finished_flag;		//标定是否结束标志
	std::string calib_info;
};
//静态变量初始化
size_t CalibZO::axisNum = 0;		//已标定零点的轴数量
double CalibZO::zeroVal[6] = { 0 };
double CalibZO::offsetVal[6] = { 0 };
auto CalibZO::prepairNrt()->void
{
	//参数初始化
	CalibZOParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "axis_id")
		{
			param.axis_id = int32Param(p.first);
		}
		else if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
		/*else if (p.first == "calib_finished_flag")
		{
			param.calib_finished_flag = int32Param(p.first);
		}*/
	}

	this->param() = param;
	////从控制器配置文件里读取待标定轴原来的零点及相应工具的零点偏移量
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			CalibZO::zeroVal[i] = mat->data().data()[i];
		}
		if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
		{
			auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"));
			for (int i = 0; i < 6; i++)
			{
				CalibZO::offsetVal[i] = mat1->data().data()[i];
			}
		}

		//计算当前标定轴的零位偏移量
		CalibZO::offsetVal[param.axis_id] = model()->motionPool().at(param.axis_id).mp() - CalibZO::zeroVal[param.axis_id];

		//将新的各轴零点位置偏移量写入控制器配置文件
		aris::core::Matrix mat2(1, 6, CalibZO::offsetVal);
		if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
		{
			dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"))->data() = mat2;
		}
		else
		{
			model()->variablePool().add<aris::dynamic::MatrixVariable>(param.tool_name + "_axis_offset", mat2);
		}
		const std::string calib_info = "The current axis's zero offset value with tool has been updated.";
		param.calib_info = calib_info.c_str();
	}
	else
	{
		throw std::runtime_error("\"tool0_axis_home\" is not exist in controller's configuration file");
		param.calib_info = std::string("\"tool0_axis_home\" is not exist in controller's configuration file,please conduct \"CalibZF\" command firstly.").c_str();
		//return;
	}

	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibZO::CalibZO(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_zo\">"
		"	<GroupParam>"
		"		<Param name=\"axis_id\" default=\"0\"/>"
		"		<Param name=\"tool_name\" default=\"tool0\"/>"
		"   </GroupParam>"
		"</Command>");
}


//带工具偏移量、有外部负载下的零点标定（机器人上有除了工具负载和关节上附加负载之外的外部负载，
//利用该负载下的关节位置值减去偏移量来计算带该负载的首次标定零点，代替无负载的首次标定零点）
struct CalibZLParam
{
	size_t axis_id;		//当前标定轴的索引值0-5
	std::string tool_name;		//工具名称
	//bool calib_finished_flag;		//标定是否结束标志
	std::string calib_info;
};
size_t CalibZL::axisNum = 0;		//已标定零点的轴数量
double CalibZL::zeroVal[6] = { 0 };
double CalibZL::offsetVal[6] = { 0 };
auto CalibZL::prepairNrt()->void
{
	//参数初始化
	CalibZLParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "axis_id")
		{
			param.axis_id = int32Param(p.first);
		}
		else if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
		/*else if (p.first == "calib_finished_flag")
		{
			param.calib_finished_flag = int32Param(p.first);
		}*/
	}
	////从控制器配置文件里读取已标定数据
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			CalibZL::zeroVal[i] = mat->data().data()[i];
		}
	}
	else
	{
		throw std::runtime_error("\"tool0_axis_home\" is not exist in controller's configuration file");
		param.calib_info = std::string("\"tool0_axis_home\" is not exist in controller's configuration file").c_str();
		//return;
	}
	if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
	{
		auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"));
		for (int i = 0; i < 6; i++)
		{
			CalibZL::offsetVal[i] = mat1->data().data()[i];
		}
	}
	else
	{
		const std::string name = "\"" + param.tool_name + "_axis_offset\"" + " is not exist in controller's configuration file";
		throw std::runtime_error(name);
	}
	this->param() = param;

	//计算有外部负载时当前标定轴的零位
	CalibZL::zeroVal[param.axis_id] = model()->motionPool().at(param.axis_id).mp() - CalibZL::offsetVal[param.axis_id];
	
	//标定完成后将有负载时新的各轴零点位置写入控制器配置文件，覆盖原零点位置
	aris::core::Matrix mat2(1, 6, CalibZL::zeroVal);
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"))->data() = mat2;
	}
	else
	{
		model()->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", mat2);
	}
	const std::string calib_info = "The current axis's zero position with tool and external load has been updated.";
	param.calib_info = calib_info.c_str();
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;
	
	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
CalibZL::CalibZL(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"calib_zl\">"
		"	<GroupParam>"
		"		<Param name=\"axis_id\" default=\"0\"/>"
		"		<Param name=\"tool_name\" default=\"tool0\"/>"
		"   </GroupParam>"
		"</Command>");
}


//更新机器人模型
struct SwitchToolParam
{
	std::string tool_name;		//工具名称
	double zeroVal[6] = { 0 };
	double offsetVal[6] = { 0 };
	std::string calib_info;
};
auto SwitchTool::prepairNrt()->void
{
	//参数初始化
	SwitchToolParam param;
	for (auto &p : cmdParams())
	{
		if (p.first == "tool_name")
		{
			param.tool_name=std::string(p.second);
		}
	}
	//从控制器配置文件里读取tool0的零位
	if (model()->variablePool().findByName("tool0_axis_home") != model()->variablePool().end())
	{
		auto mat = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("tool0_axis_home"));
		for (int i = 0; i < 6; i++)
		{
			param.zeroVal[i] = mat->data().data()[i];
		}
	}
	else
	{
		throw std::runtime_error("\"tool0_axis_home\" is not exist in controller's configuration file");
		param.calib_info = std::string("\"tool0_axis_home\" is not exist in controller's configuration file").c_str();
		//return;
	}
	//从控制器配置文件里读取当前工具的零位偏移量
	if (param.tool_name == "tool0")
	{
		for (int i = 0; i < 6; i++)
		{
			param.offsetVal[i] = 0;
		}
	}
	else
	{
		if (model()->variablePool().findByName(param.tool_name + "_axis_offset") != model()->variablePool().end())
		{
			auto mat1 = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(param.tool_name + "_axis_offset"));
			for (int i = 0; i < 6; i++)
			{
				param.offsetVal[i] = mat1->data().data()[i];
			}
		}
		else
		{
			const std::string name = "\"" + param.tool_name + "_axis_offset\"" + " is not exist in controller's configuration file";
			throw std::runtime_error(name);
		}
	}
	
	//更新控制器或模型中各轴的零位
	for (int i = 0; i < 6; i++)
	{
		controller()->motionPool()[i].setPosOffset(param.zeroVal[i] + param.offsetVal[i]);
	}
	//更新模型中的工具
	/*
	double tool_pm_g[16];
	try
	{
		auto mat1 = model()->partPool().findByName("L6")->markerPool().findByName(param.tool_name)->prtPm();
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				tool_pm_g[4 * i + j] = mat1[i][j];
			}
		}
		//更新模型的工具坐标系//

	}
	catch (std::exception)
	{
		throw std::runtime_error(param.tool_name + " node is not in partPool.");
	}
	*/
	const std::string calib_info = "The tool of robot model has been updated.";
	param.calib_info = calib_info.c_str();
	std::vector<std::pair<std::string, std::any>> out_param;
	out_param.push_back(std::make_pair<std::string, std::any>("calib_info", param.calib_info));
	ret() = out_param;

	option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
}
SwitchTool::SwitchTool(const std::string &name) :Plan(name)
{
	command().loadXmlStr(
		"<Command name=\"switchtool\">"
		"	<GroupParam>"
		"		<Param name=\"tool_name\" default=\"tool0\"/>"
		"   </GroupParam>"
		"</Command>");
}
