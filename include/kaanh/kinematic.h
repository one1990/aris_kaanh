#ifndef _KINEMATIC_H_
#define _KINEMATIC_H_
#include <aris.hpp>

const double PI = 3.141592653589793;

auto crossVector(double* a, double* s, double* n)->int;
auto mmdeg2mrad(std::string datastr, double *data, size_t data_size)->int;
auto get_teachpt_data(std::string datastr, double *data, size_t data_size)->int;
/*
机器人工具坐标系标定，包含4点、5点、6点标定算法。
机器人零点标定，包含首次标定，负载偏置标定。
*/

//4点-TCP标定，基于4点计算工具坐标系原点位置
class CalibT4P : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit CalibT4P(const std::string &name = "CalibT4P");
	ARIS_REGISTER_TYPE(CalibT4P);
	
	auto cal_TCP(double transmatric[64], double tcp[3], double &tcp_error)->int;
	auto tm2RP_4Pt(double tm[64], double *R, double *P)->int;
	auto deltaRP_4Pt(double R[36], double P[12], double * deltaR, double * deltaP)->int;

};

//5点-TCP和Z标定，基于5点计算工具坐标系原点位置及Z轴正方向姿态
class CalibT5P : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit CalibT5P(const std::string &name = "CalibT5P");
	ARIS_REGISTER_TYPE(CalibT5P);
	
	auto cal_TCP_Z(double transmatric[80], double tcp[3], double &tcp_error, double tcf[9])->int;
	auto tm2RP_5Pt(double tm[80], double *R, double *P)->int;
	auto deltaRP_5Pt(double R[45], double P[15], double * deltaR, double * deltaP)->int;

};

//6点-TCP和TCF的Z轴、Y轴标定，基于6点计算工具坐标系原点位置及Z、Y、X轴正方向姿态
class CalibT6P : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit CalibT6P(const std::string &name = "CalibT6P");
	ARIS_REGISTER_TYPE(CalibT6P);
	
	auto cal_TCP_TCF(double transmatric[96], double tcp[3], double &tcp_error, double tcf[9])->int;
	auto tm2RP_6Pt(double tm[96], double *R, double *P)->int;
	auto deltaRP_6Pt(double R[54], double P[18], double * deltaR, double * deltaP)->int;
	
};



//设定工具坐标系的标定结果
class SetTF : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit SetTF(const std::string &name = "SetTF");
	ARIS_REGISTER_TYPE(SetTF);
};

/*
//用户重命名工具名称
class RenameT : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit RenameT(const std::string &name = "RenameT");
	ARIS_REGISTER_TYPE(RenameT);

private:

};
*/



//机器人零点-首次、无负载标定
class CalibZF : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit CalibZF(const std::string &name = "CalibZF");
	ARIS_REGISTER_TYPE(CalibZF);

private:
	//static size_t axisNum;		//已标定零点的轴数量
	static double zeroVal[6];		//各轴标定后的结果

};

//机器人零点-负载偏置标定
class CalibZO : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit CalibZO(const std::string &name = "CalibZO");
	ARIS_REGISTER_TYPE(CalibZO);
private:
	static size_t axisNum;		//已标定零点的轴数量
	static double zeroVal[6];
	static double offsetVal[6];
};

//机器人零点-外部负载标定
class CalibZL : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit CalibZL(const std::string &name = "CalibZL");
	ARIS_REGISTER_TYPE(CalibZL);
private:
	static size_t axisNum;		//已标定零点的轴数量
	static double zeroVal[6];
	static double offsetVal[6];

};

//重置当前工具后更新机器人模型-
class SwitchTool : public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void;
	explicit SwitchTool(const std::string &name = "SwitchTool");
	ARIS_REGISTER_TYPE(SwitchTool);
};

#endif