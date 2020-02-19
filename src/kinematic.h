#ifndef _KINEMATIC_H_
#define _KINEMATIC_H_
#include <aris.hpp>

const double PI = 3.141592653589793;

auto crossVector(double* a, double* s, double* n)->int;
auto mmdeg2mrad(std::string datastr, double *data, size_t data_size)->int;
auto get_teachpt_data(std::string datastr, double *data, size_t data_size)->int;
/*
�����˹�������ϵ�궨������4�㡢5�㡢6��궨�㷨��
���������궨�������״α궨������ƫ�ñ궨��
*/

//4��-TCP�궨������4����㹤������ϵԭ��λ��
class CalibT4P : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit CalibT4P(const std::string &name = "CalibT4P");
	ARIS_REGISTER_TYPE(CalibT4P);
	
	auto cal_TCP(double transmatric[64], double tcp[3], double &tcp_error)->int;
	auto tm2RP_4Pt(double tm[64], double *R, double *P)->int;
	auto deltaRP_4Pt(double R[36], double P[12], double * deltaR, double * deltaP)->int;
private:
	
};

//5��-TCP��Z�궨������5����㹤������ϵԭ��λ�ü�Z����������̬
class CalibT5P : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit CalibT5P(const std::string &name = "CalibT5P");
	ARIS_REGISTER_TYPE(CalibT5P);
	
	auto cal_TCP_Z(double transmatric[80], double tcp[3], double &tcp_error, double tcf[9])->int;
	auto tm2RP_5Pt(double tm[80], double *R, double *P)->int;
	auto deltaRP_5Pt(double R[45], double P[15], double * deltaR, double * deltaP)->int;
private:
	
};

//6��-TCP��TCF��Z�ᡢY��궨������6����㹤������ϵԭ��λ�ü�Z��Y��X����������̬
class CalibT6P : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit CalibT6P(const std::string &name = "CalibT6P");
	ARIS_REGISTER_TYPE(CalibT6P);
	
	auto cal_TCP_TCF(double transmatric[96], double tcp[3], double &tcp_error, double tcf[9])->int;
	auto tm2RP_6Pt(double tm[96], double *R, double *P)->int;
	auto deltaRP_6Pt(double R[54], double P[18], double * deltaR, double * deltaP)->int;
private:
	
};



//�趨��������ϵ�ı궨���
class SetTF : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit SetTF(const std::string &name = "SetTF");
	ARIS_REGISTER_TYPE(SetTF);

private:

};

/*
//�û���������������
class RenameT : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit RenameT(const std::string &name = "RenameT");
	ARIS_REGISTER_TYPE(RenameT);

private:

};
*/



//���������-�״Ρ��޸��ر궨
class CalibZF : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit CalibZF(const std::string &name = "CalibZF");
	ARIS_REGISTER_TYPE(CalibZF);

	
private:
	//static size_t axisNum;		//�ѱ궨����������
	static double zeroVal[6];		//����궨��Ľ��

};

//���������-����ƫ�ñ궨
class CalibZO : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit CalibZO(const std::string &name = "CalibZO");
	ARIS_REGISTER_TYPE(CalibZO);
private:
	static size_t axisNum;		//�ѱ궨����������
	static double zeroVal[6];
	static double offsetVal[6];


};

//���������-�ⲿ���ر궨
class CalibZL : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit CalibZL(const std::string &name = "CalibZL");
	ARIS_REGISTER_TYPE(CalibZL);
private:
	static size_t axisNum;		//�ѱ궨����������
	static double zeroVal[6];
	static double offsetVal[6];

};

//���õ�ǰ���ߺ���»�����ģ��-
class SwitchTool : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
	explicit SwitchTool(const std::string &name = "SwitchTool");
	ARIS_REGISTER_TYPE(SwitchTool);
private:


};



#endif