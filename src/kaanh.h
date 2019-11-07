#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>
#include"forcecontrol.h"
#include"iir.h"
#include"CJsonObject.hpp"
#include "kinematic.h"

//statemachine old//
# define M_RUN 0	//手动单步执行
# define READ_RT_DATA 1		//监控实时数据
# define READ_XML 2		//监控实时数据
# define A_RUN 3	//自动执行
# define A_QUIT 4	//退出自动执行，返回到手动模式
# define buffer_length 800
//statemachine old//

//statemachine new//
# define DISABLED 100	//去使能
# define MANUAL 200		//手动模式
# define PRE_AUTO 300	//准自动模式
# define AUTO 400		//自动模式
# define ERROR 500		//错误模式
//statemachine new//

// \brief 机器人命名空间
// \ingroup aris
//


namespace kaanh
{
	//电缸力检测参数声明
	//const std::string xmlpath = "C:\\Users\\kevin\\Desktop\\aris_rokae\\ServoPressorCmdList.xml";
	constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//

	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	
	struct SpeedParam
	{
		double w_percent;	//关节速度百分比
		double v_tcp;	//TCP线速度mm/s
		double w_tcp;	//空间旋转速度°/s
		double w_ext;	//外部轴角速度°/s
		double v_ext;	//外部轴线速度mm/s
	};

	auto createInterface()->std::unique_ptr<aris::server::InterfaceRoot>;
	auto createControllerEA()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;
	
	auto createControllerSanXiang()->std::unique_ptr<aris::control::Controller>;
	auto createModelSanXiang()->std::unique_ptr<aris::dynamic::Model>;

	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>;
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>;

    auto createControllerDaye()->std::unique_ptr<aris::control::Controller>;
    auto createModelDaye(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;

	auto update_state(aris::server::ControlServer &cs)->void;
	auto get_state_code()->std::int32_t;

	struct CmdListParam
	{
		std::map<int, std::string> cmd_vec;
		int current_cmd_id = 0;
		int current_plan_id = -1;
	};
	
	class ProInterface :public aris::server::Interface
	{
	public:
		auto virtual open()->void override;
		auto virtual close()->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;

		ProInterface(const std::string &name = "interface", const std::string &port = "5868", aris::core::Socket::TYPE type = aris::core::Socket::WEB);
		ARIS_REGISTER_TYPE(ProInterface);
		ARIS_DEFINE_BIG_FOUR(ProInterface);

	private:
		aris::core::Socket *sock_;
	};

	class Speed
	{
	public:
		auto setspeed(SpeedParam speed)->void
		{
			s = speed;
		};
		auto getspeed()->SpeedParam
		{
			return s;
		};
		Speed(SpeedParam speed = {0.0,0.0,0.0,0.0,0.0})
		{
			s = speed;
		};
	
	private:
		SpeedParam s;
	};

	class Get : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit Get(const std::string &name = "Get_plan");
		ARIS_REGISTER_TYPE(Get);
	};

	class Enable : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void override;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int override;

		virtual ~Enable();
		explicit Enable(const std::string &name = "enable_plan");
		ARIS_REGISTER_TYPE(Enable);
		ARIS_DECLARE_BIG_FOUR(Enable);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class Disable : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~Disable();
		explicit Disable(const std::string &name = "enable_plan");
		ARIS_REGISTER_TYPE(Disable);
		ARIS_DECLARE_BIG_FOUR(Disable);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	class Home : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~Home();
		explicit Home(const std::string &name = "home_plan");
		ARIS_REGISTER_TYPE(Home);
		ARIS_DECLARE_BIG_FOUR(Home);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class Mode : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~Mode();
		explicit Mode(const std::string &name = "mode_plan");
		ARIS_REGISTER_TYPE(Mode);
		ARIS_DECLARE_BIG_FOUR(Mode);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	class Reset : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~Reset();
		explicit Reset(const std::string &name = "reset_plan");
		ARIS_REGISTER_TYPE(Reset);
		ARIS_DECLARE_BIG_FOUR(Reset);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	class Recover : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~Recover();
		explicit Recover(const std::string &name = "recover_plan");
		ARIS_REGISTER_TYPE(Recover);
		ARIS_DECLARE_BIG_FOUR(Recover);
	};
	
	class Sleep : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~Sleep();
		explicit Sleep(const std::string &name = "sleep_plan");
		ARIS_REGISTER_TYPE(Sleep);
		ARIS_DECLARE_BIG_FOUR(Sleep);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class MoveF : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveF(const std::string &name = "MoveF_plan");
		ARIS_REGISTER_TYPE(MoveF);
	};

	class MoveAbsJ :public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~MoveAbsJ();
		explicit MoveAbsJ(const std::string &name = "MoveAbsJ");
		ARIS_REGISTER_TYPE(kaanh::MoveAbsJ);
	};
	
	class MoveJ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~MoveJ();
		explicit MoveJ(const std::string &name = "move_j");
		ARIS_REGISTER_TYPE(kaanh::MoveJ);
		ARIS_DECLARE_BIG_FOUR(MoveJ);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	class MoveL : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~MoveL();
		explicit MoveL(const std::string &name = "move_l");
		ARIS_REGISTER_TYPE(kaanh::MoveL);
		ARIS_DECLARE_BIG_FOUR(MoveL);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class MoveC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void override;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;

		virtual ~MoveC();
		explicit MoveC(const std::string &name = "MoveC_plan");
		ARIS_REGISTER_TYPE(MoveC);
		ARIS_DECLARE_BIG_FOUR(MoveC);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogC();
		explicit JogC(const std::string &name = "JogC_plan");
		ARIS_REGISTER_TYPE(JogC);
		JogC(const JogC &);
		JogC(JogC &);
		JogC& operator=(const JogC &);
		JogC& operator=(JogC &&);
		

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogJ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ();
		explicit JogJ(const std::string &name = "JogJ_plan");
		ARIS_REGISTER_TYPE(JogJ);
		JogJ(const JogJ &);
		JogJ(JogJ &);
		JogJ& operator=(const JogJ &);
		JogJ& operator=(JogJ &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class JogJ1 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ1();
		explicit JogJ1(const std::string &name = "JogJ1_plan");
		ARIS_REGISTER_TYPE(JogJ1);
	};

	class JogJ2 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ2();
		explicit JogJ2(const std::string &name = "JogJ2_plan");
		ARIS_REGISTER_TYPE(JogJ2);
	};

	class JogJ3 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ3();
		explicit JogJ3(const std::string &name = "JogJ3_plan");
		ARIS_REGISTER_TYPE(JogJ3);
	};

	class JogJ4 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ4();
		explicit JogJ4(const std::string &name = "JogJ4_plan");
		ARIS_REGISTER_TYPE(JogJ4);
	};

	class JogJ5 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ5();
		explicit JogJ5(const std::string &name = "JogJ5_plan");
		ARIS_REGISTER_TYPE(JogJ5);
	};

	class JogJ6 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ6();
		explicit JogJ6(const std::string &name = "JogJ6_plan");
		ARIS_REGISTER_TYPE(JogJ6);
	};

	class JogJ7 : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogJ7();
		explicit JogJ7(const std::string &name = "JogJ7_plan");
		ARIS_REGISTER_TYPE(JogJ7);
	};

	class JX : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JX();
		explicit JX(const std::string &name = "JX_plan");
		ARIS_REGISTER_TYPE(JX);
	};

	class JY : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JY();
		explicit JY(const std::string &name = "JY_plan");
		ARIS_REGISTER_TYPE(JY);
	};

	class JZ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JZ();
		explicit JZ(const std::string &name = "JZ_plan");
		ARIS_REGISTER_TYPE(JZ);
	};

	class JRX : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JRX();
		explicit JRX(const std::string &name = "JRX_plan");
		ARIS_REGISTER_TYPE(JRX);
	};

	class JRY : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JRY();
		explicit JRY(const std::string &name = "JRY_plan");
		ARIS_REGISTER_TYPE(JRY);
	};

	class JRZ : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JRZ();
		explicit JRZ(const std::string &name = "JRZ_plan");
		ARIS_REGISTER_TYPE(JRZ);
	};

	class SetCon : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetCon(const std::string &name = "SetCon_plan");
		ARIS_REGISTER_TYPE(SetCon);
	};

	class SetDH : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetDH(const std::string &name = "SetDH_plan");
		ARIS_REGISTER_TYPE(SetDH);
	};
	
	class SetPG : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetPG(const std::string &name = "SetPG_plan");
		ARIS_REGISTER_TYPE(SetPG);
	};

	class SetPPath : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetPPath(const std::string &name = "SetPPath_plan");
		ARIS_REGISTER_TYPE(SetPPath);
	};

	class SetUI : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetUI(const std::string &name = "SetUI_plan");
		ARIS_REGISTER_TYPE(SetUI);
	};
	
	class SetDriver : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetDriver(const std::string &name = "SetDriver_plan");
		ARIS_REGISTER_TYPE(SetDriver);
	};

	class SaveXml : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SaveXml(const std::string &name = "SaveXml_plan");
		ARIS_REGISTER_TYPE(SaveXml);
	};

	class ScanSlave : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit ScanSlave(const std::string &name = "ScanSlave_plan");
		ARIS_REGISTER_TYPE(ScanSlave);
	};

	class GetEsiPdoList : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit GetEsiPdoList(const std::string &name = "GetEsiPdoList_plan");
		ARIS_REGISTER_TYPE(GetEsiPdoList);
	};

	class SetEsiPath : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetEsiPath(const std::string &name = "SetEsiPath_plan");
		ARIS_REGISTER_TYPE(SetEsiPath);
	};

	class ClearCon : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit ClearCon(const std::string &name = "ClearCon_plan");
		ARIS_REGISTER_TYPE(ClearCon);
	};

	class Update : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit Update(const std::string &name = "Update_plan");
		ARIS_REGISTER_TYPE(Update);
	};

	class GetXml :public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		virtual ~GetXml();
		explicit GetXml(const std::string &name = "GetXml");
		ARIS_REGISTER_TYPE(kaanh::GetXml);
	};

	class SetXml :public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		virtual ~SetXml();
		explicit SetXml(const std::string &name = "SetXml");
		ARIS_REGISTER_TYPE(kaanh::SetXml);
	};

	class Start :public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		virtual ~Start();
		explicit Start(const std::string &name = "Start");
		ARIS_REGISTER_TYPE(kaanh::Start);
	};

	class Stop :public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;

		virtual ~Stop();
		explicit Stop(const std::string &name = "Stop");
		ARIS_REGISTER_TYPE(kaanh::Stop);
	};

	class SetCT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetCT(const std::string &name = "SetCT_plan");
		ARIS_REGISTER_TYPE(SetCT);
	};

	class SetVel : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetVel(const std::string &name = "SetVel_plan");
		ARIS_REGISTER_TYPE(SetVel);
	};

	class Var : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit Var(const std::string &name = "Var_plan");
		ARIS_REGISTER_TYPE(Var);
	};

	class Run : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;
		explicit Run(const std::string &name = "Run_plan");
		ARIS_REGISTER_TYPE(Run);
	};

	class MoveSt : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit MoveSt(const std::string &name = "MoveSt_plan");
		ARIS_REGISTER_TYPE(MoveSt);
	};

	class Switch : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit Switch(const std::string &name = "Switch_plan");
		ARIS_REGISTER_TYPE(Switch);
	};
}

#endif
