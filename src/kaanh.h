#ifndef KAANH_H_
#define KAANH_H_

#include <memory>
#include <aris.hpp>
#include"forcecontrol.h"
#include"iir.h"

//statemachine//
# define M_RUN 0	//手动单步执行
# define READ_RT_DATA 1		//监控实时数据
# define READ_XML 2		//监控实时数据
# define A_RUN 3	//自动执行
# define A_QUIT 4	//退出自动执行，返回到手动模式
# define buffer_length 800
//statemachine//

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
	auto createInterface()->std::unique_ptr<aris::server::InterfaceRoot>;
	auto createControllerEA()->std::unique_ptr<aris::control::Controller>;
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
	auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerSanXiang()->std::unique_ptr<aris::control::Controller>;
	auto createModelSanXiang()->std::unique_ptr<aris::dynamic::Model>;

    auto createControllerDaye()->std::unique_ptr<aris::control::Controller>;
    auto createModelDaye(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;

	//auto registerPlan()->void;

	class Get : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit Get(const std::string &name = "Get_plan");
		ARIS_REGISTER_TYPE(Get);
	};


	class MoveX : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveX(const std::string &name = "MoveX_plan");
		ARIS_REGISTER_TYPE(MoveX);
	};

	class MoveJS : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJS(const std::string &name = "MoveJS_plan");
		ARIS_REGISTER_TYPE(MoveJS);
	};

	class MoveJSN : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJSN(const std::string &name = "MoveJSN_plan");
		ARIS_REGISTER_TYPE(MoveJSN);
	};

	class MoveJR : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJR(const std::string &name = "MoveJR_plan");
		ARIS_REGISTER_TYPE(MoveJR);
	};

	class MoveTTT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveTTT(const std::string &name = "MoveTTT_plan");
		ARIS_REGISTER_TYPE(MoveTTT);
	};

	class MoveJM : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJM(const std::string &name = "MoveJM_plan");
		ARIS_REGISTER_TYPE(MoveJM);
	};

	class MoveJI : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveJI(const std::string &name = "MoveJI_plan");
		ARIS_REGISTER_TYPE(MoveJI);
	};

	class JogC : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		virtual ~JogC();
		explicit JogC(const std::string &name = "JogJ_plan");
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

	class Grasp : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit Grasp(const std::string &name = "Grasp_plan");
		ARIS_REGISTER_TYPE(Grasp);
	};

	class ListenDI : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		explicit ListenDI(const std::string &name = "ListenDI_plan");
		ARIS_REGISTER_TYPE(ListenDI);
	};

	class MoveEA : public aris::plan::Plan
	{
	public:
		//平均值速度滤波、摩擦力滤波器初始化//
		std::vector<double> fore_vel;
		IIR_FILTER::IIR iir;
		double tempforce;
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveEA(const std::string &name = "MoveEA_plan");
		ARIS_REGISTER_TYPE(MoveEA);
	};

	class MoveEAP : public aris::plan::Plan
	{
	public:
		//平均值速度滤波、摩擦力滤波器初始化//
		std::vector<double> fore_vel;
		IIR_FILTER::IIR iir;
		double tempforce;
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit MoveEAP(const std::string &name = "MoveEAP_plan");
		ARIS_REGISTER_TYPE(MoveEAP);
	};

	class FSSignal : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		auto virtual executeRT(aris::plan::PlanTarget &target)->int;
		auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

		explicit FSSignal(const std::string &name = "FSSignal_plan");
		ARIS_REGISTER_TYPE(FSSignal);
	};

    class ATIFS : public aris::plan::Plan
    {
    public:
        auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
        auto virtual executeRT(aris::plan::PlanTarget &target)->int;
        auto virtual collectNrt(aris::plan::PlanTarget &target)->void;

        explicit ATIFS(const std::string &name = "ATIFS_plan");
        ARIS_REGISTER_TYPE(ATIFS);
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

	class SaveConfig : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SaveConfig(const std::string &name = "SaveConfig_plan");
		ARIS_REGISTER_TYPE(SaveConfig);
	};

	class ClearCon : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit ClearCon(const std::string &name = "ClearCon_plan");
		ARIS_REGISTER_TYPE(ClearCon);
	};

	class StartCS : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit StartCS(const std::string &name = "StartCS_plan");
		ARIS_REGISTER_TYPE(StartCS);
	};

	class Update : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit Update(const std::string &name = "Update_plan");
		ARIS_REGISTER_TYPE(Update);
	};

	class StopCS : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit StopCS(const std::string &name = "StopCS_plan");
		ARIS_REGISTER_TYPE(StopCS);
	};

	class SetCT : public aris::plan::Plan
	{
	public:
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void;
		explicit SetCT(const std::string &name = "SetCT_plan");
		ARIS_REGISTER_TYPE(SetCT);
	};
}

#endif
