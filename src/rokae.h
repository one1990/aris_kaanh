#ifndef ROKAE_H_
#define ROKAE_H_

#include <memory>

#include <aris_control.h>
#include <aris_dynamic.h>
#include <aris_plan.h>
#include <tinyxml2.h>  


//statemachine//
# define M_RUN 0	//手动单步执行
# define READ_RT_DATA 1		//监控实时数据
# define READ_XML 2		//监控实时数据
# define A_RUN 3	//自动执行
# define A_QUIT 4	//退出自动执行，返回到手动模式
# define buffer_length 800
//statemachine//

/// \brief 机器人命名空间
/// \ingroup aris
/// 

namespace rokae
{
	//电缸力检测参数声明
	//const std::string xmlpath = "C:\\Users\\kevin\\Desktop\\aris_rokae\\ServoPressorCmdList.xml";
	constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//
	
	//机器人力控参数声明
	constexpr double f_static[6] = { 0.116994475,0.139070885,0.057812486,0.04834123,0.032697209,0.03668566 };
	constexpr double f_vel[6] = { 0.091826484,0.189104972,0.090449316,0.044415268,0.015864525,0.007350605 };
	constexpr double f_acc[6] = { 0.011658463,0.044943276,0.005936147,0.002210092,0.000618672,0.000664163 };
	constexpr double f2c_index = 5378.33927639823, max_static_vel = 0.1, f_static_index = 0.8;
	
	//其他参数和函数声明
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
}


#endif