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

// \brief 机器人命名空间
// \ingroup aris
//

namespace rokae
{
	//电缸力检测参数声明
	//const std::string xmlpath = "C:\\Users\\kevin\\Desktop\\aris_rokae\\ServoPressorCmdList.xml";
	constexpr double ea_a = 3765.8, ea_b = 1334.8, ea_c = 45.624, ea_gra = 24, ea_index = -6, ea_gra_index = 36;  //电缸电流换算压力的系数，ea_k表示比例系数，ea_b表示截距，ea_offset表示重力影响量，ea_index表示电流扭矩系数=额定扭矩*6.28*减速比/导程/1000//
	
	//机器人力控参数声明
	constexpr double f_static[6] = { 9.349947583,11.64080253,4.770140543,3.631416685,2.58310847,1.783739862 };
	constexpr double f_vel[6] = { 7.80825641,13.26518528,7.856443575,3.354615249,1.419632126,0.319206404 };
	constexpr double f_acc[6] = { 0,3.555679326,0.344454603,0.148247716,0.048552673,0.033815455 };
    constexpr double f2c_index[6] = { 9.07327526291993, 9.07327526291993, 17.5690184835913, 39.0310903520972, 66.3992503259041, 107.566785527965 };
    constexpr double max_static_vel[6] = {0.12, 0.12, 0.1, 0.05, 0.05, 0.075};
    constexpr double f_static_index[6] = {0.5, 0.5, 0.5, 0.85, 0.95, 0.8};
    constexpr double fi_limit[6] = {15,10,10,10,5,5};
    constexpr double vt_limit = 0.05;
    constexpr double ft_limit = 10;
    constexpr double Mt_limit = 1;

	//其他参数和函数声明 
	using Size = std::size_t;
	constexpr double PI = 3.141592653589793;
	auto createModelRokaeXB4(const double *robot_pm = nullptr)->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>;
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
