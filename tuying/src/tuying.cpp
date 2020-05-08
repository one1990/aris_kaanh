#include <algorithm>
#include "tuying.h"
#include <array>
#include <stdlib.h>
#include <numeric>
#ifdef UNIX
#include <sys/time.h>
#endif // UNIX


using namespace aris::dynamic;
using namespace aris::plan;


extern std::atomic_int syn_clock;
extern std::mutex dynamixel_mutex;
extern std::atomic_int mode_dynamixel;
extern std::atomic_int dynamixel_control_mode;
extern std::atomic_int is_enabled;
extern std::atomic_int16_t target_pos1, target_pos2, target_pos3;
extern bool dxl1_active, dxl2_active, dxl3_active;
extern std::atomic_int16_t current_pos1, current_pos2, current_pos3;
extern std::vector<std::vector<double>> dxl_pos;
extern std::atomic_bool dxl_connected;	//0:未连接，1:连接
extern std::atomic_bool dxl_auto;		//0:手动运行中，1:自动运行中
extern std::atomic_bool dxl_enabled;	//0:未使能，1:使能----第一版
extern std::atomic_int dxl_normal;		//0:异常，1:正常----第一版
extern std::atomic_int dxl1_state;
extern std::atomic_int dxl2_state;
extern std::atomic_int dxl3_state;
std::atomic<std::array<double, 10> > save_point;
std::atomic_int xbox_mode = 0;			//0:未指定，1:关节，2:末端，3:舵机
extern const int dxl_timeinterval;		//舵机时间系数

//tuying::CmdListParam cmdparam;

extern kaanh::Speed g_vel;

namespace tuying
{	
	auto createControllerQifan()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

		for (aris::Size i = 0; i < 6; ++i)
		{
#ifdef WIN32
			double pos_offset[6]
			{
				0.0,   0.0,   0.0,   0.0,   0.0,   0.0
			};
#endif // WIN32
#ifdef UNIX
			double pos_offset[6]
			{
                //-0.438460099997905,   1.01949192131931,   -1.00835441988747,   0.0315385382644258,   0.0943992339950059,   5.32826577904888
                -0.438460099997905,   1.01949192131931,   -0.97083,   0.0315385382644258+2.747*3.14159/180.0,   0.0943992339950059+0.673*3.14159/180.0,   5.32826577904888+1.129*3.14159/180.0
			};
#endif
			double pos_factor[6]
			{
                8388608.0 * 166 / 2 / PI, -8388608.0 * 166 / 2 / PI, 8388608.0 * 88 / 2 / PI, 8388608.0 * 80 / 2 / PI, 8388608.0 * 80 / 2 / PI, -8388608.0 * 80 / 2 / PI
			};
			double max_pos[6]
			{
				145.0 / 360 * 2 * PI, 110.0 / 360 * 2 * PI,	55.0 / 360 * 2 * PI, 122.0 / 360 * 2 * PI, 135.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-145.0 / 360 * 2 * PI, -60.0 / 360 * 2 * PI, -55.0 / 360 * 2 * PI, -122.0 / 360 * 2 * PI, -135.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				73.0 / 360 * 2 * PI, 73.0 / 360 * 2 * PI, 136.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI, 225.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				350.0 / 360 * 2 * PI, 350.0 / 360 * 2 * PI, 700.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI, 1100.0 / 360 * 2 * PI,
			};

			if (i == 0)
			{
				std::string xml_str =
                    "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
                    "</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
			else if (i == 1)
			{
				std::string xml_str =
                    "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380009\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
                    "</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
			else if (i == 2)
			{
				std::string xml_str =
                    "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380008\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
                    "</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
			else
			{
				std::string xml_str =
                    "<EthercatMotor phy_id=\"" + std::to_string(i) + "\" product_code=\"0x60380006\""
					" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
					" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
					" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
					" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
					"	<SyncManagerPoolObject>"
					"		<SyncManager is_tx=\"false\"/>"
					"		<SyncManager is_tx=\"true\"/>"
					"		<SyncManager is_tx=\"false\">"
					"			<Pdo index=\"0x1600\" is_tx=\"false\">"
					"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"		<SyncManager is_tx=\"true\">"
					"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
					"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
					"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
					"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
					"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
					"			</Pdo>"
					"		</SyncManager>"
					"	</SyncManagerPoolObject>"
                    "</EthercatMotor>";

				controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);
			}
        }

        double pos_offset = -0.336;	//在零位时，为500count
		double pos_factor = 8388608.0 * 16.07;	//电机运行一圈，导轨运行62.22mm，即导轨运行1m需要转16.07转
        double max_pos = 1.51;
        double min_pos = 0;
        double max_vel = 0.5;	//0.1m/s
        double max_acc = 1.0;	//1.0m/s2
		std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(6) + "\" product_code=\"0x6038000D\""
			" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
			" min_pos=\"" + std::to_string(min_pos) + "\" max_pos=\"" + std::to_string(max_pos) + "\" max_vel=\"" + std::to_string(max_vel) + "\" min_vel=\"" + std::to_string(-max_vel) + "\""
			" max_acc=\"" + std::to_string(max_acc) + "\" min_acc=\"" + std::to_string(-max_acc) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
			" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor) + "\" pos_offset=\"" + std::to_string(pos_offset) + "\">"
			"	<SyncManagerPoolObject>"
			"		<SyncManager is_tx=\"false\"/>"
			"		<SyncManager is_tx=\"true\"/>"
			"		<SyncManager is_tx=\"false\">"
			"			<Pdo index=\"0x1600\" is_tx=\"false\">"
			"				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"touch_probe\" index=\"0x60B8\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"targer_tor\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_tor\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"		<SyncManager is_tx=\"true\">"
			"			<Pdo index=\"0x1a01\" is_tx=\"true\">"
			"				<PdoEntry name=\"error_code\" index=\"0x603F\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
			"				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"vel_actual_value\" index=\"0x606C\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"touch_probe_status\" index=\"0x60b9\" subindex=\"0x00\" size=\"16\"/>"
			"				<PdoEntry name=\"touch_probe_pos1\" index=\"0x60ba\" subindex=\"0x00\" size=\"32\"/>"
			"				<PdoEntry name=\"digital_input\" index=\"0x60fd\" subindex=\"0x00\" size=\"32\"/>"
			"			</Pdo>"
			"		</SyncManager>"
			"	</SyncManagerPoolObject>"
            "</EthercatMotor>";
		controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);

        return controller;
	};
	auto createModelQifan()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
		
		
		param.d1 = 0.596;
		param.a1 = 0.220;
		param.a2 = 1.020;
		param.d3 = 0.0;
		param.a3 = 0.0;
		param.d4 = 0.86;
		//param.d4 = 1.010;

		param.tool0_pe[2] = 0.153;

		auto model = aris::dynamic::createModelPuma(param);

		return std::move(model);
	}


#define CHECK_PARAM_STRING \
		"		<UniqueParam default=\"check_all\">" \
		"			<Param name=\"check_all\"/>" \
		"			<Param name=\"check_none\"/>" \
		"			<GroupParam>"\
		"				<UniqueParam default=\"check_enable\">"\
		"					<Param name=\"check_enable\"/>"\
		"					<Param name=\"not_check_enable\"/>"\
		"				</UniqueParam>"\
		"				<UniqueParam default=\"check_pos\">"\
		"					<Param name=\"check_pos\"/>"\
		"					<Param name=\"not_check_pos\"/>"\
		"					<GroupParam>"\
		"						<UniqueParam default=\"check_pos_max\">"\
		"							<Param name=\"check_pos_max\"/>"\
		"							<Param name=\"not_check_pos_max\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_min\">"\
		"							<Param name=\"check_pos_min\"/>"\
		"							<Param name=\"not_check_pos_min\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous\">"\
		"							<Param name=\"check_pos_continuous\"/>"\
		"							<Param name=\"not_check_pos_continuous\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous_second_order\">"\
		"							<Param name=\"check_pos_continuous_second_order\"/>"\
		"							<Param name=\"not_check_pos_continuous_second_order\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_following_error\">"\
		"							<Param name=\"check_pos_following_error\"/>"\
		"							<Param name=\"not_check_pos_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"				<UniqueParam default=\"check_vel\">"\
		"					<Param name=\"check_vel\"/>"\
		"					<Param name=\"not_check_vel\"/>"\
		"					<GroupParam>"\
		"						<UniqueParam default=\"check_vel_max\">"\
		"							<Param name=\"check_vel_max\"/>"\
		"							<Param name=\"not_check_vel_max\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_min\">"\
		"							<Param name=\"check_vel_min\"/>"\
		"							<Param name=\"not_check_vel_min\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_continuous\">"\
		"							<Param name=\"check_vel_continuous\"/>"\
		"							<Param name=\"not_check_vel_continuous\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_following_error\">"\
		"							<Param name=\"check_vel_following_error\"/>"\
		"							<Param name=\"not_check_vel_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"			</GroupParam>"\
		"		</UniqueParam>"
	auto set_check_option(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan)->void
	{
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "check_all")
			{
				for (auto &option : plan.motorOptions())	option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				for (auto &option : plan.motorOptions())	option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_enable")
			{
				for (auto &option : plan.motorOptions()) option &= ~(
					Plan::NOT_CHECK_ENABLE);
			}
			else if (cmd_param.first == "not_check_enable")
			{
				for (auto &option : plan.motorOptions()) option |=
					Plan::NOT_CHECK_ENABLE;
			}
			else if (cmd_param.first == "check_pos")
			{
				for (auto &option : plan.motorOptions()) option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_pos")
			{
				for (auto &option : plan.motorOptions()) option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel")
			{
				for (auto &option : plan.motorOptions()) option &= ~(
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_vel")
			{
				for (auto &option : plan.motorOptions()) option |=
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_pos_min")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "not_check_pos_min")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "check_pos_max")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "not_check_pos_max")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "check_pos_continuous")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_pos_continuous")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "check_pos_following_error")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_pos_following_error")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel_min")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "not_check_vel_min")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "check_vel_max")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "not_check_vel_max")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "check_vel_continuous")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_vel_continuous")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "check_vel_following_error")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_vel_following_error")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
		}
	}

	struct SetActiveMotor { std::vector<int> active_motor; };
#define SELECT_MOTOR_STRING \
		"		<UniqueParam default=\"all\">"\
		"			<Param name=\"all\" abbreviation=\"a\"/>"\
		"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"\
		"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"\
		"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"\
		"		</UniqueParam>"
	auto set_active_motor(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan, SetActiveMotor &param)->void
	{
		param.active_motor.clear();
		param.active_motor.resize(plan.controller()->motionPool().size(), 0);

		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "all")
			{
				std::fill(param.active_motor.begin(), param.active_motor.end(), 1);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.at(plan.int32Param(cmd_param.first)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.at(plan.controller()->motionAtPhy(plan.int32Param(cmd_param.first)).motId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.at(plan.controller()->motionAtSla(plan.int32Param(cmd_param.first)).motId()) = 1;
			}
		}
	}


	struct EnableParam :public SetActiveMotor { std::int32_t limit_time; };
	struct Enable::Imp { Imp() {} };
	auto Enable::prepareNrt()->void
	{
		EnableParam param;
        is_enabled.store(1);            //舵机使能
        std::cout << "dynamixel enabled" <<std::endl;

		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, param);
		param.limit_time = int32Param("limit_time");

		this->param() = param;
		for (auto &option : motorOptions()) option |= aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Enable::executeRT()->int
	{
		auto &param = std::any_cast<EnableParam &>(this->param());

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = controller()->motionPool().at(i);
				auto ret = cm.enable();
				if (ret)
				{
					is_all_finished = false;

					if (count() % 1000 == 0)
					{
						controller()->mout() << "Unenabled motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (count() < param.limit_time ? 1 : aris::plan::Plan::PLAN_OVER_TIME);
	}
	Enable::~Enable() = default;
	Enable::Enable(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"en\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Enable);


	struct DisableParam :public SetActiveMotor { std::int32_t limit_time; };
	struct Disable::Imp { Imp() {} };
	auto Disable::prepareNrt()->void
	{
		DisableParam param;
		is_enabled.store(0);	//舵机去使能

		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, param);
		param.limit_time = int32Param("limit_time");

		this->param() = param;
		for (auto &option : motorOptions()) option |= aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Disable::executeRT()->int
	{
		auto &param = std::any_cast<DisableParam &>(this->param());

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = controller()->motionPool().at(i);
				auto ret = cm.disable();
				if (ret)
				{
					is_all_finished = false;

					if (count() % 1000 == 0)
					{
						controller()->mout() << "Undisabled motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (count() < param.limit_time ? 1 : aris::plan::Plan::PLAN_OVER_TIME);
	}
	Disable::~Disable() = default;
	Disable::Disable(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"ds\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			SELECT_MOTOR_STRING
			CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Disable);


	struct GetParam
	{
		std::vector<double> part_pq, end_pq, end_pe, motion_pos, motion_vel, motion_acc, motion_toq, ai;
		std::vector<bool> di;
		std::int32_t state_code;
		aris::control::EthercatController::SlaveLinkState sls[6];
		aris::control::EthercatController::MasterLinkState mls{};
		std::vector<int> motion_state;
		std::string currentplan;
		int vel_percent;
	};
	auto Get::prepareNrt()->void
	{
		GetParam par;
		par.part_pq.resize(model()->partPool().size() * 7, 0.0);
		par.end_pq.resize(7, 0.0);
		par.end_pe.resize(6, 0.0);
		par.motion_pos.resize(10, 0.0);
		par.motion_vel.resize(controller()->motionPool().size(), 0.0);
		par.motion_acc.resize(controller()->motionPool().size(), 0.0);
		par.motion_toq.resize(controller()->motionPool().size(), 0.0);
		par.ai.resize(100, 1.0);
		par.di.resize(100, false);
		par.motion_state.resize(controller()->motionPool().size(), 0);
		std::any param = par;
		//std::any param = std::make_any<GetParam>();

		controlServer()->getRtData([&](aris::server::ControlServer& cs, const aris::plan::Plan *target, std::any& data)->void
		{
            kaanh::update_state(cs);

			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
			{
				cs.model().partPool().at(i).getPq(std::any_cast<GetParam &>(data).part_pq.data() + i * 7);
			}

			for (aris::Size i(0); i < cs.model().generalMotionPool().size(); i++)
			{
				cs.model().generalMotionPool().at(0).getMpq(std::any_cast<GetParam &>(data).end_pq.data());
				cs.model().generalMotionPool().at(0).getMpe(std::any_cast<GetParam &>(data).end_pe.data(), "321");
			}

			for (aris::Size i = 0; i < cs.controller().motionPool().size(); i++)
			{
#ifdef WIN32
				if (i < 6)
				{
					std::any_cast<GetParam &>(data).motion_pos[i] = cs.model().motionPool()[i].mp();
					std::any_cast<GetParam &>(data).motion_vel[i] = cs.model().motionPool()[i].mv();
					std::any_cast<GetParam &>(data).motion_acc[i] = cs.model().motionPool()[i].ma();
					std::any_cast<GetParam &>(data).motion_toq[i] = cs.model().motionPool()[i].ma();
				}
				else
				{
					std::any_cast<GetParam &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
					std::any_cast<GetParam &>(data).motion_vel[i] = cs.controller().motionPool()[i].actualVel();
					std::any_cast<GetParam &>(data).motion_acc[i] = cs.controller().motionPool()[i].actualToq();
					std::any_cast<GetParam &>(data).motion_toq[i] = cs.controller().motionPool()[i].actualToq();
				}
#endif // WIN32

#ifdef UNIX
				if (i < 6)
				{
					std::any_cast<GetParam &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
					std::any_cast<GetParam &>(data).motion_vel[i] = cs.controller().motionPool()[i].actualVel();
					std::any_cast<GetParam &>(data).motion_acc[i] = cs.model().motionPool()[i].ma();
					std::any_cast<GetParam &>(data).motion_toq[i] = cs.controller().motionPool()[i].actualToq();
				}
				else
				{
					std::any_cast<GetParam &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
					std::any_cast<GetParam &>(data).motion_vel[i] = cs.controller().motionPool()[i].actualVel();
					std::any_cast<GetParam &>(data).motion_acc[i] = cs.controller().motionPool()[i].actualToq();
					std::any_cast<GetParam &>(data).motion_toq[i] = cs.controller().motionPool()[i].actualToq();
				}
#endif // UNIX
			}

			for (aris::Size i = 0; i < 100; i++)
			{
				std::any_cast<GetParam &>(data).ai[i] = 1.0;
				std::any_cast<GetParam &>(data).di[i] = false;
			}

			auto ec = dynamic_cast<aris::control::EthercatController*>(&cs.controller());
			ec->getLinkState(&std::any_cast<GetParam &>(data).mls, std::any_cast<GetParam &>(data).sls);

			//获取motion的使能状态，0表示去使能状态，1表示使能状态//
			for (aris::Size i = 0; i < cs.controller().motionPool().size(); i++)
			{
				auto cm = dynamic_cast<aris::control::EthercatMotor*>(&cs.controller().motionPool()[i]);
				if ((cm->statusWord() & 0x6f) != 0x27)
				{
					std::any_cast<GetParam &>(data).motion_state[i] = 0;
				}
				else
				{
					std::any_cast<GetParam &>(data).motion_state[i] = 1;
				}
			}

			if (target == nullptr)
			{
				std::any_cast<GetParam &>(data).currentplan = "none";
			}
			else
			{
				std::any_cast<GetParam &>(data).currentplan = target->command().name();
			}

		}, param);

		auto out_data = std::any_cast<GetParam &>(param);
		std::vector<int> slave_online(controller()->motionPool().size(), 0), slave_al_state(controller()->motionPool().size(), 0);
		for (aris::Size i = 0; i < controller()->motionPool().size(); i++)
		{
			slave_online[i] = int(out_data.sls[i].online);
			slave_al_state[i] = int(out_data.sls[i].al_state);
		}

        out_data.state_code = kaanh::get_state_code();

		//舵机//
		out_data.motion_pos[7] = current_pos1.load() * 10;
		out_data.motion_pos[8] = current_pos2.load() * 10;
		out_data.motion_pos[9] = current_pos3.load() * 10;
		//out_data.motion_pos[7] = target_pos1.load()*10;
		//out_data.motion_pos[8] = target_pos2.load()*10;
		//out_data.motion_pos[9] = target_pos3.load()*10;

		auto &cs = *controlServer();
		auto &inter = dynamic_cast<aris::server::ProgramWebInterface&>(cs.interfacePool().at(0));

		std::vector<std::pair<std::string, std::any>> out_param;

        out_param.push_back(std::make_pair<std::string, std::any>("part_pq", out_data.part_pq));
        out_param.push_back(std::make_pair<std::string, std::any>("end_pq", out_data.end_pq));
        out_param.push_back(std::make_pair<std::string, std::any>("end_pe", out_data.end_pe));
        out_param.push_back(std::make_pair<std::string, std::any>("motion_pos", out_data.motion_pos));
        out_param.push_back(std::make_pair<std::string, std::any>("motion_vel", out_data.motion_vel));
        out_param.push_back(std::make_pair<std::string, std::any>("motion_acc", out_data.motion_acc));
        out_param.push_back(std::make_pair<std::string, std::any>("motion_toq", out_data.motion_toq));
        out_param.push_back(std::make_pair<std::string, std::any>("ai", out_data.ai));
        out_param.push_back(std::make_pair<std::string, std::any>("di", out_data.di));
        out_param.push_back(std::make_pair<std::string, std::any>("state_code", out_data.state_code));
        out_param.push_back(std::make_pair<std::string, std::any>("slave_link_num", std::int32_t(out_data.mls.slaves_responding)));
        out_param.push_back(std::make_pair<std::string, std::any>("slave_online_state", slave_online));
        out_param.push_back(std::make_pair<std::string, std::any>("slave_al_state", slave_al_state));
        out_param.push_back(std::make_pair<std::string, std::any>("motion_state", out_data.motion_state));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl_connected", dxl_connected.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl_auto", dxl_auto.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl_enabled", dxl_enabled.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl_normal", dxl_normal.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl1_state", dxl1_state.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl2_state", dxl2_state.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("dxl3_state", dxl3_state.load()));
        out_param.push_back(std::make_pair<std::string, std::any>("xbox_mode", xbox_mode.load()));
        out_param.push_back(std::make_pair(std::string("cs_err_code"), std::make_any<int>(cs.errorCode())));
        out_param.push_back(std::make_pair(std::string("cs_err_msg"), std::make_any<std::string>(cs.errorMsg())));
        out_param.push_back(std::make_pair(std::string("pro_err_code"), std::make_any<int>(inter.lastErrorCode())));
        out_param.push_back(std::make_pair(std::string("pro_err_msg"), std::make_any<std::string>(inter.lastError())));
        out_param.push_back(std::make_pair(std::string("pro_err_line"), std::make_any<int>(inter.lastErrorLine())));
        out_param.push_back(std::make_pair(std::string("line"), std::make_any<int>(inter.currentLine())));


		std::array<double, 10> temp = { 0,0,0,0,0,0,0,0,0,0 };
		std::copy(out_data.motion_pos.begin(), out_data.motion_pos.end(), temp.begin());
		save_point.store(temp);

		ret() = out_param;
		option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
	}
	auto Get::collectNrt()->void {}
	Get::Get(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get\">"
			"</Command>");
	}


    // 获取pos等 //
    struct Get1Param
    {
        std::vector<double> motion_pos;
    };
    auto Getp::prepareNrt()->void
    {
        Get1Param par;
        par.motion_pos.resize(10, 0.0);
        std::any param = par;
        //std::any param = std::make_any<GetParam>();
        int motion_num = 7;
        controlServer()->getRtData([&](aris::server::ControlServer& cs, const aris::plan::Plan *target, std::any& data)->void
        {
            for (aris::Size i = 0; i < cs.controller().motionPool().size(); i++)
            {
#ifdef WIN32
                if (i < 6)
                {
                    std::any_cast<Get1Param &>(data).motion_pos[i] = cs.model().motionPool()[i].mp();
                }
                else
                {
                    std::any_cast<Get1Param &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
                }
#endif // WIN32

#ifdef UNIX
                if (i < 6)
                {
                    std::any_cast<Get1Param &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
                }
                else
                {
                    std::any_cast<Get1Param &>(data).motion_pos[i] = cs.controller().motionPool()[i].actualPos();
                }
#endif // UNIX
            }

        }, param);

        auto out_data = std::any_cast<Get1Param &>(param);

        //舵机//
        out_data.motion_pos[7] = current_pos1.load()*10;
        out_data.motion_pos[8] = current_pos2.load()*10;
        out_data.motion_pos[9] = current_pos3.load()*10;
        //out_data.motion_pos[7] = target_pos1.load()*10;
        //out_data.motion_pos[8] = target_pos2.load()*10;
        //out_data.motion_pos[9] = target_pos3.load()*10;

        std::vector<std::pair<std::string, std::any>> out_param;
        out_param.push_back(std::make_pair<std::string, std::any>("motion_pos", out_data.motion_pos));

        std::array<double, 10> temp = { 0,0,0,0,0,0,0,0,0,0 };
        std::copy(out_data.motion_pos.begin(), out_data.motion_pos.end(), temp.begin());
        save_point.store(temp);

        ret() = out_param;
        option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
    }
    auto Getp::collectNrt()->void {}
    Getp::Getp(const std::string &name) : Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"getp\">"
            "</Command>");
    }


	// 执行emily文件 //
	auto interpolate(double x1, double x2, double y1, double y2, double x)->double
	{
		double y;
		if (abs(x2 - x1) < 1e-6)
		{
			y = x2;
		}
		else
		{
			y = y1 + (x - x1)*(y2 - y1) / (x2 - x1);
		}
		return y;
	}
	struct MoveTParam
	{
		std::vector<std::vector<double>> pos;
		std::vector<double> begin_pos;
		std::int16_t col;
		double ratio;
	};
	auto MoveT::prepareNrt()->void
	{
		//当前有指令在执行//
		auto&cs = aris::server::ControlServer::instance();
		std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();
		if (planptr && planptr->cmdName() == this->cmdName())
		{
			option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
			return;
		}

		auto c = controller();
		MoveTParam param;
		param.pos.clear();
		param.begin_pos.clear();
		std::ifstream infile;

		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "col")
			{
				param.col = int32Param(cmd_param.first);
				param.pos.resize(7);
				param.begin_pos.resize(param.col - 1);
			}
			else if (cmd_param.first == "path")
			{
				std::unique_lock<std::mutex> run_lock(dynamixel_mutex);
				dxl_pos.clear();
				dxl_pos.resize(3);
				std::vector<std::vector<double>> pos(param.col);
                auto path = std::string(cmd_param.second);
				infile.open(path);
				//检查读取文件是否成功//
				if (!infile)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " fail to open the file");

				std::string data;
				while(std::getline(infile, data))
				{
					if (data.find("[HEADER]") != std::string::npos)
					{
						continue;
					}
					if (data.find("GEAR_NOMINAL_VEL") != std::string::npos)
					{
						char *s_vel = (char *)data.c_str();
						const char *split = "=";
						// 以‘=’为分隔符拆分字符串
						char *sp_vel = strtok(s_vel, split);
						sp_vel = strtok(NULL, split);
						std::string vel = sp_vel;
						param.ratio = std::stod(vel) / 1.0;
						std::cout << "ratio:" << param.ratio << std::endl;
						continue;
					}
					if (data.find("[RECORDS]") != std::string::npos)
					{
						continue;
					}
					if (data.find("[END]") != std::string::npos)
					{
						break;
					}
						
					char *s_input = (char *)data.c_str();
					const char *split = "   ";
					// 以‘ ’为分隔符拆分字符串
					char *sp_input = strtok(s_input, split);
					double data_input;
					int i = 0;
					int j = 0;
					while (sp_input != NULL)
					{
						data_input = atof(sp_input);
						//前7轴的位置//
						if (i <= 6)
						{
							pos[i++].push_back(data_input);
							sp_input = strtok(NULL, split);
							continue;
						}
						if (i > 6)
						{
							dxl_pos[j++].push_back(data_input);
							i++;
							sp_input = strtok(NULL, split);
							continue;
						}
						if (i >= param.col)THROW_FILE_LINE("the format of emily file is not ok");
					}
				}
				for (int i = 0; i < pos.size(); i++)
				{
					for (int j = 0; j < pos[0].size() - 1; j++)
					{
						for (double count = pos[0][j]; count < pos[0][j + 1]; count = count + 0.001*param.ratio)
						{
							param.pos[i].push_back(interpolate(pos[0][j], pos[0][j + 1], pos[i][j], pos[i][j + 1], count)* PI/180.0);
						}
					}
				}
				infile.close();
			}
		}
		this->param() = param;

		//std::fill(motorOptions().begin(), motorOptions().end(), Plan::USE_TARGET_POS);
		for (int j = 0; j < param.pos.size(); j++)
		{
			for (int i = 0; i < 13; i++)
			{
				std::cout << param.pos[j][i] << "  ";
			}
			std::cout << std::endl;
		}

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;

		// 使能舵机emily功能 //
		dynamixel_control_mode.store(3);
	}
	auto MoveT::executeRT()->int
	{
		auto &param = std::any_cast<MoveTParam&>(this->param());
		

		for (int i = 0; i < controller()->motionPool().size(); i++)
		{
			controller()->motionPool()[i].setTargetPos(param.pos[i + 1][count()]);
		}
		for (int i = 0; i < model()->motionPool().size(); i++)
		{
			model()->motionPool().at(i).setMp(param.pos[i + 1][count()]);
		}
        if (model()->solverPool().at(1).kinPos())return -1;
			   
		// 打印 //
		auto &cout = controller()->mout();
        if (count() % 100 == 0)
		{
			for (Size i = 0; i < controller()->motionPool().size(); i++)
			{
				//cout << model()->motionPool().at(i).mp() << "  ";
				cout << controller()->motionPool()[i].actualPos() << "  ";
			}
			cout << std::endl;
		}
		// log //
		auto &lout = controller()->lout();
		for (Size i = 0; i < controller()->motionPool().size(); ++i)
		{
			for (Size i = 0; i < controller()->motionPool().size(); i++)
			{
				cout << controller()->motionPool()[i].actualPos() << "  ";
			}
			cout << std::endl;
		}    

		return param.pos[0].size() - count() - 1;
	}
	auto MoveT::collectNrt()->void {}
	MoveT::MoveT(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
            "<Command name=\"movet\">"
			"	<GroupParam>"
			"		<Param name=\"col\" default=\"9\"/>"
			"		<Param name=\"path\" default=\"C:\\Users\\kevin\\Desktop\\tuying\\tuying\\output.emily\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 执行emily文件 //
	struct MoveEParam
	{
		std::vector<std::vector<double>> pos;
		std::vector<std::vector<double>> target_pos;
		std::vector<std::vector<double>> temp_pos;
		std::vector<bool> active;
		double ratio;
		static std::atomic_int32_t mve_flag;
		//mve指令标志位：0:加载emily文件，1:前进，2:后退，3:替换当前emily数据点，4:保存更新emily文件，5:开始，6:暂停，7:退出
	};
	bool splitString(std::string spCharacter, const std::string& objString, std::vector<bool>& stringVector)
	{
		if (objString.length() == 0)
		{
			THROW_FILE_LINE("the ACTIVE_AXIS is NULL");
			return false;
		}
		size_t posBegin = 0;
		size_t posEnd = 0;

		while (posEnd != std::string::npos)
		{
			posBegin = posEnd;
			posEnd = objString.find(spCharacter, posBegin);

			if (posBegin == posEnd)
			{
				posEnd += spCharacter.size();
				continue;
			}
			if (posEnd == std::string::npos)
			{
				break;
			}
			std::string str = objString.substr(posBegin, posEnd - posBegin);
			stringVector.push_back(std::stoi(str));
			posEnd += spCharacter.size();
		}
		return true;
	}
	std::atomic_int32_t MoveEParam::mve_flag = 0;
	auto MoveE::prepareNrt()->void
	{
		MoveEParam param;
		auto&cs = aris::server::ControlServer::instance();
		std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();

		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "file")
			{
				//当前有指令在执行//
				if (planptr && planptr->cmdName() != this->cmdName())throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");
				if (planptr && planptr->cmdName() == this->cmdName())
				{
					option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
					return;
				}
				param.active.clear();
				param.pos.clear();
				param.target_pos.clear();
				param.temp_pos.clear();
				param.target_pos.resize(7);
				std::ifstream infile;
				param.mve_flag.store(0);//加载emily文件
				std::unique_lock<std::mutex> run_lock(dynamixel_mutex);
				dxl_pos.clear();
				dxl_pos.resize(3);
#ifdef WIN32
				auto path = "D:/tuying/emily/" + std::string(cmd_param.second);
#endif
#ifdef UNIX
				auto path = "/home/kaanh/Desktop/emily/" + std::string(cmd_param.second);
#endif

				infile.open(path);

				//检查读取文件是否成功//
				if (!infile)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " fail to open the file");
				//解析log文件//
				std::string data;

				while (std::getline(infile, data))
				{
					if (data.find("[HEADER]") != std::string::npos)
					{
						continue;
					}
					if (data.find("GEAR_NOMINAL_VEL") != std::string::npos)
					{
						std::string split = "=";
						// 以‘=’为分隔符拆分字符串
						auto vel_pos = data.find(split);
						auto sp_vel = data.substr(vel_pos + 1);
						std::string vel = sp_vel;
						param.ratio = std::stod(vel) / 1.0;
						std::cout << "GEAR_NOMINAL_VEL:" << param.ratio << std::endl;
						continue;
					}
					if (data.find("ACTIVE_AXIS") != std::string::npos)
					{
						std::string split = "=";
						// 以‘= ’为分隔符拆分字符串
						auto axis_pos = data.find(split);
						auto axis_string = data.substr(axis_pos + 1);
						std::cout << "ACTIVE_AXIS:" << axis_string << std::endl;
						std::string axis_split = ",";
						splitString(axis_split, axis_string, param.active);
						dxl1_active = param.active[7];
						dxl2_active = param.active[8];
						dxl3_active = param.active[9];
						param.pos.resize(param.active.size());//size of ACTIVE_AXIS
						continue;
					}
					if (data.find("[RECORDS]") != std::string::npos)
					{
						continue;
					}
					if (data.find("[END]") != std::string::npos)
					{
						break;
					}
					param.temp_pos.resize(param.active.size() + 1);
					char *s_input = (char *)data.c_str();
					const char *split = "  ";
					// 以‘ ’为分隔符拆分字符串//
					char *sp_input = strtok(s_input, split);

					double data_input;
					int i = 0;
					while (sp_input != NULL)
					{
						data_input = atof(sp_input);
						param.temp_pos[i++].push_back(data_input);
						sp_input = strtok(NULL, split);
						continue;
					}
					if (s_input != NULL)
					{
						s_input = NULL;
					}
					delete(sp_input);
					sp_input = NULL;
				}

				//数据提取//
				int pos_count = 1;
				for (int k = 0; k < param.active.size(); k++)
				{
					param.pos[k].assign(param.temp_pos[pos_count].begin(), param.temp_pos[pos_count].end());
					pos_count++;
				}

				//如果没有提取到数据，不执行程序，并返回
				int16_t judge_flag = 0;
				for (int k = 0; k < param.active.size(); k++)
				{
					judge_flag = judge_flag + param.pos[k].size();
				}
				if (judge_flag == 0)
				{
					option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " fail to get emily data");
					return;
				}

				//数据处理---导轨数值除以1000，单位由mm换算成m
				if (param.active[6])
				{
					for (int m = 0; m < param.pos[6].size(); m++)
					{
						param.pos[6][m] = (param.pos[6][m]) / 1000.0;
					}
				}

				//数据处理---舵机数据单位是0.1度，除以10并乘以11.378，换算成脉冲数
				for (int j = 7; j < param.active.size(); j++)
				{
					if (param.active[j])
					{
						for (int m = 0; m < param.pos[j].size(); m++)
						{
							param.pos[j][m] = param.pos[j][m] / 10.0*11.378;
						}
					}
					else
					{
						continue;
					}
				}

				//数据处理--对机械臂,外部轴,duoji数据进行插值//
				int dxl_count = 0;
				for (int i = 0; i < param.pos.size(); i++)
				{
					if (i < 6)
					{
						if (!param.pos[i].empty())
						{
							for (int j = 0; j < param.pos[i].size() - 1; j++)
							{
								for (double count = param.temp_pos[0][j]; count < param.temp_pos[0][j + 1] - 0.2*0.001*param.ratio; count = count + 0.001*param.ratio)
								{
									param.target_pos[i].push_back(interpolate(param.temp_pos[0][j], param.temp_pos[0][j + 1], param.pos[i][j], param.pos[i][j + 1], count)* PI / 180.0);
								}
							}
						}
					}
					else if (i == 6)
					{
						if (!param.pos[i].empty())
						{
							for (int j = 0; j < param.pos[i].size() - 1; j++)
							{
								for (double count = param.temp_pos[0][j]; count < param.temp_pos[0][j + 1] - 0.2*0.001*param.ratio; count = count + 0.001*param.ratio)
								{
									param.target_pos[i].push_back(interpolate(param.temp_pos[0][j], param.temp_pos[0][j + 1], param.pos[i][j], param.pos[i][j + 1], count));
								}
							}
						}
					}
					else
					{
						if (!param.pos[i].empty())
						{
							for (int j = 0; j < param.pos[i].size() - 1; j++)
							{
								for (double count = param.temp_pos[0][j]; count < param.temp_pos[0][j + 1] - 0.2*0.001*param.ratio; count = count + 0.001*param.ratio)
								{
									dxl_pos[dxl_count].push_back(interpolate(param.temp_pos[0][j], param.temp_pos[0][j + 1], param.pos[i][j], param.pos[i][j + 1], count));
								}
							}
						}
						dxl_count++;
					}
				}
				infile.close();
			}
			else if (cmd_param.first == "forward")
			{ 
				//当前有指令在执行//
				if (planptr && planptr->cmdName() != this->cmdName())throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");
				if (planptr && planptr->cmdName() == this->cmdName())
				{
					option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
					return;
				}
				param.mve_flag.store(1);
			}
			else if (cmd_param.first == "backward")
			{
				param.mve_flag.store(2);
			}
			else if (cmd_param.first == "replace")
			{
				param.mve_flag.store(3);
			}
			else if (cmd_param.first == "save")
			{
				param.mve_flag.store(4);
			}
			else if (cmd_param.first == "start")
			{
				param.mve_flag.store(5);
			}
			else if (cmd_param.first == "pause")
			{
				param.mve_flag.store(6);
			}
			else if (cmd_param.first == "quit")
			{
				param.mve_flag.store(7);
			}
		}
		//对机械臂及外部轴数据进行滑动滤波，窗口为11
		uint16_t window = 11;
		for (int i = 0; i < param.target_pos.size(); i++)
		{
			for (int j = 0; j < param.target_pos[0].size(); j++)
			{
				if (j < window)
				{
					param.target_pos[i][j] = std::accumulate(param.target_pos[i].begin(), param.target_pos[i].begin() + j + 1, 0.0) / (j + 1);
				}
				else
				{
					param.target_pos[i][j] = std::accumulate(param.target_pos[i].begin() + j - 10, param.target_pos[i].begin() + j + 1, 0.0) / window;
				}
			}
		}
		//对机械臂及外部轴数据进行滑动滤波，窗口为4
		uint16_t window2 = 4;
		for (int i = 0; i < param.target_pos.size(); i++)
		{
			for (int j = 0; j < param.target_pos[0].size(); j++)
			{
				if (j < window2)
				{
					param.target_pos[i][j] = std::accumulate(param.target_pos[i].begin(), param.target_pos[i].begin() + j + 1, 0.0) / (j + 1);
				}
				else
				{
					param.target_pos[i][j] = std::accumulate(param.target_pos[i].begin() + j - 3, param.target_pos[i].begin() + j + 1, 0.0) / window2;
				}
			}
		}

		for (int j = 0; j < param.target_pos.size(); j++)
		{
			std::cout << "前13行数据：" << std::endl;
			for (int i = 0; i < 13; i++)
			{
				std::cout << param.target_pos[j][i] << "  ";
			}
			std::cout << std::endl;
		}
        
		std::fill(motorOptions().begin(), motorOptions().end(), Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | Plan::NOT_CHECK_POS_CONTINUOUS);
		// 使能舵机emily功能 //
		dynamixel_control_mode.store(3);
		this->param() = param;
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveE::executeRT()->int
	{
		auto &param = std::any_cast<MoveEParam&>(this->param());
		
		static aris::Size total_count = 1;
		if (count() == 1)
		{
			syn_clock.store(0);
		}
		if (count() % 10 == 0)
		{
			syn_clock++;
		}

#ifdef WIN32
		for (int i = 0; i < model()->motionPool().size(); i++)
		{
			if (!param.target_pos[i].empty())
			{
				model()->motionPool().at(i).setMp(param.target_pos[i][count()-1]);
			}
			total_count = std::max(param.target_pos[i].size(), total_count);
		}
		if (model()->solverPool().at(1).kinPos())return -1;

		// 打印 //
		auto &cout = controller()->mout();
		if (count() % 1000 == 0)
		{
			for (Size i = 0; i < model()->motionPool().size(); i++)
			{
				cout << model()->motionPool().at(i).mp() << "  ";
			}
			cout << std::endl;
		}
		// log //
		auto &lout = controller()->lout();
		for (Size i = 0; i < model()->motionPool().size(); ++i)
		{
			for (Size i = 0; i < model()->motionPool().size(); i++)
			{
				lout << model()->motionPool().at(i).mp() << "  ";
			}
			lout << std::endl;
		}

#endif // WIN32

#ifdef UNIX
		//线性插值轨迹//
		for (int i = 0; i < controller()->motionPool().size(); i++)
		{
			if (!param.target_pos[i].empty())
			{
				controller()->motionPool()[i].setTargetPos(param.target_pos[i][count()-1]);
                if(i<6)
                {
                    model()->motionPool().at(i).setMp(param.target_pos[i][count()-1]);
                }
			}
			total_count = std::max(param.target_pos[i].size(), total_count);
		}
        if (model()->solverPool().at(1).kinPos())return -1;
		// 打印 //
		/*
		auto &cout = controller()->mout();
		if (count() % 100 == 0)
		{
			for (Size i = 0; i < controller()->motionPool().size(); i++)
			{
				cout << controller()->motionPool()[i].actualPos() << "  ";
			}
			cout << std::endl;
		}
		*/
		// log //
		/*
		auto &lout = controller()->lout();
		for (Size i = 0; i < controller()->motionPool().size(); ++i)
		{
			for (Size i = 0; i < controller()->motionPool().size(); i++)
			{
				lout << controller()->motionPool()[i].actualPos() << "  ";
			}
			lout << std::endl;
		}
		*/
#endif // UNIX

		return total_count - count() - 1;
	}
	auto MoveE::collectNrt()->void {}
	MoveE::MoveE(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mve\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"file\">"
			"			<Param name=\"file\" default=\"output.emily\"/>"
			"			<Param name=\"forward\"/>"	//前进
			"			<Param name=\"backward\"/>"	//后退
			"			<Param name=\"replace\"/>"	//替换当前emily数据点
			"			<Param name=\"save\"/>"		//保存更新emily文件
			"			<Param name=\"start\"/>"	//开始
			"			<Param name=\"pause\"/>"	//暂停
			"			<Param name=\"quit\"/>"		//退出
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 执行emily文件 //
	struct MoveE0Param
	{
		std::vector<std::vector<double>> pos;
		std::vector<std::vector<double>> target_pos;
		std::vector<std::vector<double>> temp_pos;
		std::vector<bool> active;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		std::int16_t col;
		double ratio;
		double percent;
	};
	auto MoveE0::prepareNrt()->void
	{
		//当前有指令在执行//
		auto&cs = aris::server::ControlServer::instance();
		std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();
        if (planptr && planptr->cmdName() == this->cmdName())
		{
            option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
			return;
		}
		auto c = controller();
		double percent = 0.1; //速度、加速度系数
        int16_t motor_num = 7; //电机数量
		MoveE0Param param;
		param.active.clear();
		param.pos.clear();
		param.target_pos.clear();
		param.temp_pos.clear();
		param.target_pos.resize(motor_num);
		std::ifstream infile;

		param.axis_begin_pos_vec.resize(motor_num, 0.0);
		param.axis_vel_vec.resize(motor_num, 0.0);
		param.axis_acc_vec.resize(motor_num, 0.0);
		param.axis_dec_vec.resize(motor_num, 0.0);
		//设置最大速度、加速度、减速度
		for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
		{
			param.axis_acc_vec[i] = percent * c->motionPool()[i].maxAcc();
			param.axis_dec_vec[i] = percent * c->motionPool()[i].minAcc();
		}

		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "col")
			{
				param.col = int32Param(cmd_param.first);
			}
			else if (cmd_param.first == "file")
			{
				std::unique_lock<std::mutex> run_lock(dynamixel_mutex);
				dxl_pos.clear();
				dxl_pos.resize(3);
#ifdef WIN32
				auto path = "D:/tuying/emily/" + std::string(cmd_param.second);
#endif
#ifdef UNIX
				auto path = "/home/kaanh/Desktop/emily/" + std::string(cmd_param.second);
#endif

				infile.open(path);

				//检查读取文件是否成功//
				if (!infile)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " fail to open the file");
				//解析log文件//
				std::string data;

				while (std::getline(infile, data))
				{
					if (data.find("[HEADER]") != std::string::npos)
					{
						continue;
					}
					if (data.find("GEAR_NOMINAL_VEL") != std::string::npos)
					{
						std::string split = "=";
						// 以‘=’为分隔符拆分字符串
						auto vel_pos = data.find(split);
						auto sp_vel = data.substr(vel_pos + 1);
						std::string vel = sp_vel;
						param.ratio = std::stod(vel) / 1.0;
						std::cout << "GEAR_NOMINAL_VEL:" << param.ratio << std::endl;
						continue;
					}
					if (data.find("ACTIVE_AXIS") != std::string::npos)
					{
						std::string split = "=";
						// 以‘= ’为分隔符拆分字符串
						auto axis_pos = data.find(split);
						auto axis_string = data.substr(axis_pos + 1);
						std::cout << "ACTIVE_AXIS:" << axis_string << std::endl;
						std::string axis_split = ",";
						splitString(axis_split, axis_string, param.active);
						param.pos.resize(param.active.size());//size of ACTIVE_AXIS
						continue;
					}
					if (data.find("[RECORDS]") != std::string::npos)
					{
						continue;
					}
					if (data.find("[END]") != std::string::npos)
					{
						break;
					}
					param.temp_pos.resize(param.active.size() + 1);
					char *s_input = (char *)data.c_str();
					const char *split = "  ";
					// 以‘ ’为分隔符拆分字符串//
					char *sp_input = strtok(s_input, split);

					double data_input;
					int i = 0;
					while (sp_input != NULL)
					{
						data_input = atof(sp_input);
						param.temp_pos[i++].push_back(data_input);
						sp_input = strtok(NULL, split);
						continue;
					}
					if (s_input != NULL)
					{
						s_input = NULL;
					}
					delete(sp_input);
					sp_input = NULL;
				}

				//数据提取//
				int pos_count = 1;
				for (int k = 0; k < param.active.size(); k++)
				{
					param.pos[k].assign(param.temp_pos[pos_count].begin(), param.temp_pos[pos_count].end());
					pos_count++;
				}

				//如果没有提取到数据，不执行程序，并返回
				int16_t judge_flag = 0;
				for (int k = 0; k < param.active.size(); k++)
				{
					judge_flag = judge_flag + param.pos[k].size();
				}
				if (judge_flag == 0)
				{
					option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " fail to get emily data");
					return;
				}

				//数据处理---导轨数值除以1000，单位由mm换算成m
				if (param.active[6])
				{
					for (int m = 0; m < param.pos[6].size(); m++)
					{
						param.pos[6][m] = (param.pos[6][m]) / 1000.0;
					}
				}

				//数据处理---舵机数据单位是0.1度，除以10并乘以11.378，换算成脉冲数
				for (int j = 7; j < param.active.size(); j++)
				{
					if (param.active[j])
					{
						for (int m = 0; m < param.pos[j].size(); m++)
						{
							param.pos[j][m] = param.pos[j][m] / 10.0*11.378;
						}
					}
					else
					{
						continue;
					}
				}

				//提取舵机数据
				if (!param.pos[7].empty())
				{
					target_pos1.store(param.pos[7][0]);
				}
				if (!param.pos[8].empty())
				{
					target_pos2.store(param.pos[8][0]);
				}
				if (!param.pos[9].empty())
				{
					target_pos3.store(param.pos[9][0]);
				}

				//提取机械臂以及外部轴数据第一行数据//
				for (int i = 0; i < param.target_pos.size(); i++)
				{
					if (!param.pos[i].empty())
					{
						if (i < 6)
						{
							param.target_pos[i].push_back(param.pos[i][0] * PI / 180.0);
						}
						else
						{
							param.target_pos[i].push_back(param.pos[i][0]);
						}
					}
				}
				
				infile.close();
			}
			else if (cmd_param.first == "percent")
			{
				param.percent = doubleParam(cmd_param.first);
				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					param.axis_vel_vec[i] = param.percent * c->motionPool()[i].maxVel();
				}
			}
		}
		dynamixel_control_mode.store(1);
        std::fill(motorOptions().begin(), motorOptions().end(), Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|Plan::NOT_CHECK_POS_CONTINUOUS);
        this->param() = param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
	}
	auto MoveE0::executeRT()->int
	{
		auto &param = std::any_cast<MoveE0Param&>(this->param());
		
		static aris::Size total_count = 1;

		// 取得起始位置 //
		if (count() == 1)
		{
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller()->motionPool().at(i).actualPos();
			}
		}

#ifdef WIN32
		for (int i = 0; i < model()->motionPool().size(); i++)
		{
			double p, v, a;
			aris::Size tcount = 0;
			if (!param.target_pos[i].empty())
			{
				aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.target_pos[i][0], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, tcount);
				model()->motionPool().at(i).setMp(p);
				total_count = std::max(tcount, total_count);
			}
		}
		if (model()->solverPool().at(1).kinPos())return -1;

#endif // WIN32

#ifdef UNIX
		//线性插值轨迹//
		for (int i = 0; i < controller()->motionPool().size(); i++)
		{
            double p, v, a;
            aris::Size tcount = 0;
			if (!param.target_pos[i].empty())
			{
				aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.target_pos[i][0], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, tcount);
				controller()->motionPool().at(i).setTargetPos(p);
                if(i<6)
                {
                    model()->motionPool().at(i).setMp(p);
                }
				total_count = std::max(tcount, total_count);
			}
		}
        if (model()->solverPool().at(1).kinPos())return -1;
#endif // UNIX

		return total_count - count();
	}
	auto MoveE0::collectNrt()->void {}
    MoveE0::MoveE0(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mve0\">"
			"	<GroupParam>"
			"		<Param name=\"col\" default=\"9\"/>"
            "		<Param name=\"file\" default=\"output.emily\"/>"
			"		<Param name=\"percent\" default=\"0.1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 多关节插值梯形轨迹 //
	static std::atomic<std::array<double, 7> > axis_temp_pos;
	struct MoveAbJParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> p_now, v_now, a_now;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	auto MoveAbJ::prepareNrt()->void
	{
		auto&cs = aris::server::ControlServer::instance();
		auto c = controller();
		MoveAbJParam param;
		param.total_count_vec.resize(c->motionPool().size(), 1);
		param.axis_pos_vec.resize(c->motionPool().size(), 0.0);
		param.p_now.resize(c->motionPool().size(), 0.0);
		param.v_now.resize(c->motionPool().size(), 0.0);
		param.a_now.resize(c->motionPool().size(), 0.0);

		//当前有其他指令在执行//
		std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();
		if (planptr && planptr->cmdName() != this->cmdName())
		{
			option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
			std::cout << "Other command is running!" << std::endl;
			return;
		}
		else if (planptr && planptr->cmdName() == this->cmdName())
		{
			std::array<double, 7> temp = { 0,0,0,0,0,0,0 };
			for (auto &p : cmdParams())
			{
				if (p.first == "pos")
				{	
					auto pos = matrixParam(p.first);
					if (pos.size() == c->motionPool().size())
					{			
						std::copy(pos.begin(), pos.end(), temp.begin());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					//超阈值保护//
					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (temp[i] > c->motionPool()[i].maxPos())
						{
							temp[i] = c->motionPool()[i].maxPos();
						}
						if (temp[i] < c->motionPool()[i].minPos())
						{
							temp[i] = c->motionPool()[i].minPos();
						}
					}
				}
			}
			
			axis_temp_pos.store(temp);
			option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
		}
		else
		{
			for (auto &p : cmdParams())
			{
				if (p.first == "pos")
				{
					auto pos = matrixParam(p.first);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(c->motionPool().size(), pos.toDouble());
					}
					else if (pos.size() == c->motionPool().size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					//超阈值保护//
					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_pos_vec[i] > c->motionPool()[i].maxPos())
						{
							param.axis_pos_vec[i] = c->motionPool()[i].maxPos();
						}
						if (param.axis_pos_vec[i] < c->motionPool()[i].minPos())
						{
							param.axis_pos_vec[i] = c->motionPool()[i].minPos();
						}
					}
					std::array<double, 7> temp = { 0,0,0,0,0,0,0 };
					std::copy(param.axis_pos_vec.begin(), param.axis_pos_vec.end(), temp.begin());
					axis_temp_pos.store(temp);
				}
				else if (p.first == "vel")
				{
					auto v = matrixParam(p.first);
					if (v.size() == 1)
					{
						param.axis_vel_vec.resize(c->motionPool().size(), v.toDouble());
					}
					else if (v.size() == c->motionPool().size())
					{
						param.axis_vel_vec.assign(v.begin(), v.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_vel_vec[i] > 1.0)
						{
							param.axis_vel_vec[i] = 1.0;
						}
						if (param.axis_vel_vec[i] < 0.0)
						{
							param.axis_vel_vec[i] = 0.0;
						}
						param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
					}
				}
				else if (p.first == "acc")
				{
					auto a = matrixParam(p.first);
					if (a.size() == 1)
					{
						param.axis_acc_vec.resize(c->motionPool().size(), a.toDouble());
					}
					else if (a.size() == c->motionPool().size())
					{
						param.axis_acc_vec.assign(a.begin(), a.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_acc_vec[i] > 1.0)
						{
							param.axis_acc_vec[i] = 1.0;
						}
						if (param.axis_acc_vec[i] < 0.0)
						{
							param.axis_acc_vec[i] = 0.0;
						}
						param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
					}
				}
				else if (p.first == "dec")
				{
					auto d = matrixParam(p.first);
					if (d.size() == 1)
					{
						param.axis_dec_vec.resize(c->motionPool().size(), d.toDouble());
					}
					else if (d.size() == c->motionPool().size())
					{
						param.axis_dec_vec.assign(d.begin(), d.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}

					for (Size i = 0; i < c->motionPool().size(); ++i)
					{
						if (param.axis_dec_vec[i] > 1.0)
						{
							param.axis_dec_vec[i] = 1.0;
						}
						if (param.axis_dec_vec[i] < 0.0)
						{
							param.axis_dec_vec[i] = 0.0;
						}
						param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
					}
				}
			}
		}

		this->param() = param;

		//std::fill(motorOptions().begin(), motorOptions().end(), Plan::USE_TARGET_POS);

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;

	}
	auto MoveAbJ::executeRT()->int
	{
		//获取驱动//
		auto &param = std::any_cast<MoveAbJParam&>(this->param());

		// 取得起始位置 //
		if (count() == 1)
		{
			for (Size i = 0; i < param.p_now.size(); ++i)
			{
				if (i < 6)
				{
					param.p_now[i] = model()->motionPool().at(i).mp();
					param.v_now[i] = model()->motionPool().at(i).mv();
					param.a_now[i] = model()->motionPool().at(i).ma();
				}
				else
				{
					param.p_now[i] = controller()->motionAtAbs(i).actualPos();
					param.v_now[i] = controller()->motionAtAbs(i).actualVel();
					param.a_now[i] = 0.0;
				}
			}
		}
		
		auto temp_pos = axis_temp_pos.load();
		param.axis_pos_vec.assign(temp_pos.begin(), temp_pos.end());
		
		// 梯形轨迹规划 //
		static double p_next, v_next, a_next;
		for (int i = 0; i < param.p_now.size(); i++)
		{
			aris::Size t;
			param.total_count_vec[i] = aris::plan::moveAbsolute2(param.p_now[i], param.v_now[i], param.a_now[i]
				, param.axis_pos_vec[i], 0.0, 0.0
				, param.axis_vel_vec[i], param.axis_acc_vec[i], param.axis_dec_vec[i]
				, 1e-3, 1e-4, p_next, v_next, a_next, t);
			controller()->motionAtAbs(i).setTargetPos(p_next);
			if (i < 6)
			{
				model()->motionPool().at(i).setMp(p_next);
			}
			param.p_now[i] = p_next;
			param.v_now[i] = v_next;
			param.a_now[i] = a_next;
		}
		if (model()->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller()->mout();
		if (count() % 100 == 0)
		{
			for (Size i = 0; i < param.axis_pos_vec.size(); i++)
			{
				cout << "pos" << i + 1 << ":" << controller()->motionAtAbs(i).actualPos() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller()->lout();
		for (Size i = 0; i < param.axis_pos_vec.size(); i++)
		{
			lout << controller()->motionAtAbs(i).actualPos() << ",";
		}
		lout << std::endl;
		auto re = *std::max_element(param.total_count_vec.begin(), param.total_count_vec.end());
		return re;
	}
	auto MoveAbJ::collectNrt()->void {}
	MoveAbJ::MoveAbJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvabj\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2,0.2}\"/>"
			"		<Param name=\"vel\" default=\"{0.2,0.2,0.2,0.2,0.2,0.2,0.2}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 多关节混合插值梯形轨迹；速度前馈 //
	struct MoveJMParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
		bool ab;
	};
	auto MoveJM::prepareNrt()->void
	{
		auto c = controller();
		MoveJMParam param;
        param.total_count_vec.resize(7, 1);
        param.axis_begin_pos_vec.resize(7, 0.0);

		for (auto &p : cmdParams())
		{
			if (p.first == "pos")
			{
				if (std::string(p.second) == "current_pos")
				{
					option() |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
				}
				else
				{
					auto pos = matrixParam(p.first);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(param.axis_begin_pos_vec.size(), pos.toDouble());
					}
					else if (pos.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}
                    for(int i=0; i<param.axis_pos_vec.size(); i++)
                    {
                        if(i<6)
                        {
                            param.axis_pos_vec[i] = param.axis_pos_vec[i]/180.0*PI;
                        }
                    }
				}
			}
			else if (p.first == "vel")
			{
				auto v = matrixParam(p.first);
				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(param.axis_begin_pos_vec.size(), v.toDouble());
				}
				else if (v.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_vel_vec[i] > 1.0)
					{
						param.axis_vel_vec[i] = 1.0;
					}
					if (param.axis_vel_vec[i] < 0.0)
					{
						param.axis_vel_vec[i] = 0.0;
					}
					param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
				}
			}
			else if (p.first == "acc")
			{
				auto a = matrixParam(p.first);
				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(param.axis_begin_pos_vec.size(), a.toDouble());
				}
				else if (a.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_acc_vec[i] > 1.0)
					{
						param.axis_acc_vec[i] = 1.0;
					}
					if (param.axis_acc_vec[i] < 0.0)
					{
						param.axis_acc_vec[i] = 0.0;
					}
					param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
				}
			}
			else if (p.first == "dec")
			{
				auto d = matrixParam(p.first);
				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(param.axis_begin_pos_vec.size(), d.toDouble());
				}
				else if (d.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_dec_vec[i] > 1.0)
					{
						param.axis_dec_vec[i] = 1.0;
					}
					if (param.axis_dec_vec[i] < 0.0)
					{
						param.axis_dec_vec[i] = 0.0;
					}
					param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
				}
			}
			else if (p.first == "ab")
			{
				param.ab = doubleParam(p.second);
			}
		}
		this->param() = param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;

	}
	auto MoveJM::executeRT()->int
	{
		//获取驱动//
		
		auto &param = std::any_cast<MoveJMParam&>(this->param());
        static double begin_pos[7];
        static double pos[7];
		// 取得起始位置 //
		if (count() == 1)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller()->motionPool().at(i).targetPos();
			}
		}
		// 设置驱动器的位置 //
		if (param.ab)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller()->motionAtAbs(i).setTargetPos(p);
			}
		}
		else
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				double p, v, a;
				aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_begin_pos_vec[i] + param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				controller()->motionAtAbs(i).setTargetPos(p);
			}
		}
				
		if (model()->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller()->mout();
		if (count() % 100 == 0)
		{
            for (Size i = 0; i < 7; i++)
			{
				cout << "pos" << i + 1 << ":" << controller()->motionAtAbs(i).actualPos() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller()->lout();
        for (Size i = 0; i < 7; i++)
		{
			lout << controller()->motionAtAbs(i).actualPos() << ",";
		}
		lout << std::endl;

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > count()) ? 1 : 0;
	}
	auto MoveJM::collectNrt()->void {}
	MoveJM::MoveJM(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJM\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
            "		<Param name=\"vel\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"v\"/>"
            "		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
            "		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"		<Param name=\"ab\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 手柄控制目标 //
	auto Xbox::prepareNrt()->void
	{
		for (auto &p : cmdParams())
		{
			if (p.first == "mode")
			{
				xbox_mode.store(int32Param(p.first));
			}
		}

        auto md = xbox_mode.load();

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
		std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
	}
	auto Xbox::collectNrt()->void {}
	Xbox::Xbox(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"xbox\">"
			"	<GroupParam>"
			"		<Param name=\"mode\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 切换舵机模式 //
	auto DMode::prepareNrt()->void
	{
		for (auto &p : cmdParams())
		{
			if (p.first == "mode")
			{
				mode_dynamixel.store(int32Param(p.first));
			}		
		}

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
	}
	auto DMode::collectNrt()->void {}
	DMode::DMode(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dmode\">"
			"	<GroupParam>"
			"		<GroupParam>"
			"			<Param name=\"mode\" default=\"0\"/>"
			"		</GroupParam>"
			"	</GroupParam>"
			"</Command>");
	}


	// 使能舵机 //
	auto DEnable::prepareNrt()->void
	{
        //当前有指令在执行//
        auto&cs = aris::server::ControlServer::instance();
        std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();
        if (planptr && planptr->cmdName() == this->cmdName())
        {
            option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION;
            return;
        }
		is_enabled.store(1);
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
	}
	auto DEnable::collectNrt()->void {}
	DEnable::DEnable(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"denable\">"
			"</Command>");
	}


	// 去使能舵机 //
	auto DDisable::prepareNrt()->void
	{
		is_enabled.store(0);
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
	}
	auto DDisable::collectNrt()->void {}
	DDisable::DDisable(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"ddisable\">"
			"</Command>");
	}

	// 1号舵机点动 //
	auto DJ1::prepareNrt()->void
	{
		int step = 0, direction = 0, dx_pos1 = 0;
		for (auto &p : cmdParams())
		{
			if (p.first == "step")
			{
				step = 11.378*doubleParam(p.first);
				direction = int32Param("direction");
				if (dxl1_state.load() > 0)dynamixel_control_mode.store(1);
			}	
			else if (p.first == "stop")
			{
				step = 0;
				dynamixel_control_mode.store(0);
			}
		}
        
		if (dxl1_state.load() > 0 && dxl1_state.load() < 3)
		{
			dx_pos1 = target_pos1.load();
			dx_pos1 += direction * step;
			dx_pos1 = std::min(28672, std::max(dx_pos1, -28672));
			target_pos1.store(dx_pos1);
		}

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
	}
	auto DJ1::collectNrt()->void{}
	DJ1::DJ1(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dj1\">"
			"	<UniqueParam default=\"group\">"
			"		<GroupParam name=\"group\">"
            "			<Param name=\"step\" default=\"0.3\"/>"
			"			<Param name=\"direction\" default=\"1\"/>"
			"		</GroupParam>"
			"		<Param name=\"stop\"/>"
			"	</UniqueParam>"
			"</Command>");
	}


	// 2号舵机点动 //
	auto DJ2::prepareNrt()->void
	{
		int step = 0, direction = 0, dx_pos2 = 0;
		for (auto &p : cmdParams())
		{
			if (p.first == "step")
			{
				step = 11.378*doubleParam(p.first);
				direction = int32Param("direction");
				if (dxl2_state.load() > 0)dynamixel_control_mode.store(1);
			}
			else if (p.first == "stop")
			{
				step = 0;
				dynamixel_control_mode.store(0);
			}
		}
		
		if (dxl2_state.load() > 0 && dxl2_state.load() < 3)
		{
			dx_pos2 = target_pos2.load();
			dx_pos2 += direction * step;
			dx_pos2 = std::min(28672, std::max(dx_pos2, -28672));
			target_pos2.store(dx_pos2);
		}
		

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
	}
	auto DJ2::collectNrt()->void {}
	DJ2::DJ2(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dj2\">"
			"	<UniqueParam default=\"group\">"
			"		<GroupParam name=\"group\">"
            "			<Param name=\"step\" default=\"0.3\"/>"
			"			<Param name=\"direction\" default=\"1\"/>"
			"		</GroupParam>"
			"		<Param name=\"stop\"/>"
			"	</UniqueParam>"
			"</Command>");
	}


	// 3号舵机点动 //
	auto DJ3::prepareNrt()->void
	{
		int step = 0, direction = 0, dx_pos3 = 0;
		for (auto &p : cmdParams())
		{
			if (p.first == "step")
			{
				step = 11.378*doubleParam(p.first);
				direction = int32Param("direction");
				if (dxl3_state.load() > 0)dynamixel_control_mode.store(1);
			}
			else if (p.first == "stop")
			{
				step = 0;
				dynamixel_control_mode.store(0);
			}
		}
        
		if (dxl3_state.load() > 0 && dxl3_state.load() < 3)
		{
			dx_pos3 = target_pos3.load();
			dx_pos3 += direction * step;
			dx_pos3 = std::min(28672, std::max(dx_pos3, -28672));
			target_pos3.store(dx_pos3);
		}
		
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
		
	}
	auto DJ3::collectNrt()->void {}
	DJ3::DJ3(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dj3\">"
			"	<UniqueParam default=\"group\">"
			"		<GroupParam name=\"group\">"
            "			<Param name=\"step\" default=\"0.3\"/>"
			"			<Param name=\"direction\" default=\"1\"/>"
			"		</GroupParam>"
			"		<Param name=\"stop\"/>"
			"	</UniqueParam>"
			"</Command>");
	}


    // 舵机找home //
    struct DHomeParam
    {
        std::vector<double> joint_pos_vec;
        std::vector<bool> joint_active_vec;
        const int d_num = 3;
    };
    auto DHome::prepareNrt()->void
    {
        DHomeParam param;
        int step = 0;
        for (auto cmd_param : cmdParams())
        {
            if (cmd_param.first == "all")
            {
                param.joint_active_vec.resize(param.d_num, true);
            }
            else if (cmd_param.first == "motion_id")
            {
                param.joint_active_vec.resize(param.d_num, false);
                param.joint_active_vec.at(int32Param(cmd_param.first)) = true;
            }
        }
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
		dynamixel_control_mode.store(2);
    }
    auto DHome::collectNrt()->void {}
    DHome::DHome(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"dhome\">"
            "	<GroupParam>"
            "		<UniqueParam default=\"all\">"
            "			<Param name=\"all\" abbreviation=\"a\"/>"
            "			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


	// 舵机moveabsj //
	struct DMoveAbsJParam
	{
		std::vector<double> joint_pos_vec;
		std::vector<bool> joint_active_vec;
		const int d_num = 3;
	};
	auto DMoveAbsJ::prepareNrt()->void
	{
		DMoveAbsJParam param;
		int step = 0;
		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(param.d_num, true);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(param.d_num, false);
				param.joint_active_vec.at(int32Param(cmd_param.first)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = matrixParam(cmd_param.first);
				if (mat.size() == 1)param.joint_pos_vec.resize(param.d_num, mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
		}
        auto cur_pos1 = current_pos1.load();
        auto cur_pos2 = current_pos2.load();
        auto cur_pos3 = current_pos3.load();
        auto end_pos1 = int(11.378*param.joint_pos_vec[0]);
        auto end_pos2 = int(11.378*param.joint_pos_vec[1]);
        auto end_pos3 = int(11.378*param.joint_pos_vec[2]);

        if (param.joint_active_vec[0])
        {
            target_pos1.store(end_pos1);
        }
        if (param.joint_active_vec[1])
        {
            target_pos2.store(end_pos2);
        }
        if (param.joint_active_vec[2])
        {
            target_pos3.store(end_pos3);
        }
			
		std::cout << "target_pos1:" << end_pos1 << std::endl;
		std::cout << "target_pos2:" << end_pos2 << std::endl;
		std::cout << "target_pos3:" << end_pos3 << std::endl;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_ENABLE);
		dynamixel_control_mode.store(1);
	}
	auto DMoveAbsJ::collectNrt()->void {}
	DMoveAbsJ::DMoveAbsJ(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"dmvaj\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 保存当前点 //
	auto SetHome::prepareNrt()->void
	{
		auto&cs = aris::server::ControlServer::instance();
	
		std::string point_name;
		for (auto &p : cmdParams())
		{
            if (p.first == "name")
			{
				point_name = std::string(p.second);
			}
		}

		auto temp_pos = save_point.load();
		aris::core::Matrix mat(1, 10, temp_pos.data());
		if (model()->variablePool().findByName(point_name) != model()->variablePool().end())
		{
			dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName(point_name))->data() = mat;
		}
		else
		{
			model()->variablePool().add<aris::dynamic::MatrixVariable>(point_name, mat);
		}

		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
		option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
	SetHome::SetHome(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"sethome\">"
			"	<GroupParam>"
            "		<Param name=\"name\" default=\"homepoint\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 保存home点 //
	auto SaveHome::prepareNrt()->void
	{
        std::vector<bool>is_true(controller()->motionPool().size(), false);
        std::vector<double>homepos(controller()->motionPool().size(), 0.0);
        auto offset0 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[0]).posOffset();
        auto offset1 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[1]).posOffset();
        auto offset2 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[2]).posOffset();
        auto offset3 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[3]).posOffset();
        auto offset4 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[4]).posOffset();
        auto offset5 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[5]).posOffset();
        auto offset6 = dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[6]).posOffset();

        for (auto cmd_param : cmdParams())
        {
            if (cmd_param.first == "j1")
            {

                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[0]).setPosOffset(controller()->motionPool()[0].actualPos() + offset0);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[0]).setPosOffset(doubleParam(cmd_param.first) + offset0);
                }
            }
            else if (cmd_param.first == "j2")
            {
                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[1]).setPosOffset(controller()->motionPool()[1].actualPos() + offset1);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[1]).setPosOffset(doubleParam(cmd_param.first) + offset1);
                }
            }
            else if (cmd_param.first == "j3")
            {
                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[2]).setPosOffset(controller()->motionPool()[2].actualPos() + offset2);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[2]).setPosOffset(doubleParam(cmd_param.first) + offset2);
                }
            }
            else if (cmd_param.first == "j4")
            {
                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[3]).setPosOffset(controller()->motionPool()[3].actualPos() + offset3);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[3]).setPosOffset(doubleParam(cmd_param.first) + offset3);
                }
            }
            else if (cmd_param.first == "j5")
            {
                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[4]).setPosOffset(controller()->motionPool()[4].actualPos() + offset4);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[4]).setPosOffset(doubleParam(cmd_param.first) + offset4);
                }
            }
            else if (cmd_param.first == "j6")
            {
                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[5]).setPosOffset(controller()->motionPool()[5].actualPos() + offset5);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[5]).setPosOffset(doubleParam(cmd_param.first) + offset5);
                }
            }
            else if (cmd_param.first == "j7")
            {
                if("current_pos" == cmd_param.second)
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[6]).setPosOffset(controller()->motionPool()[6].actualPos() + offset6);
                }
                else
                {
                    dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[6]).setPosOffset(doubleParam(cmd_param.first) + offset6);
                }
            }
            else if (cmd_param.first == "all")
            {
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[0]).setPosOffset(controller()->motionPool()[0].actualPos() + offset0);
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[1]).setPosOffset(controller()->motionPool()[1].actualPos() + offset1);
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[2]).setPosOffset(controller()->motionPool()[2].actualPos() + offset2);
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[3]).setPosOffset(controller()->motionPool()[3].actualPos() + offset3);
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[4]).setPosOffset(controller()->motionPool()[4].actualPos() + offset4);
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[5]).setPosOffset(controller()->motionPool()[5].actualPos() + offset5);
                dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool()[6]).setPosOffset(controller()->motionPool()[6].actualPos() + offset6);
            }
        }

		auto&cs = aris::server::ControlServer::instance();
		auto xmlpath = std::filesystem::absolute(".");
		const std::string xmlfile = "kaanh.xml";
		xmlpath = xmlpath / xmlfile;
		cs.saveXmlFile(xmlpath.string().c_str());

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
		option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION|NOT_RUN_COLLECT_FUNCTION;
	}
	SaveHome::SaveHome(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"savehome\">"
            "   <UniqueParam>"
            "       <Param name=\"j1\" default=\"current_pos\"/>"
            "       <Param name=\"j2\" default=\"current_pos\"/>"
            "       <Param name=\"j3\" default=\"current_pos\"/>"
            "       <Param name=\"j4\" default=\"current_pos\"/>"
            "       <Param name=\"j5\" default=\"current_pos\"/>"
            "       <Param name=\"j6\" default=\"current_pos\"/>"
            "       <Param name=\"j7\" default=\"current_pos\"/>"
            "       <Param name=\"all\"/>"
            "   </UniqueParam>"
			"</Command>");
	}


	// 回零位，舵机必须处于手动模式 //
	struct ToHomeParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	auto ToHome::prepareNrt()->void
	{
		auto c = controller();
		ToHomeParam param;
		param.total_count_vec.clear();
		param.axis_begin_pos_vec.clear();
		param.axis_pos_vec.clear();
		param.axis_vel_vec.clear();
		param.axis_acc_vec.clear();
		param.axis_dec_vec.clear();

		param.total_count_vec.resize(7, 1);
		param.axis_begin_pos_vec.resize(7, 0.0);
		param.axis_pos_vec.resize(7, 0.0);

		//cmdParams().at("pos")
		for (auto &p : cmdParams())
		{
			if (p.first == "pos")
			{
				if (std::string(p.second) == "current_pos")
				{
					double temp[10];
					auto pos = dynamic_cast<aris::dynamic::MatrixVariable*>(&*model()->variablePool().findByName("homepoint"))->data();
					std::copy(pos.begin(), pos.end(), temp);
					param.axis_pos_vec.assign(pos.begin(), pos.begin() + 7);

					target_pos1.store(temp[7] * 11.378);
					target_pos2.store(temp[8] * 11.378);
					target_pos3.store(temp[9] * 11.378);
					dynamixel_control_mode.store(1);
				}
				else
				{
					auto pos = matrixParam(p.first);
					if (pos.size() == 1)
					{
						param.axis_pos_vec.resize(param.axis_begin_pos_vec.size(), pos.toDouble());
					}
					else if (pos.size() == param.axis_begin_pos_vec.size())
					{
						param.axis_pos_vec.assign(pos.begin(), pos.end());
					}
					else
					{
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					}
					//for (int i = 0; i < param.axis_pos_vec.size(); i++)
					//{
					//	if (i < 6)
					//	{
					//		param.axis_pos_vec[i] = param.axis_pos_vec[i] / 180.0*PI;
					//	}
					//}
				}
			}
			else if (p.first == "vel")
			{
				auto v = matrixParam(p.first);
				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(param.axis_begin_pos_vec.size(), v.toDouble());
				}
				else if (v.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					//if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
					//	throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
					if (param.axis_vel_vec[i] > 1.0)
					{
						param.axis_vel_vec[i] = 1.0;
					}
					if (param.axis_vel_vec[i] < 0.0)
					{
						param.axis_vel_vec[i] = 0.0;
					}
					param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
				}
			}
			else if (p.first == "acc")
			{
				auto a = matrixParam(p.first);
				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(param.axis_begin_pos_vec.size(), a.toDouble());
				}
				else if (a.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_acc_vec[i] > 1.0)
					{
						param.axis_acc_vec[i] = 1.0;
					}
					if (param.axis_acc_vec[i] < 0.0)
					{
						param.axis_acc_vec[i] = 0.0;
					}
					param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
				}
			}
			else if (p.first == "dec")
			{
				auto d = matrixParam(p.first);
				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(param.axis_begin_pos_vec.size(), d.toDouble());
				}
				else if (d.size() == param.axis_begin_pos_vec.size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
				{
					if (param.axis_dec_vec[i] > 1.0)
					{
						param.axis_dec_vec[i] = 1.0;
					}
					if (param.axis_dec_vec[i] < 0.0)
					{
						param.axis_dec_vec[i] = 0.0;
					}
					param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
				}
			}
		}
		this->param() = param;

		//std::fill(motorOptions().begin(), motorOptions().end(), Plan::USE_VEL_OFFSET);

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;

	}
	auto ToHome::executeRT()->int
	{
		//获取驱动//
		
		auto &param = std::any_cast<ToHomeParam&>(this->param());
		static double begin_pos[7];
		static double pos[7];
		// 取得起始位置 //
		if (count() == 1)
		{
			for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller()->motionPool().at(i).targetPos();
			}
		}
		// 设置驱动器的位置 //

		for (Size i = 0; i < param.axis_begin_pos_vec.size(); ++i)
		{
			double p, v, a;
			aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
				, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
			controller()->motionAtAbs(i).setTargetPos(p);
		}
		
		if (model()->solverPool().at(1).kinPos())return -1;

		// 打印电流 //
		auto &cout = controller()->mout();

		// log 电流 //
		auto &lout = controller()->lout();

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > count()) ? 1 : 0;
	}
	auto ToHome::collectNrt()->void {}
	ToHome::ToHome(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"tohome\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"current_pos\"/>"
			"		<Param name=\"vel\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"v\"/>"
			"		<Param name=\"acc\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"a\"/>"
			"		<Param name=\"dec\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1,0.1}\" abbreviation=\"d\"/>"
			"	</GroupParam>"
			"</Command>");
	}


	// 保存示教点 //
	uint32_t p_num = 1;
	struct SavePParam
	{
		bool newfile;
	};
	auto SaveP::prepareNrt()->void
	{
		SavePParam param;
		std::string point_name;
		nlohmann::json js;

		for (auto &p : cmdParams())
		{
			if (p.first == "nf")
			{
				param.newfile = int32Param(p.first);
			}
		}

		if (param.newfile)
		{
			//read json file
#ifdef UNIX
			std::ifstream file("/home/kaanh/Desktop/emily/json/teaching.json");
#endif // UNIX
#ifdef WIN32
			std::ifstream file("./json/teaching.json");
#endif // WIN32
			file >> js;
            file.close();
			time_t t = time(0);
			tm* p = localtime(&t);
            char filename[100] = { 0 };
#ifdef UINX
			sprintf(filename, "/home/kaanh/Desktop/emily/json/%d%02d%02d%02d%02d%02d.json", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
#endif // UINX
			
#ifdef WIN32
			sprintf(filename, "./json/%d%02d%02d%02d%02d%02d.json", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec);
#endif // WIN32      
            p = NULL;
            delete (p);

			//write json file
			std::ofstream os;
			os.open(filename);
			os << js.dump(2) << std::endl;
            os.close();

			//new teaching.json
			js.clear();
			p_num = 1;
			point_name = "p" + std::to_string(p_num);
			auto temp_pos = save_point.load();
			js[point_name] = temp_pos;

			//write json file
            std::ofstream os_write;
#ifdef UINX
			os_write.open("/home/kaanh/Desktop/emily/json/teaching.json");
#endif // UINX

#ifdef WIN32
			os_write.open("./json/teaching.json");
#endif // WIN32
            
            os_write << js.dump(2) << std::endl;
            os_write.close();
		}
		else
        {

//#ifdef UINX
			std::ifstream file("/home/kaanh/Desktop/emily/json/teaching.json");
//#endif

//#ifdef WIN32
//			std::ifstream file("./json/teaching.json");
//#endif // WIN32

			file >> js;
			//get new pos
			point_name = "p" + std::to_string(++p_num);
			auto temp_pos = save_point.load();
			js[point_name] = temp_pos;
			//write json file
			std::ofstream os;
#ifdef UINX
			os.open("/home/kaanh/Desktop/emily/json/teaching.json");
#endif // UINX

#ifdef WIN32
			os.open("./json/teaching.json");
#endif // WIN32
            
			os << js.dump(2) << std::endl;
			os.close();
		}

        js.clear();

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
		option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
	}
    auto SaveP::collectNrt()->void{}
    SaveP::SaveP(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"savep\">"
			"	<GroupParam>"
			"		<Param name=\"nf\" default=\"0\"/>"
			"	</GroupParam>"
			"</Command>");
	}

    auto createPlanRoot()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<tuying::Get>();
		plan_root->planPool().add<tuying::Enable>();
		plan_root->planPool().add<tuying::Disable>();
        plan_root->planPool().add<tuying::Getp>();
		plan_root->planPool().add<tuying::MoveAbJ>();
		plan_root->planPool().add<tuying::MoveT>();
		plan_root->planPool().add<tuying::MoveE>();
        plan_root->planPool().add<tuying::MoveE0>();
		plan_root->planPool().add<tuying::MoveJM>();
		plan_root->planPool().add<tuying::Xbox>();
		plan_root->planPool().add<tuying::DMode>();
		plan_root->planPool().add<tuying::DEnable>();
		plan_root->planPool().add<tuying::DDisable>();
		plan_root->planPool().add<tuying::DJ1>();
		plan_root->planPool().add<tuying::DJ2>();
		plan_root->planPool().add<tuying::DJ3>();
		plan_root->planPool().add<tuying::DMoveAbsJ>();
		plan_root->planPool().add<tuying::DHome>();
		plan_root->planPool().add<tuying::SaveHome>();
		plan_root->planPool().add<tuying::ToHome>();
		plan_root->planPool().add<tuying::SaveP>();
		
		plan_root->planPool().add<aris::plan::Start>();
		plan_root->planPool().add<aris::plan::Stop>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Clear>();
		plan_root->planPool().add<kaanh::Recover>();
		auto &rs = plan_root->planPool().add<kaanh::Reset>();
		//for qifan robot//
		rs.command().findParam("pos")->setDefaultValue("{0.0,0.0,0.0,0.0,0.0,0.0,0.0}");
		plan_root->planPool().add<aris::server::GetInfo>();
		plan_root->planPool().add<kaanh::MoveAbsJ>();
        plan_root->planPool().add<aris::plan::MoveAbsJ>();
		plan_root->planPool().add<kaanh::MoveL>();
		plan_root->planPool().add<kaanh::MoveJ>();
		plan_root->planPool().add<kaanh::MoveC>();
		plan_root->planPool().add<kaanh::Var>();
		plan_root->planPool().add<kaanh::Evaluate>();
        plan_root->planPool().add<kaanh::JogJ1>();
        plan_root->planPool().add<kaanh::JogJ2>();
        plan_root->planPool().add<kaanh::JogJ3>();
        plan_root->planPool().add<kaanh::JogJ4>();
        plan_root->planPool().add<kaanh::JogJ5>();
        plan_root->planPool().add<kaanh::JogJ6>();
        plan_root->planPool().add<kaanh::JogJ7>();
        plan_root->planPool().add<kaanh::JX>();
		plan_root->planPool().add<kaanh::JY>();
        plan_root->planPool().add<kaanh::JZ>();
		plan_root->planPool().add<kaanh::JRX>();
		plan_root->planPool().add<kaanh::JRY>();
		plan_root->planPool().add<kaanh::JRZ>();

		plan_root->planPool().add<kaanh::ClearCon>();
		plan_root->planPool().add<kaanh::SetCon>();
		plan_root->planPool().add<kaanh::SetDH>();
		plan_root->planPool().add<kaanh::SetPG>();
		plan_root->planPool().add<kaanh::SetPPath>();
		plan_root->planPool().add<kaanh::SetUI>();
		plan_root->planPool().add<kaanh::SetDriver>();
		plan_root->planPool().add<kaanh::SaveXml>();
		plan_root->planPool().add<kaanh::ScanSlave>();
		plan_root->planPool().add<kaanh::GetEsiPdoList>();
		plan_root->planPool().add<kaanh::SetEsiPath>();
		plan_root->planPool().add<kaanh::GetXml>();
		plan_root->planPool().add<kaanh::SetXml>();
		plan_root->planPool().add<kaanh::SetCT>();
		plan_root->planPool().add<kaanh::SetVel>();
		plan_root->planPool().add<kaanh::Run>();

		return plan_root;
	}
}
