#include <algorithm>
#include "wuling.h"
#include "wulingconfig.h"
#include <array>
#include <stdlib.h>
#include <string>
#include "planfuns.h"
#include <bitset>
#include "json.hpp"


using namespace aris::dynamic;
using namespace aris::plan;

//global vel//
extern kaanh::Speed g_vel;
extern std::atomic_int g_vel_percent;
//global vel//

//state machine flag//
extern std::atomic_bool g_is_enabled;
extern std::atomic_bool g_is_error;
extern std::atomic_bool g_is_manual;
extern std::atomic_bool g_is_auto;
extern std::atomic_bool g_is_running;
extern std::atomic_bool g_is_paused;
extern std::atomic_bool g_is_stopped;
//state machine flag//

extern aris::core::Calculator g_cal;
extern aris::dynamic::Model g_model;
extern aris::dynamic::Marker *g_tool, *g_wobj;

struct CmdListParam
{
    std::map<int, std::string> cmd_vec;
    int current_cmd_id = 0;
    int current_plan_id = -1;
}cmdparam;
std::shared_ptr<kaanh::MoveBase> g_plan;


namespace kaanh
{
    int g_counter = 100;
    double g_count = 0.0;
    aris::dynamic::Marker tool1;

    //更新实时状态：使能、报错//
    auto update_state(aris::server::ControlServer &cs)->void
    {
        static bool motion_state[256] = { false };

        //获取motion的使能状态，0表示去使能状态，1表示使能状态//
        for (aris::Size i = 0; i < cs.controller().motionPool().size(); i++)
        {
            auto cm = dynamic_cast<aris::control::EthercatMotor*>(&cs.controller().motionPool()[i]);
            if ((cm->statusWord() & 0x6f) != 0x27)
            {
                motion_state[i] = 0;
            }
            else
            {
                motion_state[i] = 1;
            }
            //motion_state[i] = 1;
        }

        //获取ret_code的值，判断是否报错，if条件可以初始化变量，并且取变量进行条件判断//
        g_is_error.store(cs.errorCode());

        g_is_enabled.store(std::all_of(motion_state, motion_state + cs.controller().motionPool().size(), [](bool i) {return i; }));

        auto &inter = dynamic_cast<aris::server::ProgramWebInterface&>(cs.interfacePool().at(0));
        if (inter.isAutoMode())
        {
            g_is_auto.store(true);
        }
        else
        {
            g_is_auto.store(false);
        }

        g_is_running.store(inter.isAutoRunning());
        g_is_paused.store(inter.isAutoPaused());
        g_is_stopped.store(inter.isAutoStopped());
        //暂停、恢复功能复位//
        if (!inter.isAutoRunning())
        {
            g_counter = 100;
        }

    }

    //获取状态字——100:去使能,200:手动,300:自动,400:程序运行中,410:程序暂停中,420:程序停止，500:错误//
    auto get_state_code()->std::int32_t
    {
        if (g_is_enabled.load())
        {
            if (g_is_error.load())
            {
                return 500;
            }
            else
            {
                if (!g_is_auto.load())
                {
                    return 200;
                }
                else
                {
                    if (g_is_running.load())
                    {
                        if (g_is_stopped)
                        {
                            return 420;
                        }
                        else if (g_is_paused.load())
                        {
                            return 410;
                        }
                        else
                        {
                            return 400;
                        }
                    }
                    else
                    {
                        return 300;
                    }
                }
            }
        }
        else
        {
            return 100;
        }
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

    struct SetInputMovement
    {
        std::vector<double> axis_begin_pos_vec;
        std::vector<double> axis_pos_vec;
        std::vector<double> axis_vel_vec;
        std::vector<double> axis_acc_vec;
        std::vector<double> axis_dec_vec;
        std::vector<double> axis_jerk_vec;
    };
#define SET_INPUT_MOVEMENT_STRING \
        "		<Param name=\"pos\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"\
        "		<Param name=\"acc\" default=\"0.1\"/>"\
        "		<Param name=\"vel\" default=\"{0.025, 0.025, 3.4, 0.0, 0.0}\"/>"\
        "		<Param name=\"dec\" default=\"0.1\"/>"\
        "		<Param name=\"jerk\" default=\"0.1\"/>"
    auto set_input_movement(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan, SetInputMovement &param)->void
    {
        param.axis_begin_pos_vec.resize(plan.controller()->motionPool().size(), 0.0);
        auto &cal = plan.controlServer()->model().calculator();

        for (auto cmd_param : cmd_params)
        {
            if (cmd_param.first == "pos")
            {
                auto p = std::any_cast<aris::core::Matrix>(cal.calculateExpression("jointtarget(" + std::string(cmd_param.second) + ")").second);
                //auto p = plan.matrixParam(cmd_param.first);
                if (p.size() == plan.controller()->motionPool().size())
                {
                    param.axis_pos_vec.assign(p.begin(), p.end());
                }
                else
                {
                    THROW_FILE_LINE("");
                }
            }
            else if (cmd_param.first == "acc")
            {
                auto a = plan.matrixParam(cmd_param.first);

                if (a.size() == 1)
                {
                    param.axis_acc_vec.resize(plan.controller()->motionPool().size(), a.toDouble());
                }
                else if (a.size() == plan.controller()->motionPool().size())
                {
                    param.axis_acc_vec.assign(a.begin(), a.end());
                }
                else
                {
                    THROW_FILE_LINE("");
                }
                for (int i = 0; i < plan.controller()->motionPool().size(); ++i) param.axis_acc_vec[i] *= plan.controller()->motionPool().at(i).maxAcc();
            }
            else if (cmd_param.first == "vel")
            {
                param.axis_vel_vec.resize(plan.controller()->motionPool().size(), 0.0);
                auto v = std::any_cast<kaanh::Speed>(cal.calculateExpression("speed(" + std::string(cmd_param.second) + ")").second);
                for (int i = 0; i < 6; ++i)
                {
                    param.axis_vel_vec[i] = plan.controller()->motionPool()[i].maxVel()*v.w_per;
                }
            }
            else if (cmd_param.first == "dec")
            {
                auto d = plan.matrixParam(cmd_param.first);

                if (d.size() == 1)
                {
                    param.axis_dec_vec.resize(plan.controller()->motionPool().size(), d.toDouble());
                }
                else if (d.size() == plan.controller()->motionPool().size())
                {
                    param.axis_dec_vec.assign(d.begin(), d.end());
                }
                else
                {
                    THROW_FILE_LINE("");
                }
                for (int i = 0; i < plan.controller()->motionPool().size(); ++i) param.axis_dec_vec[i] *= plan.controller()->motionPool().at(i).maxAcc();
            }
            else if (cmd_param.first == "jerk")
            {
                auto d = plan.matrixParam(cmd_param.first);

                if (d.size() == 1)
                {
                    param.axis_jerk_vec.resize(plan.controller()->motionPool().size(), d.toDouble());
                }
                else if (d.size() == plan.controller()->motionPool().size())
                {
                    param.axis_jerk_vec.assign(d.begin(), d.end());
                }
                else
                {
                    THROW_FILE_LINE("");
                }
                for (int i = 0; i < plan.controller()->motionPool().size(); ++i) param.axis_jerk_vec[i] *= param.axis_acc_vec[i];
            }
        }
    }
    auto check_input_movement(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan, SetInputMovement &param, SetActiveMotor &active)->void
    {
        auto c = plan.controller();
        for (Size i = 0; i < c->motionPool().size(); ++i)
        {
            if (active.active_motor[i])
            {
                if (param.axis_pos_vec[i] > c->motionPool()[i].maxPos() || param.axis_pos_vec[i] < c->motionPool()[i].minPos())
                    THROW_FILE_LINE("input pos beyond range");

                if (param.axis_vel_vec[i] > c->motionPool()[i].maxVel() || param.axis_vel_vec[i] <= 0.0)
                    THROW_FILE_LINE("input vel beyond range");

                if (param.axis_acc_vec[i] > c->motionPool()[i].maxAcc() || param.axis_acc_vec[i] <= 0.0)
                    THROW_FILE_LINE("input acc beyond range");

                if (param.axis_dec_vec[i] > c->motionPool()[i].maxAcc() || param.axis_dec_vec[i] <= 0.0)
                    THROW_FILE_LINE("input dec beyond range");

                if (param.axis_jerk_vec[i] > 10*c->motionPool()[i].maxAcc() || param.axis_jerk_vec[i] <= 0.0)
                    THROW_FILE_LINE("input dec beyond range");
            }
        }
    }


    // 获取part_pq，end_pq，end_pe等 //
    std::atomic_bool having_model=false;
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
        par.motion_pos.clear();
        par.motion_vel.clear();
        par.motion_acc.clear();
        par.motion_toq.clear();
        par.ai.resize(100, 1.0);
        par.di.resize(100, false);
        par.motion_state.resize(6, 0);
        std::any param = par;
        //std::any param = std::make_any<GetParam>();

        controlServer()->getRtData([&](aris::server::ControlServer& cs, const aris::plan::Plan *target, std::any& data)->void
        {
            update_state(cs);

            for (aris::Size i(-1); ++i < cs.model().partPool().size();)
            {
                cs.model().partPool().at(i).getPq(std::any_cast<GetParam &>(data).part_pq.data() + i * 7);
            }

            for (aris::Size i(0); i < cs.model().generalMotionPool().size();i++)
            {
                cs.model().generalMotionPool().at(0).getMpq(std::any_cast<GetParam &>(data).end_pq.data());
                cs.model().generalMotionPool().at(0).getMpe(std::any_cast<GetParam &>(data).end_pe.data(), "321");
            }

#ifdef WIN32
            for (aris::Size i = 0; i < cs.model().motionPool().size(); i++)
            {
                std::any_cast<GetParam &>(data).motion_pos.push_back(cs.model().motionPool()[i].mp());
                std::any_cast<GetParam &>(data).motion_vel.push_back(cs.model().motionPool()[i].mv());
                std::any_cast<GetParam &>(data).motion_acc.push_back(cs.model().motionPool()[i].ma());
                std::any_cast<GetParam &>(data).motion_toq.push_back(cs.model().motionPool()[i].ma());
            }
#endif // WIN32

#ifdef UNIX
            for (aris::Size i = 0; i < cs.controller().motionPool().size(); i++)
            {
                std::any_cast<GetParam &>(data).motion_pos.push_back(cs.controller().motionPool()[i].actualPos());
                std::any_cast<GetParam &>(data).motion_vel.push_back(cs.controller().motionPool()[i].actualVel());
                std::any_cast<GetParam &>(data).motion_acc.push_back(cs.model().motionPool()[i].ma());
                std::any_cast<GetParam &>(data).motion_toq.push_back(cs.controller().motionPool()[i].actualToq());
            }
#endif // UNIX

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

        out_data.state_code = get_state_code();

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
        out_param.push_back(std::make_pair<std::string, std::any>("current_plan", out_data.currentplan));
        out_param.push_back(std::make_pair<std::string, std::any>("current_plan_id", cmdparam.current_plan_id));
        out_param.push_back(std::make_pair(std::string("cs_err_code"), std::make_any<int>(cs.errorCode())));
        out_param.push_back(std::make_pair(std::string("cs_err_msg"), std::make_any<std::string>(cs.errorMsg())));
        out_param.push_back(std::make_pair(std::string("pro_err_code"), std::make_any<int>(inter.lastErrorCode())));
        out_param.push_back(std::make_pair(std::string("pro_err_msg"), std::make_any<std::string>(inter.lastError())));
        out_param.push_back(std::make_pair(std::string("pro_err_line"), std::make_any<int>(inter.lastErrorLine())));
        out_param.push_back(std::make_pair(std::string("line"), std::make_any<int>(inter.currentLine())));


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


    struct Home::Imp :public SetActiveMotor
    {
        std::int32_t limit_time;
        double offset;
    };
    auto Home::prepareNrt()->void
    {
        set_active_motor(cmdParams(), *this, *imp_);
        imp_->limit_time = int32Param("limit_time");

        for (aris::Size i = 0; i < imp_->active_motor.size(); ++i)
        {
            if (imp_->active_motor[i])
            {
                std::int8_t method = int32Param("method");
                if (method < 1 || method > 35) THROW_FILE_LINE("invalid home method");

                imp_->offset = doubleParam("offset");
                std::int32_t offset = int32Param("offset");
                std::uint32_t high_speed = int32Param("high_speed");
                std::uint32_t low_speed = int32Param("low_speed");
                std::uint32_t acc = int32Param("acceleration");

                auto &cm = dynamic_cast<aris::control::EthercatMotor &>(controller()->motionPool()[i]);

                cm.writeSdo(0x6098, 0x00, method);
                std::int8_t method_read;
                cm.readSdo(0x6098, 0x00, method_read);
                if (method_read != method)THROW_FILE_LINE("home sdo write failed method");
                cm.writeSdo(0x607C, 0x00, offset);
                std::int32_t offset_read;
                cm.readSdo(0x607C, 0x00, offset_read);
                if (offset_read != offset)THROW_FILE_LINE("home sdo write failed offset");
                cm.writeSdo(0x6099, 0x01, high_speed);
                std::int32_t high_speed_read;
                cm.readSdo(0x6099, 0x01, high_speed_read);
                if (high_speed_read != high_speed)THROW_FILE_LINE("home sdo write failed high_speed");
                cm.writeSdo(0x6099, 0x02, low_speed);
                std::int32_t low_speed_read;
                cm.readSdo(0x6099, 0x02, low_speed_read);
                if (low_speed_read != low_speed)THROW_FILE_LINE("home sdo write failed low_speed");
                cm.writeSdo(0x609A, 0x00, acc);
                std::int32_t acc_read;
                cm.readSdo(0x609A, 0x00, acc_read);
                if (acc_read != acc)THROW_FILE_LINE("home sdo write failed acc");

            }
        }

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto Home::executeRT()->int
    {
        bool is_all_finished = true;
        for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
        {
            if (imp_->active_motor[i])
            {
                auto &cm = controller()->motionPool().at(i);
                auto ret = cm.home();
                if (ret)
                {
                    is_all_finished = false;

                    if (count() % 1000 == 0)
                    {
                        controller()->mout() << "Unhomed motor, slave id: " << cm.id()
                            << ", absolute id: " << i << ", ret: " << ret << std::endl;
                    }
                }
                else
                {
                    imp_->active_motor[i] = false;
                }
            }
        }

        return is_all_finished ? 0 : 1;
    }
    Home::~Home() = default;
    Home::Home(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"hm\">"
            "	<GroupParam>"
            "		<Param name=\"method\" default=\"35\"/>"
            "		<Param name=\"offset\" default=\"0\"/>"
            "		<Param name=\"high_speed\" default=\"20000\"/>"
            "		<Param name=\"low_speed\" default=\"20\"/>"
            "		<Param name=\"acceleration\" default=\"100000\"/>"
            "		<Param name=\"limit_time\" default=\"5000\"/>"
            SELECT_MOTOR_STRING
            CHECK_PARAM_STRING
            "	</GroupParam>"
            "</Command>");
    }
    ARIS_DEFINE_BIG_FOUR_CPP(Home);


    struct Reset::Imp :public SetActiveMotor, SetInputMovement { std::vector<Size> total_count_vec; };
    auto Reset::prepareNrt()->void
    {
        set_check_option(cmdParams(), *this);
        set_active_motor(cmdParams(), *this, *imp_);
        set_input_movement(cmdParams(), *this, *imp_);
/*
        for (Size i = 0; i < controller()->motionPool().size(); ++i)
        {
            auto &cm = controller()->motionPool()[i];
            //imp_->axis_pos_vec[i] = imp_->axis_pos_vec[i] * (cm.maxPos() - cm.minPos()) + cm.minPos();
            imp_->axis_acc_vec[i] = imp_->axis_acc_vec[i] * cm.maxAcc();
            imp_->axis_dec_vec[i] = imp_->axis_dec_vec[i] * cm.maxAcc();
        }
*/
        check_input_movement(cmdParams(), *this, *imp_, *imp_);

        imp_->total_count_vec.resize(controller()->motionPool().size(), 1);

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto Reset::executeRT()->int
    {
        // 取得起始位置 //
        if (count() == 1)
        {
            for (Size i = 0; i < controller()->motionPool().size(); ++i)
            {
                if (imp_->active_motor[i])
                {
                    imp_->axis_begin_pos_vec[i] = controller()->motionPool().at(i).targetPos();
                    //imp_->axis_begin_pos_vec[i] = controller()->motionPool().at(i).actualPos();
                }
            }
        }

        // 设置驱动器的位置 //
        for (Size i = 0; i < controller()->motionPool().size(); ++i)
        {
            if (imp_->active_motor[i])
            {
                double p, v, a;
                aris::plan::moveAbsolute(static_cast<double>(count()), imp_->axis_begin_pos_vec[i], imp_->axis_pos_vec[i], imp_->axis_vel_vec[i] / 1000
                    , imp_->axis_acc_vec[i] / 1000 / 1000, imp_->axis_dec_vec[i] / 1000 / 1000, p, v, a, imp_->total_count_vec[i]);
                controller()->motionAtAbs(i).setTargetPos(p);
                model()->motionPool().at(i).setMp(p);
            }
        }
        if (model()->solverPool().at(1).kinPos())return -1;
        return (static_cast<int>(*std::max_element(imp_->total_count_vec.begin(), imp_->total_count_vec.end())) > count()) ? 1 : 0;
    }
    Reset::~Reset() = default;
    Reset::Reset(const std::string &name) :Plan(name), imp_(new Imp)
    {
        command().loadXmlStr(
            "<Command name=\"rs\">"
            "	<GroupParam>"
                    SET_INPUT_MOVEMENT_STRING
                    SELECT_MOTOR_STRING
                    CHECK_PARAM_STRING
            "	</GroupParam>"
            "</Command>");
    }
    ARIS_DEFINE_BIG_FOUR_CPP(Reset);


    struct RecoverParam
    {
        std::atomic_bool is_kinematic_ready_;
        std::atomic_bool is_rt_waiting_ready_;
        std::future<void> fut;
        int kin_ret;
    };
    auto Recover::prepareNrt()->void
    {
        auto p = std::make_shared<RecoverParam>();

        p->is_kinematic_ready_ = false;
        p->is_rt_waiting_ready_ = false;
        p->fut = std::async(std::launch::async, [this](std::shared_ptr<RecoverParam> p)
        {
            // 等待正解求解的需求 //
            while (!p->is_rt_waiting_ready_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));

            // 求正解 //
            p->kin_ret = model()->solverPool()[1].kinPos();

            // 通知实时线程 //
            p->is_kinematic_ready_.store(true);
        }, p);

        this->param() = p;
        for (auto &option : motorOptions()) option |= NOT_CHECK_ENABLE | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |NOT_CHECK_POS_CONTINUOUS;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        std::cout << "error code:" << aris::server::ControlServer::instance().errorCode() << std::endl;
        std::cout << "error msg :" << aris::server::ControlServer::instance().errorMsg() << std::endl;

        aris::server::ControlServer::instance().clearError();
    }
    auto Recover::executeRT()->int
    {
        auto param = std::any_cast<std::shared_ptr<RecoverParam> &>(this->param());

        if (count() < 3)
        {
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                controller()->motionPool()[i].setTargetPos(controller()->motionPool().at(i).actualPos());
                model()->motionPool()[i].setMp(controller()->motionPool().at(i).actualPos());
            }

            param->is_rt_waiting_ready_.store(true);

            return 1;
        }

        return param->is_kinematic_ready_.load() ? param->kin_ret : 1;
    }
    auto Recover::collectNrt()->void
    {
        if (count())
        {
            std::any_cast<std::shared_ptr<RecoverParam>&>(this->param())->fut.get();
        }
        else
        {
            // 此时前面指令出错，系统清理了该命令，这时设置一下 //
            std::any_cast<std::shared_ptr<RecoverParam>&>(this->param())->is_rt_waiting_ready_.store(true);
            std::any_cast<std::shared_ptr<RecoverParam>&>(this->param())->fut.get();
        }
    }
    Recover::~Recover() = default;
    Recover::Recover(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"rc\">"
            "</Command>");
    }
    ARIS_DEFINE_BIG_FOUR_CPP(Recover);

    struct Sleep::Imp { int count; };
    auto Sleep::prepareNrt()->void
    {
        imp_->count = int32Param("count");
        for (auto &option : motorOptions()) option |= NOT_CHECK_ENABLE;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto Sleep::executeRT()->int { return imp_->count - count(); }
    Sleep::~Sleep() = default;
    Sleep::Sleep(const std::string &name) :Plan(name), imp_(new Imp)
    {
        command().loadXmlStr(
            "<Command name=\"sl\">"
            "	<Param name=\"count\" default=\"1000\" abbreviation=\"c\"/>"
            "</Command>");
    }
    ARIS_DEFINE_BIG_FOUR_CPP(Sleep);

    MoveBase::MoveBase(const MoveBase &other) :Plan(other),
        realzone(0), planzone(0), cmd_finished(false), cmd_executing(false){};

    struct MoveAbsJParam :public SetActiveMotor, SetInputMovement
    {
        std::vector<double> pos_ratio;
        std::vector<Size> total_count;
        aris::dynamic::Marker *tool, *wobj;
        std::shared_ptr<kaanh::MoveBase> pre_plan;
        uint32_t max_total_count = 0;
        bool zone_enabled = false;
        Speed sp;
        Zone zone;
        Load ld;
    };
    struct MoveJParam
    {
        std::vector<double> joint_vel, joint_acc, joint_dec, joint_jerk, ee_pq, joint_pos_begin, joint_pos_end, pos_ratio;
        std::vector<Size> total_count;
        aris::dynamic::Marker *tool, *wobj;
        std::shared_ptr<kaanh::MoveBase> pre_plan;
        uint32_t max_total_count = 0;
        bool zone_enabled = false;
        Speed sp;
        Zone zone;
        Load ld;
    };
    struct MoveLParam
    {
        std::vector<double> ee_pq, relative_pa;
        std::vector<Size> total_count;

        double acc, vel, dec, jerk, norm_pos, norm_ori, begin_pm[16], pos_ratio = 1.0, ori_ratio = 1.0;
        double angular_acc, angular_vel, angular_dec, angular_jerk;
        aris::dynamic::Marker *tool, *wobj;

        std::shared_ptr<kaanh::MoveBase> pre_plan;
        uint32_t max_total_count = 0;
        bool zone_enabled = false;
        Speed sp;
        Zone zone;
        Load ld;
    };
    struct MoveCParam
    {
        std::vector<double> joint_vel, joint_acc, joint_dec, ee_begin_pq, ee_mid_pq, ee_end_pq, joint_pos_begin, joint_pos_end;
        Size total_count[6];
        double acc, vel, dec, jerk, norm_pos;
        double angular_acc, angular_vel, angular_dec, angular_jerk;
        aris::dynamic::Marker *tool, *wobj;
        double A[9], C[3], R, theta, ori_theta, pos_ratio, ori_ratio;

        std::shared_ptr<kaanh::MoveBase> pre_plan;
        uint32_t max_total_count = 0;
        bool zone_enabled = false;
        Speed sp;
        Zone zone;
        Load ld;
    };
    template<typename MoveType>
    auto PauseContinueB(MoveType *plan, aris::server::ProgramWebInterface &pwinter)->void
    {
        if (plan->count() == 1)
        {
            g_count = 0.0;
        }
        if (pwinter.isAutoPaused() || pwinter.isAutoStopped())
        {
            g_counter--;
        }
        else
        {
            g_counter++;
        }
        g_counter = std::max(std::min(g_counter, 100), 0);
        g_count = g_count + timespeed[g_counter]* g_vel_percent.load()/100.0;
    }
    template<typename ParamType>
    auto PauseContinueE(ParamType *param, aris::server::ProgramWebInterface &pwinter, uint32_t &rzcount)->void
    {
        if (!param->zone_enabled)
        {
            //执行到最后一个count时,进行特殊处理
            if ((param->max_total_count - rzcount - int32_t(g_count) == 0) || (param->max_total_count == 0))
            {
                if (pwinter.isAutoPaused() || pwinter.isAutoStopped())
                {
                    g_counter = 0;
                }
                else
                {
                    g_counter = 100;
                }
            }
        }
    }

    //走关节//
    auto MoveAbsJ::prepareNrt()->void
    {
        MoveAbsJParam param;
        param.total_count.resize(model()->motionPool().size(), 0);
        param.pos_ratio.resize(model()->motionPool().size(), 1.0);
        auto &cal = this->controlServer()->model().calculator();
        set_check_option(cmdParams(), *this);
        set_active_motor(cmdParams(), *this, param);
        set_input_movement(cmdParams(), *this, param);
        check_input_movement(cmdParams(), *this, param, param);

        param.tool = &*model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        param.wobj = &*model()->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));

        // find joint acc/vel/dec/jerk/zone
        for (auto cmd_param : cmdParams())
        {
            auto c = controller();
            if (cmd_param.first == "zone")
            {
                if (cmd_param.second == "fine")//不设置转弯区
                {
                    param.zone.dis = 0.0;
                    param.zone.per = 0.0;
                    param.zone_enabled = 0;
                }
                else//设置转弯区
                {
                    auto z = std::any_cast<kaanh::Zone>(cal.calculateExpression("zone(" + std::string(cmd_param.second) + ")").second);
                    param.zone = z;
                    param.zone_enabled = 1;
                    if (param.zone.per > 0.5&&param.zone.per < 0.001)
                    {
                        THROW_FILE_LINE("zone out of range");
                    }
                }
            }
            else if (cmd_param.first == "load")
            {
                auto temp = std::any_cast<kaanh::Load>(cal.calculateExpression("load(" + std::string(cmd_param.second) + ")").second);
                param.ld = temp;
            }
        }


        //本指令cmdid-上一条指令的cmdid>1，且上一条指令执行完毕，g_plan赋值为nullptr
        if (param.pre_plan != nullptr)
        {
            if ((this->cmdId() - param.pre_plan->cmdId() > 1) && (param.pre_plan->cmd_finished.load()))g_plan = nullptr;
        }

        //更新全局变量g_plan//
        param.pre_plan = g_plan;
        g_plan = std::dynamic_pointer_cast<MoveBase>(sharedPtrForThis());//------这里需要潘博aris库提供一个plan接口，返回std::shared_ptr<MoveBase>类型指针

        if (param.pre_plan == nullptr)
        {
            std::cout << "preplan:" << "nullptr" << std::endl;
        }
        else
        {
            std::cout << "preplan:" << param.pre_plan->name() << std::endl;
        }

        // 设置转弯区 //
        double p, v, a, j;
        double end_pm[16];
        if (param.pre_plan == nullptr)//转弯第一条指令
        {
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                param.axis_begin_pos_vec[i] = controller()->motionPool()[i].targetPos();
            }
        }
        else if (std::string(param.pre_plan->name()) == this->name())//转弯第二或第n条指令
        {
            std::cout << "precmdname1:" << param.pre_plan->name() << "  cmdname1:" << this->name() << std::endl;
            //从全局变量中获取上一条转弯区指令的目标点//
            auto preparam = std::any_cast<MoveAbsJParam>(&param.pre_plan->param());
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                param.axis_begin_pos_vec[i] = preparam->axis_pos_vec[i];
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveL,moveC,moveJ
        {
            std::cout << "precmdname2:" << param.pre_plan->name() << "  cmdname2:" << this->name() << std::endl;
            //获取起始点位置//
            if (std::string(param.pre_plan->name()) == "MoveJ")
            {
                auto preparam = std::any_cast<MoveJParam>(&param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_pq.data(), end_pm);
                preparam->tool->setPm(*preparam->wobj, end_pm);
            }
            if (std::string(param.pre_plan->name()) == "MoveL")
            {
                auto preparam = std::any_cast<MoveLParam>(&param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_pq.data(), end_pm);
                preparam->tool->setPm(*preparam->wobj, end_pm);
            }
            if (std::string(param.pre_plan->name()) == "MoveC")
            {
                auto preparam = std::any_cast<MoveCParam>(&param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_end_pq.data(), end_pm);
                preparam->tool->setPm(*preparam->wobj, end_pm);
            }
            model()->generalMotionPool().at(0).updMpm();
            if (model()->solverPool().at(0).kinPos())THROW_FILE_LINE("");
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                param.axis_begin_pos_vec[i] = model()->motionPool()[i].mp();
            }
        }

        //计算转弯区对应的count数//
        double max_pos = 0.0;
        Size max_i = 0;
        for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
        {
            //S形轨迹规划//
            traplan::sCurve(1, param.axis_begin_pos_vec[i], param.axis_pos_vec[i],
                param.axis_vel_vec[i] / 1000, param.axis_acc_vec[i] / 1000 / 1000, param.axis_jerk_vec[i] / 1000 / 1000 / 1000,
                p, v, a, j, param.total_count[i]);
            if (std::abs(max_pos) < std::abs(param.axis_pos_vec[i] - param.axis_begin_pos_vec[i]))
            {
                max_pos = param.axis_pos_vec[i] - param.axis_begin_pos_vec[i];
                max_i = i;
            }
        }
        //本条指令的最大规划count数//
        param.max_total_count = *std::max_element(param.total_count.begin(), param.total_count.end());

        //获取不同轴的比率//
        for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
        {
            param.pos_ratio[i] = 1.0* param.total_count[i] / param.max_total_count;
        }
        for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
        {
            //S形轨迹规划//
            traplan::sCurve(1, param.axis_begin_pos_vec[i], param.axis_pos_vec[i],
                param.axis_vel_vec[i] / 1000 * param.pos_ratio[i], param.axis_acc_vec[i] / 1000 / 1000 * param.pos_ratio[i] * param.pos_ratio[i], param.axis_jerk_vec[i] / 1000 / 1000 / 1000 * param.pos_ratio[i] * param.pos_ratio[i] * param.pos_ratio[i],
                p, v, a, j, param.total_count[i]);
        }
        param.max_total_count = *std::max_element(param.total_count.begin(), param.total_count.end());

        //二分法//
        auto pos_zone = param.axis_pos_vec[max_i] - max_pos * param.zone.per;
        aris::Size begin_count = 1, target_count = 0, end_count;
        end_count = param.max_total_count;
        if (param.zone_enabled)
        {
            if (std::abs(max_pos) > 2e-3)//转弯半径大于0.002rad
            {
                while (std::abs(p - pos_zone) > 1e-9)
                {
                    if (p < pos_zone)
                    {
                        begin_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, param.axis_begin_pos_vec[max_i], param.axis_pos_vec[max_i],
                            param.axis_vel_vec[max_i] / 1000 * param.pos_ratio[max_i], param.axis_acc_vec[max_i] / 1000 / 1000 * param.pos_ratio[max_i] * param.pos_ratio[max_i], param.axis_jerk_vec[max_i] / 1000 / 1000 / 1000 * param.pos_ratio[max_i] * param.pos_ratio[max_i] * param.pos_ratio[max_i],
                            p, v, a, j, param.total_count[max_i]);
                    }
                    else
                    {
                        end_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, param.axis_begin_pos_vec[max_i], param.axis_pos_vec[max_i],
                            param.axis_vel_vec[max_i] / 1000 * param.pos_ratio[max_i], param.axis_acc_vec[max_i] / 1000 / 1000 * param.pos_ratio[max_i] * param.pos_ratio[max_i], param.axis_jerk_vec[max_i] / 1000 / 1000 / 1000 * param.pos_ratio[max_i] * param.pos_ratio[max_i] * param.pos_ratio[max_i],
                            p, v, a, j, param.total_count[max_i]);
                    }
                    if ((begin_count == end_count) || (begin_count + 1 == end_count) || (end_count == 0))
                    {
                        break;
                    }
                }
            }
        }

        //更新转弯区//
        if (param.pre_plan == nullptr)//转弯第一条指令
        {
            //更新本条指令的planzone//
            this->planzone.store(target_count);
        }
        else if (param.pre_plan->name() == this->name())//转弯第二或第n条指令
        {
            //更新本plan的planzone//
            this->planzone.store(target_count);
            //更新上一条转弯指令的realzone//
            if (param.pre_plan->cmd_executing.load())
            {
                param.pre_plan->realzone.store(0);
            }
            else
            {
                uint32_t min_zone = std::min(param.pre_plan->planzone.load(), param.max_total_count / 2);//当前指令所需count数/2
                param.pre_plan->realzone.store(min_zone);
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveL,moveC,moveAbsJ
        {
            //更新本plan的planzone//
            this->planzone.store(target_count);
            //上一条指令不进行转弯//
            param.pre_plan->realzone.store(0);
        }

        this->param() = param;
        for (auto &option : motorOptions()) option |= aris::plan::Plan::NOT_CHECK_ENABLE | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_CONTINUOUS;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MoveAbsJ::executeRT()->int
    {
        this->cmd_executing.load();
        auto param = std::any_cast<MoveAbsJParam>(&this->param());
        auto &pwinter = dynamic_cast<aris::server::ProgramWebInterface&>(controlServer()->interfacePool().at(0));
        /*
        if (count() == 1)
        {
            for (Size i = 0; i < param->active_motor.size(); ++i)
            {
                if (param->active_motor[i])
                {
                    param->axis_begin_pos_vec[i] = controller()->motionPool()[i].targetPos();
                    //param->axis_begin_pos_vec[i] = model()->motionPool()[i].mp();
                }
            }
        }

        aris::Size total_count{ 1 };
        for (Size i = 0; i < param->active_motor.size(); ++i)
        {
            if (param->active_motor[i])
            {
                double p, v, a, j;
                aris::Size t_count;
                //aris::plan::moveAbsolute(count(),
                //	param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                //	param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_dec_vec[i] / 1000 / 1000,
                //	p, v, a, t_count);

                //s形规划//
                traplan::sCurve(static_cast<double>(count()), param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                    param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_jerk_vec[i] / 1000 / 1000 / 1000,
                    p, v, a, j, t_count);
                controller()->motionPool()[i].setTargetPos(p);
                model()->motionPool()[i].setMp(p);
                total_count = std::max(total_count, t_count);
            }
        }
        if (model()->solverPool().at(1).kinPos())return -1;

        if (g_move_pause)
        {
            return -4;
        }

        return total_count - count();
        */

        //如果停止功能开启，并且时间已经停止，退出本条指令//
        if (count() == 1)
        {
            if (pwinter.isAutoStopped() && (g_counter == 0))
            {
                this->cmd_finished.store(true);
                g_plan = nullptr;
                return 0;
            }
        }

        //暂停、恢复//
        PauseContinueB(this, pwinter);

        if (param->pre_plan == nullptr) //转弯第一条指令
        {
            if (count() == 1)
            {
                // init joint_pos //
                for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
                {
                    param->axis_begin_pos_vec[i] = controller()->motionPool()[i].targetPos();
                }
            }
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                //S形轨迹规划//
                double p, v, a, j;
                traplan::sCurve(g_count, param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                    param->axis_vel_vec[i] / 1000 * param->pos_ratio[i], param->axis_acc_vec[i] / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i], param->axis_jerk_vec[i] / 1000 / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i] * param->pos_ratio[i],
                    p, v, a, j, param->total_count[i]);

                controller()->motionPool()[i].setTargetPos(p);
                model()->motionPool()[i].setMp(p);
            }
        }
        else if (param->pre_plan->name() == this->name()) //转弯区第二条指令或者第n条指令
        {
            auto preparam = std::any_cast<MoveAbsJParam&>(param->pre_plan->param());
            auto zonecount = param->pre_plan->realzone.load();
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                //preplan//
                double prep = 0.0, prev, prea, prej;

                //count数小于等于上一条指令的realzone，zone起作用//
                if (g_count <= zonecount)
                {
                    //lastplan//
                    traplan::sCurve(preparam.max_total_count - zonecount + g_count, preparam.axis_begin_pos_vec[i], preparam.axis_pos_vec[i],
                        preparam.axis_vel_vec[i] / 1000 * preparam.pos_ratio[i], preparam.axis_acc_vec[i] / 1000 / 1000 * preparam.pos_ratio[i] * preparam.pos_ratio[i], preparam.axis_jerk_vec[i] / 1000 / 1000 / 1000 * preparam.pos_ratio[i] * preparam.pos_ratio[i] * preparam.pos_ratio[i],
                        prep, prev, prea, prej, preparam.total_count[i]);

                    //thisplan//
                    double p, v, a, j;
                    traplan::sCurve(g_count, param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                        param->axis_vel_vec[i] / 1000 * param->pos_ratio[i], param->axis_acc_vec[i] / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i], param->axis_jerk_vec[i] / 1000 / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i] * param->pos_ratio[i],
                        p, v, a, j, param->total_count[i]);

                    controller()->motionPool()[i].setTargetPos(1.0*(zonecount - g_count) / zonecount * prep + 1.0*g_count / zonecount * p);
                    model()->motionPool()[i].setMp(1.0*(zonecount - g_count) / zonecount * prep + 1.0*g_count / zonecount * p);
                }
                else
                {
                    //thisplan//
                    param->pre_plan->cmd_finished.store(true);
                    double p, v, a, j;
                    traplan::sCurve(g_count, param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                        param->axis_vel_vec[i] / 1000 * param->pos_ratio[i], param->axis_acc_vec[i] / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i], param->axis_jerk_vec[i] / 1000 / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i] * param->pos_ratio[i],
                        p, v, a, j, param->total_count[i]);

                    controller()->motionPool()[i].setTargetPos(p);
                    model()->motionPool()[i].setMp(p);
                }
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveL,moveC,moveAbsJ
        {
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                //S形轨迹规划//
                double p, v, a, j;
                traplan::sCurve(g_count, param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
                    param->axis_vel_vec[i] / 1000 * param->pos_ratio[i], param->axis_acc_vec[i] / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i], param->axis_jerk_vec[i] / 1000 / 1000 / 1000 * param->pos_ratio[i] * param->pos_ratio[i] * param->pos_ratio[i],
                    p, v, a, j, param->total_count[i]);

                controller()->motionPool()[i].setTargetPos(p);
                model()->motionPool()[i].setMp(p);
            }
        }

        if (model()->solverPool().at(1).kinPos())return -1;

        auto rzcount = this->realzone.load();

        //本条指令没有转弯区
        PauseContinueE(param, pwinter, rzcount);

        if (param->max_total_count - rzcount - int32_t(g_count) == 0)
        {
            //realzone为0时，返回值为0时，本条指令执行完毕
            if(rzcount == 0)this->cmd_finished.store(true);
            controller()->mout() << "param->max_total_count:" << param->max_total_count << "this->realzone.load():" << rzcount << std::endl;
        }
        if (pwinter.isAutoStopped() && (g_counter == 0))
        {
            //指令停止且返回值为0时，本条指令执行完毕
            this->cmd_finished.store(true);
            g_plan.reset();
            return 0;
        }
        return param->max_total_count == 0 ? 0 : param->max_total_count - rzcount - int32_t(g_count);

    }
    auto MoveAbsJ::collectNrt()->void
    {
        if(this->retCode()<0)
        {
            g_plan.reset();
        }
        std::any_cast<MoveAbsJParam>(&this->param())->pre_plan.reset();
    }
    MoveAbsJ::~MoveAbsJ() = default;
    MoveAbsJ::MoveAbsJ(const std::string &name) : MoveBase(name)
    {
        command().loadXmlStr(
            "<Command name=\"mvaj\">"
            "	<GroupParam>"
            "		<Param name=\"pos\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
            "		<Param name=\"vel\" default=\"{0.025, 0.025, 3.4, 0.0, 0.0}\"/>"
            "		<Param name=\"acc\" default=\"0.5\"/>"
            "		<Param name=\"dec\" default=\"0.5\"/>"
            "		<Param name=\"jerk\" default=\"10.0\"/>"
            "		<Param name=\"zone\" default=\"fine\"/>"
            "		<Param name=\"load\" default=\"{1,0.05,0.05,0.05,0,0.97976,0,0.200177,1.0,1.0,1.0}\"/>"
            "		<Param name=\"tool\" default=\"tool0\"/>"
            "		<Param name=\"wobj\" default=\"wobj0\"/>"
            SELECT_MOTOR_STRING
            CHECK_PARAM_STRING
            "	</GroupParam>"
            "</Command>");
    }


    //轴空间//
    auto check_eul_validity(const std::string &eul_type)->bool
    {
        if (eul_type.size() < 3)return false;

        for (int i = 0; i < 3; ++i)if (eul_type[i] > '3' || eul_type[i] < '1')return false;

        if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

        return true;
    }
    auto find_pq(const std::map<std::string_view, std::string_view> &params, Plan &plan, double *pq_out)->bool
    {
        double pos_unit;
        auto pos_unit_found = params.find("pos_unit");
        if (pos_unit_found == params.end()) pos_unit = 1.0;
        else if (pos_unit_found->second == "m")pos_unit = 1.0;
        else if (pos_unit_found->second == "mm")pos_unit = 0.001;
        else if (pos_unit_found->second == "cm")pos_unit = 0.01;
        else THROW_FILE_LINE("");

        auto &cal = plan.controlServer()->model().calculator();
        for (auto cmd_param : params)
        {
            if (cmd_param.first == "pq")
            {
                auto pq_mat = std::any_cast<aris::core::Matrix>(cal.calculateExpression("robtarget(" + std::string(cmd_param.second) + ")").second);
                if (pq_mat.size() != 7)THROW_FILE_LINE("");
                aris::dynamic::s_vc(7, pq_mat.data(), pq_out);
                aris::dynamic::s_nv(3, pos_unit, pq_out);
                return true;
            }
            else if (cmd_param.first == "pm")
            {
                auto pm_mat = plan.matrixParam(cmd_param.first);
                if (pm_mat.size() != 16)THROW_FILE_LINE("");
                aris::dynamic::s_pm2pq(pm_mat.data(), pq_out);
                aris::dynamic::s_nv(3, pos_unit, pq_out);
                return true;
            }
            else if (cmd_param.first == "pe")
            {
                double ori_unit;
                auto ori_unit_found = params.find("ori_unit");
                if (ori_unit_found == params.end()) ori_unit = 1.0;
                else if (ori_unit_found->second == "rad")ori_unit = 1.0;
                else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
                else THROW_FILE_LINE("");

                std::string eul_type;
                auto eul_type_found = params.find("eul_type");
                if (eul_type_found == params.end()) eul_type = "321";
                else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
                else THROW_FILE_LINE("");

                auto pe_mat = plan.matrixParam(cmd_param.first);
                if (pe_mat.size() != 6)THROW_FILE_LINE("");
                aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
                aris::dynamic::s_pe2pq(pe_mat.data(), pq_out, eul_type.data());
                aris::dynamic::s_nv(3, pos_unit, pq_out);
                return true;
            }
        }

        THROW_FILE_LINE("No pose input");
    }
    struct MoveJ::Imp {};
    auto MoveJ::prepareNrt()->void
    {
        set_check_option(cmdParams(), *this);

        MoveJParam mvj_param;

        // find ee pq //
        mvj_param.ee_pq.resize(7);
        find_pq(cmdParams(), *this, mvj_param.ee_pq.data());

        mvj_param.joint_pos_begin.resize(model()->motionPool().size(), 0.0);
        mvj_param.joint_pos_end.resize(model()->motionPool().size(), 0.0);
        mvj_param.total_count.resize(model()->motionPool().size(), 0);
        mvj_param.pos_ratio.resize(model()->motionPool().size(), 1.0);

        mvj_param.tool = &*model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        mvj_param.wobj = &*model()->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));
        g_tool = &*g_model.generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        g_wobj = &*g_model.generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));

        auto &cal = this->controlServer()->model().calculator();
        // find joint acc/vel/dec/jerk/zone
        for (auto cmd_param : cmdParams())
        {
            auto c = controller();
            if (cmd_param.first == "joint_acc")
            {
                mvj_param.joint_acc.clear();
                mvj_param.joint_acc.resize(model()->motionPool().size(), 0.0);

                auto acc_mat = matrixParam(cmd_param.first);
                if (acc_mat.size() == 1)std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(), acc_mat.toDouble());
                else if (acc_mat.size() == model()->motionPool().size()) std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
                else THROW_FILE_LINE("");

                for (int i = 0; i < 6; ++i)mvj_param.joint_acc[i] *= controller()->motionPool()[i].maxAcc();

                // check value validity //
                for (Size i = 0; i < std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
                    if (mvj_param.joint_acc[i] <= 0 || mvj_param.joint_acc[i] > c->motionPool()[i].maxAcc())
                        THROW_FILE_LINE("");
            }
            else if (cmd_param.first == "vel")
            {
                mvj_param.joint_vel.clear();
                mvj_param.joint_vel.resize(model()->motionPool().size(), 0.0);

                auto[type, value] = cal.calculateExpression("speed(" + std::string(cmd_param.second) + ")");
                auto s = std::any_cast<kaanh::Speed>(cal.calculateExpression("speed(" + std::string(cmd_param.second) + ")").second);
                mvj_param.sp = s;
                for (int i = 0; i < model()->motionPool().size(); ++i)
                {
                    mvj_param.joint_vel[i] = controller()->motionPool()[i].maxVel()*mvj_param.sp.w_per;
                }

                // check value validity //
                for (Size i = 0; i < std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
                    if (mvj_param.joint_vel[i] <= 0 || mvj_param.joint_vel[i] > c->motionPool()[i].maxVel())
                        THROW_FILE_LINE("");
            }
            else if (cmd_param.first == "joint_dec")
            {
                mvj_param.joint_dec.clear();
                mvj_param.joint_dec.resize(model()->motionPool().size(), 0.0);

                auto dec_mat = matrixParam(cmd_param.first);
                if (dec_mat.size() == 1)std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(), dec_mat.toDouble());
                else if (dec_mat.size() == model()->motionPool().size()) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
                else THROW_FILE_LINE("");

                for (int i = 0; i < 6; ++i) mvj_param.joint_dec[i] *= controller()->motionPool()[i].maxAcc();

                // check value validity //
                for (Size i = 0; i < std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
                    if (mvj_param.joint_dec[i] <= 0 || mvj_param.joint_dec[i] > c->motionPool()[i].maxAcc())
                        THROW_FILE_LINE("");
            }
            else if (cmd_param.first == "joint_jerk")
            {
                mvj_param.joint_jerk.clear();
                mvj_param.joint_jerk.resize(model()->motionPool().size(), 0.0);

                auto dec_mat = matrixParam(cmd_param.first);
                if (dec_mat.size() == 1)std::fill(mvj_param.joint_jerk.begin(), mvj_param.joint_jerk.end(), dec_mat.toDouble());
                else if (dec_mat.size() == model()->motionPool().size()) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_jerk.begin());
                else THROW_FILE_LINE("");

                for (int i = 0; i < 6; ++i) mvj_param.joint_jerk[i] *= mvj_param.joint_acc[i];

                // check value validity //
                for (Size i = 0; i < std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
                    if (mvj_param.joint_jerk[i] <= 0 || mvj_param.joint_jerk[i] > 1000*c->motionPool()[i].maxAcc())
                        THROW_FILE_LINE("");
            }
            else if (cmd_param.first == "zone")
            {
                if (cmd_param.second == "fine")//不设置转弯区
                {
                    mvj_param.zone.dis = 0.0;
                    mvj_param.zone.per = 0.0;
                    mvj_param.zone_enabled = 0;
                }
                else//设置转弯区
                {
                    auto z = std::any_cast<kaanh::Zone>(cal.calculateExpression("zone(" + std::string(cmd_param.second) + ")").second);
                    mvj_param.zone = z;
                    mvj_param.zone_enabled = 1;
                    if (mvj_param.zone.per > 0.5&&mvj_param.zone.per<0.001)
                    {
                        THROW_FILE_LINE("zone out of range");
                    }
                }
            }
            else if (cmd_param.first == "load")
            {
                auto temp = std::any_cast<kaanh::Load>(cal.calculateExpression("load(" + std::string(cmd_param.second) + ")").second);
                mvj_param.ld = temp;
            }
        }

        //本指令cmdid-上一条指令的cmdid>1，且上一条指令执行完毕，g_plan赋值为nullptr
        if (mvj_param.pre_plan != nullptr)
        {
            if ((this->cmdId() - mvj_param.pre_plan->cmdId() > 1) && (mvj_param.pre_plan->cmd_finished.load()))g_plan = nullptr;
        }
        //更新全局变量g_plan//
        mvj_param.pre_plan = g_plan;
        g_plan = std::dynamic_pointer_cast<MoveBase>(sharedPtrForThis());//------这里需要潘博aris库提供一个plan接口，返回std::shared_ptr<MoveBase>类型指针

        if (mvj_param.pre_plan == nullptr)
        {
            std::cout << "preplan:" << "nullptr" << std::endl;
        }
        else
        {
            std::cout << "preplan:" << mvj_param.pre_plan->name() << std::endl;
        }

        // 设置转弯区 //
        double p, v, a, j;
        double end_pm[16];
        if (mvj_param.pre_plan == nullptr)//转弯第一条指令
        {
            for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
            {
                mvj_param.joint_pos_begin[i] = controller()->motionPool()[i].targetPos();
            }
        }
        else if (std::string(mvj_param.pre_plan->name()) == this->name())//转弯第二或第n条指令
        {
            std::cout << "precmdname1:" << mvj_param.pre_plan->name() << "  cmdname1:"<< this->name() <<std::endl;
            //从全局变量中获取上一条转弯区指令的目标点//
            aris::dynamic::s_pq2pm(std::any_cast<MoveJParam>(&mvj_param.pre_plan->param())->ee_pq.data(), end_pm);
            /*
            mvj_param.tool->setPm(*mvj_param.wobj, end_pm);
            model()->generalMotionPool().at(0).updMpm();
            if (model()->solverPool().at(0).kinPos())THROW_FILE_LINE("");
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                mvj_param.joint_pos_begin[i] = model()->motionPool()[i].mp();
            }
            */
            g_tool->setPm(*g_wobj, end_pm);
            g_model.generalMotionPool().at(0).updMpm();
            if (g_model.solverPool().at(0).kinPos())THROW_FILE_LINE("");
            for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
            {
                mvj_param.joint_pos_begin[i] = g_model.motionPool()[i].mp();
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveL,moveC,moveAbsJ
        {
            std::cout << "precmdname2:" << mvj_param.pre_plan->name() << "  cmdname2:" << this->name() << std::endl;
            //获取起始点位置//
            if (std::string(mvj_param.pre_plan->name()) == "MoveAbsJ")
            {
                auto preparam = std::any_cast<MoveAbsJParam>(&mvj_param.pre_plan->param());
                for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
                {
                    mvj_param.joint_pos_begin[i] = preparam->axis_pos_vec[i];
                }
            }
            if (std::string(mvj_param.pre_plan->name()) == "MoveL")
            {
                auto preparam = std::any_cast<MoveLParam>(&mvj_param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_pq.data(), end_pm);
                g_tool->setPm(*g_wobj, end_pm);
                g_model.generalMotionPool().at(0).updMpm();
                if (g_model.solverPool().at(0).kinPos())THROW_FILE_LINE("");
                for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
                {
                    mvj_param.joint_pos_begin[i] = g_model.motionPool()[i].mp();
                }
            }
            if (std::string(mvj_param.pre_plan->name()) == "MoveC")
            {
                auto preparam = std::any_cast<MoveCParam>(&mvj_param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_end_pq.data(), end_pm);
                g_tool->setPm(*g_wobj, end_pm);
                g_model.generalMotionPool().at(0).updMpm();
                if (g_model.solverPool().at(0).kinPos())THROW_FILE_LINE("");
                for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
                {
                    mvj_param.joint_pos_begin[i] = g_model.motionPool()[i].mp();
                }
            }

        }

        //获取终止点位置//
        aris::dynamic::s_pq2pm(mvj_param.ee_pq.data(), end_pm);
        /*
        mvj_param.tool->setPm(*mvj_param.wobj, end_pm);
        model()->generalMotionPool().at(0).updMpm();
        if (model()->solverPool().at(0).kinPos())THROW_FILE_LINE("");
        */
        g_tool->setPm(*g_wobj, end_pm);
        g_model.generalMotionPool().at(0).updMpm();
        //dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(g_model.solverPool().at(0)).setWhichRoot(0);
        if (g_model.solverPool().at(0).kinPos())THROW_FILE_LINE("");
        //计算转弯区对应的count数//
        double max_pos = 0.0;
        Size max_i = 0;
        for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
        {
            mvj_param.joint_pos_end[i] = g_model.motionPool()[i].mp();

            //S形轨迹规划//
            traplan::sCurve(1, mvj_param.joint_pos_begin[i], mvj_param.joint_pos_end[i],
                mvj_param.joint_vel[i] / 1000, mvj_param.joint_acc[i] / 1000 / 1000, mvj_param.joint_jerk[i] / 1000 / 1000 / 1000,
                p, v, a, j, mvj_param.total_count[i]);

            if (std::abs(max_pos) < std::abs(mvj_param.joint_pos_end[i] - mvj_param.joint_pos_begin[i]))
            {
                max_pos = mvj_param.joint_pos_end[i] - mvj_param.joint_pos_begin[i];
                max_i = i;
            }
        }
        //本条指令的最大规划count数//
        mvj_param.max_total_count = *std::max_element(mvj_param.total_count.begin(), mvj_param.total_count.end());

        //获取不同轴的比率//
        for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
        {
            mvj_param.pos_ratio[i] = 1.0* mvj_param.total_count[i] / mvj_param.max_total_count;
        }

        for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
        {
            //S形轨迹规划//
            traplan::sCurve(1, mvj_param.joint_pos_begin[i], mvj_param.joint_pos_end[i],
                mvj_param.joint_vel[i] / 1000 * mvj_param.pos_ratio[i], mvj_param.joint_acc[i] / 1000 / 1000 * mvj_param.pos_ratio[i] * mvj_param.pos_ratio[i], mvj_param.joint_jerk[i] / 1000 / 1000 / 1000 * mvj_param.pos_ratio[i] * mvj_param.pos_ratio[i] * mvj_param.pos_ratio[i],
                p, v, a, j, mvj_param.total_count[i]);
        }
        mvj_param.max_total_count = *std::max_element(mvj_param.total_count.begin(), mvj_param.total_count.end());

        //二分法//
        auto pos_zone = mvj_param.joint_pos_end[max_i] - max_pos * mvj_param.zone.per;
        aris::Size begin_count = 1, target_count = 0, end_count;
        end_count = mvj_param.max_total_count;
        if (mvj_param.zone_enabled)
        {
            if (std::abs(max_pos) > 2e-3)//转弯半径大于0.002rad
            {
                while (std::abs(p - pos_zone) > 1e-9)
                {
                    if (p < pos_zone)
                    {
                        begin_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, mvj_param.joint_pos_begin[max_i], mvj_param.joint_pos_end[max_i],
                            mvj_param.joint_vel[max_i] / 1000 * mvj_param.pos_ratio[max_i], mvj_param.joint_acc[max_i] / 1000 / 1000 * mvj_param.pos_ratio[max_i] * mvj_param.pos_ratio[max_i], mvj_param.joint_jerk[max_i] / 1000 / 1000 / 1000 * mvj_param.pos_ratio[max_i] * mvj_param.pos_ratio[max_i] * mvj_param.pos_ratio[max_i],
                            p, v, a, j, mvj_param.total_count[max_i]);
                    }
                    else
                    {
                        end_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, mvj_param.joint_pos_begin[max_i], mvj_param.joint_pos_end[max_i],
                            mvj_param.joint_vel[max_i] / 1000 * mvj_param.pos_ratio[max_i], mvj_param.joint_acc[max_i] / 1000 / 1000 * mvj_param.pos_ratio[max_i] * mvj_param.pos_ratio[max_i], mvj_param.joint_jerk[max_i] / 1000 / 1000 / 1000 * mvj_param.pos_ratio[max_i] * mvj_param.pos_ratio[max_i] * mvj_param.pos_ratio[max_i],
                            p, v, a, j, mvj_param.total_count[max_i]);
                    }
                    if ((begin_count == end_count) || (begin_count + 1 == end_count) || (end_count == 0))
                    {
                        break;
                    }
                }
            }
        }


        //更新转弯区//
        if (mvj_param.pre_plan == nullptr)//转弯第一条指令
        {
            //更新本条指令的planzone//
            this->planzone.store(mvj_param.max_total_count*mvj_param.zone.per);
        }
        else if (mvj_param.pre_plan->name() == this->name())//转弯第二或第n条指令
        {
            //更新本plan的planzone//
            this->planzone.store(mvj_param.max_total_count*mvj_param.zone.per);
            //更新上一条转弯指令的realzone//
            if (mvj_param.pre_plan->cmd_executing.load())
            {
                mvj_param.pre_plan->realzone.store(0);
            }
            else
            {
                Size min_zone = std::min(mvj_param.pre_plan->planzone.load(), mvj_param.max_total_count / 2);//当前指令所需count数/2
                mvj_param.pre_plan->realzone.store(min_zone);
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveL,moveC,moveAbsJ
        {
            //更新本plan的planzone//
            this->planzone.store(mvj_param.max_total_count*mvj_param.zone.per);
            //上一条指令不进行转弯//
            mvj_param.pre_plan->realzone.store(0);
        }

        this->param() = mvj_param;
        std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS);

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MoveJ::executeRT()->int
    {
        this->cmd_executing.load();
        auto mvj_param = std::any_cast<MoveJParam>(&this->param());
        auto &pwinter = dynamic_cast<aris::server::ProgramWebInterface&>(controlServer()->interfacePool().at(0));
        /*
        // 取得起始位置 //
        double p, v, a, j;
        if (count() == 1)
        {
            // inverse kinematic //
            double end_pm[16];
            aris::dynamic::s_pq2pm(mvj_param->ee_pq.data(), end_pm);
            //model()->generalMotionPool().at(0).setMpm(end_pm);
            mvj_param->tool->setPm(*mvj_param->wobj, end_pm);
            model()->generalMotionPool().at(0).updMpm();

            if (model()->solverPool().at(0).kinPos())return -1;

            // init joint_pos //
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                mvj_param->joint_pos_begin[i] = controller()->motionPool()[i].targetPos();
                mvj_param->joint_pos_end[i] = model()->motionPool()[i].mp();
                //梯形轨迹规划//
                //aris::plan::moveAbsolute(count(), mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i]
                //	, mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000
                //	, p, v, a, mvj_param->total_count[i]);

                //S形轨迹规划//
                traplan::sCurve(static_cast<double>(count()), mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                    mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_jerk[i] / 1000 / 1000 / 1000,
                    p, v, a, j, mvj_param->total_count[i]);
            }

            mvj_param->max_total_count = *std::max_element(mvj_param->total_count.begin(), mvj_param->total_count.end());
        }

        for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
        {
            aris::plan::moveAbsolute(static_cast<double>(count()) * mvj_param->total_count[i] / mvj_param->max_total_count,
                mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000,
                p, v, a, mvj_param->total_count[i]);

            controller()->motionPool()[i].setTargetPos(p);
            model()->motionPool()[i].setMp(p);
        }
        if (model()->solverPool().at(1).kinPos())return -1;

        if (g_move_pause)
        {
            return -4;
        }
        return mvj_param->max_total_count == 0 ? 0 : mvj_param->max_total_count - count();
        */

        //如果停止功能开启，并且时间已经停止，退出本条指令//
        if (count() == 1)
        {
            if (pwinter.isAutoStopped() && (g_counter == 0))
            {
                this->cmd_finished.store(true);
                g_plan = nullptr;
                return 0;
            }
        }

        //暂停、恢复//
        PauseContinueB(this, pwinter);

        if (mvj_param->pre_plan == nullptr) //转弯第一条指令
        {
            if (count() == 1)
            {
                // init joint_pos //
                for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
                {
                    mvj_param->joint_pos_begin[i] = controller()->motionPool()[i].targetPos();
                }
            }
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                //梯形轨迹规划//
                //aris::plan::moveAbsolute(static_cast<double>(count()) * mvj_param->total_count[i] / mvj_param->max_total_count,
                //	mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                //	mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000,
                //	p, v, a, mvj_param->total_count[i]);
                //S形轨迹规划//
                double p, v, a, j;
                traplan::sCurve(g_count, mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                    mvj_param->joint_vel[i] / 1000 * mvj_param->pos_ratio[i], mvj_param->joint_acc[i] / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i], mvj_param->joint_jerk[i] / 1000 / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i],
                    p, v, a, j, mvj_param->total_count[i]);

                controller()->motionPool()[i].setTargetPos(p);
                model()->motionPool()[i].setMp(p);
            }
        }
        else if (mvj_param->pre_plan->name() == this->name()) //转弯区第二条指令或者第n条指令
        {

            auto param = std::any_cast<MoveJParam&>(mvj_param->pre_plan->param());
            auto zonecount = mvj_param->pre_plan->realzone.load();
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                //preplan//
                double prep = 0.0, prev, prea, prej;

                //count数小于等于上一条指令的realzone，zone起作用//
                if (g_count <= zonecount)
                {
                    //lastplan//
                    traplan::sCurve(param.max_total_count - zonecount + g_count,
                        param.joint_pos_begin[i], param.joint_pos_end[i],
                        param.joint_vel[i] / 1000 * param.pos_ratio[i], param.joint_acc[i] / 1000 / 1000 * param.pos_ratio[i] * param.pos_ratio[i], param.joint_jerk[i] / 1000 / 1000 / 1000 * param.pos_ratio[i] * param.pos_ratio[i] * param.pos_ratio[i],
                        prep, prev, prea, prej, param.total_count[i]);

                    //thisplan//
                    double p, v, a, j;
                    traplan::sCurve(g_count,mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                        mvj_param->joint_vel[i] / 1000 * mvj_param->pos_ratio[i], mvj_param->joint_acc[i] / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i], mvj_param->joint_jerk[i] / 1000 / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i],
                        p, v, a, j, mvj_param->total_count[i]);

                    controller()->motionPool()[i].setTargetPos(1.0*(zonecount - g_count) / zonecount * prep + 1.0*g_count / zonecount * p);
                    model()->motionPool()[i].setMp(1.0*(zonecount - g_count) / zonecount * prep + 1.0*g_count / zonecount * p);
                }
                else
                {
                    //thisplan//
                    mvj_param->pre_plan->cmd_finished.store(true);
                    double p, v, a, j;
                    traplan::sCurve(g_count, mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                        mvj_param->joint_vel[i] / 1000 * mvj_param->pos_ratio[i], mvj_param->joint_acc[i] / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i], mvj_param->joint_jerk[i] / 1000 / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i],
                        p, v, a, j, mvj_param->total_count[i]);

                    if (std::abs(p) > 7)
                    {
                        controller()->mout() << "p:" << p << std::endl;
                    }
                    controller()->motionPool()[i].setTargetPos(p);
                    model()->motionPool()[i].setMp(p);
                }
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveL,moveC,moveAbsJ
        {
            for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
            {
                //S形轨迹规划//
                double p, v, a, j;
                traplan::sCurve(g_count, mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
                    mvj_param->joint_vel[i] / 1000 * mvj_param->pos_ratio[i], mvj_param->joint_acc[i] / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i], mvj_param->joint_jerk[i] / 1000 / 1000 / 1000 * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i] * mvj_param->pos_ratio[i],
                    p, v, a, j, mvj_param->total_count[i]);

                controller()->motionPool()[i].setTargetPos(p);
                model()->motionPool()[i].setMp(p);
            }
        }
        if (model()->solverPool().at(1).kinPos())
        {
            controller()->mout() << "fanjie-----" << std::endl;
            return -1;
        }

        auto rzcount = this->realzone.load();

        //本条指令没有转弯区
        PauseContinueE(mvj_param, pwinter, rzcount);

        if (mvj_param->max_total_count - rzcount - int32_t(g_count) == 0)
        {
            //realzone为0时，返回值为0时，本条指令执行完毕
            if (rzcount == 0)this->cmd_finished.store(true);
            controller()->mout() << "mvj_param->max_total_count:" << mvj_param->max_total_count <<"this->realzone.load():" << rzcount << std::endl;
        }
        if (pwinter.isAutoStopped() && (g_counter == 0))
        {
            //指令停止且返回值为0时，本条指令执行完毕
            this->cmd_finished.store(true);
            g_plan.reset();
            return 0;
        }
        return mvj_param->max_total_count == 0 ? 0 : mvj_param->max_total_count - rzcount - int32_t(g_count);

    }
    auto MoveJ::collectNrt()->void
    {
        if(this->retCode()<0)
        {
            g_plan.reset();
        }
        std::any_cast<MoveJParam>(&this->param())->pre_plan.reset();
    }
    MoveJ::~MoveJ() = default;
    MoveJ::MoveJ(const MoveJ &other) = default;
    MoveJ::MoveJ(const std::string &name) :MoveBase(name)
    {
        command().loadXmlStr(
            "<Command name=\"mvj\">"
            "	<GroupParam>"
            "		<Param name=\"pos_unit\" default=\"m\"/>"
            "		<UniqueParam default=\"pq\">"
            "			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
            "			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
            "			<GroupParam>"
            "				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
            "				<Param name=\"ori_unit\" default=\"rad\"/>"
            "				<Param name=\"eul_type\" default=\"321\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "		<Param name=\"joint_acc\" default=\"0.5\"/>"
            "		<Param name=\"vel\" default=\"{0.025, 0.025, 3.4, 0.0, 0.0}\"/>"
            "		<Param name=\"joint_dec\" default=\"0.5\"/>"
            "		<Param name=\"joint_jerk\" default=\"10.0\"/>"
            "		<Param name=\"speed\" default=\"{0.1, 0.1, 3.49, 0.0, 0.0}\"/>"
            "		<Param name=\"zone\" default=\"fine\"/>"
            "		<Param name=\"load\" default=\"{1,0.05,0.05,0.05,0,0.97976,0,0.200177,1.0,1.0,1.0}\"/>"
            "		<Param name=\"tool\" default=\"tool0\"/>"
            "		<Param name=\"wobj\" default=\"wobj0\"/>"
            CHECK_PARAM_STRING
            "	</GroupParam>"
            "</Command>");
    }


    //走直线//
    struct MoveL::Imp {};
    auto MoveL::prepareNrt()->void
    {
        set_check_option(cmdParams(), *this);

        MoveLParam mvl_param;
        mvl_param.ee_pq.resize(7);
        mvl_param.relative_pa.resize(6, 0.0);
        mvl_param.total_count.resize(model()->motionPool().size(), 0);

        if (!find_pq(cmdParams(), *this, mvl_param.ee_pq.data()))THROW_FILE_LINE("");

        mvl_param.tool = &*model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        mvl_param.wobj = &*model()->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));
        g_tool = &*g_model.generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        g_wobj = &*g_model.generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));

        auto &cal = this->controlServer()->model().calculator();
        for (auto cmd_param : cmdParams())
        {
            if (cmd_param.first == "acc")
            {
                mvl_param.acc = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "vel")
            {
                auto s = std::any_cast<kaanh::Speed>(cal.calculateExpression("speed(" + std::string(cmd_param.second) + ")").second);
                mvl_param.sp = s;
                mvl_param.vel = mvl_param.sp.v_tcp;
                mvl_param.angular_vel = mvl_param.sp.w_tcp;
                mvl_param.angular_vel = 0.52;//调试
            }
            else if (cmd_param.first == "dec")
            {
                mvl_param.dec = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "jerk")
            {
                mvl_param.jerk = doubleParam(cmd_param.first)*mvl_param.acc;
            }
            else if (cmd_param.first == "angular_acc")
            {
                mvl_param.angular_acc = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "angular_dec")
            {
                mvl_param.angular_dec = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "angular_jerk")
            {
                mvl_param.angular_jerk = doubleParam(cmd_param.first)*mvl_param.angular_acc;
            }
            else if (cmd_param.first == "zone")
            {
                if (cmd_param.second == "fine")//不设置转弯区
                {
                    mvl_param.zone.dis = 0.0;
                    mvl_param.zone.per = 0.0;
                    mvl_param.zone_enabled = 0;
                }
                else//设置转弯区
                {
                    auto z = std::any_cast<kaanh::Zone>(cal.calculateExpression("zone(" + std::string(cmd_param.second) + ")").second);
                    mvl_param.zone = z;
                    mvl_param.zone_enabled = 1;
                    if (mvl_param.zone.dis > 0.2&&mvl_param.zone.dis < 0.001)
                    {
                        THROW_FILE_LINE("zone out of range");
                    }
                }
            }
            else if (cmd_param.first == "load")
            {
                auto temp = std::any_cast<kaanh::Load>(cal.calculateExpression("load(" + std::string(cmd_param.second) + ")").second);
                mvl_param.ld = temp;
            }
        }

        //本指令cmdid-上一条指令的cmdid>1，且上一条指令执行完毕，g_plan赋值为nullptr
        if (mvl_param.pre_plan != nullptr)
        {
            if ((this->cmdId() - mvl_param.pre_plan->cmdId() > 1) && (mvl_param.pre_plan->cmd_finished.load()))g_plan = nullptr;
        }

        //更新全局变量g_plan//
        mvl_param.pre_plan = g_plan;
        g_plan = std::dynamic_pointer_cast<MoveBase>(sharedPtrForThis());//------这里需要潘博aris库提供一个plan接口，返回std::shared_ptr<MoveBase>类型指针

        if (mvl_param.pre_plan == nullptr)
        {
            std::cout << "preplan:" << "nullptr" << std::endl;
        }
        else
        {
            std::cout << "preplan:" << mvl_param.pre_plan->name() << std::endl;
        }

        // 获取起始点位姿 //
        double end_pm[16], relative_pm[16];
        if (mvl_param.pre_plan == nullptr)//转弯第一条指令
        {
            g_model.generalMotionPool().at(0).updMpm();
            g_tool->getPm(*g_wobj, mvl_param.begin_pm);
        }
        else if (std::string(mvl_param.pre_plan->name()) == "MoveL")//转弯第二或第n条指令
        {
            std::cout << "precmdname1:" << mvl_param.pre_plan->name() << "  cmdname1:" << this->name() << std::endl;
            //从全局变量中获取上一条转弯区指令的目标点//
            aris::dynamic::s_pq2pm(std::any_cast<MoveLParam>(&mvl_param.pre_plan->param())->ee_pq.data(), end_pm);
            g_tool->setPm(*g_wobj, end_pm);
            g_model.generalMotionPool().at(0).updMpm();
            g_tool->getPm(*g_wobj, mvl_param.begin_pm);
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveJ,moveAbsJ,moveC
        {
            std::cout << "precmdname2:" << mvl_param.pre_plan->name() << "  cmdname2:" << this->name() << std::endl;
            //获取起始点位置//
            if (std::string(mvl_param.pre_plan->name()) == "MoveAbsJ")
            {
                auto preparam = std::any_cast<MoveAbsJParam>(&mvl_param.pre_plan->param());
                for (Size i = 0; i < std::min(controller()->motionPool().size(), g_model.motionPool().size()); ++i)
                {
                    g_model.motionPool().at(i).setMp(preparam->axis_pos_vec[i]);
                }
                g_model.solverPool().at(1).kinPos();
                g_model.generalMotionPool().at(0).updMpm();
                g_model.generalMotionPool().at(0).getMpm(end_pm);
            }
            if (std::string(mvl_param.pre_plan->name()) == "MoveJ")
            {
                auto preparam = std::any_cast<MoveJParam>(&mvl_param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_pq.data(), end_pm);
            }
            if (std::string(mvl_param.pre_plan->name()) == "MoveC")
            {
                auto preparam = std::any_cast<MoveCParam>(&mvl_param.pre_plan->param());
                aris::dynamic::s_pq2pm(preparam->ee_end_pq.data(), end_pm);
            }

            g_tool->setPm(*g_wobj, end_pm);
            g_model.generalMotionPool().at(0).updMpm();
            g_tool->getPm(*g_wobj, mvl_param.begin_pm);
        }

        //获取终止点位置//
        aris::dynamic::s_pq2pm(mvl_param.ee_pq.data(), end_pm);
        aris::dynamic::s_inv_pm_dot_pm(mvl_param.begin_pm, end_pm, relative_pm);
        aris::dynamic::s_pm2pa(relative_pm, mvl_param.relative_pa.data());
        mvl_param.norm_pos = aris::dynamic::s_norm(3, mvl_param.relative_pa.data());
        mvl_param.norm_ori = aris::dynamic::s_norm(3, mvl_param.relative_pa.data() + 3);

        //计算转弯区对应的count数//
        double p, v, a, j;
        aris::Size pos_total_count, ori_total_count;

        traplan::sCurve(1, 0.0, mvl_param.norm_pos, mvl_param.vel / 1000, mvl_param.acc / 1000 / 1000, mvl_param.jerk / 1000 / 1000 / 1000, p, v, a, j, pos_total_count);
        traplan::sCurve(1, 0.0, mvl_param.norm_ori, mvl_param.angular_vel / 1000, mvl_param.angular_acc / 1000 / 1000, mvl_param.angular_jerk / 1000 / 1000 / 1000, p, v, a, j, ori_total_count);

        mvl_param.pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
        mvl_param.ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

        traplan::sCurve(1, 0.0, mvl_param.norm_pos, mvl_param.vel / 1000 * mvl_param.pos_ratio, mvl_param.acc / 1000 / 1000 * mvl_param.pos_ratio* mvl_param.pos_ratio, mvl_param.jerk / 1000 / 1000 / 1000 * mvl_param.pos_ratio* mvl_param.pos_ratio * mvl_param.pos_ratio, p, v, a, j, pos_total_count);
        traplan::sCurve(1, 0.0, mvl_param.norm_ori, mvl_param.angular_vel / 1000 * mvl_param.ori_ratio, mvl_param.angular_acc / 1000 / 1000* mvl_param.ori_ratio* mvl_param.ori_ratio, mvl_param.angular_jerk / 1000 / 1000 / 1000* mvl_param.ori_ratio* mvl_param.ori_ratio* mvl_param.ori_ratio, p, v, a, j, ori_total_count);

        mvl_param.max_total_count = std::max(pos_total_count, ori_total_count);

        //二分法//
        auto pos_zone = mvl_param.norm_pos - mvl_param.zone.dis;
        auto ori_zone = mvl_param.norm_ori - mvl_param.norm_ori*mvl_param.zone.per;
        aris::Size begin_count = 1, target_count = 0, end_count;
        if (mvl_param.zone_enabled)
        {
            if (std::abs(mvl_param.norm_pos) > 2e-3)//转弯半径大于1mm,直线长度至少为2mm
            {
                end_count = pos_total_count;
                while (std::abs(p - pos_zone) > 1e-9)
                {
                    if (p < pos_zone)
                    {
                        begin_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, 0.0, mvl_param.norm_pos, mvl_param.vel / 1000 * mvl_param.pos_ratio, mvl_param.acc / 1000 / 1000 * mvl_param.pos_ratio* mvl_param.pos_ratio, mvl_param.jerk / 1000 / 1000 / 1000 * mvl_param.pos_ratio* mvl_param.pos_ratio * mvl_param.pos_ratio, p, v, a, j, pos_total_count);
                    }
                    else
                    {
                        end_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, 0.0, mvl_param.norm_pos, mvl_param.vel / 1000 * mvl_param.pos_ratio, mvl_param.acc / 1000 / 1000 * mvl_param.pos_ratio* mvl_param.pos_ratio, mvl_param.jerk / 1000 / 1000 / 1000 * mvl_param.pos_ratio* mvl_param.pos_ratio * mvl_param.pos_ratio, p, v, a, j, pos_total_count);
                    }
                    if ((begin_count == end_count) || (begin_count + 1 == end_count) || (end_count==0))
                    {
                        break;
                    }
                }
            }
            else if ((std::abs(mvl_param.norm_pos) <= 2e-3) && (std::abs(mvl_param.norm_ori) > 2e-3))//转弯半径小于1mm,转动半径大于0.001rad
            {
                end_count = ori_total_count;
                while (std::abs(p - ori_zone) > 1e-9)
                {
                    if (p < ori_zone)
                    {
                        begin_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(1, 0.0, mvl_param.norm_ori, mvl_param.angular_vel / 1000 * mvl_param.ori_ratio, mvl_param.angular_acc / 1000 / 1000 * mvl_param.ori_ratio* mvl_param.ori_ratio, mvl_param.angular_jerk / 1000 / 1000 / 1000 * mvl_param.ori_ratio* mvl_param.ori_ratio* mvl_param.ori_ratio, p, v, a, j, ori_total_count);
                    }
                    else
                    {
                        end_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(1, 0.0, mvl_param.norm_ori, mvl_param.angular_vel / 1000 * mvl_param.ori_ratio, mvl_param.angular_acc / 1000 / 1000 * mvl_param.ori_ratio* mvl_param.ori_ratio, mvl_param.angular_jerk / 1000 / 1000 / 1000 * mvl_param.ori_ratio* mvl_param.ori_ratio* mvl_param.ori_ratio, p, v, a, j, ori_total_count);
                    }
                    if ((begin_count == end_count) || (begin_count + 1 == end_count) || (end_count == 0))
                    {
                        break;
                    }
                }
            }
            else{}
        }

        //更新转弯区//
        if (mvl_param.pre_plan == nullptr)//转弯第一条指令
        {
            //更新本条指令的planzone//
            this->planzone.store(target_count);
        }
        else if (mvl_param.pre_plan->name() == "MoveL")//转弯第二或第n条指令
        {
            auto param = std::any_cast<MoveLParam&>(mvl_param.pre_plan->param());
            if ((std::abs(param.norm_pos) > 2e-3) && (std::abs(mvl_param.norm_pos) > 2e-3))//前后两条轨迹都含直线运动部分
            {
                //更新本plan的planzone//
                this->planzone.store(target_count);
                //更新上一条转弯指令的realzone//
                if (mvl_param.pre_plan->cmd_executing.load())
                {
                    mvl_param.pre_plan->realzone.store(0);
                }
                else
                {
                    aris::Size min_zone = std::min(mvl_param.pre_plan->planzone.load(), mvl_param.max_total_count / 2);//当前指令所需count数/2
                    mvl_param.pre_plan->realzone.store(min_zone);
                }
            }
            else if ((std::abs(param.norm_pos) <= 2e-3) && (std::abs(param.norm_ori) > 2e-3) && (std::abs(mvl_param.norm_pos) <= 2e-3) && (std::abs(mvl_param.norm_ori) > 2e-3))//两条轨迹都是纯转动
            {
                //更新本plan的planzone//
                this->planzone.store(target_count);
                //更新上一条转弯指令的realzone//
                if (mvl_param.pre_plan->cmd_executing.load())
                {
                    mvl_param.pre_plan->realzone.store(0);
                }
                else
                {
                    aris::Size min_zone = std::min(mvl_param.pre_plan->planzone.load(), mvl_param.max_total_count / 2);//当前指令所需count数/2
                    mvl_param.pre_plan->realzone.store(min_zone);
                }
            }
            else//前后两条轨迹有一条是纯转动，取消上一条指令转弯区
            {
                //更新本plan的planzone//
                this->planzone.store(target_count);
                //更新上一条转弯指令的realzone//
                mvl_param.pre_plan->realzone.store(0);
            }

        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveJ,moveC,moveAbsJ
        {
            //更新本plan的planzone//
            this->planzone.store(target_count);
            //上一条指令不进行转弯//
            mvl_param.pre_plan->realzone.store(0);
        }

        for (auto &option : motorOptions())	option |= USE_TARGET_POS|NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        this->param() = mvl_param;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MoveL::executeRT()->int
    {
        this->cmd_executing.load();
        auto mvl_param = std::any_cast<MoveLParam>(&this->param());
        auto &pwinter = dynamic_cast<aris::server::ProgramWebInterface&>(controlServer()->interfacePool().at(0));

        //如果停止功能开启，并且时间已经停止，退出本条指令//
        if (count() == 1)
        {
            if (pwinter.isAutoStopped() && (g_counter == 0))
            {
                this->cmd_finished.store(true);
                g_plan = nullptr;
                return 0;
            }
        }

        //暂停、恢复//
        PauseContinueB(this, pwinter);

        // 取得起始位置 //
        static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
        double p, v, a, j;
        aris::Size pos_total_count, ori_total_count;
        /*
        if (count() == 1)
        {
            double end_pm[16];
            aris::dynamic::s_pq2pm(mvl_param->ee_pq.data(), end_pm);
            model()->generalMotionPool().at(0).updMpm();
            //model()->generalMotionPool().at(0).getMpm(begin_pm);
            mvl_param->tool->getPm(*mvl_param->wobj, begin_pm);
            aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);


            // relative_pa //
            aris::dynamic::s_pm2pa(relative_pm, relative_pa);

            norm_pos = aris::dynamic::s_norm(3, relative_pa);
            norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);

            //aris::plan::moveAbsolute(count(), 0.0, norm_pos, mvl_param->vel / 1000, mvl_param->acc / 1000 / 1000, mvl_param->dec / 1000 / 1000, p, v, a, pos_total_count);
            //aris::plan::moveAbsolute(count(), 0.0, norm_ori, mvl_param->angular_vel / 1000, mvl_param->angular_acc / 1000 / 1000, mvl_param->angular_dec / 1000 / 1000, p, v, a, ori_total_count);

            traplan::sCurve(static_cast<double>(count()), 0.0, norm_pos, mvl_param->vel / 1000, mvl_param->acc / 1000 / 1000, mvl_param->jerk / 1000 / 1000 / 1000, p, v, a, j, pos_total_count);
            traplan::sCurve(static_cast<double>(count()), 0.0, norm_ori, mvl_param->angular_vel / 1000, mvl_param->angular_acc / 1000 / 1000, mvl_param->angular_jerk / 1000 / 1000 / 1000, p, v, a, j, ori_total_count);

            pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
            ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

            //aris::plan::moveAbsolute(count(), 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
            //aris::plan::moveAbsolute(count(), 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);

            traplan::sCurve(static_cast<double>(count()), 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio * pos_ratio, mvl_param->jerk / 1000 / 1000 / 1000 * pos_ratio* pos_ratio* pos_ratio, p, v, a, j, pos_total_count);
            traplan::sCurve(static_cast<double>(count()), 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_jerk / 1000 / 1000 / 1000 * ori_ratio * ori_ratio* ori_ratio, p, v, a, j, ori_total_count);

        }

        double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

        //aris::plan::moveAbsolute(count(), 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
        traplan::sCurve(static_cast<double>(count()), 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->jerk / 1000 / 1000 /1000 * pos_ratio* pos_ratio* pos_ratio, p, v, a, j, pos_total_count);
        if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);

        //aris::plan::moveAbsolute(count(), 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
        traplan::sCurve(static_cast<double>(count()), 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_jerk / 1000 / 1000 /1000 * ori_ratio * ori_ratio * ori_ratio, p, v, a, j, ori_total_count);
        if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);

        aris::dynamic::s_pa2pm(pa, pm);
        aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);

        // 反解计算电机位置 //
        //model()->generalMotionPool().at(0).setMpm(pm2);
        mvl_param->tool->setPm(*mvl_param->wobj, pm2);
        model()->generalMotionPool().at(0).updMpm();
        if (model()->solverPool().at(0).kinPos())return -1;

        if (g_move_pause)
        {
            return -4;
        }
        return std::max(pos_total_count, ori_total_count) > count() ? 1 : 0;
        */

        if (mvl_param->pre_plan == nullptr) //转弯第一条指令
        {
            if (count() == 1)
            {
                // init joint_pos //
                for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
                {
                    model()->motionPool().at(i).setMp(controller()->motionPool()[i].targetPos());
                }
                if (model()->solverPool().at(1).kinPos())return -1;
                double end_pm[16];
                aris::dynamic::s_pq2pm(mvl_param->ee_pq.data(), end_pm);
                model()->generalMotionPool().at(0).updMpm();
                mvl_param->tool->getPm(*mvl_param->wobj, mvl_param->begin_pm);
                aris::dynamic::s_inv_pm_dot_pm(mvl_param->begin_pm, end_pm, relative_pm);

                // relative_pa //
                aris::dynamic::s_pm2pa(relative_pm, mvl_param->relative_pa.data());

                mvl_param->norm_pos = aris::dynamic::s_norm(3, mvl_param->relative_pa.data());
                mvl_param->norm_ori = aris::dynamic::s_norm(3, mvl_param->relative_pa.data() + 3);
            }
            double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

            traplan::sCurve(g_count, 0.0, mvl_param->norm_pos, mvl_param->vel / 1000 * mvl_param->pos_ratio, mvl_param->acc / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio, mvl_param->jerk / 1000 / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio* mvl_param->pos_ratio, p, v, a, j, pos_total_count);
            if (mvl_param->norm_pos > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_pos, mvl_param->relative_pa.data(), pa);

            traplan::sCurve(g_count, 0.0, mvl_param->norm_ori, mvl_param->angular_vel / 1000 * mvl_param->ori_ratio, mvl_param->angular_acc / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio, mvl_param->angular_jerk / 1000 / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio * mvl_param->ori_ratio, p, v, a, j, ori_total_count);
            if (mvl_param->norm_ori > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_ori, mvl_param->relative_pa.data() + 3, pa + 3);

            aris::dynamic::s_pa2pm(pa, pm);
            aris::dynamic::s_pm_dot_pm(mvl_param->begin_pm, pm, pm2);

            // 反解计算电机位置 //
            mvl_param->tool->setPm(*mvl_param->wobj, pm2);
            model()->generalMotionPool().at(0).updMpm();
        }
        else if (mvl_param->pre_plan->name() == "MoveL") //转弯区第二条指令或者第n条指令
        {
            auto param = std::any_cast<MoveLParam&>(mvl_param->pre_plan->param());
            auto zonecount = mvl_param->pre_plan->realzone.load();

            //preplan//
            double prep = 0.0, prev, prea, prej;

            //count数小于等于上一条指令的realzone，zone起作用//
            if (g_count <= zonecount)
            {
                //preplan//
                double pre_pa[6]{ 0,0,0,0,0,0 }, pre_pm[16], pre_pm2[16];

                traplan::sCurve(param.max_total_count - zonecount + g_count, 0.0, param.norm_pos, param.vel / 1000 * param.pos_ratio, param.acc / 1000 / 1000 * param.pos_ratio* param.pos_ratio, param.jerk / 1000 / 1000 / 1000 * param.pos_ratio* param.pos_ratio* param.pos_ratio, p, v, a, j, pos_total_count);
                if (param.norm_pos > 1e-10)aris::dynamic::s_vc(3, p / param.norm_pos, param.relative_pa.data(), pre_pa);

                traplan::sCurve(param.max_total_count - zonecount + g_count, 0.0, param.norm_ori, param.angular_vel / 1000 * param.ori_ratio, param.angular_acc / 1000 / 1000 * param.ori_ratio * param.ori_ratio, param.angular_jerk / 1000 / 1000 / 1000 * param.ori_ratio * param.ori_ratio * param.ori_ratio, p, v, a, j, ori_total_count);
                if (param.norm_ori > 1e-10)aris::dynamic::s_vc(3, p / param.norm_ori, param.relative_pa.data() + 3, pre_pa + 3);

                aris::dynamic::s_pa2pm(pre_pa, pre_pm);
                aris::dynamic::s_pm_dot_pm(param.begin_pm, pre_pm, pre_pm2);
                aris::dynamic::s_pm2pa(pre_pm2, pre_pa);

                //thisplan//
                double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

                traplan::sCurve(g_count, 0.0, mvl_param->norm_pos, mvl_param->vel / 1000 * mvl_param->pos_ratio, mvl_param->acc / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio, mvl_param->jerk / 1000 / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio* mvl_param->pos_ratio, p, v, a, j, pos_total_count);
                if (mvl_param->norm_pos > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_pos, mvl_param->relative_pa.data(), pa);

                traplan::sCurve(g_count, 0.0, mvl_param->norm_ori, mvl_param->angular_vel / 1000 * mvl_param->ori_ratio, mvl_param->angular_acc / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio, mvl_param->angular_jerk / 1000 / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio * mvl_param->ori_ratio, p, v, a, j, ori_total_count);
                if (mvl_param->norm_ori > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_ori, mvl_param->relative_pa.data() + 3, pa + 3);

                aris::dynamic::s_pa2pm(pa, pm);
                aris::dynamic::s_pm_dot_pm(mvl_param->begin_pm, pm, pm2);
                aris::dynamic::s_pm2pa(pm2, pa);

                // 反解计算电机位置 //
                double target_pa[6]{ 0,0,0,0,0,0 }, target_pm[16];
                for (aris::Size i = 0; i < 6; i++)
                {
                    target_pa[i] = 1.0*(zonecount - g_count) / zonecount * pre_pa[i] + 1.0*g_count / zonecount * pa[i];
                }
                aris::dynamic::s_pa2pm(target_pa, target_pm);
                mvl_param->tool->setPm(*mvl_param->wobj, target_pm);
                model()->generalMotionPool().at(0).updMpm();
            }
            else
            {
                //thisplan//
                mvl_param->pre_plan->cmd_finished.store(true);
                double pa[6]{ 0.0,0.0,0.0,0.0,0.0,0.0 }, pm[16], pm2[16];

                traplan::sCurve(g_count, 0.0, mvl_param->norm_pos, mvl_param->vel / 1000 * mvl_param->pos_ratio, mvl_param->acc / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio, mvl_param->jerk / 1000 / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio* mvl_param->pos_ratio, p, v, a, j, pos_total_count);
                if (mvl_param->norm_pos > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_pos, mvl_param->relative_pa.data(), pa);

                traplan::sCurve(g_count, 0.0, mvl_param->norm_ori, mvl_param->angular_vel / 1000 * mvl_param->ori_ratio, mvl_param->angular_acc / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio, mvl_param->angular_jerk / 1000 / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio * mvl_param->ori_ratio, p, v, a, j, ori_total_count);
                if (mvl_param->norm_ori > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_ori, mvl_param->relative_pa.data() + 3, pa + 3);

                aris::dynamic::s_pa2pm(pa, pm);
                aris::dynamic::s_pm_dot_pm(mvl_param->begin_pm, pm, pm2);

                // 反解计算电机位置 //
                mvl_param->tool->setPm(*mvl_param->wobj, pm2);
                model()->generalMotionPool().at(0).updMpm();
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveJ,moveC,moveAbsJ
        {
            double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

            traplan::sCurve(g_count, 0.0, mvl_param->norm_pos, mvl_param->vel / 1000 * mvl_param->pos_ratio, mvl_param->acc / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio, mvl_param->jerk / 1000 / 1000 / 1000 * mvl_param->pos_ratio* mvl_param->pos_ratio* mvl_param->pos_ratio, p, v, a, j, pos_total_count);
            if (mvl_param->norm_pos > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_pos, mvl_param->relative_pa.data(), pa);

            traplan::sCurve(g_count, 0.0, mvl_param->norm_ori, mvl_param->angular_vel / 1000 * mvl_param->ori_ratio, mvl_param->angular_acc / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio, mvl_param->angular_jerk / 1000 / 1000 / 1000 * mvl_param->ori_ratio * mvl_param->ori_ratio * mvl_param->ori_ratio, p, v, a, j, ori_total_count);
            if (mvl_param->norm_ori > 1e-10)aris::dynamic::s_vc(3, p / mvl_param->norm_ori, mvl_param->relative_pa.data() + 3, pa + 3);

            aris::dynamic::s_pa2pm(pa, pm);
            aris::dynamic::s_pm_dot_pm(mvl_param->begin_pm, pm, pm2);

            // 反解计算电机位置 //
            mvl_param->tool->setPm(*mvl_param->wobj, pm2);
            model()->generalMotionPool().at(0).updMpm();
        }

        if (model()->solverPool().at(0).kinPos())return -1;

        auto rzcount = this->realzone.load();
        //本条指令没有转弯区
        PauseContinueE(mvl_param, pwinter, rzcount);

        if (mvl_param->max_total_count - rzcount - int32_t(g_count) == 0)
        {
            //realzone为0时，返回值为0时，本条指令执行完毕
            if (rzcount == 0)this->cmd_finished.store(true);
            controller()->mout() << "mvl_param->max_total_count:" << mvl_param->max_total_count << "this->realzone.load():" << rzcount << std::endl;
        }

        if (pwinter.isAutoStopped() && (g_counter == 0))
        {
            //指令停止且返回值为0时，本条指令执行完毕
            this->cmd_finished.store(true);
            g_plan.reset();
            return 0;
        }
        return mvl_param->max_total_count == 0 ? 0 : mvl_param->max_total_count - rzcount - int32_t(g_count);
    }
    auto MoveL::collectNrt()->void
    {
        if(this->retCode()<0)
        {
            g_plan.reset();
        }
        std::any_cast<MoveLParam>(&this->param())->pre_plan.reset();
    }
    MoveL::~MoveL() = default;
    MoveL::MoveL(const MoveL &other) = default;
    MoveL::MoveL(const std::string &name) :MoveBase(name)
    {
        command().loadXmlStr(
            "<Command name=\"mvl\">"
            "	<GroupParam>"
            "		<Param name=\"pos_unit\" default=\"m\"/>"
            "		<UniqueParam default=\"pq\">"
            "			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
            "			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
            "			<GroupParam>"
            "				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
            "				<Param name=\"ori_unit\" default=\"rad\"/>"
            "				<Param name=\"eul_type\" default=\"321\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "		<Param name=\"acc\" default=\"0.5\"/>"
            "		<Param name=\"vel\" default=\"{0.025, 0.025, 3.4, 0.0, 0.0}\"/>"
            "		<Param name=\"dec\" default=\"0.5\"/>"
            "		<Param name=\"jerk\" default=\"10.0\"/>"
            "		<Param name=\"angular_acc\" default=\"0.5\"/>"
            "		<Param name=\"angular_vel\" default=\"0.1\"/>"
            "		<Param name=\"angular_dec\" default=\"0.5\"/>"
            "		<Param name=\"angular_jerk\" default=\"10.0\"/>"
            "		<Param name=\"zone\" default=\"fine\"/>"
            "		<Param name=\"load\" default=\"{1,0.05,0.05,0.05,0,0.97976,0,0.200177,1.0,1.0,1.0}\"/>"
            "		<Param name=\"tool\" default=\"tool0\"/>"
            "		<Param name=\"wobj\" default=\"wobj0\"/>"
            CHECK_PARAM_STRING
            "	</GroupParam>"
            "</Command>");
    }


    //走圆弧//
    auto find_mid_pq(const std::map<std::string_view, std::string_view> &params, Plan &plan, double *mid_pq_out)->bool
    {
        double pos_unit;
        auto pos_unit_found = plan.cmdParams().find("pos_unit");
        if (pos_unit_found == plan.cmdParams().end()) pos_unit = 1.0;
        else if (pos_unit_found->second == "m")pos_unit = 1.0;
        else if (pos_unit_found->second == "mm")pos_unit = 0.001;
        else if (pos_unit_found->second == "cm")pos_unit = 0.01;
        else THROW_FILE_LINE("");

        auto &cal = plan.controlServer()->model().calculator();

        for (auto cmd_param : params)
        {
            if (cmd_param.first == "mid_pq")
            {
                auto pq_mat = std::any_cast<aris::core::Matrix>(cal.calculateExpression("robtarget(" + std::string(cmd_param.second) + ")").second);
                if (pq_mat.size() != 7)THROW_FILE_LINE("");

                aris::dynamic::s_vc(7, pq_mat.data(), mid_pq_out);
                aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
                return true;
            }
            else if (cmd_param.first == "mid_pm")
            {
                auto pm_mat = plan.matrixParam(cmd_param.first);

                if (pm_mat.size() != 16)THROW_FILE_LINE("");
                aris::dynamic::s_pm2pq(pm_mat.data(), mid_pq_out);
                aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
                return true;
            }
            else if (cmd_param.first == "mid_pe")
            {
                double ori_unit;
                auto ori_unit_found = plan.cmdParams().find("mid_ori_unit");
                if (ori_unit_found == plan.cmdParams().end()) ori_unit = 1.0;
                else if (ori_unit_found->second == "rad")ori_unit = 1.0;
                else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
                else THROW_FILE_LINE("");

                std::string eul_type;
                auto eul_type_found = plan.cmdParams().find("mid_eul_type");
                if (eul_type_found == plan.cmdParams().end()) eul_type = "321";
                else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
                else THROW_FILE_LINE("");

                auto pe_mat = plan.matrixParam(cmd_param.first);
                if (pe_mat.size() != 6)THROW_FILE_LINE("");
                aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
                aris::dynamic::s_pe2pq(pe_mat.data(), mid_pq_out, eul_type.data());
                aris::dynamic::s_nv(3, pos_unit, mid_pq_out);
                return true;
            }
        }

        THROW_FILE_LINE("No mid pose input");
    }
    auto find_end_pq(const std::map<std::string_view, std::string_view> &params, Plan &plan, double *end_pq_out)->bool
    {
        double pos_unit;
        auto pos_unit_found = plan.cmdParams().find("pos_unit");
        if (pos_unit_found == plan.cmdParams().end()) pos_unit = 1.0;
        else if (pos_unit_found->second == "m")pos_unit = 1.0;
        else if (pos_unit_found->second == "mm")pos_unit = 0.001;
        else if (pos_unit_found->second == "cm")pos_unit = 0.01;
        else THROW_FILE_LINE("");

        auto &cal = plan.controlServer()->model().calculator();

        for (auto cmd_param : params)
        {
            if (cmd_param.first == "end_pq")
            {
                auto pq_mat = std::any_cast<aris::core::Matrix>(cal.calculateExpression("robtarget(" + std::string(cmd_param.second) + ")").second);
                if (pq_mat.size() != 7)THROW_FILE_LINE("");
                aris::dynamic::s_vc(7, pq_mat.data(), end_pq_out);
                aris::dynamic::s_nv(3, pos_unit, end_pq_out);
                return true;
            }
            else if (cmd_param.first == "end_pm")
            {
                auto pm_mat = plan.matrixParam(cmd_param.first);
                if (pm_mat.size() != 16)THROW_FILE_LINE("");
                aris::dynamic::s_pm2pq(pm_mat.data(), end_pq_out);
                aris::dynamic::s_nv(3, pos_unit, end_pq_out);
                return true;
            }
            else if (cmd_param.first == "end_pe")
            {
                double ori_unit;
                auto ori_unit_found = plan.cmdParams().find("mid_ori_unit");
                if (ori_unit_found == plan.cmdParams().end()) ori_unit = 1.0;
                else if (ori_unit_found->second == "rad")ori_unit = 1.0;
                else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
                else THROW_FILE_LINE("");

                std::string eul_type;
                auto eul_type_found = plan.cmdParams().find("mid_eul_type");
                if (eul_type_found == plan.cmdParams().end()) eul_type = "321";
                else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
                else THROW_FILE_LINE("");

                auto pe_mat = plan.matrixParam(cmd_param.first);
                if (pe_mat.size() != 6)THROW_FILE_LINE("");
                aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
                aris::dynamic::s_pe2pq(pe_mat.data(), end_pq_out, eul_type.data());
                aris::dynamic::s_nv(3, pos_unit, end_pq_out);
                return true;
            }
        }

        THROW_FILE_LINE("No end pose input");
    }
    void slerp(double starting[4], double ending[4], double result[4], double t)
    {
        double cosa = starting[0] * ending[0] + starting[1] * ending[1] + starting[2] * ending[2] + starting[3] * ending[3];

        // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
        // the shorter path. Fix by reversing one quaternion.
        if (cosa < 0.0f)
        {
            ending[0] = -ending[0];
            ending[1] = -ending[1];
            ending[2] = -ending[2];
            ending[3] = -ending[3];
            cosa = -cosa;
        }

        double k0, k1;

        // If the inputs are too close for comfort, linearly interpolate
        if (cosa > 0.9995f)
        {
            k0 = 1.0f - t;
            k1 = t;
        }
        else
        {
            double sina = sqrt(1.0f - cosa * cosa);
            double a = atan2(sina, cosa);
            k0 = sin((1.0f - t)*a) / sina;
            k1 = sin(t*a) / sina;
        }
        result[0] = starting[0] * k0 + ending[0] * k1;
        result[1] = starting[1] * k0 + ending[1] * k1;
        result[2] = starting[2] * k0 + ending[2] * k1;
        result[3] = starting[3] * k0 + ending[3] * k1;
    }
    auto cal_circle_par(MoveCParam &par)->bool
    {
        //排除3点共线的情况//
        double b[3];
        std::vector<double> mul_cross(3);
        std::vector<double> p1(3);
        std::vector<double> p2(3);
        std::vector<double> p3(3);
        std::copy(par.ee_begin_pq.data(), par.ee_begin_pq.data() + 3, p1.data());
        std::copy(par.ee_mid_pq.data(), par.ee_mid_pq.data() + 3, p2.data());
        std::copy(par.ee_end_pq.data(), par.ee_end_pq.data() + 3, p3.data());
        s_vs(3, p2.data(), p3.data());
        s_vs(3, p1.data(), p2.data());
        s_c3(p2.data(), p3.data(), mul_cross.data());
        double normv = aris::dynamic::s_norm(3, mul_cross.data());
        if (normv <= 1e-10) return 0;

        //通过AC=b 解出圆心C//
        par.A[0] = 2 * (par.ee_begin_pq[0] - par.ee_mid_pq[0]);
        par.A[1] = 2 * (par.ee_begin_pq[1] - par.ee_mid_pq[1]);
        par.A[2] = 2 * (par.ee_begin_pq[2] - par.ee_mid_pq[2]);
        par.A[3] = 2 * (par.ee_mid_pq[0] - par.ee_end_pq[0]);
        par.A[4] = 2 * (par.ee_mid_pq[1] - par.ee_end_pq[1]);
        par.A[5] = 2 * (par.ee_mid_pq[2] - par.ee_end_pq[2]);
        par.A[6] = (par.A[1] * par.A[5] - par.A[4] * par.A[2]) / 4;
        par.A[7] = -(par.A[0] * par.A[5] - par.A[3] * par.A[2]) / 4;
        par.A[8] = (par.A[0] * par.A[4] - par.A[3] * par.A[1]) / 4;
        b[0] = pow(par.ee_begin_pq[0], 2) + pow(par.ee_begin_pq[1], 2) + pow(par.ee_begin_pq[2], 2) - pow(par.ee_mid_pq[0], 2) - pow(par.ee_mid_pq[1], 2) - pow(par.ee_mid_pq[2], 2);
        b[1] = pow(par.ee_mid_pq[0], 2) + pow(par.ee_mid_pq[1], 2) + pow(par.ee_mid_pq[2], 2) - pow(par.ee_end_pq[0], 2) - pow(par.ee_end_pq[1], 2) - pow(par.ee_end_pq[2], 2);
        b[2] = par.A[6] * par.ee_begin_pq[0] + par.A[7] * par.ee_begin_pq[1] + par.A[8] * par.ee_begin_pq[2];

        //解线性方程组
        {
            double pinv[9];
            std::vector<double> U_vec(9);
            auto U = U_vec.data();
            double tau[3];
            aris::Size p[3];
            aris::Size rank;
            s_householder_utp(3, 3, par.A, U, tau, p, rank, 1e-10);
            double tau2[3];
            s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);
            //获取圆心
            s_mm(3, 1, 3, pinv, b, par.C);
            //获取半径
            par.R = sqrt(pow(par.ee_begin_pq[0] - par.C[0], 2) + pow(par.ee_begin_pq[1] - par.C[1], 2) + pow(par.ee_begin_pq[2] - par.C[2], 2));
        }

        //求旋转角度theta//
        double u, v, w;
        double u1, v1, w1;
        u = par.A[6];
        v = par.A[7];
        w = par.A[8];
        u1 = (par.ee_begin_pq[1] - par.C[1])*(par.ee_end_pq[2] - par.ee_begin_pq[2]) - (par.ee_begin_pq[2] - par.C[2])*(par.ee_end_pq[1] - par.ee_begin_pq[1]);
        v1 = (par.ee_begin_pq[2] - par.C[2])*(par.ee_end_pq[0] - par.ee_begin_pq[0]) - (par.ee_begin_pq[0] - par.C[0])*(par.ee_end_pq[2] - par.ee_begin_pq[2]);
        w1 = (par.ee_begin_pq[0] - par.C[0])*(par.ee_end_pq[1] - par.ee_begin_pq[1]) - (par.ee_begin_pq[1] - par.C[1])*(par.ee_end_pq[0] - par.ee_begin_pq[0]);

        double H = u * u1 + v * v1 + w * w1;

        // 判断theta 与 pi 的关系
        if (H >= 0)
        {
            par.theta = 2 * asin(sqrt(pow((par.ee_end_pq[0] - par.ee_begin_pq[0]), 2) + pow((par.ee_end_pq[1] - par.ee_begin_pq[1]), 2) + pow((par.ee_end_pq[2] - par.ee_begin_pq[2]), 2)) / (2 * par.R));
        }
        else
        {
            par.theta = 2 * 3.1415 - 2 * asin(sqrt(pow((par.ee_end_pq[0] - par.ee_begin_pq[0]), 2) + pow((par.ee_end_pq[1] - par.ee_begin_pq[1]), 2) + pow((par.ee_end_pq[2] - par.ee_begin_pq[2]), 2)) / (2 * par.R));
        }

        double normv_begin_pq = aris::dynamic::s_norm(4, par.ee_begin_pq.data() + 3);
        double normv_end_pq = aris::dynamic::s_norm(4, par.ee_end_pq.data() + 3);

        par.ori_theta = std::abs((par.ee_begin_pq[3] * par.ee_end_pq[3] + par.ee_begin_pq[4] * par.ee_end_pq[4] + par.ee_begin_pq[5] * par.ee_end_pq[5] + par.ee_begin_pq[6] * par.ee_end_pq[6]) / (normv_begin_pq* normv_end_pq));
        return 1;
    }
    auto cal_pqt(MoveCParam &par, Plan &plan, double pqt[7])->void
    {
        //位置规划//
        double p, v, a, j;
        aris::Size pos_total_count, ori_total_count;
        double w[3], pmr[16];
        aris::dynamic::s_vc(3, par.A + 6, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        traplan::sCurve(g_count, 0.0, par.theta, par.vel / 1000 / par.R * par.pos_ratio, par.acc / 1000 / 1000 / par.R * par.pos_ratio * par.pos_ratio,
            par.jerk / 1000 / 1000 / 1000 / par.R * par.pos_ratio * par.pos_ratio * par.pos_ratio, p, v, a, j, pos_total_count);
        double pqr[7]{ par.C[0], par.C[1], par.C[2], w[0] * sin(p / 2.0), w[1] * sin(p / 2.0), w[2] * sin(p / 2.0), cos(p / 2.0) };
        double pos[4]{ par.ee_begin_pq[0] - par.C[0], par.ee_begin_pq[1] - par.C[1], par.ee_begin_pq[2] - par.C[2], 1 };
        aris::dynamic::s_pq2pm(pqr, pmr);
        s_mm(4, 1, 4, pmr, aris::dynamic::RowMajor{ 4 }, pos, 1, pqt, 1);

        //姿态规划//
        traplan::sCurve(g_count, 0.0, 1.0, par.angular_vel / 1000 / par.ori_theta / 2.0 * par.ori_ratio, par.angular_acc / 1000 / 1000 / par.ori_theta / 2.0 * par.ori_ratio * par.ori_ratio,
            par.angular_jerk / 1000 / 1000 / 1000 / par.ori_theta / 2.0 * par.ori_ratio * par.ori_ratio * par.ori_ratio, p, v, a, j, ori_total_count);
        slerp(par.ee_begin_pq.data() + 3, par.ee_end_pq.data() + 3, pqt + 3, p);
    }
    struct MoveC::Imp {};
    auto MoveC::prepareNrt()->void
    {
        MoveCParam mvc_param;
        mvc_param.ee_begin_pq.resize(7);
        mvc_param.ee_mid_pq.resize(7);
        mvc_param.ee_end_pq.resize(7);
        if (!find_mid_pq(cmdParams(), *this, mvc_param.ee_mid_pq.data()))THROW_FILE_LINE("");
        if (!find_end_pq(cmdParams(), *this, mvc_param.ee_end_pq.data()))THROW_FILE_LINE("");

        mvc_param.tool = &*model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        mvc_param.wobj = &*model()->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));
        g_tool = &*g_model.generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmdParams().at("tool")));
        g_wobj = &*g_model.generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmdParams().at("wobj")));

        auto &cal = this->controlServer()->model().calculator();
        for (auto cmd_param : cmdParams())
        {
            if (cmd_param.first == "acc")
            {
                mvc_param.acc = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "vel")
            {
                auto s = std::any_cast<kaanh::Speed>(cal.calculateExpression("speed(" + std::string(cmd_param.second) + ")").second);
                mvc_param.sp = s;
                mvc_param.vel = mvc_param.sp.v_tcp;
                mvc_param.angular_vel = mvc_param.sp.w_tcp;
            }
            else if (cmd_param.first == "dec")
            {
                mvc_param.dec = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "jerk")
            {
                mvc_param.jerk = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "angular_acc")
            {
                mvc_param.angular_acc = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "angular_dec")
            {
                mvc_param.angular_dec = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "angular_jerk")
            {
                mvc_param.angular_jerk = doubleParam(cmd_param.first);
            }
            else if (cmd_param.first == "zone")
            {
                if (cmd_param.second == "fine")//不设置转弯区
                {
                    mvc_param.zone.dis = 0.0;
                    mvc_param.zone.per = 0.0;
                    mvc_param.zone_enabled = 0;
                }
                else//设置转弯区
                {
                    auto z = std::any_cast<kaanh::Zone>(cal.calculateExpression("zone(" + std::string(cmd_param.second) + ")").second);
                    mvc_param.zone = z;
                    mvc_param.zone_enabled = 1;
                    if (mvc_param.zone.dis > 0.2&&mvc_param.zone.dis < 0.001)
                    {
                        THROW_FILE_LINE("zone out of range");
                    }
                }
            }
            else if (cmd_param.first == "load")
            {
                auto temp = std::any_cast<kaanh::Load>(cal.calculateExpression("load(" + std::string(cmd_param.second) + ")").second);
                mvc_param.ld = temp;
            }
        }

        mvc_param.jerk = mvc_param.jerk *mvc_param.acc;
        mvc_param.angular_jerk = mvc_param.angular_jerk*mvc_param.angular_acc;

        //本指令cmdid-上一条指令的cmdid>1，且上一条指令执行完毕，g_plan赋值为nullptr
        if (mvc_param.pre_plan != nullptr)
        {
            if ((this->cmdId() - mvc_param.pre_plan->cmdId() > 1) && (mvc_param.pre_plan->cmd_finished.load()))g_plan = nullptr;
        }

        //更新全局变量g_plan//
        mvc_param.pre_plan = g_plan;
        g_plan = std::dynamic_pointer_cast<MoveBase>(sharedPtrForThis());//------这里需要潘博aris库提供一个plan接口，返回std::shared_ptr<MoveBase>类型指针

        if (mvc_param.pre_plan == nullptr)
        {
            std::cout << "preplan:" << "nullptr" << std::endl;
        }
        else
        {
            std::cout << "preplan:" << mvc_param.pre_plan->name() << std::endl;
        }

        // 获取起始点位姿 //
        double end_pq[7];
        if (mvc_param.pre_plan == nullptr)//转弯第一条指令
        {
            g_model.generalMotionPool().at(0).updMpm();
            g_tool->getPq(*g_wobj, mvc_param.ee_begin_pq.data());
        }
        else if (std::string(mvc_param.pre_plan->name()) == "MoveC")//转弯第二或第n条指令
        {
            std::cout << "precmdname1:" << mvc_param.pre_plan->name() << "  cmdname1:" << this->name() << std::endl;
            //从全局变量中获取上一条转弯区指令的目标点//
            auto param = std::any_cast<MoveCParam>(&mvc_param.pre_plan->param());
            g_tool->setPq(*g_wobj, param->ee_end_pq.data());
            g_model.generalMotionPool().at(0).updMpm();
            g_tool->getPq(*g_wobj, mvc_param.ee_begin_pq.data());
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveJ,moveAbsJ
        {
            std::cout << "precmdname2:" << mvc_param.pre_plan->name() << "  cmdname2:" << this->name() << std::endl;
            //获取起始点位置//
            if (std::string(mvc_param.pre_plan->name()) == "MoveJ")
            {
                auto preparam = std::any_cast<MoveJParam>(&mvc_param.pre_plan->param());
                g_tool->setPq(*g_wobj, preparam->ee_pq.data());
                g_model.generalMotionPool().at(0).updMpm();
                g_tool->getPq(*g_wobj, mvc_param.ee_begin_pq.data());
            }
            if (std::string(mvc_param.pre_plan->name()) == "MoveL")
            {
                auto preparam = std::any_cast<MoveLParam>(&mvc_param.pre_plan->param());
                g_tool->setPq(*g_wobj, preparam->ee_pq.data());
                g_model.generalMotionPool().at(0).updMpm();
                g_tool->getPq(*g_wobj, mvc_param.ee_begin_pq.data());
            }
            if (std::string(mvc_param.pre_plan->name()) == "MoveAbsJ")
            {
                auto preparam = std::any_cast<MoveAbsJParam>(&mvc_param.pre_plan->param());
                for (Size i = 0; i < g_model.motionPool().size(); ++i)
                {
                    g_model.motionPool()[i].setMp(preparam->axis_pos_vec[i]);
                }
                g_model.solverPool().at(1).kinPos();
                g_model.generalMotionPool().at(0).getMpq(mvc_param.ee_begin_pq.data());
            }
        }

        if(!cal_circle_par(mvc_param))throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "three point in line,movec trajectory planning failed");

        //计算转弯区对应的count数//
        double p, v, a, j;
        aris::Size pos_total_count, ori_total_count;
        traplan::sCurve(1, 0.0, mvc_param.theta, mvc_param.vel / 1000 / mvc_param.R, mvc_param.acc / 1000 / 1000 / mvc_param.R, mvc_param.jerk / 1000 / 1000 / 1000 / mvc_param.R, p, v, a, j, pos_total_count);
        traplan::sCurve(1, 0.0, 1.0, mvc_param.angular_vel / 1000 / mvc_param.ori_theta / 2.0, mvc_param.angular_acc / 1000 / 1000 / mvc_param.ori_theta / 2.0, mvc_param.angular_jerk / 1000 / 1000 / 1000 / mvc_param.ori_theta / 2.0, p, v, a, j, ori_total_count);

        mvc_param.pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
        mvc_param.ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

        traplan::sCurve(1, 0.0, mvc_param.theta, mvc_param.vel / 1000 / mvc_param.R * mvc_param.pos_ratio, mvc_param.acc / 1000 / 1000 / mvc_param.R * mvc_param.pos_ratio * mvc_param.pos_ratio,
            mvc_param.jerk / 1000 / 1000 / 1000 / mvc_param.R * mvc_param.pos_ratio * mvc_param.pos_ratio * mvc_param.pos_ratio, p, v, a, j, pos_total_count);
        traplan::sCurve(1, 0.0, 1.0, mvc_param.angular_vel / 1000 / mvc_param.ori_theta / 2.0 * mvc_param.ori_ratio, mvc_param.angular_acc / 1000 / 1000 / mvc_param.ori_theta / 2.0 * mvc_param.ori_ratio * mvc_param.ori_ratio,
            mvc_param.angular_jerk / 1000 / 1000 / 1000 / mvc_param.ori_theta / 2.0 * mvc_param.ori_ratio * mvc_param.ori_ratio * mvc_param.ori_ratio, p, v, a, j, ori_total_count);

        mvc_param.max_total_count = std::max(pos_total_count, ori_total_count);

        //二分法//
        auto pos_zone = mvc_param.theta - mvc_param.theta*mvc_param.zone.per;
        aris::Size begin_count = 1, target_count = 0, end_count;
        end_count = mvc_param.max_total_count;
        if (mvc_param.zone_enabled)
        {
            if (std::abs(mvc_param.norm_pos) > 2e-3)//转弯半径大于0.001rad,角度至少为0.002rad
            {

                while (std::abs(p - pos_zone) > 1e-9)
                {
                    if (p < pos_zone)
                    {
                        begin_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, 0.0, mvc_param.theta, mvc_param.vel / 1000 / mvc_param.R * mvc_param.pos_ratio, mvc_param.acc / 1000 / 1000 / mvc_param.R * mvc_param.pos_ratio * mvc_param.pos_ratio,
                            mvc_param.jerk / 1000 / 1000 / 1000 / mvc_param.R * mvc_param.pos_ratio * mvc_param.pos_ratio * mvc_param.pos_ratio, p, v, a, j, pos_total_count);
                    }
                    else
                    {
                        end_count = target_count;
                        target_count = (begin_count + end_count) / 2;
                        traplan::sCurve(target_count, 0.0, mvc_param.theta, mvc_param.vel / 1000 / mvc_param.R * mvc_param.pos_ratio, mvc_param.acc / 1000 / 1000 / mvc_param.R * mvc_param.pos_ratio * mvc_param.pos_ratio,
                            mvc_param.jerk / 1000 / 1000 / 1000 / mvc_param.R * mvc_param.pos_ratio * mvc_param.pos_ratio * mvc_param.pos_ratio, p, v, a, j, pos_total_count);
                    }
                    if ((begin_count == end_count) || (begin_count + 1 == end_count) || (end_count == 0))
                    {
                        break;
                    }
                }
            }
        }

        //更新转弯区//
        if (mvc_param.pre_plan == nullptr)//转弯第一条指令
        {
            //更新本条指令的planzone//
            this->planzone.store(target_count);
        }
        else if (mvc_param.pre_plan->name() == "MoveC")//转弯第二或第n条指令
        {
            auto param = std::any_cast<MoveCParam&>(mvc_param.pre_plan->param());
            //更新本plan的planzone//
            this->planzone.store(target_count);
            //更新上一条转弯指令的realzone//
            if (mvc_param.pre_plan->cmd_executing.load())
            {
                mvc_param.pre_plan->realzone.store(0);
            }
            else
            {
                aris::Size min_zone = std::min(mvc_param.pre_plan->planzone.load(), mvc_param.max_total_count / 2);//当前指令所需count数/2
                mvc_param.pre_plan->realzone.store(min_zone);
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveJ,moveC,moveAbsJ
        {
            //更新本plan的planzone//
            this->planzone.store(target_count);
            //上一条指令不进行转弯//
            mvc_param.pre_plan->realzone.store(0);
        }

        for (auto &option : motorOptions())	option |= Plan::USE_TARGET_POS|NOT_CHECK_POS_CONTINUOUS|NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
        this->param() = mvc_param;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MoveC::executeRT()->int
    {
        this->cmd_executing.load();
        auto mvc_param = std::any_cast<MoveCParam>(&this->param());
        auto &pwinter = dynamic_cast<aris::server::ProgramWebInterface&>(controlServer()->interfacePool().at(0));

        //如果停止功能开启，并且时间已经停止，退出本条指令//
        if (count() == 1)
        {
            if (pwinter.isAutoStopped() && (g_counter == 0))
            {
                this->cmd_finished.store(true);
                g_plan = nullptr;
                return 0;
            }
        }

        //暂停、恢复//
        PauseContinueB(this, pwinter);

        /*
        // 取得起始位置 //
        static double pos_ratio, ori_ratio;
        static double A[9], b[3], C[3], R, theta, ori_theta;
        double p, v, a, j;
        aris::Size pos_total_count, ori_total_count;

        if (count() == 1)
        {
            //model()->generalMotionPool().at(0).getMpq(mvc_param.ee_begin_pq.data());
            mvc_param.tool->getPq(*mvc_param.wobj, mvc_param.ee_begin_pq.data());

            //排除3点共线的情况//
            std::vector<double> mul_cross(3);
            std::vector<double> p1(3);
            std::vector<double> p2(3);
            std::vector<double> p3(3);
            std::copy(mvc_param.ee_begin_pq.data(), mvc_param.ee_begin_pq.data() + 3, p1.data());
            std::copy(mvc_param.ee_mid_pq.data(), mvc_param.ee_mid_pq.data() + 3, p2.data());
            std::copy(mvc_param.ee_end_pq.data(), mvc_param.ee_end_pq.data() + 3, p3.data());
            s_vs(3, p2.data(), p3.data());
            s_vs(3, p1.data(), p2.data());
            s_c3(p2.data(), p3.data(), mul_cross.data());
            double normv = aris::dynamic::s_norm(3, mul_cross.data());
            if (normv <= 1e-10) return -1;

            //通过AC=b 解出圆心C//
            A[0] = 2 * (mvc_param.ee_begin_pq[0] - mvc_param.ee_mid_pq[0]);
            A[1] = 2 * (mvc_param.ee_begin_pq[1] - mvc_param.ee_mid_pq[1]);
            A[2] = 2 * (mvc_param.ee_begin_pq[2] - mvc_param.ee_mid_pq[2]);
            A[3] = 2 * (mvc_param.ee_mid_pq[0] - mvc_param.ee_end_pq[0]);
            A[4] = 2 * (mvc_param.ee_mid_pq[1] - mvc_param.ee_end_pq[1]);
            A[5] = 2 * (mvc_param.ee_mid_pq[2] - mvc_param.ee_end_pq[2]);
            A[6] = (A[1] * A[5] - A[4] * A[2]) / 4;
            A[7] = -(A[0] * A[5] - A[3] * A[2]) / 4;
            A[8] = (A[0] * A[4] - A[3] * A[1]) / 4;
            b[0] = pow(mvc_param.ee_begin_pq[0], 2) + pow(mvc_param.ee_begin_pq[1], 2) + pow(mvc_param.ee_begin_pq[2], 2) - pow(mvc_param.ee_mid_pq[0], 2) - pow(mvc_param.ee_mid_pq[1], 2) - pow(mvc_param.ee_mid_pq[2], 2);
            b[1] = pow(mvc_param.ee_mid_pq[0], 2) + pow(mvc_param.ee_mid_pq[1], 2) + pow(mvc_param.ee_mid_pq[2], 2) - pow(mvc_param.ee_end_pq[0], 2) - pow(mvc_param.ee_end_pq[1], 2) - pow(mvc_param.ee_end_pq[2], 2);
            b[2] = A[6] * mvc_param.ee_begin_pq[0] + A[7] * mvc_param.ee_begin_pq[1] + A[8] * mvc_param.ee_begin_pq[2];

            //解线性方程组
            {
                double pinv[9];
                std::vector<double> U_vec(9);
                auto U = U_vec.data();
                double tau[3];
                aris::Size p[3];
                aris::Size rank;
                s_householder_utp(3, 3, A, U, tau, p, rank, 1e-10);
                double tau2[3];
                s_householder_utp2pinv(3, 3, rank, U, tau, p, pinv, tau2, 1e-10);
                //获取圆心
                s_mm(3, 1, 3, pinv, b, C);
                //获取半径
                R = sqrt(pow(mvc_param.ee_begin_pq[0] - C[0], 2) + pow(mvc_param.ee_begin_pq[1] - C[1], 2) + pow(mvc_param.ee_begin_pq[2] - C[2], 2));
            }

            //求旋转角度theta//
            double u, v, w;
            double u1, v1, w1;
            u = A[6];
            v = A[7];
            w = A[8];
            u1 = (mvc_param.ee_begin_pq[1] - C[1])*(mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]) - (mvc_param.ee_begin_pq[2] - C[2])*(mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]);
            v1 = (mvc_param.ee_begin_pq[2] - C[2])*(mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]) - (mvc_param.ee_begin_pq[0] - C[0])*(mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]);
            w1 = (mvc_param.ee_begin_pq[0] - C[0])*(mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]) - (mvc_param.ee_begin_pq[1] - C[1])*(mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]);

            double H = u * u1 + v * v1 + w * w1;

            // 判断theta 与 pi 的关系
            if (H >= 0)
            {
                theta = 2 * asin(sqrt(pow((mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]), 2) + pow((mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]), 2) + pow((mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]), 2)) / (2 * R));
            }
            else
            {
                theta = 2 * 3.1415 - 2 * asin(sqrt(pow((mvc_param.ee_end_pq[0] - mvc_param.ee_begin_pq[0]), 2) + pow((mvc_param.ee_end_pq[1] - mvc_param.ee_begin_pq[1]), 2) + pow((mvc_param.ee_end_pq[2] - mvc_param.ee_begin_pq[2]), 2)) / (2 * R));
            }

            double normv_begin_pq = aris::dynamic::s_norm(4, mvc_param.ee_begin_pq.data() + 3);
            double normv_end_pq = aris::dynamic::s_norm(4, mvc_param.ee_end_pq.data() + 3);

            ori_theta = std::abs((mvc_param.ee_begin_pq[3] * mvc_param.ee_end_pq[3] + mvc_param.ee_begin_pq[4] * mvc_param.ee_end_pq[4] + mvc_param.ee_begin_pq[5] * mvc_param.ee_end_pq[5] + mvc_param.ee_begin_pq[6] * mvc_param.ee_end_pq[6])/ (normv_begin_pq* normv_end_pq));

            //aris::plan::moveAbsolute(count(), 0.0, theta, mvc_param.vel / 1000 / R, mvc_param.acc / 1000 / 1000 / R, mvc_param.dec / 1000 / 1000 / R, p, v, a, pos_total_count);
            //aris::plan::moveAbsolute(count(), 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0, mvc_param.angular_dec / 1000 / 1000 / ori_theta / 2.0, p, v, a, ori_total_count);

            traplan::sCurve(static_cast<double>(count()), 0.0, theta, mvc_param.vel / 1000 / R, mvc_param.acc / 1000 / 1000 / R, mvc_param.jerk / 1000 / 1000 /1000 / R, p, v, a, j, pos_total_count);
            traplan::sCurve(static_cast<double>(count()), 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0, mvc_param.angular_jerk / 1000 / 1000 /1000 / ori_theta / 2.0, p, v, a, j, ori_total_count);

            pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
            ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;
        }
        */

        /*
        //位置规划//
        double w[3], pmr[16], pqt[7];
        aris::dynamic::s_vc(3, A + 6, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        //aris::plan::moveAbsolute(count(), 0.0, theta, mvc_param.vel / 1000 / R * pos_ratio, mvc_param.acc / 1000 / 1000 / R * pos_ratio * pos_ratio, mvc_param.dec / 1000 / 1000 / R * pos_ratio* pos_ratio, p, v, a, pos_total_count);
        traplan::sCurve(static_cast<double>(count()), 0.0, theta, mvc_param.vel / 1000 / R * pos_ratio, mvc_param.acc / 1000 / 1000 / R * pos_ratio * pos_ratio, mvc_param.jerk / 1000 / 1000 / 1000 / R * pos_ratio* pos_ratio* pos_ratio, p, v, a, j, pos_total_count);

        double pqr[7]{ C[0], C[1], C[2], w[0] * sin(p / 2.0), w[1] * sin(p / 2.0), w[2] * sin(p / 2.0), cos(p / 2.0) };
        double pos[4]{ mvc_param.ee_begin_pq[0] - C[0], mvc_param.ee_begin_pq[1] - C[1], mvc_param.ee_begin_pq[2] - C[2], 1};
        aris::dynamic::s_pq2pm(pqr, pmr);
        s_mm(4, 1, 4, pmr, aris::dynamic::RowMajor{ 4 }, pos, 1, pqt, 1);

        //姿态规划//
        //aris::plan::moveAbsolute(count(), 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0 * ori_ratio, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0 * ori_ratio * ori_ratio, mvc_param.angular_dec / 1000 / 1000 / ori_theta / 2.0* ori_ratio * ori_ratio, p, v, a, ori_total_count);
        traplan::sCurve(static_cast<double>(count()), 0.0, 1.0, mvc_param.angular_vel / 1000 / ori_theta / 2.0 * ori_ratio, mvc_param.angular_acc / 1000 / 1000 / ori_theta / 2.0 * ori_ratio * ori_ratio, mvc_param.angular_jerk / 1000 / 1000 /1000 / ori_theta / 2.0* ori_ratio * ori_ratio* ori_ratio, p, v, a, j, ori_total_count);
        slerp(mvc_param.ee_begin_pq.data() + 3, mvc_param.ee_end_pq.data() + 3, pqt + 3, p);

        //雅克比矩阵判断奇异点//
        {
            auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
            fwd.cptJacobiWrtEE();
            //QR分解求方程的解
            double U[36], tau[6];
            aris::Size p[6];
            Size rank;
            //auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
            //A为输入
            s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-3);
            if(rank < 6)return -1002;
        }

        // set目标位置，并进行运动学反解 //
        //model()->generalMotionPool().at(0).setMpq(pqt);
        mvc_param.tool->setPq(*mvc_param.wobj, pqt);
        model()->generalMotionPool().at(0).updMpm();

        if (model()->solverPool().at(0).kinPos())return -1;

        ////////////////////////////////////// log ///////////////////////////////////////
        auto &lout = controller()->lout();

        if (g_move_pause)
        {
            return -4;
        }
        return std::max(pos_total_count, ori_total_count) > count() ? 1 : 0;
        */

        double pqt[7], pre_pqt[7];
        if (mvc_param->pre_plan == nullptr) //转弯第一条指令
        {
            if (count() == 1)
            {
                // init joint_pos //
                for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
                {
                    model()->motionPool().at(i).setMp(controller()->motionPool()[i].targetPos());
                }
                if (model()->solverPool().at(1).kinPos())return -1;
                model()->generalMotionPool().at(0).updMpm();
                mvc_param->tool->getPq(*mvc_param->wobj, mvc_param->ee_begin_pq.data());
                if(!cal_circle_par(*mvc_param)) return -1;
            }
            cal_pqt(*mvc_param, *this, pqt);
        }
        else if (mvc_param->pre_plan->name() == "MoveC") //转弯区第二条指令或者第n条指令
        {
            auto &param = std::any_cast<MoveCParam&>(mvc_param->pre_plan->param());
            auto zonecount = mvc_param->pre_plan->realzone.load();

            //count数小于等于上一条指令的realzone，zone起作用//
            if (g_count <= zonecount)
            {
                //preplan//
                double w[3], pmr[16], p, v, a, j;
                aris::Size pos_total_count, ori_total_count;

                //位置规划//
                aris::dynamic::s_vc(3, param.A + 6, w);
                auto normv = aris::dynamic::s_norm(3, w);
                if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
                traplan::sCurve(param.max_total_count - zonecount + g_count, 0.0, param.theta, param.vel / 1000 / param.R * param.pos_ratio, param.acc / 1000 / 1000 / param.R * param.pos_ratio * param.pos_ratio,
                    param.jerk / 1000 / 1000 / 1000 / param.R * param.pos_ratio * param.pos_ratio * param.pos_ratio, p, v, a, j, pos_total_count);

                double pqr[7]{ param.C[0], param.C[1], param.C[2], w[0] * sin(p / 2.0), w[1] * sin(p / 2.0), w[2] * sin(p / 2.0), cos(p / 2.0) };
                double pos[4]{ param.ee_begin_pq[0] - param.C[0], param.ee_begin_pq[1] - param.C[1], param.ee_begin_pq[2] - param.C[2], 1 };
                aris::dynamic::s_pq2pm(pqr, pmr);
                s_mm(4, 1, 4, pmr, aris::dynamic::RowMajor{ 4 }, pos, 1, pre_pqt, 1);

                //姿态规划//
                traplan::sCurve(param.max_total_count - zonecount + g_count, 0.0, 1.0, param.angular_vel / 1000 / param.ori_theta / 2.0 * param.ori_ratio, param.angular_acc / 1000 / 1000 / param.ori_theta / 2.0 * param.ori_ratio * param.ori_ratio,
                    param.angular_jerk / 1000 / 1000 / 1000 / param.ori_theta / 2.0 * param.ori_ratio * param.ori_ratio * param.ori_ratio, p, v, a, j, ori_total_count);
                slerp(param.ee_begin_pq.data() + 3, param.ee_end_pq.data() + 3, pre_pqt + 3, p);

                //thisplan//
                cal_pqt(*mvc_param, *this, pqt);

                //求目标位姿//
                for (aris::Size i = 0; i < 7; i++)
                {
                    pqt[i] = 1.0*(zonecount - g_count) / zonecount * pre_pqt[i] + 1.0*g_count / zonecount * pqt[i];
                }
            }
            else
            {
                mvc_param->pre_plan->cmd_finished.store(true);
                cal_pqt(*mvc_param, *this, pqt);
            }
        }
        else//本条指令设置了转弯区，但是与上一条指令无法实现转弯，比如moveJ,moveL,moveAbsJ
        {
            cal_pqt(*mvc_param, *this, pqt);
        }

        //雅克比矩阵判断奇异点//
        {
            auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(model()->solverPool()[1]);
            fwd.cptJacobiWrtEE();
            //QR分解求方程的解
            double U[36], tau[6];
            aris::Size p[6];
            Size rank;
            //auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
            //A为输入
            s_householder_utp(6, 6, fwd.Jf(), U, tau, p, rank, 1e-3);
            if (rank < 6)return -1002;
        }

        // 更新目标点 //
        mvc_param->tool->setPq(*mvc_param->wobj, pqt);
        model()->generalMotionPool().at(0).updMpm();

        //运动学反解//
        if (model()->solverPool().at(0).kinPos())return -1;

        auto rzcount = this->realzone.load();
        //本条指令没有转弯区
        PauseContinueE(mvc_param, pwinter, rzcount);

        if (mvc_param->max_total_count - rzcount - int32_t(g_count) == 0)
        {
            //realzone为0时，返回值为0时，本条指令执行完毕
            if (rzcount == 0)this->cmd_finished.store(true);
            controller()->mout() << "mvc_param->max_total_count:" << mvc_param->max_total_count << "this->realzone.load():" << rzcount << std::endl;
        }
        if (pwinter.isAutoStopped() && (g_counter == 0))
        {
            //指令停止且返回值为0时，本条指令执行完毕
            this->cmd_finished.store(true);
            g_plan.reset();
            return 0;
        }
        return mvc_param->max_total_count == 0 ? 0 : mvc_param->max_total_count - rzcount - int32_t(g_count);
    }
    auto MoveC::collectNrt()->void
    {
        if(this->retCode()<0)
        {
            g_plan.reset();
        }
        std::any_cast<MoveCParam>(&this->param())->pre_plan.reset();
    }
    MoveC::~MoveC() = default;
    MoveC::MoveC(const MoveC &other) = default;
    MoveC::MoveC(const std::string &name) :MoveBase(name)
    {
        command().loadXmlStr(
            "<Command name=\"mvc\">"
            "	<GroupParam>"
            "		<Param name=\"pos_unit\" default=\"m\"/>"
            "		<UniqueParam default=\"mid_pq\">"
            "			<Param name=\"mid_pq\" default=\"{0,0,0,0,0,0,1}\"/>"
            "			<Param name=\"mid_pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
            "			<GroupParam>"
            "				<Param name=\"mid_pe\" default=\"{0,0,0,0,0,0}\"/>"
            "				<Param name=\"mid_ori_unit\" default=\"rad\"/>"
            "				<Param name=\"mid_eul_type\" default=\"321\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "		<UniqueParam default=\"end_pq\">"
            "			<Param name=\"end_pq\" default=\"{0,0,0,0,0,0,1}\"/>"
            "			<Param name=\"end_pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
            "			<GroupParam>"
            "				<Param name=\"end_pe\" default=\"{0,0,0,0,0,0}\"/>"
            "				<Param name=\"end_ori_unit\" default=\"rad\"/>"
            "				<Param name=\"end_eul_type\" default=\"321\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "		<Param name=\"acc\" default=\"0.5\"/>"
            "		<Param name=\"vel\" default=\"{0.025, 0.025, 3.4, 0.0, 0.0}\"/>"
            "		<Param name=\"dec\" default=\"0.5\"/>"
            "		<Param name=\"jerk\" default=\"10\"/>"
            "		<Param name=\"angular_acc\" default=\"0.5\"/>"
            "		<Param name=\"angular_vel\" default=\"0.1\"/>"
            "		<Param name=\"angular_dec\" default=\"0.5\"/>"
            "		<Param name=\"angular_jerk\" default=\"10\"/>"
            "		<Param name=\"speed\" default=\"{0.1, 0.1, 3.49, 0.0, 0.0}\"/>"
            "		<Param name=\"zone\" default=\"fine\"/>"
            "		<Param name=\"load\" default=\"{1,0.05,0.05,0.05,0,0.97976,0,0.200177,1.0,1.0,1.0}\"/>"
            "		<Param name=\"tool\" default=\"tool0\"/>"
            "		<Param name=\"wobj\" default=\"wobj0\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    //复现文件位置//
    struct MoveFParam
    {
        std::vector<Size> total_count_vec;
        std::vector<double> axis_begin_pos_vec;
        std::vector<double> axis_first_pos_vec;
        std::vector<std::vector<double>> pos_vec;
        double vel, acc, dec;
        std::string path;
    };
    auto MoveF::prepareNrt()->void
    {
        MoveFParam param;
        param.vel = doubleParam("vel");
        param.acc = doubleParam("acc");
        param.dec = doubleParam("dec");
        param.path = cmdParams().at("path");

        std::cout<<param.path;
        param.total_count_vec.resize(6, 0);
        param.axis_begin_pos_vec.resize(6, 0.0);
        param.axis_first_pos_vec.resize(6, 0.0);

        param.pos_vec.resize(6, std::vector<double>(1, 0.0));
        //std::cout << "size:" << param.pos_vec.size() << std::endl;
        //初始化pos_vec//
        for (int j = 0; j < param.pos_vec.size(); j++)
        {
            param.pos_vec[j].clear();
        }

        //定义读取log文件的输入流oplog//
        std::ifstream oplog;
        int cal = 0;
        oplog.open(param.path);

        //以下检查是否成功读取文件//
        if (!oplog)
        {
            throw std::runtime_error("fail to open the file");
        }
        while (!oplog.eof())
        {
            for (int j = 0; j < param.pos_vec.size(); j++)
            {
                double data;
                oplog >> data;
                param.pos_vec[j].push_back(data);
            }
        }
        oplog.close();
        //oplog.clear();
        for (int j = 0; j < param.pos_vec.size(); j++)
        {
            param.pos_vec[j].pop_back();
            param.axis_first_pos_vec[j] = param.pos_vec[j][0];
        }

        this->param() = param;
        std::fill(motorOptions().begin(), motorOptions().end(),
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER|
            Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MoveF::executeRT()->int
    {
        auto &param = std::any_cast<MoveFParam&>(this->param());

        double p, v, a;
        aris::Size t_count;
        static aris::Size first_total_count = 1;
        aris::Size total_count = 1;
        aris::Size return_value = 0;

        // 获取6个电机初始位置 //
        if (count() == 1)
        {
            for (int i = 0; i < 6; i++)
            {
                param.axis_begin_pos_vec[i] = model()->motionPool().at(i).mp();
            }
            for (int i = 0; i < 6; i++)
            {
                // 梯形规划到log开始点 //
                aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
                first_total_count = std::max(first_total_count, t_count);
            }
        }

        // 机械臂走到log开始点 //
        if (count() <= first_total_count)
        {
            for (int i = 0; i < 6; i++)
            {
                // 在第一个周期走梯形规划复位
                aris::plan::moveAbsolute(count(), param.axis_begin_pos_vec[i], param.axis_first_pos_vec[i], param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
                controller()->motionAtAbs(i).setTargetPos(p);
                model()->motionPool().at(i).setMp(p);
            }
        }

        // 机械臂开始从头到尾复现log中点 //
        if (count() > first_total_count)
        {
            for (int i = 0; i < 6; i++)
            {
                controller()->motionAtAbs(i).setTargetPos(param.pos_vec[i][count() - first_total_count]);
                model()->motionPool().at(i).setMp(param.pos_vec[i][count() - first_total_count]);
            }
        }
        if (model()->solverPool().at(1).kinPos())return -1;

        //输出6个轴的实时位置log文件//
        auto &lout = controller()->lout();
        for (int i = 0; i < 6; i++)
        {
            lout << controller()->motionAtAbs(i).actualPos() << " ";//第一列数字必须是位置
        }
        lout << std::endl;

        return count() > (first_total_count - 2 + param.pos_vec[0].size()) ? 0 : 1;

    }
    auto MoveF::collectNrt()->void {}
    MoveF::MoveF(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"mvf\">"
            "	<GroupParam>"
            "		<Param name=\"path\" default=\"/home/kaanh/Desktop/build-Kaanh-gk-Desktop_Qt_5_11_2_GCC_64bit-Default/log/motion_replay.txt\"/>"
            "		<Param name=\"vel\" default=\"0.5\" abbreviation=\"v\"/>"
            "		<Param name=\"acc\" default=\"0.6\" abbreviation=\"a\"/>"
            "		<Param name=\"dec\" default=\"0.6\" abbreviation=\"d\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // 示教运动--输入末端大地坐标系的位姿pe，控制动作 //
    struct JogCParam {};
    struct JogCStruct
    {
        bool jogc_is_running = false;
        int cor_system;
        int vel_percent;
        std::array<int, 6> is_increase;
    };
    struct JogC::Imp
    {
        JogCStruct s1_rt, s2_nrt;
        std::vector<double> pm_target;
        double vel[6], acc[6], dec[6];
        int increase_count;
    };
    std::atomic_bool jogc_is_changing = false;
    auto JogC::prepareNrt()->void
    {
        auto c = controller();
        JogCParam param;
        imp_->pm_target.resize(16, 0.0);

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        for (auto &p : cmdParams())
        {
            if (p.first == "start")
            {
                if (imp_->s1_rt.jogc_is_running)throw std::runtime_error("auto mode already started");

                imp_->s2_nrt.jogc_is_running = true;
                std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);
                imp_->s2_nrt.cor_system= 0;
                imp_->s2_nrt.vel_percent = 10;

                imp_->s1_rt.jogc_is_running = true;
                std::fill_n(imp_->s1_rt.is_increase.data(), 6, 0);
                imp_->s1_rt.cor_system = 0;
                imp_->s1_rt.vel_percent = 10;

                imp_->increase_count = std::stoi(std::string(cmdParams().at("increase_count")));
                if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");

                auto mat = matrixParam(cmdParams().at("vel"));
                if (mat.size() != 6)THROW_FILE_LINE("");
                std::copy(mat.begin(), mat.end(), imp_->vel);

                mat = matrixParam(cmdParams().at("acc"));
                if (mat.size() != 6)THROW_FILE_LINE("");
                std::copy(mat.begin(), mat.end(), imp_->acc);

                mat = matrixParam(cmdParams().at("dec"));
                if (mat.size() != 6)THROW_FILE_LINE("");
                std::copy(mat.begin(), mat.end(), imp_->dec);

                std::fill(motorOptions().begin(), motorOptions().end(), USE_TARGET_POS);
                //target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
            }
            else if (p.first == "stop")
            {
                if (!imp_->s1_rt.jogc_is_running)throw std::runtime_error("manual mode not started, when stop");

                imp_->s2_nrt.jogc_is_running = false;
                std::fill_n(imp_->s2_nrt.is_increase.data(), 6, 0);

                option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
                jogc_is_changing = true;
                while (jogc_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            else if (p.first == "cor")
            {
                if (!imp_->s1_rt.jogc_is_running)throw std::runtime_error("manual mode not started, when pe");

                imp_->s2_nrt.cor_system = int32Param("cor");
                auto velocity = int32Param("vel_percent");
                velocity = std::max(std::min(100, velocity), 0);
                imp_->s2_nrt.vel_percent = velocity;
                imp_->s2_nrt.is_increase[0] = std::max(std::min(1, int32Param("x")), -1) * imp_->increase_count;
                imp_->s2_nrt.is_increase[1] = std::max(std::min(1, int32Param("y")), -1) * imp_->increase_count;
                imp_->s2_nrt.is_increase[2] = std::max(std::min(1, int32Param("z")), -1) * imp_->increase_count;
                imp_->s2_nrt.is_increase[3] = std::max(std::min(1, int32Param("a")), -1) * imp_->increase_count;
                imp_->s2_nrt.is_increase[4] = std::max(std::min(1, int32Param("b")), -1) * imp_->increase_count;
                imp_->s2_nrt.is_increase[5] = std::max(std::min(1, int32Param("c")), -1) * imp_->increase_count;

                imp_->s2_nrt.jogc_is_running = true;

                option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
                jogc_is_changing = true;
                while (jogc_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        this->param() = param;
    }
    auto JogC::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        static int increase_status[6]{ 0,0,0,0,0,0 };
        double max_vel[6];
        if (jogc_is_changing)
        {
            imp_->s1_rt = imp_->s2_nrt;
            jogc_is_changing.store(false);
            for (int i = 0; i < 6; i++)
            {
                increase_status[i] = imp_->s1_rt.is_increase[i];
            }
            //model()->generalMotionPool()[0].getMpe(imp_->pe_start, eu_type);
        }

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = imp_->vel[i]*1.0*imp_->s1_rt.vel_percent / 100.0;
            target_p[i] += aris::dynamic::s_sgn(increase_status[i])*max_vel[i] * 1e-3;
            increase_status[i] -= aris::dynamic::s_sgn(increase_status[i]);
        }
        //std::copy_n(target_pos, 6, imp_->pe_start);

        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        for(int i=0; i<6; i++)
        {
            aris::Size t;
            aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], imp_->acc[i], imp_->dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0]*sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        model()->generalMotionPool()[0].getMpm(pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);
        /*
        s_pe2pm(p_now, imp_->pm_now.data(), eu_type);
        for (int i = 0; i < 6; i++)
        {
            p_now[i] = p_next[i];
            v_now[i] = v_next[i];
            a_now[i] = a_next[i];
        }
        */

        //绝对坐标系
        if (imp_->s1_rt.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, imp_->pm_target.data());
        }
        //工具坐标系
        else if (imp_->s1_rt.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, imp_->pm_target.data());
        }

        model()->generalMotionPool().at(0).setMpm(imp_->pm_target.data());

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;

        // 打印 //
        auto &cout = controller()->mout();
        if (count() % 200 == 0)
        {
            cout << "pm_target:" << std::endl;
            for (Size i = 0; i < 16; i++)
            {
                cout << imp_->pm_target[i] << "  ";
            }
            cout << std::endl;
            cout << "increase_status:" << std::endl;
            for (Size i = 0; i < 6; i++)
            {
                cout << increase_status[i] << "  ";
            }
            cout << std::endl;
            cout << "p_next:" << std::endl;
            for (Size i = 0; i < 6; i++)
            {
                cout << p_next[i] << "  ";
            }
            cout << std::endl;
            cout << "v_next:" << std::endl;
            for (Size i = 0; i < 6; i++)
            {
                cout << v_next[i] << "  ";
            }
            cout << std::endl;
            cout << "p_now:" << std::endl;
            for (Size i = 0; i < 6; i++)
            {
                cout << p_now[i] << "  ";
            }
            cout << std::endl;
            cout << "v_now:" << std::endl;
            for (Size i = 0; i < 6; i++)
            {
                cout << v_now[i] << "  ";
            }
            cout << std::endl;
        }

        // log //
        auto &lout = controller()->lout();
        for (int i = 0; i < 6; i++)
        {
            lout << target_p[i] << " ";
            lout << p_now[i] << " ";
            lout << v_now[i] << " ";
            lout << a_now[i] << " ";
            lout << controller()->motionAtAbs(i).actualPos() << " ";
            lout << controller()->motionAtAbs(i).actualVel() << " ";
        }
        lout << std::endl;

        return imp_->s1_rt.jogc_is_running ? 1 : 0;
    }
    auto JogC::collectNrt()->void {}
    JogC::~JogC() = default;
    JogC::JogC(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jogC\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<GroupParam name=\"start_group\">"
            "				<Param name=\"start\"/>"
            "				<Param name=\"increase_count\" default=\"100\"/>"
            "				<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.25,0.25,0.25}\"/>"
            "				<Param name=\"acc\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
            "				<Param name=\"dec\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"
            "			</GroupParam>"
            "			<Param name=\"stop\"/>"
            "			<GroupParam>"
            "				<Param name=\"cor\" default=\"0\"/>"
            "				<Param name=\"vel_percent\" default=\"10\"/>"
            "				<Param name=\"x\" default=\"0\"/>"
            "				<Param name=\"y\" default=\"0\"/>"
            "				<Param name=\"z\" default=\"0\"/>"
            "				<Param name=\"a\" default=\"0\"/>"
            "				<Param name=\"b\" default=\"0\"/>"
            "				<Param name=\"c\" default=\"0\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }
    JogC::JogC(const JogC &other) = default;
    JogC::JogC(JogC &other) = default;
    JogC& JogC::operator=(const JogC &other) = default;
    JogC& JogC::operator=(JogC &&other) = default;


    // 示教运动--关节空间点动 //
    struct JogJStruct
    {
        bool jogj_is_running = false;
        int vel_percent;
        std::vector<int> is_increase;
    };
    struct JogJ::Imp
    {
        JogJStruct s1_rt, s2_nrt;
        double vel, acc, dec;
        std::vector<double> p_now, v_now, a_now, target_pos, max_vel;
        std::vector<int> increase_status;
        int increase_count;
    };
    std::atomic_bool jogj_is_changing = false;
    auto JogJ::prepareNrt()->void
    {
        auto c = controller();

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        imp_->p_now.resize(c->motionPool().size(), 0.0);
        imp_->v_now.resize(c->motionPool().size(), 0.0);
        imp_->a_now.resize(c->motionPool().size(), 0.0);
        imp_->target_pos.resize(c->motionPool().size(), 0.0);
        imp_->max_vel.resize(c->motionPool().size(), 0.0);
        imp_->increase_status.resize(c->motionPool().size(), 0);

        for (auto &p : cmdParams())
        {
            if (p.first == "start")
            {
                if (imp_->s1_rt.jogj_is_running)throw std::runtime_error("auto mode already started");

                imp_->s2_nrt.jogj_is_running = true;
                imp_->s2_nrt.is_increase.clear();
                imp_->s2_nrt.is_increase.resize(c->motionPool().size(), 0);
                imp_->s2_nrt.vel_percent = 10;

                imp_->s1_rt.jogj_is_running = true;
                imp_->s1_rt.is_increase.clear();
                imp_->s1_rt.is_increase.resize(c->motionPool().size(), 0);
                imp_->s1_rt.vel_percent = 10;

                imp_->increase_count = int32Param("increase_count");
                if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");
                imp_->vel = doubleParam("vel");
                imp_->acc = doubleParam("acc");
                imp_->dec = doubleParam("dec");

                std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_POS_FOLLOWING_ERROR | USE_TARGET_POS);
                //target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
            }
            else if (p.first == "stop")
            {
                if (!imp_->s1_rt.jogj_is_running)throw std::runtime_error("manual mode not started, when stop");

                imp_->s2_nrt.jogj_is_running = false;
                imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
                jogj_is_changing = true;
                while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
                option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
            }
            else if (p.first == "vel_percent")
            {
                if (!imp_->s1_rt.jogj_is_running)throw std::runtime_error("manual mode not started, when pe");

                auto velocity = int32Param("vel_percent");
                velocity = std::max(std::min(100, velocity), 0);
                imp_->s2_nrt.vel_percent = velocity;

                imp_->s2_nrt.is_increase.assign(imp_->s2_nrt.is_increase.size(), 0);
                imp_->s2_nrt.is_increase[int32Param("motion_id")] = std::max(std::min(1, int32Param("direction")), -1) * imp_->increase_count;

                imp_->s2_nrt.jogj_is_running = true;

                option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
                jogj_is_changing = true;
                while (jogj_is_changing.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

    }
    auto JogJ::executeRT()->int
    {
        // get current pe //
        if (count() == 1)
        {
            for (Size i = 0; i < imp_->p_now.size(); ++i)
            {
                /*
                imp_->p_start[i] = model()->motionPool().at(i).mp();
                imp_->p_now[i] = model()->motionPool().at(i).mp();
                imp_->v_now[i] = model()->motionPool().at(i).mv();
                imp_->a_now[i] = model()->motionPool().at(i).ma();
                */
                imp_->target_pos[i] = controller()->motionAtAbs(i).actualPos();
                imp_->p_now[i] = controller()->motionAtAbs(i).actualPos();
                imp_->v_now[i] = controller()->motionAtAbs(i).actualVel();
                imp_->a_now[i] = 0.0;
            }
        }
        // init status and calculate target pos and max vel //

        if (jogj_is_changing)
        {
            jogj_is_changing.store(false);
            imp_->s1_rt = imp_->s2_nrt;
            for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
            {
                imp_->increase_status[i] = imp_->s1_rt.is_increase[i];
            }
        }
        for (int i = 0; i < imp_->s1_rt.is_increase.size(); i++)
        {
            imp_->max_vel[i] = imp_->vel*1.0*imp_->s1_rt.vel_percent / 100.0;
            imp_->target_pos[i] += aris::dynamic::s_sgn(imp_->increase_status[i])*imp_->max_vel[i] * 1e-3;
            imp_->increase_status[i] -= aris::dynamic::s_sgn(imp_->increase_status[i]);
        }
        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;
        for (int i = 0; i < imp_->p_now.size(); i++)
        {
            aris::Size t;
            aris::plan::moveAbsolute2(imp_->p_now[i], imp_->v_now[i], imp_->a_now[i]
                , imp_->target_pos[i], 0.0, 0.0
                , imp_->max_vel[i], imp_->acc, imp_->dec
                , 1e-3, 1e-10, p_next, v_next, a_next, t);

            model()->motionPool().at(i).setMp(p_next);
            controller()->motionAtAbs(i).setTargetPos(p_next);
            imp_->p_now[i] = p_next;
            imp_->v_now[i] = v_next;
            imp_->a_now[i] = a_next;
        }

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        // 打印 //
        auto &cout = controller()->mout();
        if (count() % 200 == 0)
        {
            for (int i = 0; i < imp_->p_now.size(); i++)
            {
                cout << imp_->increase_status[i] << "  ";
            }
            cout << std::endl;
            for (int i = 0; i < imp_->p_now.size(); i++)
            {
                cout << imp_->target_pos[i] << "  ";
            }
            cout << std::endl;
            for (int i = 0; i < imp_->p_now.size(); i++)
            {
                cout << imp_->p_now[i] << "  ";
            }
            cout << std::endl;
            for (int i = 0; i < imp_->p_now.size(); i++)
            {
                cout << imp_->v_now[i] << "  ";
            }
            cout << std::endl;
            cout << "------------------------------------------" << std::endl;
        }

        // log //
        auto &lout = controller()->lout();
        for (int i = 0; i < imp_->p_now.size(); i++)
        {
            lout << imp_->target_pos[i] << " ";
            lout << imp_->v_now[i] << " ";
            lout << imp_->a_now[i] << " ";
            lout << controller()->motionAtAbs(i).actualPos() << " ";
            lout << controller()->motionAtAbs(i).actualVel() << " ";
            //lout << controller->motionAtAbs(i).actualCur() << " ";
        }
        lout << std::endl;

        return imp_->s1_rt.jogj_is_running ? 1 : 0;
    }
    auto JogJ::collectNrt()->void {}
    JogJ::~JogJ() = default;
    JogJ::JogJ(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jogJ\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<GroupParam name=\"start_group\">"
            "				<Param name=\"start\"/>"
            "				<Param name=\"increase_count\" default=\"100\"/>"
            "				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
            "				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
            "				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
            "			</GroupParam>"
            "			<Param name=\"stop\"/>"
            "			<GroupParam>"
            "				<Param name=\"vel_percent\" default=\"10\"/>"
            "				<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
            "				<Param name=\"direction\" default=\"1\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }
    JogJ::JogJ(const JogJ &other) = default;
    JogJ::JogJ(JogJ &other) = default;
    JogJ& JogJ::operator=(const JogJ &other) = default;
    JogJ& JogJ::operator=(JogJ &&other) = default;


#define JOGJ_PARAM_STRING \
        "	<UniqueParam>"\
        "		<GroupParam>"\
        "			<Param name=\"increase_count\" default=\"200\"/>"\
        "			<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"\
        "			<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"\
        "			<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"\
        "			<Param name=\"vel_percent\" default=\"10\"/>"\
        "			<Param name=\"direction\" default=\"1\"/>"\
        "		</GroupParam>"\
        "		<Param name=\"stop\"/>"\
        "	</UniqueParam>"
    // 示教运动--关节1点动 //
    struct JogJParam
    {
        int motion_id;
        double vel, acc, dec;
        double p_now, v_now, a_now, target_pos, max_vel;
        int increase_status;
        int increase_count;
        int vel_percent;
        static std::atomic_int32_t j1_count, j2_count, j3_count, j4_count, j5_count, j6_count, j7_count;
    };
    template<typename JogType>
    auto set_jogj_input_param(JogType* this_p, const std::map<std::string_view, std::string_view> &cmd_param, JogJParam &param, std::atomic_int32_t& j_count)->void
    {
        auto&cs = aris::server::ControlServer::instance();
        param.p_now = 0.0;
        param.v_now = 0.0;
        param.a_now = 0.0;
        param.target_pos = 0.0;
        param.max_vel = 0.0;
        param.increase_status = 0;
        param.vel_percent = 0;

        for (auto &p : cmd_param)
        {
            if (p.first == "increase_count")
            {
                param.increase_count = this_p->int32Param("increase_count");
                if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

                param.vel = this_p->controller()->motionPool().at(param.motion_id).maxVel()*g_vel.w_per;
                param.acc = std::min(std::max(this_p->doubleParam("acc"), 0.0), 1.0)*this_p->controller()->motionPool().at(param.motion_id).maxAcc();
                param.dec = std::min(std::max(this_p->doubleParam("dec"), 0.0), 1.0)*this_p->controller()->motionPool().at(param.motion_id).maxAcc();

                auto velocity = this_p->int32Param("vel_percent");
                velocity = std::max(std::min(100, velocity), 0);
                param.vel_percent = velocity;
                param.increase_status = std::max(std::min(1, this_p->int32Param("direction")), -1);

                std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();
                //当前有指令在执行//
                if (planptr && planptr->cmdName() != this_p->cmdName())throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");
                if (j_count.exchange(param.increase_count))
                {
                    this_p->option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
                }
                else
                {
                    std::fill(this_p->motorOptions().begin(), this_p->motorOptions().end(), aris::plan::Plan::MotorOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotorOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotorOption::USE_TARGET_POS | aris::plan::Plan::MotorOption::NOT_CHECK_ENABLE);
                    this_p->motorOptions()[param.motion_id] = aris::plan::Plan::MotorOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotorOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotorOption::USE_TARGET_POS;
                }
            }
            else if (p.first == "stop")
            {
                j_count.exchange(0);
                this_p->option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
            }
        }
    }
    std::atomic_int32_t JogJParam::j1_count = 0;
    auto JogJ1::prepareNrt()->void
    {
        JogJParam param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 0;
        set_jogj_input_param(this, cmdParams(), param, param.j1_count);

        this->param() = param;
    }
    auto JogJ1::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            /*
            imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
            imp_->a_now = 0.0;
            */
            param.target_pos = model()->motionPool().at(param.motion_id).mp();
            param.p_now = model()->motionPool()[param.motion_id].mp();
            param.v_now = model()->motionPool()[param.motion_id].mv();
            param.a_now = 0.0;

        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;

        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        if (param.j1_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j1_count;
        }

        model()->motionPool().at(param.motion_id).setMp(p_next);
        //controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        return finished;
    }
    auto JogJ1::collectNrt()->void
    {
        JogJParam::j1_count=0;
        if (retCode() < 0)
        {
            JogJParam::j1_count.store(0);
        }
    }
    JogJ1::~JogJ1() = default;
    JogJ1::JogJ1(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j1\">"
                JOGJ_PARAM_STRING
            "</Command>");
    }


    // 示教运动--关节2点动 //
    std::atomic_int32_t JogJParam::j2_count = 0;
    auto JogJ2::prepareNrt()->void
    {
        JogJParam param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 1;

        set_jogj_input_param(this, cmdParams(), param, param.j2_count);

        this->param() = param;
    }
    auto JogJ2::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            /*
            imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
            imp_->a_now = 0.0;
            */
            param.target_pos = model()->motionPool().at(param.motion_id).mp();
            param.p_now = model()->motionPool()[param.motion_id].mp();
            param.v_now = model()->motionPool()[param.motion_id].mv();
            param.a_now = 0.0;

        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;

        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        if (param.j2_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j2_count;
        }

        model()->motionPool().at(param.motion_id).setMp(p_next);
        //controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        return finished;
    }
    auto JogJ2::collectNrt()->void
    {
        JogJParam::j2_count = 0;
        if (retCode() < 0)
        {
            JogJParam::j2_count.store(0);
        }
    }
    JogJ2::~JogJ2() = default;
    JogJ2::JogJ2(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j2\">"
                JOGJ_PARAM_STRING
            "</Command>");
    }


    // 示教运动--关节3点动 //
    std::atomic_int32_t JogJParam::j3_count = 0;
    auto JogJ3::prepareNrt()->void
    {
        JogJParam param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 2;
        set_jogj_input_param(this, cmdParams(), param, param.j3_count);

        this->param() = param;
    }
    auto JogJ3::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            /*
            imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
            imp_->a_now = 0.0;
            */
            param.target_pos = model()->motionPool().at(param.motion_id).mp();
            param.p_now = model()->motionPool()[param.motion_id].mp();
            param.v_now = model()->motionPool()[param.motion_id].mv();
            param.a_now = 0.0;

        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;

        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        if (param.j3_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j3_count;
        }

        model()->motionPool().at(param.motion_id).setMp(p_next);
        //controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        return finished;
    }
    auto JogJ3::collectNrt()->void
    {
        JogJParam::j3_count = 0;
        if (retCode() < 0)
        {
            JogJParam::j3_count.store(0);
        }
    }
    JogJ3::~JogJ3() = default;
    JogJ3::JogJ3(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j3\">"
                JOGJ_PARAM_STRING
            "</Command>");
    }


    // 示教运动--关节4点动 //
    std::atomic_int32_t JogJParam::j4_count = 0;
    auto JogJ4::prepareNrt()->void
    {
        JogJParam param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 3;
        set_jogj_input_param(this, cmdParams(), param, param.j4_count);

        this->param() = param;
    }
    auto JogJ4::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            /*
            imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
            imp_->a_now = 0.0;
            */
            param.target_pos = model()->motionPool().at(param.motion_id).mp();
            param.p_now = model()->motionPool()[param.motion_id].mp();
            param.v_now = model()->motionPool()[param.motion_id].mv();
            param.a_now = 0.0;

        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;

        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        if (param.j4_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j4_count;
        }

        model()->motionPool().at(param.motion_id).setMp(p_next);
        //controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        return finished;
    }
    auto JogJ4::collectNrt()->void
    {
        JogJParam::j4_count = 0;
        if (retCode() < 0)
        {
            JogJParam::j4_count.store(0);
        }
    }
    JogJ4::~JogJ4() = default;
    JogJ4::JogJ4(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j4\">"
                JOGJ_PARAM_STRING
            "</Command>");
    }


    // 示教运动--关节5点动 //
    std::atomic_int32_t JogJParam::j5_count = 0;
    auto JogJ5::prepareNrt()->void
    {
        JogJParam param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 4;
        set_jogj_input_param(this, cmdParams(), param, param.j5_count);

        this->param() = param;
    }
    auto JogJ5::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            /*
            imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
            imp_->a_now = 0.0;
            */
            param.target_pos = model()->motionPool().at(param.motion_id).mp();
            param.p_now = model()->motionPool()[param.motion_id].mp();
            param.v_now = model()->motionPool()[param.motion_id].mv();
            param.a_now = 0.0;

        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;

        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        if (param.j5_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j5_count;
        }

        model()->motionPool().at(param.motion_id).setMp(p_next);
        //controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        return finished;
    }
    auto JogJ5::collectNrt()->void
    {
        JogJParam::j5_count = 0;
        if (retCode() < 0)
        {
            JogJParam::j5_count.store(0);
        }
    }
    JogJ5::~JogJ5() = default;
    JogJ5::JogJ5(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j5\">"
                JOGJ_PARAM_STRING
            "</Command>");
    }


    // 示教运动--关节6点动 //
    std::atomic_int32_t JogJParam::j6_count = 0;
    auto JogJ6::prepareNrt()->void
    {
        JogJParam param;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 5;
        set_jogj_input_param(this, cmdParams(), param, param.j6_count);

        this->param() = param;
    }
    auto JogJ6::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            /*
            imp_->target_pos = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->p_now = controller->motionAtAbs(imp_->motion_id).actualPos();
            imp_->v_now = controller->motionAtAbs(imp_->motion_id).actualVel();
            imp_->a_now = 0.0;
            */
            param.target_pos = model()->motionPool().at(param.motion_id).mp();
            param.p_now = model()->motionPool()[param.motion_id].mp();
            param.v_now = model()->motionPool()[param.motion_id].mv();
            param.a_now = 0.0;

        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;
        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        //当计数器j6_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.j6_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j6_count;
        }

        model()->motionPool().at(param.motion_id).setMp(p_next);
        //controller->motionAtAbs(imp_->motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        // 运动学正解//
        if (model()->solverPool().at(1).kinPos())return -1;

        return finished;
    }
    auto JogJ6::collectNrt()->void
    {
        JogJParam::j6_count = 0;
        if (retCode() < 0)
        {
            JogJParam::j6_count.store(0);
        }
    }
    JogJ6::~JogJ6() = default;
    JogJ6::JogJ6(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j6\">"
                JOGJ_PARAM_STRING
            "</Command>");
    }


    // 示教运动--外部轴点动 //
    std::atomic_int32_t JogJParam::j7_count = 0;
    auto JogJ7::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        auto c = controller();
        JogJParam param;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        param.motion_id = 6;
        param.p_now = 0.0;
        param.v_now = 0.0;
        param.a_now = 0.0;
        param.target_pos = 0.0;
        param.max_vel = 0.0;
        param.increase_status = 0;
        param.vel_percent = 0;

        for (auto &p : cmdParams())
        {
            if (p.first == "increase_count")
            {
                param.increase_count = int32Param("increase_count");
                if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

                param.vel = std::min(std::max(doubleParam("vel"), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxVel();
                param.acc = std::min(std::max(doubleParam("acc"), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();
                param.dec = std::min(std::max(doubleParam("dec"), 0.0), 1.0)*c->motionPool().at(param.motion_id).maxAcc();

                auto velocity = int32Param("vel_percent");
                velocity = std::max(std::min(100, velocity), 0);
                param.vel_percent = velocity;
                param.increase_status = std::max(std::min(1, int32Param("direction")), -1);

                std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();
                //当前有指令在执行//
                if (planptr && planptr->cmdName() != this->cmdName())throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");

                if (param.j7_count.exchange(param.increase_count))
                {
                    option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
                }
                else
                {
                    std::fill(motorOptions().begin(), motorOptions().end(), NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_ENABLE);
                    motorOptions()[param.motion_id] = NOT_CHECK_POS_FOLLOWING_ERROR | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
                }
            }
            else if (p.first == "stop")
            {
                param.j7_count.exchange(0);
                option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
            }
        }

        this->param() = param;
    }
    auto JogJ7::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JogJParam&>(this->param());

        // get current pos //
        if (count() == 1)
        {
            param.target_pos = controller()->motionAtAbs(param.motion_id).actualPos();
            param.p_now = controller()->motionAtAbs(param.motion_id).actualPos();
            param.v_now = controller()->motionAtAbs(param.motion_id).actualVel();
            param.a_now = 0.0;
        }

        // init status and calculate target pos and max vel //
        param.max_vel = param.vel*1.0*g_vel_percent.load() / 100.0;
        param.target_pos += aris::dynamic::s_sgn(param.increase_status)*param.max_vel * 1e-3;

        // 梯形轨迹规划 //
        static double p_next, v_next, a_next;

        aris::Size t;
        auto finished = aris::plan::moveAbsolute2(param.p_now, param.v_now, param.a_now
            , param.target_pos, 0.0, 0.0
            , param.max_vel, param.acc, param.dec
            , 1e-3, 1e-10, p_next, v_next, a_next, t);

        //当计数器j7_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.j7_count == 0)
        {
            param.increase_status = 0;
        }
        else
        {
            --param.j7_count;
        }

        controller()->motionAtAbs(param.motion_id).setTargetPos(p_next);
        param.p_now = p_next;
        param.v_now = v_next;
        param.a_now = a_next;

        return finished;
    }
    auto JogJ7::collectNrt()->void
    {
        JogJParam::j7_count = 0;
        if (retCode() < 0)
        {
            JogJParam::j7_count.store(0);
        }
    }
    JogJ7::~JogJ7() = default;
    JogJ7::JogJ7(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"j7\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<GroupParam>"
            "				<Param name=\"increase_count\" default=\"500\"/>"
            "				<Param name=\"vel\" default=\"1\" abbreviation=\"v\"/>"
            "				<Param name=\"acc\" default=\"5\" abbreviation=\"a\"/>"
            "				<Param name=\"dec\" default=\"5\" abbreviation=\"d\"/>"
            "				<Param name=\"vel_percent\" default=\"10\"/>"
            "				<Param name=\"direction\" default=\"1\"/>"
            "			</GroupParam>"
            "			<Param name=\"stop\"/>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


#define JOGC_PARAM_STRING \
        "	<UniqueParam>"\
        "		<GroupParam>"\
        "			<Param name=\"increase_count\" default=\"200\"/>"\
        "			<Param name=\"vel\" default=\"{0.05,0.05,0.05,0.15,0.15,0.15}\"/>"\
        "			<Param name=\"acc\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"\
        "			<Param name=\"dec\" default=\"{0.2,0.2,0.2,1,1,1}\"/>"\
        "			<Param name=\"cor\" default=\"0\"/>"\
        "			<Param name=\"tool\" default=\"tool0\"/>"\
        "			<Param name=\"wobj\" default=\"wobj0\"/>"\
        "			<Param name=\"vel_percent\" default=\"20\"/>"\
        "			<Param name=\"direction\" default=\"1\"/>"\
        "		</GroupParam>"\
        "		<Param name=\"stop\"/>"\
        "	</UniqueParam>"
    // 示教运动--jogx //
    struct JCParam
    {
        std::vector<double> pm_target;
        double vel[6], acc[6], dec[6];
        int increase_count;
        int cor_system;
        int vel_percent;
        int moving_type;
        int increase_status[6]{0,0,0,0,0,0};
        static std::atomic_int32_t jx_count, jy_count, jz_count, jrx_count, jry_count, jrz_count;
        aris::dynamic::Marker *tool, *wobj;
    };
    template<typename JogType>
    auto set_jogc_input_param(JogType* this_p, const std::map<std::string_view, std::string_view> &cmd_params, JCParam &param, std::atomic_int32_t& j_count)->void
    {
        auto&cs = aris::server::ControlServer::instance();

        for (auto &p : cmd_params)
        {
            if (p.first == "increase_count")
            {
                std::cout << this_p->model()->generalMotionPool()[0].makI().name() << std::endl;
                std::cout << this_p->model()->generalMotionPool()[0].makI().fatherPart().name() << std::endl;
                auto found = this_p->model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmd_params.at("tool")));

                auto found2 = this_p->model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string("tool0"));
                param.tool = &*this_p->model()->generalMotionPool()[0].makI().fatherPart().markerPool().findByName(std::string(cmd_params.at("tool")));
                param.wobj = &*this_p->model()->generalMotionPool()[0].makJ().fatherPart().markerPool().findByName(std::string(cmd_params.at("wobj")));

                param.increase_count = this_p->int32Param("increase_count");
                if (param.increase_count < 0 || param.increase_count>1e5)THROW_FILE_LINE("");

                param.cor_system = this_p->int32Param("cor");
                auto velocity = this_p->int32Param("vel_percent");
                velocity = std::max(std::min(100, velocity), 0);
                param.vel_percent = velocity;

                auto mat = this_p->matrixParam("vel");
                if (mat.size() != 6)THROW_FILE_LINE("");
                std::copy(mat.begin(), mat.end(), param.vel);
                std::fill(param.vel, param.vel + 3, g_vel.v_tcp);
                std::fill(param.vel + 3, param.vel + 6, g_vel.w_tcp*0.1);

                mat = this_p->matrixParam("acc");
                if (mat.size() != 6)THROW_FILE_LINE("");
                std::copy(mat.begin(), mat.end(), param.acc);

                mat = this_p->matrixParam("dec");
                if (mat.size() != 6)THROW_FILE_LINE("");
                std::copy(mat.begin(), mat.end(), param.dec);

                param.increase_status[param.moving_type] = std::max(std::min(1, this_p->int32Param("direction")), -1);

                std::shared_ptr<aris::plan::Plan> planptr = cs.currentExecutePlan();

                //当前有指令在执行//
                if (planptr && planptr->cmdName() != this_p->cmdName())throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + "Other command is running");

                if (j_count.exchange(param.increase_count))
                {
                    this_p->option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
                }
                else
                {
                    std::fill(this_p->motorOptions().begin(), this_p->motorOptions().end(), aris::plan::Plan::MotorOption::NOT_CHECK_POS_FOLLOWING_ERROR | aris::plan::Plan::MotorOption::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | aris::plan::Plan::MotorOption::USE_TARGET_POS);
                }
            }
            else if (p.first == "stop")
            {
                j_count.exchange(0);
                this_p->option() |= aris::plan::Plan::Option::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::Option::NOT_RUN_COLLECT_FUNCTION;
            }
        }
    }
    std::atomic_int32_t JCParam::jx_count = 0;
    auto JX::prepareNrt()->void
    {
        JCParam param;

        param.pm_target.resize(16, 0.0);
        param.moving_type = 0;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        set_jogc_input_param(this, cmdParams(), param, param.jx_count);

        this->param() = param;

    }
    auto JX::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        double max_vel[6]{ 0,0,0,0,0,0 };

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load() / 100.0;
            target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
        }
        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        int finished[6]{0,0,0,0,0,0};
        for (int i = 0; i < 6; i++)
        {
            aris::Size t;
            finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], param.acc[i], param.dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //当计数器jx_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.jx_count == 0)
        {
            param.increase_status[param.moving_type] = 0;
        }
        else
        {
            --param.jx_count;
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        //model()->generalMotionPool()[0].getMpm(pm_now);
        param.tool->getPm(*param.wobj, pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);

        //绝对坐标系
        if (param.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, param.pm_target.data());
        }
        //工具坐标系
        else if (param.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, param.pm_target.data());
        }

        //model()->generalMotionPool().at(0).setMpm(param.pm_target.data());
        param.tool->setPm(*param.wobj, param.pm_target.data());
        model()->generalMotionPool().at(0).updMpm();

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;


        return finished[param.moving_type];
    }
    auto JX::collectNrt()->void
    {
        JCParam::jx_count = 0;
        if (retCode() < 0)
        {
            JCParam::jx_count.store(0);
        }
    }
    JX::~JX() = default;
    JX::JX(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jx\">"
                JOGC_PARAM_STRING
            "</Command>");
    }


    // 示教运动--jogy //
    std::atomic_int32_t JCParam::jy_count = 0;
    auto JY::prepareNrt()->void
    {
        JCParam param;

        param.pm_target.resize(16, 0.0);
        param.moving_type = 1;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        set_jogc_input_param(this, cmdParams(), param, param.jy_count);

        this->param() = param;

    }
    auto JY::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        double max_vel[6]{ 0,0,0,0,0,0 };

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load() / 100.0;
            target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
        }
        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        int finished[6]{ 0,0,0,0,0,0 };
        for (int i = 0; i < 6; i++)
        {
            aris::Size t;
            finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], param.acc[i], param.dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //当计数器jy_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.jy_count == 0)
        {
            param.increase_status[param.moving_type] = 0;
        }
        else
        {
            --param.jy_count;
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        param.tool->getPm(*param.wobj, pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);

        //绝对坐标系
        if (param.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, param.pm_target.data());
        }
        //工具坐标系
        else if (param.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, param.pm_target.data());
        }

        param.tool->setPm(*param.wobj, param.pm_target.data());
        model()->generalMotionPool().at(0).updMpm();

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;

        return finished[param.moving_type];
    }
    auto JY::collectNrt()->void
    {
        JCParam::jy_count = 0;
        if (retCode() < 0)
        {
            JCParam::jy_count.store(0);
        }
    }
    JY::~JY() = default;
    JY::JY(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jy\">"
                JOGC_PARAM_STRING
            "</Command>");
    }


    // 示教运动--jogz //
    std::atomic_int32_t JCParam::jz_count = 0;
    auto JZ::prepareNrt()->void
    {
        JCParam param;

        param.pm_target.resize(16, 0.0);
        param.moving_type = 2;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        set_jogc_input_param(this, cmdParams(),  param, param.jz_count);

        this->param() = param;

    }
    auto JZ::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        double max_vel[6]{ 0,0,0,0,0,0 };

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load() / 100.0;
            target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
        }
        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        int finished[6]{ 0,0,0,0,0,0 };
        for (int i = 0; i < 6; i++)
        {
            aris::Size t;
            finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], param.acc[i], param.dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //当计数器jz_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.jz_count == 0)
        {
            param.increase_status[param.moving_type] = 0;
        }
        else
        {
            --param.jz_count;
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        param.tool->getPm(*param.wobj, pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);

        //绝对坐标系
        if (param.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, param.pm_target.data());
        }
        //工具坐标系
        else if (param.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, param.pm_target.data());
        }

        param.tool->setPm(*param.wobj, param.pm_target.data());
        model()->generalMotionPool().at(0).updMpm();

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;


        return finished[param.moving_type];
    }
    auto JZ::collectNrt()->void
    {
        JCParam::jz_count = 0;
        if (retCode() < 0)
        {
            JCParam::jz_count.store(0);
        }
    }
    JZ::~JZ() = default;
    JZ::JZ(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jz\">"
                JOGC_PARAM_STRING
            "</Command>");
    }


    // 示教运动--jogrx //
    std::atomic_int32_t JCParam::jrx_count = 0;
    auto JRX::prepareNrt()->void
    {
        JCParam param;

        param.pm_target.resize(16, 0.0);
        param.moving_type = 3;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        set_jogc_input_param(this, cmdParams(),  param, param.jrx_count);

        this->param() = param;

    }
    auto JRX::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        double max_vel[6]{ 0,0,0,0,0,0 };

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load() / 100.0;
            target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
        }
        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        int finished[6]{ 0,0,0,0,0,0 };
        for (int i = 0; i < 6; i++)
        {
            aris::Size t;
            finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], param.acc[i], param.dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //当计数器jrx_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.jrx_count == 0)
        {
            param.increase_status[param.moving_type] = 0;
        }
        else
        {
            --param.jrx_count;
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        param.tool->getPm(*param.wobj, pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);

        //绝对坐标系
        if (param.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, param.pm_target.data());
        }
        //工具坐标系
        else if (param.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, param.pm_target.data());
        }

        param.tool->setPm(*param.wobj, param.pm_target.data());
        model()->generalMotionPool().at(0).updMpm();

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;

        return finished[param.moving_type];
    }
    auto JRX::collectNrt()->void
    {
        JCParam::jrx_count = 0;
        if (retCode() < 0)
        {
            JCParam::jrx_count.store(0);
        }
    }
    JRX::~JRX() = default;
    JRX::JRX(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jrx\">"
                JOGC_PARAM_STRING
            "</Command>");
    }


    // 示教运动--jogry //
    std::atomic_int32_t JCParam::jry_count = 0;
    auto JRY::prepareNrt()->void
    {
        JCParam param;

        param.pm_target.resize(16, 0.0);
        param.moving_type = 4;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        set_jogc_input_param(this, cmdParams(),  param, param.jry_count);

        this->param() = param;

    }
    auto JRY::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        double max_vel[6]{ 0,0,0,0,0,0 };

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load() / 100.0;
            target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
        }
        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        int finished[6]{ 0,0,0,0,0,0 };
        for (int i = 0; i < 6; i++)
        {
            aris::Size t;
            finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], param.acc[i], param.dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //当计数器jry_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.jry_count == 0)
        {
            param.increase_status[param.moving_type] = 0;
        }
        else
        {
            --param.jry_count;
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        param.tool->getPm(*param.wobj, pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);

        //绝对坐标系
        if (param.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, param.pm_target.data());
        }
        //工具坐标系
        else if (param.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, param.pm_target.data());
        }

        param.tool->setPm(*param.wobj, param.pm_target.data());
        model()->generalMotionPool().at(0).updMpm();

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;

        return finished[param.moving_type];
    }
    auto JRY::collectNrt()->void
    {
        JCParam::jry_count = 0;
        if (retCode() < 0)
        {
            JCParam::jry_count.store(0);
        }
    }
    JRY::~JRY() = default;
    JRY::JRY(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jry\">"
                JOGC_PARAM_STRING
            "</Command>");
    }


    // 示教运动--jogrz //
    std::atomic_int32_t JCParam::jrz_count = 0;
    auto JRZ::prepareNrt()->void
    {
        JCParam param;

        param.pm_target.resize(16, 0.0);
        param.moving_type = 5;//0,1,2,3,4,5分别表示沿x,y,z,Rx,Ry,Rz动作//

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;

        set_jogc_input_param(this, cmdParams(),  param, param.jrz_count);

        this->param() = param;

    }
    auto JRZ::executeRT()->int
    {
        //获取驱动//
        auto &param = std::any_cast<JCParam&>(this->param());
        char eu_type[4]{ '1', '2', '3', '\0' };

        // 前三维为xyz，后三维是w的积分，注意没有物理含义
        static double target_p[6];

        // get current pe //
        static double p_now[6], v_now[6], a_now[6];
        if (count() == 1)
        {
            model()->generalMotionPool()[0].getMpe(target_p);
            std::fill_n(target_p + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMpe(p_now, eu_type);
            std::fill_n(p_now + 3, 3, 0.0);

            model()->generalMotionPool()[0].getMve(v_now, eu_type);
            model()->generalMotionPool()[0].getMae(a_now, eu_type);
        }

        // init status //
        double max_vel[6]{ 0,0,0,0,0,0 };

        // calculate target pos and max vel //
        for (int i = 0; i < 6; i++)
        {
            max_vel[i] = param.vel[i] * 1.0 * g_vel_percent.load() / 100.0;
            target_p[i] += aris::dynamic::s_sgn(param.increase_status[i]) * max_vel[i] * 1e-3;
        }
        // 梯形轨迹规划 calculate real value //
        double p_next[6]{ 0,0,0,0,0,0 }, v_next[6]{ 0,0,0,0,0,0 }, a_next[6]{ 0,0,0,0,0,0 };
        int finished[6]{ 0,0,0,0,0,0 };
        for (int i = 0; i < 6; i++)
        {
            aris::Size t;
            finished[i] = aris::plan::moveAbsolute2(p_now[i], v_now[i], a_now[i]
                , target_p[i], 0.0, 0.0
                , max_vel[i], param.acc[i], param.dec[i]
                , 1e-3, 1e-10, p_next[i], v_next[i], a_next[i], t);
        }

        //当计数器jrz_count等于0时，increase_status=0，即目标位置不再变更//
        if (param.jrz_count == 0)
        {
            param.increase_status[param.moving_type] = 0;
        }
        else
        {
            --param.jrz_count;
        }

        //将欧拉角转换成四元数，求绕任意旋转轴转动的旋转矩阵//
        double w[3], pm[16];
        aris::dynamic::s_vc(3, v_next + 3, w);
        auto normv = aris::dynamic::s_norm(3, w);
        if (std::abs(normv) > 1e-10)aris::dynamic::s_nv(3, 1 / normv, w); //数乘
        auto theta = normv * 1e-3;
        double pq[7]{ p_next[0] - p_now[0], p_next[1] - p_now[1], p_next[2] - p_now[2], w[0] * sin(theta / 2.0), w[1] * sin(theta / 2.0), w[2] * sin(theta / 2.0), cos(theta / 2.0) };
        s_pq2pm(pq, pm);

        // 获取当前位姿矩阵 //
        double pm_now[16];
        param.tool->getPm(*param.wobj, pm_now);

        // 保存下个周期的copy //
        s_vc(6, p_next, p_now);
        s_vc(6, v_next, v_now);
        s_vc(6, a_next, a_now);

        //绝对坐标系
        if (param.cor_system == 0)
        {
            s_pm_dot_pm(pm, pm_now, param.pm_target.data());
        }
        //工具坐标系
        else if (param.cor_system == 1)
        {
            s_pm_dot_pm(pm_now, pm, param.pm_target.data());
        }

        param.tool->setPm(*param.wobj, param.pm_target.data());
        model()->generalMotionPool().at(0).updMpm();

        // 运动学反解 //
        if (model()->solverPool().at(0).kinPos())return -1;

        return finished[param.moving_type];
    }
    auto JRZ::collectNrt()->void
    {
        JCParam::jrz_count = 0;
        if (retCode() < 0)
        {
            JCParam::jrz_count.store(0);
        }
    }
    JRZ::~JRZ() = default;
    JRZ::JRZ(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"jrz\">"
                JOGC_PARAM_STRING
            "</Command>");
    }


    // 清理slavepool //
    auto ClearCon::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        controller()->slavePool().clear();

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    ClearCon::ClearCon(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"clearCon\">"
            "</Command>");
    }


    // 配置控制器参数 //
    struct SetConParam
    {
        uint16_t motion_phyid;
        uint16_t ecslave_phyid;
        uint16_t ecslave_dc;
        bool is_motion;
    };
    auto SetCon::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        SetConParam param;
        param.motion_phyid = 0;
        param.ecslave_phyid = 0;
        param.is_motion = 1;
        for (auto &p : cmdParams())
        {
            //motion physical id number//
            if (p.first == "motion_phyid")
            {
                param.motion_phyid = int32Param(p.first);
                param.is_motion = 1;
            }
            //ethercat slave physical id number//
            else if (p.first == "ecslave_phyid")
            {
                param.ecslave_phyid = int32Param(p.first);
                param.is_motion = 0;
                param.ecslave_dc = int32Param("ecslave_dc");
            }
        }

        //std::unique_ptr<aris::control::Controller> controller(aris::robot::createControllerRokaeXB4());
        //controller->slavePool().clear();	//清除slavePool中的元素，后面重新添加

        //configure motion//
        if(param.is_motion)
        {
            controller()->slavePool().add<aris::control::EthercatMotor>();
            controller()->slavePool().back().setPhyId(param.motion_phyid);
            dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool().back()).scanInfoForCurrentSlave();
            dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool().back()).scanPdoForCurrentSlave();
            dynamic_cast<aris::control::EthercatMotor&>(controller()->slavePool().back()).setDcAssignActivate(0x300);
        }
        //configure ect slave//
        else if(!param.is_motion)
        {
            controller()->slavePool().add<aris::control::EthercatSlave>();
            controller()->slavePool().back().setPhyId(param.ecslave_phyid);
            dynamic_cast<aris::control::EthercatSlave&>(controller()->slavePool().back()).scanInfoForCurrentSlave();
            dynamic_cast<aris::control::EthercatSlave&>(controller()->slavePool().back()).scanPdoForCurrentSlave();
            //倍福ECT Slave 耦合器,ecslave_dc=0x300；耦合器后面的模块，,ecslave_dc=0x00//
            dynamic_cast<aris::control::EthercatSlave&>(controller()->slavePool().back()).setDcAssignActivate(param.ecslave_dc);
        }
        //cs.resetController(controller);

        //std::cout << controller->xmlString() << std::endl;
        /*
        auto xmlpath = std::filesystem::absolute(".");
        const std::string xmlfile = "kaanh.xml";
        xmlpath = xmlpath / xmlfile;
        cs.saveXmlFile(xmlpath.string().c_str());
        */

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    SetCon::SetCon(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setCon\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<Param name=\"motion_phyid\" default=\"0\"/>"
            "			<GroupParam>"
            "				<Param name=\"ecslave_phyid\" default=\"6\"/>"
            "				<Param name=\"ecslave_dc\" default=\"768\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


    // 配置DH参数 //
    struct SetDHParam
    {
        std::vector<double>dh;
        double tool_offset;
        int axis_num;
    };
    auto SetDH::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        SetDHParam dhparam;
        dhparam.dh.clear();

        for (auto &p : cmdParams())
        {
            if (p.first == "six_axes")//6轴DH参数//
            {
                dhparam.dh.resize(6, 0.0);
                dhparam.dh[0] = doubleParam("d1_six_axes");
                dhparam.dh[1] = doubleParam("a1_six_axes");
                dhparam.dh[2] = doubleParam("a2_six_axes");
                dhparam.dh[3] = doubleParam("d3_six_axes");
                dhparam.dh[4] = doubleParam("a3_six_axes");
                dhparam.dh[5] = doubleParam("d4_six_axes");
                dhparam.tool_offset = doubleParam("tool0_six_axes");
                dhparam.axis_num = 6;
            }
            else if (p.first == "seven_axes")//7轴DH参数//
            {
                dhparam.dh.resize(3, 0.0);
                dhparam.dh[0] = doubleParam("d1_seven_axes");
                dhparam.dh[1] = doubleParam("d3_seven_axes");
                dhparam.dh[2] = doubleParam("d5_seven_axes");
                dhparam.tool_offset = doubleParam("tool0_seven_axes");
                dhparam.axis_num = 7;
            }
        }

        aris::dynamic::PumaParam param_puma;
        aris::dynamic::SevenAxisParam param_7axes;
        if (dhparam.axis_num == 6)
        {
            param_puma.d1 = dhparam.dh[0];
            param_puma.a1 = dhparam.dh[1];
            param_puma.a2 = dhparam.dh[2];
            param_puma.d3 = dhparam.dh[3];
            param_puma.a3 = dhparam.dh[4];
            param_puma.d4 = dhparam.dh[5];
            param_puma.tool0_pe[2] = dhparam.tool_offset;

            auto m = aris::dynamic::createModelPuma(param_puma);
            m->init();
            this->controlServer()->resetModel(m.release());

        }
        else if (dhparam.axis_num == 7)
        {
            param_7axes.d1 = dhparam.dh[0];
            param_7axes.d3 = dhparam.dh[1];
            param_7axes.d5 = dhparam.dh[2];
            param_7axes.tool0_pe[2] = dhparam.tool_offset;

            auto m = aris::dynamic::createModelSevenAxis(param_7axes);
            m->init();
            this->controlServer()->resetModel(m.release());
        }
        else{ }

        //auto modelxmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
        //const std::string mxmlfile = "model_rokae.xml";
        //modelxmlpath = modelxmlpath / mxmlfile;
        //cs.model().loadXmlFile(modelxmlpath.string().c_str());

        auto &cal = this->controlServer()->model().calculator();
        wulingconfig::createUserDataType(cal);

        /*
        //动力学标定参数//
        param.iv_vec =
        {
            { 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.00000000000000,   0.59026333537827,   0.00000000000000,   0.00000000000000,   0.00000000000000 },
            { 0.00000000000000, -0.02551872200978,   0.00000000000000,   3.05660683326413,   2.85905166943306,   0.00000000000000,   0.00000000000000, -0.00855352993039, -0.09946674483372, -0.00712210734359 },
            { 0.00000000000000,   0.00000000000000,   0.00000000000000,   0.02733022277747,   0.00000000000000,   0.37382629693302,   0.00000000000000,   0.00312006493276, -0.00578410451516,   0.00570606128540 },
            { 0.00000000000000,   1.06223330086669,   0.00000000000000,   0.00311748242960,   0.00000000000000,   0.24420385558544,   0.24970286555981,   0.00305759215246, -0.66644096559686,   0.00228253380852 },
            { 0.00000000000000,   0.05362286897910,   0.00528925153464, -0.00842588023014,   0.00128498153337, -0.00389810210572,   0.00000000000000, -0.00223677867576, -0.03365036368035, -0.00415647085627 },
            { 0.00000000000000,   0.00000000000000,   0.00066049870832,   0.00012563800445, -0.00085124094833,   0.04209529937135,   0.04102481443654, -0.00067596644891,   0.00017482449876, -0.00041025776053 },
        };
        param.mot_frc_vec =
        {
            { 9.34994758321915, 7.80825641041495, 0.00000000000000 },
            { 11.64080253106441, 13.26518528472506, 3.55567932576820 },
            { 4.77014054273075, 7.85644357492508, 0.34445460269183 },
            { 3.63141668516122, 3.35461524886318, 0.14824771620542 },
            { 2.58310846982020, 1.41963212641879, 0.04855267273770 },
            { 1.78373986219597, 0.31920640440152, 0.03381545544099 },
        };
        */

        /*
        auto xmlpath = std::filesystem::absolute(".");
        const std::string xmlfile = "kaanh.xml";
        xmlpath = xmlpath / xmlfile;
        cs.saveXmlFile(xmlpath.string().c_str());
        */

        having_model.store(true);
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

    }
    SetDH::SetDH(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setdh\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<GroupParam name=\"start_group\">"
            "				<Param name=\"six_axes\"/>"
            "				<Param name=\"d1_six_axes\" default=\"0.3295\"/>"
            "				<Param name=\"a1_six_axes\" default=\"0.04\"/>"
            "				<Param name=\"a2_six_axes\" default=\"0.275\"/>"
            "				<Param name=\"d3_six_axes\" default=\"0.0\"/>"
            "				<Param name=\"a3_six_axes\" default=\"0.025\"/>"
            "				<Param name=\"d4_six_axes\" default=\"0.28\"/>"
            "				<Param name=\"tool0_six_axes\" default=\"0.078\"/>"
            "			</GroupParam>"
            "			<GroupParam>"
            "				<Param name=\"seven_axes\"/>"
            "				<Param name=\"d1_seven_axes\" default=\"0.3705\"/>"
            "				<Param name=\"d3_seven_axes\" default=\"0.330\"/>"
            "				<Param name=\"d5_seven_axes\" default=\"0.320\"/>"
            "				<Param name=\"tool0_seven_axes\" default=\"0.2205\"/>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


    // 配置PG参数 //
    struct SetPGParam
    {
        std::string name;
        std::vector<double> pe;
        uint16_t part_id;
        std::string file_path;
    };
    auto SetPG::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        SetPGParam param;
        param.pe.clear();
        param.pe.resize(6, 0.0);
        for (auto &p : cmdParams())
        {
            if (p.first == "name")
            {
                param.name = p.second;
            }
            else if (p.first == "pe")
            {
                auto mat = matrixParam(cmdParams().at("pe"));
                if (mat.size() == param.pe.size())
                {
                    param.pe.assign(mat.begin(), mat.end());
                }
                else
                {
                    throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
                }
            }
            else if (p.first == "part_id")
            {
                param.part_id = int32Param(p.first);
            }
            else if (p.first == "file_path")
            {
                param.file_path = p.second;
            }
        }

        model()->partPool().at(param.part_id).geometryPool().clear();
        model()->partPool().at(param.part_id).geometryPool().add<FileGeometry>(param.name, param.file_path, param.pe.data());

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

    }
    SetPG::SetPG(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setpgpath\">"
            "	<GroupParam>"
            "		<Param name=\"name\" default=\"test\"/>"
            "		<Param name=\"pe\" default=\"{0, 0, 0, -0, 0, -0}\"/>"
            "		<Param name=\"part_id\" default=\"6\"/>"
            "		<Param name=\"file_path\" default=\"/RobotGallery/Rokae/XB4/l0.data\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // 配置PG参数 //
    struct SetPPathParam
    {
        std::string name;
        std::vector<double> pe;
        std::vector<std::string> file_path;
    };
    auto SetPPath::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        //if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        SetPPathParam param;
        param.pe.clear();
        param.pe.resize(6, 0.0);
        param.file_path.clear();

        for (auto &p : cmdParams())
        {
            if (p.first == "name")
            {
                param.name = p.second;
            }
            else if (p.first == "pe")
            {
                auto mat = this->matrixParam(p.first);
                if (mat.size() == param.pe.size())
                {
                    param.pe.assign(mat.begin(), mat.end());
                }
                else
                {
                    throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
                }
            }
            else if (p.first == "file_path")
            {
                auto data = std::string(p.second);
                char *s_input = (char *)data.c_str();
                const char *split = ";";
                // 以‘;’为分隔符拆分字符串
                char *sp_input = strtok(s_input, split);
                std::string s_data;
                while (sp_input != NULL)
                {
                    s_data = sp_input;
                    param.file_path.push_back(s_data);
                    sp_input = strtok(NULL, split);
                }
            }
        }

        double pm[16];
        aris::dynamic::s_pq2pm(param.pe.data(), pm);
        for (int i = 0; i < param.file_path.size(); i++)
        {
            model()->partPool().at(i).geometryPool().clear();
            model()->partPool().at(i).geometryPool().add<FileGeometry>(param.name, param.file_path[i], pm);
        }
        model()->init();
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

    }
    SetPPath::SetPPath(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setppath\">"
            "	<GroupParam>"
            "		<Param name=\"name\" default=\"test\"/>"
            "		<Param name=\"pe\" default=\"{0.0, 0.0, 0.0, -0.0, 0.0, -0.0}\"/>"
            "		<Param name=\"file_path\" default=\"/RobotGallery/Rokae/XB4/l0.data;/RobotGallery/Rokae/XB4/l1.data;/RobotGallery/Rokae/XB4/l2.data;/RobotGallery/Rokae/XB4/l3.data;/RobotGallery/Rokae/XB4/l4.data;/RobotGallery/Rokae/XB4/l5.data;/RobotGallery/Rokae/XB4/l6.data\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // 配置UI //
    struct SetUIParam
    {
        std::string ui_path;
    };
    auto SetUI::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        SetUIParam param;

        for (auto &p : cmdParams())
        {
            if (p.first == "ui_path")
            {
                param.ui_path = p.second;
            }
        }

        auto xmlpath = std::filesystem::absolute(".");
        const std::string xmlfile = "kaanh.xml";
        auto xmlpath_ui = xmlpath / param.ui_path;
        xmlpath = xmlpath / xmlfile;

        cs.interfaceRoot().loadXmlFile(xmlpath_ui.string().c_str());
        //cs.saveXmlFile(xmlpath.string().c_str());

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;

    }
    SetUI::SetUI(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setUI\">"
            "	<GroupParam>"
            "		<Param name=\"ui_path\" default=\"interface_kaanh.xml\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // 配置关节——home偏置、角度-编码系数、角度、角速度、角加速度上下限 //
    struct SetDriverParam
    {
        std::vector<bool> joint_active_vec;
        double pos_factor;
        double pos_max;
        double pos_min;
        double vel_max;
        double vel_min;
        double acc_max;
        double acc_min;
        double pos_offset;
    };
    auto SetDriver::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");
        //std::unique_ptr<aris::control::Controller> controller(kaanh::createControllerRokaeXB4());
        SetDriverParam param;

        //initial//
        {
            param.joint_active_vec.clear();
            param.pos_factor = 0.0;
            param.pos_max = 0.0;
            param.pos_min = 0.0;
            param.vel_max = 0.0;
            param.vel_min = 0.0;
            param.acc_max = 0.0;
            param.acc_min = 0.0;
            param.pos_offset = 0.0;
        }

        for (auto &p : cmdParams())
        {
            if (p.first == "motion_id")
            {
                param.joint_active_vec.resize(50, false);
                param.joint_active_vec.at(int32Param(p.first)) = true;
            }
            else if(p.first == "pos_factor")
            {
                auto m = matrixParam(p.second);
                param.pos_factor = m.toDouble() / (2.0 * PI);
            }
            else if (p.first == "pos_max")
            {
                auto m = matrixParam(p.second);
                param.pos_max = m.toDouble() * 2 * PI / 360;
            }
            else if (p.first == "pos_min")
            {
                auto m = matrixParam(p.second);
                param.pos_min = m.toDouble() * 2 * PI / 360;
            }
            else if (p.first == "vel_max")
            {
                auto m = matrixParam(p.second);
                param.vel_max = m.toDouble() * 2 * PI / 360;
            }
            else if (p.first == "vel_min")
            {
                auto m = matrixParam(p.second);
                param.vel_min = m.toDouble() * 2 * PI / 360;
            }
            else if (p.first == "acc_max")
            {
                auto m = matrixParam(p.second);
                param.acc_max = m.toDouble() * 2 * PI / 360;
            }
            else if (p.first == "acc_min")
            {
                auto m = matrixParam(p.second);
                param.acc_min = m.toDouble() * 2 * PI / 360;
            }
            else if (p.first == "pos_offset")
            {
                auto m = matrixParam(p.second);
                param.pos_offset = m.toDouble();
            }
        }

        // 设置驱动pos_factor,pos,vel,acc //
        for (int i = 0; i < param.joint_active_vec.size(); i++)
        {
            if (param.joint_active_vec[i])
            {
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setPosFactor(param.pos_factor);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setMaxPos(param.pos_max);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setMinPos(param.pos_min);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setMaxVel(param.vel_max);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setMinVel(param.vel_min);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setMaxAcc(param.acc_max);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setMinAcc(param.acc_min);
                dynamic_cast<aris::control::Motor&>(controller()->slavePool()[i]).setPosOffset(param.pos_offset);
            }
        }

        std::cout << param.pos_factor << std::endl;
        //cs.resetController(controller);
        /*
        auto xmlpath = std::filesystem::absolute(".");
        const std::string xmlfile = "kaanh.xml";
        xmlpath = xmlpath / xmlfile;
        cs.saveXmlFile(xmlpath.string().c_str());
        */
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    SetDriver::SetDriver(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setDriver\">"
            "	<GroupParam>"
            "		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
            "		<Param name=\"pos_factor\" default=\"0.0\"/>"
            "		<Param name=\"pos_max\" default=\"0.0\"/>"
            "		<Param name=\"pos_min\" default=\"0.0\"/>"
            "		<Param name=\"vel_max\" default=\"0.0\"/>"
            "		<Param name=\"vel_min\" default=\"0.0\"/>"
            "		<Param name=\"acc_max\" default=\"0.0\"/>"
            "		<Param name=\"acc_min\" default=\"0.0\"/>"
            "		<Param name=\"pos_offset\" default=\"0.0\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // 从硬件扫描从站 //
    auto ScanSlave::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");
        aris::control::EthercatController ec_controller;

#ifdef UNIX
        ec_controller.scan();
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret_value.push_back(std::make_pair<std::string, std::any>("controller_xml", ec_controller.xmlString()));
#endif
#ifdef WIN32
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret_value.push_back(std::make_pair<std::string, std::any>("controller_xml", cs.controller().xmlString()));
#endif // WIN32

        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    ScanSlave::ScanSlave(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"scanslave\">"
            "</Command>");
    }


    // 根据ESI补全从站信息 //
    auto GetEsiPdoList::prepareNrt()->void
    {
        if (this->controlServer()->running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        std::vector<std::pair<std::string, std::any>> ret_value;
        try
        {
            auto pdolist = this->ecController()->getPdoList(int32Param("vendor_id"), int32Param("product_code"), int32Param("revision_num"));
            ret_value.push_back(std::make_pair<std::string, std::any>("is_esi_found", true));
            ret_value.push_back(std::make_pair<std::string, std::any>("pdo_list_xml", pdolist));
        }
        catch (std::exception &e)
        {
            ret_value.push_back(std::make_pair<std::string, std::any>("is_esi_found", false));
        }
        ret() = ret_value;
        option() = NOT_RUN_EXECUTE_FUNCTION|NOT_RUN_COLLECT_FUNCTION;
    }
    GetEsiPdoList::GetEsiPdoList(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"getesipdolist\">"
            "	<GroupParam>"
            "		<Param name=\"vendor_id\" default=\"0\"/>"
            "		<Param name=\"product_code\" default=\"0\"/>"
            "		<Param name=\"revision_num\" default=\"0\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // 选择ESI路径 //
    struct SetEsiPathParam
    {
        std::vector<std::filesystem::path> path;
    };
    auto SetEsiPath::prepareNrt()->void
    {
        if (this->controlServer()->running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        std::cout << this->cmdParams().at("path") << std::endl;

        this->ecController()->setEsiDirs({this->cmdParams().at("path")});
        this->ecController()->updateDeviceList();
        auto device_list = this->ecController()->getDeviceList();

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret_value.push_back(std::make_pair<std::string, std::any>("device_list_xml", device_list));

        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    SetEsiPath::SetEsiPath(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setesipath\">"
            "	<GroupParam>"
            "		<Param name=\"path\" default=\"\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    // update controller //
    auto Update::prepareNrt()->void
    {
        system("bash /home/kaanh/Document/kaanh/update_arislib.sh");
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    Update::Update(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"update_controller\">"
            "</Command>");
    }


    // 保存配置 //
    struct SaveXmlParam
    {
        std::string path;
    };
    auto SaveXml::prepareNrt()->void
    {
        auto&cs = aris::server::ControlServer::instance();
        if (cs.running())throw std::runtime_error("cs is running, please stop the cs using cs_stop!");

        SaveXmlParam param;
        for (auto &p : cmdParams())
        {
            if (p.first == "path")
            {
                param.path = p.second;
            }
        }
        if (param.path == "")
        {
            param.path = std::filesystem::absolute(".").string();
        }
        const std::string xmlfile = "kaanh.xml";
        param.path = param.path + '/' + xmlfile;

        std::cout << "input path:" << param.path << std::endl;
        cs.saveXmlFile(param.path.c_str());

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    SaveXml::SaveXml(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"savexml\">"
            "	<GroupParam>"
            "		<Param name=\"path\" default=\"\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    auto GetXml::prepareNrt()->void
    {
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret_value.push_back(std::make_pair(std::string("configure_xml"), controlServer()->xmlString()));
        ret() = ret_value;
        option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
    }
    GetXml::~GetXml() = default;
    GetXml::GetXml(const std::string &name) : Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"get_xml\">"
            "</Command>");
    }


    auto SetXml::prepareNrt()->void
    {
        // remove all symbols "{" "}"
        if (this->controlServer()->running())THROW_FILE_LINE("server is running, can't set xml");
        auto xml_str = std::string(cmdParams().at("xml").substr(1, cmdParams().at("xml").size() - 2));
        // 这一句要小心，此时 this 已被销毁，后面不能再调用this了 //

        //controlServer()->close();
        //controlServer()->loadXmlStr(xml_str);
        // server load 会导致interface失败 ////////////////////////////////////////
        aris::core::XmlDocument doc;
        if (doc.Parse(xml_str.c_str()) != tinyxml2::XML_SUCCESS) THROW_FILE_LINE("XML failed");

        if (doc.RootElement()->FirstChildElement("EthercatController"))
        {
            controlServer()->controller().loadXml(*doc.RootElement()->FirstChildElement("EthercatController"));
            controlServer()->controller().init();
        }

        if (doc.RootElement()->FirstChildElement("Model"))
        {
            controlServer()->model().loadXml(*doc.RootElement()->FirstChildElement("Model"));
            controlServer()->model().init();
        }
        // 这里后面需要改 ////////////////////////////////////////


        option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    SetXml::~SetXml() = default;
    SetXml::SetXml(const std::string &name) : Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"set_xml\">"
            "	<Param name=\"xml\"/>"
            "</Command>");
    }


    auto Start::prepareNrt()->void
    {
        controlServer()->start();
        option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    Start::~Start() = default;
    Start::Start(const std::string &name) : Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"cs_start\">"
            "</Command>");
    }


    auto Stop::prepareNrt()->void
    {
        controlServer()->stop();
        option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    Stop::~Stop() = default;
    Stop::Stop(const std::string &name) : Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"cs_stop\">"
            "</Command>");
    }


    // set cycle_time for driver with SDO //
    struct SetCTParam
    {
        int16_t cycle_time;
    };
    auto SetCT::prepareNrt()->void
    {
        SetCTParam param;
        param.cycle_time = 1;

        for (auto &p : cmdParams())
        {
            if (p.first == "ct")
            {
                param.cycle_time = int32Param(p.first);
            }
        }

        for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
        {
            ecController()->slavePool().at(i).writePdo(0x60c2, 0x01, &param.cycle_time);
        }

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    SetCT::SetCT(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setCT\">"
            "	<GroupParam>"
            "		<Param name=\"ct\" abbreviation=\"t\" default=\"1\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    //设置全局速度//
    struct SetVelParam
    {
        int vel_percent;
    };
    auto SetVel::prepareNrt()->void
    {
        SetVelParam param;
        for (auto &p : cmdParams())
        {
            if (p.first == "vel_percent")
            {
                param.vel_percent = int32Param(p.first);
            }
        }
        g_vel_percent.store(param.vel_percent);

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    SetVel::SetVel(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"setvel\">"
            "	<GroupParam>"
            "		<Param name=\"vel_percent\" abbreviation=\"p\" default=\"0\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    //var//
    struct VarParam
    {
        std::string name;
        std::string value;
        std::string type;
    };
    auto Var::prepareNrt()->void
    {
        VarParam param;
        for (auto &p : cmdParams())
        {
            if(p.first == "name")
            {
                param.name = p.second;
            }
            else if (p.first == "value")
            {
                param.value = p.second;
            }
            else if (p.first == "type")
            {
                param.type = p.second;
            }
            else if (p.first == "clear")
            {
                g_cal.clearVariables();
            }
        }
        if ((param.type == "int") || (param.type == "double") || (param.type == "bool"))//di1,di2,do1,do2,ai1,ai2,int counter
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
            std::cout << "lnc:" << this->cmdString() << std::endl;
        }
        else if (param.type == "string")//string="kaanh"
        {
            g_cal.addVariable(param.name, param.type, param.value);
        }
        else if (param.type == "load")//load1={mass,cogx,cogy,cogz,q1,q2,q3,q4,ix,iy,iz}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "pose")//pose1={x,y,z,q1,q2,q3,q4}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "jointtarget")//jointtarget={j1,j2,j3,j4,j5,j6,ej1}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "robtarget")//robtarget={x,y,z,q1,q2,q3,q4}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "zone")//zone={dis,per}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "speed")	//speed={per,tcp,ori,exj_r,exj_l}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "tool")//tool={x,y,z,a,b,c}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }
        else if (param.type == "wobj")//wobj={x,y,z,a,b,c}
        {
            g_cal.addVariable(param.name, param.type, matrixParam(param.value));
        }

        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    Var::Var(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"var\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<GroupParam>"
            "				<Param name=\"name\" default=\"0\"/>"
            "				<Param name=\"value\" default=\"0\"/>"
            "				<Param name=\"type\" default=\"0\"/>"
            "			</GroupParam>"
            "			<Param name=\"clear\"/>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


    //Evaluate//
    auto Evaluate::prepareNrt()->void
    {
        std::string value;
        for (auto &p : cmdParams())
        {
            if (p.first == "value")
            {
                value = p.second;
            }
        }

        auto ret_mat = std::any_cast<aris::core::Matrix>(g_cal.calculateExpression(value).second);
        std::cout << ret_mat.toString() << std::endl;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret_value.push_back(std::pair<std::string, std::any>("evaluate", ret_mat));
        ret() = ret_value;
    }
    Evaluate::Evaluate(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"evaluate\">"
            "	<GroupParam>"
            "		<Param name=\"value\" default=\"0\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    //运行指令//
    struct RunParam
    {
        int goto_cmd_id;
        std::thread run;
    };
    std::mutex mymutex;
    std::mutex runmutex;
    bool is_running = false;
    auto is_auto_executing()->bool
    {
        std::unique_lock<std::mutex>runmutex;
        return is_running;
    }
    auto set_is_auto_executing(bool param)->bool
    {
        std::unique_lock<std::mutex>runmutex;
        bool now = is_running;
        is_running = param;
        return now;
    }
    auto Run::prepareNrt()->void
    {
        std::vector<std::pair<std::string, std::any>> run_ret;
        auto param = std::make_shared<RunParam>();
        static CmdListParam cmdparam;

        for (auto &p : cmdParams())
        {
            if (p.first == "forward")
            {
                //有指令在执行//
                if (is_auto_executing())
                {
                    option() = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
                }
                //没有指令在执行//
                else
                {
                    const bool is_forward = true;
                    set_is_auto_executing(is_forward);
                    param->run = std::thread([&]()->void
                    {
                        try
                        {
                            auto&cs = aris::server::ControlServer::instance();
                            {
                                std::unique_lock<std::mutex> run_lock(mymutex);
                                auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
                                if (iter == cmdparam.cmd_vec.end())
                                {
                                    set_is_auto_executing(false);
                                    return;
                                }
                                else
                                {
                                    cmdparam.current_plan_id = iter->first;
                                }
                            }

                            cs.executeCmdInCmdLine(aris::core::Msg(cmdparam.cmd_vec[cmdparam.current_cmd_id]).data(), [&](aris::plan::Plan &plan)->void
                            {
                                std::unique_lock<std::mutex> run_lock(mymutex);
                                auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
                                iter++;
                                std::cout << "current_cmd_id:" << iter->second << std::endl;
                                if (iter != cmdparam.cmd_vec.end())
                                {
                                    cmdparam.current_plan_id = iter->first;
                                    cmdparam.current_cmd_id = iter->first;
                                }
                                else
                                {
                                    cmdparam.current_plan_id = cmdparam.cmd_vec.rbegin()->first + 1;
                                }
                                set_is_auto_executing(false);
                            });
                        }
                        catch (std::exception &e)
                        {
                            std::cout << e.what() << std::endl;
                            LOG_ERROR << e.what() << std::endl;
                        }
                    });
                }
            }
            else if (p.first == "goto")
            {
                param->goto_cmd_id = int32Param(p.first);
                //有指令在执行//
                if (is_auto_executing())
                {
                    option() = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
                }
                //没有指令在执行//
                else
                {
                    const bool is_goto = true;
                    set_is_auto_executing(is_goto);
                    param->run = std::thread([&]()->void
                    {
                        bool is_existed = false;
                        {
                            std::unique_lock<std::mutex> run_lock(mymutex);
                            for (auto iter = cmdparam.cmd_vec.begin(); iter != cmdparam.cmd_vec.end(); ++iter)
                            {
                                if ((iter->first == param->goto_cmd_id) && (cmdparam.current_cmd_id != iter->first))
                                {
                                    auto temp_iter = iter;
                                    temp_iter--;
                                    cmdparam.current_cmd_id = temp_iter->first;
                                    cmdparam.current_cmd_id = std::max(cmdparam.current_cmd_id, 0);
                                    is_existed = true;
                                    cmdparam.current_plan_id = cmdparam.cmd_vec.find(cmdparam.current_cmd_id)->first;
                                }
                            }
                        }
                        if (!is_existed)
                        {
                            set_is_auto_executing(false);
                            return;
                        }
                        else
                        {
                            try
                            {

                                aris::server::ControlServer::instance().executeCmdInCmdLine(aris::core::Msg(cmdparam.cmd_vec[cmdparam.current_cmd_id]).data(), [&](aris::plan::Plan &plan)->void
                                {
                                    std::unique_lock<std::mutex> run_lock(mymutex);
                                    auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
                                    iter++;
                                    if (iter != cmdparam.cmd_vec.end())
                                    {
                                        cmdparam.current_plan_id = iter->first;
                                        cmdparam.current_cmd_id = iter->first;
                                    }
                                    else
                                    {
                                        cmdparam.current_plan_id = cmdparam.cmd_vec.rbegin()->first + 1;
                                    }
                                    const bool is_not_goto = false;
                                    set_is_auto_executing(is_not_goto);
                                });
                            }
                            catch (std::exception &e)
                            {
                                std::cout << e.what() << std::endl;
                                LOG_ERROR << e.what() << std::endl;
                            }
                        }
                    });
                }

            }
            else if (p.first == "start")
            {
                g_is_auto.store(true);
                //有指令在执行//
                if (is_auto_executing())
                {
                    option() = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
                }
                //没有指令在执行//
                else
                {
                    const bool is_start = true;
                    set_is_auto_executing(is_start);
                    param->run = std::thread([&]()->void
                    {
                        try
                        {
                            int begin_cmd_id = 0, end_cmd_id = 0;
                            auto&cs = aris::server::ControlServer::instance();
                            {
                                std::unique_lock<std::mutex> run_lock(mymutex);
                                auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id);
                                if (iter == cmdparam.cmd_vec.end())
                                {
                                    set_is_auto_executing(false);
                                    return;
                                }
                                else
                                {
                                    cmdparam.current_plan_id = iter->first;
                                }
                            }
                            for (auto iter = cmdparam.cmd_vec.find(cmdparam.current_cmd_id); iter != cmdparam.cmd_vec.end(); ++iter)
                            {
                                cs.executeCmdInCmdLine(aris::core::Msg(iter->second).data(), [iter](aris::plan::Plan &plan)->void
                                {
                                    std::unique_lock<std::mutex> run_lock(mymutex);
                                    auto temp_iter = iter;
                                    temp_iter++;
                                    if (temp_iter != cmdparam.cmd_vec.end())
                                    {
                                        cmdparam.current_plan_id = temp_iter->first;
                                        cmdparam.current_cmd_id = temp_iter->first;
                                    }
                                    else
                                    {
                                        cmdparam.current_plan_id = cmdparam.cmd_vec.rbegin()->first + 1;
                                        const bool is_not_start = false;
                                        set_is_auto_executing(is_not_start);
                                    }
                                    g_is_auto.store(false);

                                });
                            }
                        }
                        catch (std::exception &e)
                        {
                            std::cout << e.what() << std::endl;
                            LOG_ERROR << e.what() << std::endl;
                        }
                    });
                }
            }
            else if (p.first == "pause")
            {
                std::unique_lock<std::mutex> run_lock(mymutex);
                const bool is_pause = false;
                set_is_auto_executing(is_pause);

                option() = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
            }
            else if (p.first == "stop")
            {
                std::unique_lock<std::mutex> run_lock(mymutex);
                cmdparam.current_cmd_id = 0;
                cmdparam.current_plan_id = -1;
                const bool is_stop = false;
                set_is_auto_executing(is_stop);

                option() = Plan::NOT_RUN_COLLECT_FUNCTION | Plan::NOT_RUN_EXECUTE_FUNCTION;
            }
        }
        ret() = run_ret;
        this->param() = param;
    }
    auto Run::collectNrt()->void
    {
        auto param = std::any_cast<std::shared_ptr<RunParam>>(this->param());
        param->run.join();
    }
    Run::Run(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"run\">"
            "	<GroupParam>"
            "		<UniqueParam>"
            "			<Param name=\"forward\"/>"
            "			<Param name=\"goto\" default=\"2\"/>"
            "			<Param name=\"start\"/>"
            "			<Param name=\"pause\"/>"
            "			<Param name=\"stop\"/>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


    //编程界面指令//
    bool splitString(std::string spCharacter, const std::string& objString, std::map<int, std::string>& stringVector)
    {
        if (objString.length() == 0)
        {
            return true;
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
            auto sep_pos = str.find(":");
            auto id = str.substr(0, sep_pos);
            auto command = str.substr(sep_pos + 1);
            stringVector.insert(std::pair<int, std::string>(std::stoi(id), command.c_str()));
            posEnd += spCharacter.size();
        }
        return true;
    }
    auto onReceivedMsg(aris::core::Socket *socket, aris::core::Msg &msg)->int
    {
        std::string msg_data = msg.toString();

        {
            LOG_INFO << "receive cmd:"
                << msg.header().msg_size_ << "&"
                << msg.header().msg_id_ << "&"
                << msg.header().msg_type_ << "&"
                << msg.header().reserved1_ << "&"
                << msg.header().reserved2_ << "&"
                << msg.header().reserved3_ << ":"
                << msg_data << std::endl;
        }

        //std::string data = "collectcmd --cmdlist = {1:moveJ\r\n2:moveJ\r\n3:moveL\r\n}";
        std::string s = " --";
        auto cmd_name_pos = msg_data.find(s);
        auto cmd_name = msg_data.substr(0, cmd_name_pos);

        //解析指令失败//
        if (cmd_name.empty())
        {
            aris::core::Msg ret_msg(msg);
            std::vector<std::pair<std::string, std::any>> *js;
            js->push_back(std::make_pair<std::string, std::any>("return_code", -3));
            js->push_back(std::make_pair<std::string, std::any>("return_message", std::string("PARSE_EXCEPTION")));
            ret_msg.copy(aris::server::parse_ret_value(*js));
            // return back to source
            try
            {
                socket->sendMsg(ret_msg);
            }
            catch (std::exception &e)
            {
                std::cout << e.what() << std::endl;
                LOG_ERROR << e.what() << std::endl;
            }
        }
        else
        {
            if (strcmp(cmd_name.c_str(), "collectcmd") == 0)
            {
                {
                    std::unique_lock<std::mutex> run_lock(mymutex);
                    cmdparam.cmd_vec.clear();
                    cmdparam.current_cmd_id = 0;
                    cmdparam.current_plan_id = -1;
                }
                auto begin_pos = msg_data.find("{");
                auto end_pos = msg_data.rfind("}");
                auto cmd_str = msg_data.substr(begin_pos + 1, end_pos - 1 - begin_pos);


                std::cout << "aaaaaaaaaaaaaaa\r\nbbbbbbbbbbbbbbbb" << std::endl;
                std::cout << cmd_str << std::endl;
                const std::string split = "\\r\\n";
                splitString(split, cmd_str, cmdparam.cmd_vec);

                auto iter = cmdparam.cmd_vec.begin();
                if (iter != cmdparam.cmd_vec.end())
                {
                    cmdparam.current_cmd_id = iter->first;
                    std::cout << "cmd_vec:" << iter->second << std::endl;
                }
                else
                {
                    std::cout << "cmd_vec is null" << std::endl;
                }
            }
            else
            {
                try
                {
                    aris::server::ControlServer::instance().executeCmdInCmdLine(aris::core::Msg(msg).data(), [socket, msg](aris::plan::Plan &plan)->void
                    {
                        // make return msg
                        aris::core::Msg ret_msg(msg);
                        // only copy if it is a str
                        if (auto str = std::any_cast<std::string>(&plan.ret()))
                        {
                            ret_msg.copy(*str);
                        }
                        else if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
                        {
                            js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
                            js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
                            ret_msg.copy(aris::server::parse_ret_value(*js));
                        }
                        // return back to source//
                        try
                        {
                            socket->sendMsg(ret_msg);
                        }
                        catch (std::exception &e)
                        {
                            std::cout << e.what() << std::endl;
                            LOG_ERROR << e.what() << std::endl;
                        }
                    });
                }
                catch (std::exception &e)
                {
                    std::vector<std::pair<std::string, std::any>> ret_pair;
                    ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
                    ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
                    std::string ret_str = aris::server::parse_ret_value(ret_pair);

                    //std::cout << ret_str << std::endl;
                    LOG_ERROR << ret_str << std::endl;

                    try
                    {
                        aris::core::Msg m = msg;
                        m.copy(ret_str);
                        socket->sendMsg(m);
                    }
                    catch (std::exception &e)
                    {
                        std::cout << e.what() << std::endl;
                        LOG_ERROR << e.what() << std::endl;
                    }
                }
            }
        }
        return 0;
    }
    auto onReceivedConnection(aris::core::Socket *sock, const char *ip, int port)->int
    {
        std::cout << "socket receive connection" << std::endl;
        LOG_INFO << "socket receive connection:\n"
            << std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
            << std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
        return 0;
    }
    auto onLoseConnection(aris::core::Socket *socket)->int
    {
        std::cout << "socket lose connection" << std::endl;
        LOG_INFO << "socket lose connection" << std::endl;
        for (;;)
        {
            try
            {
                socket->startServer(socket->port());
                break;
            }
            catch (std::runtime_error &e)
            {
                std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
                LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        std::cout << "socket restart successful" << std::endl;
        LOG_INFO << "socket restart successful" << std::endl;

        return 0;
    }
    auto ProInterface::open()->void { sock_->startServer(); }
    auto ProInterface::close()->void { sock_->stop(); }
    auto ProInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
    {
        Interface::loadXml(xml_ele);
        this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);

        sock_->setOnReceivedMsg(onReceivedMsg);
        sock_->setOnReceivedConnection(onReceivedConnection);
        sock_->setOnLoseConnection(onLoseConnection);
    }
    ProInterface::ProInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type) :Interface(name)
    {
        sock_ = &add<aris::core::Socket>("socket", "", port, type);
        sock_->setOnReceivedMsg(onReceivedMsg);
        sock_->setOnReceivedConnection(onReceivedConnection);
        sock_->setOnLoseConnection(onLoseConnection);
    }


    // 手动--自动模式切换； manual=1，手动;manual=0，自动//
    struct SwitchParam
    {
        bool is_manual;
    };
    auto Switch::prepareNrt()->void
    {
        SwitchParam param;
        for (auto &p : cmdParams())
        {
            if (p.first == "manual")
            {
                param.is_manual = int32Param(p.first);
            }
        }
        g_is_manual.store(param.is_manual);
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
        option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
    }
    Switch::Switch(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"switch\">"
            "		<Param name=\"manual\" default=\"1\"/>"
            "</Command>");
    }


    struct MotorModeParam
    {
        uint8_t mode;
    };
    auto MotorMode::prepareNrt()->void
    {
        MotorModeParam param;
        param.mode = 1;

        for (auto &p : cmdParams())
        {
            if (p.first == "mode")
            {
                param.mode = int32Param(p.first);
            }
        }

        uint16_t status_word = 0;
        for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
        {
            ecController()->motionPool().at(i).writeSdo(0x6060, 0x00, &param.mode, 1);
            ecController()->motionPool().at(i).readPdo(0x6041, 0x00, &status_word, 16);
        }

        for (auto &option : motorOptions())	option |= CHECK_NONE;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto MotorMode::collectNrt()->void {}
    MotorMode::MotorMode(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"md\">"
            "	<GroupParam>"
            "		<Param name=\"mode\" default=\"8\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    auto EnableMotor::prepareNrt()->void
    {
        for (auto &option : motorOptions())	option |= CHECK_NONE;
        std::vector<std::pair<std::string, std::any>> ret_value;
        ret() = ret_value;
    }
    auto EnableMotor::executeRT()->int
    {
        uint16_t step2 = 0x0007;
        uint16_t step3 = 0x000f;
        uint16_t step4 = 0x007f;
        uint16_t status_word[6];
        static std::int64_t total_count = 1;
        for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
        {
            ecController()->motionPool().at(i).readPdo(0x6041, 0x00, &status_word[i], 16);
        }
        static bool ret = true;
        static std::int64_t ref_count[6] = {1,1,1,1,1,1};
        static int16_t temp = 1;
        for (aris::Size i = 0; i < controller()->slavePool().size(); i++)
        {
            if ((status_word[i] & 0x02) == 0x00)
            {
                auto cur_pos = ecController()->motionPool().at(i).actualPos();
                ecController()->motionPool().at(i).setTargetPos(cur_pos);
                //ecController()->motionPool().at(i).setModeOfOperation(8);
                //ecController()->motionPool().at(i).writePdo(0x607a, 0x00, &cur_pos, 32);
                ecController()->motionPool().at(i).writePdo(0x6040, 0x00, &step2, 16);
                controller()->mout() << "1" << std::endl;
            }
            else if ((status_word[i] & 0x02) == 0x02)
            {
                ecController()->motionPool().at(i).writePdo(0x6040, 0x00, &step3, 16);
                controller()->mout() << "2" << std::endl;
                if (temp == 1)
                {
                    ref_count[i] = count() + 1000;
                    temp = 0;
                }
                ret = false;
            }
            else if ((status_word[i] & 0x04) == 0x04)
            {
                if (temp == 1)
                {
                    ref_count[i] = count() + 1000;
                    temp = 0;
                }
                ret = false;
            }
            total_count = std::max(ref_count[i], total_count);
        }

        return ret ? 1 : (total_count - count());

    }
    auto EnableMotor::collectNrt()->void {}
    EnableMotor::EnableMotor(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"en\">"
            "</Command>");
    }


}
