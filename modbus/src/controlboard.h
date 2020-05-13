#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "aris.hpp"
#include "modbus.h"
#define print_time false	//是否需要打印时间


class controlboard
{
public:
    auto send_command(uint16_t *buffer, aris::server::ControlServer &cs)->bool;//发送指令

private:
	bool cor = 0;				//默认工具坐标系
    bool joint_cart = true;     //默认关节坐标系
	int velocity = 1;			//默认加减速值为1

	//控制指令集
	std::string control_other[13] = { "program --set_auto","program --set_manual","mvJoint","FCStop","mvf","rs","cl","rc","ds","ds","md","en","rc" };

	//关节指令集
	std::string control_motor[12] = { "j1 --direction=-1","j1 --direction=1",
							"j2 --direction=-1","j2 --direction=1",
							"j3 --direction=-1","j3 --direction=1",
							"j4 --direction=-1","j4 --direction=1",
							"j5 --direction=-1","j5 --direction=1",
							"j6 --direction=-1","j6 --direction=1" };

    //笛卡尔坐标系指令集
	std::string control_pose[12] = { "jx --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jx --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jy --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jy --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jz --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jz --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrx --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrx --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jry --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jry --direction=1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrz --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
							"jrz --direction=1 --cor=0 --tool=tool0 --wobj=wobj0" };
};


#endif // CONTROLBOARD_H
