#include "modbus.h"
#include <sys/timeb.h>		//用于打印时间
#include "controlboard.h"


//识别按键并发送指令
auto controlboard::send_command(uint16_t *buffer, aris::server::ControlServer &cs)->bool
{
	//buffer[0]和[1]分别对应华途示教器寄存器地址30001和30002,至少有一个二进制位为1
    uint32_t bit32 = (buffer[1] << 16) + buffer[0];
    int count_press = 0, index = -1;	//1的个数和1的下标
	for(int i = 0; i < 32; i++)
	{
		if((bit32 >> i) & 0x01)
		{
			count_press++;
			index = i;
		}
	}
	if(count_press > 1)
	{
		cout << "ERROR: Don't press 2 buttons at same time." <<endl;
		return 0;
	}
	
	// index的编号范围0~30，按键编号范围1~31，index=0代表按键1，index=30代表按键31
    char command[100];
    if (index <= 5)	
	{
        sprintf(command, control_other[index].c_str());
		cs.executeCmd(command);//"program --set_auto","program --set_manual","mvJoint","FCStop","mvf","rs"
		cout << command << endl;
	}
    else if(index == 6)	//清除错误
    {
        sprintf(command, control_other[index].c_str());
		cs.executeCmd(command);//cl
        cout << command << endl;
        sprintf(command, control_other[index+1].c_str());
		cs.executeCmd(command);//rc
        cout << command << endl;
    }
    else if(index == 7)
    {
        sprintf(command, control_other[index+1].c_str());
		cs.executeCmd(command);//ds
        cout << command << endl;
    }
    else if(index == 8)
    {
        sprintf(command, control_other[index+1].c_str());
		cs.executeCmd(command);//ds
        cout << command << endl;
        sprintf(command, control_other[index+2].c_str());
		cs.executeCmd(command);//md
        cout << command << endl;
        sprintf(command, control_other[index+3].c_str());
		cs.executeCmd(command);//en
        cout << command << endl;
        sprintf(command, control_other[index+4].c_str());
		cs.executeCmd(command);//rc
        cout << command << endl;

    }
    else if(index >= 9 && index <= 20)
	{
		//按键10至按键21，即右侧+-号
        if(joint_cart){
            sprintf(command,control_motor[index-9].c_str());
            cs.executeCmd(command);
            cout << command << endl;
        }
        else
        {
            sprintf(command,control_pose[index-9].c_str());
			cs.executeCmd(command);
            cout << command << endl;
        }
    }
	else if(index >= 29)
	{	
		//按键30和31，更改速度
		if(index == 29)	//按键30减速
		{
			velocity = velocity - 1 >= 0 ? velocity - 1 : 1;
            string s_velocity = to_string(velocity);
            string cmd = "setvel --vel_percent="+s_velocity;
            cs.executeCmd(cmd);
            cout<<cmd<<endl;		
		}
		else//按键31加速
        {
			velocity = velocity + 1 <= 100 ? velocity + 1 : 100;
            string s_velocity = to_string(velocity);
            string cmd = "setvel --vel_percent="+s_velocity;
            cs.executeCmd(cmd);
            cout<<cmd<<endl;
        }
	}
	else if (index == 24)
	{
		joint_cart = true;
	}
	else if (index == 25)
	{
		joint_cart = false;
	}
    else if(index == 21)//切换坐标系
    {
		cor = ~cor;
        cout<<"change coordinate system"<<endl;
        if(cor)
        {
            cout<<"this is Cartesian coordinates"<<endl;
        }
        else
        {
            cout<<"this is Axis coordinates"<<endl;
        }
		std::this_thread::sleep_for(std::chrono::seconds(1));
    }
	else
	{
        cout << "Reserved" << endl;
    }
	return 1;
}

auto controlboard::fun_modbus(aris::server::ControlServer &cs)->bool
{
	// 创建modbus master
	modbus mb = modbus("192.168.0.21", 502);	//从站的ip地址和端口号
	mb.modbus_set_slave_id(1);					// set slave id
	
	// 连接modbus slave，并且响应modbus slave的信息
start:
	for (;;)
	{
		try
		{
			mb.modbus_connect();				// connect with the server		
			break;
		}
		catch (std::exception &e)
		{
			std::cout << "failed to connect server, will retry in 1 second" << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
	
	// 主要功能逻辑区
	try
	{
		cs.executeCmd("setvel --vel_percent=1;");

#if print_time
		struct timeb t1;
		double time_new, time_last, time_max = 0;
		ftime(&t1);
		time_new = time_last = t1.time + 0.001* t1.millitm;
#endif

		uint16_t read_input_regs[2];
		while (1)
		{
#if print_time
			ftime(&t1);
			time_last = time_new;
			time_new = t1.time + 0.001* t1.millitm;
			if (time_new - time_last > time_max)
				time_max = time_new - time_last;
			printf("[%.3f %.3f %.3f]\n", time_new, time_new - time_last, time_max);
#endif

			/*************以上打印时间相关**************************/
			//read_input_regs[0]存地址为30001的寄存器,read_input_regs[1]存地址为30002的寄存器
			//读输入寄存器 功能码04
			mb.modbus_read_input_registers(0, 2, read_input_regs);
			if (read_input_regs[0] != 0 || read_input_regs[1] != 0)			//有按键按下
				send_command(read_input_regs, cs);					//发送指令
			std::this_thread::sleep_for(std::chrono::milliseconds(100));	//100ms
		}

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	mb.modbus_close();	// close connection
	
	goto start;	// 断连接后，尝试重新连接

	delete(&mb); // 释放空间

	return 0;
}
