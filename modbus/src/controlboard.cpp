//
// Created by Fanzhe on 5/29/2017.
//

#include "modbus.h"
#include <unistd.h> //用于usleep() 延迟挂起进程
#include <sys/timeb.h>//用于打印时间
#include "controlboard.h"


#define print_time true //是否需要打印时间
bool control_mode = true;//true代表控制电机，false代表控制末端
unsigned char velocity = 10;//速度值 取0~100
//控制电机的指令
std::string control_other[] = { "program --set_auto","program --set_manual","mvJoint","FCStop","mvf","rs","cl","rc","ds","ds","md","en","rc"};
std::string control_motor[] = {"j1 --direction=-1","j1 --direction=1",
                        "j2 --direction=-1","j2 --direction=1",
                        "j3 --direction=-1","j3 --direction=1",
                        "j4 --direction=-1","j4 --direction=1",
                        "j5 --direction=-1","j5 --direction=1",
                        "j6 --direction=-1","j6 --direction=1"};
//控制末端的指令
std::string control_pose[] = {"jx --direction=-1 --cor=0 --tool=tool0 --wobj=wobj0",
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
                        "jrz --direction=1 --cor=0 --tool=tool0 --wobj=wobj0"};
//识别按键号并发送指令
void send_command(uint16_t *buffer, int j[2])
{//buffer[0]和[1]分别对应华途示教器寄存器地址30001和30002,至少有一个二进制位为1
    uint32_t bit32 = (buffer[1] << 16) + buffer[0];
    int count_press = 0, index = -1;//1的个数和1的下标
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
		return;
	}
	//index=0代表按键1，index=30代表按键31
    char command[100];
    if (index <= 5)
	{
        sprintf(command, control_other[index].c_str(), velocity);//生成指令
		cout << command << endl;
	}
    else if(index == 6)
    {
        sprintf(command, control_other[index].c_str(), velocity);//生成指令
        cout << command << endl;
        sprintf(command, control_other[index+1].c_str(), velocity);//生成指令
        cout << command << endl;
    }
    else if(index == 7)
    {
        sprintf(command, control_other[index+1].c_str(), velocity);//生成指令
        cout << command << endl;
    }
    else if(index == 8)
    {
        sprintf(command, control_other[index+1].c_str(), velocity);//生成指令
        cout << command << endl;
        sprintf(command, control_other[index+2].c_str(), velocity);//生成指令
        cout << command << endl;
        sprintf(command, control_other[index+3].c_str(), velocity);//生成指令
        cout << command << endl;
        sprintf(command, control_other[index+4].c_str(), velocity);//生成指令
        cout << command << endl;
    }
    else if(index >= 9 && index <= 20)
	{//按键10至按键21，即右侧+-号
        j[1] = 1;
        if((j[0]%2==0)){
            sprintf(command,control_motor[index-9].c_str(),velocity);//生成指令
            cout << command << endl;
        }
        else
        {
            sprintf(command,control_pose[index-9].c_str(),velocity);//生成指令
            cout << command << endl;
        }
    }
	else if(index >= 29)
	{//按键30和31，更改速度
		if(index == 29)//按键30减速
		{
			velocity = velocity-10 >= 0 ? velocity-10 : 0;
			cout << "V-" << endl;
		}
		else//按键31加速
        {
			velocity = velocity+10 <= 100 ? velocity+10 : 100;
			cout << "V+" << endl;
        }
	}
    else if(index == 21)
    {
        if(j[1]==1)
        {
            j[0]=j[0]+1;
            cout<<j<<endl;
            cout<<"change coordinate system"<<endl;
            j[1]=0;
        }
    }
	else
	{
        //sprintf(command, control_other[index], velocity);
        //cout << command << endl;
        cout << "Reserved" << endl;
    }
	return;
}

auto fun_modbus()->bool
{
	// create a modbus object
	modbus mb = modbus("192.168.0.21", 502);//从站的ip地址和端口号
	int j[2] = { 0,0 };

	// set slave id
	mb.modbus_set_slave_id(1);

	// connect with the server
	mb.modbus_connect();


	while (1)
	{
		//添加执行部分代码
	}

	// close connection and free the memory
	mb.modbus_close();

	//delete(&mb);//报错“double free”
	return 0;
}

int t_main()
{
    // create a modbus object
	modbus mb = modbus("192.168.0.21", 502);//从站的ip地址和端口号
    int j[2] = {0,0};
    // set slave id
    mb.modbus_set_slave_id(1);

    // connect with the server
    mb.modbus_connect();
/*
    // read coil                        function 0x01
    bool read_coil;
    mb.modbus_read_coils(0, 1, &read_coil);


    // read input bits(discrete input)  function 0x02
    bool read_bits;
    mb.modbus_read_input_bits(0, 1, &read_bits);


    // read holding registers           function 0x03
    uint16_t read_holding_regs[1];
    mb.modbus_read_holding_registers(0, 1, read_holding_regs);
*/
/*************以下打印时间相关**************************/
#if print_time
	struct timeb t1;
	double time_new, time_last, time_max = 0;
	ftime(&t1);
	time_new = time_last = t1.time + 0.001* t1.millitm;
#endif
/*************以上打印时间相关**************************/
    // read input registers             function 0x04
    uint16_t read_input_regs[2];
    while(1)
    {
/*************以下打印时间相关**************************/
	#if print_time
        ftime(&t1);
        time_last = time_new;
        time_new = t1.time + 0.001* t1.millitm;
        if(time_new-time_last > time_max)
            time_max = time_new-time_last;
        printf("[%.3f %.3f %.3f]\n", time_new, time_new-time_last, time_max);
    #endif
/*************以上打印时间相关**************************/
        //read_input_regs[0]存地址为30001的寄存器,read_input_regs[1]存地址为30002的寄存器
		//读输入寄存器 功能码04
        mb.modbus_read_input_registers(0, 2, read_input_regs);
        if(read_input_regs[0] != 0 || read_input_regs[1] != 0)//有按键按下
            send_command(read_input_regs,j);//发送指令
        usleep(50*1000);//usleep微秒，sleep秒
    }


/*
    // write single coil                function 0x05
    mb.modbus_write_coil(0, true);
*/


    // write single reg                 function 0x06
    //mb.modbus_write_register(1, 0);//写地址为40002的寄存器(控制4个LED和蜂鸣器)

/*
    // write multiple coils             function 0x0F
    bool write_cols[4] = {true, true, true, true};
    mb.modbus_write_coils(0,4,write_cols);


    // write multiple regs              function 0x10
    uint16_t write_regs[4] = {123, 123, 123};
    mb.modbus_write_registers(0, 4, write_regs);
*/

    // close connection and free the memory
    mb.modbus_close();
    //delete(&mb);//报错“double free”
    return 0;
}
