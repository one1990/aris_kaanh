#include <iostream>
#include <aris.hpp>
#include "tuying.h"
#include<atomic>
#include<string>
#include<filesystem>
#include<chrono>


#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library


// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_PRESENT_LOAD            40


// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_PRESENT_LOAD             2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1
#define DXL_ID1                         1                   // Dynamixel ID: 1
#define DXL_ID2                         2
#define DXL_ID3                         3
#define BAUDRATE                        57600
#define BAUDRATE1                       1000000
#define BAUDRATE2                       1000000
#define BAUDRATE3                       1000000

#ifdef UNIX
    #define DEVICENAME                  "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                                // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#endif

#ifdef WIN32
    #define DEVICENAME                  "COM6"
#endif

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      -28672              // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      28672               // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
#define ERRBIT_OVERLOAD         32							// The current load cannot be controlled by the set torque.

#define ESC_ASCII_VALUE                 0x1b
#define SCALING                         11.378

std::thread t_dynamixel;
std::mutex dynamixel_mutex;


std::atomic_int syn_clock = 0;
std::atomic_int mode_dynamixel = 2;		//mode_dynamixel——0:manual, 1:auto, 2:NA---不再使用
std::atomic_bool find_zero = false;		//找零
std::atomic_int dynamixel_control_mode = 0;		//dynamixel_mode----0：待命模式，1：手动模式，2：home模式，3：emily模式
std::atomic_bool dx1_home_succsess = false, dx2_home_succsess = false, dx3_home_succsess = false;				//home_succsess——1:home_succsess
std::atomic_int is_enabled = 2;						//is_enabled——0:disable, 1:enable, 2:NA
std::atomic_int16_t target_pos1 = 0, target_pos2 = 0, target_pos3 = 0;


bool dxl1_active = 0, dxl2_active = 0, dxl3_active = 0;
bool dxl1_exist = 0, dxl2_exist = 0, dxl3_exist = 0;						//舵机是否存在标志位
std::atomic_int16_t current_pos1 = 0, current_pos2 = 0, current_pos3 = 0;	//单位是度
std::atomic_int16_t pos1_ref = 0, pos2_ref = 0, pos3_ref = 0;			//第一次装机时位置标定值
std::atomic_int16_t pos1_offset = 0, pos2_offset = 0, pos3_offset = 0;	//相对于第一次装机位置参数的偏置
std::vector<std::vector<double>> dxl_pos;
bool dxl_addparam_result = false;                 // addParam result
bool dxl_getdata_result = false;                  // GetParam result

//dynamixel state flag//
std::atomic_bool dxl_connected = 0;	//0:未连接，1:连接
std::atomic_bool dxl_auto = 0;		//0:手动运行中，1:自动运行中----第一版
//舵机状态
//0:未使能，1:使能，2:手动运行中，3，正在执行emily，4:正在找home，5:等待命名
//-1:使能失败，-2:去使能失败，-3:读位置失败，-4:写位置失败，-5:同步写失败，-6:同步读失败，-7:找home失败，-8:舵机未连接，-9:其他异常
std::atomic_int dxl1_state = 0;		
std::atomic_int dxl2_state = 0;
std::atomic_int dxl3_state = 0;
std::atomic_bool dxl_enabled = 0;	//0:未使能，1:使能----第一版
std::atomic_int dxl_normal = 1;		//0:异常，1:正常----第一版

const int dxl_timeinterval = 10;	//舵机时间系数
int index = 0;
int dxl_comm_result1 = COMM_TX_FAIL, dxl_comm_result2 = COMM_TX_FAIL, dxl_comm_result3 = COMM_TX_FAIL;             // Communication result
int16_t dxl_goal_position = 0;
uint8_t dxl_error = 0;                          // Dynamixel error
uint16_t dxl_present_position1 = 0, dxl_present_position2 = 0, dxl_present_position3 = 0;
uint16_t dxl_present_load1 = 0, dxl_present_load2 = 0, dxl_present_load3 = 0;


int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}
int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}
auto enable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool
{
    uint8_t dxl_error = 0;
    // Set port baudrate for Dynamixel
    if (portHandler->setBaudRate(baudrate))
    {
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        //getch();
		dxl_connected.store(0);
        return 0;
    }
    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 0;
    }
    else if (dxl_error != 0)
    {
        std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return 0;
    }
    else
    {
		dxl_normal.store(1);//第一版
        //std::cout << "Dynamixel " << dxl_id << " has been successfully enabled" << std::endl;
		return 1;
    }
}
auto read_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, uint16_t &dxl_present_position)->bool
{
    uint8_t dxl_error = 0;
    if (portHandler->setBaudRate(baudrate))
    {
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        //getch();
		dxl_connected.store(0);
        return 0;
    }
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0)
    {
        //printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
	else
	{
		//std::cout << "Dynamixel " << dxl_id << " has been successfully read" << std::endl;
		return 1;
	}
}
auto write_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, int target_pos)->bool
{
    uint8_t dxl_error = 0;
    if (portHandler->setBaudRate(baudrate))
    {
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        //getch();
		dxl_connected.store(0);
        return 0;
    }
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, target_pos, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return 0;
    }
    else if (dxl_error != 0)
    {
        //printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        return 0;
    }
	else
	{
		//std::cout << "Dynamixel " << dxl_id << " has been successfully written" << std::endl;
		return 1;
	}
}
auto disable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool
{
    uint8_t dxl_error = 0;
    // Set port baudrate for Dynamixel
    if (portHandler->setBaudRate(baudrate))
    {
        //std::cout << "Succeeded to change the baudrate!" << std::endl;
    }
    else
    {
        //std::cout << "Failed to change the baudrate!" << std::endl;
        //getch();
		dxl_connected.store(0);
        return 0;
    }
    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //std::cout << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        return 0;
    }
    else if (dxl_error != 0)
    {
        //std::cout << packetHandler->getRxPacketError(dxl_error) << std::endl;
        return 0;
    }
    else
    {
        //std::cout << "Dynamixel " << dxl_id << " has been successfully disabled" << std::endl;
		return 1;
    }
}
auto groupBulkread_dynamixel(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead groupBulkRead) -> bool
{
	//bulkread function//
	// Add parameter storage for Dynamixel#1,Dynamixel#2,Dynamixel#3 present position value
	if (dxl1_exist && dxl1_active)
	{
		dxl_addparam_result = groupBulkRead.addParam(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_addparam_result != true)
		{
			dxl1_state.store(-6);
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID1);
			return -1;
		}
	}
	if (dxl2_exist && dxl2_active)
	{
		dxl_addparam_result = groupBulkRead.addParam(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_addparam_result != true)
		{
			dxl2_state.store(-6);
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID2);
			return -1;
		}
	}
	if (dxl3_exist && dxl3_active)
	{
		dxl_addparam_result = groupBulkRead.addParam(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_addparam_result != true)
		{
			dxl3_state.store(-6);
			fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID3);
			return -1;
		}
	}

	// Read Dynamixel#1,Dynamixel#2,Dynamixel#3 present position value
	auto dxl_comm_result = groupBulkRead.txRxPacket();
	if (dxl1_exist && dxl1_active)
	{
		dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
			dxl1_state.store(-6);
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID1);
			return -1;
		}
		// Get Dynamixel#1 present position value
		dxl_present_position1 = groupBulkRead.getData(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		auto dxl1 = std::int16_t(dxl_present_position1);
		current_pos1.store(1.0*dxl1 / SCALING);
	}
	if (dxl2_exist && dxl2_active)
	{
		dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
			dxl2_state.store(-6);
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID2);
			return -1;
		}
		// Get Dynamixel#2 present position value
		dxl_present_position2 = groupBulkRead.getData(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		auto dxl2 = std::int16_t(dxl_present_position2);
		current_pos2.store(1.0*dxl2 / SCALING);
	}
	if (dxl3_exist && dxl3_active)
	{
		dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
			dxl3_state.store(-6);
			fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID3);
			return -1;
		}
		// Get Dynamixel#3 present position value
		dxl_present_position3 = groupBulkRead.getData(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
		auto dxl3 = std::int16_t(dxl_present_position3);
		current_pos3.store(1.0*dxl3 / SCALING);
	}
	groupBulkRead.clearParam();
	return 0;
}


std::atomic_bool is_automatic = false;
using namespace aris::dynamic;

auto xmlpath = std::filesystem::absolute(".");//获取当前工程所在的路径
auto uixmlpath = std::filesystem::absolute(".");
auto modelxmlpath = std::filesystem::absolute(".");
const std::string xmlfile = "kaanh.xml";
const std::string uixmlfile = "interface_kaanh.xml";
//for qifan robot//
const std::string modelxmlfile = "model_qifan.xml";
//const std::string modelxmlfile = "model_rokae.xml";

int main(int argc, char *argv[])
{
    //指定线程cpu号
    /*
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(1, &mask);
    CPU_SET(2, &mask);
    */


    //Start t_Dynamixel thread//
    t_dynamixel = std::thread([&]()->bool
    {
        //指定线程运行cpu号
        //pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

	start:
        // Initialize PortHandler instance
        // Set the port path
        // Get methods and members of PortHandlerLinux or PortHandlerWindows
        dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

        // Initialize PacketHandler instance
        // Set the protocol version
        // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        //dynamixel::PacketHandler *packetHandler;

          // Initialize GroupSyncWrite instance
        dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

        // Initialize GroupBulkRead instance
        dynamixel::GroupBulkRead groupBulkRead(portHandler, packetHandler);

        // Open port 每次运行程序尝试连接端口，每秒钟一次，1min连接失败，退出 //
		dxl_enabled.store(0);//第一版
		dxl_normal.store(0);//第一版
		dxl_connected.store(0);
		dxl1_state.store(-8);
		dxl2_state.store(-8);
		dxl3_state.store(-8);
		syn_clock.store(0);
		dxl1_exist = 0, dxl2_exist = 0, dxl3_exist = 0;
		
		// When UI shows connected, the User can send DReset socket command to set dxl_connected to 0, and reconnect
		int connecting_couter = 10;
		while (dxl_connected.load() == 0)
		{
			if (portHandler->openPort())
			{
				std::cout << "Succeeded to open the port!" << std::endl;
				break;
			}
			else
			{
				std::cout << "Trying to open the port for 10 times. " << connecting_couter << " rounds to left." << std::endl;
				if (connecting_couter == 1)
				{
					dxl_connected.store(0);
					std::cout << "Failed to open the port!" << std::endl;
					std::cout << "Press any key to reconnect again ...! or close the window." << std::endl;
					//getch();//这句话会退出线程
					goto start;
					//break;
					//return 0;
				}
				std::this_thread::sleep_for(std::chrono::seconds(6));
			}
			connecting_couter--;
		}

		// Set port baudrate for Dynamixel
		if (portHandler->setBaudRate(BAUDRATE1))
		{
			dxl_connected.store(1);
			dxl1_state.store(5);
			dxl2_state.store(5);
			dxl3_state.store(5);
			std::cout << "Succeeded to change the baudrate!" << std::endl;
		}
		else
		{
			std::cout << "Failed to change the baudrate, Please configure the servos first!" << std::endl;
			std::cout << "Press any key to terminate...!" << std::endl;
			//getch();
			dxl_connected.store(0);
		}

		//端口连接成功//
		while (dxl_connected.load())
		{
			auto en = is_enabled.load();
			static bool enabled = false;
			try
			{
				if (en == 1)
				{
					// 使能3个舵机 //
					if (enable_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1) == 0)
					{
						dxl1_state.store(1);
						dxl1_exist = 1;
					}
					else
					{
						dxl1_state.store(-1);
					}
					if (enable_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2) == 0)
					{
						dxl2_state.store(1);
						dxl2_exist = 1;
					}
					else
					{
						dxl2_state.store(-1);
					}
					if (enable_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3) == 0)
					{
						dxl3_state.store(1);
						dxl3_exist = 1;
					}
					else
					{
						dxl3_state.store(-1);
					}
					std::cout << "enable_dynamixel 1, dxl1_state:" << dxl1_state.load() << std::endl;
					std::cout << "enable_dynamixel 2, dxl2_state:" << dxl2_state.load() << std::endl;
					std::cout << "enable_dynamixel 3, dxl3_state:" << dxl3_state.load() << std::endl;
					
					// 使能第一个周期设置目标位置为当前位置，避免点动时舵机快速回零
					groupBulkread_dynamixel(packetHandler, groupBulkRead);
					auto dxl1 = std::int16_t(dxl_present_position1);
					auto dxl2 = std::int16_t(dxl_present_position2);
					auto dxl3 = std::int16_t(dxl_present_position3);
					target_pos1.store(1.0*dxl1 / SCALING);
					target_pos2.store(1.0*dxl2 / SCALING);
					target_pos3.store(1.0*dxl3 / SCALING);
					is_enabled.store(2);
					enabled = true;
					dxl_enabled.store(true);//第一版
				}
				else if (en == 0)// Disable //
				{
					//去使能3个舵机//
					dxl1_state.store(disable_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1) ? 0 : -2);
					dxl2_state.store(disable_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2) ? 0 : -2);
					dxl3_state.store(disable_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3) ? 0 : -2);

					is_enabled.store(2);
					enabled = false;
					dxl_enabled.store(false);//第一版
				}
			}
			catch (std::exception &e)
			{
				std::cout << "Exception on setting enable/disable/reconnect mode" << std::endl;
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
				goto close;
			}
			
			try
			{
				// dynamixel_control_mode--0:standby mode;1:manual mode;2:home mode;3:emily mode //
				// Read the position of dynamixel1, dynamixel2, dynamixel3 //
				if (dynamixel_control_mode.load() == 0)//standby mode
				{
					groupBulkread_dynamixel(packetHandler, groupBulkRead);
					/*
					if (dxl1_exist)
					{
						if (read_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1, dxl_present_position1) != 0)
							dxl1_state.store(-3);
						std::cout << "read_dynamixel 1, dxl1_state:" << dxl1_state.load() << std::endl;
					}
					if (dxl2_exist)
					{
						if (read_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2, dxl_present_position2) != 0)
							dxl2_state.store(-3);
						std::cout << "read_dynamixel 2, dxl2_state:" << dxl2_state.load() << std::endl;
					}
					if (dxl3_exist)
					{
						if (read_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3, dxl_present_position3) != 0)
							dxl3_state.store(-3);
						std::cout << "read_dynamixel 3, dxl3_state:" << dxl3_state.load() << std::endl;
					}
					auto dxl1 = std::int16_t(dxl_present_position1);
					auto dxl2 = std::int16_t(dxl_present_position2);
					auto dxl3 = std::int16_t(dxl_present_position3);
					current_pos1.store(1.0 * dxl1 / SCALING);
					current_pos2.store(1.0 * dxl2 / SCALING);
					current_pos3.store(1.0 * dxl3 / SCALING);
					*/
				}
				else if (dynamixel_control_mode.load() == 1)//manual mode
				{
					std::cout << "manual mode begin" << std::endl;
					//舵机的状态处于使能或者手动状态下，才能进行手动控制//
					if (dxl1_exist)
					{
						auto pos1 = target_pos1.load();
						dxl1_state.store(write_dynamixel(portHandler, packetHandler, dxl_comm_result1, DXL_ID1, BAUDRATE1, pos1) ? 2 : -4);
					}
					if (dxl2_exist)
					{
						auto pos2 = target_pos2.load();
						dxl2_state.store(write_dynamixel(portHandler, packetHandler, dxl_comm_result2, DXL_ID2, BAUDRATE2, pos2) ? 2 : -4);
					}
					if (dxl3_exist)
					{
						auto pos3 = target_pos3.load();
						dxl3_state.store(write_dynamixel(portHandler, packetHandler, dxl_comm_result3, DXL_ID3, BAUDRATE3, pos3) ? 2 : -4);
					}
					std::cout << "manual mode end" << std::endl;
				}
				else if (dynamixel_control_mode.load() == 2)//home mode
				{
					std::cout << "homeing mode begin" << std::endl;
					dxl_auto.store(true);//第一版
					int16_t step = 2;	//homing speed
					int16_t pos = 0;
					int16_t load = 150;	//判断找到home的载荷值
					//1号舵机找home//
					if (dxl1_exist && dxl1_state.load() > 0)
					{
						dxl1_state.store(4);
						int16_t pos1 = current_pos1.load()*SCALING;
						for (int pos = pos1; pos >= DXL_MINIMUM_POSITION_VALUE; pos -= step)
						{
							auto dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_GOAL_POSITION, pos, &dxl_error);
							if (dxl_error & ERRBIT_OVERLOAD)
							{
								std::cout << "overload" << std::endl;
								dxl1_state.store(-7);
								break;
							}
							auto dxl_load_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID1, ADDR_MX_PRESENT_LOAD, &dxl_present_load1, &dxl_error);
							if (dxl_load_result != COMM_SUCCESS)
							{
								printf("%s\n", packetHandler->getTxRxResult(dxl_load_result));
								break;
							}
							else if (dxl_error != 0)
							{
								printf("%s\n", packetHandler->getRxPacketError(dxl_error));
								break;
							}
							if (dxl_present_load1 >= load)
							{
								pos1_offset = pos1_ref - pos;
								std::cout << "pos1:" << pos << "\t" << "pos1_offset:" << pos1_offset << "\t" << std::endl;
								dxl1_state.store(5);
								break;
							}

							std::this_thread::sleep_for(std::chrono::milliseconds(10));
						}
						if (pos < DXL_MINIMUM_POSITION_VALUE)
						{
							dxl1_state.store(-7);
							std::cout << "dx1 homing failed:" << std::endl;
						}

					}
					//2号舵机找home//
					if (dxl2_exist && dxl2_state.load() > 0)
					{
						dxl2_state.store(4);
						int16_t pos2 = current_pos2.load()*SCALING;
						for (int pos = pos2; pos >= DXL_MINIMUM_POSITION_VALUE; pos -= step)
						{
							auto dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_GOAL_POSITION, pos, &dxl_error);
							if (dxl_error & ERRBIT_OVERLOAD)
							{
								std::cout << "overload" << std::endl;
								dxl2_state.store(-7);
								break;
							}
							auto dxl_load_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID2, ADDR_MX_PRESENT_LOAD, &dxl_present_load2, &dxl_error);
							if (dxl_load_result != COMM_SUCCESS)
							{
								printf("%s\n", packetHandler->getTxRxResult(dxl_load_result));
								break;
							}
							else if (dxl_error != 0)
							{
								printf("%s\n", packetHandler->getRxPacketError(dxl_error));
								break;
							}
							if (dxl_present_load2 >= load)
							{
								pos2_offset = pos2_ref - pos;
								std::cout << "pos2:" << pos << "\t" << "pos2_offset:" << pos2_offset << "\t" << std::endl;
								dxl2_state.store(5);
								break;
							}

							std::this_thread::sleep_for(std::chrono::milliseconds(10));
						}
						if (pos < DXL_MINIMUM_POSITION_VALUE)
						{
							dxl2_state.store(-7);
							std::cout << "dx2 homing failed:" << std::endl;
						}

					}
					//3号舵机找home//
					if (dxl3_exist && dxl3_state.load() > 0)
					{
						dxl3_state.store(4);
						int16_t pos3 = current_pos3.load()*SCALING;
						for (int pos = pos3; pos >= DXL_MINIMUM_POSITION_VALUE; pos -= step)
						{
							auto dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID3, ADDR_MX_GOAL_POSITION, pos, &dxl_error);
							if (dxl_error & ERRBIT_OVERLOAD)
							{
								std::cout << "overload" << std::endl;
								dxl3_state.store(-7);
								break;
							}
							auto dxl_load_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID3, ADDR_MX_PRESENT_LOAD, &dxl_present_load3, &dxl_error);
							if (dxl_load_result != COMM_SUCCESS)
							{
								printf("%s\n", packetHandler->getTxRxResult(dxl_load_result));
								break;
							}
							else if (dxl_error != 0)
							{
								printf("%s\n", packetHandler->getRxPacketError(dxl_error));
								break;
							}
							if (dxl_present_load3 >= load)
							{
								pos3_offset = pos3_ref - pos;
								std::cout << "pos3:" << pos << "\t" << "pos3_offset:" << pos3_offset << "\t" << std::endl;
								dxl3_state.store(5);
								break;
							}

							std::this_thread::sleep_for(std::chrono::milliseconds(10));
						}
						if (pos < DXL_MINIMUM_POSITION_VALUE)
						{
							dxl3_state.store(-7);
							std::cout << "dx3 homing failed:" << std::endl;
						}

					}

					dynamixel_control_mode.store(0);
					std::cout << "homeing mode end" << std::endl;
				}
				else if (dynamixel_control_mode.load() == 3)//emily mode
				{
					std::cout << "emily mode begin" << std::endl;
					std::unique_lock<std::mutex> run_lock(dynamixel_mutex);
					dxl_auto.store(true);//第一版
					int dxl_couter = 0;
					int data_length = std::max(std::max(dxl_pos[0].size(), dxl_pos[1].size()), dxl_pos[2].size());
					while (1)
					{
						if (syn_clock.load())//syn_clock为10ms计时标记位
						{
							syn_clock--;//10ms计时标记位     
							uint8_t param_goal_position[2];
							if (dxl_couter > data_length - 1)
							{
								dynamixel_control_mode.store(0);
								syn_clock.store(0);
								break;
							}
							//syncwrite function//
							auto start1 = std::chrono::system_clock::now();

							// Allocate goal position value into byte array
							if (dxl1_exist && dxl1_active)
							{
								dxl1_state.store(3);
								dxl_goal_position = std::int16_t(dxl_pos[0][dxl_couter]);
								target_pos1.store(dxl_goal_position);
								param_goal_position[0] = DXL_LOBYTE(dxl_goal_position);
								param_goal_position[1] = DXL_HIBYTE(dxl_goal_position);
								dxl_addparam_result = groupSyncWrite.addParam(DXL_ID1, param_goal_position);
								if (dxl_addparam_result != true)
								{
									dxl1_state.store(-5);
									fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID1);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
							}
							if (dxl2_exist && dxl2_active)
							{
								dxl2_state.store(3);
								dxl_goal_position = std::int16_t(dxl_pos[1][dxl_couter]);
								target_pos2.store(dxl_goal_position);
								param_goal_position[0] = DXL_LOBYTE(dxl_goal_position);
								param_goal_position[1] = DXL_HIBYTE(dxl_goal_position);
								dxl_addparam_result = groupSyncWrite.addParam(DXL_ID2, param_goal_position);
								if (dxl_addparam_result != true)
								{
									dxl2_state.store(-5);
									fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID2);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
							}
							if (dxl3_exist && dxl3_active)
							{
								dxl3_state.store(3);
								dxl_goal_position = std::int16_t(dxl_pos[2][dxl_couter]);
								target_pos3.store(dxl_goal_position);
								param_goal_position[0] = DXL_LOBYTE(dxl_goal_position);
								param_goal_position[1] = DXL_HIBYTE(dxl_goal_position);
								dxl_addparam_result = groupSyncWrite.addParam(DXL_ID3, param_goal_position);
								if (dxl_addparam_result != true)
								{
									dxl3_state.store(-5);
									fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL_ID3);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
							}
							// Syncwrite goal position
							auto dxl_comm_result = groupSyncWrite.txPacket();
							if (dxl_comm_result != COMM_SUCCESS)
							{
								dxl1_state.store(-5);
								dxl2_state.store(-5);
								dxl3_state.store(-5);
								printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
							}
							// Clear syncwrite parameter storage
							groupSyncWrite.clearParam();
							auto start2 = std::chrono::system_clock::now();

							
							//bulkread function//
							// Add parameter storage for Dynamixel#1,Dynamixel#2,Dynamixel#3 present position value
							if (dxl1_exist && dxl1_active)
							{
								//std::cout << "read1" <<std::endl;
								dxl_addparam_result = groupBulkRead.addParam(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								if (dxl_addparam_result != true)
								{
									dxl1_state.store(-6);
									fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID1);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
							}
							if (dxl2_exist && dxl2_active)
							{
								//std::cout << "read2" <<std::endl;
								dxl_addparam_result = groupBulkRead.addParam(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								if (dxl_addparam_result != true)
								{
									dxl2_state.store(-6);
									fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID2);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
							}
							if (dxl3_exist && dxl3_active)
							{
								//std::cout << "read3" <<std::endl;
								dxl_addparam_result = groupBulkRead.addParam(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								if (dxl_addparam_result != true)
								{
									dxl3_state.store(-6);
									fprintf(stderr, "[ID:%03d] grouBulkRead addparam failed", DXL_ID3);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
							}

							// Read Dynamixel#1,Dynamixel#2,Dynamixel#3 present position value
							dxl_comm_result = groupBulkRead.txRxPacket();
							if (dxl1_exist && dxl1_active)
							{
								dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								if (dxl_getdata_result != true)
								{
									dxl1_state.store(-6);
									fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID1);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
								// Get Dynamixel#1 present position value
								dxl_present_position1 = groupBulkRead.getData(DXL_ID1, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								auto dxl1 = std::int16_t(dxl_present_position1);
								current_pos1.store(1.0*dxl1 / SCALING);
							}
							if (dxl2_exist && dxl2_active)
							{
								dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								if (dxl_getdata_result != true)
								{
									dxl2_state.store(-6);
									fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID2);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
								// Get Dynamixel#2 present position value
								dxl_present_position2 = groupBulkRead.getData(DXL_ID2, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								auto dxl2 = std::int16_t(dxl_present_position2);
								current_pos2.store(1.0*dxl2 / SCALING);
							}
							if (dxl3_exist && dxl3_active)
							{
								dxl_getdata_result = groupBulkRead.isAvailable(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								if (dxl_getdata_result != true)
								{
									dxl3_state.store(-6);
									fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", DXL_ID3);
									dynamixel_control_mode.store(0);
									syn_clock.store(0);
									break;
								}
								// Get Dynamixel#3 present position value
								dxl_present_position3 = groupBulkRead.getData(DXL_ID3, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
								auto dxl3 = std::int16_t(dxl_present_position3);
								current_pos3.store(1.0*dxl3 / SCALING);
							}
							groupBulkRead.clearParam();
							
							/*
							auto end = std::chrono::system_clock::now();
							auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>(start2 - start1);
							auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(end - start2);
							std::cout <<  "syncwrite:" << "\t" <<double(duration1.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den   << "s"
								<< "\t" << "bulkread:"<< "\t" << double(duration2.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den   << "s"
														<< std::endl;
							*/

							dxl_couter = dxl_couter + dxl_timeinterval;//emily舵机位置指向变量，dxl_timeinterval为舵机时间系数，默认值10
						}
						if (syn_clock.load() == 0 && dxl_couter >= data_length - 10)//舵机执行到最后一行
						{
							dynamixel_control_mode.store(0);
							syn_clock.store(0);
							break;
						}
						std::this_thread::sleep_for(std::chrono::microseconds(500));
					}
					std::cout << "emily mode end" << std::endl;
				}
				
			}
			catch (std::exception &e)
			{
				dxl1_state.store(-9);
				dxl2_state.store(-9);
				dxl3_state.store(-9);
				std::cout << "Exception happens on dynamixel_control_mode:" << dynamixel_control_mode.load() << std::endl;
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
				goto close;
			}
			
			std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
		}

		// 关闭舵机端口，等待用户DEnable来进行重连 //
	close:
		dxl1_state.store(-8);
		dxl2_state.store(-8);
		dxl3_state.store(-8);
		dxl_connected.store(0);
		portHandler->closePort();
		std::cout << "Port is closing now." << std::endl;
		std::cout << "Press denable to reconnect again ...! or close the window." << std::endl;
		while (true)
		{
			std::this_thread::sleep_for(std::chrono::seconds(1));
			if (is_enabled.load() == 1)break;
		}
		goto start;
    });


    std::cout <<"new"<<std::endl;
    xmlpath = xmlpath / xmlfile;
    uixmlpath = uixmlpath / uixmlfile;
    modelxmlpath = modelxmlpath / modelxmlfile;
    std::cout<< xmlpath <<std::endl;
    auto&cs = aris::server::ControlServer::instance();
    auto port = argc < 2 ? 5866 : std::stoi(argv[1]);


    //生成kaanh.xml文档//
    /*
	//-------for qifan robot begin//
    cs.resetController(tuying::createControllerQifan().release());
    cs.resetModel(tuying::createModelQifan().release());
    cs.resetPlanRoot(tuying::createPlanRoot().release());
    cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
    cs.interfacePool().add<aris::server::WebInterface>("", "5867", aris::core::Socket::TCP);
    cs.interfacePool().add<aris::server::WebInterface>("", "5868", aris::core::Socket::TCP);
    cs.interfacePool().add<aris::server::WebInterface>("", "5869", aris::core::Socket::TCP);
    cs.resetSensorRoot(new aris::sensor::SensorRoot);
    //cs.model().loadXmlFile(modelxmlpath.string().c_str());
    cs.saveXmlFile(xmlpath.string().c_str());
	*/
    //-------for qifan robot end//

    cs.loadXmlFile(xmlpath.string().c_str());
    cs.resetPlanRoot(tuying::createPlanRoot().release());
    cs.saveXmlFile(xmlpath.string().c_str());

    cs.init();
	auto &cal = cs.model().calculator();
	kaanhconfig::createUserDataType(cal);
	kaanhconfig::createPauseTimeSpeed();
	//cs.start();

	//实时回调函数，每个实时周期调用一次//
	cs.setRtPlanPostCallback(kaanh::update_state);
	g_model = cs.model();

#ifdef WIN32
	for (auto &m : cs.controller().motionPool())
	{
		dynamic_cast<aris::control::EthercatMotor&>(m).setVirtual(true);
	}
#endif // WIN32


    //Start Web Socket//
    cs.open();

    //Receive Command//
    cs.runCmdLine();

    return 0;
}
