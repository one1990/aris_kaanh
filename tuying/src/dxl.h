#ifndef DXL_H_
#define DXL_H_

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"  // Uses Dynamixel SDK library

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

#ifdef UNIX
#define DEVICENAME                  "/dev/ttyUSB0"			// Check which port is being used on your controller
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
#define ERRBIT_OVERLOAD					32					// The current load cannot be controlled by the set torque.
#define ESC_ASCII_VALUE                 0x1b
#define SCALING                         11.378


namespace dnmxl
{
	class dxl
	{
	public:
		auto get_id()->uint8_t { return dxl_id; };
		auto set_id(uint8_t id)->void { dxl_id = id; };
		auto get_baudrate()->uint8_t { return baudrate; };
		auto set_baudrate(uint8_t brt)->void { baudrate = brt; };
		auto get_status()->bool { return dxl_state; };
		auto set_status(int statuscode)->void { dxl_state = statuscode; };
		auto get_exist()->uint8_t { return is_existing; };
		auto set_exist(uint8_t ie)->void { is_existing = ie; };
		auto get_active()->uint8_t { return is_active; };
		auto set_active(uint8_t iat)->void { is_active = iat; };
		auto get_comm_result()->uint8_t { return dxl_comm_result; };
		auto set_comm_result(uint8_t dcr)->void { dxl_comm_result = dcr; };
		auto get_error()->uint8_t { return dxl_error; };
		auto set_error(uint8_t err)->void { dxl_error = err; };
		auto get_addparam_result()->uint8_t { return dxl_addparam_result; };
		auto set_addparam_result(uint8_t addpr)->void { dxl_addparam_result = addpr; };
		auto get_getdata_result()->uint8_t { return dxl_getdata_result; };
		auto set_getdata_result(uint8_t id)->void { dxl_getdata_result = id; };
		auto get_present_load()->uint8_t { return dxl_present_load; };
		auto set_present_load(uint8_t pl)->void { dxl_present_load = pl; };
		auto get_present_position()->uint8_t { return dxl_present_position; };
		auto set_present_position(uint8_t pp)->void { dxl_present_position = pp; };
		auto get_goal_position()->uint8_t { return dxl_goal_position; };
		auto set_goal_position(uint8_t gp)->void { dxl_goal_position = gp; };

		auto enable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool;
		auto disable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool;
		auto read_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, uint16_t &dxl_present_position)->bool;
		auto write_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, int target_pos)->bool;
	private:
		uint8_t dxl_id;					//dynamxiel identification
		int baudrate;					//dynamxiel baudrate
		int dxl_state;					//dynamxiel state
		bool is_existing;				//value is true when dynamixel is existing
		bool is_active;					//value is true when the dynamixel data in emily is set true
		int dxl_comm_result;			//dynamixel communication result
		uint8_t dxl_error;				//dynamixel error
		bool dxl_addparam_result;		//addParam result for groupBulkRead
		bool dxl_getdata_result;		//getParam result for groupBulkRead
		uint16_t dxl_present_load;		//dynamixel present load
		uint16_t dxl_present_position;	//dynamixel present position
		uint16_t dxl_goal_position;		//dynamixel goal position
	};

	class dxlcontroller
	{
	public:
		auto groupBulkRead_dynamixel(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead groupBulkRead) -> bool;	//the function of groupBulkRead
		auto groupSyncWrite_dynamixel(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead groupBulkRead) -> bool;	//the function of groupSyncWrite
	private:
		std::string devicename;						//device name
		float protocolversion;						//protocol version of dynamixel
		dynamixel::PortHandler *portHandler;		//the handler of port
		dynamixel::PacketHandler *packetHandler;	//the handler of packet
		dynamixel::GroupSyncWrite groupSyncWrite;	//groupSyncWrite instance
		dynamixel::GroupBulkRead groupBulkRead;		//groupBulkRead instance
		std::vector<dxl>dxlpool;					//dynamixel pool, the size represent the number of dynamixel
		int dxl_timeinterval = 10;					//emily data time interval
	};
}

#endif
