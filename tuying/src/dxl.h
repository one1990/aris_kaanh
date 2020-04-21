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
#define ERRBIT_OVERLOAD         32							// The current load cannot be controlled by the set torque.
#define ESC_ASCII_VALUE                 0x1b
#define SCALING                         11.378


namespace dnmxl
{
	class dxl
	{
	public:
		auto status()->bool { return dxl_state; };
		auto set_status(int statuscode)->void { dxl_state = statuscode };
		auto enable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool;
		auto disable_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate)->bool;
		auto read_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, uint16_t &dxl_present_position)->bool;
		auto write_dynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int dxl_comm_result, const int dxl_id, const int baudrate, int target_pos)->bool;
		auto groupBulkread_dynamixel(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead groupBulkRead) -> bool;
	private:
		uint8_t dxl_id;
		int baudtate;
		int dxl_state;
		bool is_existing;
		bool is_active;
	};

	class dxlcontroller
	{
	public:
		auto groupBulkRead_dynamixel(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead groupBulkRead) -> bool;
		auto groupSyncWrite_dynamixel(dynamixel::PacketHandler *packetHandler, dynamixel::GroupBulkRead groupBulkRead) -> bool;
	private:
		std::string devicename;
		float protocolversion;
		dynamixel::PortHandler *portHandler;
		dynamixel::PacketHandler *packetHandler;
		dynamixel::GroupSyncWrite groupSyncWrite;
		dynamixel::GroupBulkRead groupBulkRead;
	};

}

#endif
