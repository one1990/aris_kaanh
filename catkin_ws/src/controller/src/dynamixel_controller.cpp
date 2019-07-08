#include "ros/ros.h"
#include "controller/interface.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include "controller/pose.h"
#include "controller/motorpos.h"
#include "controller/setpos.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "stdio.h"
#include "unistd.h"
#include <sensor_msgs/JointState.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

// Baud rate 57142
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

#define SCALING                         11
// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

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

bool exector(controller::setpos::Request  &req,
  controller::setpos::Response &res)
{
  //ROS_INFO("receive request from client: cmd=%s", req.cmd.c_str());
  //char * end;
  //int target_pos = static_cast<int>(strtol(req.cmd.c_str(),&end,10));
  int target_pos = req.cmd;
  if(target_pos <= 0)
  {
    target_pos = 0;
  }
  else if(target_pos>358)
  {
    target_pos = 359;
  }
  uint8_t dxl_error = 0;  
  ROS_INFO("target_pos is %i", target_pos);
  int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, target_pos*SCALING, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  res.return_code=1;
  ROS_INFO("sending back response to client: [%ld]", (long int)res.return_code);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_Controller");

  ros::NodeHandle n;

  ros::Rate loop_rate(2); //2hz 
  ros::Publisher chatter_pub = n.advertise<controller::motorpos>("motorpos_topic", 1000);
  ros::ServiceServer service = n.advertiseService("gettargetpos", exector);
  // return 0;


  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    ROS_INFO("Succeeded to open the port!\n");
  }
  else
  {
    ROS_ERROR("Failed to open the port!\n");
    ROS_ERROR("Failed to open the port!\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    ROS_INFO("Succeeded to change the baudrate!\n");
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!\n");
    ROS_ERROR("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_INFO("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
  }
  else
  {
    ROS_INFO("Dynamixel has been successfully connected \n");
  }

  while(ros::ok())
  {
    

    // ROS_INFO("Press any key to continue! (or press ESC to quit!)\n");
    // if (getch() == ESC_ASCII_VALUE)
    //   break;

    // // Write goal position
    // dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index], &dxl_error);
    // if (dxl_comm_result != COMM_SUCCESS)
    // {
    //   ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    // }
    // else if (dxl_error != 0)
    // {
    //   ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
    // }
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    
    // do
    // {
    //   // Read present position
    //   dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION, &dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      ROS_ERROR("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      ROS_ERROR("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    //   ROS_INFO("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position[index], dxl_present_position);

    // }while((abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    // // Change goal position
    // if (index == 0)
    // {
    //   index = 1;
    // }
    // else
    // {
    //   index = 0;
    // }
    controller::motorpos msg;
    msg.angular = dxl_present_position/SCALING; //dxl_present_position [0-4096]
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  {
    ROS_INFO("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  }
  else if (dxl_error != 0)
  {
    ROS_INFO("%s\n", packetHandler->getRxPacketError(dxl_error));
  }

  // Close port
  portHandler->closePort();

  return 0;
}