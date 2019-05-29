// Generated by gencpp from file controller/pose.msg
// DO NOT EDIT!


#ifndef CONTROLLER_MESSAGE_POSE_H
#define CONTROLLER_MESSAGE_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace controller
{
template <class ContainerAllocator>
struct pose_
{
  typedef pose_<ContainerAllocator> Type;

  pose_()
    : pose()  {
    }
  pose_(const ContainerAllocator& _alloc)
    : pose(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::controller::pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::controller::pose_<ContainerAllocator> const> ConstPtr;

}; // struct pose_

typedef ::controller::pose_<std::allocator<void> > pose;

typedef boost::shared_ptr< ::controller::pose > posePtr;
typedef boost::shared_ptr< ::controller::pose const> poseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::controller::pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::controller::pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'controller': ['/root/catkin_ws/src/controller/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::controller::pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::controller::pose_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::controller::pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::controller::pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::controller::pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::controller::pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::controller::pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0cbc29f91adcb71214b885e1c0073aa7";
  }

  static const char* value(const ::controller::pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0cbc29f91adcb712ULL;
  static const uint64_t static_value2 = 0x14b885e1c0073aa7ULL;
};

template<class ContainerAllocator>
struct DataType< ::controller::pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "controller/pose";
  }

  static const char* value(const ::controller::pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::controller::pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string pose\n\
";
  }

  static const char* value(const ::controller::pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::controller::pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::controller::pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::controller::pose_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROLLER_MESSAGE_POSE_H
