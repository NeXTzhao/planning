// Generated by gencpp from file cartesian_planner/DynamicTrajectoryPoint.msg
// DO NOT EDIT!


#ifndef CARTESIAN_PLANNER_MESSAGE_DYNAMICTRAJECTORYPOINT_H
#define CARTESIAN_PLANNER_MESSAGE_DYNAMICTRAJECTORYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cartesian_planner
{
template <class ContainerAllocator>
struct DynamicTrajectoryPoint_
{
  typedef DynamicTrajectoryPoint_<ContainerAllocator> Type;

  DynamicTrajectoryPoint_()
    : time(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  DynamicTrajectoryPoint_(const ContainerAllocator& _alloc)
    : time(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)  {
  (void)_alloc;
    }



   typedef double _time_type;
  _time_type time;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _theta_type;
  _theta_type theta;





  typedef boost::shared_ptr< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> const> ConstPtr;

}; // struct DynamicTrajectoryPoint_

typedef ::cartesian_planner::DynamicTrajectoryPoint_<std::allocator<void> > DynamicTrajectoryPoint;

typedef boost::shared_ptr< ::cartesian_planner::DynamicTrajectoryPoint > DynamicTrajectoryPointPtr;
typedef boost::shared_ptr< ::cartesian_planner::DynamicTrajectoryPoint const> DynamicTrajectoryPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator1> & lhs, const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return lhs.time == rhs.time &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator1> & lhs, const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cartesian_planner

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3500997421bb06e40b442d83f83150dc";
  }

  static const char* value(const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3500997421bb06e4ULL;
  static const uint64_t static_value2 = 0x0b442d83f83150dcULL;
};

template<class ContainerAllocator>
struct DataType< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartesian_planner/DynamicTrajectoryPoint";
  }

  static const char* value(const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 time\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
;
  }

  static const char* value(const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.time);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DynamicTrajectoryPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartesian_planner::DynamicTrajectoryPoint_<ContainerAllocator>& v)
  {
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTESIAN_PLANNER_MESSAGE_DYNAMICTRAJECTORYPOINT_H