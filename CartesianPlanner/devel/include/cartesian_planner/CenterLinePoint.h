// Generated by gencpp from file cartesian_planner/CenterLinePoint.msg
// DO NOT EDIT!


#ifndef CARTESIAN_PLANNER_MESSAGE_CENTERLINEPOINT_H
#define CARTESIAN_PLANNER_MESSAGE_CENTERLINEPOINT_H


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
struct CenterLinePoint_
{
  typedef CenterLinePoint_<ContainerAllocator> Type;

  CenterLinePoint_()
    : s(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , kappa(0.0)
    , left_bound(0.0)
    , right_bound(0.0)  {
    }
  CenterLinePoint_(const ContainerAllocator& _alloc)
    : s(0.0)
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , kappa(0.0)
    , left_bound(0.0)
    , right_bound(0.0)  {
  (void)_alloc;
    }



   typedef double _s_type;
  _s_type s;

   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _theta_type;
  _theta_type theta;

   typedef double _kappa_type;
  _kappa_type kappa;

   typedef double _left_bound_type;
  _left_bound_type left_bound;

   typedef double _right_bound_type;
  _right_bound_type right_bound;





  typedef boost::shared_ptr< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> const> ConstPtr;

}; // struct CenterLinePoint_

typedef ::cartesian_planner::CenterLinePoint_<std::allocator<void> > CenterLinePoint;

typedef boost::shared_ptr< ::cartesian_planner::CenterLinePoint > CenterLinePointPtr;
typedef boost::shared_ptr< ::cartesian_planner::CenterLinePoint const> CenterLinePointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cartesian_planner::CenterLinePoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cartesian_planner::CenterLinePoint_<ContainerAllocator1> & lhs, const ::cartesian_planner::CenterLinePoint_<ContainerAllocator2> & rhs)
{
  return lhs.s == rhs.s &&
    lhs.x == rhs.x &&
    lhs.y == rhs.y &&
    lhs.theta == rhs.theta &&
    lhs.kappa == rhs.kappa &&
    lhs.left_bound == rhs.left_bound &&
    lhs.right_bound == rhs.right_bound;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cartesian_planner::CenterLinePoint_<ContainerAllocator1> & lhs, const ::cartesian_planner::CenterLinePoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cartesian_planner

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8c676686a759a5bfb0f50165bdf3705";
  }

  static const char* value(const ::cartesian_planner::CenterLinePoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8c676686a759a5bULL;
  static const uint64_t static_value2 = 0xfb0f50165bdf3705ULL;
};

template<class ContainerAllocator>
struct DataType< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cartesian_planner/CenterLinePoint";
  }

  static const char* value(const ::cartesian_planner::CenterLinePoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 s\n"
"float64 x\n"
"float64 y\n"
"float64 theta\n"
"float64 kappa\n"
"float64 left_bound\n"
"float64 right_bound\n"
;
  }

  static const char* value(const ::cartesian_planner::CenterLinePoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.s);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
      stream.next(m.kappa);
      stream.next(m.left_bound);
      stream.next(m.right_bound);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CenterLinePoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cartesian_planner::CenterLinePoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cartesian_planner::CenterLinePoint_<ContainerAllocator>& v)
  {
    s << indent << "s: ";
    Printer<double>::stream(s, indent + "  ", v.s);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
    s << indent << "kappa: ";
    Printer<double>::stream(s, indent + "  ", v.kappa);
    s << indent << "left_bound: ";
    Printer<double>::stream(s, indent + "  ", v.left_bound);
    s << indent << "right_bound: ";
    Printer<double>::stream(s, indent + "  ", v.right_bound);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CARTESIAN_PLANNER_MESSAGE_CENTERLINEPOINT_H