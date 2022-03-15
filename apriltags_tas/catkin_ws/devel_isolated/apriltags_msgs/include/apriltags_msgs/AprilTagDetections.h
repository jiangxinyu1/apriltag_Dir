// Generated by gencpp from file apriltags_msgs/AprilTagDetections.msg
// DO NOT EDIT!


#ifndef APRILTAGS_MSGS_MESSAGE_APRILTAGDETECTIONS_H
#define APRILTAGS_MSGS_MESSAGE_APRILTAGDETECTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <apriltags_msgs/AprilTag.h>

namespace apriltags_msgs
{
template <class ContainerAllocator>
struct AprilTagDetections_
{
  typedef AprilTagDetections_<ContainerAllocator> Type;

  AprilTagDetections_()
    : header()
    , detections()  {
    }
  AprilTagDetections_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , detections(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::apriltags_msgs::AprilTag_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::apriltags_msgs::AprilTag_<ContainerAllocator> >::other >  _detections_type;
  _detections_type detections;





  typedef boost::shared_ptr< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> const> ConstPtr;

}; // struct AprilTagDetections_

typedef ::apriltags_msgs::AprilTagDetections_<std::allocator<void> > AprilTagDetections;

typedef boost::shared_ptr< ::apriltags_msgs::AprilTagDetections > AprilTagDetectionsPtr;
typedef boost::shared_ptr< ::apriltags_msgs::AprilTagDetections const> AprilTagDetectionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator1> & lhs, const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.detections == rhs.detections;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator1> & lhs, const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace apriltags_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d4bc35daf77e797dc95ce1f2fcc1ea5d";
  }

  static const char* value(const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd4bc35daf77e797dULL;
  static const uint64_t static_value2 = 0xc95ce1f2fcc1ea5dULL;
};

template<class ContainerAllocator>
struct DataType< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "apriltags_msgs/AprilTagDetections";
  }

  static const char* value(const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"AprilTag[] detections\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: apriltags_msgs/AprilTag\n"
"string id\n"
"geometry_msgs/Point[] corners_px\n"
"geometry_msgs/Pose pose_3d\n"
"bool pose_valid\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.detections);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AprilTagDetections_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::apriltags_msgs::AprilTagDetections_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::apriltags_msgs::AprilTagDetections_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "detections[]" << std::endl;
    for (size_t i = 0; i < v.detections.size(); ++i)
    {
      s << indent << "  detections[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::apriltags_msgs::AprilTag_<ContainerAllocator> >::stream(s, indent + "    ", v.detections[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // APRILTAGS_MSGS_MESSAGE_APRILTAGDETECTIONS_H