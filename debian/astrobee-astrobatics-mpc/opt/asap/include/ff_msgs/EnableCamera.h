// Generated by gencpp from file ff_msgs/EnableCamera.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_ENABLECAMERA_H
#define FF_MSGS_MESSAGE_ENABLECAMERA_H

#include <ros/service_traits.h>


#include <ff_msgs/EnableCameraRequest.h>
#include <ff_msgs/EnableCameraResponse.h>


namespace ff_msgs
{

struct EnableCamera
{

typedef EnableCameraRequest Request;
typedef EnableCameraResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EnableCamera
} // namespace ff_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ff_msgs::EnableCamera > {
  static const char* value()
  {
    return "4180836a5b8bc96980a6bee8edb99cea";
  }

  static const char* value(const ::ff_msgs::EnableCamera&) { return value(); }
};

template<>
struct DataType< ::ff_msgs::EnableCamera > {
  static const char* value()
  {
    return "ff_msgs/EnableCamera";
  }

  static const char* value(const ::ff_msgs::EnableCamera&) { return value(); }
};


// service_traits::MD5Sum< ::ff_msgs::EnableCameraRequest> should match 
// service_traits::MD5Sum< ::ff_msgs::EnableCamera > 
template<>
struct MD5Sum< ::ff_msgs::EnableCameraRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::EnableCamera >::value();
  }
  static const char* value(const ::ff_msgs::EnableCameraRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::EnableCameraRequest> should match 
// service_traits::DataType< ::ff_msgs::EnableCamera > 
template<>
struct DataType< ::ff_msgs::EnableCameraRequest>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::EnableCamera >::value();
  }
  static const char* value(const ::ff_msgs::EnableCameraRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ff_msgs::EnableCameraResponse> should match 
// service_traits::MD5Sum< ::ff_msgs::EnableCamera > 
template<>
struct MD5Sum< ::ff_msgs::EnableCameraResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::EnableCamera >::value();
  }
  static const char* value(const ::ff_msgs::EnableCameraResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::EnableCameraResponse> should match 
// service_traits::DataType< ::ff_msgs::EnableCamera > 
template<>
struct DataType< ::ff_msgs::EnableCameraResponse>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::EnableCamera >::value();
  }
  static const char* value(const ::ff_msgs::EnableCameraResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FF_MSGS_MESSAGE_ENABLECAMERA_H