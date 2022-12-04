// Generated by gencpp from file ff_msgs/ResetMap.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_RESETMAP_H
#define FF_MSGS_MESSAGE_RESETMAP_H

#include <ros/service_traits.h>


#include <ff_msgs/ResetMapRequest.h>
#include <ff_msgs/ResetMapResponse.h>


namespace ff_msgs
{

struct ResetMap
{

typedef ResetMapRequest Request;
typedef ResetMapResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ResetMap
} // namespace ff_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ff_msgs::ResetMap > {
  static const char* value()
  {
    return "a377c8d7c4f71636969846ebf44e4df2";
  }

  static const char* value(const ::ff_msgs::ResetMap&) { return value(); }
};

template<>
struct DataType< ::ff_msgs::ResetMap > {
  static const char* value()
  {
    return "ff_msgs/ResetMap";
  }

  static const char* value(const ::ff_msgs::ResetMap&) { return value(); }
};


// service_traits::MD5Sum< ::ff_msgs::ResetMapRequest> should match 
// service_traits::MD5Sum< ::ff_msgs::ResetMap > 
template<>
struct MD5Sum< ::ff_msgs::ResetMapRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::ResetMap >::value();
  }
  static const char* value(const ::ff_msgs::ResetMapRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::ResetMapRequest> should match 
// service_traits::DataType< ::ff_msgs::ResetMap > 
template<>
struct DataType< ::ff_msgs::ResetMapRequest>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::ResetMap >::value();
  }
  static const char* value(const ::ff_msgs::ResetMapRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ff_msgs::ResetMapResponse> should match 
// service_traits::MD5Sum< ::ff_msgs::ResetMap > 
template<>
struct MD5Sum< ::ff_msgs::ResetMapResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::ResetMap >::value();
  }
  static const char* value(const ::ff_msgs::ResetMapResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::ResetMapResponse> should match 
// service_traits::DataType< ::ff_msgs::ResetMap > 
template<>
struct DataType< ::ff_msgs::ResetMapResponse>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::ResetMap >::value();
  }
  static const char* value(const ::ff_msgs::ResetMapResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FF_MSGS_MESSAGE_RESETMAP_H