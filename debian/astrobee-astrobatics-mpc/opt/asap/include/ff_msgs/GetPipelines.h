// Generated by gencpp from file ff_msgs/GetPipelines.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_GETPIPELINES_H
#define FF_MSGS_MESSAGE_GETPIPELINES_H

#include <ros/service_traits.h>


#include <ff_msgs/GetPipelinesRequest.h>
#include <ff_msgs/GetPipelinesResponse.h>


namespace ff_msgs
{

struct GetPipelines
{

typedef GetPipelinesRequest Request;
typedef GetPipelinesResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetPipelines
} // namespace ff_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ff_msgs::GetPipelines > {
  static const char* value()
  {
    return "4fb31d141d0f152e9301905ffcaa8f48";
  }

  static const char* value(const ::ff_msgs::GetPipelines&) { return value(); }
};

template<>
struct DataType< ::ff_msgs::GetPipelines > {
  static const char* value()
  {
    return "ff_msgs/GetPipelines";
  }

  static const char* value(const ::ff_msgs::GetPipelines&) { return value(); }
};


// service_traits::MD5Sum< ::ff_msgs::GetPipelinesRequest> should match 
// service_traits::MD5Sum< ::ff_msgs::GetPipelines > 
template<>
struct MD5Sum< ::ff_msgs::GetPipelinesRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::GetPipelines >::value();
  }
  static const char* value(const ::ff_msgs::GetPipelinesRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::GetPipelinesRequest> should match 
// service_traits::DataType< ::ff_msgs::GetPipelines > 
template<>
struct DataType< ::ff_msgs::GetPipelinesRequest>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::GetPipelines >::value();
  }
  static const char* value(const ::ff_msgs::GetPipelinesRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ff_msgs::GetPipelinesResponse> should match 
// service_traits::MD5Sum< ::ff_msgs::GetPipelines > 
template<>
struct MD5Sum< ::ff_msgs::GetPipelinesResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::GetPipelines >::value();
  }
  static const char* value(const ::ff_msgs::GetPipelinesResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::GetPipelinesResponse> should match 
// service_traits::DataType< ::ff_msgs::GetPipelines > 
template<>
struct DataType< ::ff_msgs::GetPipelinesResponse>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::GetPipelines >::value();
  }
  static const char* value(const ::ff_msgs::GetPipelinesResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FF_MSGS_MESSAGE_GETPIPELINES_H