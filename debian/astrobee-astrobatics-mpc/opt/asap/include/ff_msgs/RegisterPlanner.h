// Generated by gencpp from file ff_msgs/RegisterPlanner.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_REGISTERPLANNER_H
#define FF_MSGS_MESSAGE_REGISTERPLANNER_H

#include <ros/service_traits.h>


#include <ff_msgs/RegisterPlannerRequest.h>
#include <ff_msgs/RegisterPlannerResponse.h>


namespace ff_msgs
{

struct RegisterPlanner
{

typedef RegisterPlannerRequest Request;
typedef RegisterPlannerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RegisterPlanner
} // namespace ff_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ff_msgs::RegisterPlanner > {
  static const char* value()
  {
    return "e247f0a3c6e3085865c44afa8ad187df";
  }

  static const char* value(const ::ff_msgs::RegisterPlanner&) { return value(); }
};

template<>
struct DataType< ::ff_msgs::RegisterPlanner > {
  static const char* value()
  {
    return "ff_msgs/RegisterPlanner";
  }

  static const char* value(const ::ff_msgs::RegisterPlanner&) { return value(); }
};


// service_traits::MD5Sum< ::ff_msgs::RegisterPlannerRequest> should match 
// service_traits::MD5Sum< ::ff_msgs::RegisterPlanner > 
template<>
struct MD5Sum< ::ff_msgs::RegisterPlannerRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::RegisterPlanner >::value();
  }
  static const char* value(const ::ff_msgs::RegisterPlannerRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::RegisterPlannerRequest> should match 
// service_traits::DataType< ::ff_msgs::RegisterPlanner > 
template<>
struct DataType< ::ff_msgs::RegisterPlannerRequest>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::RegisterPlanner >::value();
  }
  static const char* value(const ::ff_msgs::RegisterPlannerRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ff_msgs::RegisterPlannerResponse> should match 
// service_traits::MD5Sum< ::ff_msgs::RegisterPlanner > 
template<>
struct MD5Sum< ::ff_msgs::RegisterPlannerResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::RegisterPlanner >::value();
  }
  static const char* value(const ::ff_msgs::RegisterPlannerResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::RegisterPlannerResponse> should match 
// service_traits::DataType< ::ff_msgs::RegisterPlanner > 
template<>
struct DataType< ::ff_msgs::RegisterPlannerResponse>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::RegisterPlanner >::value();
  }
  static const char* value(const ::ff_msgs::RegisterPlannerResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FF_MSGS_MESSAGE_REGISTERPLANNER_H