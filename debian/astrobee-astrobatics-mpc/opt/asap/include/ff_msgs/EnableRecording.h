// Generated by gencpp from file ff_msgs/EnableRecording.msg
// DO NOT EDIT!


#ifndef FF_MSGS_MESSAGE_ENABLERECORDING_H
#define FF_MSGS_MESSAGE_ENABLERECORDING_H

#include <ros/service_traits.h>


#include <ff_msgs/EnableRecordingRequest.h>
#include <ff_msgs/EnableRecordingResponse.h>


namespace ff_msgs
{

struct EnableRecording
{

typedef EnableRecordingRequest Request;
typedef EnableRecordingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct EnableRecording
} // namespace ff_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::ff_msgs::EnableRecording > {
  static const char* value()
  {
    return "68fd83cd501355d809d1d0420334c998";
  }

  static const char* value(const ::ff_msgs::EnableRecording&) { return value(); }
};

template<>
struct DataType< ::ff_msgs::EnableRecording > {
  static const char* value()
  {
    return "ff_msgs/EnableRecording";
  }

  static const char* value(const ::ff_msgs::EnableRecording&) { return value(); }
};


// service_traits::MD5Sum< ::ff_msgs::EnableRecordingRequest> should match 
// service_traits::MD5Sum< ::ff_msgs::EnableRecording > 
template<>
struct MD5Sum< ::ff_msgs::EnableRecordingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::EnableRecording >::value();
  }
  static const char* value(const ::ff_msgs::EnableRecordingRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::EnableRecordingRequest> should match 
// service_traits::DataType< ::ff_msgs::EnableRecording > 
template<>
struct DataType< ::ff_msgs::EnableRecordingRequest>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::EnableRecording >::value();
  }
  static const char* value(const ::ff_msgs::EnableRecordingRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::ff_msgs::EnableRecordingResponse> should match 
// service_traits::MD5Sum< ::ff_msgs::EnableRecording > 
template<>
struct MD5Sum< ::ff_msgs::EnableRecordingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::ff_msgs::EnableRecording >::value();
  }
  static const char* value(const ::ff_msgs::EnableRecordingResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::ff_msgs::EnableRecordingResponse> should match 
// service_traits::DataType< ::ff_msgs::EnableRecording > 
template<>
struct DataType< ::ff_msgs::EnableRecordingResponse>
{
  static const char* value()
  {
    return DataType< ::ff_msgs::EnableRecording >::value();
  }
  static const char* value(const ::ff_msgs::EnableRecordingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FF_MSGS_MESSAGE_ENABLERECORDING_H