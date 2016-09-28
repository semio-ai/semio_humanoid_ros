#ifndef _SEMIO_ROS_HUMANOIDSINKADAPTER_H_
#define _SEMIO_ROS_HUMANOIDSINKADAPTER_H_

#include <semio/recognition/humanoid_sink.h>
#include <ros/node_handle.h>
#include <string>

namespace semio
{

namespace ros
{

class HumanoidSinkAdapter
{
protected:
    ::ros::NodeHandle _nh_rel;
    std::string _default_sink;

public:
    HumanoidSinkAdapter( ::ros::NodeHandle const & nh_rel, std::string const & default_sink = "ros" );

    HumanoidSink::Ptr getHumanoidSink( std::string const & sink = "param" );
};

}

}
#endif // _SEMIO_ROS_HUMANOIDSINKADAPTER_H_
