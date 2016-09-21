#ifndef _SEMIO_ROS_HUMANOIDSOURCEADAPTER_H_
#define _SEMIO_ROS_HUMANOIDSOURCEADAPTER_H_

#include <semio/recognition/humanoid_source.h>
#include <ros/node_handle.h>
#include <string>

namespace semio
{

namespace ros
{

class HumanoidSourceAdapter
{
protected:
    ::ros::NodeHandle _nh_rel;
    std::string _default_source;

public:
    HumanoidSourceAdapter( ::ros::NodeHandle const & nh_rel, std::string const & default_source = "fullbody" );

    HumanoidSource::Ptr getHumanoidSource( std::string const & source = "param" );
};

}

}
#endif // _SEMIO_ROS_HUMANOIDSOURCEADAPTER_H_
