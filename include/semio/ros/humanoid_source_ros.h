#ifndef _SEMIO_ROS_HUMANOIDSOURCEROS_H_
#define _SEMIO_ROS_HUMANOIDSOURCEROS_H_

#include <semio/recognition/humanoid_source.h>
#include <ros/node_handle.h>
#include <semio_msgs_ros/Humanoids.h>

namespace semio
{

namespace ros
{

class HumanoidSourceROS : public HumanoidSource
{
public:
    typedef semio_msgs_ros::Humanoids _HumanoidsMsg;
    typedef semio_msgs_ros::Humanoid _HumanoidMsg;
    typedef semio_msgs_ros::HumanoidJoint _HumanoidJointMsg;

protected:
    ::ros::NodeHandle _nh_rel;
    ::ros::Subscriber _humanoids_sub;
    _HumanoidsMsg::ConstPtr _last_humanoids_msg;

public:
    HumanoidSourceROS( ::ros::NodeHandle const & nh_rel );

protected:
    HumanoidArray updateFromSource();

    void humanoidsCB( _HumanoidsMsg::ConstPtr const & msg );
};

}

}

#endif // _SEMIO_ROS_HUMANOIDSOURCEROS_H_
