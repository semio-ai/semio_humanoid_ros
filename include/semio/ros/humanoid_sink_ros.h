#ifndef _SEMIO_ROS_HUMANOIDSINKROS_H_
#define _SEMIO_ROS_HUMANOIDSINKROS_H_

#include <semio/recognition/humanoid_sink.h>
#include <ros/node_handle.h>
#include <semio_msgs_ros/Humanoids.h>

namespace semio
{

namespace ros
{

class HumanoidSinkROS : public HumanoidSink
{
public:
    typedef semio_msgs_ros::Humanoids _HumanoidsMsg;
    typedef semio_msgs_ros::Humanoid _HumanoidMsg;
    typedef semio_msgs_ros::HumanoidJoint _HumanoidJointMsg;

protected:
    ::ros::NodeHandle _nh_rel;
    ::ros::Publisher _humanoids_pub;

public:
    HumanoidSinkROS( ::ros::NodeHandle const & nh_rel, std::string const & topic = "humanoids" );

    static _HumanoidJointMsg::Ptr toROSMsgPtr( HumanoidJoint const & joint );
    static _HumanoidJointMsg toROSMsg( HumanoidJoint const & joint );

    static _HumanoidMsg::Ptr toROSMsgPtr( Humanoid const & humanoid );
    static _HumanoidMsg toROSMsg( Humanoid const & humanoid );

    static _HumanoidsMsg::Ptr toROSMsgPtr( HumanoidArray const & humanoids );
    static _HumanoidsMsg toROSMsg( HumanoidArray const & humanoids );

protected:
    virtual void publishToSink( HumanoidArray const & humanoids );
};

}

}

#endif // _SEMIO_ROS_HUMANOIDSINKROS_H_
