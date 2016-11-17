#ifndef _SEMIO_ROS_HUMANOIDSINKROS_H_
#define _SEMIO_ROS_HUMANOIDSINKROS_H_

#include <semio/recognition/humanoid_sink.h>
#include <ros/node_handle.h>
#include <semio_msgs_ros/Humanoids.h>

namespace semio
{

namespace ros
{

//! semio::HumanoidSink to sink humanoids to ROS
/**
@par ROS Topics
<table>
<tr><th>Topic</th><th>Type</th><th>Description</th></tr>
<tr><td colspan="3" align="center">@b Publications</td></tr>
<tr><td>variable; see @p topic param of @ref HumanoidSinkROS()</td><td>@ref semio_msgs_ros<a href=Humanoids_8msg_source.html><b>/Humanoids</b></a></td><td>#_HumanoidsMsg messages to convert to @c semio::HumanoidArray</td></tr>
</table>
*/
class HumanoidSinkROS : public HumanoidSink
{
public:
    //! ROS message for a vector of humanoids
    typedef semio_msgs_ros::Humanoids _HumanoidsMsg;
    //! ROS message for a single humanoid
    typedef semio_msgs_ros::Humanoid _HumanoidMsg;
    //! ROS message for a humanoid joint
    typedef semio_msgs_ros::HumanoidJoint _HumanoidJointMsg;

protected:
    //! NodeHandle copy for interfacing with ROS
    ::ros::NodeHandle _nh_rel;
    //! Publisher for humanoid messages
    ::ros::Publisher _humanoids_pub;

public:
    /**
    @param nh_rel @copybrief _nh_rel
    @param topic ROS topic on which to publish humanoids
    */
    HumanoidSinkROS( ::ros::NodeHandle const & nh_rel, std::string const & topic = "humanoids" );

    //! Create a #_HumanoidJointMsg\::Ptr from a semio::HumanoidJoint
    /**
    @param joint The joint to use
    @return A #_HumanoidJointMsg\::Ptr initialized from the data in @p joint
    */
    static _HumanoidJointMsg::Ptr toROSMsgPtr( HumanoidJoint const & joint );
    //! Create a #_HumanoidJointMsg from a semio::HumanoidJoint
    /**
    @param joint The joint to use
    @return A #_HumanoidJointMsg initialized from the data in @p joint
    */
    static _HumanoidJointMsg toROSMsg( HumanoidJoint const & joint );

    //! Create a #_HumanoidMsg\::Ptr from a semio::Humanoid
    /**
    @param humanoid The humanoid to use
    @return A #_HumanoidMsg\::Ptr initialized from the data in @p humanoid
    */
    static _HumanoidMsg::Ptr toROSMsgPtr( Humanoid const & humanoid );
    //! Create a #_HumanoidMsg from a semio::Humanoid
    /**
    @param humanoid The humanoid to use
    @return A #_HumanoidMsg initialized from the data in @p humanoid
    */
    static _HumanoidMsg toROSMsg( Humanoid const & humanoid );

    //! Create a #_HumanoidsMsg\::Ptr from a semio::Humanoids
    /**
    @param humanoids The humanoid array to use
    @return A #_HumanoidsMsg\::Ptr initialized from the data in @p humanoids
    */
    static _HumanoidsMsg::Ptr toROSMsgPtr( HumanoidArray const & humanoids );
    //! Create a #_HumanoidsMsg from a semio::Humanoids
    /**
    @param humanoids The humanoid array to use
    @return A #_HumanoidsMsg initialized from the data in @p humanoids
    */
    static _HumanoidsMsg toROSMsg( HumanoidArray const & humanoids );

protected:
    //! Convert @p humanoids to a ROS message and publish it to ROS
    /**
    @param humanoids The semio::HumanoidArray to convert and publish to ROS
    */
    virtual void publishToSink( HumanoidArray const & humanoids );
};

}

}

#endif // _SEMIO_ROS_HUMANOIDSINKROS_H_
