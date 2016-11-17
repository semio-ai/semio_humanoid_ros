#ifndef _SEMIO_ROS_HUMANOIDSOURCEROS_H_
#define _SEMIO_ROS_HUMANOIDSOURCEROS_H_

#include <semio/recognition/humanoid_source.h>
#include <ros/node_handle.h>
#include <semio_msgs_ros/Humanoids.h>

namespace semio
{

namespace ros
{

//! semio::HumanoidSource to source humanoids from ROS
/**
@par ROS Topics
<table>
<tr><th>Topic</th><th>Type</th><th>Description</th></tr>
<tr><td colspan="3" align="center">@b Subscriptions</td></tr>
<tr><td>variable; see @p topic param of @ref HumanoidSourceROS()</td><td>@ref semio_msgs_ros<a href=Humanoids_8msg_source.html><b>/Humanoids</b></a></td><td>#_HumanoidsMsg messages to convert to @c semio::HumanoidArray</td></tr>
</table>
*/
class HumanoidSourceROS : public HumanoidSource
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
    //! Subscriber for humanoid messages
    ::ros::Subscriber _humanoids_sub;
    //! ConstPtr to the last message received from ROS
    _HumanoidsMsg::ConstPtr _last_humanoids_msg;

public:
    /**
    @param nh_rel @copybrief _nh_rel
    @param topic ROS topic on which to look for humanoids
    */
    HumanoidSourceROS( ::ros::NodeHandle const & nh_rel, std::string const & topic = "humanoids" );

    //! Create a semio::HumanoidJoint from a humanoid joint message
    /**
    @param msg ConstPtr to the humanoid joint message
    @return A semio::HumanoidJoint initialized from the data in @p msg
    */
    static HumanoidJoint fromROSMsg( _HumanoidJointMsg const & msg );
    //! Create a semio::Humanoid from a humanoid message
    /**
    @param msg ConstPtr to the humanoid message
    @return A semio::Humanoid initialized from the data in @p msg
    */
    static Humanoid fromROSMsg( _HumanoidMsg const & msg );
    //! Create a semio::HumanoidArray from a humanoids message
    /**
    @param msg ConstPtr to the humanoids message
    @return A semio::HumanoidArray initialized from the data in @p msg
    */
    static HumanoidArray fromROSMsg( _HumanoidsMsg const & msg );

protected:
    //! Convert #_last_humanoids_msg to a semio::HumanoidArray
    /**
    @return #_last_humanoids_msg converted to a semio::HumanoidArray
    @post #_last_humanoids_msg is released via _HumanoidsMsg::ConstPtr::reset()
    @note Result will be empty if #_last_humanoids_msg is undefined (no messages have been received since last updateFromSource())
    */
    HumanoidArray updateFromSource();

    //! ROS callback for humanoids messages
    /**
    @param msg ConstPtr to the humanoids message
    @post #_last_humanoids_msg shares ownership of message data with @p msg
    */
    void humanoidsCB( _HumanoidsMsg::ConstPtr const & msg );
};

}

}

#endif // _SEMIO_ROS_HUMANOIDSOURCEROS_H_
