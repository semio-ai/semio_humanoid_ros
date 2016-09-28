#include <semio/ros/humanoid_sink_ros.h>

semio::ros::HumanoidSinkROS::HumanoidSinkROS( ::ros::NodeHandle const & nh_rel )
:
    _nh_rel( nh_rel ),
    _humanoids_pub( _nh_rel.advertise<_HumanoidsMsg>( "humanoids", 10 ) )
{
    //
}

semio::ros::HumanoidSinkROS::_HumanoidJointMsg::Ptr semio::ros::HumanoidSinkROS::toROSMsgPtr( HumanoidJoint const & joint )
{
    _HumanoidJointMsg::Ptr joint_msg_ptr( new _HumanoidJointMsg() );
    auto & joint_msg( *joint_msg_ptr );

    joint_msg.type = static_cast<uint32_t>( joint.joint_type_ );
    joint_msg.position_confidence = joint.position_confidence_;
    joint_msg.orientation_confidence = joint.orientation_confidence_;
    joint_msg.position.x = joint.position_.x();
    joint_msg.position.y = joint.position_.y();
    joint_msg.position.z = joint.position_.z();
    joint_msg.orientation.w = joint.orientation_.w();
    joint_msg.orientation.x = joint.orientation_.x();
    joint_msg.orientation.y = joint.orientation_.y();
    joint_msg.orientation.z = joint.orientation_.z();

    return joint_msg_ptr;
}

semio::ros::HumanoidSinkROS::_HumanoidJointMsg semio::ros::HumanoidSinkROS::toROSMsg( HumanoidJoint const & joint )
{
    return *HumanoidSinkROS::toROSMsgPtr( joint );
}

semio::ros::HumanoidSinkROS::_HumanoidMsg::Ptr semio::ros::HumanoidSinkROS::toROSMsgPtr( Humanoid const & humanoid )
{
    _HumanoidMsg::Ptr humanoid_msg_ptr( new _HumanoidMsg() );
    auto & humanoid_msg( *humanoid_msg_ptr );

    humanoid_msg.id = humanoid.id_;
    humanoid_msg.tracking_state = static_cast<uint32_t>( humanoid.tracking_state_ );
    humanoid_msg.joints.reserve( humanoid.joints_.size() );

    for( auto const & joint_item : humanoid.joints_ )
    {
        humanoid_msg.joints.emplace_back( std::move( HumanoidSinkROS::toROSMsg( joint_item.second ) ) );
    }

    return humanoid_msg_ptr;
}

semio::ros::HumanoidSinkROS::_HumanoidMsg semio::ros::HumanoidSinkROS::toROSMsg( Humanoid const & humanoid )
{
    return *HumanoidSinkROS::toROSMsgPtr( humanoid );
}

semio::ros::HumanoidSinkROS::_HumanoidsMsg::Ptr semio::ros::HumanoidSinkROS::toROSMsgPtr( HumanoidArray const & humanoids )
{
    _HumanoidsMsg::Ptr humanoids_msg_ptr( new _HumanoidsMsg() );
    auto & humanoids_msg( *humanoids_msg_ptr );

    humanoids_msg.humanoids.reserve( humanoids.size() );

    for( auto const & humanoid : humanoids )
    {
        humanoids_msg.humanoids.emplace_back( std::move( HumanoidSinkROS::toROSMsg( humanoid ) ) );
    }

    return humanoids_msg_ptr;
}

semio::ros::HumanoidSinkROS::_HumanoidsMsg semio::ros::HumanoidSinkROS::toROSMsg( HumanoidArray const & humanoids )
{
    return *HumanoidSinkROS::toROSMsgPtr( humanoids );
}

void semio::ros::HumanoidSinkROS::publish( HumanoidArray const & humanoids )
{
    _humanoids_pub.publish( HumanoidSinkROS::toROSMsgPtr( humanoids ) );
}
