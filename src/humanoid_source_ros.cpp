#include <semio/ros/humanoid_source_ros.h>

semio::ros::HumanoidSourceROS::HumanoidSourceROS( ::ros::NodeHandle const & nh_rel, std::string const & topic )
:
    _nh_rel( nh_rel ),
    _humanoids_sub( _nh_rel.subscribe( topic, 10, &HumanoidSourceROS::humanoidsCB, this ) )
{
    //
}

semio::HumanoidArray semio::ros::HumanoidSourceROS::updateFromSource()
{
    if( _last_humanoids_msg )
    {
        return HumanoidSourceROS::fromROSMsg( *_last_humanoids_msg );
        _last_humanoids_msg.reset();
    }

    return HumanoidArray();
}

semio::HumanoidJoint semio::ros::HumanoidSourceROS::fromROSMsg( _HumanoidJointMsg const & msg )
{
    return semio::HumanoidJoint(
        static_cast<semio::HumanoidJoint::JointType>( msg.type ),
        Eigen::Vector3d( msg.position.x, msg.position.y, msg.position.z ),
        Eigen::Quaterniond( msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z ),
        msg.position_confidence,
        msg.orientation_confidence );
}

semio::Humanoid semio::ros::HumanoidSourceROS::fromROSMsg( _HumanoidMsg const & msg )
{
    semio::Humanoid::_JointArray joints;

    for( auto const & joint_msg : msg.joints )
    {
        joints.emplace( std::make_pair(
            static_cast<semio::HumanoidJoint::JointType>( joint_msg.type ),
            std::move( HumanoidSourceROS::fromROSMsg( joint_msg ) ) ) );
    }

    return semio::Humanoid( msg.id, static_cast<semio::Humanoid::TrackingState>( msg.tracking_state ), std::move( joints ) );
}

semio::HumanoidArray semio::ros::HumanoidSourceROS::fromROSMsg( _HumanoidsMsg const & msg )
{
    semio::HumanoidArray humanoids;
    humanoids.reserve( msg.humanoids.size() );

    for( auto const & humanoid_msg : msg.humanoids )
    {
        humanoids.emplace_back( std::move( HumanoidSourceROS::fromROSMsg( humanoid_msg ) ) );
    }

    return humanoids;
}

void semio::ros::HumanoidSourceROS::humanoidsCB( _HumanoidsMsg::ConstPtr const & msg )
{
    _last_humanoids_msg = msg;
}
