#include <semio/ros/humanoid_source_ros.h>

semio::ros::HumanoidSourceROS::HumanoidSourceROS( ::ros::NodeHandle const & nh_rel )
:
    _nh_rel( nh_rel ),
    _humanoids_sub( _nh_rel.subscribe( "humanoids", 10, &HumanoidSourceROS::humanoidsCB, this ) )
{
    //
}

semio::HumanoidArray semio::ros::HumanoidSourceROS::updateFromSource()
{
    _HumanoidsMsg::ConstPtr const & msg_ptr( _last_humanoids_msg );
    _HumanoidsMsg const & msg( *msg_ptr );

    semio::HumanoidArray humanoids;

    if( _last_humanoids_msg )
    {
        humanoids.reserve( msg.humanoids.size() );

        for( auto const & humanoid_msg : msg.humanoids )
        {
            semio::Humanoid::_JointArray joints;

            for( auto const & joint_msg : humanoid_msg.joints )
            {
                joints.emplace( std::make_pair( static_cast<semio::HumanoidJoint::JointType>( joint_msg.type ), semio::HumanoidJoint(
                    static_cast<semio::HumanoidJoint::JointType>( joint_msg.type ),
                    Eigen::Vector3d( joint_msg.position.x, joint_msg.position.y, joint_msg.position.z ),
                    Eigen::Quaterniond( joint_msg.orientation.w, joint_msg.orientation.x, joint_msg.orientation.y, joint_msg.orientation.z ),
                    joint_msg.position_confidence,
                    joint_msg.orientation_confidence ) ) );
            }
            humanoids.emplace_back( humanoid_msg.id, static_cast<semio::Humanoid::TrackingState>( humanoid_msg.tracking_state ), joints );
        }
    }
    return humanoids;
}

void semio::ros::HumanoidSourceROS::humanoidsCB( _HumanoidsMsg::ConstPtr const & msg )
{
    _last_humanoids_msg = msg;
}
