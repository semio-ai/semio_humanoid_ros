#include <iostream>

#include <ros/ros.h>

#include <semio_msgs_ros/Humanoids.h>

#include <semio/ros/humanoid_source_adapter.h>

class SemioHumanoidServerNode
{
public:
    typedef semio_msgs_ros::Humanoids _HumanoidsMsg;
    typedef semio_msgs_ros::Humanoid _HumanoidMsg;
    typedef semio_msgs_ros::HumanoidJoint _HumanoidJointMsg;

    ros::NodeHandle nh_rel_;
    ros::Publisher humanoids_pub_;

    semio::HumanoidSource::Ptr humanoid_source_ptr_;

    SemioHumanoidServerNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr )
    :
        nh_rel_( nh_rel ),
        humanoids_pub_( nh_rel_.advertise<_HumanoidsMsg>( "humanoids", 10 ) ),
        humanoid_source_ptr_( humanoid_source_ptr )
    {
        //
    }

    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            semio::HumanoidArray const & humanoids = humanoid_source_ptr_->update();

            _HumanoidsMsg humanoids_msg;

            humanoids_msg.humanoids.reserve( humanoids.size() );

            for( auto const & humanoid : humanoids )
            {
                auto & joints( humanoid.joints_ );

                _HumanoidMsg humanoid_msg;

                humanoid_msg.id = humanoid.id_;
                humanoid_msg.tracking_state = static_cast<uint32_t>( humanoid.tracking_state_ );
                humanoid_msg.joints.reserve( joints.size() );

                for( auto const & joint_item : joints )
                {
                    semio::HumanoidJoint const & joint( joint_item.second );

                    _HumanoidJointMsg joint_msg;

                    joint_msg.type = static_cast<size_t>( joint.joint_type_ );
                    joint_msg.position_confidence = joint.position_confidence_;
                    joint_msg.orientation_confidence = joint.orientation_confidence_;
                    joint_msg.position.x = joint.position_.x();
                    joint_msg.position.y = joint.position_.y();
                    joint_msg.position.z = joint.position_.z();
                    joint_msg.orientation.w = joint.orientation_.w();
                    joint_msg.orientation.x = joint.orientation_.x();
                    joint_msg.orientation.y = joint.orientation_.y();
                    joint_msg.orientation.z = joint.orientation_.z();

                    humanoid_msg.joints.push_back( std::move( joint_msg ) );
                }
                humanoids_msg.humanoids.push_back( std::move( humanoid_msg ) );
            }

            humanoids_pub_.publish( std::move( humanoids_msg ) );

            loop_rate.sleep();
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_humanoid_server_node" );
    ros::NodeHandle nh_rel( "~" );

    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel );

    SemioHumanoidServerNode semio_humanoid_server_node( nh_rel, humanoid_source_adapter.getHumanoidSource() );
    semio_humanoid_server_node.spin();

    return 0;
}
