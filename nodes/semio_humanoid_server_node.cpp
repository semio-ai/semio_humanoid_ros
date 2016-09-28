#include <ros/ros.h>

#include <semio/ros/humanoid_source_adapter.h>
#include <semio/ros/humanoid_sink_adapter.h>

class SemioHumanoidServerNode
{
protected:
    semio::HumanoidSource::Ptr _humanoid_source_ptr;
    semio::HumanoidSink::Ptr _humanoid_sink_ptr;

public:
    SemioHumanoidServerNode( semio::HumanoidSource::Ptr humanoid_source_ptr, semio::HumanoidSink::Ptr humanoid_sink_ptr )
    :
        _humanoid_source_ptr( humanoid_source_ptr ),
        _humanoid_sink_ptr( humanoid_sink_ptr )
    {
        //
    }

    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            ros::spinOnce();

            _humanoid_sink_ptr->publish( _humanoid_source_ptr->update() );

            loop_rate.sleep();
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_humanoid_server_node" );
    ros::NodeHandle nh_rel( "~" );

    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel );
    semio::ros::HumanoidSinkAdapter humanoid_sink_adapter( nh_rel, "ros" );

    SemioHumanoidServerNode semio_humanoid_server_node( humanoid_source_adapter.getHumanoidSource(), humanoid_sink_adapter.getHumanoidSink() );
    semio_humanoid_server_node.spin();

    return 0;
}
