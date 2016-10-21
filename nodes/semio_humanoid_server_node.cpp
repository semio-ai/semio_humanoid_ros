#include <ros/ros.h>

#include <semio/ros/humanoid_source_adapter.h>
#include <semio/ros/humanoid_sink_adapter.h>
#include <semio/ros/humanoid_sink_ros.h>

class SemioHumanoidServerNode
{
protected:
    semio::HumanoidSource::Ptr _humanoid_source_ptr;
    std::vector<semio::HumanoidSink::Ptr> _humanoid_sinks;

public:
    SemioHumanoidServerNode( semio::HumanoidSource::Ptr humanoid_source_ptr, std::vector<semio::HumanoidSink::Ptr> const & humanoid_sinks )
    :
        _humanoid_source_ptr( humanoid_source_ptr ),
        _humanoid_sinks( humanoid_sinks )
    {
        //
    }

    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            ros::spinOnce();

            auto const & humanoids( _humanoid_source_ptr->update() );
            for( auto & sink_ptr : _humanoid_sinks )
            {
                if( sink_ptr ) sink_ptr->publish( humanoids );
            }

            loop_rate.sleep();
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_humanoid_server_node" );
    ros::NodeHandle nh_rel( "~" );

    std::vector<semio::HumanoidSink::Ptr> humanoid_sinks{
        std::make_shared<semio::ros::HumanoidSinkROS>( nh_rel, "humanoids/raw" ) };

    // only create an additional sink if smoothing is enabled
    if( nh_rel.hasParam( "humanoid_sink/filter/smoothing" ) )
    {
        humanoid_sinks.push_back( semio::ros::HumanoidSinkAdapter( nh_rel, "ros" ).getHumanoidSink() );
    }

    SemioHumanoidServerNode semio_humanoid_server_node(
        semio::ros::HumanoidSourceAdapter( nh_rel ).getHumanoidSource(),
        humanoid_sinks );

    semio_humanoid_server_node.spin();

    return 0;
}
