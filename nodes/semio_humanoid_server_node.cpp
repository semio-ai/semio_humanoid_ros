#include <ros/ros.h>

#include <semio/ros/humanoid_source_adapter.h>
#include <semio/ros/humanoid_sink_adapter.h>
#include <semio/ros/humanoid_sink_ros.h>

//! Serve humanoids using source/sink model
class SemioHumanoidServerNode
{
protected:
    //! Pointer to the input source for humanoids
    semio::HumanoidSource::Ptr _humanoid_source_ptr;
    //! Vector of pointers to the output sinks for humanoids
    std::vector<semio::HumanoidSink::Ptr> _humanoid_sinks;

public:
    /**
    @param humanoid_source_ptr @copybrief _humanoid_source_ptr
    @param humanoid_sinks @copybrief _humanoid_sinks
    */
    SemioHumanoidServerNode( semio::HumanoidSource::Ptr humanoid_source_ptr, std::vector<semio::HumanoidSink::Ptr> const & humanoid_sinks )
    :
        _humanoid_source_ptr( humanoid_source_ptr ),
        _humanoid_sinks( humanoid_sinks )
    {
        //
    }

    //! Main loop
    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            //! - Trigger ROS callbacks
            ros::spinOnce();

            //----------
            //! - Publish humanoids from our input source to our output sinks
            auto const & humanoids( _humanoid_source_ptr->update() );
            for( auto & sink_ptr : _humanoid_sinks )
            {
                if( sink_ptr ) sink_ptr->publish( humanoids );
            }
            //----------

            loop_rate.sleep();
        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_humanoid_server_node" );
    //! - Create NodeHandle with relative namespace
    ros::NodeHandle nh_rel( "~" );

    //! - Create vector of humanoid sinks; initialize with a semio::ros::HumanoidSinkROS publishing to `~humanoids/raw`
    std::vector<semio::HumanoidSink::Ptr> humanoid_sinks{
        std::make_shared<semio::ros::HumanoidSinkROS>( nh_rel, "humanoids/raw" ) };

    //! - If any form of smoothing is enabled, create a standard ROS humanoid sink via semio::ros::HumanoidSinkAdapter
    if( nh_rel.hasParam( "humanoid_sink/filter/smoothing" ) )
    {
        humanoid_sinks.push_back( semio::ros::HumanoidSinkAdapter( nh_rel, "ros" ).getHumanoidSink() );
    }

    //! - Create SemioHumanoidServerNode; pass node handle, humanoid source, and humanoid sinks
    SemioHumanoidServerNode semio_humanoid_server_node(
        semio::ros::HumanoidSourceAdapter( nh_rel ).getHumanoidSource(),
        humanoid_sinks );

    //! - Start main loop SemioHumanoidServerNode::spin()
    semio_humanoid_server_node.spin();

    return 0;
}
