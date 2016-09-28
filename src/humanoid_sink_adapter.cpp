#include <semio/ros/humanoid_sink_adapter.h>

#include <semio/ros/humanoid_sink_ros.h>

semio::ros::HumanoidSinkAdapter::HumanoidSinkAdapter( ::ros::NodeHandle const & nh_rel, std::string const & default_sink )
:
    _nh_rel( nh_rel ),
    _default_sink( default_sink )
{
    //
}

semio::HumanoidSink::Ptr semio::ros::HumanoidSinkAdapter::getHumanoidSink( std::string const & sink )
{
    std::string const & sink_type( sink == "param" ? _nh_rel.param( std::string( "humanoid_sink_type" ), _default_sink ) : sink );

    if( sink_type == "ros" ) return std::make_shared<semio::ros::HumanoidSinkROS>( _nh_rel );

    // none
    return std::make_shared<semio::ros::HumanoidSinkROS>( ::ros::NodeHandle( _nh_rel, "/null" ) );
}
