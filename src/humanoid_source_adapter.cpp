#include <semio/ros/humanoid_source_adapter.h>

semio::ros::HumanoidSourceAdapter::HumanoidSourceAdapter( ::ros::NodeHandle & nh_rel, std::string const & default_source )
:
    _nh_rel( nh_rel ),
    _default_source( default_source )
{
    //
}

semio::HumanoidSource::Ptr semio::ros::HumanoidSourceAdapter::getHumanoidSource( std::string const & source )
{
    std::string const & source_type( source == "param" ? _nh_rel.param( std::string( "humanoid_source_type" ), _default_source ) : source );

    if( source_type == "nite" ) return std::make_shared<semio::HumanoidSourceNiTE>();
    else if( source_type == "virtual" ) return std::make_shared<semio::HumanoidSourceVirtual>();
    else if( source_type == "ros" ) return std::make_shared<semio::ros::HumanoidSourceROS>( _nh_rel );

    return std::make_shared<HumanoidSourceVirtual>();
}
