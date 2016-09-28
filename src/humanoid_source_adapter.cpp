#include <semio/ros/humanoid_source_adapter.h>

#include <semio/ros/humanoid_source_ros.h>
#include <semio/recognition/humanoid_source_virtual.h>
#include <semio/recognition/humanoid_source_NiTE.h>
#include <semio/recognition/humanoid_source_openface.h>
#include <semio/recognition/humanoid_source_fullbody.h>

semio::ros::HumanoidSourceAdapter::HumanoidSourceAdapter( ::ros::NodeHandle const & nh_rel, std::string const & default_source )
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
    else if( source_type == "openface" ) return std::make_shared<semio::HumanoidSourceOpenFace>();
    else if( source_type == "fullbody" ) return std::make_shared<semio::HumanoidSourceFullBody>();
    else if( source_type == "ros" ) return std::make_shared<semio::ros::HumanoidSourceROS>( _nh_rel );
    else if( source_type == "none" ) return std::make_shared<semio::ros::HumanoidSourceROS>( ::ros::NodeHandle( _nh_rel, "/null" ) );

    // virtual
    return std::make_shared<HumanoidSourceVirtual>();
}
