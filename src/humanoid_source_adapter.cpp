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
    std::string const & source_type( source == "param" ? _nh_rel.param( std::string( "humanoid_source/type" ), _default_source ) : source );

    semio::HumanoidSource::Ptr result;

    if( source_type == "nite" ) result = std::make_shared<semio::HumanoidSourceNiTE>();
    else if( source_type == "openface" ) result = std::make_shared<semio::HumanoidSourceOpenFace>();
    else if( source_type == "fullbody" )
    {
        auto local_result( std::make_shared<semio::HumanoidSourceFullBody>() );
        local_result->show_cropped_image_ = _nh_rel.param<bool>( std::string( "humanoid_source/fullbody/show_cropped_image" ), false );
        result = local_result;
    }
    else if( source_type == "ros" ) result = std::make_shared<semio::ros::HumanoidSourceROS>( _nh_rel );
    else if( source_type == "none" ) return std::make_shared<semio::ros::HumanoidSourceROS>( ::ros::NodeHandle( _nh_rel, "/null" ) );
    else result = std::make_shared<HumanoidSourceVirtual>();

    // apply smoothing filter if specified via params
    {
        std::shared_ptr<semio::HumanoidSmoothingFilter> smoothing_filter_ptr;

        typedef std::function<void(std::string const &)> _ParamFunc;
        typedef std::pair<std::string, _ParamFunc> _ParamOp;

        for( auto const & param_op : {
            _ParamOp(
                "humanoid_source/filter/smoothing/add",
                []( std::string const & param_name ){} ),
            _ParamOp(
                "humanoid_source/filter/smoothing/position",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setPositionSmoothing( semio::HumanoidSmoothingFilter::getSmoothing( 0.75, this->_nh_rel.param<double>( param_name, 0 ) ) );
                } ),
            _ParamOp(
                "humanoid_source/filter/smoothing/orientation",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setOrientationSmoothing( semio::HumanoidSmoothingFilter::getSmoothing( 0.75, this->_nh_rel.param<double>( param_name, 0 ) ) );
                } ),
            _ParamOp(
                "humanoid_source/filter/smoothing/confidence",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setConfidenceSmoothing( semio::HumanoidSmoothingFilter::getSmoothing( 0.75, this->_nh_rel.param<double>( param_name, 0 ) ) );
                } ),
            _ParamOp(
                "humanoid_source/filter/smoothing/window",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setSmoothingWindow( this->_nh_rel.param<double>( param_name, 0 ) );
                } ) } )
        {
            auto const & param_name( param_op.first );

            if( _nh_rel.hasParam( param_name ) )
            {
                if( !smoothing_filter_ptr ) smoothing_filter_ptr = std::make_shared<semio::HumanoidSmoothingFilter>();

                param_op.second( param_name );
            }
        }

        // add a smoothing filter and a basic state filter (to remove untracked smoothed humanoids)
        if( smoothing_filter_ptr )
        {
            auto & result_filter( result->getFilter() );
            result_filter.addFilter( smoothing_filter_ptr );
            result_filter.addFilter( std::make_shared<semio::HumanoidStateFilter>( semio::HumanoidStateFilter::getStandardFilterHumanoid() ) );
        }
    }

    return result;
}
