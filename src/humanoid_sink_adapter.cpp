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
    std::string const & sink_type( sink == "param" ? _nh_rel.param( std::string( "humanoid_sink/type" ), _default_sink ) : sink );

    semio::HumanoidSink::Ptr result;

    // apply smoothing filter if specified via params
    std::shared_ptr<semio::HumanoidSmoothingFilter> smoothing_filter_ptr;
    {
        typedef std::function<void(std::string const &)> _ParamFunc;
        typedef std::pair<std::string, _ParamFunc> _ParamOp;

        for( auto const & param_op : {
            _ParamOp(
                "humanoid_sink/filter/smoothing/add",
                []( std::string const & param_name ){} ),
            _ParamOp(
                "humanoid_sink/filter/smoothing/position",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setPositionSmoothing( semio::HumanoidSmoothingFilter::getSmoothing( 0.75, this->_nh_rel.param<double>( param_name, 0 ) ) );
                } ),
            _ParamOp(
                "humanoid_sink/filter/smoothing/orientation",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setOrientationSmoothing( semio::HumanoidSmoothingFilter::getSmoothing( 0.75, this->_nh_rel.param<double>( param_name, 0 ) ) );
                } ),
            _ParamOp(
                "humanoid_sink/filter/smoothing/confidence",
                [this,&smoothing_filter_ptr]( std::string const & param_name ){
                    smoothing_filter_ptr->setConfidenceSmoothing( semio::HumanoidSmoothingFilter::getSmoothing( 0.75, this->_nh_rel.param<double>( param_name, 0 ) ) );
                } ),
            _ParamOp(
                "humanoid_sink/filter/smoothing/window",
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
    }

    if( sink_type == "ros" )
    {
        if( smoothing_filter_ptr ) result = std::make_shared<semio::ros::HumanoidSinkROS>( _nh_rel, "humanoids/smoothed" );
        else result = std::make_shared<semio::ros::HumanoidSinkROS>( _nh_rel );
    }
    else return std::make_shared<semio::ros::HumanoidSinkROS>( ::ros::NodeHandle( _nh_rel, "/null" ) );

    // add a smoothing filter and a basic state filter (to remove untracked smoothed humanoids)
    if( smoothing_filter_ptr )
    {
        auto & result_filter( result->getFilter() );
        result_filter.addFilter( smoothing_filter_ptr );
        result_filter.addFilter( std::make_shared<semio::HumanoidStateFilter>( semio::HumanoidStateFilter::getStandardFilterHumanoid() ) );
    }

    return result;
}
