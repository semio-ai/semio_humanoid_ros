#ifndef _SEMIO_ROS_HUMANOIDSINKADAPTER_H_
#define _SEMIO_ROS_HUMANOIDSINKADAPTER_H_

#include <semio/recognition/humanoid_sink.h>
#include <ros/node_handle.h>
#include <string>

namespace semio
{

namespace ros
{

//! Utility class to simplify the creation of semio::HumanoidSink instances
class HumanoidSinkAdapter
{
protected:
    //! NodeHandle copy for interfacing with ROS
    ::ros::NodeHandle _nh_rel;
    //! The type of sink to use by default if param mode is selected and the ROS param @b `_humanoid_sink/type` is not defined @sa getHumanoidSink()
    std::string _default_sink;

public:
    /**
    @param nh_rel @copybrief _nh_rel
    @param default_sink @copybrief _default_sink
    */
    HumanoidSinkAdapter( ::ros::NodeHandle const & nh_rel, std::string const & default_sink = "ros" );

    //! Create a semio::HumanoidSink
    /**
    If the ROS param @b `_humanoid_sink/filter/smoothing` is defined, then a semio::HumanoidSmoothingFilter will be added to the sink's filter list. See @ref getHumanoidSink-ros-params "ROS Params" for more info.

    @param sink The type of sink to create
    <table>
    <tr><th>Value</th><th>Resulting pointer type</th><th>Notes</th></tr>
    <tr><td>`"param"`</td><td>variable based on ROS param</td><td>Uses the value of the ROS param @b `_humanoid_sink/type` to set @p sink; defaults to #_default_sink defined during construction</td></tr>
    <tr><td>`"ros"`</td><td>semio::ros::HumanoidSinkROS</td><td>Use ROS to sink humanoids; if the smoothing filter is enabled, creates the sink on topic @b `~humanoids/smoothed`</td></tr>
    <tr><td>`"none"`</td><td>semio::ros::HumanoidSinkROS</td><td>Specialization of semio::ros::HumanoidSinkROS, connects to topic @b `/null`</td></tr>
    </table>
    @note `"none"` will be used if the value of @p sink is not recognized

    @anchor getHumanoidSink-ros-params
    @par ROS Params
    <table>
    <tr><th>Name</th><th>Type</th><th>Default</th><th>Description</th></tr>
    <tr><td>@b `_humanoid_sink/type`</td><td>string</td><td>#_default_sink</td><td>Type of sink to create when @p sink is `"param"`</td></tr>
    <tr><td>@b `_humanoid_sink/filter/smoothing/add`</td><td>any</td><td>undefined</td><td>Define this parameter to add a default smoothing filter to outgoing humanoids (when no other smoothing params are defined)</td></tr>
    <tr><td>@b `_humanoid_sink/filter/smoothing/position`</td><td>float64</td><td>undefined</td><td>The strength of the position component of the smoothing filter</td></tr>
    <tr><td>@b `_humanoid_sink/filter/smoothing/orientation`</td><td>float64</td><td>undefined</td><td>The strength of the orientation component of the smoothing filter</td></tr>
    <tr><td>@b `_humanoid_sink/filter/smoothing/confidence`</td><td>float64</td><td>undefined</td><td>The strength of the confidence component of the smoothing filter</td></tr>
    <tr><td>@b `_humanoid_sink/filter/smoothing/window`</td><td>float64</td><td>undefined</td><td>The window (duration) of the smoothing filter; @c 0 = use all samples</td></tr>
    </table>

    @return The requested semio::HumanoidSink
    */
    HumanoidSink::Ptr getHumanoidSink( std::string const & sink = "param" );
};

}

}
#endif // _SEMIO_ROS_HUMANOIDSINKADAPTER_H_
