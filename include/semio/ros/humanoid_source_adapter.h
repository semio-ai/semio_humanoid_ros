#ifndef _SEMIO_ROS_HUMANOIDSOURCEADAPTER_H_
#define _SEMIO_ROS_HUMANOIDSOURCEADAPTER_H_

#include <semio/recognition/humanoid_source.h>
#include <ros/node_handle.h>
#include <string>

namespace semio
{

namespace ros
{

//! Utility class to simplify the creation of semio::HumanoidSource instances
class HumanoidSourceAdapter
{
protected:
    //! NodeHandle copy for interfacing with ROS
    ::ros::NodeHandle _nh_rel;
    //! The type of source to use by default if param mode is selected and the ROS param @b `_humanoid_source/type` is not defined @sa getHumanoidSource()
    std::string _default_source;

public:
    /**
    @param nh_rel @copybrief _nh_rel
    @param default_source @copybrief _default_source
    */
    HumanoidSourceAdapter( ::ros::NodeHandle const & nh_rel, std::string const & default_source = "fullbody" );

    //! Create a semio::HumanoidSource
    /**
    If the ROS param @b `_humanoid_source/filter/smoothing` is defined, then a semio::HumanoidSmoothingFilter will be added to the source's filter list. See @ref getHumanoidSource-ros-params "ROS Params" for more info.

    @param source The type of source to create
    <table>
    <tr><th>Value</th><th>Resulting pointer type</th><th>Notes</th></tr>
    <tr><td>`"param"`</td><td>variable based on ROS param</td><td>Uses the value of the ROS param @b `_humanoid_source/type` to set @p source; defaults to #_default_source defined during construction</td></tr>
    <tr><td>`"nite"`</td><td>semio::HumanoidSourceNiTE</td><td>Use Kinect V2/NiTE to source humanoids (full body, only rough head pose)</td></tr>
    <tr><td>`"openface"`</td><td>semio::HumanoidSourceOpenFace</td><td>Use OpenFace to source humanoids (head pose only)</td></tr>
    <tr><td>`"fullbody"`</td><td>semio::HumanoidSourceFullBody</td><td>Merge humanoids from Kinect V2/NiTE and OpenFace (full body, including head)</td></tr>
    <tr><td>`"ros"`</td><td>semio::ros::HumanoidSourceROS</td><td>Use ROS to source humanoids</td></tr>
    <tr><td>`"none"`</td><td>semio::ros::HumanoidSourceROS</td><td>Specialization of semio::ros::HumanoidSourceROS, connects to topic @b `/null`</td></tr>
    <tr><td>`"virtual"`</td><td>semio::ros::HumanoidSourceVirtual</td><td>Create virtual humanoid that can be posed in code</td></tr>
    </table>
    @note `"none"` will be used if the value of @p source is not recognized

    @anchor getHumanoidSource-ros-params
    @par ROS Params
    <table>
    <tr><th>Name</th><th>Type</th><th>Default</th><th>Description</th></tr>
    <tr><td>@b `_humanoid_source/type`</td><td>string</td><td>#_default_source</td><td>Type of source to create when @p source is `"param"`</td></tr>
    <tr><td>@b `_humanoid_source/filter/smoothing/add`</td><td>any</td><td>undefined</td><td>Define this parameter to add a default smoothing filter to incoming humanoids (when no other smoothing params are defined)</td></tr>
    <tr><td>@b `_humanoid_source/filter/smoothing/position`</td><td>float64</td><td>undefined</td><td>The strength of the position component of the smoothing filter</td></tr>
    <tr><td>@b `_humanoid_source/filter/smoothing/orientation`</td><td>float64</td><td>undefined</td><td>The strength of the orientation component of the smoothing filter</td></tr>
    <tr><td>@b `_humanoid_source/filter/smoothing/confidence`</td><td>float64</td><td>undefined</td><td>The strength of the confidence component of the smoothing filter</td></tr>
    <tr><td>@b `_humanoid_source/filter/smoothing/window`</td><td>float64</td><td>undefined</td><td>The window (duration) of the smoothing filter; @c 0 = use all samples</td></tr>
    <tr><td>@b `_humanoid_source/fullbody/show_cropped_image`</td><td>bool</td><td>false</td><td>Whether to display the cropped face image from semio::HumanoidSourceFullBody</td></tr>
    </table>

    @return The requested semio::HumanoidSource
    */
    HumanoidSource::Ptr getHumanoidSource( std::string const & source = "param" );
};

}

}

#endif // _SEMIO_ROS_HUMANOIDSOURCEADAPTER_H_
