// Copyright [2020] [Daniel Peter, peter@fh-aachen.de, Fachhochschule Aachen]
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.

#include "../include/radar_conti_ars408_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <math.h>

#define _USE_MATH_DEFINES

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace FHAC
{
radar_conti_ars408::radar_conti_ars408(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("radar_conti_ars408", options)
{
    
} 

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_configure(
    const rclcpp_lifecycle::State&)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
  auto node = shared_from_this();
  node->declare_parameter("can_channel", rclcpp::ParameterValue("can0"));
  node->declare_parameter("object_list_topic_name", rclcpp::ParameterValue("/ars408/objectlist"));
  node->declare_parameter("marker_array_topic_name", rclcpp::ParameterValue("/ars408/marker_array"));
  node->declare_parameter("radar_tracks_topic_name", rclcpp::ParameterValue("/ars408/radar_tracks"));
  node->declare_parameter("radar_link", rclcpp::ParameterValue("radar_link"));
  
  node->get_parameter("can_channel", can_channel_);
  node->get_parameter("object_list_topic_name", object_list_topic_name_);
  node->get_parameter("marker_array_topic_name", marker_array_topic_name_);
  node->get_parameter("radar_tracks_topic_name", radar_tracks_topic_name_);
  node->get_parameter("radar_link", radar_link_);	

  auto radar_tracks_qos = rclcpp::QoS(rclcpp::KeepLast(5), rmw_qos_profile_sensor_data);

  object_list_publisher_ = this->create_publisher<radar_conti_ars408_msgs::msg::ObjectList>(object_list_topic_name_, qos);
  tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(pub_tf_topic_name, qos);
  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_array_topic_name_, qos);
  radar_tracks_publisher_ = this->create_publisher<radar_msgs::msg::RadarTracks>(radar_tracks_topic_name_, radar_tracks_qos);
  
  canChannel0.Init(can_channel_.c_str(), std::bind(&radar_conti_ars408::can_receive_callback, this, _1));
  object_count = 0.0;
  set_filter_service_ = create_service<radar_conti_ars408_msgs::srv::SetFilter>("/set_filter", std::bind(&radar_conti_ars408::setFilter, this, std::placeholders::_1, std::placeholders::_2));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS; 
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_shutdown(
  const rclcpp_lifecycle::State& previous_state)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_error(
    const rclcpp_lifecycle::State& previous_state) 
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_activate(
    const rclcpp_lifecycle::State&) 
{   
  object_list_publisher_->on_activate();
  tf_publisher_->on_activate();
  marker_array_publisher_->on_activate();
  radar_tracks_publisher_->on_activate();

  bond_ = std::make_unique<bond::Bond>(std::string("bond"), this->get_name(), shared_from_this());
  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();

  RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_deactivate(
    const rclcpp_lifecycle::State&)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_cleanup(
    const rclcpp_lifecycle::State&)
{
  RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
  if (bond_) {
    bond_.reset();
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void radar_conti_ars408::can_receive_callback(const can_msgs::msg::Frame msg)
{    
  if (msg.id == ID_RadarState) {
      operation_mode_ =CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(msg.data),1.0);
  }

  //no output
  if (operation_mode_ == 0x00) {
      return;
  }

  //object list
  if (operation_mode_ == 0x01) {
      handle_object_list(msg);
  }
}

void radar_conti_ars408::handle_object_list(const can_msgs::msg::Frame msg) {

  if (msg.id == ID_Obj_0_Status) {
      
  if(object_count == object_list_.object_count.data){
    publish_object_map();
  }else{
    RCLCPP_INFO(this->get_logger(), "Error");
  }

  object_count = 0;
  object_list_.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
  object_list_.object_count.data = GET_Obj_0_Status_Obj_NofObjects(msg.data);

  object_map_.clear();


  }

  //Object General Information
  //for each Obj_1_General message a new object has to be created in the map
  if (msg.id == ID_Obj_1_General) {

      radar_conti_ars408_msgs::msg::Object o;

      //object ID
      int id = GET_Obj_1_General_Obj_ID(msg.data);
      o.obj_id.data = GET_Obj_1_General_Obj_ID(msg.data);

      //longitudinal distance
      o.object_general.obj_distlong.data =
              CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(msg.data), 1.0);

      //lateral distance
      o.object_general.obj_distlat.data =
              CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(msg.data), 1.0);

      //relative longitudinal velocity
      o.object_general.obj_vrellong.data =
              CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(msg.data), 1.0);

      //relative lateral velocity
      o.object_general.obj_vrellat.data =
              CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(msg.data), 1.0);

      o.object_general.obj_dynprop.data =
              CALC_Obj_1_General_Obj_DynProp(GET_Obj_1_General_Obj_DynProp(msg.data), 1.0);

      o.object_general.obj_rcs.data = 
              CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(msg.data), 1.0);

      //insert object into map
      object_map_.insert(std::pair<int, radar_conti_ars408_msgs::msg::Object>(id, o));
  }

  //Object Quality Information
  //for each Obj_2_Quality message the existing object in the map has to be updated
  if (msg.id == ID_Obj_2_Quality) {

      // //RCLCPP_INFO(this->get_logger(), "Received Object_2_Quality msg (0x60c)");

      int id = GET_Obj_2_Quality_Obj_ID(msg.data);

      object_map_[id].object_quality.obj_distlong_rms.data =
              CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(msg.data), 1.0);

      object_map_[id].object_quality.obj_distlat_rms.data =
              CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(msg.data), 1.0);

      object_map_[id].object_quality.obj_vrellong_rms.data =
              CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(msg.data), 1.0);

      object_map_[id].object_quality.obj_vrellat_rms.data =
              CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(msg.data), 1.0);

  }

  //Object Extended Information
  //for each Obj_3_ExtInfo message the existing object in the map has to be updated
  if (msg.id == ID_Obj_3_Extended) {

      int id = GET_Obj_3_Extended_Obj_ID(msg.data);

      object_map_[id].object_extended.obj_arellong.data =
              CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(msg.data), 1.0);

      object_map_[id].object_extended.obj_arellat.data =
              CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(msg.data), 1.0);

      object_map_[id].object_extended.obj_class.data =
              CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(msg.data), 1.0);

      object_map_[id].object_extended.obj_orientationangle.data =
              CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(msg.data), 1.0);

      object_map_[id].object_extended.obj_length.data =
              CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(msg.data), 1.0);

      object_map_[id].object_extended.obj_width.data =
              CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(msg.data), 1.0);

      object_count = object_count + 1;

  };
}

void radar_conti_ars408::publish_object_map() {

  visualization_msgs::msg::MarkerArray marker_array;
  radar_msgs::msg::RadarTracks radar_tracks;

  radar_tracks.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
  radar_tracks.header.frame_id = radar_link_;

  marker_array.markers.clear();

  //delete old marker
  visualization_msgs::msg::Marker ma;
  ma.action=3;
  marker_array.markers.push_back(ma);
  marker_array_publisher_->publish(marker_array);
  marker_array.markers.clear();

  //marker for ego car
  visualization_msgs::msg::Marker mEgoCar;

  mEgoCar.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
  mEgoCar.header.frame_id = radar_link_;
  mEgoCar.ns = "";
  mEgoCar.id = 999;

  //if you want to use a cube comment out the next 2 lines
  mEgoCar.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  mEgoCar.type = 1; // cube
  mEgoCar.pose.position.x = -2.0;
  mEgoCar.pose.position.y = 0.0;
  mEgoCar.pose.position.z = 1.0;

  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, M_PI/2);

  mEgoCar.pose.orientation.w = myQuaternion.getW();
  mEgoCar.pose.orientation.x = myQuaternion.getX();
  mEgoCar.pose.orientation.y = myQuaternion.getY();
  mEgoCar.pose.orientation.z = myQuaternion.getZ();
  mEgoCar.scale.x = 1.0;
  mEgoCar.scale.y = 1.0;
  mEgoCar.scale.z = 1.0;
  mEgoCar.color.r = 0.0;
  mEgoCar.color.g = 0.0;
  mEgoCar.color.b = 1.0;
  mEgoCar.color.a = 1.0;
  mEgoCar.lifetime = rclcpp::Duration::from_seconds(0.2);
  mEgoCar.frame_locked = false;

  marker_array.markers.push_back(mEgoCar);

  std::map<int, radar_conti_ars408_msgs::msg::Object>::iterator itr;
  
  for (itr = object_map_.begin(); itr != object_map_.end(); ++itr) {

      visualization_msgs::msg::Marker mobject;

      mobject.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
      mobject.header.frame_id = radar_link_;
      mobject.ns = "";
      mobject.id = itr->first;
      mobject.type = 1; //Cube
      mobject.action = 0; // add/modify
      mobject.pose.position.x = itr->second.object_general.obj_distlong.data;
      mobject.pose.position.y = itr->second.object_general.obj_distlat.data;
      mobject.pose.position.z = 1.0;

      double yaw = itr->second.object_extended.obj_orientationangle.data*3.1416/180.0;

      myQuaternion.setRPY(0, 0, yaw);

      mobject.pose.orientation.w = myQuaternion.getW();
      mobject.pose.orientation.x = myQuaternion.getX();
      mobject.pose.orientation.y = myQuaternion.getY();
      mobject.pose.orientation.z = myQuaternion.getZ();
      mobject.scale.x = itr->second.object_extended.obj_length.data;
      mobject.scale.y = itr->second.object_extended.obj_width.data;
      mobject.scale.z = 1.0;
      mobject.color.r = 0.0;
      mobject.color.g = 1.0;
      mobject.color.b = 0.0;
      mobject.color.a = 1.0;
      mobject.lifetime = rclcpp::Duration::from_seconds(0.2);
      mobject.frame_locked = false;


      radar_msgs::msg::RadarTrack radar_track;
      radar_track.position.x = itr->second.object_general.obj_distlong.data;
      radar_track.position.y = itr->second.object_general.obj_distlat.data; 
      radar_track.position.z = 1.0;

      radar_track.velocity.x = 0.0;
      radar_track.velocity.y = 0.0;
      radar_track.velocity.z = 0.0;

      radar_track.acceleration.x = 0.0;
      radar_track.acceleration.y = 0.0;
      radar_track.acceleration.z = 0.0;

      radar_track.size.x = itr->second.object_extended.obj_length.data;
      radar_track.size.y = itr->second.object_extended.obj_width.data;
      radar_track.size.z = 0.0;


      marker_array.markers.push_back(mobject); 
      radar_tracks.tracks.push_back(radar_track);

  }

  marker_array_publisher_->publish(marker_array);
  radar_tracks_publisher_->publish(radar_tracks);
            
}

void radar_conti_ars408::setFilter(
    const std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Request> request,
    std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Response> response)
{

  can_msgs::msg::Frame msg;

  msg.id = ID_FilterCfg;
  msg.dlc = 5;
  SET_FilterCfg_FilterCfg_Active(msg.data, 1);
  SET_FilterCfg_FilterCfg_Valid(msg.data, 1);
  SET_FilterCfg_FilterCfg_Type(msg.data, request->type);
  SET_FilterCfg_FilterCfg_Index(msg.data, request->index);

  if(request->index > 14){
    RCLCPP_ERROR(this->get_logger(), "Unknown Filter Index");
    response->success = false;
    return;
  }

  if(request->index == 0){
    RCLCPP_INFO(this->get_logger(), "Setting Number Of Objects Filter");
    SET_FilterCfg_FilterCfg_Max_NofObj(msg.data, request->max_value);
    SET_FilterCfg_FilterCfg_Min_NofObj(msg.data, request->min_value);
  }else if(request->index == 1){
    RCLCPP_INFO(this->get_logger(), "Setting Distance Filter");
    SET_FilterCfg_FilterCfg_Max_Distance(msg.data, request->max_value/0.1);
    SET_FilterCfg_FilterCfg_Min_Distance(msg.data, request->min_value/0.1);
  }else if(request->index == 2){
    RCLCPP_INFO(this->get_logger(), "Setting Azimuth Filter");
    SET_FilterCfg_FilterCfg_Max_Azimuth(msg.data, request->max_value/0.025);
    SET_FilterCfg_FilterCfg_Min_Azimuth(msg.data, request->min_value/0.025);
  }else if(request->index == 3){
    RCLCPP_INFO(this->get_logger(), "Setting Oncoming Velocity Filter");
    SET_FilterCfg_FilterCfg_Max_VrelOncome(msg.data, request->max_value/0.0315);
    SET_FilterCfg_FilterCfg_Min_VrelOncome(msg.data, request->min_value/0.0315);
  }else if(request->index == 4){
    RCLCPP_INFO(this->get_logger(), "Setting Departing Velocity Filter");
    SET_FilterCfg_FilterCfg_Max_VrelDepart(msg.data, request->max_value/0.0315);
    SET_FilterCfg_FilterCfg_Min_VrelDepart(msg.data, request->min_value/0.0315);
  }else if(request->index == 5){
    RCLCPP_INFO(this->get_logger(), "Setting RCS Filter");
    SET_FilterCfg_FilterCfg_Max_RCS(msg.data, request->max_value/0.025);
    SET_FilterCfg_FilterCfg_Min_RCS(msg.data, request->min_value/0.025);
  }else if(request->index == 6){
    RCLCPP_INFO(this->get_logger(), "Setting Lifetime Filter");
    SET_FilterCfg_FilterCfg_Max_Lifetime(msg.data, request->max_value/0.1);
    SET_FilterCfg_FilterCfg_Min_Lifetime(msg.data, request->min_value/0.1);
  }else if(request->index == 7){
    RCLCPP_INFO(this->get_logger(), "Setting Size Filter");
    SET_FilterCfg_FilterCfg_Max_Size(msg.data, request->max_value/0.025);
    SET_FilterCfg_FilterCfg_Min_Size(msg.data, request->min_value/0.025);
  }else if(request->index == 8){
    RCLCPP_INFO(this->get_logger(), "Setting Probability of Existence Filter");
    SET_FilterCfg_FilterCfg_Max_ProbExists(msg.data, request->max_value);
    SET_FilterCfg_FilterCfg_Min_ProbExists(msg.data, request->min_value);
  }else if(request->index == 9){ 
    RCLCPP_INFO(this->get_logger(), "Setting Y Filter");
    SET_FilterCfg_FilterCfg_Max_Y(msg.data, request->max_value/0.2);
    SET_FilterCfg_FilterCfg_Min_Y(msg.data, request->min_value/0.2);
  }else if(request->index == 10){ 
    //TODO: MAKE THIS 13BIT
    RCLCPP_INFO(this->get_logger(), "X Filter currently not implemented");
    response->success = false;
    return;
  }else if(request->index == 11){ 
     RCLCPP_INFO(this->get_logger(), "Setting Right Left Filter");
    SET_FilterCfg_FilterCfg_Max_VYRightLeft(msg.data, request->max_value/0.0315);
    SET_FilterCfg_FilterCfg_Min_VYRightLeft(msg.data, request->min_value/0.0315);
  }else if(request->index == 12){ 
    RCLCPP_INFO(this->get_logger(), "Setting X Oncoming Filter");
    SET_FilterCfg_FilterCfg_Max_VXOncome(msg.data, request->max_value/0.0315);
    SET_FilterCfg_FilterCfg_Min_VXOncome(msg.data, request->min_value/0.0315);
  }else if(request->index == 13){ 
    RCLCPP_INFO(this->get_logger(), "Setting Left Right Filter");
    SET_FilterCfg_FilterCfg_Max_VYLeftRight(msg.data, request->max_value/0.0315);
    SET_FilterCfg_FilterCfg_Min_VYLeftRight(msg.data, request->min_value/0.0315);
  }else if(request->index == 14){ 
    RCLCPP_INFO(this->get_logger(), "Setting X Departing Filter");
    SET_FilterCfg_FilterCfg_Max_VXDepart(msg.data, request->max_value/0.0315);
    SET_FilterCfg_FilterCfg_Min_VXDepart(msg.data, request->min_value/0.0315); 
  }

  canChannel0.CanSend(msg);
  response->success = true;
}

} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
//CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_ars408, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_ars408)
