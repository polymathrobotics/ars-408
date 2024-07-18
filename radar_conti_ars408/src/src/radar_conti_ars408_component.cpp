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

  const std::vector<FilterType> filterTypes = {
      FilterType::NOFOBJ,
      FilterType::DISTANCE,
      FilterType::AZIMUTH,
      FilterType::VRELONCOME,
      FilterType::VRELDEPART,
      FilterType::RCS,
      FilterType::LIFETIME,
      FilterType::SIZE,
      FilterType::PROBEXISTS,
      FilterType::Y,
      FilterType::X,
      FilterType::VYRIGHTLEFT,
      FilterType::VXONCOME,
      FilterType::VYLEFTRIGHT,
      FilterType::VXDEPART,
      FilterType::UNKNOWN // Add this to handle default case
  };

  radar_conti_ars408::radar_conti_ars408(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("radar_conti_ars408", options)
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_configure(
      const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure() is called.");
    auto node = shared_from_this();
    node->declare_parameter("can_channel", rclcpp::ParameterValue(""));
    node->declare_parameter("object_list_topic_name", rclcpp::ParameterValue("ars408/objectlist"));
    node->declare_parameter("marker_array_topic_name", rclcpp::ParameterValue("ars408/marker_array"));
    node->declare_parameter("radar_tracks_topic_name", rclcpp::ParameterValue("ars408/radar_tracks"));
    node->declare_parameter("obstacle_array_topic_name", rclcpp::ParameterValue("ars408/obstacle_array"));
    node->declare_parameter("filter_config_topic_name", rclcpp::ParameterValue("ars408/filter_config"));

    node->get_parameter("can_channel", can_channel_);
    node->get_parameter("object_list_topic_name", object_list_topic_name_);
    node->get_parameter("marker_array_topic_name", marker_array_topic_name_);
    node->get_parameter("radar_tracks_topic_name", radar_tracks_topic_name_);
    node->get_parameter("obstacle_array_topic_name", obstacle_array_topic_name_);
    node->get_parameter("filter_config_topic_name", filter_config_topic_name_);

    if (can_channel_.empty())
    {
      RCUTILS_LOG_ERROR_NAMED(get_name(), "No can_channel_ specified.");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    RCUTILS_LOG_INFO_NAMED(get_name(), "Listening on can_channel %s", can_channel_.c_str());

    auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
    auto radar_tracks_qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);

    size_t topic_ind = 0;
    bool more_params = false;
    number_of_radars_ = 0;
    do
    {
      // Build the string in the form of "radar_link_X", where X is the sensor ID of
      // the rader on the CANBUS, then check if we have any parameters with that value. Users need
      // to make sure they don't have gaps in their configs (e.g.,footprint0 and then
      // footprint2)
      std::stringstream ss;
      ss << "radar_" << topic_ind;
      std::string radar_name = ss.str();

      node->declare_parameter(radar_name + ".link_name", rclcpp::PARAMETER_STRING);

      rclcpp::Parameter parameter;
      if (node->get_parameter(radar_name + ".link_name", parameter))
      {
        more_params = true;
        object_list_publishers_.push_back(this->create_publisher<radar_conti_ars408_msgs::msg::ObjectList>(parameter.as_string() + "/" + object_list_topic_name_, qos));
        tf_publishers_.push_back(this->create_publisher<tf2_msgs::msg::TFMessage>(parameter.as_string() + "/" + pub_tf_topic_name, qos));
        marker_array_publishers_.push_back(this->create_publisher<visualization_msgs::msg::MarkerArray>(parameter.as_string() + "/" + marker_array_topic_name_, qos));
        radar_tracks_publishers_.push_back(this->create_publisher<radar_msgs::msg::RadarTracks>(parameter.as_string() + "/" + radar_tracks_topic_name_, radar_tracks_qos));
        obstacle_array_publishers_.push_back(this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>(parameter.as_string() + "/" + obstacle_array_topic_name_, radar_tracks_qos));
        filter_config_publishers_.push_back(this->create_publisher<radar_conti_ars408_msgs::msg::FilterStateCfg>(parameter.as_string() + "/" + filter_config_topic_name_, transient_local_qos));
        object_map_list_.push_back(std::map<int, radar_conti_ars408_msgs::msg::Object>());
        object_list_list_.push_back(radar_conti_ars408_msgs::msg::ObjectList());
        radar_filter_configs_.push_back(radar_conti_ars408_msgs::msg::FilterStateCfg());
        radar_filter_active_.push_back(std::vector<bool>());
        radar_filter_valid_.push_back(std::vector<bool>());

        std::vector<bool> init_radar_active_values = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        node->declare_parameter(radar_name + ".active", rclcpp::ParameterValue(init_radar_active_values));
        node->get_parameter(radar_name + ".active", radar_filter_active_[topic_ind]);
        std::vector<bool> init_radar_valid_values = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        node->declare_parameter(radar_name + ".valid", rclcpp::ParameterValue(init_radar_valid_values));
        node->get_parameter(radar_name + ".valid", radar_filter_valid_[topic_ind]);

        radar_link_names_.push_back(parameter.as_string());

        RCLCPP_DEBUG(node->get_logger(), "radar frame is: %s", parameter.as_string().c_str());

        // Initialize Number of Objects Filters
        initializeFilterConfig<uint32_t>(radar_name, std::string("filtercfg_min_nofobj"), 0, radar_filter_configs_[topic_ind].filtercfg_min_nofobj.data);
        initializeFilterConfig<uint32_t>(radar_name, std::string("filtercfg_max_nofobj"), 20, radar_filter_configs_[topic_ind].filtercfg_max_nofobj.data);

        // Initialize Distance Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_distance"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_distance.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_distance"), 409.0, radar_filter_configs_[topic_ind].filtercfg_max_distance.data);

        // Initialize Azimuth Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_azimuth"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_azimuth.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_azimuth"), 100.0, radar_filter_configs_[topic_ind].filtercfg_max_azimuth.data);

        // Initialize Oncoming Velocity Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vreloncome"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_vreloncome.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vreloncome"), 128.0, radar_filter_configs_[topic_ind].filtercfg_max_vreloncome.data);

        // Initialize Departing Velocity Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vreldepart"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_vreldepart.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vreldepart"), 128.0, radar_filter_configs_[topic_ind].filtercfg_max_vreldepart.data);

        // Initialize Radar Cross Section Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_rcs"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_rcs.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_rcs"), 100.0, radar_filter_configs_[topic_ind].filtercfg_max_rcs.data);

        // Initialize Lifetime Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_lifetime"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_lifetime.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_lifetime"), 409.0, radar_filter_configs_[topic_ind].filtercfg_max_lifetime.data);

        // Initialize Size Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_size"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_size.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_size"), 102.0, radar_filter_configs_[topic_ind].filtercfg_max_size.data);

        // Initialize Probability of Existence Filters
        initializeFilterConfig<uint8_t>(radar_name, std::string("filtercfg_min_probexists"), 0, radar_filter_configs_[topic_ind].filtercfg_min_probexists.data);
        initializeFilterConfig<uint8_t>(radar_name, std::string("filtercfg_max_probexists"), 7, radar_filter_configs_[topic_ind].filtercfg_max_probexists.data);

        // Initialize Y Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_y"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_y.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_y"), 818.0, radar_filter_configs_[topic_ind].filtercfg_max_y.data);

        // Initialize X Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_x"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_x.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_x"), 0.0, radar_filter_configs_[topic_ind].filtercfg_max_x.data);

        // Initialize Y Velocity going left to right Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vyrightleft"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_vyrightleft.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vyrightleft"), 128.0, radar_filter_configs_[topic_ind].filtercfg_max_vyrightleft.data);

        // Initialize X Velocity oncoming Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vxoncome"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_vxoncome.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vxoncome"), 128.0, radar_filter_configs_[topic_ind].filtercfg_max_vxoncome.data);

        // Initialize Y Velocity going right to left Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vyleftright"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_vyleftright.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vyleftright"), 128.0, radar_filter_configs_[topic_ind].filtercfg_max_vyleftright.data);

        // Initialize X Velocity Departing Filters
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_min_vxdepart"), 0.0, radar_filter_configs_[topic_ind].filtercfg_min_vxdepart.data);
        initializeFilterConfig<float>(radar_name, std::string("filtercfg_max_vxdepart"), 128.0, radar_filter_configs_[topic_ind].filtercfg_max_vxdepart.data);

        filter_config_initialized_list_.push_back(false);
        RCLCPP_WARN(this->get_logger(), "link_name is: %s", parameter.as_string().c_str());
        number_of_radars_++;
        topic_ind++;
      }
      else
      {
        more_params = false;
      }
    } while (more_params);

    // TODO(troy): Make a user configurable recv_timeout
    constexpr std::chrono::duration<float> recv_timeout{10};
    socketcan_adapter_ = std::make_unique<polymath::socketcan::SocketcanAdapter>(can_channel_, recv_timeout);
    object_count = 0.0;
    set_filter_service_ = create_service<radar_conti_ars408_msgs::srv::SetFilter>("/set_filter", std::bind(&radar_conti_ars408::setFilterService, this, std::placeholders::_1, std::placeholders::_2));

    generateUUIDTable();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  template <typename T>
  void radar_conti_ars408::initializeFilterConfig(std::string radar_name, std::string config_name, T value, T &config)
  {
    auto node = shared_from_this();
    T config_value;
    declare_parameter_with_type(node, radar_name + "." + config_name, value);
    get_parameter_with_type(node, radar_name + "." + config_name, config_value);
    config = config_value;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_shutdown(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on shutdown is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_error(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_activate(
      const rclcpp_lifecycle::State &)
  {

    if (!socketcan_adapter_->openSocket())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open socket on can channel '%s'", can_channel_.c_str());
    }

    // The middle byte is used by ARS408 as the sensor id, so we just clear it out here.
    canid_t id_mask = 0xF0F;
    can_filter radar_obj_filter{
        ID_Obj_1_General,
        id_mask,
    };

    can_filter filter_state_filter{
        ID_FilterState_Cfg,
        id_mask,
    };

    can_filter radar_state_filter{
        ID_RadarState,
        id_mask,
    };

    can_filter radar_obj_status_filter{
        ID_Obj_0_Status,
        id_mask,
    };

    can_filter radar_obj_quality_filter{
        ID_Obj_2_Quality,
        id_mask,
    };

    can_filter radar_obj_extended{
        ID_Obj_3_Extended,
        id_mask,
    };

    socketcan_adapter_->setFilters(std::vector<can_filter>{
        radar_obj_filter,
        filter_state_filter,
        radar_state_filter,
        radar_obj_status_filter,
        radar_obj_quality_filter,
        radar_obj_extended,
    });

    auto cb = [this](std::unique_ptr<const polymath::socketcan::CanFrame> frame)
    {
      std::shared_ptr<const polymath::socketcan::CanFrame> shared_frame = std::move(frame);
      this->can_receive_callback(shared_frame);
    };

    socketcan_adapter_->setOnReceiveCallback(std::move(cb));
    if (!socketcan_adapter_->startReceptionThread())
    {
      RCLCPP_ERROR(this->get_logger(), "Socket state is not opened");
    }

    for (size_t i = 0; i < object_list_publishers_.size(); i++)
    {
      object_list_publishers_[i]->on_activate();
      tf_publishers_[i]->on_activate();
      marker_array_publishers_[i]->on_activate();
      radar_tracks_publishers_[i]->on_activate();
      obstacle_array_publishers_[i]->on_activate();
      filter_config_publishers_[i]->on_activate();
    }

    initializeFilterConfigs();

    bond_ = std::make_unique<bond::Bond>(std::string("bond"), this->get_name(), shared_from_this());
    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(4.0);
    bond_->start();

    RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_deactivate(
      const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called.");
    socketcan_adapter_->joinReceptionThread();
    if (!socketcan_adapter_->closeSocket())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to close socket");
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn radar_conti_ars408::on_cleanup(
      const rclcpp_lifecycle::State &)
  {
    RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
    if (bond_)
    {
      bond_.reset();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  unique_identifier_msgs::msg::UUID radar_conti_ars408::generateRandomUUID()
  {
    unique_identifier_msgs::msg::UUID uuid;
    std::mt19937 gen(std::random_device{}());
    std::independent_bits_engine<std::mt19937, 8, uint8_t> bit_eng(gen);
    std::generate(uuid.uuid.begin(), uuid.uuid.end(), bit_eng);
    return uuid;
  }

  void radar_conti_ars408::generateUUIDTable()
  {
    for (size_t i = 0; i <= (max_radar_id * number_of_radars_); i++)
    {
      UUID_table_.emplace_back(radar_conti_ars408::generateRandomUUID());
    }
  }

  void radar_conti_ars408::initializeFilterConfigs()
  {
    const int type = FilterCfg_FilterCfg_Type_Object;
    int min_value = 0;
    int max_value = 0;

    for (int radar_index = 0; radar_index < radar_filter_configs_.size(); radar_index++)
    {
      for (int filter_index = 0; filter_index < MAX_FilterState_Cfg_FilterState_Index; filter_index++)
      {
        int active = radar_filter_active_[radar_index][filter_index] ? FilterCfg_FilterCfg_Active_active : FilterCfg_FilterCfg_Active_inactive;
        int valid = radar_filter_valid_[radar_index][filter_index] ? FilterCfg_FilterCfg_Valid_valid : FilterCfg_FilterCfg_Valid_invalid;
        switch (filterTypes[filter_index])
        {
        case FilterType::NOFOBJ:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_nofobj.data);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_nofobj.data);
          break;
        case FilterType::DISTANCE:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_distance.data / 0.1);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_distance.data / 0.1);
          break;
        case FilterType::AZIMUTH:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_azimuth.data / 0.025);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_azimuth.data / 0.025);
          break;
        case FilterType::VRELONCOME:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_vreloncome.data / 0.0315);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_vreloncome.data / 0.0315);
          break;
        case FilterType::VRELDEPART:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_vreldepart.data / 0.0315);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_vreldepart.data / 0.0315);
          break;
        case FilterType::RCS:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_rcs.data / 0.025);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_rcs.data / 0.025);
          break;
        case FilterType::LIFETIME:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_lifetime.data / 0.1);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_lifetime.data / 0.1);
          break;
        case FilterType::SIZE:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_size.data / 0.025);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_size.data / 0.025);
          break;
        case FilterType::PROBEXISTS:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_probexists.data);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_probexists.data);
          break;
        case FilterType::Y:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_y.data / 0.2);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_y.data / 0.2);
          break;
        case FilterType::X:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_x.data);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_x.data);
          break;
        case FilterType::VYRIGHTLEFT:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_vyrightleft.data / 0.0315);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_vyrightleft.data / 0.0315);
          break;
        case FilterType::VXONCOME:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_vxoncome.data / 0.0315);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_vxoncome.data / 0.0315);
          break;
        case FilterType::VYLEFTRIGHT:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_vyleftright.data / 0.0315);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_vyleftright.data / 0.0315);
          break;
        case FilterType::VXDEPART:
          min_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_min_vxdepart.data / 0.0315);
          max_value = static_cast<int>(radar_filter_configs_[radar_index].filtercfg_max_vxdepart.data / 0.0315);
          break;
        default:
          break;
        }
        setFilter(radar_index, active, valid, type, filter_index, min_value, max_value);
        // need to sleep in order to read properly and not spam the can bus
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      // initialized and publish once
      filter_config_initialized_list_[radar_index] = true;
      filter_config_publishers_[radar_index]->publish(radar_filter_configs_[radar_index]);
    }
  }

  void radar_conti_ars408::can_receive_callback(std::shared_ptr<const polymath::socketcan::CanFrame> frame)
  {

    int sensor_id = Get_SensorID_From_MsgID(frame->get_id());

    // If the sensor_id is greater than the size of the number of object lists, break
    if (sensor_id > object_list_list_.size() - 1)
    {
      return;
    }

    // When a filter configuration message is sent, the sensor replies with the messages
    // FilterState_Header (0x203) with the number of configured filters and one message FilterState_Cfg
    // (0x204) for the filter that has been changed.
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_FilterState_Cfg)
    {
      updateFilterConfig(frame, sensor_id);
    }

    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_RadarState)
    {
      operation_mode_ = CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(frame->get_data()), 1.0);
    }

    // no output
    if (operation_mode_ == RadarState_RadarState_OutputTypeCfg_None)
    {
      return;
    }

    // object list
    if (operation_mode_ == RadarState_RadarState_OutputTypeCfg_SendObjects)
    {
      handle_object_list(frame);
    }
  }

  void radar_conti_ars408::handle_object_list(std::shared_ptr<const polymath::socketcan::CanFrame> frame)
  {

    int sensor_id = Get_SensorID_From_MsgID(frame->get_id());

    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_0_Status)
    {
      publish_object_map(sensor_id);
      object_list_list_[sensor_id].header.stamp = rclcpp_lifecycle::LifecycleNode::now();
      object_list_list_[sensor_id].object_count.data = GET_Obj_0_Status_Obj_NofObjects(frame->get_data());
      object_map_list_[sensor_id].clear();
    }

    // Object General Information
    // for each Obj_1_General message a new object has to be created in the map
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_1_General)
    {

      radar_conti_ars408_msgs::msg::Object o;

      // object ID
      int id = GET_Obj_1_General_Obj_ID(frame->get_data());
      o.obj_id.data = GET_Obj_1_General_Obj_ID(frame->get_data());

      o.sensor_id.data = Get_SensorID_From_MsgID(frame->get_id());

      // longitudinal distance
      o.object_general.obj_distlong.data =
          CALC_Obj_1_General_Obj_DistLong(GET_Obj_1_General_Obj_DistLong(frame->get_data()), 1.0);

      // lateral distance
      o.object_general.obj_distlat.data =
          CALC_Obj_1_General_Obj_DistLat(GET_Obj_1_General_Obj_DistLat(frame->get_data()), 1.0);

      // relative longitudinal velocity
      o.object_general.obj_vrellong.data =
          CALC_Obj_1_General_Obj_VrelLong(GET_Obj_1_General_Obj_VrelLong(frame->get_data()), 1.0);

      // relative lateral velocity
      o.object_general.obj_vrellat.data =
          CALC_Obj_1_General_Obj_VrelLat(GET_Obj_1_General_Obj_VrelLat(frame->get_data()), 1.0);

      o.object_general.obj_dynprop.data =
          CALC_Obj_1_General_Obj_DynProp(GET_Obj_1_General_Obj_DynProp(frame->get_data()), 1.0);

      o.object_general.obj_rcs.data =
          CALC_Obj_1_General_Obj_RCS(GET_Obj_1_General_Obj_RCS(frame->get_data()), 1.0);

      // insert object into map
      object_map_list_[sensor_id].insert(std::pair<int, radar_conti_ars408_msgs::msg::Object>(id, o));
    }

    // Object Quality Information
    // for each Obj_2_Quality message the existing object in the map has to be updated
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_2_Quality)
    {

      // //RCLCPP_DEBUG(this->get_logger(), "Received Object_2_Quality msg (0x60c)");

      int id = GET_Obj_2_Quality_Obj_ID(frame->get_data());

      object_map_list_[sensor_id][id].object_quality.obj_distlong_rms.data =
          CALC_Obj_2_Quality_Obj_DistLong_rms(GET_Obj_2_Quality_Obj_DistLong_rms(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_quality.obj_distlat_rms.data =
          CALC_Obj_2_Quality_Obj_DistLat_rms(GET_Obj_2_Quality_Obj_DistLat_rms(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_quality.obj_vrellong_rms.data =
          CALC_Obj_2_Quality_Obj_VrelLong_rms(GET_Obj_2_Quality_Obj_VrelLong_rms(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_quality.obj_vrellat_rms.data =
          CALC_Obj_2_Quality_Obj_VrelLat_rms(GET_Obj_2_Quality_Obj_VrelLat_rms(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_quality.obj_probofexist.data =
          CALC_Obj_2_Quality_Obj_ProbOfExist(GET_Obj_2_Quality_Obj_ProbOfExist(frame->get_data()), 1.0);
    }

    // Object Extended Information
    // for each Obj_3_ExtInfo message the existing object in the map has to be updated
    if (Get_MsgID0_From_MsgID(frame->get_id()) == ID_Obj_3_Extended)
    {

      int id = GET_Obj_3_Extended_Obj_ID(frame->get_data());

      object_map_list_[sensor_id][id].object_extended.obj_arellong.data =
          CALC_Obj_3_Extended_Obj_ArelLong(GET_Obj_3_Extended_Obj_ArelLong(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_extended.obj_arellat.data =
          CALC_Obj_3_Extended_Obj_ArelLat(GET_Obj_3_Extended_Obj_ArelLat(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_extended.obj_class.data =
          CALC_Obj_3_Extended_Obj_Class(GET_Obj_3_Extended_Obj_Class(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_extended.obj_orientationangle.data =
          CALC_Obj_3_Extended_Obj_OrientationAngle(GET_Obj_3_Extended_Obj_OrientationAngle(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_extended.obj_length.data =
          CALC_Obj_3_Extended_Obj_Length(GET_Obj_3_Extended_Obj_Length(frame->get_data()), 1.0);

      object_map_list_[sensor_id][id].object_extended.obj_width.data =
          CALC_Obj_3_Extended_Obj_Width(GET_Obj_3_Extended_Obj_Width(frame->get_data()), 1.0);

      object_count = object_count + 1;
    };
  }

  void radar_conti_ars408::publish_object_map(int sensor_id)
  {

    visualization_msgs::msg::MarkerArray marker_array;
    radar_msgs::msg::RadarTracks radar_tracks;
    nav2_dynamic_msgs::msg::ObstacleArray obstacle_array;

    radar_tracks.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    radar_tracks.header.frame_id = radar_link_names_[sensor_id];

    obstacle_array.header = radar_tracks.header;

    marker_array.markers.clear();

    // delete old marker
    visualization_msgs::msg::Marker ma;
    ma.action = 3;
    marker_array.markers.push_back(ma);
    marker_array_publishers_[sensor_id]->publish(marker_array);
    marker_array.markers.clear();

    tf2::Quaternion myQuaternion;

    std::map<int, radar_conti_ars408_msgs::msg::Object>::iterator itr;

    for (itr = object_map_list_[sensor_id].begin(); itr != object_map_list_[sensor_id].end(); ++itr)
    {

      visualization_msgs::msg::Marker mobject;

      mobject.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
      mobject.header.frame_id = radar_link_names_[sensor_id];
      mobject.ns = "";
      mobject.id = itr->first;
      mobject.type = 1;   // Cube
      mobject.action = 0; // add/modify
      mobject.pose.position.x = itr->second.object_general.obj_distlong.data;
      mobject.pose.position.y = itr->second.object_general.obj_distlat.data;

      double yaw = itr->second.object_extended.obj_orientationangle.data * 3.1416 / 180.0;

      myQuaternion.setRPY(0, 0, yaw);

      mobject.pose.orientation.w = myQuaternion.getW();
      mobject.pose.orientation.x = myQuaternion.getX();
      mobject.pose.orientation.y = myQuaternion.getY();
      mobject.pose.orientation.z = myQuaternion.getZ();
      mobject.color.r = 0.0;
      mobject.color.g = 1.0;
      mobject.color.b = 0.0;
      mobject.color.a = 1.0;
      mobject.lifetime = rclcpp::Duration::from_seconds(0.2);
      mobject.frame_locked = false;

      radar_msgs::msg::RadarTrack radar_track;
      radar_track.uuid = UUID_table_[itr->first + max_radar_id * sensor_id];
      radar_track.position.x = itr->second.object_general.obj_distlong.data;
      radar_track.position.y = itr->second.object_general.obj_distlat.data;
      radar_track.position.z = 0.0;

      radar_track.velocity.x = itr->second.object_general.obj_vrellong.data;
      radar_track.velocity.y = itr->second.object_general.obj_vrellat.data;
      ;
      radar_track.velocity.z = 0.0;

      radar_track.acceleration.x = 0.0;
      radar_track.acceleration.y = 0.0;
      radar_track.acceleration.z = 0.0;

      radar_track.size.y = itr->second.object_extended.obj_length.data;
      radar_track.size.x = itr->second.object_extended.obj_width.data;
      radar_track.size.z = 1.0;

      nav2_dynamic_msgs::msg::Obstacle obstacle;
      obstacle.score = itr->second.object_quality.obj_probofexist.data;
      obstacle.uuid = radar_track.uuid;
      obstacle.position = radar_track.position;
      obstacle.velocity = radar_track.velocity;
      obstacle.size = radar_track.size;

      obstacle.position_covariance[0] = covariance[static_cast<int>(itr->second.object_quality.obj_distlong_rms.data)];
      obstacle.position_covariance[1] = 0.0;
      obstacle.position_covariance[2] = 0.0;
      obstacle.position_covariance[3] = 0.0;
      obstacle.position_covariance[4] = covariance[static_cast<int>(itr->second.object_quality.obj_distlat_rms.data)];
      obstacle.position_covariance[5] = 0.0;
      obstacle.position_covariance[6] = 0.0;
      obstacle.position_covariance[7] = 0.0;
      obstacle.position_covariance[8] = 0.0;

      obstacle.velocity_covariance[0] = covariance[static_cast<int>(itr->second.object_quality.obj_vrellong_rms.data)];
      obstacle.velocity_covariance[1] = 0.0;
      obstacle.velocity_covariance[2] = 0.0;
      obstacle.velocity_covariance[3] = 0.0;
      obstacle.velocity_covariance[4] = covariance[static_cast<int>(itr->second.object_quality.obj_vrellat_rms.data)];
      obstacle.velocity_covariance[5] = 0.0;
      obstacle.velocity_covariance[6] = 0.0;
      obstacle.velocity_covariance[7] = 0.0;
      obstacle.velocity_covariance[8] = 0.0;

      mobject.pose.position = radar_track.position;
      mobject.scale = radar_track.size;

      marker_array.markers.push_back(mobject);
      radar_tracks.tracks.push_back(radar_track);
      obstacle_array.obstacles.push_back(obstacle);
    }

    marker_array_publishers_[sensor_id]->publish(marker_array);
    radar_tracks_publishers_[sensor_id]->publish(radar_tracks);
    obstacle_array_publishers_[sensor_id]->publish(obstacle_array);
  }

  void radar_conti_ars408::setFilterService(
      const std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Request> request,
      std::shared_ptr<radar_conti_ars408_msgs::srv::SetFilter::Response> response)
  {
    auto req = *request;
    // Add small delay so the CAN on Orin does not fault
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!setFilter(req.sensor_id,
                   FilterCfg_FilterCfg_Active_active,
                   FilterCfg_FilterCfg_Valid_valid,
                   req.type, req.index, req.min_value, req.max_value))
    {
      response->success = false;
      return;
    }
    response->success = true;
  }

  bool radar_conti_ars408::setFilter(const int &sensor_id, const int &active, const int &valid, const int &type, const int &index, const int &min_value, const int &max_value)
  {
    polymath::socketcan::CanFrame frame;
    frame.set_len(DLC_FilterCfg);

    uint32_t msg_id = ID_FilterCfg;
    Set_SensorID_In_MsgID(msg_id, sensor_id);
    frame.set_can_id(msg_id);

    std::array<unsigned char, CAN_MAX_DLC> data;
    SET_FilterCfg_FilterCfg_Active(data, active);
    SET_FilterCfg_FilterCfg_Valid(data, valid);
    SET_FilterCfg_FilterCfg_Type(data, type);
    SET_FilterCfg_FilterCfg_Index(data, index);

    switch (filterTypes[index])
    {
    case FilterType::NOFOBJ:
      RCLCPP_DEBUG(this->get_logger(), "Setting Number Of Objects Filter");
      SET_FilterCfg_FilterCfg_Max_NofObj(data, max_value);
      SET_FilterCfg_FilterCfg_Min_NofObj(data, min_value);
      break;
    case FilterType::DISTANCE:
      RCLCPP_DEBUG(this->get_logger(), "Setting Distance Filter");
      SET_FilterCfg_FilterCfg_Max_Distance(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Distance(data, min_value);
      break;
    case FilterType::AZIMUTH:
      RCLCPP_DEBUG(this->get_logger(), "Setting Azimuth Filter");
      SET_FilterCfg_FilterCfg_Max_Azimuth(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Azimuth(data, min_value);
      break;
    case FilterType::VRELONCOME:
      RCLCPP_DEBUG(this->get_logger(), "Setting Oncoming Velocity Filter");
      SET_FilterCfg_FilterCfg_Max_VrelOncome(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VrelOncome(data, min_value);
      break;
    case FilterType::VRELDEPART:
      RCLCPP_DEBUG(this->get_logger(), "Setting Departing Velocity Filter");
      SET_FilterCfg_FilterCfg_Max_VrelDepart(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VrelDepart(data, min_value);
      break;
    case FilterType::RCS:
      RCLCPP_DEBUG(this->get_logger(), "Setting RCS Filter");
      SET_FilterCfg_FilterCfg_Max_RCS(data, max_value);
      SET_FilterCfg_FilterCfg_Min_RCS(data, min_value);
      break;
    case FilterType::LIFETIME:
      RCLCPP_DEBUG(this->get_logger(), "Setting Lifetime Filter");
      SET_FilterCfg_FilterCfg_Max_Lifetime(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Lifetime(data, min_value);
      break;
    case FilterType::SIZE:
      RCLCPP_DEBUG(this->get_logger(), "Setting Size Filter");
      SET_FilterCfg_FilterCfg_Max_Size(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Size(data, min_value);
      break;
    case FilterType::PROBEXISTS:
      RCLCPP_DEBUG(this->get_logger(), "Setting Probability of Existence Filter");
      SET_FilterCfg_FilterCfg_Max_ProbExists(data, max_value);
      SET_FilterCfg_FilterCfg_Min_ProbExists(data, min_value);
      break;
    case FilterType::Y:
      RCLCPP_DEBUG(this->get_logger(), "Setting Y Filter");
      SET_FilterCfg_FilterCfg_Max_Y(data, max_value);
      SET_FilterCfg_FilterCfg_Min_Y(data, min_value);
      break;
    case FilterType::X:
      // TODO: MAKE THIS 13BIT
      RCLCPP_DEBUG(this->get_logger(), "X Filter currently not implemented");
      return false;
    case FilterType::VYRIGHTLEFT:
      RCLCPP_DEBUG(this->get_logger(), "Setting Right Left Filter");
      SET_FilterCfg_FilterCfg_Max_VYRightLeft(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VYRightLeft(data, min_value);
      break;
    case FilterType::VXONCOME:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Oncoming Filter");
      SET_FilterCfg_FilterCfg_Max_VXOncome(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VXOncome(data, min_value);
      break;
    case FilterType::VYLEFTRIGHT:
      RCLCPP_DEBUG(this->get_logger(), "Setting Left Right Filter");
      SET_FilterCfg_FilterCfg_Max_VYLeftRight(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VYLeftRight(data, min_value);
      break;
    case FilterType::VXDEPART:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Departing Filter");
      SET_FilterCfg_FilterCfg_Max_VXDepart(data, max_value);
      SET_FilterCfg_FilterCfg_Min_VXDepart(data, min_value);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown Filter Index");
      return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "valid: %i", valid);
    RCLCPP_DEBUG(this->get_logger(), "active: %i", active);
    RCLCPP_DEBUG(this->get_logger(), "min_value is: %i", min_value);
    RCLCPP_DEBUG(this->get_logger(), "max_value is: %i", max_value);

    frame.set_data(data);
    auto err = socketcan_adapter_->send(frame);
    if (err.has_value())
    {
      RCLCPP_ERROR(this->get_logger(), "Error sending frame: %s", err.value().c_str());
      return false;
    }

    return true;
  }

  void radar_conti_ars408::updateFilterConfig(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id)
  {
    radar_filter_configs_[sensor_id].header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    radar_filter_configs_[sensor_id].header.frame_id = radar_link_names_[sensor_id];

    radar_filter_configs_[sensor_id].filtercfg_type.data = CALC_FilterState_Cfg_FilterState_Type(GET_FilterState_Cfg_FilterState_Type(frame->get_data()), 1.0);
    int index = CALC_FilterState_Cfg_FilterState_Index(GET_FilterState_Cfg_FilterState_Index(frame->get_data()), 1.0);

    switch (filterTypes[index])
    {
    case FilterType::NOFOBJ:
      radar_filter_configs_[sensor_id].filtercfg_min_nofobj.data = CALC_FilterState_Cfg_FilterState_Min_NofObj(
          GET_FilterState_Cfg_FilterState_Min_NofObj(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_nofobj.data = CALC_FilterState_Cfg_FilterState_Max_NofObj(
          GET_FilterState_Cfg_FilterState_Max_NofObj(frame->get_data()), 1.0);
      break;
    case FilterType::DISTANCE:
      radar_filter_configs_[sensor_id].filtercfg_min_distance.data = CALC_FilterState_Cfg_FilterState_Min_Distance(
          GET_FilterState_Cfg_FilterState_Min_Distance(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_distance.data = CALC_FilterState_Cfg_FilterState_Max_Distance(
          GET_FilterState_Cfg_FilterState_Max_Distance(frame->get_data()), 1.0);
      break;
    case FilterType::AZIMUTH:
      radar_filter_configs_[sensor_id].filtercfg_min_azimuth.data = CALC_FilterState_Cfg_FilterState_Min_Azimuth(
          GET_FilterState_Cfg_FilterState_Min_Azimuth(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_azimuth.data = CALC_FilterState_Cfg_FilterState_Max_Azimuth(
          GET_FilterState_Cfg_FilterState_Max_Azimuth(frame->get_data()), 1.0);
      break;
    case FilterType::VRELONCOME:
      radar_filter_configs_[sensor_id].filtercfg_min_vreloncome.data = CALC_FilterState_Cfg_FilterState_Min_VrelOncome(
          GET_FilterState_Cfg_FilterState_Min_VrelOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vreloncome.data = CALC_FilterState_Cfg_FilterState_Max_VrelOncome(
          GET_FilterState_Cfg_FilterState_Max_VrelOncome(frame->get_data()), 1.0);
      break;
    case FilterType::VRELDEPART:
      radar_filter_configs_[sensor_id].filtercfg_min_vreldepart.data = CALC_FilterState_Cfg_FilterState_Min_VrelDepart(
          GET_FilterState_Cfg_FilterState_Min_VrelDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vreldepart.data = CALC_FilterState_Cfg_FilterState_Max_VrelDepart(
          GET_FilterState_Cfg_FilterState_Max_VrelDepart(frame->get_data()), 1.0);
      break;
    case FilterType::RCS:
      radar_filter_configs_[sensor_id].filtercfg_min_rcs.data = CALC_FilterState_Cfg_FilterState_Min_RCS(
          GET_FilterState_Cfg_FilterState_Min_RCS(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_rcs.data = CALC_FilterState_Cfg_FilterState_Max_RCS(
          GET_FilterState_Cfg_FilterState_Max_RCS(frame->get_data()), 1.0);
      break;
    case FilterType::LIFETIME:
      radar_filter_configs_[sensor_id].filtercfg_min_lifetime.data = CALC_FilterState_Cfg_FilterState_Min_Lifetime(
          GET_FilterState_Cfg_FilterState_Min_Lifetime(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_lifetime.data = CALC_FilterState_Cfg_FilterState_Max_Lifetime(
          GET_FilterState_Cfg_FilterState_Max_Lifetime(frame->get_data()), 1.0);
      break;
    case FilterType::SIZE:
      radar_filter_configs_[sensor_id].filtercfg_min_size.data = CALC_FilterState_Cfg_FilterState_Min_Size(
          GET_FilterState_Cfg_FilterState_Min_Size(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_size.data = CALC_FilterState_Cfg_FilterState_Max_Size(
          GET_FilterState_Cfg_FilterState_Max_Size(frame->get_data()), 1.0);
      break;
    case FilterType::PROBEXISTS:
      radar_filter_configs_[sensor_id].filtercfg_min_probexists.data = CALC_FilterState_Cfg_FilterState_Min_ProbExists(
          GET_FilterState_Cfg_FilterState_Min_ProbExists(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_probexists.data = CALC_FilterState_Cfg_FilterState_Max_ProbExists(
          GET_FilterState_Cfg_FilterState_Max_ProbExists(frame->get_data()), 1.0);
      break;
    case FilterType::Y:
      radar_filter_configs_[sensor_id].filtercfg_min_y.data = CALC_FilterState_Cfg_FilterState_Min_Y(
          GET_FilterState_Cfg_FilterState_Min_Y(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_y.data = CALC_FilterState_Cfg_FilterState_Max_Y(
          GET_FilterState_Cfg_FilterState_Max_Y(frame->get_data()), 1.0);
      break;
    case FilterType::X:
      radar_filter_configs_[sensor_id].filtercfg_min_x.data = CALC_FilterState_Cfg_FilterState_Min_X(
          GET_FilterState_Cfg_FilterState_Min_X(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_x.data = CALC_FilterState_Cfg_FilterState_Max_X(
          GET_FilterState_Cfg_FilterState_Max_X(frame->get_data()), 1.0);
      break;
    case FilterType::VYRIGHTLEFT:
      radar_filter_configs_[sensor_id].filtercfg_min_vyrightleft.data = CALC_FilterState_Cfg_FilterState_Min_VYRightLeft(
          GET_FilterState_Cfg_FilterState_Min_VYRightLeft(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vyrightleft.data = CALC_FilterState_Cfg_FilterState_Max_VYRightLeft(
          GET_FilterState_Cfg_FilterState_Max_VYRightLeft(frame->get_data()), 1.0);
      break;
    case FilterType::VXONCOME:
      radar_filter_configs_[sensor_id].filtercfg_min_vxoncome.data = CALC_FilterState_Cfg_FilterState_Min_VXOncome(
          GET_FilterState_Cfg_FilterState_Min_VXOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vxoncome.data = CALC_FilterState_Cfg_FilterState_Max_VXOncome(
          GET_FilterState_Cfg_FilterState_Max_VXOncome(frame->get_data()), 1.0);
      break;
    case FilterType::VYLEFTRIGHT:
      radar_filter_configs_[sensor_id].filtercfg_min_vyleftright.data = CALC_FilterState_Cfg_FilterState_Min_VYLeftRight(
          GET_FilterState_Cfg_FilterState_Min_VYLeftRight(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vyleftright.data = CALC_FilterState_Cfg_FilterState_Max_VYLeftRight(
          GET_FilterState_Cfg_FilterState_Max_VYLeftRight(frame->get_data()), 1.0);
      break;
    case FilterType::VXDEPART:
      radar_filter_configs_[sensor_id].filtercfg_min_vxdepart.data = CALC_FilterState_Cfg_FilterState_Min_VXDepart(
          GET_FilterState_Cfg_FilterState_Min_VXDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].filtercfg_max_vxdepart.data = CALC_FilterState_Cfg_FilterState_Max_VXDepart(
          GET_FilterState_Cfg_FilterState_Max_VXDepart(frame->get_data()), 1.0);
      break;
    default:
      break;
    }
    // only publish once we have initialized values in the filter config by sending invalid values for a response
    if (filter_config_initialized_list_[sensor_id])
    {
      filter_config_publishers_[sensor_id]->publish(radar_filter_configs_[sensor_id]);
    }
  }

} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_ars408, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_ars408)
