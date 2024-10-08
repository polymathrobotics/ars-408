#include "../include/radar_conti_ars408_component.hpp"
#include "../include/offsets.hpp"
#include "../include/visualization.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <math.h>
#include <fmt/core.h>

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
    node->declare_parameter("odom_topic_name", rclcpp::ParameterValue(""));
    node->declare_parameter("object_list_topic_name", rclcpp::ParameterValue("ars408/objectlist"));
    node->declare_parameter("marker_array_topic_name", rclcpp::ParameterValue("ars408/marker_array"));
    node->declare_parameter("radar_tracks_topic_name", rclcpp::ParameterValue("ars408/radar_tracks"));
    node->declare_parameter("obstacle_array_topic_name", rclcpp::ParameterValue("ars408/obstacle_array"));
    node->declare_parameter("filter_config_topic_name", rclcpp::ParameterValue("ars408/filter_config"));
    node->declare_parameter("radar_state_topic_name", rclcpp::ParameterValue("ars408/radar_state"));

    node->get_parameter("can_channel", can_channel_);
    node->get_parameter("odom_topic_name", odom_topic_name_);
    node->get_parameter("object_list_topic_name", object_list_topic_name_);
    node->get_parameter("marker_array_topic_name", marker_array_topic_name_);
    node->get_parameter("radar_tracks_topic_name", radar_tracks_topic_name_);
    node->get_parameter("obstacle_array_topic_name", obstacle_array_topic_name_);
    node->get_parameter("filter_config_topic_name", filter_config_topic_name_);
    node->get_parameter("radar_state_topic_name", radar_state_topic_name_);

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
        fov_marker_publishers_.push_back(this->create_publisher<visualization_msgs::msg::MarkerArray>(parameter.as_string() + "/fov", qos));
        fov_filter_marker_publishers_.push_back(this->create_publisher<visualization_msgs::msg::Marker>(parameter.as_string() + "/fov_filter", qos));
        radar_tracks_publishers_.push_back(this->create_publisher<radar_msgs::msg::RadarTracks>(parameter.as_string() + "/" + radar_tracks_topic_name_, radar_tracks_qos));
        obstacle_array_publishers_.push_back(this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>(parameter.as_string() + "/" + obstacle_array_topic_name_, radar_tracks_qos));
        filter_config_publishers_.push_back(this->create_publisher<radar_conti_ars408_msgs::msg::FilterStateCfg>(parameter.as_string() + "/" + filter_config_topic_name_, transient_local_qos));
        radar_state_publishers_.push_back(this->create_publisher<radar_conti_ars408_msgs::msg::RadarState>(parameter.as_string() + "/" + radar_state_topic_name_, transient_local_qos));
        object_map_list_.push_back(std::map<int, radar_conti_ars408_msgs::msg::Object>());
        object_list_list_.push_back(radar_conti_ars408_msgs::msg::ObjectList());
        radar_filter_configs_.push_back(radar_conti_ars408_msgs::msg::FilterStateCfg());
        radar_configuration_configs_.emplace(topic_ind, radar_conti_ars408_msgs::msg::RadarConfiguration());
        radar_filter_active_.push_back(std::vector<bool>());
        radar_filter_valid_.push_back(std::vector<bool>());

        std::vector<bool> init_radar_active_values = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        node->declare_parameter(radar_name + ".active", rclcpp::ParameterValue(init_radar_active_values));
        node->get_parameter(radar_name + ".active", radar_filter_active_[topic_ind]);
        std::vector<bool> init_radar_valid_values = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        node->declare_parameter(radar_name + ".valid", rclcpp::ParameterValue(init_radar_valid_values));
        node->get_parameter(radar_name + ".valid", radar_filter_valid_[topic_ind]);

        // Send motion state
        node->declare_parameter(radar_name + ".send_motion", rclcpp::ParameterValue(false));
        node->get_parameter(radar_name + ".send_motion", motion_configs_[topic_ind]);

        radar_link_names_.push_back(parameter.as_string());

        RCLCPP_DEBUG(node->get_logger(), "radar frame is: %s", parameter.as_string().c_str());

        // RADAR CONFIGS

        // NVM Storage
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_store_in_nvm"), 0, radar_configuration_configs_[topic_ind].radarcfg_storeinnvm.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_store_in_nvm_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_storeinnvm_valid.data);

        // Ext Info
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_ext_info"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendextinfo.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_ext_info_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendextinfo_valid.data);

        // Ctrl Relay
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_ctrl_relay"), 0, radar_configuration_configs_[topic_ind].radarcfg_ctrlrelay.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_ctrl_relay_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_ctrlrelay_valid.data);

        // Radar Power
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_radar_power"), START_RadarConfiguration_RadarCfg_RadarPower, radar_configuration_configs_[topic_ind].radarcfg_radarpower.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_radar_power_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_radarpower_valid.data);

        // Send Quality
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_quality"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendquality.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_send_quality_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sendquality_valid.data);

        // Max Distance
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_max_distance"), 0, radar_configuration_configs_[topic_ind].radarcfg_maxdistance.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_max_distance_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_maxdistance_valid.data);

        // Output Type
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_output_type"), 0, radar_configuration_configs_[topic_ind].radarcfg_outputtype.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_output_type_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_outputtype_valid.data);

        // Sensor ID
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_sensor_id"), 0, radar_configuration_configs_[topic_ind].radarcfg_sensorid.data);
        initializeConfig<uint8_t>(radar_name, std::string("radarcfg_sensor_id_valid"), 0, radar_configuration_configs_[topic_ind].radarcfg_sensorid_valid.data);

        // FILTER CONFIGS
        // Initialize Number of Objects Filters
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_min_nofobj"), 0, radar_filter_configs_[topic_ind].nofobj.min);
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_max_nofobj"), 20, radar_filter_configs_[topic_ind].nofobj.max);

        // Initialize Distance Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_distance"), 0.0, radar_filter_configs_[topic_ind].distance.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_distance"), 409.0, radar_filter_configs_[topic_ind].distance.max);

        // Initialize Azimuth Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_azimuth"), -50.0, radar_filter_configs_[topic_ind].azimuth.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_azimuth"), 50.0, radar_filter_configs_[topic_ind].azimuth.max);

        // Initialize Oncoming Velocity Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vreloncome"), 0.0, radar_filter_configs_[topic_ind].vreloncome.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vreloncome"), 128.0, radar_filter_configs_[topic_ind].vreloncome.max);

        // Initialize Departing Velocity Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vreldepart"), 0.0, radar_filter_configs_[topic_ind].vreldepart.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vreldepart"), 128.0, radar_filter_configs_[topic_ind].vreldepart.max);

        // Initialize Radar Cross Section Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_rcs"), -20.0, radar_filter_configs_[topic_ind].rcs.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_rcs"), 30.0, radar_filter_configs_[topic_ind].rcs.max);

        // Initialize Lifetime Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_lifetime"), 0.0, radar_filter_configs_[topic_ind].lifetime.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_lifetime"), 409.0, radar_filter_configs_[topic_ind].lifetime.max);

        // Initialize Size Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_size"), 0.0, radar_filter_configs_[topic_ind].size.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_size"), 102.0, radar_filter_configs_[topic_ind].size.max);

        // Initialize Probability of Existence Filters
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_min_probexists"), 0, radar_filter_configs_[topic_ind].probexists.min);
        initializeConfig<uint32_t>(radar_name, std::string("filtercfg_max_probexists"), 7, radar_filter_configs_[topic_ind].probexists.max);

        // Initialize Y Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_y"), -400.0, radar_filter_configs_[topic_ind].y.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_y"), 400.0, radar_filter_configs_[topic_ind].y.max);

        // Initialize X Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_x"), 0.0, radar_filter_configs_[topic_ind].x.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_x"), 0.0, radar_filter_configs_[topic_ind].x.max);

        // Initialize Y Velocity going left to right Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vyrightleft"), 0.0, radar_filter_configs_[topic_ind].vyrightleft.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vyrightleft"), 128.0, radar_filter_configs_[topic_ind].vyrightleft.max);

        // Initialize X Velocity oncoming Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vxoncome"), 0.0, radar_filter_configs_[topic_ind].vxoncome.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vxoncome"), 128.0, radar_filter_configs_[topic_ind].vxoncome.max);

        // Initialize Y Velocity going right to left Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vyleftright"), 0.0, radar_filter_configs_[topic_ind].vyleftright.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vyleftright"), 128.0, radar_filter_configs_[topic_ind].vyleftright.max);

        // Initialize X Velocity Departing Filters
        initializeConfig<float>(radar_name, std::string("filtercfg_min_vxdepart"), 0.0, radar_filter_configs_[topic_ind].vxdepart.min);
        initializeConfig<float>(radar_name, std::string("filtercfg_max_vxdepart"), 128.0, radar_filter_configs_[topic_ind].vxdepart.max);

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
    constexpr std::chrono::duration<float> recv_timeout{0.1};
    socketcan_adapter_ = std::make_unique<polymath::socketcan::SocketcanAdapter>(can_channel_, recv_timeout);
    object_count = 0.0;
    set_filter_service_ = create_service<radar_conti_ars408_msgs::srv::SetFilter>("/radar_conti_ars408/set_filter", std::bind(&radar_conti_ars408::setFilterService, this, std::placeholders::_1, std::placeholders::_2));
    radar_config_service_ = create_service<radar_conti_ars408_msgs::srv::TriggerSetCfg>("/radar_conti_ars408/set_radar_configuration", std::bind(&radar_conti_ars408::setRadarConfigurationService, this, std::placeholders::_1, std::placeholders::_2));
    if (!odom_topic_name_.empty())
    {
      rclcpp::QoS qos_settings(10);
      qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_name_, qos_settings, std::bind(&radar_conti_ars408::odomCallback, this, std::placeholders::_1));
    }

    generateUUIDTable();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  template <typename T>
  void radar_conti_ars408::initializeConfig(std::string radar_name, std::string config_name, T default_value, T &config)
  {
    auto node = shared_from_this();
    T config_value;
    declare_parameter_with_type(node, radar_name + "." + config_name, default_value);
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
      fov_marker_publishers_[i]->on_activate();
      fov_filter_marker_publishers_[i]->on_activate();
      radar_tracks_publishers_[i]->on_activate();
      obstacle_array_publishers_[i]->on_activate();
      filter_config_publishers_[i]->on_activate();
      radar_state_publishers_[i]->on_activate();
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
    filter_config_timer_->cancel();
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
          min_value = radar_filter_configs_[radar_index].nofobj.min;
          max_value = radar_filter_configs_[radar_index].nofobj.max;
          break;
        case FilterType::DISTANCE:
          min_value = radar_filter_configs_[radar_index].distance.min;
          max_value = radar_filter_configs_[radar_index].distance.max;
          break;
        case FilterType::AZIMUTH:
          min_value = radar_filter_configs_[radar_index].azimuth.min;
          max_value = radar_filter_configs_[radar_index].azimuth.max;
          break;
        case FilterType::VRELONCOME:
          min_value = radar_filter_configs_[radar_index].vreloncome.min;
          max_value = radar_filter_configs_[radar_index].vreloncome.max;
          break;
        case FilterType::VRELDEPART:
          min_value = radar_filter_configs_[radar_index].vreldepart.min;
          max_value = radar_filter_configs_[radar_index].vreldepart.max;
          break;
        case FilterType::RCS:
          min_value = radar_filter_configs_[radar_index].rcs.min;
          max_value = radar_filter_configs_[radar_index].rcs.max;
          break;
        case FilterType::LIFETIME:
          min_value = radar_filter_configs_[radar_index].lifetime.min;
          max_value = radar_filter_configs_[radar_index].lifetime.max;
          break;
        case FilterType::SIZE:
          min_value = radar_filter_configs_[radar_index].size.min;
          max_value = radar_filter_configs_[radar_index].size.max;
          break;
        case FilterType::PROBEXISTS:
          min_value = radar_filter_configs_[radar_index].probexists.min;
          max_value = radar_filter_configs_[radar_index].probexists.max;
          break;
        case FilterType::Y:
          min_value = radar_filter_configs_[radar_index].y.min;
          max_value = radar_filter_configs_[radar_index].y.max;
          break;
        case FilterType::X:
          min_value = radar_filter_configs_[radar_index].x.min;
          max_value = radar_filter_configs_[radar_index].x.max;
          break;
        case FilterType::VYRIGHTLEFT:
          min_value = radar_filter_configs_[radar_index].vyrightleft.min;
          max_value = radar_filter_configs_[radar_index].vyrightleft.max;
          break;
        case FilterType::VXONCOME:
          min_value = radar_filter_configs_[radar_index].vxoncome.min;
          max_value = radar_filter_configs_[radar_index].vxoncome.max;
          break;
        case FilterType::VYLEFTRIGHT:
          min_value = radar_filter_configs_[radar_index].vyleftright.min;
          max_value = radar_filter_configs_[radar_index].vyleftright.max;
          break;
        case FilterType::VXDEPART:
          min_value = radar_filter_configs_[radar_index].vxdepart.min;
          max_value = radar_filter_configs_[radar_index].vxdepart.max;
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

      // TODO(troy): Make timer durations configurable.
      filter_config_timer_ = this->create_wall_timer(
          1s, std::bind(&radar_conti_ars408::publishFilterConfigMetadata, this));

      fov_marker_timer_ = this->create_wall_timer(
          3s, std::bind(&radar_conti_ars408::publishFovMetadata, this));
    }
  }

  void radar_conti_ars408::publishRadarState(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id)
  {
    radar_conti_ars408_msgs::msg::RadarState radar_state_msg;
    radar_state_msg.header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    radar_state_msg.header.frame_id = radar_link_names_[sensor_id];

    radar_state_msg.nvmwritestatus = CALC_RadarState_RadarState_NVMwriteStatus(GET_RadarState_RadarState_NVMwriteStatus(frame->get_data()), 1.0);
    radar_state_msg.nvmreadstatus = CALC_RadarState_RadarState_NVMReadStatus(GET_RadarState_RadarState_NVMReadStatus(frame->get_data()), 1.0);
    radar_state_msg.maxdistancecfg = CALC_RadarState_RadarState_MaxDistanceCfg(GET_RadarState_RadarState_MaxDistanceCfg(frame->get_data()), 1.0);
    radar_state_msg.persistent_error = CALC_RadarState_RadarState_Persistent_Error(GET_RadarState_RadarState_Persistent_Error(frame->get_data()), 1.0);
    radar_state_msg.interference = CALC_RadarState_RadarState_Interference(GET_RadarState_RadarState_Interference(frame->get_data()), 1.0);
    radar_state_msg.temperature_error = CALC_RadarState_RadarState_Temperature_Error(GET_RadarState_RadarState_Temperature_Error(frame->get_data()), 1.0);
    radar_state_msg.temporary_error = CALC_RadarState_RadarState_Temporary_Error(GET_RadarState_RadarState_Temporary_Error(frame->get_data()), 1.0);
    radar_state_msg.voltage_error = CALC_RadarState_RadarState_Voltage_Error(GET_RadarState_RadarState_Voltage_Error(frame->get_data()), 1.0);
    radar_state_msg.radarpowercfg = CALC_RadarState_RadarState_RadarPowerCfg(GET_RadarState_RadarState_RadarPowerCfg(frame->get_data()), 1.0);
    radar_state_msg.sortindex = CALC_RadarState_RadarState_SortIndex(GET_RadarState_RadarState_SortIndex(frame->get_data()), 1.0);
    radar_state_msg.sensorid = CALC_RadarState_RadarState_SensorID(GET_RadarState_RadarState_SensorID(frame->get_data()), 1.0);
    radar_state_msg.motionrxstate = CALC_RadarState_RadarState_MotionRxState(GET_RadarState_RadarState_MotionRxState(frame->get_data()), 1.0);
    radar_state_msg.sendextinfocfg = CALC_RadarState_RadarState_SendExtInfoCfg(GET_RadarState_RadarState_SendExtInfoCfg(frame->get_data()), 1.0);
    radar_state_msg.sendqualitycfg = CALC_RadarState_RadarState_SendQualityCfg(GET_RadarState_RadarState_SendQualityCfg(frame->get_data()), 1.0);
    radar_state_msg.outputtypecfg = CALC_RadarState_RadarState_OutputTypeCfg(GET_RadarState_RadarState_OutputTypeCfg(frame->get_data()), 1.0);
    radar_state_msg.ctrlrelaycfg = CALC_RadarState_RadarState_CtrlRelayCfg(GET_RadarState_RadarState_CtrlRelayCfg(frame->get_data()), 1.0);
    radar_state_msg.rcs_threshold = CALC_RadarState_RadarState_RCS_Threshold(GET_RadarState_RadarState_RCS_Threshold(frame->get_data()), 1.0);

    if (radar_state_publishers_[sensor_id])
    {
      radar_state_publishers_[sensor_id]->publish(radar_state_msg);
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
      publishRadarState(frame, sensor_id);
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
    frame.set_len(CAN_MAX_DLC);

    uint32_t msg_id = ID_FilterCfg;
    Set_SensorID_In_MsgID(msg_id, sensor_id);
    frame.set_can_id(msg_id);

    std::array<unsigned char, CAN_MAX_DLC> data{0};
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
      SET_FilterCfg_FilterCfg_Max_Distance(data, max_value / FilterConfig::DISTANCE_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_Distance(data, min_value / FilterConfig::DISTANCE_RESOLUTION);
      break;
    case FilterType::AZIMUTH:
      RCLCPP_DEBUG(this->get_logger(), "Setting Azimuth Filter");
      SET_FilterCfg_FilterCfg_Max_Azimuth(data, (max_value + FilterConfig::AZIMUTH_OFFSET) / FilterConfig::AZIMUTH_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_Azimuth(data, (min_value + FilterConfig::AZIMUTH_OFFSET) / FilterConfig::AZIMUTH_RESOLUTION);
      break;
    case FilterType::VRELONCOME:
      RCLCPP_DEBUG(this->get_logger(), "Setting Oncoming Velocity Filter");
      SET_FilterCfg_FilterCfg_Max_VrelOncome(data, max_value / FilterConfig::VRELONCOME_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_VrelOncome(data, min_value / FilterConfig::VRELONCOME_RESOLUTION);
      break;
    case FilterType::VRELDEPART:
      RCLCPP_DEBUG(this->get_logger(), "Setting Departing Velocity Filter");
      SET_FilterCfg_FilterCfg_Max_VrelDepart(data, max_value / FilterConfig::VRELDEPART_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_VrelDepart(data, min_value / FilterConfig::VRELDEPART_RESOLUTION);
      break;
    case FilterType::RCS:
      RCLCPP_DEBUG(this->get_logger(), "Setting RCS Filter");
      SET_FilterCfg_FilterCfg_Max_RCS(data, (max_value + FilterConfig::RCS_OFFSET) / FilterConfig::RCS_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_RCS(data, (min_value + FilterConfig::RCS_OFFSET) / FilterConfig::RCS_RESOLUTION);
      break;
    case FilterType::LIFETIME:
      RCLCPP_DEBUG(this->get_logger(), "Setting Lifetime Filter");
      SET_FilterCfg_FilterCfg_Max_Lifetime(data, max_value / FilterConfig::LIFETIME_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_Lifetime(data, min_value / FilterConfig::LIFETIME_RESOLUTION);
      break;
    case FilterType::SIZE:
      RCLCPP_DEBUG(this->get_logger(), "Setting Size Filter");
      SET_FilterCfg_FilterCfg_Max_Size(data, max_value / FilterConfig::SIZE_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_Size(data, min_value / FilterConfig::SIZE_RESOLUTION);
      break;
    case FilterType::PROBEXISTS:
      RCLCPP_DEBUG(this->get_logger(), "Setting Probability of Existence Filter");
      SET_FilterCfg_FilterCfg_Max_ProbExists(data, max_value);
      SET_FilterCfg_FilterCfg_Min_ProbExists(data, min_value);
      break;
    case FilterType::Y:
      RCLCPP_DEBUG(this->get_logger(), "Setting Y Filter");
      SET_FilterCfg_FilterCfg_Max_Y(data, (max_value + FilterConfig::Y_OFFSET) / FilterConfig::Y_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_Y(data, (min_value + FilterConfig::Y_OFFSET) / FilterConfig::Y_RESOLUTION);
      break;
    case FilterType::X:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Filter");
      SET_FilterCfg_FilterCfg_Max_X(data, (max_value + FilterConfig::X_OFFSET) / FilterConfig::X_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_X(data, (min_value + FilterConfig::X_OFFSET) / FilterConfig::X_RESOLUTION);
      break;
    case FilterType::VYRIGHTLEFT:
      RCLCPP_DEBUG(this->get_logger(), "Setting Right Left Filter");
      SET_FilterCfg_FilterCfg_Max_VYRightLeft(data, max_value / FilterConfig::VYRIGHTLEFT_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_VYRightLeft(data, min_value / FilterConfig::VYRIGHTLEFT_RESOLUTION);
      break;
    case FilterType::VXONCOME:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Oncoming Filter");
      SET_FilterCfg_FilterCfg_Max_VXOncome(data, max_value / FilterConfig::VXONCOME_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_VXOncome(data, min_value / FilterConfig::VXONCOME_RESOLUTION);
      break;
    case FilterType::VYLEFTRIGHT:
      RCLCPP_DEBUG(this->get_logger(), "Setting Left Right Filter");
      SET_FilterCfg_FilterCfg_Max_VYLeftRight(data, max_value / FilterConfig::VYLEFTRIGHT_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_VYLeftRight(data, min_value / FilterConfig::VYLEFTRIGHT_RESOLUTION);
      break;
    case FilterType::VXDEPART:
      RCLCPP_DEBUG(this->get_logger(), "Setting X Departing Filter");
      SET_FilterCfg_FilterCfg_Max_VXDepart(data, max_value / FilterConfig::VXDEPART_RESOLUTION);
      SET_FilterCfg_FilterCfg_Min_VXDepart(data, min_value / FilterConfig::VXDEPART_RESOLUTION);
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

  void radar_conti_ars408::setRadarConfigurationService(
      const std::shared_ptr<radar_conti_ars408_msgs::srv::TriggerSetCfg::Request> request,
      std::shared_ptr<radar_conti_ars408_msgs::srv::TriggerSetCfg::Response> response)
  {
    auto req = *request;
    if (!setRadarConfiguration(req.sensor_id, response))
    {
      response->success = false;
      return;
    }
    response->success = true;
  }

  bool radar_conti_ars408::setRadarConfiguration(const int &sensor_id, std::shared_ptr<radar_conti_ars408_msgs::srv::TriggerSetCfg::Response> &response)
  {
    RCLCPP_INFO(this->get_logger(), "Setting radar configuration for sensor_id: %i", sensor_id);
    polymath::socketcan::CanFrame frame;
    frame.set_len(DLC_RadarConfiguration);

    uint32_t msg_id = ID_RadarConfiguration;
    Set_SensorID_In_MsgID(msg_id, sensor_id);
    frame.set_can_id(msg_id);

    std::array<unsigned char, DLC_RadarConfiguration> data{0};

    auto it = radar_configuration_configs_.find(sensor_id);
    if (it == radar_configuration_configs_.end())
    {
      auto msg = fmt::format("Sensor ID '{}' not found in radar configurations.", sensor_id);
      response->message = msg;
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
      return false;
    }
    auto current_radar_config = it->second;

    // RADAR POWER CFG
    if (current_radar_config.radarcfg_radarpower.data < MIN_RadarConfiguration_RadarCfg_RadarPower || current_radar_config.radarcfg_radarpower.data > MAX_RadarConfiguration_RadarCfg_RadarPower)
    {
      std::string msg = fmt::format("Radar Power '{}' outside of range", current_radar_config.radarcfg_radarpower.data);
      response->message = msg;
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
      return false;
    }

    // Radar Power
    SET_RadarConfiguration_RadarCfg_RadarPower_valid(data, current_radar_config.radarcfg_radarpower_valid.data);
    SET_RadarConfiguration_RadarCfg_RadarPower(data, current_radar_config.radarcfg_radarpower.data);

    // NVM Storage
    SET_RadarConfiguration_RadarCfg_StoreInNVM(data, current_radar_config.radarcfg_storeinnvm.data);
    SET_RadarConfiguration_RadarCfg_StoreInNVM_valid(data, current_radar_config.radarcfg_storeinnvm_valid.data);

    // Send Ext Info
    SET_RadarConfiguration_RadarCfg_SendExtInfo(data, current_radar_config.radarcfg_sendextinfo.data);
    SET_RadarConfiguration_RadarCfg_SendExtInfo_valid(data, current_radar_config.radarcfg_sendextinfo_valid.data);

    // Ctrl Relay
    SET_RadarConfiguration_RadarCfg_CtrlRelay(data, current_radar_config.radarcfg_ctrlrelay.data);
    SET_RadarConfiguration_RadarCfg_CtrlRelay_valid(data, current_radar_config.radarcfg_ctrlrelay_valid.data);

    // Send Quality
    SET_RadarConfiguration_RadarCfg_SendQuality(data, current_radar_config.radarcfg_sendquality.data);
    SET_RadarConfiguration_RadarCfg_SendQuality_valid(data, current_radar_config.radarcfg_sendquality_valid.data);

    // Max Distance - Resolution of 2m
    SET_RadarConfiguration_RadarCfg_MaxDistance(data, current_radar_config.radarcfg_maxdistance.data / 2);
    SET_RadarConfiguration_RadarCfg_MaxDistance_valid(data, current_radar_config.radarcfg_maxdistance_valid.data);

    // Output Type
    SET_RadarConfiguration_RadarCfg_OutputType(data, current_radar_config.radarcfg_outputtype.data);
    SET_RadarConfiguration_RadarCfg_OutputType_valid(data, current_radar_config.radarcfg_outputtype_valid.data);

    // Sensor ID
    SET_RadarConfiguration_RadarCfg_SensorID(data, current_radar_config.radarcfg_sensorid.data);
    SET_RadarConfiguration_RadarCfg_SensorID_valid(data, current_radar_config.radarcfg_sensorid_valid.data);

    frame.set_data(data);
    auto err = socketcan_adapter_->send(frame);
    if (err.has_value())
    {
      auto msg = fmt::format("Error sending frame: %s", err.value().c_str());
      response->message = msg;
      RCLCPP_ERROR(this->get_logger(), msg.c_str());

      return false;
    }

    return true;
  }

  void radar_conti_ars408::updateFilterConfig(std::shared_ptr<const polymath::socketcan::CanFrame> frame, const int &sensor_id)
  {
    radar_filter_configs_[sensor_id].header.stamp = rclcpp_lifecycle::LifecycleNode::now();
    radar_filter_configs_[sensor_id].header.frame_id = radar_link_names_[sensor_id];

    int index = CALC_FilterState_Cfg_FilterState_Index(GET_FilterState_Cfg_FilterState_Index(frame->get_data()), 1.0);

    switch (filterTypes[index])
    {
    case FilterType::NOFOBJ:
      radar_filter_configs_[sensor_id].nofobj.min = CALC_FilterState_Cfg_FilterState_Min_NofObj(
          GET_FilterState_Cfg_FilterState_Min_NofObj(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].nofobj.max = CALC_FilterState_Cfg_FilterState_Max_NofObj(
          GET_FilterState_Cfg_FilterState_Max_NofObj(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].nofobj.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::DISTANCE:
      radar_filter_configs_[sensor_id].distance.min = CALC_FilterState_Cfg_FilterState_Min_Distance(
          GET_FilterState_Cfg_FilterState_Min_Distance(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].distance.max = CALC_FilterState_Cfg_FilterState_Max_Distance(
          GET_FilterState_Cfg_FilterState_Max_Distance(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].distance.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::AZIMUTH:
      radar_filter_configs_[sensor_id].azimuth.min = CALC_FilterState_Cfg_FilterState_Min_Azimuth(
          GET_FilterState_Cfg_FilterState_Min_Azimuth(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].azimuth.max = CALC_FilterState_Cfg_FilterState_Max_Azimuth(
          GET_FilterState_Cfg_FilterState_Max_Azimuth(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].azimuth.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VRELONCOME:
      radar_filter_configs_[sensor_id].vreloncome.min = CALC_FilterState_Cfg_FilterState_Min_VrelOncome(
          GET_FilterState_Cfg_FilterState_Min_VrelOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreloncome.max = CALC_FilterState_Cfg_FilterState_Max_VrelOncome(
          GET_FilterState_Cfg_FilterState_Max_VrelOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreloncome.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VRELDEPART:
      radar_filter_configs_[sensor_id].vreldepart.min = CALC_FilterState_Cfg_FilterState_Min_VrelDepart(
          GET_FilterState_Cfg_FilterState_Min_VrelDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreldepart.max = CALC_FilterState_Cfg_FilterState_Max_VrelDepart(
          GET_FilterState_Cfg_FilterState_Max_VrelDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vreldepart.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::RCS:
      radar_filter_configs_[sensor_id].rcs.min = CALC_FilterState_Cfg_FilterState_Min_RCS(
          GET_FilterState_Cfg_FilterState_Min_RCS(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].rcs.max = CALC_FilterState_Cfg_FilterState_Max_RCS(
          GET_FilterState_Cfg_FilterState_Max_RCS(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].rcs.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::LIFETIME:
      radar_filter_configs_[sensor_id].lifetime.min = CALC_FilterState_Cfg_FilterState_Min_Lifetime(
          GET_FilterState_Cfg_FilterState_Min_Lifetime(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].lifetime.max = CALC_FilterState_Cfg_FilterState_Max_Lifetime(
          GET_FilterState_Cfg_FilterState_Max_Lifetime(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].lifetime.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::SIZE:
      radar_filter_configs_[sensor_id].size.min = CALC_FilterState_Cfg_FilterState_Min_Size(
          GET_FilterState_Cfg_FilterState_Min_Size(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].size.max = CALC_FilterState_Cfg_FilterState_Max_Size(
          GET_FilterState_Cfg_FilterState_Max_Size(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].size.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::PROBEXISTS:
      radar_filter_configs_[sensor_id].probexists.min = CALC_FilterState_Cfg_FilterState_Min_ProbExists(
          GET_FilterState_Cfg_FilterState_Min_ProbExists(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].probexists.max = CALC_FilterState_Cfg_FilterState_Max_ProbExists(
          GET_FilterState_Cfg_FilterState_Max_ProbExists(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].probexists.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::Y:
      radar_filter_configs_[sensor_id].y.min = CALC_FilterState_Cfg_FilterState_Min_Y(
          GET_FilterState_Cfg_FilterState_Min_Y(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].y.max = CALC_FilterState_Cfg_FilterState_Max_Y(
          GET_FilterState_Cfg_FilterState_Max_Y(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].y.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::X:
      radar_filter_configs_[sensor_id].x.min = CALC_FilterState_Cfg_FilterState_Min_X(
          GET_FilterState_Cfg_FilterState_Min_X(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].x.max = CALC_FilterState_Cfg_FilterState_Max_X(
          GET_FilterState_Cfg_FilterState_Max_X(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].x.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VYRIGHTLEFT:
      radar_filter_configs_[sensor_id].vyrightleft.min = CALC_FilterState_Cfg_FilterState_Min_VYRightLeft(
          GET_FilterState_Cfg_FilterState_Min_VYRightLeft(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyrightleft.max = CALC_FilterState_Cfg_FilterState_Max_VYRightLeft(
          GET_FilterState_Cfg_FilterState_Max_VYRightLeft(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyrightleft.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VXONCOME:
      radar_filter_configs_[sensor_id].vxoncome.min = CALC_FilterState_Cfg_FilterState_Min_VXOncome(
          GET_FilterState_Cfg_FilterState_Min_VXOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxoncome.max = CALC_FilterState_Cfg_FilterState_Max_VXOncome(
          GET_FilterState_Cfg_FilterState_Max_VXOncome(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxoncome.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VYLEFTRIGHT:
      radar_filter_configs_[sensor_id].vyleftright.min = CALC_FilterState_Cfg_FilterState_Min_VYLeftRight(
          GET_FilterState_Cfg_FilterState_Min_VYLeftRight(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyleftright.max = CALC_FilterState_Cfg_FilterState_Max_VYLeftRight(
          GET_FilterState_Cfg_FilterState_Max_VYLeftRight(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vyleftright.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    case FilterType::VXDEPART:
      radar_filter_configs_[sensor_id].vxdepart.min = CALC_FilterState_Cfg_FilterState_Min_VXDepart(
          GET_FilterState_Cfg_FilterState_Min_VXDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxdepart.max = CALC_FilterState_Cfg_FilterState_Max_VXDepart(
          GET_FilterState_Cfg_FilterState_Max_VXDepart(frame->get_data()), 1.0);
      radar_filter_configs_[sensor_id].vxdepart.active = GET_FilterState_Cfg_FilterState_Active(frame->get_data());
      break;
    default:
      break;
    }
  }

  void radar_conti_ars408::publishFilterConfigMetadata()
  {
    for (size_t sensor_id = 0; sensor_id < radar_filter_configs_.size(); ++sensor_id)
    {

      // only publish once we have initialized values in the filter config by sending invalid values for a response
      if (!filter_config_initialized_list_[sensor_id])
      {
        continue;
      }

      filter_config_publishers_[sensor_id]->publish(radar_filter_configs_[sensor_id]);

      if (radar_filter_configs_[sensor_id].azimuth.active && radar_filter_configs_[sensor_id].distance.active)
      {
        auto fov_filter_markers = radar_visualization::createFilteredRadarFOVMarker(radar_link_names_[sensor_id], radar_filter_configs_[sensor_id].azimuth.min, radar_filter_configs_[sensor_id].azimuth.max, radar_filter_configs_[sensor_id].distance.min, radar_filter_configs_[sensor_id].distance.max, this->get_clock());
        fov_filter_marker_publishers_[sensor_id]->publish(fov_filter_markers);
      }
    }
  }

  void radar_conti_ars408::publishFovMetadata()
  {
    for (size_t sensor_id = 0; sensor_id < radar_filter_configs_.size(); ++sensor_id)
    {

      // only publish once we have initialized values in the filter config by sending invalid values for a response
      if (!filter_config_initialized_list_[sensor_id])
      {
        continue;
      }

      auto fov_markers = radar_visualization::createRadarFOVMarkers(
          radar_link_names_[sensor_id],
          this->get_clock());
      fov_marker_publishers_[sensor_id]->publish(fov_markers);
    }
  }

  void radar_conti_ars408::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Threshold for standstill detection
    const double standstill_threshold = 0.01;

    double velocity_x = msg->twist.twist.linear.x;
    // Calculate speed in m/s
    double speed = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);

    // Calculate direction (0x0 standstill, 0x1 forward, 0x2 backward)
    uint8_t direction = 0x0; // Default to standstill
    if (std::abs(velocity_x) > standstill_threshold)
    {
      direction = (velocity_x > 0) ? 0x1 : 0x2; // Forward or backward
    }

    // Calculate yaw rate in deg/s (angular velocity around the z-axis)
    double yaw_rate_rad_per_sec = msg->twist.twist.angular.z;
    double yaw_rate_deg_per_sec = yaw_rate_rad_per_sec * (180.0 / M_PI);

    for (auto &motion_config : motion_configs_)
    {
      auto sensor_id = motion_config.first;
      auto enable = motion_config.second;
      if (enable)
      {
        sendMotionInputSignals(sensor_id, direction, speed, yaw_rate_deg_per_sec);
      }
    }
  }

  void radar_conti_ars408::sendMotionInputSignals(const size_t &sensor_id, uint8_t direction, double speed, double yaw_rate)
  {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Sending motion input");
    // Set up speed frame
    polymath::socketcan::CanFrame speed_frame;
    speed_frame.set_len(DLC_SpeedInformation);
    uint32_t speed_msg_id = ID_SpeedInformation;
    Set_SensorID_In_MsgID(speed_msg_id, sensor_id);
    speed_frame.set_can_id(speed_msg_id);
    std::array<unsigned char, CAN_MAX_DLC> speed_data{0};

    SET_SpeedInformation_RadarDevice_Speed(speed_data, speed / SpeedInformation::SPEED_RESOLUTION);
    // TODO(troy): Use odom tf to see if the radar is mounted backwards or forwards
    SET_SpeedInformation_RadarDevice_SpeedDirection(speed_data, direction);

    speed_frame.set_data(speed_data);
    auto speed_err = socketcan_adapter_->send(speed_frame);
    if (speed_err.has_value())
    {
      auto msg = fmt::format("Error sending speed frame: %s", speed_err.value().c_str());
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
    }

    // Set up yaw rate frame
    polymath::socketcan::CanFrame yaw_rate_frame;
    yaw_rate_frame.set_len(DLC_YawRateInformation);
    uint32_t yaw_rate_msg_id = ID_YawRateInformation;
    Set_SensorID_In_MsgID(yaw_rate_msg_id, sensor_id);
    yaw_rate_frame.set_can_id(yaw_rate_msg_id);
    std::array<unsigned char, CAN_MAX_DLC> yaw_rate_info_data{0};

    SET_YawRateInformation_RadarDevice_YawRate(yaw_rate_info_data, (yaw_rate + YawRateInformation::YAW_RATE_OFFSET) / YawRateInformation::YAW_RATE_RESOLUTION);

    yaw_rate_frame.set_data(yaw_rate_info_data);
    auto yaw_err = socketcan_adapter_->send(yaw_rate_frame);
    if (yaw_err.has_value())
    {
      auto msg = fmt::format("Error sending yaw frame: %s", yaw_err.value().c_str());
      RCLCPP_ERROR(this->get_logger(), msg.c_str());
    }
  }

} // end namespace

#include "rclcpp_components/register_node_macro.hpp"
// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// CLASS_LOADER_REGISTER_CLASS(FHAC::radar_conti_ars408, rclcpp_lifecycle::LifecycleNode)

RCLCPP_COMPONENTS_REGISTER_NODE(FHAC::radar_conti_ars408)
