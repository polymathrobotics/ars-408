#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do \
                           // this in one cpp file
#include <math.h>

#include <catch2/catch.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "radar_conti_ars408_component.hpp"
#include <radar_conti_ars408_msgs/srv/trigger_set_cfg.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "socketcan_adapter/socketcan_adapter.hpp"

struct ROS2Fixture
{
  ROS2Fixture() { rclcpp::init(0, nullptr); }
  ~ROS2Fixture() { rclcpp::shutdown(); }
};

class ROSTestWrapper
{
public:
  ROSTestWrapper() {}

  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spin_thread;
  std::shared_ptr<FHAC::radar_conti_ars408> node;
  std::unique_ptr<polymath::socketcan::SocketcanAdapter> socketcan_adapter_;
  std::vector<std::unique_ptr<const polymath::socketcan::CanFrame>> frames;

  void Setup(const int radar_power, const int radar_power_valid)
  {
    rclcpp::NodeOptions node_options;
    node_options.parameter_overrides({{"can_channel", "vcan0"}, {"radar_0.link_name", "link_0"}, {"radar_0.radarcfg_radar_power", radar_power}, {"radar_0.radarcfg_radar_power_valid", radar_power_valid}});
    node = std::make_shared<FHAC::radar_conti_ars408>(node_options);

    executor.add_node(node->get_node_base_interface());

    const std::chrono::duration<float> recv_timeout{10};
    socketcan_adapter_ = std::make_unique<polymath::socketcan::SocketcanAdapter>("vcan0", recv_timeout);
    auto cb = [this](std::unique_ptr<const polymath::socketcan::CanFrame> frame)
    {
      this->frames.push_back(std::move(frame));
    };

    socketcan_adapter_->setOnReceiveCallback(std::move(cb));
    REQUIRE(socketcan_adapter_->openSocket());
    REQUIRE(socketcan_adapter_->startReceptionThread());

    // Lifecycle Setup
    node->configure();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->activate();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    spin_thread = std::thread([&]
                              { executor.spin(); });
  }

  void Teardown()
  {

    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    node->deactivate();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    node->cleanup();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

    node->shutdown();
    REQUIRE(node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);

    executor.remove_node(node->get_node_base_interface());
    executor.cancel();

    if (spin_thread.joinable())
    {
      spin_thread.join();
    }

    socketcan_adapter_->closeSocket();
  }
};

TEST_CASE_METHOD(ROS2Fixture, "Continental Radar Configuration", "[ars408]")
{
  SECTION("Happy path for radar configuration")
  {
    try
    {
      ROSTestWrapper test_wrapper;
      test_wrapper.Setup(0, 1);

      // Create a client for the service
      auto client = test_wrapper.node->create_client<radar_conti_ars408_msgs::srv::TriggerSetCfg>("/radar_conti_ars408/set_radar_configuration");

      // Wait until the service is available
      REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

      auto start_request = std::make_shared<radar_conti_ars408_msgs::srv::TriggerSetCfg::Request>();
      start_request->sensor_id = 0;

      bool service_called = false;
      auto start_request_future = client->async_send_request(start_request, [&](rclcpp::Client<radar_conti_ars408_msgs::srv::TriggerSetCfg>::SharedFuture response)
                                                             {
                                                               service_called = true;
                                                               REQUIRE(response.get()->success == true); });

      // Allow the executor to process callbacks, including the service response
      auto start = std::chrono::steady_clock::now();
      while (!service_called && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(5))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      REQUIRE(service_called);
      REQUIRE(test_wrapper.frames.size() > 1);
      std::array<unsigned char, DLC_RadarConfiguration> radar_config_frame = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      REQUIRE(test_wrapper.frames.back()->get_data() == radar_config_frame);

      test_wrapper.Teardown();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Test"), "Caught exception during teardown: %s", e.what());
      REQUIRE(false); // Fail the test if an exception occurs
    }
  }

  SECTION("Outside of radar power range")
  {
    try
    {
      ROSTestWrapper test_wrapper;
      test_wrapper.Setup(-1, 1);

      // Create a client for the service
      auto client = test_wrapper.node->create_client<radar_conti_ars408_msgs::srv::TriggerSetCfg>("/radar_conti_ars408/set_radar_configuration");

      // Wait until the service is available
      REQUIRE(client->wait_for_service(std::chrono::seconds(1)));

      auto start_request = std::make_shared<radar_conti_ars408_msgs::srv::TriggerSetCfg::Request>();
      start_request->sensor_id = 0;

      bool service_called = false;
      auto start_request_future = client->async_send_request(start_request, [&](rclcpp::Client<radar_conti_ars408_msgs::srv::TriggerSetCfg>::SharedFuture response)
                                                             {
                                                               service_called = true;
                                                               REQUIRE(response.get()->success == false);
                                                               REQUIRE(response.get()->message == "Radar Power '255' outside of range"); });

      auto start = std::chrono::steady_clock::now();
      while (!service_called && (std::chrono::steady_clock::now() - start) < std::chrono::seconds(5))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      REQUIRE(service_called);
      test_wrapper.Teardown();
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Test"), "Caught exception during teardown: %s", e.what());
      REQUIRE(false); // Fail the test if an exception occurs
    }
  }
}
