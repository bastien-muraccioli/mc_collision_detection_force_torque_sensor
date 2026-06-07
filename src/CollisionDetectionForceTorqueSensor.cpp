#include "CollisionDetectionForceTorqueSensor.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CollisionDetectionForceTorqueSensor::~CollisionDetectionForceTorqueSensor() = default;

void CollisionDetectionForceTorqueSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();

  // Make sure to have obstacle detection datastore entry
  if(!ctl.controller().datastore().has("Obstacle detected"))
  {
    ctl.controller().datastore().make<bool>("Obstacle detected", false);
  }

  // Get plugin configuration
  auto plugin_config = config("collision_detection_force_torque_sensor");
  forceTorqueSensorName_ = "EEForceSensor";
  if(config.has("forceTorqueSensorName"))
  {
    forceTorqueSensorName_.assign(config("forceTorqueSensorName"));
  }
  threshold_filtering_ = plugin_config("threshold_filtering", 0.05);
  threshold_offset_ = plugin_config("threshold_offset");
  if(threshold_offset_.size() != 6)
  {
    threshold_offset_ = Eigen::Vector6d::Constant(1.0);
  }
  lpf_threshold_.setValues(threshold_offset_, threshold_filtering_, 6);

  // Check if the robot has the specified force/torque sensor
  if(!robot.hasForceSensor(forceTorqueSensorName_))
  {
    mc_rtc::log::error("[CollisionDetectionForceTorqueSensor] The force/torque sensor {} does not exist in the robot", forceTorqueSensorName_);
    return;
  }

  forceTorque_ = Eigen::Vector6d::Zero();

  addGui(ctl);
  addLog(ctl);
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::init called with configuration:\n{}", config.dump(true, true));
}

void CollisionDetectionForceTorqueSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::reset called");
}

void CollisionDetectionForceTorqueSensor::before(mc_control::MCGlobalController & controller)
{
  // mc_rtc::log::info("CollisionDetectionForceTorqueSensor::before");
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  counter_ += ctl.timestep();
  if(activate_plot_ & !plot_added_)
  {
    addPlot(ctl);
    plot_added_ = true;
  }
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());
  forceTorque_ = realRobot.forceSensor(forceTorqueSensorName_).wrenchWithoutGravity(realRobot).vector();
  threshold_high_ = lpf_threshold_.adaptiveThreshold(forceTorque_, true);
  threshold_low_ = lpf_threshold_.adaptiveThreshold(forceTorque_, false);

  obstacle_detected_ = false;
  // For each axis
  for(int i = 0; i < 3; i++)
  {
    if(forceTorque_(i) > threshold_high_(i) || forceTorque_(i) < threshold_low_(i))
    {
      obstacle_detected_ = true;
      if(activate_verbose_)
        mc_rtc::log::info("[CollisionDetectionForceTorque] Obstacle detected on axis {}", i);
    }
  }
  if(collision_stop_activated_)
  {
    ctl.controller().datastore().get<bool>("Obstacle detected") = obstacle_detected_;
  }
}

void CollisionDetectionForceTorqueSensor::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CollisionDetectionForceTorqueSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void CollisionDetectionForceTorqueSensor::addGui(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  ctl.controller().gui()->addElement(
      {"Plugins", "CollisionDetectionForceTorqueSensor"},
      mc_rtc::gui::NumberInput(
          "threshold_filtering", [this]() { return threshold_filtering_; },
          [this](double threshold_filtering)
          {
            this->threshold_filtering_ = threshold_filtering;
            lpf_threshold_.setFiltering(threshold_filtering);
          }),
      mc_rtc::gui::ArrayInput(
          "threshold_offset", {"τx", "τy", "τz", "Fx", "Fy", "Fz"}, [this]() { return this->threshold_offset_; },
          [this](const Eigen::Vector6d & offset)
          {
            threshold_offset_ = offset;
            lpf_threshold_.setOffset(threshold_offset_);
          }),
      mc_rtc::gui::IntegerInput(
          "Axis shown", [this]() { return axis_shown_; }, [this](int axis) { this->axis_shown_ = axis; }),
      mc_rtc::gui::Checkbox("Collision stop", collision_stop_activated_),
      mc_rtc::gui::Checkbox("Verbose", activate_verbose_),
      mc_rtc::gui::Button("Add plot", [this]() { return activate_plot_ = true; }));
}

void CollisionDetectionForceTorqueSensor::addLog(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  ctl.controller().logger().addLogEntry("CollisionDetectionForceTorqueSensor_forceTorque",
                                        [this]() { return forceTorque_; });
  ctl.controller().logger().addLogEntry("CollisionDetectionForceTorqueSensor_forceTorque_norm",
                                        [this]() { return forceTorque_.norm(); });
}

void CollisionDetectionForceTorqueSensor::addPlot(mc_control::MCGlobalController & ctl)
{
  auto & gui = *ctl.controller().gui();
  gui.addPlot(
      "forceTorque", mc_rtc::gui::plot::X("t", [this]() { return counter_; }),
      mc_rtc::gui::plot::Y(
          "forceTorque", [this]() { return forceTorque_[axis_shown_]; }, mc_rtc::gui::Color::Red),
      mc_rtc::gui::plot::Y(
          "threshold_high", [this]() { return threshold_high_[axis_shown_]; }, mc_rtc::gui::Color::Gray),
      mc_rtc::gui::plot::Y(
          "threshold_low", [this]() { return threshold_low_[axis_shown_]; }, mc_rtc::gui::Color::Gray));
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionDetectionForceTorqueSensor", mc_plugin::CollisionDetectionForceTorqueSensor)
