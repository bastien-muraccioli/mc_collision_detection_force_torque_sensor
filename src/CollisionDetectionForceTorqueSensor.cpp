#include "CollisionDetectionForceTorqueSensor.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

CollisionDetectionForceTorqueSensor::~CollisionDetectionForceTorqueSensor() = default;

void CollisionDetectionForceTorqueSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::init called with configuration:\n{}", config.dump(true, true));
}

void CollisionDetectionForceTorqueSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::reset called");
}

void CollisionDetectionForceTorqueSensor::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::before");
}

void CollisionDetectionForceTorqueSensor::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("CollisionDetectionForceTorqueSensor::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration CollisionDetectionForceTorqueSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("CollisionDetectionForceTorqueSensor", mc_plugin::CollisionDetectionForceTorqueSensor)
