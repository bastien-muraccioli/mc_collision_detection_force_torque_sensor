/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include "LpfThreshold.h"

namespace mc_plugin
{

struct CollisionDetectionForceTorqueSensor : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  void addGui(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);
  void addPlot(mc_control::MCGlobalController & controller);

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~CollisionDetectionForceTorqueSensor() override;

private:

  // GUI
  double counter_ = 0.0;
  int axis_shown_ = 0; // x, y, z
  bool activate_plot_ = false;
  bool plot_added_ = false;
  bool collision_stop_activated_ = false;
  bool obstacle_detected_ = false;
  bool activate_verbose_ = true;

  // Threshold base method
  LpfThreshold lpf_threshold_;
  Eigen::Vector6d threshold_offset_;
  double threshold_filtering_;
  Eigen::Vector6d threshold_high_;
  Eigen::Vector6d threshold_low_;

  std::string forceTorqueSensorName_; // Name of the force/torque sensor
  Eigen::Vector6d forceTorque_;

};

} // namespace mc_plugin
