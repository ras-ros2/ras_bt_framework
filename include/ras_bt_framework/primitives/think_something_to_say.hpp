#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

class ThinkSomethingToSay : public PrimitiveBehavior
{
public:
  ThinkSomethingToSay(const std::string& name, const BT::NodeConfig& config)
    : PrimitiveBehavior(name, config)
  {}

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("reference");

    setOutput<std::string>("message", msg.value());
    return BT::NodeStatus::SUCCESS;
  };

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("reference"),BT::OutputPort<std::string>("message") };
  }
};
    
}