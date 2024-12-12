#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

class SaySomething : public PrimitiveBehavior
{
public:
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : PrimitiveBehavior(name, config)
  {}

  BT::NodeStatus tick() override
  {
    auto msg = getInput<std::string>("message");
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  };

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }
};
    
}