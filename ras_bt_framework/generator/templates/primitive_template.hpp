// this file is generated using primitive generator template

#pragma once

#include "ras_bt_framework/PrimitiveBehavior.hpp"

namespace ras_bt_framework
{

class ${class_name} : public PrimitiveBehavior {
public:
    ${class_name}(const std::string& name, const BT::NodeConfig& config)
    : PrimitiveBehavior(name, config)
    {}

    ~${class_name}() {}
    
    static BT::PortsList providedPorts()
    {
        return { ${provided_ports} };
    }

   virtual BT::NodeStatus tick() override {
    std::cout << ("${class_name}") << std::endl;
    // add your code here

    return BT::NodeStatus::SUCCESS;
   }

private:
};

}
