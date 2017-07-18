#include <cnoid/Plugin>

using namespace cnoid;

class ROSPlugin : public Plugin
{
public:
    ROSPlugin() : Plugin("ROS") { }
  
    virtual bool initialize()
    {
        return true;
    }
};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ROSPlugin)

