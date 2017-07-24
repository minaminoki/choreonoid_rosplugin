#ifndef CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H
#define CNOID_ROS_PLUGIN_BODY_ROS_ITEM_H

#include <cnoid/ControllerItem>

namespace cnoid {

class BodyROSItemImpl;

class BodyROSItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);

    BodyROSItem();
    BodyROSItem(const BodyROSItem& org);
    virtual ~BodyROSItem();

    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    
protected:
    virtual Item* doDuplicate() const override;

    virtual void onPositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    BodyROSItemImpl* impl;
};

}

#endif
