#include "BodyROSItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>

using namespace std;
using namespace cnoid;

void BodyROSItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyROSItem>("BodyROSItem");
    ext->itemManager().addCreationPanel<BodyROSItem>();
}

namespace cnoid {

class BodyROSItemImpl
{
public:
    BodyROSItem* self;
    BodyItem* bodyItem;
    ScopedConnectionSet bodyItemConnections;
    std::shared_ptr<ros::NodeHandle> rosNode;

    BodyROSItemImpl(BodyROSItem* self);
    ~BodyROSItemImpl();
    void updateBodyItem(BodyItem* item);
};

}


BodyROSItem::BodyROSItem()
{
    impl = new BodyROSItemImpl(this);
}


BodyROSItem::BodyROSItem(const BodyROSItem& org)
    : ControllerItem(org)
{
    impl = new BodyROSItemImpl(this);
}
    

BodyROSItemImpl::BodyROSItemImpl(BodyROSItem* self)
    : self(self)
{
    bodyItem =0;
}


BodyROSItem::~BodyROSItem()
{
    delete impl;
}


BodyROSItemImpl::~BodyROSItemImpl()
{

}


Item* BodyROSItem::doDuplicate() const
{
    return new BodyROSItem(*this);
}


void BodyROSItem::onPositionChanged()
{
    auto newBodyItem = findOwnerItem<BodyItem>();
    if(newBodyItem != impl->bodyItem){
        impl->updateBodyItem(newBodyItem);
    }
}


void BodyROSItem::onDisconnectedFromRoot()
{

}


void BodyROSItemImpl::updateBodyItem(BodyItem* item)
{
    bodyItem = item;
    bodyItemConnections.disconnect();

    if(!bodyItem){
        rosNode.reset();

    } else {
        string name = bodyItem->name();
        std::replace(name.begin(), name.end(), '-', '_');
        rosNode = make_shared<ros::NodeHandle>(name);

        bodyItemConnections.add(
            bodyItem->sigNameChanged().connect(
                [&](const std::string& oldName){ updateBodyItem(bodyItem); }));
    }

}


double BodyROSItem::timeStep() const
{
    return 0.0;
}


void BodyROSItem::input()
{

}


bool BodyROSItem::control()
{

}


void BodyROSItem::output()
{

}


void BodyROSItem::doPutProperties(PutPropertyFunction& putProperty)
{

}


bool BodyROSItem::store(Archive& archive)
{
    return true;
}


bool BodyROSItem::restore(const Archive& archive)
{
    return true;
}
