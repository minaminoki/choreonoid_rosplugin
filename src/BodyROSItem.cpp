#include "BodyROSItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/TimeBar>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace cnoid;

namespace {

class BodyNode
{
public:
    std::unique_ptr<ros::NodeHandle> rosNode;
    BodyItem* bodyItem;
    TimeBar* timeBar;
    ScopedConnectionSet connections;
    sensor_msgs::JointState jointState;
    ros::Publisher jointStatePublisher;

    BodyNode(BodyItem* bodyItem);
    void initializeJointState();
    void publishJointState();    
};

}

namespace cnoid {

class BodyROSItemImpl
{
public:
    BodyROSItem* self;
    unique_ptr<BodyNode> bodyNode;

    BodyROSItemImpl(BodyROSItem* self);
    ~BodyROSItemImpl();
    void setBodyItem(BodyItem* bodyItem, bool forceUpdate);
};

}


void BodyROSItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyROSItem>("BodyROSItem");
    ext->itemManager().addCreationPanel<BodyROSItem>();
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
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyROSItemImpl::setBodyItem(BodyItem* bodyItem, bool forceUpdate)
{
    if(bodyNode){
        if(forceUpdate || bodyItem != bodyNode->bodyItem){
            bodyNode.reset();
        }
    }
    if(bodyItem){
        bodyNode.reset(new BodyNode(bodyItem));

        bodyNode->connections.add(
            bodyItem->sigNameChanged().connect(
                [&](const std::string& oldName){ setBodyItem(bodyItem, true); }));
    }
}


void BodyROSItem::onDisconnectedFromRoot()
{
    impl->bodyNode.reset();
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


BodyNode::BodyNode(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      timeBar(TimeBar::instance())
{
    string name = bodyItem->name();
    std::replace(name.begin(), name.end(), '-', '_');

    rosNode.reset(new ros::NodeHandle(name));

    jointStatePublisher = rosNode->advertise<sensor_msgs::JointState>("joint_state", 1000);

    if(jointStatePublisher){
        initializeJointState();
        publishJointState();
        connections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ publishJointState(); }));
    }
}


void BodyNode::initializeJointState()
{
    Body* body = bodyItem->body();
    const int n = body->numJoints();
    jointState.name.resize(n);
    jointState.position.resize(n);
    jointState.velocity.resize(n);
    jointState.effort.resize(n);
    for(int i=0; i < n; ++i){
        jointState.name[i] = body->joint(i)->name();
    }
}
    

void BodyNode::publishJointState()
{
    jointState.header.stamp.fromSec(timeBar->time());

    Body* body = bodyItem->body();
    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        jointState.position[i] = joint->q();
        jointState.velocity[i] = joint->dq();
        jointState.effort[i] = joint->u();
    }

    jointStatePublisher.publish(jointState);
}
