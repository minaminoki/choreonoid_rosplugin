#include "BodyPublisherItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyItem>
#include <cnoid/TimeBar>
#include <cnoid/Archive>
#include <cnoid/ConnectionSet>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <memory>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyNode
{
public:
    unique_ptr<ros::NodeHandle> rosNode;
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;
    BodyItem* bodyItem;
    ScopedConnectionSet connections;
    ScopedConnection connectionOfKinematicStateChange;
    TimeBar* timeBar;

    BodyNode(BodyItem* bodyItem);
    void startToPublishKinematicStateChangeOnGUI();
    void stopToPublishKinematicStateChangeOnGUI();
    void initializeJointState(Body* body);
    void publishJointState(Body* body, double time);
};

}

namespace cnoid {

class BodyPublisherItemImpl
{
public:
    BodyPublisherItem* self;
    unique_ptr<BodyNode> bodyNode;
    ControllerIO* io;
    Body* ioBody;
    double time;
    double timeToPublishNext;
    double maxPublishRate;
    double minPublishCycle;
    double timeStep;
    
    BodyPublisherItemImpl(BodyPublisherItem* self);
    BodyPublisherItemImpl(BodyPublisherItem* self, const BodyPublisherItemImpl& org);
    ~BodyPublisherItemImpl();
    void setBodyItem(BodyItem* bodyItem, bool forceUpdate);
    bool start();
    void input();
};

}


void BodyPublisherItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<BodyPublisherItem>("BodyPublisherItem");
    ext->itemManager().addCreationPanel<BodyPublisherItem>();
}


BodyPublisherItem::BodyPublisherItem()
{
    impl = new BodyPublisherItemImpl(this);
}


BodyPublisherItemImpl::BodyPublisherItemImpl(BodyPublisherItem* self)
    : self(self)
{
    io = nullptr;
    maxPublishRate = 30.0;
}


BodyPublisherItem::BodyPublisherItem(const BodyPublisherItem& org)
    : ControllerItem(org)
{
    impl = new BodyPublisherItemImpl(this, *org.impl);
}
    

BodyPublisherItemImpl::BodyPublisherItemImpl(BodyPublisherItem* self, const BodyPublisherItemImpl& org)
    : self(self)
{
    io = nullptr;
    maxPublishRate = org.maxPublishRate;
}


BodyPublisherItem::~BodyPublisherItem()
{
    delete impl;
}


BodyPublisherItemImpl::~BodyPublisherItemImpl()
{

}


Item* BodyPublisherItem::doDuplicate() const
{
    return new BodyPublisherItem(*this);
}


void BodyPublisherItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyPublisherItemImpl::setBodyItem(BodyItem* bodyItem, bool forceUpdate)
{
    if(bodyNode){
        if(forceUpdate || bodyItem != bodyNode->bodyItem){
            bodyNode.reset();
        }
    }
    
    if(bodyItem && !bodyNode){
        bodyNode.reset(new BodyNode(bodyItem));

        bodyNode->connections.add(
            bodyItem->sigNameChanged().connect(
                [&](const std::string& oldName){ setBodyItem(bodyItem, true); }));
    }
}


void BodyPublisherItem::onDisconnectedFromRoot()
{
    impl->bodyNode.reset();
}


double BodyPublisherItem::timeStep() const
{
    return 0.0;
}


bool BodyPublisherItem::initialize(ControllerIO* io)
{
    impl->io = io;
    return true;
}


bool BodyPublisherItem::start()
{
    return impl->start();
}


bool BodyPublisherItemImpl::start()
{
    ioBody = io->body();
    bodyNode->stopToPublishKinematicStateChangeOnGUI();
    bodyNode->initializeJointState(io->body());
    time = 0.0;
    minPublishCycle = maxPublishRate > 0.0 ? (1.0 / maxPublishRate) : 0.0;
    timeToPublishNext = minPublishCycle;
    timeStep = io->timeStep();
    return true;
}


void BodyPublisherItem::input()
{
    impl->input();
}


void BodyPublisherItemImpl::input()
{
    timeToPublishNext += timeStep;
    if(timeToPublishNext > minPublishCycle){
        bodyNode->publishJointState(ioBody, time);
        timeToPublishNext -= minPublishCycle;
    }
}


bool BodyPublisherItem::control()
{
    impl->time += impl->timeStep;
    return true;
}


void BodyPublisherItem::output()
{

}


void BodyPublisherItem::stop()
{
    impl->bodyNode->stopToPublishKinematicStateChangeOnGUI();
    impl->io = nullptr;
}


void BodyPublisherItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Max publish rate"), impl->maxPublishRate, changeProperty(impl->maxPublishRate));
}


bool BodyPublisherItem::store(Archive& archive)
{
    archive.write("maxPublishRate", impl->maxPublishRate);
    return true;
}


bool BodyPublisherItem::restore(const Archive& archive)
{
    archive.read("maxPublishRate", impl->maxPublishRate);
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
    startToPublishKinematicStateChangeOnGUI();
}


void BodyNode::startToPublishKinematicStateChangeOnGUI()
{
    if(jointStatePublisher){
        auto body = bodyItem->body();
        initializeJointState(body);
        publishJointState(body, timeBar->time());
        connectionOfKinematicStateChange.reset(
            bodyItem->sigKinematicStateChanged().connect(
                [&](){ publishJointState(bodyItem->body(), timeBar->time()); }));
    }
}


void BodyNode::stopToPublishKinematicStateChangeOnGUI()
{
    connectionOfKinematicStateChange.disconnect();
}

        
void BodyNode::initializeJointState(Body* body)
{
    const int n = body->numJoints();
    jointState.name.resize(n);
    jointState.position.resize(n);
    jointState.velocity.resize(n);
    jointState.effort.resize(n);
    for(int i=0; i < n; ++i){
        jointState.name[i] = body->joint(i)->name();
    }
}
    

void BodyNode::publishJointState(Body* body, double time)
{
    jointState.header.stamp.fromSec(time);

    for(int i=0; i < body->numJoints(); ++i){
        Link* joint = body->joint(i);
        jointState.position[i] = joint->q();
        jointState.velocity[i] = joint->dq();
        jointState.effort[i] = joint->u();
    }

    jointStatePublisher.publish(jointState);
}
