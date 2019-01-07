# choreonoid ros plugin

# プログラム構成についてのメモ  
## ROSPlugin.cpp
* ROSPluginクラスの記述

## BodyPublisherItem.h
* BodyPublisherクラスを定義している  

## BodyPublisherItem.cpp
* BodyNodeクラスの記述
* BodyPublisherItemのメンバ関数の記述
* BodyPublisherItemImplクラスの記述

# Class
## ROSPluginクラス
* ChoreonoidをROSで走らせるPluginを書いている場所  
* choreonoidのROSノードを作っている  
* BodyPublisherItemをinitializeしている  
 
```
class ROSPlugin : public Plugin
{
...
    virtual bool initialize()
    {
	    ros::init(argc, argv, "choreonoid", ros::init_options::NoSigintHandler);
	...
        spinner->start();
        BodyPublisherItem::initialize(this);
	...
   }
...
};
```

## BodyPublisherItemクラス 
* BodyPublisherItemImplのインスタンスをプライベートで持つ  
* ext系をimplでオーバーライドしている（よくわかっていない）  
* BodyPublisherItemImplのラッパーの様子  
* ところどころcnoidのArchiveの機能を使用している  

```
class CNOID_EXPORT BodyPublisherItem : public ControllerItem
{
public:
...
    virtual bool initialize(ControllerIO* io) override;
...
private:
    BodyPublisherItemImpl* impl;
...
};

bool BodyPublisherItem::initialize(ControllerIO* io)
{
    impl->io = io;
    return true;
}

```

## BodyPublisherItemImplクラス 
* BodyNodeのインスタンスをpublicで持つ
* setBodyItem関数を持つ  

```
class CNOID_EXPORT BodyPublisherItem : public ControllerItem
{
public:
...
    virtual bool initialize(ControllerIO* io) override;
...
private:
    BodyPublisherItemImpl* impl;
...
};
```

## BodyNodeクラス
* rosのノードハンドラを持つ  
* JointStatePublisherとCameraImagePublishersを持つ
* ChoreonoidのBodyItemがBodyNodeのような存在？  

```
class BodyNode
{
public:
    unique_ptr<ros::NodeHandle> rosNode;
    BodyItem* bodyItem;
...
    Body* ioBody;
...
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;
    DeviceList<Camera> cameras;
    vector<image_transport::Publisher> cameraImagePublishers;
    BodyNode(BodyItem* bodyItem);
...
    void startToPublishKinematicStateChangeOnGUI();
    void stopToPublishKinematicStateChangeOnGUI();
    void initializeJointState(Body* body);
    void publishJointState(Body* body, double time);
    void publishCameraImage(int index);
...
};
```

### BodyNodeコンストラクタ
* bodyItemにあるbodyのさらにdevicesを取り出してdeviceListにいれている
* deviceListの中からCameraを抽出しカメラインスタンスにアサインしている

```
BodyNode::BodyNode(BodyItem* bodyItem)
    : bodyItem(bodyItem),
      timeBar(TimeBar::instance())
{
    string name = bodyItem->name();
    std::replace(name.begin(), name.end(), '-', '_');

    rosNode.reset(new ros::NodeHandle(name));

    jointStatePublisher = rosNode->advertise<sensor_msgs::JointState>("joint_state", 1000);
    startToPublishKinematicStateChangeOnGUI();

    auto body = bodyItem->body();
    DeviceList<> devices = body->devices();

    cameras.assign(devices.extract<Camera>());
    image_transport::ImageTransport it(*rosNode);
    cameraImagePublishers.resize(cameras.size());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        cameraImagePublishers[i] = it.advertise(camera->name() + "/image", 1);
    }
}
```
