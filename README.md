# choreonoid ros plugin
少し触ったので忘れないようにメモ  

## choreonoidとROSをつなげる環境構築  
* 環境構築は[ROSによる遠隔操作サンプル](http://choreonoid.org/ja/manuals/latest/wrs2018/teleoperation-ros.html)を参照

## choreonoid_ros_samplesが動かない件  
* 最新のchoreonoidでは動かなくなってしまっているため[このコミット](https://github.com/minaminoki/choreonoid_ros_samples/commit/fd449c6206d561d753d9de9326119a290ef2b43f)を参照して少し書き換える必要がある  
* build
```
roscd
catkin build choreonoid_ros_samples
source devel/setup.bash
```

[このリンク](http://choreonoid.org/ja/manuals/latest/simulation/howto-implement-controller.html#id4)と[このリンク](http://choreonoid.org/ja/manuals/latest/simulation/pseudo-continuous-track.html)は古い情報？  
無限軌道JOINT_SURFACE_VELOCITYの場合dqはただの状態変数でdq_targetに目的値を入れる必要がある様子  
高精度無限軌道シミュレーションへの対応の影響の可能性あり  

# choreonoid ros pluginの改造方法  
## IMU情報をROSでPublishしたい！
* [Bodyファイル リファレンスマニュアル](http://choreonoid.org/ja/manuals/latest/handling-models/modelfile/yaml-reference.html)を参考に対象のモデルのBodyファイルにAccelertionSensorノードとRateGyroSensorノードをつける  
* choreonoid/share/model/Tank/TankBody.bodyファイルにはこのノードが付いている
```
        type: AccelerationSensor
        name: ACCEL_SENSOR
        id: 0
      -
        type: RateGyroSensor
        name: GYRO
        id: 0
```
* choreonoidをROSで動かす際のmodelファイルはdevel/shareの中にあるので注意！  
* [このコミット](https://github.com/minaminoki/choreonoid_rosplugin/commit/4a6614a0b9d04b95f92dc74f0c2388eefbd418dd)を参考にBodyPublisherItem.cppにプログラムを追加する
* build
```
roscd
catkin build choreonoid_rosplugin
source devel/setup.bash
```
* run
```
roscore
CNOID_USE_GLSL=1 choreonoid src/choreonoid_ros_samples/project/ROS_Tank.cnoid
rostopic list
rostopic echo /Tank/GYRO/gyro
rostopic echo /Tank/ACCEL_SENSOR/accel
```

## Subscribeしてみる
サンプル  
```
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class SensorSub
{
public:
	SensorSub() {
		ros::NodeHandle node;
		accel_sub_ = node.subscribe("ROBOT_NAME/ACCEL_SENSOR/accel",1,&SensorSub::accelCallback_,this);
	}
private:
	ros::Subscriber accel_sub_;
	void accelCallback_(const sensor_msgs::Imu &accel) {
		std::cout << accel.header.frame_id << std::endl;
		std::cout << "linear_acceleration "
			<< accel.linear_acceleration.x << " "
			<< accel.linear_acceleration.y << " "
			<< accel.linear_acceleration.z 
			<< std::endl;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "sensor_sub");
	SensorSub sensorsub;
	ros::spin();
}
```

# プログラム構成についてのメモ  
## ROSPlugin.cpp
* ROSPluginクラスの記述

## BodyPublisherItem.h
* BodyPublisherクラスを定義している  

## BodyPublisherItem.cpp
* BodyNodeクラスの記述
* BodyPublisherItemのメンバ関数の記述
* BodyPublisherItemImplクラスの記述

# Classについてのメモ
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
* publisherを作成  

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

### BodyNode startメンバ関数  
* publish関数をconnect  

```
void BodyNode::start(ControllerIO* io, double maxPublishRate)
{
    ioBody = io->body();
    time = 0.0;
    minPublishCycle = maxPublishRate > 0.0 ? (1.0 / maxPublishRate) : 0.0;
    timeToPublishNext = minPublishCycle;
    timeStep = io->timeStep();

    stopToPublishKinematicStateChangeOnGUI();
    initializeJointState(ioBody);

    sensorConnections.disconnect();
    DeviceList<> devices = ioBody->devices();

    cameras.assign(devices.extract<Camera>());
    for(size_t i=0; i < cameras.size(); ++i){
        auto camera = cameras[i];
        sensorConnections.add(
            camera->sigStateChanged().connect(
                [&, i](){ publishCameraImage(i); }));
    }
}
```
