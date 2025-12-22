---
sidebar_position: 4
---

# باب 3: کسٹم میسجز اور لانچ فائلز

## جائزہ

یہ باب ROS 2 میں کسٹم میسج ٹائپس بنانے اور لانچ فائلز کے ذریعے متعدد نوڈز کو مینیج کرنے کا طریقہ سکھاتا ہے۔

## کسٹم میسجز

ROS 2 معیاری میسج ٹائپس فراہم کرتا ہے، لیکن آپ اپنی مخصوص ضروریات کے لیے کسٹم میسجز بھی بنا سکتے ہیں۔

### میسج فائل بنانا

`msg/RobotStatus.msg` فائل بنائیں:

```
# روبوٹ کی حالت کی معلومات
string robot_name
float64 battery_level
bool is_active
float64[] joint_positions
```

### میسج کا استعمال

```python
from my_package.msg import RobotStatus

msg = RobotStatus()
msg.robot_name = "میرا روبوٹ"
msg.battery_level = 85.5
msg.is_active = True
msg.joint_positions = [0.0, 1.57, -0.5]
```

## سروس انٹرفیسز

کسٹم سروسز `.srv` فائلز میں تعریف کی جاتی ہیں:

```
# srv/GetRobotInfo.srv
string robot_id
---
string robot_name
float64 battery_level
bool success
```

## لانچ فائلز

لانچ فائلز آپ کو متعدد نوڈز کو ایک ساتھ شروع کرنے کی اجازت دیتی ہیں۔

### Python لانچ فائل

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='publisher_node',
            name='my_publisher',
            output='screen'
        ),
        Node(
            package='my_package',
            executable='subscriber_node',
            name='my_subscriber',
            output='screen'
        ),
    ])
```

### لانچ کمانڈ

```bash
ros2 launch my_package my_launch.py
```

## پیکج کی ساخت

ایک مکمل ROS 2 پیکج کی ساخت:

```
my_package/
├── package.xml
├── setup.py
├── my_package/
│   ├── __init__.py
│   ├── publisher_node.py
│   └── subscriber_node.py
├── msg/
│   └── RobotStatus.msg
├── srv/
│   └── GetRobotInfo.srv
└── launch/
    └── my_launch.py
```

## خلاصہ

اس باب میں آپ نے سیکھا:
- کسٹم میسج ٹائپس کیسے بنائیں
- سروس انٹرفیسز کی تعریف
- لانچ فائلز کا استعمال
- پیکج کی صحیح ساخت
