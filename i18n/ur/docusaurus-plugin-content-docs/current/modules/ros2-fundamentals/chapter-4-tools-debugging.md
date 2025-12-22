---
sidebar_position: 5
---

# باب 4: ROS 2 ٹولز اور ڈیبگنگ

## جائزہ

یہ باب ROS 2 کے طاقتور کمانڈ لائن ٹولز اور ڈیبگنگ تکنیکوں کا جائزہ لیتا ہے۔

## ROS 2 CLI ٹولز

### نوڈ کمانڈز

```bash
# فعال نوڈز کی فہرست
ros2 node list

# نوڈ کی معلومات
ros2 node info /my_node
```

### ٹاپک کمانڈز

```bash
# فعال ٹاپکس کی فہرست
ros2 topic list

# ٹاپک کی معلومات
ros2 topic info /my_topic

# ٹاپک پر سننا
ros2 topic echo /my_topic

# ٹاپک پر پبلش کرنا
ros2 topic pub /my_topic std_msgs/msg/String "data: 'ہیلو'"
```

### سروس کمانڈز

```bash
# فعال سروسز کی فہرست
ros2 service list

# سروس کال کرنا
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
```

### پیرامیٹر کمانڈز

```bash
# پیرامیٹرز کی فہرست
ros2 param list

# پیرامیٹر حاصل کرنا
ros2 param get /my_node my_parameter

# پیرامیٹر سیٹ کرنا
ros2 param set /my_node my_parameter 100
```

## RQT ٹولز

RQT گرافیکل ٹولز کا مجموعہ ہے:

### rqt_graph

سسٹم کے نوڈز اور ٹاپکس کا گرافیکل نقشہ:

```bash
rqt_graph
```

### rqt_console

لاگ پیغامات دیکھنے کے لیے:

```bash
rqt_console
```

### rqt_plot

ڈیٹا کو ریئل ٹائم میں پلاٹ کرنے کے لیے:

```bash
rqt_plot /my_topic/data
```

## ڈیبگنگ تکنیکیں

### لاگنگ لیولز

```python
self.get_logger().debug('ڈیبگ پیغام')
self.get_logger().info('معلوماتی پیغام')
self.get_logger().warn('انتباہی پیغام')
self.get_logger().error('خرابی کا پیغام')
self.get_logger().fatal('مہلک پیغام')
```

### Bag فائلز

ڈیٹا ریکارڈ اور پلے بیک:

```bash
# ریکارڈنگ
ros2 bag record -a

# پلے بیک
ros2 bag play my_bag
```

## خلاصہ

اس باب میں آپ نے سیکھا:
- ROS 2 CLI ٹولز کا استعمال
- RQT گرافیکل ٹولز
- ڈیبگنگ تکنیکیں
- ڈیٹا ریکارڈنگ اور پلے بیک
