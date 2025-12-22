---
sidebar_position: 3
---

# باب 2: ROS 2 نوڈز، ٹاپکس، سروسز، اور ایکشنز

## جائزہ

یہ باب ROS 2 کے بنیادی کمیونیکیشن پیٹرنز کو گہرائی سے دریافت کرتا ہے۔ نوڈز، ٹاپکس، سروسز، اور ایکشنز کو سمجھنا کسی بھی ROS 2 ایپلیکیشن کی تعمیر کے لیے ضروری ہے۔

## نوڈز

نوڈز ROS 2 میں سب سے بنیادی تعمیراتی بلاکس ہیں۔ ہر نوڈ ایک عمل ہے جو ایک مخصوص کام انجام دیتا ہے۔

### نوڈز کی خصوصیات

- ہر نوڈ ایک ہی، ماڈیولر مقصد کا ذمہ دار ہے
- نوڈز ٹاپکس، سروسز، اور ایکشنز کے ذریعے بات چیت کرتے ہیں
- نوڈز کو مختلف پروگرامنگ زبانوں میں لکھا جا سکتا ہے

### نوڈ بنانا (Python)

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('میرا نوڈ شروع ہو گیا!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## ٹاپکس

ٹاپکس نامزد چینلز ہیں جن پر نوڈز پیغامات پبلش اور سبسکرائب کر سکتے ہیں۔

### پبلشر/سبسکرائبر پیٹرن

- **پبلشر**: ایک ٹاپک پر پیغامات بھیجتا ہے
- **سبسکرائبر**: ایک ٹاپک سے پیغامات وصول کرتا ہے
- کمیونیکیشن غیر ہم وقت ہے

### پبلشر مثال

```python
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'ہیلو ROS 2!'
        self.publisher.publish(msg)
```

### سبسکرائبر مثال

```python
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String, 'my_topic', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'موصول ہوا: {msg.data}')
```

## سروسز

سروسز ہم وقت درخواست/جواب کمیونیکیشن فراہم کرتی ہیں۔

### سروس سرور

```python
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_callback)

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        return response
```

### سروس کلائنٹ

```python
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        return self.client.call_async(request)
```

## ایکشنز

ایکشنز طویل مدتی ٹاسکس کے لیے استعمال ہوتے ہیں جو فیڈ بیک فراہم کرتے ہیں۔

### ایکشن کی خصوصیات

- ایک ہدف جو کلائنٹ بھیجتا ہے
- فیڈ بیک جو عمل کے دوران فراہم کیا جاتا ہے
- ایک نتیجہ جو مکمل ہونے پر واپس کیا جاتا ہے
- منسوخ کرنے کی صلاحیت

## خلاصہ

اس باب میں ہم نے ROS 2 کے بنیادی کمیونیکیشن پیٹرنز سیکھے:
- نوڈز: بنیادی عمل
- ٹاپکس: غیر ہم وقت پیغام رسانی
- سروسز: ہم وقت درخواست/جواب
- ایکشنز: طویل مدتی ٹاسکس
