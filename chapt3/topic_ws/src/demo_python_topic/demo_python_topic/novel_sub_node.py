import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
import threading
from queue import Queue
import time
import espeakng


class NovelSubNode(Node):

    def __init__(self, node_name):

        # 这里注意：解释一下子类构造函数传参：
        # class Node:
        # def __init__(self, name: str, *, namespace: str = '', context: Context = None, ...):
        # 其他参数大多数是高级用法，初学可以忽略
        super().__init__(node_name)
        self.novels_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(
            String, 'novel', self.novel_callback, 10)

        # target 参数指定了 线程运行时要执行的函数。
        # 也就是说，当你调用 self.speech_thread_.start() 时，线程就会去执行你传给 target 的函数 —— 本例中是 self.speak_thread()。
        # 不要加括号！不加括号，只是传递函数对象本身，而不是调用函数！
        self.speech_thread_ = threading.Thread(target=self.speak_thread)
        self.speech_thread_.start()

    # msg是 ROS 2 自动传入的一个消息对象，msg.data 就是发布者发来的字符串，比如 "你好，这是第一章小说"
    def novel_callback(self, msg):
        self.novels_queue_.put(msg.data)

    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'zh'
        while rclpy.ok():
            if self.novels_queue_.qsize() > 0:
                text = self.novels_queue_.get()
                self.get_logger().info(f'正在朗读 {text}')
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = NovelSubNode("novel_read")
    rclpy.spin(node)
    rclpy.shutdown()
