#include "rclcpp/rclcpp.hpp"//rclcpp是 ROS 2 中官方提供的 C++客户端库,提供了一套用 C++ 编写 ROS 2 节点、发布者、订阅者、服务、定时器等功能的接口。
//.hpp 是 C++ 中的头文件（Header File），类似于 .h 文件。
#include "geometry_msgs/msg/twist.hpp"//ROS 2 官方提供的标准消息库
#include <chrono> // 引入时间相关头文件
// 使用时间单位的字面量，可以在代码中使用 s 和 ms 表示时间
using namespace std::chrono_literals;

class TurtleCircle : public rclcpp::Node//：表示继承、：：表示作用域解析运算符————表示属于谁的意思
{
private:
  rclcpp::TimerBase::SharedPtr timer_; // 定时器智能指针
/*
rclcpp	    ROS 2 的 C++ 客户端库命名空间。。这是官方命名，如果我以后要写自己的命名空间，也可以自己定义
TimerBase	  表示 ROS 2 中的定时器基类
SharedPtr	  表示这个类的共享智能指针（shared pointer）类型
timer_	    变量名，表示你创建的定时器指针
*/

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 发布者智能指针
/*
rclcpp	                        ROS 2 的命名空间
Publisher<T>	                  表示要发布的消息类型是 T
<geometry_msgs::msg::Twist>	    发布的消息类型是 Twist，用于发送线速度和角速度
SharedPtr	                      表示这个 Publisher 的共享智能指针类型
publisher_	                    变量名，发布者指针
*/

public:
  explicit TurtleCircle(const std::string& node_name) : Node(node_name)
  {
  	// 调用继承而来的父类函数创建订阅者
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    // 调用继承而来的父类函数创建定时器
    timer_ = this->create_wall_timer(1000ms, std::bind(&TurtleCircle::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 1.0;
    msg.angular.z = 0.5;
    publisher_->publish(msg);
  }

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtleCircle>("turtle_square");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
