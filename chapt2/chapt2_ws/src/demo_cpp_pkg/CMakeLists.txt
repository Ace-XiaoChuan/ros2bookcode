cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 1.查找并导入 rclcpp 头文件和库，相当于python的import
find_package(rclcpp REQUIRED)
# 2. 添加可执行文件 cpp_node（指定源文件）
add_executable(cpp_node src/cpp_node.cpp)
# 3. 为 cpp_node 添加依赖，告诉 CMake，这个可执行文件需要依赖 rclcpp
ament_target_dependencies(cpp_node rclcpp)

add_executable(person_node src/person_node.cpp)
ament_target_dependencies(person_node rclcpp)


add_executable(learn_auto src/learn_auto.cpp)
ament_target_dependencies(learn_auto rclcpp)


add_executable(learn_shared_ptr src/learn_shared_ptr.cpp)
ament_target_dependencies(learn_shared_ptr rclcpp)

add_executable(learn_lambda src/learn_lambda.cpp)
ament_target_dependencies(learn_lambda rclcpp)

add_executable(learn_function src/learn_function.cpp)
ament_target_dependencies(learn_function rclcpp)
include_directories(include)
add_executable(learn_thread src/learn_thread.cpp)
ament_target_dependencies(learn_thread rclcpp)
# 4. 让 cpp_node 在 colcon build 安装时被自动拷贝到 install 目录中
install(TARGETS
  cpp_node
  person_node
  learn_auto
  learn_shared_ptr
  learn_lambda
  learn_function
  learn_thread
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
