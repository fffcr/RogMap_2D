/**
* rog_map 的 ROS2 独立调试节点。
*
* ROGMapROS 本身是 header-only 的类（include/rog_map_ros/rog_map_ros2.hpp），
* 只有一个吃 (node, cfg_path) 的构造函数，没有 main。
* 这里按 SUPER 的方式补一个入口：config_name 传文件名，路径相对包源码根目录解析，
* 这样不依赖 install，直接 ros2 run 就能起，方便单独调试。
*/

#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rog_map_ros/rog_map_ros2.hpp"
#include "super_utils/color_text.hpp"

#define BACKWARD_HAS_DW 1
#include "super_utils/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}

/* 和 SUPER 一致：config_name 只是文件名，拼到包源码根目录下。
 * 例：config_name := rog_map.yaml  ->  <pkg>/config/rog_map.yaml */
#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("rog_map_node");

    std::string config_name = "rog_map.yaml";
    node->declare_parameter("config_name", config_name);
    node->get_parameter("config_name", config_name);
    const std::string cfg_path = CONFIG_FILE_DIR(config_name);

    std::cout << color_text::GREEN << " -- [ROG-Map] Loading config: " << cfg_path
            << color_text::RESET << std::endl;

    auto map_ptr = std::make_shared<rog_map::ROGMapROS>(node, cfg_path);

    /* ROGMapROS 内部开了多个 MutuallyExclusive 回调组
     * （odom / cloud / 1ms 更新定时器 / 可视化），单线程执行器会互相堵住，必须多线程。 */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
