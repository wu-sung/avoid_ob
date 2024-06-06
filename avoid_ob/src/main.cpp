#include "avoid_ob/sub.hpp"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 노드 생성
    auto node = std::make_shared<VM>();
    // 노드 실행
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}