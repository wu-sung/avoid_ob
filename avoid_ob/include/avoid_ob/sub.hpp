#ifndef _VM_HPP_
#define _VM_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include <math.h>
#include <memory>
#include <chrono>
#include <algorithm>
#define RAD2DEG(x) ((x)*180./M_PI)
#define LENGTH 800.0
#define XXX 250.0
#define ROI 250.0

using std::placeholders::_1;
using namespace std::chrono_literals;

class VM : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;    
        void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
        std_msgs::msg::Int32 intmsg;
        void publish_msg();
        float degree = 40/30; 
        int err;
        cv::Mat img;

    public:
        VM();
        cv::VideoWriter writer1;
};

#endif