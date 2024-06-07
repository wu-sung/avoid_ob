#include "avoid_ob/sub.hpp"

// 생성자: 노드를 초기화하고 LiDAR 구독자와 퍼블리셔를 생성하며 타이머를 설정합니다.
VM::VM() : Node("mysub")
{
  writer1.open("avoid_sim.mp4", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 10, cv::Size(LENGTH, LENGTH));

  // LiDAR 데이터 구독자 생성
  lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS(), std::bind(&VM::scanCb, this, std::placeholders::_1));
  
  // 에러 메시지 퍼블리셔 생성
  pub_ = this->create_publisher<std_msgs::msg::Int32>("err", rclcpp::QoS(rclcpp::KeepLast(10)));
  
  // 타이머 설정: 100ms마다 publish_msg 함수를 호출
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&VM::publish_msg, this));
}

// LiDAR 데이터 콜백 함수: 장애물을 감지하고 경로를 계획합니다.
void VM::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  int count = scan->scan_time / scan->time_increment;  // LiDAR 데이터 포인트 수
  int x, y;
  float right_x, right_y, left_x, left_y, right_width, right_height, left_width, left_height, right_detect_min, left_detect_min, rtheta, ltheta;
  
  // 이미지를 초기화합니다.
  cv::Mat img(LENGTH, LENGTH, CV_8UC3, cv::Scalar(255, 255, 255));
  
  // LiDAR 데이터 포인트를 이미지에 그립니다.
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    x = LENGTH / 2 + scan->ranges[i] * XXX * sin(degree * M_PI / 180);
    y = LENGTH / 2 + scan->ranges[i] * XXX * cos(degree * M_PI / 180);
    cv::circle(img, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), 2);
  }

  // 이미지의 좌우 영역을 나누어 처리합니다.
  cv::Mat gray_left, gray_right;
  cv::Mat ROI_LEFT = img(cv::Rect(ROI, ROI, LENGTH / 2 - ROI, LENGTH / 2 - ROI));
  cv::Mat ROI_RIGHT = img(cv::Rect(LENGTH / 2 + 1, ROI, LENGTH / 2 - ROI + 1, LENGTH / 2 - ROI));
  cv::cvtColor(ROI_LEFT, gray_left, cv::COLOR_BGR2GRAY);
  cv::cvtColor(ROI_RIGHT, gray_right, cv::COLOR_BGR2GRAY);

  // 이진화를 통해 객체를 감지합니다.
  cv::Mat bin_left, bin_right;
  cv::threshold(gray_left, bin_left, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  cv::threshold(gray_right, bin_right, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
  
  // 연결된 컴포넌트(객체)를 감지하고 통계를 계산합니다.
  cv::Mat labels_left, stats_left, centroids_left;
  cv::Mat labels_right, stats_right, centroids_right;
  int objectcount_left = cv::connectedComponentsWithStats(bin_left, labels_left, stats_left, centroids_left); 
  int objectcount_right = cv::connectedComponentsWithStats(bin_right, labels_right, stats_right, centroids_right);

  // 감지된 객체를 이미지에 표시합니다.
  cv::Mat dst_left, dst_right;
  cv::cvtColor(bin_left, dst_left, cv::COLOR_GRAY2BGR);
  cv::cvtColor(bin_right, dst_right, cv::COLOR_GRAY2BGR);
  cv::circle(img, cv::Point(LENGTH / 2, LENGTH / 2), 2, cv::Scalar(0, 0, 0), 2);

  // 왼쪽 영역에서 가장 가까운 객체를 찾습니다.
  left_detect_min = sqrt(pow(LENGTH / 2 - stats_left.at<int>(1, 0) - stats_left.at<int>(1, 2), 2) +
                         pow(LENGTH / 2 - stats_left.at<int>(1, 1) - stats_left.at<int>(1, 3), 2));
  for (int i = 1; i < objectcount_left; i++) {
    int* p = stats_left.ptr<int>(i);
    if (p[4] < 50) continue;
    cv::rectangle(img, cv::Rect(p[0] + ROI, p[1] + ROI, p[2], p[3]), cv::Scalar(255, 0, 0), 2);
    float dist = sqrt(pow(LENGTH / 2 - ROI - p[0] - p[2], 2) + pow(LENGTH / 2 - ROI - p[1] - p[3], 2));
    if (left_detect_min > dist) {
      left_detect_min = dist;
      left_x = p[0];
      left_y = p[1];
      left_width = p[2];
      left_height = p[3];
    }
  }

  // 오른쪽 영역에서 가장 가까운 객체를 찾습니다.
  right_detect_min = sqrt(pow(stats_right.at<int>(1, 0), 2) +
                          pow(LENGTH / 2 - stats_right.at<int>(1, 1) - stats_right.at<int>(1, 3), 2));
  for (int j = 1; j < objectcount_right; j++) {
    int* p = stats_right.ptr<int>(j);
    if (p[4] < 50) continue;
    cv::rectangle(img, cv::Rect(p[0] + LENGTH / 2, p[1] + ROI, p[2], p[3]), cv::Scalar(255, 0, 255), 2);
    float dist = sqrt(pow(p[0], 2) + pow(LENGTH / 2 - ROI - p[1] - p[3], 2));
    if (right_detect_min > dist) {
      right_detect_min = dist;
      right_x = p[0];
      right_y = p[1];
      right_width = p[2];
      right_height = p[3];
    }
  }

  // 객체의 개수에 따라 각도를 계산합니다.
  if (objectcount_left == 1 && objectcount_right != 1) {
    ltheta = 0;
    rtheta = atan((400 - ROI - right_y - right_height) / (right_x + 0.0000001));
    left_detect_min = 200;
  }
  else if (objectcount_right == 1 && objectcount_left != 1) {
    ltheta = atan((400 - ROI - left_y - left_height) / (400 - ROI - left_x - left_width + 0.0000001));
    rtheta = 0;
    right_detect_min = 200;
  }
  else if (objectcount_left == 1 && objectcount_right == 1) {
    ltheta = 0;
    rtheta = 0;
    left_detect_min = 200;
    right_detect_min = 200;
  }
  else {
    ltheta = atan((400 - ROI - left_y - left_height) / (400 - ROI - left_x - left_width + 0.0000001));
    rtheta = atan((400 - ROI - right_y - right_height) / (right_x + 0.0000001));
  }

  // 경로를 시각화합니다.
  cv::arrowedLine(img, cv::Point(LENGTH / 2, LENGTH / 2), cv::Point(400 - 200 * sin(rtheta / 2 - ltheta / 2), 400 - 200 * cos(rtheta / 2 - ltheta / 2)), cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
  cv::drawMarker(img, cv::Point(400, 400), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 40);

  // 로그 메시지 출력
  printf("left_detect_min = %f, right_detect_min = %f\n", left_detect_min, right_detect_min);
  printf("objectcount_left = %d, objectcount_right = %d\n", objectcount_left, objectcount_right);
  printf("left_x = %f, left_y = %f\n", left_x + left_width + ROI, left_y + left_height + ROI);
  printf("right_x = %f, right_y = %f\n", right_x + LENGTH / 2, right_y + right_height + ROI);

  // 에러 값을 계산합니다.
  if (left_detect_min <= 50 && right_detect_min <= 50 && RAD2DEG(ltheta) > 80 && RAD2DEG(rtheta) > 80) err = 100;
  else if (left_detect_min <= 40 && RAD2DEG(rtheta) > 80) err = 200;
  else if (right_detect_min <= 40 && RAD2DEG(ltheta) > 80) err = 300;
  else if (left_detect_min <= 20 && right_detect_min > 20) err = 400;
  else if (right_detect_min <= 20 && left_detect_min > 20) err = 500;
  else err = (RAD2DEG(ltheta) - RAD2DEG(rtheta)) / 2;

  // 에러 값을 출력합니다.
  printf("err = %d\n", err);

  // 비디오 파일에 이미지 추가
  writer1 << img;

  // 이미지 화면에 표시
  cv::imshow("img", img);
  cv::waitKey(1);
}

// 주기적으로 메시지를 퍼블리시하는 함수
void VM::publish_msg()
{
  intmsg.data = err;
  pub_->publish(intmsg);
}
