#pragma once
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class FollowTheGap {
public:
    // <<--- 수정: car_width를 받도록 생성자 변경 --->>
    FollowTheGap(double max_range, double bubble_radius, double car_width);
    ackermann_msgs::msg::AckermannDriveStamped calculateCommand(const sensor_msgs::msg::LaserScan &scan);

private:
    double max_range_;
    double bubble_radius_;
    // <<--- 추가 --->>
    double car_width_;
};
