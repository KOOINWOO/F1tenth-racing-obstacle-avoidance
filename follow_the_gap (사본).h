#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vector>

class FollowTheGap {
public:
    // 생성자: 파라미터를 받아 초기화합니다.
    FollowTheGap(double max_range, double bubble_radius, double safe_speed, double slow_speed);

    // 메인 함수: LiDAR 데이터를 받아 제어 명령을 계산하고 반환합니다.
    ackermann_msgs::msg::AckermannDriveStamped calculateCommand(const sensor_msgs::msg::LaserScan& scan);

private:
    // 파라미터 저장 변수
    double max_range_;
    double bubble_radius_;
    double safe_speed_;
    double slow_speed_;
};
