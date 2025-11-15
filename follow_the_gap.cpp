#include "follow_the_gap.h"
#include <vector>
#include <algorithm>
#include <cmath>

FollowTheGap::FollowTheGap(double max_range, double bubble_radius, double car_width)
    : max_range_(max_range), bubble_radius_(bubble_radius), car_width_(car_width) {}

ackermann_msgs::msg::AckermannDriveStamped FollowTheGap::calculateCommand(const sensor_msgs::msg::LaserScan& scan) {
    std::vector<float> ranges = scan.ranges;
    for (auto &r : ranges) {
        if (!std::isfinite(r) || r > max_range_) r = max_range_;
    }

    // 1. 버블 생성 (이전과 동일)
    auto min_it = std::min_element(ranges.begin(), ranges.end());
    int min_idx = std::distance(ranges.begin(), min_it);
    double min_dist = *min_it;
    if (min_dist < max_range_) {
        if (bubble_radius_ / min_dist <= 1.0) {
            double bubble_angle = std::asin(bubble_radius_ / min_dist);
            int bubble_size = static_cast<int>(bubble_angle / scan.angle_increment);
            for (int i = std::max(0, min_idx - bubble_size); i < std::min((int)ranges.size(), min_idx + bubble_size); ++i) {
                ranges[i] = 0.0;
            }
        } else {
            std::fill(ranges.begin(), ranges.end(), 0.0);
        }
    }

    // <<--- 2. "가장 넓은 틈 찾기" 로직 (더 튼튼하게 수정) --->>
    int best_start = 0;
    int best_end = 0;
    int max_gap_size = 0;
    int current_start = -1;

    for (int i = 0; i < (int)ranges.size(); ++i) {
        if (ranges[i] > 0.1) { // 0.1m 이상을 유효한 거리로 간주
            if (current_start == -1) current_start = i; // 틈 시작
        } else { // 0.1m 이하는 장애물로 간주
            if (current_start != -1) { // 틈이 방금 끝났다면
                int current_gap_size = i - 1 - current_start;
                if (current_gap_size > max_gap_size) {
                    max_gap_size = current_gap_size;
                    best_start = current_start;
                    best_end = i - 1;
                }
                current_start = -1; // 틈 리셋
            }
        }
    }
    // 마지막까지 틈이 이어지는 경우 처리
    if (current_start != -1) {
        int current_gap_size = (int)ranges.size() - 1 - current_start;
        if (current_gap_size > max_gap_size) {
            max_gap_size = current_gap_size;
            best_start = current_start;
            best_end = (int)ranges.size() - 1;
        }
    }

    // 3. 새로운 목표 지점 계산
    int target_idx = (best_start + best_end) / 2; // 기본값: 가장 넓은 틈의 중앙

    // 만약 유효한 틈을 하나도 못찾았다면 (모든 빔이 0.1m 이내),
    // 안전하게 정면을 바라보게 합니다.
    if (max_gap_size == 0) {
        target_idx = (int)ranges.size() / 2;
    }
    
    // "안쪽으로 돌기" (최소 회피) 로직
    double gap_dist = ranges[target_idx];
    if (gap_dist > 0.1) {
        double safety_margin_dist = 0.07; // 5cm 여유
        
        if (safety_margin_dist / gap_dist <= 1.0) {
            double safety_angle = std::asin(safety_margin_dist / gap_dist);
            int safety_indices = static_cast<int>(safety_angle / scan.angle_increment);
            int center_idx = scan.ranges.size() / 2;

            if (best_end - best_start > 2 * safety_indices) { // 틈이 충분히 넓으면
                // 정면에서 가까운 가장자리 + 안전 마진을 목표로 함
                if (std::abs(center_idx - best_start) < std::abs(center_idx - best_end)) {
                    target_idx = best_start + safety_indices;
                } else {
                    target_idx = best_end - safety_indices;
                }
            }
            // 틈이 좁으면 그냥 중앙을 목표로 함 (target_idx 기본값 유지)
        }
    }

    double target_angle = scan.angle_min + target_idx * scan.angle_increment;
    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.drive.steering_angle = target_angle;
    
    return cmd;
}
