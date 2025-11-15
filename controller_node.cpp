#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <algorithm> // for std::clamp
#include <cmath> // for std::abs, std::isfinite, std::cos, std::sin
#include <vector> // for std::vector

class ControllerNode : public rclcpp::Node {
public:
    enum class State { PURSUIT, AVOIDING, RECOVERING };

    ControllerNode() : Node("controller_node") {
        
        // --- 1. 파라미터 선언 ---
        
        // [신규] APF 파라미터
        // 장애물이 밀어내는 힘의 강도 (게인)
        apf_k_repulsive_ = declare_parameter<double>("apf.k_repulsive", 0.03); // [튜닝] 0.03
        // PP 목표 지점이 끌어당기는 힘의 강도 (게인4
        apf_k_attractive_ = declare_parameter<double>("apf.k_attractive", 3.0); // [튜닝] 2.5
        // APF가 장애물로 인식할 최대 반경
        apf_obstacle_radius_ = declare_parameter<double>("apf.obstacle_radius", 3.0); // 2.0m

        // 기존 파라미터 (유지)
        max_steering_change_ = declare_parameter<double>("max_steering_change", 0.1);
        avoidance_duration_ = declare_parameter<double>("avoidance_duration", 0.5); //0.5
        car_width_ = declare_parameter<double>("car_width", 0.4);
        avoid_max_steer_ = declare_parameter<double>("avoid.max_steer", 0.41);

        // --- 2. 토픽 이름 및 발행/구독 설정 ---
        std::string scan_topic = declare_parameter<std::string>("topics.scan", "/scan0");
        std::string pp_drive_topic = declare_parameter<std::string>("topics.pp_drive", "/pure_pursuit/cmd_vel");
        std::string final_drive_topic = declare_parameter<std::string>("topics.final_drive", "/ackermann_cmd0");

        final_drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(final_drive_topic, 10);
        pp_drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            pp_drive_topic, 10, [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
                latest_pp_cmd_ = *msg; has_pp_cmd_ = true;
            });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic, 10, std::bind(&ControllerNode::scan_callback, this, std::placeholders::_1));

        state_timer_end_ = this->get_clock()->now();
    }

private:
    // --- 3. 멤버 변수 선언부 ---
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr final_drive_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pp_drive_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    ackermann_msgs::msg::AckermannDriveStamped latest_pp_cmd_;
    bool has_pp_cmd_{false};
    double car_width_;
    double last_steering_angle_{0.0}; 
    double max_steering_change_;
    double avoid_max_steer_;
    State current_state_{State::PURSUIT};
    rclcpp::Time state_timer_end_;
    double avoidance_duration_;

    // APF 변수
    double apf_k_repulsive_;
    double apf_k_attractive_;
    double apf_obstacle_radius_;


    // --- 4. 멤버 함수 ---

    /**
     * @brief "인공 전위 필드 (APF)" 조향각 계산 함수
     * [신규]
     * 모든 라이다 점은 '밀어내는 힘(Repulsive)'을,
     * PP의 목표는 '끌어당기는 힘(Attractive)'을 생성.
     * 두 힘의 벡터 합(Vector Sum)으로 최종 조향각을 결정.
     */
    double calculate_apf_steering(const sensor_msgs::msg::LaserScan& scan, double pp_steering_angle)
    {
        // 1. "끌어당기는 힘" (Attractive Force) - PP의 목표
        // (x, y) 벡터로 변환
        double attractive_x = apf_k_attractive_ * std::cos(pp_steering_angle);
        double attractive_y = apf_k_attractive_ * std::sin(pp_steering_angle);

        // 2. "밀어내는 힘" (Repulsive Force) - 모든 장애물
        double repulsive_x = 0.0;
        double repulsive_y = 0.0;

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double dist = scan.ranges[i];
            
            // 'apf_obstacle_radius_' (예: 2.0m) 이내의 점만 고려
            if (!std::isfinite(dist) || dist > apf_obstacle_radius_) continue;

            double angle = scan.angle_min + i * scan.angle_increment;

            // [APF 공식]
            // 힘의 크기 = K * (1/D - 1/D_max)^2
            // (가까울수록 강하고, 2.0m에서 0이 됨)
            double magnitude = apf_k_repulsive_ * std::pow(
                (1.0 / dist) - (1.0 / apf_obstacle_radius_), 2.0);

            // 힘의 방향은 장애물 반대편 (angle + 180도)
            // 모든 힘 벡터를 합산
            repulsive_x += std::cos(angle + M_PI) * magnitude;
            repulsive_y += std::sin(angle + M_PI) * magnitude;
        }

        // 3. "최종 힘" (Vector Sum)
        double final_x = attractive_x + repulsive_x;
        double final_y = attractive_y + repulsive_y;

        // 4. 최종 힘의 방향(각도)이 목표 조향각이 됨
        double target_steering = std::atan2(final_y, final_x);

        return std::clamp(target_steering, -avoid_max_steer_, avoid_max_steer_);
    }

    /**
     * @brief PP 경로가 막혔는지 확인하는 함수 (하이브리드 다이나믹 박스)
     * (직선: 2.0m, 커브: 1.0m) - 이전 로직 유지
     */
    bool is_obstacle_in_path(const sensor_msgs::msg::LaserScan& scan, double steering_angle) {
        if (scan.ranges.empty()) return false;
        
        double box_length = 0.0;
        const double CURVE_THRESHOLD = 0.15; // 8.6도
        const double trigger_box_half_width = (car_width_ * 0.8) / 2.0;

        if (std::abs(steering_angle) < CURVE_THRESHOLD) {
            box_length = 2.0; // 직선: 2.0m
        } else {
            box_length = 1.0; // 커브: 1.0m
        }

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            double angle = scan.angle_min + i * scan.angle_increment;
            double distance = scan.ranges[i];

            if (!std::isfinite(distance) || distance > box_length) continue;

            double x = distance * std::cos(angle);
            double y = distance * std::sin(angle);

            if (x > 0.1 && std::abs(y) < trigger_box_half_width) {
                return true; 
            }
        }
        
        return false;
    }


    /**
     * @brief 메인 콜백 함수 (상태 머신)
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!has_pp_cmd_) return;

        ackermann_msgs::msg::AckermannDriveStamped final_command = latest_pp_cmd_;
        double target_steering = latest_pp_cmd_.drive.steering_angle; 
        auto now = this->get_clock()->now();

        bool pp_path_blocked = is_obstacle_in_path(*msg, latest_pp_cmd_.drive.steering_angle);

        // 상태 머신 로직
        if (current_state_ == State::PURSUIT) {
            if (pp_path_blocked) {
                RCLCPP_WARN(this->get_logger(), "Mode: [PURSUIT] -> [AVOIDING]");
                current_state_ = State::AVOIDING;
                state_timer_end_ = now + rclcpp::Duration::from_seconds(avoidance_duration_);
                
                // [수정] APF 함수 호출
                target_steering = calculate_apf_steering(*msg, latest_pp_cmd_.drive.steering_angle);
            }
        } 
        else if (current_state_ == State::AVOIDING) {
            // [수정] APF 함수 호출
            target_steering = calculate_apf_steering(*msg, latest_pp_cmd_.drive.steering_angle);

            if (now >= state_timer_end_) {
                RCLCPP_INFO(this->get_logger(), "Mode: [AVOIDING] -> [RECOVERING]");
                current_state_ = State::RECOVERING;
            }
        }
        else if (current_state_ == State::RECOVERING) { 
            if (pp_path_blocked) {
                RCLCPP_WARN(this->get_logger(), "Mode: [RECOVERING] -> [AVOIDING] (Path still blocked)");
                current_state_ = State::AVOIDING;
                state_timer_end_ = now + rclcpp::Duration::from_seconds(avoidance_duration_); 
                
                // [수정] APF 함수 호출
                target_steering = calculate_apf_steering(*msg, latest_pp_cmd_.drive.steering_angle);
            } else {
                RCLCPP_INFO(this->get_logger(), "Mode: [RECOVERING] -> [PURSUIT] (Path is clear)");
                current_state_ = State::PURSUIT;
            }
        }

        // 스티어링 속도 제한 로직
        double desired_change = target_steering - last_steering_angle_;
        double clamped_change = std::clamp(desired_change, -max_steering_change_, max_steering_change_);
        
        final_command.drive.steering_angle = last_steering_angle_ + clamped_change;
        last_steering_angle_ = final_command.drive.steering_angle; 

        final_command.header.stamp = now;
        final_drive_pub_->publish(final_command);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
