#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm> // for std::clamp

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        // 파라미터 (튜닝 값)
        lookahead_ = declare_parameter<double>("lookahead", 1.0); 
        wheelbase_ = declare_parameter<double>("wheelbase", 0.34);
        v_min_ = declare_parameter<double>("speed_min", 1.0); 
        v_max_ = declare_parameter<double>("speed_max", 2.0);  

        // "뒷걸음질" 방지 튜닝 값 (유지)
        k_speed_ = declare_parameter<double>("k_speed", 4.0);
        k_accel_ = declare_parameter<double>("k_accel", 1.0);
        a_max_ = declare_parameter<double>("accel_max", 3.0); 
        a_min_ = declare_parameter<double>("accel_min", -1.5);

        // 토픽 이름
        std::string path_topic = declare_parameter<std::string>("topics.path", "/center_path");
        std::string odom_topic = declare_parameter<std::string>("topics.odom", "/odom0");
        std::string drive_topic = declare_parameter<std::string>("topics.drive", "/pure_pursuit/cmd_vel");

        // 발행자 및 구독자
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            path_topic, 10, [this](nav_msgs::msg::Path::SharedPtr msg){ center_path_ = *msg; });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg){ odom_ = *msg; has_odom_ = true; });

        timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&PurePursuitNode::onTimer, this));
    }

private:
    void publish_zero_command() {
        ackermann_msgs::msg::AckermannDriveStamped zero_cmd;
        zero_cmd.drive.steering_angle = 0.0;
        zero_cmd.drive.acceleration = 0.0;
        zero_cmd.header.stamp = now();
        drive_pub_->publish(zero_cmd);
    }

    // onTimer (동적 룩어헤드 제거)
    void onTimer() {
        if (!has_odom_ || center_path_.poses.empty()) {
            publish_zero_command(); 
            return;
        }

        const auto &p = odom_.pose.pose.position;
        const auto &q = odom_.pose.pose.orientation;
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf2::Quaternion(q.x, q.y, q.z, q.w)).getRPY(roll, pitch, yaw);

        // 고정 룩어헤드(lookahead_) 사용
        int target_idx = find_lookahead_index(p.x, p.y, yaw);
        
        if (target_idx < 0) {
            publish_zero_command();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "PP: No valid lookahead point found, stopping.");
            return;
        }

        const auto &tp = center_path_.poses[target_idx].pose.position;
        const double dx = tp.x - p.x, dy = tp.y - p.y;
        const double xL = std::cos(yaw) * dx + std::sin(yaw) * dy;
        const double yL = -std::sin(yaw) * dx + std::cos(yaw) * dy;
        
        ackermann_msgs::msg::AckermannDriveStamped cmd;

        if (xL <= 0.1) {
            cmd.drive.steering_angle = 0.0;
            cmd.drive.acceleration = a_min_;
        } else {
            const double Ld_sq = xL*xL + yL*yL;
            const double curvature = 2.0 * yL / Ld_sq;
            
            double v_ref = std::max(v_min_, v_max_ - k_speed_ * std::abs(curvature));
            const double current_v = odom_.twist.twist.linear.x;
            double a_cmd = k_accel_ * (v_ref - current_v);
            
            cmd.drive.steering_angle = std::atan(wheelbase_ * curvature);
            cmd.drive.acceleration = std::clamp(a_cmd, a_min_, a_max_);
        }

        cmd.header.stamp = now();
        cmd.header.frame_id = "base_link";
        drive_pub_->publish(cmd);
    }

    // find_lookahead_index (고정 룩어헤드)
    int find_lookahead_index(double current_x, double current_y, double current_yaw) {
        if (center_path_.poses.empty()) return -1;

        int closest_idx = -1;
        double min_dist_sq = -1.0;

        for (size_t i = 0; i < center_path_.poses.size(); ++i) {
            double dx = center_path_.poses[i].pose.position.x - current_x;
            double dy = center_path_.poses[i].pose.position.y - current_y;
            double dist_sq = dx * dx + dy * dy;
            if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                closest_idx = i;
            }
        }
        if (closest_idx < 0) return -1;

        for (size_t i = closest_idx; i < center_path_.poses.size(); ++i) {
            double dx = center_path_.poses[i].pose.position.x - current_x;
            double dy = center_path_.poses[i].pose.position.y - current_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= lookahead_) { 
                double point_angle = std::atan2(dy, dx);
                double angle_diff = point_angle - current_yaw;
                
                while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
                while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

                if (std::abs(angle_diff) < M_PI_2) { 
                    return i;
                }
            }
        }
        
        return -1;
    }

    // 멤버 변수
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    
    // [FIX] Changed 'msg/Odometry' to 'msg::Odometry'
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path center_path_;
    nav_msgs::msg::Odometry odom_;
    bool has_odom_{false};
    
    double lookahead_, wheelbase_, v_min_, v_max_, k_speed_, k_accel_, a_min_, a_max_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}
