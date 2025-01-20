#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <opencv2/opencv.hpp>
#include <array>
#include <string>

class JointStatePublisher : public rclcpp::Node {
public:
    JointStatePublisher() : Node("joint_state_publisher"), joint_angles_{0, 0, 0, 0, 0, 0} {
        // 创建 Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // 初始化 OpenCV 窗口和滑条
        cv::namedWindow("Joint Control", cv::WINDOW_AUTOSIZE);
        for (int i = 0; i < joint_angles_.size(); ++i) {
            std::string trackbar_name = "Joint " + std::to_string(i + 1);
            cv::createTrackbar(trackbar_name, "Joint Control", &joint_angles_[i], 360, nullptr);
            cv::setTrackbarPos(trackbar_name, "Joint Control", 180); // 默认值设置为 0 度
        }

        // 定时器，每隔 1ms 发布一次关节状态
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&JointStatePublisher::publishJointStates, this)
        );

        RCLCPP_INFO(this->get_logger(), "Joint State Publisher Node Started");

        // 开启 OpenCV 窗口显示
        while (rclcpp::ok()) {
            cv::imshow("Joint Control", cv::Mat::zeros(100, 400, CV_8UC3));
            if (cv::waitKey(10) == 27) { // 按下 ESC 键退出
                break;
            }
            rclcpp::spin_some(this->get_node_base_interface());
        }

        cv::destroyAllWindows();
    }

private:
    // 发布关节状态消息
    void publishJointStates() {
        auto message = sensor_msgs::msg::JointState();
        message.header.stamp = this->get_clock()->now();

        // 设置关节名称
        message.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // 将滑条值（角度）转换为弧度，并设置到消息中
        message.position.resize(joint_angles_.size());
        for (size_t i = 0; i < joint_angles_.size(); ++i) {
            message.position[i] = (joint_angles_[i] - 180) * M_PI / 180.0; // 滑条范围 -180 ~ +180
        }

        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published joint states: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    message.position[0], message.position[1], message.position[2],
                    message.position[3], message.position[4], message.position[5]);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::array<int, 6> joint_angles_; // 存储滑条值（角度）
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStatePublisher>();
    rclcpp::shutdown();
    return 0;
}
