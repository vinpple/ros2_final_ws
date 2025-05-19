#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // JPEG 압축 이미지 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty image frame received.");
        return;
    }

    // 컬러 → 그레이
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 그레이 → 이진화 (threshold 사용)
    cv::Mat binary;
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    // 각 영상 창에 출력
    cv::imshow("Original", frame);  // 원본 이미지
    cv::imshow("Gray", gray);       // 그레이 이미지
    cv::imshow("Binary", binary);   // 이진화된 이미지

    // 영상 갱신
    cv::waitKey(1);  // 1ms 대기 후 창을 갱신

    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");

    // 각 창을 미리 생성
    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Binary", cv::WINDOW_AUTOSIZE);

    // QoS 설정
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 콜백 함수 바인딩
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    // Subscriber 생성
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    // spin 호출: 노드를 실행
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
