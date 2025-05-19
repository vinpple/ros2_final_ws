#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

// 전송용 GStreamer 파이프라인 (GRAY8 포맷)
std::string dst = "appsrc ! videoconvert ! video/x-raw, format=GRAY8 ! \
	nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
	h264parse ! rtph264pay pt=96 ! \
	udpsink host=192.168.0.15 port=9001 sync=false";

cv::VideoWriter writer;

// 콜백 함수
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // JPEG 압축 이미지 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty image frame received.");
        return;
    }

    // 컬러 → 그레이
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 그레이 → 이진 (threshold 사용)
    cv::Mat binary;
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    // 이진 영상 전송
    writer << binary;

    // 영상 출력: 각각 다른 창에 출력
    cv::imshow("Original", frame);
    cv::imshow("Gray", gray);
    cv::imshow("Binary", binary);
    cv::waitKey(1);  // 창 갱신

    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d x %d", msg->format.c_str(), binary.rows, binary.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");

    // VideoWriter 초기화 (컬러 false)
    writer.open(dst, 0, (double)30, cv::Size(640, 360), false);
    if(!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

    // 👉 창 미리 생성
    cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Gray", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Binary", cv::WINDOW_AUTOSIZE);

    // QoS 설정 및 콜백 함수 바인딩
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    // Subscriber 생성
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
