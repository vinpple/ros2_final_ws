#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <csignal>  // Ctrl+C 감지를 위한 추가

using std::placeholders::_1;

cv::VideoWriter writer;         // 영상 저장 객체
bool stop_signal = false;       // 종료 플래그

// Ctrl+C 핸들러
void signal_handler(int)
{
    stop_signal = true;
    RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "SIGINT (Ctrl+C) 감지, 종료 중...");
}

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // JPEG 압축 이미지 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    if (frame.empty()) {
        RCLCPP_WARN(node->get_logger(), "Empty image frame received.");
        return;
    }

    // VideoWriter 초기화 (첫 프레임 수신 시)
    if (!writer.isOpened()) {
        std::string filename = "output.mp4";
        int codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        double fps = 30.0;
        cv::Size size(frame.cols, frame.rows);

        writer.open(filename, codec, fps, size, true);
        if (!writer.isOpened()) {
            RCLCPP_ERROR(node->get_logger(), "영상 파일 저장 실패: %s", filename.c_str());
            return;
        }

        RCLCPP_INFO(node->get_logger(), "영상 저장 시작: %s", filename.c_str());
    }

    // 원본 영상 저장
    writer.write(frame);

    // 컬러 → 그레이
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 그레이 → 이진화
    cv::Mat binary;
    cv::threshold(gray, binary, 128, 255, cv::THRESH_BINARY);

    // 영상 출력
    cv::imshow("Original", frame);
    cv::imshow("Gray", gray);
    cv::imshow("Binary", binary);
    cv::waitKey(1);

    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d x %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");

    // Ctrl+C 시 종료 플래그 설정
    signal(SIGINT, signal_handler);

    // 영상 창 미리 생성
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

    // 수동 spin loop (Ctrl+C 종료 지원)
    rclcpp::Rate rate(30);
    while (rclcpp::ok() && !stop_signal) {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // 종료 정리
    writer.release();
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
