// vision.cpp
#include "finalproject/vision.hpp"

Vision::Vision(const std::string &node_name)
    : Node(node_name),
      yellow_line_detected(false),
      white_line_detected(false),
      yellow_line_count(0),
      white_line_count(0),
      array_index(0),
      yellow_line_valid(false),
      white_line_valid(false)
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&Vision::imageCallback, this, std::placeholders::_1));

    original_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/original", 10);
    yellow_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/yellow_mask", 10);
    white_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/white_mask", 10);
    line_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/line_detection", 10);

    yellow_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/yellow_line_detected", 10);
    white_detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vision/white_line_detected", 10);

    yellow_detection_array.fill(false);
    white_detection_array.fill(false);
}

bool Vision::isLineValid(std::array<bool, 10> &detection_array, bool current_detection)
{
    // 현재 감지 결과를 배열에 저장
    detection_array[array_index] = current_detection;

    // true의 개수 계산
    int detection_count = 0;
    for (bool detection : detection_array)
    {
        if (detection)
        {
            detection_count++;
        }
    }

    // 임계값 이상이면 유효한 선으로 판단
    return detection_count >= DETECTION_THRESHOLD;
}
void Vision::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat resized_frame;
        cv::resize(frame, resized_frame, cv::Size(640, 480));

        int height = resized_frame.rows;
        int width = resized_frame.cols;

        // ROI 설정
        cv::Point2f src_vertices[4];
        cv::Point2f dst_vertices[4];

        src_vertices[0] = cv::Point2f(width * 0.15f, height * 0.85f);
        src_vertices[1] = cv::Point2f(width * 0.85f, height * 0.85f);
        src_vertices[2] = cv::Point2f(width * 0.9f, height * 1.0f);
        src_vertices[3] = cv::Point2f(width * 0.1f, height * 1.0f);

        dst_vertices[0] = cv::Point2f(0, 0);
        dst_vertices[1] = cv::Point2f(width, 0);
        dst_vertices[2] = cv::Point2f(width, height);
        dst_vertices[3] = cv::Point2f(0, height);

        // 버드아이뷰 변환
        cv::Mat perspective_matrix = cv::getPerspectiveTransform(src_vertices, dst_vertices);
        cv::Mat birds_eye_view;
        cv::warpPerspective(resized_frame, birds_eye_view, perspective_matrix, cv::Size(width, height));

        // 전처리 과정
        cv::Mat preprocessed;
        cv::GaussianBlur(birds_eye_view, preprocessed, cv::Size(5, 5), 0);

        // CLAHE 적용 (L*a*b* 색공간)
        cv::Mat lab;
        cv::cvtColor(preprocessed, lab, cv::COLOR_BGR2Lab);
        std::vector<cv::Mat> lab_channels;
        cv::split(lab, lab_channels);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(lab_channels[0], lab_channels[0]);

        cv::merge(lab_channels, lab);
        cv::cvtColor(lab, preprocessed, cv::COLOR_Lab2BGR);

        cv::Mat hsv;
        cv::cvtColor(preprocessed, hsv, cv::COLOR_BGR2HSV);

        cv::Mat yellow_mask_combined;

        cv::Mat yellow_mask_hsv;
        cv::Scalar lower_yellow_hsv(20, 50, 100);
        cv::Scalar upper_yellow_hsv(35, 255, 255);
        cv::inRange(hsv, lower_yellow_hsv, upper_yellow_hsv, yellow_mask_hsv);

        cv::Mat yellow_mask_lab;
        cv::inRange(lab, cv::Scalar(150, 120, 140), cv::Scalar(250, 135, 190), yellow_mask_lab);

        cv::Mat yellow_mask_rgb;
        cv::inRange(preprocessed, cv::Scalar(150, 150, 0), cv::Scalar(255, 255, 130), yellow_mask_rgb);

        cv::bitwise_or(yellow_mask_hsv, yellow_mask_lab, yellow_mask_combined);
        cv::bitwise_or(yellow_mask_combined, yellow_mask_rgb, yellow_mask_combined);

        // 모폴로지 연산
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        cv::morphologyEx(yellow_mask_combined, yellow_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(yellow_mask_combined, yellow_mask_combined, cv::MORPH_CLOSE, kernel_large);

        // 개선된 흰색 검출
        cv::Mat white_mask_combined;

        // 1. HSV 기반 흰색 검출
        cv::Mat white_mask_hsv;
        cv::Scalar lower_white_hsv(0, 20, 220);
        cv::Scalar upper_white_hsv(180, 25, 255);
        cv::inRange(hsv, lower_white_hsv, upper_white_hsv, white_mask_hsv);

        // 2. Lab 기반 흰색 검출
        cv::Mat white_mask_lab;
        std::vector<cv::Mat> lab_channels_white;
        cv::split(lab, lab_channels_white);
        cv::threshold(lab_channels_white[0], white_mask_lab, 200, 255, cv::THRESH_BINARY);

        // 3. RGB 기반 흰색 검출
        cv::Mat white_mask_rgb;
        cv::inRange(preprocessed, cv::Scalar(240, 240, 240), cv::Scalar(255, 255, 255), white_mask_rgb);

        // 노란색 마스크 제외
        cv::Mat yellow_mask_dilated;
        cv::dilate(yellow_mask_combined, yellow_mask_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        cv::bitwise_or(white_mask_hsv, white_mask_lab, white_mask_combined);
        cv::bitwise_or(white_mask_combined, white_mask_rgb, white_mask_combined);
        cv::bitwise_and(white_mask_combined, ~yellow_mask_dilated, white_mask_combined);

        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_CLOSE, kernel_large);
        cv::dilate(white_mask_combined, white_mask_combined, kernel, cv::Point(-1, -1), 2);

        // 선 검출 (컨투어 기반)
        std::vector<std::vector<cv::Point>> yellow_contours, white_contours;
        cv::findContours(yellow_mask_combined, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(white_mask_combined, white_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 컨투어 필터링 및 선 그리기
        cv::Mat line_display = preprocessed.clone();

        yellow_line_detected = false;
        white_line_detected = false;
        yellow_line_count = 0;
        white_line_count = 0;

        // 노란색 선 그리기
        for (const auto &contour : yellow_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 150.0)
            {
                yellow_line_detected = true;
                yellow_line_count++;
                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx, 10, true);

                cv::RotatedRect rot_rect = cv::minAreaRect(contour);
                cv::Point2f vertices[4];
                rot_rect.points(vertices);

                float max_length = 0;
                int max_idx = 0;
                for (int i = 0; i < 4; i++)
                {
                    float length = cv::norm(vertices[i] - vertices[(i + 1) % 4]);
                    if (length > max_length)
                    {
                        max_length = length;
                        max_idx = i;
                    }
                }
                cv::line(line_display, vertices[max_idx], vertices[(max_idx + 1) % 4],
                         cv::Scalar(0, 255, 255), 2);
            }
        }

        // 흰색 선 그리기
        for (const auto &contour : white_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 150.0)
            {
                white_line_detected = true;
                white_line_count++;

                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx, 10, true);

                // 긴 축을 찾기 위한 최소 영역 사각형
                cv::RotatedRect rot_rect = cv::minAreaRect(contour);
                cv::Point2f vertices[4];
                rot_rect.points(vertices);

                float max_length = 0;
                int max_idx = 0;
                for (int i = 0; i < 4; i++)
                {
                    float length = cv::norm(vertices[i] - vertices[(i + 1) % 4]);
                    if (length > max_length)
                    {
                        max_length = length;
                        max_idx = i;
                    }
                }
                cv::line(line_display, vertices[max_idx], vertices[(max_idx + 1) % 4],
                         cv::Scalar(255, 255, 255), 2);
            }
        }

        if (yellow_line_detected || white_line_detected)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Lines detected - Yellow: %d (count: %d), White: %d (count: %d)",
                        yellow_line_detected, yellow_line_count,
                        white_line_detected, white_line_count);
        }

        // ROI 영역 표시
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, src_vertices[i], src_vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }

        yellow_line_valid = isLineValid(yellow_detection_array, yellow_line_detected);
        white_line_valid = isLineValid(white_detection_array, white_line_detected);

        array_index = (array_index + 1) % ARRAY_SIZE;

        auto yellow_detected_msg = std_msgs::msg::Bool();
        auto white_detected_msg = std_msgs::msg::Bool();

        yellow_detected_msg.data = yellow_line_valid;
        white_detected_msg.data = white_line_valid;

        yellow_detected_pub_->publish(yellow_detected_msg);
        white_detected_pub_->publish(white_detected_msg);

        // Publish images
        sensor_msgs::msg::Image::SharedPtr original_msg =
            cv_bridge::CvImage(msg->header, "bgr8", resized_frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr yellow_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", yellow_mask_combined).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr white_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", white_mask_combined).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr line_msg =
            cv_bridge::CvImage(msg->header, "bgr8", line_display).toImageMsg();

        original_pub_->publish(*original_msg);
        yellow_mask_pub_->publish(*yellow_mask_msg);
        white_mask_pub_->publish(*white_mask_msg);
        line_pub_->publish(*line_msg);

        cv::imshow("Original Image", resized_frame);
        cv::imshow("Preprocessed", preprocessed);
        cv::imshow("Yellow Mask", yellow_mask_combined);
        cv::imshow("White Mask", white_mask_combined);
        cv::imshow("Detected Lines", line_display);
        cv::waitKey(1);
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
