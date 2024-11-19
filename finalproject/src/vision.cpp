// vision.cpp
#include "finalproject/vision.hpp"

Vision::Vision(const std::string &node_name)
    : Node(node_name)
{
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/image_raw", 10,
        std::bind(&Vision::imageCallback, this, std::placeholders::_1));

    original_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/original", 10);
    yellow_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/yellow_mask", 10);
    white_mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/white_mask", 10);
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

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(lab_channels[0], lab_channels[0]);

        cv::merge(lab_channels, lab);
        cv::cvtColor(lab, preprocessed, cv::COLOR_Lab2BGR);

        // 노란색 검출 (HSV)
        cv::Mat hsv;
        cv::cvtColor(preprocessed, hsv, cv::COLOR_BGR2HSV);

        cv::Mat yellow_mask;
        cv::Scalar lower_yellow(20, 50, 100);
        cv::Scalar upper_yellow(35, 255, 255);
        cv::inRange(hsv, lower_yellow, upper_yellow, yellow_mask);

        // 개선된 흰색 검출
        cv::Mat white_mask_combined;

        // 1. HSV 기반 흰색 검출
        cv::Mat white_mask_hsv;
        cv::Scalar lower_white_hsv(0, 25, 220);
        cv::Scalar upper_white_hsv(180, 30, 255);
        cv::inRange(hsv, lower_white_hsv, upper_white_hsv, white_mask_hsv);

        // 2. Lab 기반 흰색 검출
        cv::Mat white_mask_lab;
        std::vector<cv::Mat> lab_channels_white;
        cv::split(lab, lab_channels_white);
        cv::threshold(lab_channels_white[0], white_mask_lab, 240, 255, cv::THRESH_BINARY);

        // 3. RGB 기반 흰색 검출
        cv::Mat white_mask_rgb;
        cv::inRange(preprocessed, cv::Scalar(245, 245, 245), cv::Scalar(255, 255, 255), white_mask_rgb);

        // 노란색 마스크 제외
        cv::Mat yellow_mask_dilated;
        cv::dilate(yellow_mask, yellow_mask_dilated, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

        cv::bitwise_or(white_mask_hsv, white_mask_lab, white_mask_combined);
        cv::bitwise_or(white_mask_combined, white_mask_rgb, white_mask_combined);
        cv::bitwise_and(white_mask_combined, ~yellow_mask_dilated, white_mask_combined);

        // 모폴로지 연산
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel_large);

        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(white_mask_combined, white_mask_combined, cv::MORPH_CLOSE, kernel_large);
        cv::dilate(white_mask_combined, white_mask_combined, kernel, cv::Point(-1, -1), 2);

        // 선 검출 (컨투어 기반)
        std::vector<std::vector<cv::Point>> yellow_contours, white_contours;
        cv::findContours(yellow_mask, yellow_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::findContours(white_mask_combined, white_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 컨투어 필터링 및 선 그리기
        cv::Mat line_display = preprocessed.clone();

        // 노란색 선 그리기
        for (const auto &contour : yellow_contours)
        {
            double area = cv::contourArea(contour);
            if (area > 150.0)
            {
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
            if (area > 100.0)
            {
                std::vector<cv::Point> approx;
                cv::approxPolyDP(contour, approx, 10, true);
ect = cv::minAreaRect(contour);
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

        // ROI 영역 표시
        for (int i = 0; i < 4; i++)
        {
            cv::line(resized_frame, src_vertices[i], src_vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2);
        }

        // Publish images
        sensor_msgs::msg::Image::SharedPtr original_msg =
            cv_bridge::CvImage(msg->header, "bgr8", resized_frame).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr yellow_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", yellow_mask).toImageMsg();
        sensor_msgs::msg::Image::SharedPtr white_mask_msg =
            cv_bridge::CvImage(msg->header, "mono8", white_mask_combined).toImageMsg();

        original_pub_->publish(*original_msg);
        yellow_mask_pub_->publish(*yellow_mask_msg);
        white_mask_pub_->publish(*white_mask_msg);

        cv::imshow("Original Image", resized_frame);
        cv::imshow("Preprocessed", preprocessed);
        cv::imshow("Yellow Mask", yellow_mask);
        cv::imshow("White Mask", white_mask_combined);
        cv::imshow("Detected Lines", line_display);
        cv::waitKey(1);

    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}
