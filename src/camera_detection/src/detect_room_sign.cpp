#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/opencv.hpp>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>

using namespace cv;
using namespace std;

#define SHOW_DEBUG_GUI 0 // Set to 1 to enable local OpenCV window display on Pi desktop

class RoomDetectorNode : public rclcpp::Node {
public:
    RoomDetectorNode() : Node("room_detector_node"), frameCount_(0), textHoldCounter_(0) {
        // Declare and retrieve parameters
        this->declare_parameter<int>("camera_id", 0);
        this->declare_parameter<int>("frame_width", 640);
        this->declare_parameter<int>("frame_height", 360);
        this->declare_parameter<string>("topic_name", "detected_room");

        int cameraId = this->get_parameter("camera_id").as_int();
        int width = this->get_parameter("frame_width").as_int();
        int height = this->get_parameter("frame_height").as_int();
        string topicName = this->get_parameter("topic_name").as_string();

        // ROS 2 Publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>(topicName, 10);

        // Initialize Tesseract C++ API
        tess_api_ = std::make_unique<tesseract::TessBaseAPI>();
        if (tess_api_->Init(nullptr, "eng", tesseract::OEM_LSTM_ONLY)) {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize Tesseract C++ API!");
            return;
        }

        // Configure PSM 7 (Single Line) and character whitelist (0-9, B)
        tess_api_->SetPageSegMode(tesseract::PSM_SINGLE_LINE);
        tess_api_->SetVariable("tessedit_char_whitelist", "0123456789B");

        // Initialize USB Camera using V4L2 backend
        camera_.open(cameraId, CAP_V4L2);
        if (!camera_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera on /dev/video%d", cameraId);
            return;
        }

        camera_.set(CAP_PROP_FRAME_WIDTH, width);
        camera_.set(CAP_PROP_FRAME_HEIGHT, height);

        frameBounds_ = Rect(0, 0, width, height);

        RCLCPP_INFO(this->get_logger(), "Room Detector Node started (Tesseract C++ API mode). Publishing to topic '%s'...", topicName.c_str());

        // Timer loop running at ~30 FPS (33ms interval)
        timer_ = this->create_wall_timer(
            chrono::milliseconds(33),
            std::bind(&RoomDetectorNode::processFrameCallback, this)
        );
    }

    ~RoomDetectorNode() {
        if (tess_api_) {
            tess_api_->End();
        }
        if (camera_.isOpened()) {
            camera_.release();
        }
        destroyAllWindows();
    }

private:
    // Detects text angle and rotates image back to horizontal
    Mat deskewCrop(const Mat& input) {
        if (input.empty() || input.rows < 10 || input.cols < 10) return input;

        Mat gray;
        if (input.channels() == 3) {
            cvtColor(input, gray, COLOR_BGR2GRAY);
        } else {
            gray = input.clone();
        }

        Mat bin;
        threshold(gray, bin, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

        vector<Point> points;
        findNonZero(bin, points);
        if (points.size() < 15) return input;

        RotatedRect box = minAreaRect(points);
        double angle = box.angle;

        if (angle < -45.0) {
            angle += 90.0;
        } else if (angle > 45.0) {
            angle -= 90.0;
        }

        if (abs(angle) < 1.5 || abs(angle) > 35.0) {
            return input;
        }

        Point2f center = box.center;
        Mat rotMat = getRotationMatrix2D(center, angle, 1.0);
        Mat rotated;
        warpAffine(input, rotated, rotMat, input.size(), INTER_CUBIC, BORDER_CONSTANT, Scalar(255, 255, 255));

        return rotated;
    }

    // Preprocesses ROI crop into clean 8-bit Grayscale for Tesseract
    Mat prepareGrayscaleForTesseract(const Mat& input) {
        if (input.empty() || input.rows < 10 || input.cols < 10) {
            return Mat();
        }

        Mat deskewed = deskewCrop(input);

        Mat gray;
        if (deskewed.channels() == 3) {
            cvtColor(deskewed, gray, COLOR_BGR2GRAY);
        } else {
            gray = deskewed.clone();
        }

        Mat scaled;
        double targetHeight = 90.0;
        double scale = targetHeight / static_cast<double>(gray.rows);
        
        int newW = max(16, cvRound(gray.cols * scale));
        int newH = max(16, cvRound(gray.rows * scale));
        resize(gray, scaled, Size(newW, newH), 0, 0, scale > 1.0 ? INTER_CUBIC : INTER_AREA);

        GaussianBlur(scaled, scaled, Size(3, 3), 0);

        Mat bin;
        threshold(scaled, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);

        if (countNonZero(bin) < (bin.rows * bin.cols / 2)) {
            bitwise_not(bin, bin);
        }

        Mat padded;
        copyMakeBorder(bin, padded, 20, 20, 20, 20, BORDER_CONSTANT, Scalar(255));

        return padded;
    }

    // Direct in-memory OCR execution via Tesseract C++ API
    string runTesseractAPI(const Mat& roiCrop) {
        if (roiCrop.empty() || roiCrop.cols < 12 || roiCrop.rows < 12) return "";

        Mat processed = prepareGrayscaleForTesseract(roiCrop);
        if (processed.empty()) return "";

        // Pass OpenCV matrix buffer directly to Tesseract API (Zero disk write)
        tess_api_->SetImage(
            processed.data, 
            processed.cols, 
            processed.rows, 
            processed.channels(), 
            static_cast<int>(processed.step)
        );

        char* outText = tess_api_->GetUTF8Text();
        if (!outText) return "";

        string recognizedText(outText);
        delete[] outText; // Free memory allocated by Tesseract
        tess_api_->Clear(); // Reset state for next iteration

        // Trim leading and trailing whitespace
        size_t start = recognizedText.find_first_not_of(" \n\r\t");
        size_t end = recognizedText.find_last_not_of(" \n\r\t");
        if (start == string::npos) return "";
        recognizedText = recognizedText.substr(start, (end - start + 1));

        if (recognizedText.length() < 2) {
            return "";
        }

        return recognizedText;
    }

    // Main vision processing loop called by ROS 2 timer
    void processFrameCallback() {
        Mat frame;
        camera_ >> frame;
        if (frame.empty()) return;

        if (frame.cols != 640 || frame.rows != 360) {
            resize(frame, frame, Size(640, 360));
        }

        frameCount_++;
        Mat gray, hsv, maroonMask;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // 1. Denoise frame
        Mat blurredFrame;
        GaussianBlur(frame, blurredFrame, Size(3, 3), 0);
        cvtColor(blurredFrame, hsv, COLOR_BGR2HSV);

        // 2. High-Precision Dual-Range Maroon HSV Mask
        Mat mask1, mask2;
        inRange(hsv, Scalar(150, 30, 25), Scalar(179, 255, 230), mask1);
        inRange(hsv, Scalar(0, 30, 25), Scalar(15, 255, 230), mask2);
        bitwise_or(mask1, mask2, maroonMask);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(maroonMask, maroonMask, MORPH_CLOSE, kernel);

        // 3. Candidate Contour Filtering (Width > Height & Rectangularity)
        vector<vector<Point>> contours;
        findContours(maroonMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<Rect> rawBoxes;
        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area < 30) continue;

            Rect box = boundingRect(contour);
            if (box.width <= box.height) continue;

            double rectArea = static_cast<double>(box.width * box.height);
            double solidity = area / rectArea;
            if (solidity < 0.50) continue;

            int pad = 5;
            int x = max(0, box.x - pad);
            int y = max(0, box.y - pad);
            int w = min(frame.cols - x, box.width + 2 * pad);
            int h = min(frame.rows - y, box.height + 2 * pad);

            if (w > h && w >= 20 && h >= 12) {
                rawBoxes.push_back(Rect(x, y, w, h));
            }
        }

        // Merge overlapping candidate regions
        vector<Rect> mergedBoxes;
        for (const auto& b : rawBoxes) {
            bool merged = false;
            for (auto& m : mergedBoxes) {
                Rect expandedM(max(0, m.x - 10), max(0, m.y - 10), m.width + 20, m.height + 20);
                if ((b & expandedM).area() > 0) {
                    m |= b;
                    merged = true;
                    break;
                }
            }
            if (!merged) {
                mergedBoxes.push_back(b);
            }
        }

        // 4. Discard Blue Rectangles that do not contain white pixels
        vector<Rect> whiteFilteredBoxes;
        for (const auto& box : mergedBoxes) {
            Rect safeBox = box & frameBounds_;
            if (safeBox.width < 12 || safeBox.height < 12) continue;

            Mat grayRoi = gray(safeBox);
            Mat brightMask;
            threshold(grayRoi, brightMask, 160, 255, THRESH_BINARY);

            if (countNonZero(brightMask) >= 15) {
                whiteFilteredBoxes.push_back(box);
            }
        }
        mergedBoxes = whiteFilteredBoxes;

        sort(mergedBoxes.begin(), mergedBoxes.end(), [](const Rect& a, const Rect& b) {
            return a.area() > b.area();
        });
        if (mergedBoxes.size() > 3) mergedBoxes.resize(3);

        // 5. Generate Cyan ROIs (Strictly Left 60% of Width)
        vector<Rect> focusedCyanBoxes;
        for (const Rect& fullRoi : mergedBoxes) {
            int focusedX = fullRoi.x;
            int focusedWidth = cvRound(fullRoi.width * 0.60);

            int focusedY = fullRoi.y + cvRound(fullRoi.height * 0.15);
            int focusedHeight = cvRound(fullRoi.height * 0.70);

            Rect candidateCyan(focusedX, focusedY, focusedWidth, focusedHeight);
            Rect safeCyan = candidateCyan & frameBounds_;

            if (safeCyan.width >= 12 && safeCyan.height >= 12) {
                focusedCyanBoxes.push_back(safeCyan);
            }
        }

        // 6. Run Tesseract C++ API on Cyan ROIs every 10 frames
        if (frameCount_ % 10 == 0 && !focusedCyanBoxes.empty()) {
            for (const Rect& cyanRoi : focusedCyanBoxes) {
                if ((cyanRoi & frameBounds_) == cyanRoi && cyanRoi.width >= 12 && cyanRoi.height >= 12) {
                    Mat roiCrop = frame(cyanRoi).clone();
                    string text = runTesseractAPI(roiCrop);

                    if (!text.empty()) {
                        lastRecognizedText_ = text;
                        textHoldCounter_ = 15;

                        // Publish ROS 2 String Message
                        auto msg = std_msgs::msg::String();
                        msg.data = lastRecognizedText_;
                        publisher_->publish(msg);

                        RCLCPP_INFO(this->get_logger(), "[ROOM DETECTED] Published: '%s'", msg.data.c_str());
                        break;
                    }
                }
            }
        }

#if SHOW_DEBUG_GUI
        Mat displayFrame = frame.clone();
        for (const Rect& box : mergedBoxes) rectangle(displayFrame, box, Scalar(255, 0, 0), 1);
        for (const Rect& cyanBox : focusedCyanBoxes) rectangle(displayFrame, cyanBox, Scalar(255, 255, 0), 2);
        imshow("ROS2 Camera Feed", displayFrame);
        waitKey(1);
#endif
    }

    // Node state members
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VideoCapture camera_;
    std::unique_ptr<tesseract::TessBaseAPI> tess_api_;

    int frameCount_;
    int textHoldCounter_;
    string lastRecognizedText_;
    Rect frameBounds_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoomDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}