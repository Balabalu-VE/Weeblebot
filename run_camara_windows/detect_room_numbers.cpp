#include <opencv2/opencv.hpp>
#include <opencv2/geometry.hpp>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <chrono>
#include <thread>
#include <cmath>

using namespace cv;
using namespace std;

#define SHOW_DEBUG_GUI 1 // Set to 0 for headless deployment on Pi

// Automatically detects text angle and rotates image back to horizontal
Mat deskewCrop(const Mat& input) {
    if (input.empty() || input.rows < 10 || input.cols < 10) return input;

    Mat gray;
    if (input.channels() == 3) {
        cvtColor(input, gray, COLOR_BGR2GRAY);
    }
    else {
        gray = input.clone();
    }

    // Threshold to isolate foreground text pixels
    Mat bin;
    threshold(gray, bin, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

    vector<Point> points;
    findNonZero(bin, points);
    if (points.size() < 15) return input; // Not enough pixels to determine tilt

    // Calculate minimum area oriented bounding box around text
    RotatedRect box = minAreaRect(points);
    double angle = box.angle;

    // Adjust OpenCV angle convention (-90 to 0 degrees)
    if (angle < -45.0) {
        angle += 90.0;
    }
    else if (angle > 45.0) {
        angle -= 90.0;
    }

    // Ignore tiny tilts (less than 1.5 deg) or extreme errors (> 35 deg)
    if (abs(angle) < 1.5 || abs(angle) > 35.0) {
        return input;
    }

    // Perform rotation around center point
    Point2f center = box.center;
    Mat rotMat = getRotationMatrix2D(center, angle, 1.0);
    Mat rotated;
    warpAffine(input, rotated, rotMat, input.size(), INTER_CUBIC, BORDER_CONSTANT, Scalar(255, 255, 255));

    return rotated;
}

// Preprocesses raw candidate ROI crop into clean 8-bit Grayscale for Tesseract
Mat prepareGrayscaleForTesseract(const Mat& input) {
    if (input.empty() || input.rows < 10 || input.cols < 10) {
        return Mat();
    }

    // 1. Deskew tilted text before resizing and binarizing
    Mat deskewed = deskewCrop(input);

    Mat gray;
    if (deskewed.channels() == 3) {
        cvtColor(deskewed, gray, COLOR_BGR2GRAY);
    }
    else {
        gray = deskewed.clone();
    }

    // 2. Dynamic Scaling: Target ~90px ROI height (~40-50px character height optimal for Tesseract)
    Mat scaled;
    double targetHeight = 90.0;
    double scale = targetHeight / static_cast<double>(gray.rows);

    int newW = max(16, cvRound(gray.cols * scale));
    int newH = max(16, cvRound(gray.rows * scale));
    resize(gray, scaled, Size(newW, newH), 0, 0, scale > 1.0 ? INTER_CUBIC : INTER_AREA);

    // 3. Smooth sensor noise/pixelation
    GaussianBlur(scaled, scaled, Size(3, 3), 0);

    // 4. Otsu Global Binarization
    Mat bin;
    threshold(scaled, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);

    // 5. Polarity Check: Guarantee Black Text on White Background
    if (countNonZero(bin) < (bin.rows * bin.cols / 2)) {
        bitwise_not(bin, bin);
    }

    // 6. Add clean white margin border for Tesseract layout detection
    Mat padded;
    copyMakeBorder(bin, padded, 20, 20, 20, 20, BORDER_CONSTANT, Scalar(255));

    return padded;
}

// Executes Tesseract CLI with disk I/O race condition guard
string runTesseractCLI(const Mat& roiCrop, Mat& outDebugImg) {
    if (roiCrop.empty() || roiCrop.cols < 12 || roiCrop.rows < 12) return "";

    outDebugImg = prepareGrayscaleForTesseract(roiCrop);
    if (outDebugImg.empty()) return "";

    string tempImg = "temp_roi.png";
    string tempOut = "temp_out"; // Tesseract appends .txt automatically
    string txtFilePath = tempOut + ".txt";

    // Clear previous temporary outputs
    remove(txtFilePath.c_str());

    if (!imwrite(tempImg, outDebugImg)) return "";

    // --psm 7: Single line of text with updated character whitelist (0-9, B)
    string cmd = "C:\\\"Program Files\"\\Tesseract-OCR\\tesseract.exe " + tempImg + " " + tempOut + " --psm 7 -c tessedit_char_whitelist=0123456789B > nul 2>&1";
    system(cmd.c_str());

    // Disk I/O Race Guard: Wait for Windows file handle flush
    string recognizedText = "";
    for (int attempts = 0; attempts < 5; ++attempts) {
        ifstream outFile(txtFilePath, ios::in);
        if (outFile.is_open()) {
            string line;
            while (getline(outFile, line)) {
                if (!line.empty()) {
                    recognizedText += line;
                }
            }
            outFile.close();

            if (!recognizedText.empty()) {
                break;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    remove(tempImg.c_str());
    remove(txtFilePath.c_str());

    // Clean leading/trailing whitespace
    size_t start = recognizedText.find_first_not_of(" \n\r\t");
    size_t end = recognizedText.find_last_not_of(" \n\r\t");
    if (start == string::npos) return "";
    recognizedText = recognizedText.substr(start, (end - start + 1));

    // Filter out single-character noise specks
    if (recognizedText.length() < 2) {
        return "";
    }

    return recognizedText;
}

int main() {
    // -----------------------------------------------
    // 1. Initialize Camera Stream
    // -----------------------------------------------
    VideoCapture camera(0, CAP_DSHOW);
    if (!camera.isOpened()) {
        camera.open(0);
        if (!camera.isOpened()) {
            cerr << "[ERROR] Could not open camera." << endl;
            return 1;
        }
    }

    camera.set(CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CAP_PROP_FRAME_HEIGHT, 360);

    cout << "Camera stream active (640x360). Auto-Deskewing + White-Pixel Filtered Maroon Boxes..." << endl;
    cout << "Debug window will refresh the image passed to Tesseract every 1.0s." << endl;
    cout << "Press 'Q' or ESC to exit." << endl;

    Mat frame, displayFrame, gray, hsv, maroonMask;
    int frameCount = 0;

    string lastRecognizedText = "";
    Rect activeCyanBox;
    int textHoldCounter = 0;

    Rect frameBounds(0, 0, 640, 360);

    // Timer & Storage for 1-Second Debug Image Window
    auto lastDebugTime = chrono::steady_clock::now();
    Mat lastDebugImg;

    // -----------------------------------------------
    // 2. Real-time Video Processing Loop
    // -----------------------------------------------
    while (true) {
        camera >> frame;
        if (frame.empty()) break;

        if (frame.cols != 640 || frame.rows != 360) {
            resize(frame, frame, Size(640, 360));
        }

        frameCount++;
        displayFrame = frame.clone();
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

            if (box.width <= box.height) continue; // Width > Height check

            double rectArea = static_cast<double>(box.width * box.height);
            double solidity = area / rectArea;
            if (solidity < 0.50) continue; // Solidity check

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
            Rect safeBox = box & frameBounds;
            if (safeBox.width < 12 || safeBox.height < 12) continue;

            Mat grayRoi = gray(safeBox);
            Mat brightMask;

            // Threshold for white text/characters (intensity > 160)
            threshold(grayRoi, brightMask, 160, 255, THRESH_BINARY);

            // Keep box only if it contains at least 15 white text pixels
            if (countNonZero(brightMask) >= 15) {
                whiteFilteredBoxes.push_back(box);
            }
        }
        mergedBoxes = whiteFilteredBoxes;

        // Sort by area and keep top candidate boxes
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
            Rect safeCyan = candidateCyan & frameBounds;

            if (safeCyan.width >= 12 && safeCyan.height >= 12) {
                focusedCyanBoxes.push_back(safeCyan);
            }
        }

        // 6. Run Tesseract CLI on Cyan ROIs every 10 frames
        if (frameCount % 10 == 0 && !focusedCyanBoxes.empty()) {
            bool matchedInFrame = false;

            for (const Rect& cyanRoi : focusedCyanBoxes) {
                if ((cyanRoi & frameBounds) == cyanRoi && cyanRoi.width >= 12 && cyanRoi.height >= 12) {
                    Mat roiCrop = frame(cyanRoi).clone();
                    Mat currentDebug;

                    string text = runTesseractCLI(roiCrop, currentDebug);

                    if (!currentDebug.empty()) {
                        lastDebugImg = currentDebug.clone();
                    }

                    if (!text.empty()) {
                        lastRecognizedText = text;
                        activeCyanBox = cyanRoi;
                        textHoldCounter = 15; // Hold text overlay for 15 frames (~0.5s)
                        matchedInFrame = true;

                        cout << "----------------------------------------" << endl;
                        cout << "[OCR SUCCESS] Recognized Text: \"" << lastRecognizedText << "\"" << endl;
                        cout << "Position: (X: " << cyanRoi.x << ", Y: " << cyanRoi.y
                            << ", W: " << cyanRoi.width << ", H: " << cyanRoi.height << ")" << endl;
                        break;
                    }
                }
            }

            if (!matchedInFrame && textHoldCounter > 0) {
                textHoldCounter--;
            }
        }
        else if (textHoldCounter > 0) {
            textHoldCounter--;
        }

        // 7. Check 1-Second Timer to Update Debug Window
        auto now = chrono::steady_clock::now();
        double elapsedSeconds = chrono::duration_cast<chrono::duration<double>>(now - lastDebugTime).count();

        if (elapsedSeconds >= 1.0) {
            lastDebugTime = now;
            if (!lastDebugImg.empty()) {
                imshow("DEBUG: Actual Image Passed to Tesseract (1s Sample)", lastDebugImg);
            }
        }

        // 8. Visual Overlays
        // Full Rectangular Maroon Candidate Boxes in Blue (BGR: 255, 0, 0)
        for (const Rect& box : mergedBoxes) {
            rectangle(displayFrame, box, Scalar(255, 0, 0), 1);
        }

        // Spatial Focus Left 60% ROIs in Cyan (BGR: 255, 255, 0)
        for (const Rect& cyanBox : focusedCyanBoxes) {
            rectangle(displayFrame, cyanBox, Scalar(255, 255, 0), 2);
        }

        // Highlight Winning Cyan Box in Green and Overlay Text
        if (textHoldCounter > 0 && activeCyanBox.width > 0 && activeCyanBox.height > 0) {
            rectangle(displayFrame, activeCyanBox, Scalar(0, 255, 0), 3);

            string label = "ROOM: " + lastRecognizedText;
            Point textPos(max(15, activeCyanBox.x), max(25, activeCyanBox.y - 10));

            putText(displayFrame, label, textPos + Point(1, 1), FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 0, 0), 3);
            putText(displayFrame, label, textPos, FONT_HERSHEY_SIMPLEX, 0.65, Scalar(0, 255, 0), 2);
        }
        else {
            putText(displayFrame, "Scanning Cyan (Left 60%) ROIs...", Point(15, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
        }

        // 9. GUI Display
#if SHOW_DEBUG_GUI
        imshow("Camera Feed - Tesseract Room Detector", displayFrame);
        imshow("DEBUG: Maroon Mask", maroonMask);
        int key = waitKey(1);
#else
        int key = cv::pollKey();
#endif

        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }
    }

    camera.release();
    destroyAllWindows();
    return 0;
}
