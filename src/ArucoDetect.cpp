#include "../include/ArucoDetect.h"

using namespace std;

int ArucoDetect() {
    float markerLength = 0.1f;  // Example marker length, adjust as needed
    cv::Mat camMatrix = cv::Mat::eye(3, 3, CV_64F); // Dummy camera matrix (replace with actual)
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F); // Dummy distortion coefficients (replace with actual)
    bool estimatePose = true;
    bool showRejected = false;

    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);

    cv::VideoCapture inputVideo(2);
    if (!inputVideo.isOpened()) {
        cout << "Error: Couldn't open the video stream." << endl;
        return -1;
    }

    int waitTime = 10;
    double totalTime = 0;
    int totalIterations = 0;

    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

    while (inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)cv::getTickCount();

        vector<int> ids;
        vector<vector<cv::Point2f>> corners, rejected;

        detector.detectMarkers(image, corners, ids, rejected);

        size_t nMarkers = corners.size();
        vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if (estimatePose && !ids.empty()) {
            for (size_t i = 0; i < nMarkers; i++) {
                cv::solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }

        double currentTime = ((double)cv::getTickCount() - tick) / cv::getTickFrequency();
        totalTime += currentTime;
        totalIterations++;

        if (totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        image.copyTo(imageCopy);
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if (estimatePose) {
                for (unsigned int i = 0; i < ids.size(); i++) {
                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
                }
            }

            for (int id : ids) {
                cout << "Id = " << id << endl;
            }
        }

        if (showRejected && !rejected.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, rejected, cv::noArray(), cv::Scalar(100, 0, 255));
        }

        cv::imshow("out", imageCopy);
        cv::waitKey(waitTime);
    }

    cv::destroyAllWindows();
    return 0;
}

