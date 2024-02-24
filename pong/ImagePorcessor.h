#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>


struct BallInfo {
	int BallXPosition;
	int BallYPosition;

	int BallXVelocity;
	int BallYVelocity;
};


class ImagePorcessor
{
public:
	void initCam(int camId);
	cv::Mat updatePos();

	BallInfo getBallInfo() const;

private:
	bool getFrame(cv::Mat& frame);
	cv::Point getPosBall(cv::Mat& frame);

	cv::VideoCapture cap;

	cv::Point posBall;
	cv::Point velocityBall;

	float WidthCam;
	float HeightCam;

	const int targetHeight = 30;
	const int targetWidth = 60;

	std::string window_name = "Camera Feed";

	std::chrono::time_point<std::chrono::system_clock> frameTime = std::chrono::system_clock::now();



};

