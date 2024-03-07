#pragma once

#include <opencv2/opencv.hpp>
#include <array>

struct BallInfo {
	int BallXPosition;
	int BallYPosition;

	int BallXVelocity;
	int BallYVelocity;
};


class Simulation
{
public:
	Simulation() {};

	void calibration();
	cv::Point simulateStep(BallInfo info);

private:
	std::array<cv::Point, 4> corners{};

};

