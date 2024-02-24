#include "ImagePorcessor.h"

BallInfo ImagePorcessor::getBallInfo() const
{
	return { posBall.x, posBall.y, velocityBall.x, velocityBall.y, frame };
}

void ImagePorcessor::initCam(int camId)
{
	cap = cv::VideoCapture{ camId };

	if (cap.isOpened() == false)
	{
		std::cout << "Cannot open the video camera\n";
		std::cin.get(); //wait for any key press
		return;
	}

	WidthCam = cap.get(cv::CAP_PROP_FRAME_WIDTH); 
	HeightCam = cap.get(cv::CAP_PROP_FRAME_HEIGHT);

	cv::namedWindow(window_name); //create a window called "My Camera Feed"

	std::cout << "camOpen with Height = " << HeightCam << "and width = " << WidthCam << "\n";
}



void ImagePorcessor::updatePos()
{
	getFrame(frame);

	cv::Point point = getPosBall(frame);

	std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - frameTime;
	frameTime = std::chrono::system_clock::now();

	velocityBall = (posBall - point) / (elapsed_seconds.count()); // in pixel per sec
	posBall = point;

	// Draw a red circle at the position of the most orange pixel
	cv::circle(frame, cv::Point{ (point.x * (int)WidthCam / targetWidth), (point.y * (int)HeightCam / targetHeight) }, 10, cv::Scalar(0, 0, 255), -1);

	//show the frame in the created window
	cv::imshow(window_name, frame);

}

bool ImagePorcessor::getFrame(cv::Mat& frame)
{
	
	bool bSuccess = cap.read(frame); // read a new frame from video 

	//Breaking the while loop if the frames cannot be captured
	if (bSuccess == false)
	{
		std::cout << "Video camera is disconnected" << std::endl;
		std::cin.get(); //Wait for any key press
	}

	//frame = cv::imread("C:\\Users\\loann\\Desktop\\pingPongBall.jpg");
	return bSuccess;
}

cv::Point ImagePorcessor::getPosBall(cv::Mat& frame)
{
	// resizee window
	cv::Mat resizedFrame;

	cv::resize(frame, resizedFrame, cv::Size(targetWidth, targetHeight), cv::INTER_LINEAR);

	cvtColor(resizedFrame, resizedFrame, cv::COLOR_BGR2HSV);

	// Define the range of orange color in HSV
	cv::Scalar lowerOrange = cv::Scalar(0, 100, 100);
	cv::Scalar upperOrange = cv::Scalar(30, 255, 255);

	// Threshold the image to get a binary mask
	cv::Mat mask;
	cv::inRange(resizedFrame, lowerOrange, upperOrange, mask);

	// Find contours in the binary mask
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// Find the contour with the largest area (most orange)
	double maxArea = 0;
	int maxAreaIndex = -1;
	for (size_t i = 0; i < contours.size(); i++) {
		double area = contourArea(contours[i]);
		if (area > maxArea) {
			maxArea = area;
			maxAreaIndex = i;
		}
	}

	// Get the centroid of the largest contour
	if (maxAreaIndex != -1) {
		cv::Moments mu = moments(contours[maxAreaIndex]);
		cv::Point centroid(mu.m10 / mu.m00, mu.m01 / mu.m00);

		//std::cout << "Position of the most orange pixel: " << centroid << std::endl;

		return centroid;
	}
	else {
		std::cout << "No orange pixels found in the image." << std::endl;

		return cv::Point{ -1 };
	}

	
}
