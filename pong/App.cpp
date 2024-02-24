#include "App.h"

void App::run()
{
	imageProcessor.initCam(1);

	while (true)
	{
		imageProcessor.updatePos();
		BallInfo ballInfo = imageProcessor.getBallInfo();

		std::cout << "position = " << ballInfo.BallXPosition << " " << ballInfo.BallYPosition << " velocity = " << ballInfo.BallXVelocity << " " << ballInfo.BallYVelocity << "\n";



		//show the frame in the created window
		cv::imshow(window_name, ballInfo.frame);

		//wait for for 10 ms until any key is pressed.  
		//If the 'Esc' key is pressed, break the while loop.
		if (cv::waitKey(10) == 27)
		{
			std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
			break;
		}
	}
}
