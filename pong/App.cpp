#include "App.h"

void App::run()
{
	imageProcessor.initCam(2);

	while (true)
	{
		imageProcessor.updatePos();
		BallInfo ballInfo = imageProcessor.getBallInfo();

		std::cout << "position = " << ballInfo.BallXPosition << " " << ballInfo.BallYPosition << " velocity = " << ballInfo.BallXVelocity << " " << ballInfo.BallYVelocity << "\n";
	}
}
