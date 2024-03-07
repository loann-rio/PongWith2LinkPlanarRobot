#include "Simulation.h"

void Simulation::calibration()
{

}

int Simulation::predict(BallInfo ballInfo)
{
    double BallPos[2];
    double BallSpeed[2];
    BallPos[0] = ballInfo.BallXPosition;
    BallPos[1] = ballInfo.BallYPosition;
    BallSpeed[0] = ballInfo.BallXVelocity;
    BallSpeed[1] = ballInfo.BallYVelocity;
// POINT 1 AND 2 ARE UP RIGHT AND DOWN RIGHT
    std::array<float, 2> rightmost{};
    rightmost[0]=corners[1];
    rightmost[1]=corners[2];
    int remainingWidth = BallPos[0]+rightmost2.x;
    int multiplier = remainingWidth/BallSpeed[0];
    if (BallSpeed[0]<0){ 
        multiplier=-multiplier;
    }
    int predictedY=BallPos[1]+multiplier*BallSpeed[1];
    if (predictedY>Rightmost[0].y){
        predictedY= Rightmost[0].y;
    }
    else if (predictedY<Rightmost[1].y){
        predictedY =Rightmost[1].y;
    }
    return predictedY;
}

cv::Point Simulation::simulateStep(BallInfo info)
{
	return cv::Point();
}
	