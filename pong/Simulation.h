#pragma once

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

	void simulateStep(BallInfo info);
};

