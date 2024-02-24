#pragma once

#include "ImagePorcessor.h"

#include <iostream>
#include <string>



class App
{
public:
	void run();
private:
	ImagePorcessor imageProcessor;
	std::string window_name = "Camera Feed";
};

