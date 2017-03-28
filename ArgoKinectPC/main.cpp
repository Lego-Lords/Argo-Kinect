// PLCTry.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "Kinect.h"
int main(int argc, char* argv[])
{
	try {
		Kinect kinect;
		kinect.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	return 0;
}