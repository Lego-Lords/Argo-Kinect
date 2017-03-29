#include "stdafx.h"
#include "Kinect.h"

#include <wrl/client.h>
using namespace Microsoft::WRL;

int main(int argc, char* argv[])
{
	//icpCompare.comparePCD(); DITO PASUKAN NG PCD NA ICOCOMPARE

	try {
		Kinect kinect;
		kinect.run();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}

	cin.get();

	return 0;
}
