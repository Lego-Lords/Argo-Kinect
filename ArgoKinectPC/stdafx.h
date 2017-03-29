// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>



// TODO: reference additional headers your program requires here
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#undef NOMINMAX
#include <Kinect.h>
#include <pcl\visualization\cloud_viewer.h>
#include <pcl\common\projection_matrix.h>
#include <pcl\point_cloud.h>
#include <iostream>
#include <winsock2.h>